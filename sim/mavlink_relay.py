#!/usr/bin/env python3
"""Bidirectional UDP relay for MAVLink.

Forwards MAVLink traffic between PX4 SITL (localhost:14540) and a remote MAVSDK
instance so you can run the simulation on one machine and drone-follow on
another.

The relay binds to an ephemeral local port and registers with PX4 by sending
periodic heartbeats.  PX4 starts replying to the relay, which forwards all
traffic to the remote machine on port 14540.  Return traffic from the remote
MAVSDK is forwarded back to PX4.

Usage:
    sim/mavlink_relay.py 192.168.1.50
    sim/mavlink_relay.py 192.168.1.50 --px4-port 14540 --remote-port 14540
"""

import argparse
import select
import signal
import socket
import struct
import sys
import threading
import time


def _mavlink_crc(data: bytes) -> int:
    """CRC-16/MCRF4XX used by MAVLink."""
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF
    return crc


def _make_heartbeat(seq: int = 0) -> bytes:
    """Build a minimal MAVLink v2 GCS heartbeat packet."""
    header = struct.pack(
        "<BBBBBBB",
        0xFD,          # magic (MAVLink v2)
        9,             # payload length
        0,             # incompat_flags
        0,             # compat_flags
        seq & 0xFF,    # sequence
        255,           # system ID (GCS)
        190,           # component ID (MAV_COMP_ID_MISSIONPLANNER)
    )
    msgid = struct.pack("<I", 0)[:3]  # HEARTBEAT = 0, 3-byte LE
    # Payload: custom_mode(u32) type autopilot base_mode system_status mavlink_version
    payload = struct.pack("<IBBBBB", 0, 6, 8, 0, 0, 3)
    # CRC covers header[1:] + msgid + payload + CRC_EXTRA (50 for HEARTBEAT)
    crc = _mavlink_crc(header[1:] + msgid + payload + bytes([50]))
    return header + msgid + payload + struct.pack("<H", crc)


def main():
    p = argparse.ArgumentParser(description="MAVLink bidirectional UDP relay")
    p.add_argument("remote_host",
                   help="IP address of the remote machine running drone-follow")
    p.add_argument("--px4-port", type=int, default=14540,
                   help="PX4 SITL MAVLink port (default: 14540)")
    p.add_argument("--remote-port", type=int, default=14540,
                   help="Port on the remote machine (default: 14540)")
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 0))  # ephemeral port — no conflict with PX4
    local_port = sock.getsockname()[1]

    px4_addr = ("127.0.0.1", args.px4_port)
    remote_addr = (args.remote_host, args.remote_port)

    print(f"[relay] Bound to 0.0.0.0:{local_port}")
    print(f"[relay] PX4 SITL:   127.0.0.1:{args.px4_port}")
    print(f"[relay] Remote:     {args.remote_host}:{args.remote_port}")

    running = True

    def shutdown(*_):
        nonlocal running
        running = False
        print("\n[relay] Stopped.")
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Send heartbeats to PX4 so it registers us as a client and starts
    # streaming MAVLink back.  Keeps running so PX4 doesn't drop us.
    def heartbeat_loop():
        seq = 0
        while running:
            try:
                sock.sendto(_make_heartbeat(seq), px4_addr)
                seq = (seq + 1) & 0xFF
            except OSError:
                pass
            time.sleep(1)

    threading.Thread(target=heartbeat_loop, daemon=True).start()

    fwd_count = 0
    while running:
        readable, _, _ = select.select([sock], [], [], 1.0)
        for _ in readable:
            data, addr = sock.recvfrom(65535)
            if addr[0] == "127.0.0.1":
                # From PX4 → forward to remote MAVSDK
                sock.sendto(data, remote_addr)
            else:
                # From remote MAVSDK → forward to PX4
                sock.sendto(data, px4_addr)
            fwd_count += 1
            if fwd_count % 1000 == 0:
                print(f"[relay] Forwarded {fwd_count} packets")


if __name__ == "__main__":
    main()
