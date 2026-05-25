#!/usr/bin/env python3
"""Loopback bridge: drone-follow JSON  →  QOpenHD v4 binary detection payload.

Lets you verify the QOpenHD-side cross / mode-badge / TARGET-CHANGED toast
rendering on a single dev box, without a wifibroadcast radio link.

Architecture:

  drone-follow openhd_bridge   ──UDP JSON──▶   THIS script   ──UDP v4──▶   QOpenHD
       (sender, port 5511)                    (listener 5511 /                (listener 5520)
                                               sender 5520)

Normally the OpenHD daemon sits in this script's slot, receiving the JSON
over the localhost loopback, encoding it as the v4 binary payload, and
emitting it to a wfb radio tx. The wfb rx on the other end forwards the
decoded payload to localhost:5520 where QOpenHD picks it up. This script
shortcuts the radio path entirely.

Usage:

  # Terminal 1 — drone-follow (the OpenHD bridge always starts; no --openhd
  # needed, we just want the JSON report port to come up):
  robot-follow --input usb --webui

  # Terminal 2 — this script:
  python3 scripts/openhd_loopback.py

  # Terminal 3 — QOpenHD on the same box:
  ~/Desktop/hailo-pai/qopenHD/build/release/release/QOpenHD

QOpenHD will render the detection overlay on top of whatever video it
finds (or a blank background if no air video stream is configured —
the cross / badge / toast still draw, you just see them on black).

Wire format mirrors OpenHD/ohd_video/src/hailo_follow_bridge.cpp v4:

  Byte 0:      version = 4
  Byte 1-2:    active_id (uint16 LE, 0 = none)
  Byte 3-4:    follow_id (int16 LE, -1=idle, 0=auto, N=locked)
  Byte 5:      mode  (0=AUTO, 1=LOCKED, 2=SEARCH, 3=IDLE)
  Byte 6:      count (uint8)
  Per bbox (11 bytes): id(2) cx(2) cy(2) w(2) h(2) flags(1)
"""

from __future__ import annotations

import json
import socket
import struct
import sys

LISTEN_HOST = "127.0.0.1"
LISTEN_PORT = 5511          # drone-follow openhd_bridge sends JSON here
QOPENHD_HOST = "127.0.0.1"
QOPENHD_PORT = 5520         # QOpenHD HailoDetectionModel listens here

MODE_BYTE = {"AUTO": 0, "LOCKED": 1, "SEARCH": 2, "IDLE": 3}


def _norm_u16(v: float) -> int:
    """Normalize [0..1] to uint16 [0..65535] (matches the OpenHD encoder)."""
    iv = int(v * 65535.0 + 0.5)
    return max(0, min(65535, iv))


def encode_v4(report: dict) -> bytes:
    """Build the v4 binary detection payload from a drone-follow JSON report."""
    bboxes = report.get("bboxes") or []
    active_id = int(report.get("active_id") or 0)
    # follow_id sits inside the params block (DF_FOLLOW_ID maps to "follow_id").
    params = report.get("params") or {}
    follow_id_raw = params.get("follow_id", 0)
    try:
        follow_id = int(follow_id_raw)
    except (TypeError, ValueError):
        follow_id = 0
    mode_byte = MODE_BYTE.get(report.get("mode", "AUTO"), 0)

    count = min(len(bboxes), 126)
    out = bytearray(7 + count * 11)
    out[0] = 4
    struct.pack_into("<HhBB", out, 1,
                     active_id & 0xFFFF,
                     max(-32768, min(32767, follow_id)),
                     mode_byte,
                     count)
    off = 7
    for i in range(count):
        b = bboxes[i]
        struct.pack_into("<HHHHHB", out, off,
                         int(b.get("id", 0)) & 0xFFFF,
                         _norm_u16(float(b.get("cx", 0.0))),
                         _norm_u16(float(b.get("cy", 0.0))),
                         _norm_u16(float(b.get("w", 0.0))),
                         _norm_u16(float(b.get("h", 0.0))),
                         0x01 if b.get("tracked") else 0x00)
        off += 11
    return bytes(out)


def main() -> int:
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        rx.bind((LISTEN_HOST, LISTEN_PORT))
    except OSError as e:
        print(
            f"[loopback] could not bind {LISTEN_HOST}:{LISTEN_PORT} — {e}\n"
            f"           ▸ is the OpenHD daemon already running? "
            f"kill it: `sudo pkill -f openhd`",
            file=sys.stderr,
        )
        return 1
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"[loopback] listening for drone-follow JSON on "
          f"{LISTEN_HOST}:{LISTEN_PORT}, forwarding v4 binary to "
          f"{QOPENHD_HOST}:{QOPENHD_PORT}")
    print("[loopback] Ctrl-C to stop")

    n_seen = 0
    last_active = None
    last_mode = None
    while True:
        data, _ = rx.recvfrom(64 * 1024)
        n_seen += 1
        try:
            report = json.loads(data.decode("utf-8", errors="replace"))
        except json.JSONDecodeError:
            # The bridge also sends non-bboxes JSON (param sync); skip those.
            continue
        if "bboxes" not in report:
            continue
        payload = encode_v4(report)
        tx.sendto(payload, (QOPENHD_HOST, QOPENHD_PORT))

        # Compact one-liner whenever mode or active id transitions, so the
        # operator can see the loopback is alive without spamming the
        # terminal every frame.
        mode = report.get("mode", "AUTO")
        active = int(report.get("active_id") or 0)
        if mode != last_mode or active != last_active:
            print(f"[loopback] mode={mode:<6}  active_id={active:<3}  "
                  f"bboxes={len(report.get('bboxes', []))}  "
                  f"payload={len(payload)}B  total_seen={n_seen}")
            last_mode = mode
            last_active = active


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[loopback] bye")
