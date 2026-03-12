#!/usr/bin/env python3
"""Gazebo camera -> UDP JPEG bridge.

Subscribes to a Gazebo gz-transport camera topic, encodes each frame as JPEG,
and sends it as a UDP datagram.  The drone-follow app consumes this via
``--input udp://0.0.0.0:5600``.

Usage:
    sim/bridge/video_bridge.py                 # defaults
    sim/bridge/video_bridge.py --discover      # list available gz topics
    sim/bridge/video_bridge.py --port 5600 --quality 80
"""

import os
# Must be set BEFORE importing gz.msgs (protobuf version mismatch workaround)
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import argparse
import signal
import socket
import sys
import time

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node


def parse_args():
    p = argparse.ArgumentParser(description="Gazebo camera -> UDP JPEG bridge")
    p.add_argument(
        "--topic",
        default="/camera",
        help="Gazebo gz-transport image topic (default: /camera)",
    )
    p.add_argument("--host", default="127.0.0.1", help="UDP destination host (default: 127.0.0.1)")
    p.add_argument("--port", type=int, default=5600, help="UDP destination port (default: 5600)")
    p.add_argument("--quality", type=int, default=80, help="JPEG quality 0-100 (default: 80)")
    p.add_argument("--fps", type=float, default=0, help="Max FPS, 0=unlimited (default: 0)")
    p.add_argument("--discover", action="store_true", help="List available gz topics and exit")
    return p.parse_args()


# Pixel format enum values from gz.msgs.PixelFormatType
_RGB_INT8 = 3
_BGR_INT8 = 13


def main():
    args = parse_args()
    node = Node()

    if args.discover:
        print("Discovering gz-transport topics (waiting 2s)...")
        time.sleep(2)
        topics = node.topic_list()
        for t in topics:
            print(f"  {t}")
        if not topics:
            print("  (none found — is Gazebo running?)")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.port)
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, args.quality]
    min_interval = 1.0 / args.fps if args.fps > 0 else 0
    last_send = [0.0]
    frame_count = [0]

    def on_image(msg: Image):
        now = time.monotonic()
        if min_interval and (now - last_send[0]) < min_interval:
            return

        try:
            w, h = msg.width, msg.height
            fmt = msg.pixel_format_type
            data = msg.data

            if fmt == _RGB_INT8:
                channels = 3
            elif fmt == _BGR_INT8:
                channels = 3
            else:
                # Fall back to 3 channels (most common)
                channels = 3

            frame = np.frombuffer(data, dtype=np.uint8).reshape(h, w, channels)

            if fmt == _RGB_INT8:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            ok, jpeg = cv2.imencode(".jpg", frame, encode_params)
            if not ok:
                return

            sock.sendto(jpeg.tobytes(), dest)
            last_send[0] = now
            frame_count[0] += 1

            if frame_count[0] % 300 == 0:
                print(f"[bridge] Sent {frame_count[0]} frames ({w}x{h})")

        except Exception as e:
            print(f"[bridge] Error: {e}", file=sys.stderr)

    print(f"[bridge] Subscribing to: {args.topic}")
    print(f"[bridge] Sending JPEG to udp://{args.host}:{args.port} (quality={args.quality})")

    subscribed = node.subscribe(Image, args.topic, on_image)
    if not subscribed:
        print(f"[bridge] ERROR: Failed to subscribe to {args.topic}", file=sys.stderr)
        print("[bridge] Run with --discover to list available topics.", file=sys.stderr)
        sys.exit(1)

    print("[bridge] Waiting for frames... (Ctrl+C to stop)")

    # Block until interrupted
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        signal.pause()
    except AttributeError:
        # Windows fallback
        while True:
            time.sleep(1)


if __name__ == "__main__":
    main()
