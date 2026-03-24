#!/usr/bin/env python3
"""Gazebo camera -> UDP video bridge.

Subscribes to a Gazebo gz-transport camera topic and sends frames over UDP.

Two output modes:
  JPEG (default) — raw JPEG datagrams for drone-follow ``--input udp://...``
  RTP  (--rtp)   — H.264 RTP stream for OpenHD air unit (Mode C, port 5500)

Usage:
    sim/bridge/video_bridge.py                          # JPEG to localhost:5600
    sim/bridge/video_bridge.py --discover               # list gz topics
    sim/bridge/video_bridge.py --rtp --host 10.0.0.2    # H.264 RTP to RPi:5500
    sim/bridge/video_bridge.py --rtp --host 10.0.0.2 --port 5500 --bitrate 4000
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
    p = argparse.ArgumentParser(description="Gazebo camera -> UDP video bridge")
    p.add_argument(
        "--topic",
        default="/camera",
        help="Gazebo gz-transport image topic (default: /camera)",
    )
    p.add_argument("--host", default="127.0.0.1", help="UDP destination host (default: 127.0.0.1)")
    p.add_argument("--port", type=int, default=None,
                   help="UDP destination port (default: 5600 for JPEG, 5500 for RTP)")
    p.add_argument("--quality", type=int, default=80, help="JPEG quality 0-100 (default: 80)")
    p.add_argument("--fps", type=float, default=0, help="Max FPS, 0=unlimited (default: 0)")
    p.add_argument("--discover", action="store_true", help="List available gz topics and exit")
    # RTP mode
    p.add_argument("--rtp", action="store_true",
                   help="Output H.264 RTP instead of JPEG (for OpenHD Mode C)")
    p.add_argument("--bitrate", type=int, default=4000,
                   help="H.264 bitrate in kbps for RTP mode (default: 4000)")
    return p.parse_args()


# Pixel format enum values from gz.msgs.PixelFormatType
_RGB_INT8 = 3
_BGR_INT8 = 13


def _parse_gz_frame(msg):
    """Convert a gz.msgs Image to a BGR numpy array."""
    w, h = msg.width, msg.height
    fmt = msg.pixel_format_type
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    if fmt == _RGB_INT8:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame


def _run_jpeg_mode(args, node):
    """Original JPEG-over-UDP mode."""
    port = args.port if args.port is not None else 5600
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, port)
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, args.quality]
    min_interval = 1.0 / args.fps if args.fps > 0 else 0
    last_send = [0.0]
    frame_count = [0]

    def on_image(msg: Image):
        now = time.monotonic()
        if min_interval and (now - last_send[0]) < min_interval:
            return
        try:
            frame = _parse_gz_frame(msg)
            ok, jpeg = cv2.imencode(".jpg", frame, encode_params)
            if not ok:
                return
            sock.sendto(jpeg.tobytes(), dest)
            last_send[0] = now
            frame_count[0] += 1
            if frame_count[0] % 300 == 0:
                print(f"[bridge] Sent {frame_count[0]} frames ({msg.width}x{msg.height})")
        except Exception as e:
            print(f"[bridge] Error: {e}", file=sys.stderr)

    print(f"[bridge] Subscribing to: {args.topic}")
    print(f"[bridge] Sending JPEG to udp://{args.host}:{port} (quality={args.quality})")

    subscribed = node.subscribe(Image, args.topic, on_image)
    if not subscribed:
        print(f"[bridge] ERROR: Failed to subscribe to {args.topic}", file=sys.stderr)
        print("[bridge] Run with --discover to list available topics.", file=sys.stderr)
        sys.exit(1)


def _run_rtp_mode(args, node):
    """H.264 RTP mode using GStreamer appsrc pipeline."""
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstApp', '1.0')
    from gi.repository import Gst, GstApp, GLib

    Gst.init(None)

    port = args.port if args.port is not None else 5500

    # Build pipeline: appsrc -> videoconvert -> x264enc -> rtph264pay -> udpsink
    pipeline_str = (
        "appsrc name=src emit-signals=false is-live=true format=time "
        "caps=video/x-raw,format=BGR,framerate=0/1 ! "
        "videoconvert ! video/x-raw,format=I420 ! "
        f"x264enc tune=zerolatency bitrate={args.bitrate} speed-preset=ultrafast "
        "key-int-max=30 ! "
        "video/x-h264,profile=baseline ! "
        f"rtph264pay config-interval=1 mtu=1400 ! "
        f"udpsink host={args.host} port={port}"
    )
    pipeline = Gst.parse_launch(pipeline_str)
    appsrc = pipeline.get_by_name("src")

    min_interval = 1.0 / args.fps if args.fps > 0 else 0
    last_send = [0.0]
    frame_count = [0]
    caps_set = [False]

    def on_image(msg: Image):
        now = time.monotonic()
        if min_interval and (now - last_send[0]) < min_interval:
            return
        try:
            frame = _parse_gz_frame(msg)
            h, w = frame.shape[:2]

            # Set caps on first frame (once we know the resolution)
            if not caps_set[0]:
                caps = Gst.Caps.from_string(
                    f"video/x-raw,format=BGR,width={w},height={h},framerate=30/1")
                appsrc.set_property("caps", caps)
                caps_set[0] = True
                print(f"[bridge] First frame: {w}x{h}, caps set")

            buf = Gst.Buffer.new_wrapped(frame.tobytes())
            # Timestamp for encoder rate control
            buf.pts = int((now - last_send[0]) * Gst.SECOND) if last_send[0] > 0 else 0
            buf.duration = Gst.SECOND // 30
            appsrc.emit("push-buffer", buf)

            last_send[0] = now
            frame_count[0] += 1
            if frame_count[0] % 300 == 0:
                print(f"[bridge] Sent {frame_count[0]} frames ({w}x{h})")
        except Exception as e:
            print(f"[bridge] Error: {e}", file=sys.stderr)

    print(f"[bridge] Subscribing to: {args.topic}")
    print(f"[bridge] Sending H.264 RTP to udp://{args.host}:{port} "
          f"(bitrate={args.bitrate} kbps)")

    subscribed = node.subscribe(Image, args.topic, on_image)
    if not subscribed:
        print(f"[bridge] ERROR: Failed to subscribe to {args.topic}", file=sys.stderr)
        print("[bridge] Run with --discover to list available topics.", file=sys.stderr)
        sys.exit(1)

    pipeline.set_state(Gst.State.PLAYING)

    # Monitor pipeline errors on the main loop
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_bus_message(bus, msg):
        if msg.type == Gst.MessageType.ERROR:
            err, debug = msg.parse_error()
            print(f"[bridge] GStreamer error: {err.message}", file=sys.stderr)
            if debug:
                print(f"[bridge] Debug: {debug}", file=sys.stderr)
        elif msg.type == Gst.MessageType.WARNING:
            warn, debug = msg.parse_warning()
            print(f"[bridge] GStreamer warning: {warn.message}", file=sys.stderr)

    bus.connect("message", on_bus_message)


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

    if args.rtp:
        _run_rtp_mode(args, node)
    else:
        _run_jpeg_mode(args, node)

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
