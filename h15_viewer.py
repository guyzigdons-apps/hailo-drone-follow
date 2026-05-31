#!/usr/bin/env python3
"""H15 drone-follow viewer — receives H264 stream + detection overlay via UDP.

Run on the PC (10.0.0.2) while drone_follow_h15 runs on the H15 device.

Usage:
    python3 h15_viewer.py

Requires: opencv-python, numpy
"""

import json
import socket
import threading
import time

import cv2
import numpy as np

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

H15_IP = "10.0.0.1"
VIDEO_PORT = 5002
DET_PORT = 5003  # VIDEO_PORT + 1

WINDOW_NAME = "H15 Drone Follow"


class DetectionReceiver:
    """Receives detection JSON packets via UDP in a background thread."""

    def __init__(self, port):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", port))
        self._sock.settimeout(0.1)
        self._detections = []
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def _recv_loop(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(4096)
                dets = json.loads(data.decode())
                with self._lock:
                    self._detections = dets
            except socket.timeout:
                pass
            except Exception:
                pass

    def get(self):
        with self._lock:
            return list(self._detections)

    def stop(self):
        self._running = False
        self._sock.close()


def main():
    Gst.init(None)

    # GStreamer pipeline: low-latency H264 decode for overlay viewing
    pipeline_str = (
        f"udpsrc port={VIDEO_PORT} buffer-size=65536 "
        f'caps="application/x-rtp,encoding-name=H264" ! '
        f"rtph264depay ! h264parse ! avdec_h264 max-threads=1 ! "
        f"queue max-size-buffers=1 leaky=downstream ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink name=sink emit-signals=true sync=false drop=true max-buffers=1"
    )

    pipeline = Gst.parse_launch(pipeline_str)
    appsink = pipeline.get_by_name("sink")

    det_recv = DetectionReceiver(DET_PORT)

    pipeline.set_state(Gst.State.PLAYING)
    print(f"Listening for H264 on UDP port {VIDEO_PORT}, detections on {DET_PORT}")
    print(f"Press 'q' to quit")

    try:
        while True:
            sample = appsink.emit("try-pull-sample", int(100 * 1e6))  # 100ms timeout
            if sample is None:
                # Check for window close
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            buf = sample.get_buffer()
            caps = sample.get_caps()
            struct = caps.get_structure(0)
            w = struct.get_int("width")[1]
            h = struct.get_int("height")[1]

            success, map_info = buf.map(Gst.MapFlags.READ)
            if not success:
                continue

            frame = np.ndarray(shape=(h, w, 3), dtype=np.uint8,
                               buffer=map_info.data).copy()
            buf.unmap(map_info)

            # Draw detections
            detections = det_recv.get()
            for det in detections:
                cx = det["cx"]
                cy = det["cy"]
                bw = det.get("bw", 0) or det["bh"] * 0.5
                bh = det["bh"]
                conf = det["conf"]

                x1 = int((cx - bw / 2) * w)
                y1 = int((cy - bh / 2) * h)
                x2 = int((cx + bw / 2) * w)
                y2 = int((cy + bh / 2) * h)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{conf:.0%}", (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow(WINDOW_NAME, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)
        det_recv.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
