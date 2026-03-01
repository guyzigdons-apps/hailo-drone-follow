#!/usr/bin/env python3
"""
Video Bridge — connect Gazebo simulation camera to external GStreamer pipelines.

This script subscribes to the Gazebo transport topic `/camera` (protobuf images),
decodes the raw pixel data, compresses it to JPEG, and streams it via UDP
to 127.0.0.1:5600.

This allows the `drone_follow` application to run against a simulated drone
camera feed by treating the UDP stream as a video source (e.g. via udpsrc).

Requirements:
    - Must run inside the Gazebo simulation container or environment where
      `gz.transport` and `gz.msgs` are available.
    - `opencv-python` and `numpy` must be installed.
"""

import os
# Required for gz.msgs* protobuf compatibility with system-installed protoc-generated code
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import sys
import time
import socket
import cv2
import numpy as np
import traceback

# Flush stdout to ensure logs appear in docker logs immediately
def print_flush(msg):
    print(msg, flush=True)

print_flush("Video Bridge starting...")

# Try to import gz transport
try:
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    print_flush("Imported gz.transport13")
except ImportError:
    try:
        from gz.transport12 import Node
        from gz.msgs9.image_pb2 import Image
        print_flush("Imported gz.transport12")
    except ImportError:
        print_flush("Warning: Could not import gz.transport. This script must be run inside the simulation container.")
        # We continue to let it fail later or maybe mock if needed, but for now just warn.
        pass

# UDP Setup
UDP_IP = "127.0.0.1"
UDP_PORT = 5600
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Increase buffer size
sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)

print_flush(f"UDP Target: {UDP_IP}:{UDP_PORT}")

def cb(msg):
    try:
        width = msg.width
        height = msg.height
        # print_flush(f"Received image: {width}x{height}, format: {msg.pixel_format_type}")

        # Convert to numpy array
        # Assuming RGB (pixel_format_type == 2 usually)
        # Check format if needed.

        img_data = np.frombuffer(msg.data, dtype=np.uint8)
        img_data = img_data.reshape((height, width, 3))

        # Convert RGB to BGR for OpenCV
        img_data = cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR)

        # Compress to JPEG
        _, encoded_img = cv2.imencode('.jpg', img_data, [int(cv2.IMWRITE_JPEG_QUALITY), 50])

        # Send
        data = encoded_img.tobytes()
        if len(data) > 65000:
            print_flush("Warning: Image too large for UDP")
            return

        sock.sendto(data, (UDP_IP, UDP_PORT))

    except Exception as e:
        print_flush(f"Error in callback: {e}")
        traceback.print_exc()

def main():
    try:
        node = Node()
    except NameError:
        print_flush("gz.transport Node not defined. Exiting.")
        return

    topic = "/camera"

    print_flush(f"Subscribing to {topic}")
    if not node.subscribe(Image, topic, cb):
        print_flush(f"Error subscribing to {topic}")
        # Try to find topic?
        print_flush("Available topics (checking for 5s):")
        # This part requires topic listing which is not straightforward in python bindings
        # without calling service.
        return

    print_flush(f"Bridge running. Streaming to {UDP_IP}:{UDP_PORT}")
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
