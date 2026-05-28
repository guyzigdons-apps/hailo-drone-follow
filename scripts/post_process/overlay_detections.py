#!/usr/bin/env python3
"""Overlay bounding boxes from a .jsonl sidecar onto a recorded .mkv video.

The H15 records two files when --record is active:
  - drone_<ts>.mkv    : H264 video from the camera (no overlays)
  - drone_<ts>.jsonl  : one JSON record per inference frame with detections

This script syncs them by timestamp and writes a new video with boxes drawn.

Usage:
    overlay_detections.py drone_xxx.mkv
    overlay_detections.py drone_xxx.mkv drone_xxx.jsonl  -o drone_xxx_annot.mp4

Requires: opencv-python  (pip install opencv-python)
"""

import argparse
import json
import os
import sys

import cv2


def load_detections(jsonl_path):
    """Load all detection records, sorted by timestamp."""
    records = []
    with open(jsonl_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            records.append(json.loads(line))
    records.sort(key=lambda r: r["t"])
    return records


def find_record_for_time(records, t, last_idx=0):
    """Find the record with the largest t <= query_t (most recent past).
    Uses last_idx as a hint since we iterate sequentially through video frames.
    """
    idx = last_idx
    while idx + 1 < len(records) and records[idx + 1]["t"] <= t:
        idx += 1
    return idx, records[idx] if records else None


CROSS_HALF_SIZE = 10      # pixels, half-length of cross arm (matches UI)
CROSS_STROKE = 3
CROSS_HALO_STROKE = 6


def _draw_cross(frame, cx, cy):
    """Draw a green '+' with black halo at (cx, cy) — matches the UI overlay."""
    # Black halo first
    cv2.line(frame, (cx - CROSS_HALF_SIZE, cy), (cx + CROSS_HALF_SIZE, cy),
             (0, 0, 0), CROSS_HALO_STROKE, cv2.LINE_AA)
    cv2.line(frame, (cx, cy - CROSS_HALF_SIZE), (cx, cy + CROSS_HALF_SIZE),
             (0, 0, 0), CROSS_HALO_STROKE, cv2.LINE_AA)
    # Green cross on top (BGR for OpenCV: #80f060 → (96, 240, 128))
    cv2.line(frame, (cx - CROSS_HALF_SIZE, cy), (cx + CROSS_HALF_SIZE, cy),
             (96, 240, 128), CROSS_STROKE, cv2.LINE_AA)
    cv2.line(frame, (cx, cy - CROSS_HALF_SIZE), (cx, cy + CROSS_HALF_SIZE),
             (96, 240, 128), CROSS_STROKE, cv2.LINE_AA)


def draw_overlay(frame, record, w, h):
    """Draw bounding boxes + followed-target cross onto the frame in-place."""
    if record is None:
        return
    following_id = record.get("following_id")
    for det in record.get("detections", []):
        bb = det.get("bbox")
        if not bb:
            continue  # skip diagnostic / malformed entries
        x1 = int(bb["x"] * w)
        y1 = int(bb["y"] * h)
        x2 = int((bb["x"] + bb["w"]) * w)
        y2 = int((bb["y"] + bb["h"]) * h)
        is_followed = (det.get("id") is not None and det["id"] == following_id)
        color = (0, 255, 0) if is_followed else (255, 255, 255)
        thickness = 3 if is_followed else 2
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        label = f"ID {det.get('id', '?')}  {int(det.get('confidence', 0) * 100)}%"
        cv2.putText(frame, label, (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)
        if is_followed:
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            _draw_cross(frame, cx, cy)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video", help="Path to .mkv recording")
    ap.add_argument("detections", nargs="?",
                    help="Path to .jsonl detections (default: <video>.jsonl)")
    ap.add_argument("-o", "--output", help="Output video path "
                    "(default: <video>_annotated.mp4)")
    ap.add_argument("--codec", default="mp4v",
                    help="OpenCV fourcc codec (mp4v, avc1, XVID; default: mp4v)")
    args = ap.parse_args()

    video_path = args.video
    base = os.path.splitext(video_path)[0]
    jsonl_path = args.detections or (base + ".jsonl")
    out_path = args.output or (base + "_annotated.mp4")

    if not os.path.isfile(video_path):
        sys.exit(f"Video not found: {video_path}")
    if not os.path.isfile(jsonl_path):
        sys.exit(f"Detection log not found: {jsonl_path}")

    records = load_detections(jsonl_path)
    print(f"Loaded {len(records)} detection records ({records[0]['t']:.2f}s "
          f"→ {records[-1]['t']:.2f}s)")

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        sys.exit(f"Failed to open video: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"Video: {w}x{h} @ {fps:.2f} fps, {total} frames")

    fourcc = cv2.VideoWriter_fourcc(*args.codec)
    writer = cv2.VideoWriter(out_path, fourcc, fps, (w, h))
    if not writer.isOpened():
        sys.exit(f"Failed to open writer for: {out_path}")

    last_idx = 0
    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        t = frame_idx / fps
        last_idx, record = find_record_for_time(records, t, last_idx)
        draw_overlay(frame, record, w, h)
        writer.write(frame)
        frame_idx += 1
        if frame_idx % 100 == 0:
            print(f"  Processed {frame_idx}/{total} frames "
                  f"({100 * frame_idx / max(total, 1):.0f}%)", end="\r")

    cap.release()
    writer.release()
    print(f"\nWrote {frame_idx} annotated frames to {out_path}")


if __name__ == "__main__":
    main()
