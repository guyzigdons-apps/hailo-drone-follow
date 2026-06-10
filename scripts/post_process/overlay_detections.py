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
    """Load all detection records, sorted by timestamp.

    Returns ``(header, records)``. ``header`` is the optional first
    ``{"_header": True, "tiles": [...]}`` line written by the subscriber
    when the C++ pipeline used a non-default tile geometry — None if
    absent. Skips malformed lines with a warning so a truncated final
    line (from a killed run) doesn't abort the whole post-process job.
    """
    header = None
    records = []
    skipped = 0
    with open(jsonl_path) as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError as e:
                skipped += 1
                print(f"warning: skipping malformed line {line_no} ({e})",
                      file=sys.stderr)
                continue
            if obj.get("_header") is True:
                header = obj
            else:
                records.append(obj)
    if skipped:
        print(f"warning: skipped {skipped} malformed line(s) total",
              file=sys.stderr)
    records.sort(key=lambda r: r["t"])
    return header, records


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

# Must match hailo_analytics::analytics::tiling::DEFAULT_TILES on the C++
# side (4 overlapping 60% quadrants + 1 full-frame tile). Same set is used
# in the UI overlay (App.jsx) and the Python subscriber's stats helper.
# (name, x, y, w, h, BGR-color)
TILES = [
    ("TL",   0.0, 0.0, 0.6, 0.6, (79, 213, 255)),   # #ffd54f → BGR
    ("TR",   0.4, 0.0, 0.6, 0.6, (79, 213, 255)),
    ("BL",   0.0, 0.4, 0.6, 0.6, (79, 213, 255)),
    ("BR",   0.4, 0.4, 0.6, 0.6, (79, 213, 255)),
    ("FULL", 0.0, 0.0, 1.0, 1.0, (255, 255, 255)),
]


def _draw_dashed_rect(frame, x1, y1, x2, y2, color, thickness=2,
                      dash_len=14, gap_len=10):
    """Approximate the UI's dashed tile outline with short line segments.
    OpenCV doesn't have a dashed-line primitive — we step along each edge
    and draw alternating fixed-length segments. Matches the look of
    `strokeDasharray="6 4"` in App.jsx (scaled up so it stays readable at
    4K)."""
    # Top + bottom edges
    for x in range(x1, x2, dash_len + gap_len):
        x_end = min(x + dash_len, x2)
        cv2.line(frame, (x, y1), (x_end, y1), color, thickness, cv2.LINE_AA)
        cv2.line(frame, (x, y2), (x_end, y2), color, thickness, cv2.LINE_AA)
    # Left + right edges
    for y in range(y1, y2, dash_len + gap_len):
        y_end = min(y + dash_len, y2)
        cv2.line(frame, (x1, y), (x1, y_end), color, thickness, cv2.LINE_AA)
        cv2.line(frame, (x2, y), (x2, y_end), color, thickness, cv2.LINE_AA)


def _tiles_from_header(header):
    """Decode the (name, x, y, w, h, BGR-color) list from a header dict
    written by NativePipelineSubscriber. Full-frame tiles get white,
    everything else gets the same yellow we use for the UI overlay."""
    out = []
    for t in header.get("tiles", []):
        try:
            name = t["name"]
            x = float(t["x"])
            y = float(t["y"])
            w = float(t["w"])
            h = float(t["h"])
        except (KeyError, ValueError, TypeError):
            continue
        is_full = w >= 0.999 and h >= 0.999
        color = (255, 255, 255) if is_full else (79, 213, 255)
        out.append((name, x, y, w, h, color))
    return out


def _draw_tiles(frame, w, h, tiles):
    """Render tile boundaries with labels, matching the UI overlay style."""
    for name, tx, ty, tw, th, color in tiles:
        x1 = int(tx * w)
        y1 = int(ty * h)
        x2 = int((tx + tw) * w)
        y2 = int((ty + th) * h)
        _draw_dashed_rect(frame, x1, y1, x2, y2, color, thickness=2)
        cv2.putText(frame, name, (x1 + 8, y1 + 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)


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


def draw_overlay(frame, record, w, h, show_tiles=False, tiles=None):
    """Draw bounding boxes + followed-target cross onto the frame in-place.
    ``tiles`` is the list returned by ``_tiles_from_header`` (or the
    DEFAULT_TILES fallback ``TILES``) when ``show_tiles`` is set."""
    if show_tiles:
        # Tiles drawn first so detections render on top — same z-order as
        # the live UI (SVG renders elements in document order).
        _draw_tiles(frame, w, h, tiles or TILES)
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
    ap.add_argument("--tiles", action="store_true",
                    help="Also overlay the 5 DEFAULT_TILES boundaries "
                    "(matches the UI's [show tiles] toggle). Useful for "
                    "verifying tiling spatial coverage in post.")
    args = ap.parse_args()

    video_path = args.video
    base = os.path.splitext(video_path)[0]
    jsonl_path = args.detections or (base + ".jsonl")
    out_path = args.output or (base + "_annotated.mp4")

    if not os.path.isfile(video_path):
        sys.exit(f"Video not found: {video_path}")
    if not os.path.isfile(jsonl_path):
        sys.exit(f"Detection log not found: {jsonl_path}")

    header, records = load_detections(jsonl_path)
    print(f"Loaded {len(records)} detection records ({records[0]['t']:.2f}s "
          f"→ {records[-1]['t']:.2f}s)")
    # Use tiles from header if present (custom geometry from --tiles run);
    # otherwise fall back to the DEFAULT_TILES constant. Either way only
    # consulted when --tiles is enabled.
    header_tiles = _tiles_from_header(header) if header else None
    if args.tiles and header_tiles:
        print(f"Using {len(header_tiles)} tiles from .jsonl header")
    elif args.tiles:
        print(f"Using {len(TILES)} DEFAULT_TILES (no header in .jsonl)")

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
        draw_overlay(frame, record, w, h, show_tiles=args.tiles,
                     tiles=header_tiles)
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
