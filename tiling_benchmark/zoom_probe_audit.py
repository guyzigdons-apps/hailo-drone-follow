"""Visual audit for zoom_probe targets.

Reads a zoom_probe_*.csv and produces a contact-sheet PNG (one row per
(class, src-px bin)) showing each probed target with a fixed pixel
context margin. Lets a human eyeball the small-target bins to confirm
the GT bboxes are real persons/vehicles, not artefacts.

Usage:
    python zoom_probe_audit.py \
        --csv  pxt_runs/zoom_probe_1080p.csv \
        --video pxt_runs/scaled_sources/DJI_1080p.mp4 \
        --out  pxt_runs/zoom_probe_1080p_audit.png
"""
from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np


SRC_BIN_ORDER = [
    "(0-8]", "(8-12]", "(12-16]", "(16-24]", "(24-32]",
    "(32-64]", "(64-128]", "(128-256]", "(256+]",
]


def src_size_bin(h: float) -> str:
    if h <= 8:    return "(0-8]"
    if h <= 12:   return "(8-12]"
    if h <= 16:   return "(12-16]"
    if h <= 24:   return "(16-24]"
    if h <= 32:   return "(24-32]"
    if h <= 64:   return "(32-64]"
    if h <= 128:  return "(64-128]"
    if h <= 256:  return "(128-256]"
    return "(256+]"


def load_probe_targets(csv_path: Path) -> dict:
    """Return {(cls, src_bin): [target dict]} with unique targets per bucket."""
    rows = list(csv.DictReader(csv_path.open()))
    seen = set()
    buckets: dict = defaultdict(list)
    for r in rows:
        tid = int(r["target_id"])
        if tid in seen:
            continue
        seen.add(tid)
        cls = r["cls"]
        src_h = float(r["src_h_px"])
        bucket = (cls, src_size_bin(src_h))
        # Read native (scale ≈ 1.0) row for det / conf metadata.
        # The csv has many rows per target — pick whichever, all share src_bbox.
        buckets[bucket].append({
            "target_id": tid,
            "frame": int(r["frame"]),
            "cls": cls,
            "src_h_px": src_h,
            "src_w_px": float(r["src_w_px"]),
        })
    return buckets


def find_native_row(rows: list, target_id: int) -> dict | None:
    """Native = scale closest to 1.0."""
    cands = [r for r in rows if int(r["target_id"]) == target_id]
    if not cands:
        return None
    return min(cands, key=lambda r: abs(float(r["scale"]) - 1.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, type=Path)
    ap.add_argument("--video", required=True, type=Path)
    ap.add_argument("--gt", type=Path,
                    help="GT frames.json (to re-derive bbox normalised "
                         "coords — needed because CSV stored only the "
                         "src-pixel area, not the bbox xy origin).")
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--per-bin", type=int, default=4,
                    help="Targets shown per bin (left-to-right)")
    ap.add_argument("--cell-px", type=int, default=180,
                    help="Pixel size of each thumbnail cell (square).")
    ap.add_argument("--context-mul", type=float, default=2.5,
                    help="Crop side = max(bbox_h, bbox_w) * mul")
    args = ap.parse_args()

    if args.gt is None:
        # Default to the May-26 GT — bbox positions live there.
        args.gt = (Path(__file__).resolve().parent / "pxt_runs"
                   / "pxt_GT-12x9-25-multi.frames.json")
    import json
    gt_doc = json.load(args.gt.open())

    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise SystemExit(f"cannot open video: {args.video}")
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"source dims: {src_w}x{src_h}")

    # Index GT by frame for fast lookup.
    gt_by_frame: dict = {}
    for fr in gt_doc.get("frames", []):
        gt_by_frame[fr["frame"]] = fr.get("detections", [])

    buckets = load_probe_targets(args.csv)

    classes = ("person", "vehicle")
    rows_layout: list = []
    for cls in classes:
        for sb in SRC_BIN_ORDER:
            lst = buckets.get((cls, sb), [])
            if not lst:
                continue
            rows_layout.append((cls, sb, lst[:args.per_bin]))

    if not rows_layout:
        raise SystemExit("no buckets found in CSV")

    cell = args.cell_px
    header_h = 32
    row_label_w = 220
    pad = 6
    cols = args.per_bin
    grid_h = header_h + len(rows_layout) * (cell + pad) + pad
    grid_w = row_label_w + cols * (cell + pad) + pad
    out_img = np.full((grid_h, grid_w, 3), 245, dtype=np.uint8)

    def put_text(img, text, org, scale=0.5, color=(20, 20, 20), thick=1):
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    color, thick, cv2.LINE_AA)

    put_text(out_img, "Zoom-probe target audit — per (class, src_h_px bin) — "
             f"video {args.video.name} {src_w}x{src_h}",
             (8, 22), 0.6, (10, 10, 80), 1)

    # Frame cache.
    cache: dict = {}

    def get_frame(idx: int):
        if idx in cache: return cache[idx]
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, fr = cap.read()
        if not ok or fr is None: return None
        if len(cache) > 80: cache.pop(next(iter(cache)))
        cache[idx] = fr
        return fr

    for ri, (cls, sb, tgts) in enumerate(rows_layout):
        y0 = header_h + ri * (cell + pad) + pad
        # Row label.
        put_text(out_img, f"{cls}", (10, y0 + 22), 0.6, (180, 0, 30), 2)
        put_text(out_img, f"src_h {sb}  (n_total={len(buckets[(cls, sb)])})",
                 (10, y0 + 50), 0.55, (10, 10, 10), 1)
        put_text(out_img, f"showing {len(tgts)}/{len(buckets[(cls, sb)])}",
                 (10, y0 + 75), 0.45, (90, 90, 90), 1)

        for ci, t in enumerate(tgts):
            x0 = row_label_w + ci * (cell + pad) + pad
            fr_img = get_frame(t["frame"])
            if fr_img is None:
                continue
            # Locate the bbox in the GT for this frame & class & approximate size.
            target_h_norm_min = (t["src_h_px"] - 1) / src_h
            target_h_norm_max = (t["src_h_px"] + 1) / src_h
            cand = None
            for det in gt_by_frame.get(t["frame"], []):
                if det.get("label") != cls: continue
                bh = det["bbox"][3]
                if target_h_norm_min <= bh <= target_h_norm_max:
                    cand = det
                    break
            if cand is None:
                # Pad an empty cell with the frame number as hint.
                out_img[y0:y0+cell, x0:x0+cell] = (220, 220, 220)
                put_text(out_img, f"frame {t['frame']}", (x0+5, y0+18),
                         0.4, (20, 20, 20))
                put_text(out_img, "(no GT match)", (x0+5, y0+34),
                         0.4, (160, 0, 0))
                continue
            bx, by, bw, bh = cand["bbox"]
            cx_src = (bx + bw / 2.0) * src_w
            cy_src = (by + bh / 2.0) * src_h
            bw_src = bw * src_w
            bh_src = bh * src_h
            side = max(bw_src, bh_src) * args.context_mul
            side = max(side, 32)  # at least show some context
            x_src_lo = int(round(cx_src - side / 2))
            y_src_lo = int(round(cy_src - side / 2))
            x_src_hi = int(round(cx_src + side / 2))
            y_src_hi = int(round(cy_src + side / 2))
            # Clip to source.
            x_src_lo_c = max(0, x_src_lo)
            y_src_lo_c = max(0, y_src_lo)
            x_src_hi_c = min(src_w, x_src_hi)
            y_src_hi_c = min(src_h, y_src_hi)
            crop = fr_img[y_src_lo_c:y_src_hi_c, x_src_lo_c:x_src_hi_c]
            if crop.size == 0: continue
            # Resize to cell.
            thumb = cv2.resize(crop, (cell, cell),
                               interpolation=cv2.INTER_AREA
                               if max(crop.shape[:2]) >= cell
                               else cv2.INTER_LINEAR)
            # Draw bbox on the thumb (mapped from src crop to thumb).
            scale_x = cell / float(x_src_hi_c - x_src_lo_c)
            scale_y = cell / float(y_src_hi_c - y_src_lo_c)
            tbx1 = int(round((bx * src_w - x_src_lo_c) * scale_x))
            tby1 = int(round((by * src_h - y_src_lo_c) * scale_y))
            tbx2 = int(round(((bx + bw) * src_w - x_src_lo_c) * scale_x))
            tby2 = int(round(((by + bh) * src_h - y_src_lo_c) * scale_y))
            # BGR (OpenCV): green box for person, orange for vehicle.
            box_color = (0, 220, 0) if cls == "person" else (0, 165, 255)
            cv2.rectangle(thumb, (tbx1, tby1), (tbx2, tby2), box_color, 1)
            out_img[y0:y0+cell, x0:x0+cell] = thumb
            # Caption: src_h_px, frame.
            put_text(out_img, f"sh={t['src_h_px']:.0f} fr={t['frame']}",
                     (x0+4, y0+cell-6), 0.4, (255, 255, 255), 2)
            put_text(out_img, f"sh={t['src_h_px']:.0f} fr={t['frame']}",
                     (x0+4, y0+cell-6), 0.4, (10, 10, 10), 1)

    cap.release()
    cv2.imwrite(str(args.out), out_img)
    print(f"wrote {args.out}  ({grid_w}x{grid_h}px, "
          f"{len(rows_layout)} rows)")


if __name__ == "__main__":
    main()
