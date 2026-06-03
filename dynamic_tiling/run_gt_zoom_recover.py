"""CLI: recover missed target detections with a 2x-zoom re-detection pass.

    source setup_env.sh
    python -m dynamic_tiling.run_gt_zoom_recover \
        --outdir dynamic_tiling/runs/gt_verify_0027_fov50 \
        --video <clip>_prepared__fov50.mp4 --track-ids 4,5 --zoom 2.0

For each target track, every frame MISSING inside its span gets a predicted
centre (linear interp of the bounding detections; `plan_gap_recovery`). In one
pass over the video, a zoom ROI is placed at each predicted centre and re-run
through the HEF; the ROI is ~`zoom`x more zoomed than the dense 12x9 grid, so a
small person the dense pass missed is resolved. The best same-class detection
whose centre lands inside the ROI is written back into the track. Frames where
the target is genuinely occluded yield nothing and stay gaps.

Rewrites gt_tracks.verified.json + overlay + corrections.json. NOTE: this is a
chip-dependent detection step (like the dense pass) -- the recovered boxes are
new data, recorded with params for provenance but not re-derivable offline.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2

from hailo_tiling.types import CropRect
from .aggregator import map_to_source, nms
from .gt_edit import plan_gap_recovery
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id

DENSE_TILE_FRAC = 0.108  # 12x9 grid tile width as a fraction of frame width


def _center(b):
    return (b[0] + b[2] / 2, b[1] + b[3] / 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--video", required=True, type=Path)
    ap.add_argument("--track-ids", required=True, help="targets to densify, e.g. 4,5")
    ap.add_argument("--zoom", type=float, default=2.0,
                    help="ROI zoom vs the dense 12x9 grid (default 2.0 = ~2x more zoomed)")
    ap.add_argument("--conf", type=float, default=0.3, help="min detection score to accept")
    ap.add_argument("--hef", default="/usr/local/hailo/resources/models/hailo10h/"
                                     "hailo_yolov8n_4_classes_vga.hef")
    ap.add_argument("--nms-thresh", type=float, default=0.25)
    ap.add_argument("--source", default="gt_tracks.verified.json")
    args = ap.parse_args()
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / args.source).read_text()))
    ids = [int(x) for x in args.track_ids.split(",")]
    by_id = {t.track_id: t for t in tracks}

    cap = cv2.VideoCapture(str(args.video))
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    crop_w_px = max(1, round(src_w * DENSE_TILE_FRAC / args.zoom))
    crop_w_norm = crop_w_px / src_w

    # frame -> list of (track_id, predicted (cx,cy,w,h))
    plans = {}
    for tid in ids:
        for f, pred in plan_gap_recovery(by_id[tid].frames).items():
            plans.setdefault(f, []).append((tid, pred))
    target_cls = {tid: by_id[tid].cls for tid in ids}

    from .inference import HefBackend
    backend = HefBackend(args.hef, nms_score_threshold=args.nms_thresh, class_offset=1)

    recovered = {tid: 0 for tid in ids}
    fidx = -1
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            fidx += 1
            for tid, (cx, cy, pw, ph) in plans.get(fidx, []):
                crop = CropRect.from_center_width(cx * src_w, cy * src_h, crop_w_px).clamp(src_w, src_h)
                dets = nms(map_to_source(backend.infer(frame, crop, fidx), crop, src_w, src_h),
                           iou_thr=0.5)
                cand = [d for d in dets if d.cls == target_cls[tid] and d.score >= args.conf
                        and abs((d.x + d.w / 2) - cx) < crop_w_norm
                        and abs((d.y + d.h / 2) - cy) < crop_w_norm]
                if not cand:
                    continue
                best = min(cand, key=lambda d: (d.x + d.w / 2 - cx) ** 2 + (d.y + d.h / 2 - cy) ** 2)
                by_id[tid].frames[fidx] = (best.x, best.y, best.w, best.h)
                recovered[tid] += 1
    finally:
        cap.release()
        backend.close()

    fixed = list(by_id.values()) + [t for t in tracks if t.track_id not in by_id]
    (out / "gt_tracks.verified.json").write_text(json.dumps(tracks_to_doc(fixed, clip=out.name)))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(fixed)))
    corr_path = out / "corrections.json"
    corrections = json.loads(corr_path.read_text()) if corr_path.exists() else {}
    corrections["zoom_recover"] = {"track_ids": ids, "zoom": args.zoom, "conf": args.conf,
                                   "crop_frac": round(crop_w_norm, 4), "video": str(args.video),
                                   "source": args.source}
    corr_path.write_text(json.dumps(corrections, indent=2))
    print(f"recovered frames per target: {recovered} (zoom={args.zoom}, crop={crop_w_px}px)")
    for tid in ids:
        print(f"  track {tid}: now {len(by_id[tid].frames)} frames")


if __name__ == "__main__":
    main()
