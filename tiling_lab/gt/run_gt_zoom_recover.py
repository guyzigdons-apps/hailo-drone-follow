"""CLI: recover missed target detections with a 2x-zoom re-detection pass.

    source setup_env.sh
    python -m tiling_lab.gt.run_gt_zoom_recover \
        --outdir tiling_lab/runs/gt_verify_0027_fov50 \
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
from tiling_lab.harness.aggregator import map_to_source, nms
from .gt_edit import plan_gap_recovery
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id

DENSE_TILE_FRAC = 0.108  # 12x9 grid tile width as a fraction of frame width


def _center(b):
    return (b[0] + b[2] / 2, b[1] + b[3] / 2)


def _detect_at(backend, frame, cx, cy, cls, conf, crop_w_px, src_w, src_h):
    """Zoom-detect at (cx,cy) normalized; return the best same-class det whose
    centre lands inside the ROI, or None."""
    crop = CropRect.from_center_width(cx * src_w, cy * src_h, crop_w_px).clamp(src_w, src_h)
    gate = crop_w_px / src_w
    dets = nms(map_to_source(backend.infer(frame, crop, 0), crop, src_w, src_h), iou_thr=0.5)
    cand = [d for d in dets if d.cls == cls and d.score >= conf
            and abs(d.x + d.w / 2 - cx) < gate and abs(d.y + d.h / 2 - cy) < gate]
    if not cand:
        return None
    return min(cand, key=lambda d: (d.x + d.w / 2 - cx) ** 2 + (d.y + d.h / 2 - cy) ** 2)


def _walk(cap, backend, *, start_frame, start_center, step, bound, cls, conf,
          crop_w_px, src_w, src_h, miss_tol):
    """Walk frame-by-frame from start_frame toward bound (step -1 head / +1 tail),
    re-detecting at the propagating search centre; stop after miss_tol consecutive
    misses. Returns {frame -> bbox}."""
    found = {}
    sc = start_center
    miss_run = 0
    f = start_frame
    while (f >= bound if step < 0 else f <= bound):
        cap.set(cv2.CAP_PROP_POS_FRAMES, f)
        ok, frame = cap.read()
        if not ok:
            break
        d = _detect_at(backend, frame, sc[0], sc[1], cls, conf, crop_w_px, src_w, src_h)
        if d is not None:
            found[f] = (d.x, d.y, d.w, d.h)
            sc = (d.x + d.w / 2, d.y + d.h / 2)  # propagate to follow motion
            miss_run = 0
        else:
            miss_run += 1
            if miss_run >= miss_tol:
                break
        f += step
    return found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--video", required=True, type=Path)
    ap.add_argument("--track-ids", required=True, help="targets to densify, e.g. 4,5")
    ap.add_argument("--zoom", type=float, default=2.0,
                    help="ROI zoom vs the dense 12x9 grid (default 2.0 = ~2x more zoomed)")
    ap.add_argument("--conf", type=float, default=0.3, help="min detection score to accept")
    ap.add_argument("--gaps", action="store_true", help="fill in-span gaps (sequential pass)")
    ap.add_argument("--extend-head", action="store_true",
                    help="walk backward before the first frame to recover an under-started track")
    ap.add_argument("--extend-tail", action="store_true",
                    help="walk forward after the last frame to recover an early-ended track")
    ap.add_argument("--miss-tol", type=int, default=60,
                    help="stop a head/tail walk after this many consecutive misses (default 60)")
    ap.add_argument("--hef", default="/usr/local/hailo/resources/models/hailo10h/"
                                     "hailo_yolov8n_4_classes_vga.hef")
    ap.add_argument("--nms-thresh", type=float, default=0.25)
    ap.add_argument("--source", default="gt_tracks.verified.json")
    args = ap.parse_args()
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / args.source).read_text()))
    ids = [int(x) for x in args.track_ids.split(",")]
    by_id = {t.track_id: t for t in tracks}
    # default to in-span gap fill if no mode flag is given
    do_gaps = args.gaps or not (args.extend_head or args.extend_tail)

    cap = cv2.VideoCapture(str(args.video))
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    clip_max = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) - 1
    crop_w_px = max(1, round(src_w * DENSE_TILE_FRAC / args.zoom))
    crop_w_norm = crop_w_px / src_w
    target_cls = {tid: by_id[tid].cls for tid in ids}
    recovered = {tid: 0 for tid in ids}

    from tiling_lab.harness.inference import HefBackend
    backend = HefBackend(args.hef, nms_score_threshold=args.nms_thresh, class_offset=1)

    def kw(cls):
        return dict(cls=cls, conf=args.conf, crop_w_px=crop_w_px, src_w=src_w,
                    src_h=src_h, miss_tol=args.miss_tol)

    try:
        # (1) in-span gaps — one sequential forward pass
        if do_gaps:
            plans = {}
            for tid in ids:
                for f, pred in plan_gap_recovery(by_id[tid].frames).items():
                    plans.setdefault(f, []).append((tid, pred))
            fidx = -1
            while plans:
                ok, frame = cap.read()
                if not ok:
                    break
                fidx += 1
                for tid, (cx, cy, pw, ph) in plans.get(fidx, []):
                    d = _detect_at(backend, frame, cx, cy, target_cls[tid], args.conf,
                                   crop_w_px, src_w, src_h)
                    if d is not None:
                        by_id[tid].frames[fidx] = (d.x, d.y, d.w, d.h)
                        recovered[tid] += 1
        # (2) head / tail extension — seek-based propagating walk per target
        for tid in ids:
            fis = sorted(by_id[tid].frames)
            if args.extend_head and fis[0] > 0:
                got = _walk(cap, backend, start_frame=fis[0] - 1,
                            start_center=_center(by_id[tid].frames[fis[0]]),
                            step=-1, bound=0, **kw(target_cls[tid]))
                by_id[tid].frames.update(got); recovered[tid] += len(got)
            fis = sorted(by_id[tid].frames)
            if args.extend_tail and fis[-1] < clip_max:
                got = _walk(cap, backend, start_frame=fis[-1] + 1,
                            start_center=_center(by_id[tid].frames[fis[-1]]),
                            step=+1, bound=clip_max, **kw(target_cls[tid]))
                by_id[tid].frames.update(got); recovered[tid] += len(got)
    finally:
        cap.release()
        backend.close()

    fixed = list(by_id.values()) + [t for t in tracks if t.track_id not in by_id]
    (out / "gt_tracks.verified.json").write_text(json.dumps(tracks_to_doc(fixed, clip=out.name)))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(fixed)))
    corr_path = out / "corrections.json"
    corrections = json.loads(corr_path.read_text()) if corr_path.exists() else {}
    recs = corrections.get("zoom_recover", [])
    if isinstance(recs, dict):  # back-compat with the single-dict format
        recs = [recs]
    recs.append({"track_ids": ids, "zoom": args.zoom, "conf": args.conf,
                 "crop_frac": round(crop_w_norm, 4),
                 "modes": {"gaps": do_gaps, "head": args.extend_head, "tail": args.extend_tail},
                 "miss_tol": args.miss_tol, "video": str(args.video), "source": args.source})
    corrections["zoom_recover"] = recs
    corr_path.write_text(json.dumps(corrections, indent=2))
    print(f"recovered frames per target: {recovered} (zoom={args.zoom}, crop={crop_w_px}px)")
    for tid in ids:
        print(f"  track {tid}: now {len(by_id[tid].frames)} frames")


if __name__ == "__main__":
    main()
