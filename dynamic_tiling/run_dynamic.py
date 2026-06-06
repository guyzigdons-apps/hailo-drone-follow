"""Run the dynamic tile scheduler over a recorded video with real inference,
score the locked target vs the GT, and emit a viewer-compatible frames.json.

Example:
    source setup_env.sh
    python -m dynamic_tiling.run_dynamic \
        --video /home/giladn/Videos/Drone/Training/DJI_20260430103421_0010_D_rotated.MP4 \
        --gt tiling_benchmark/pxt_runs/pxt_GT-12x9-25-multi.frames.json \
        --hef /usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef \
        --budget 300 --fps 30 --discovery-fps 2 \
        --out dynamic_tiling/runs/dynamic_run.frames.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2

from .budget import BudgetMeter
from .scheduler import TileScheduler, MultiTargetTileScheduler
from .target_lock import TargetLock, MultiTargetLock
from .inference import HefBackend
from .gt_track import build_target_trajectory
from .score import score_run
from .replay import run, run_multi, emit_frames_json
from hailo_tiling.classes import PERSON


def _frame_iter(cap, max_frames):
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok or (max_frames and n >= max_frames):
            break
        yield frame
        n += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True, type=Path)
    ap.add_argument("--gt", required=True, type=Path)
    ap.add_argument("--hef",
                    default="/usr/local/hailo/resources/models/hailo10h/"
                            "hailo_yolov8n_4_classes_vga.hef")
    ap.add_argument("--budget", type=float, default=300.0)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--discovery-fps", type=float, default=2.0)
    ap.add_argument("--max-zoom", type=float, default=2.0)
    ap.add_argument("--target-model-h", type=float, default=40.0)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--nms-thresh", type=float, default=0.25)
    ap.add_argument("--out", type=Path,
                    default=Path("dynamic_tiling/runs/dynamic_run.frames.json"))
    ap.add_argument("--multi-target", action="store_true",
                    help="Use multi-target dynamic tiling (v2) instead of single-target (v1).")
    ap.add_argument("--target-classes", default="1,2",
                    help="Comma-separated class ids for multi-target mode "
                    "(default: 1,2 = person,vehicle per hailo_tiling.classes; "
                    "the network emits person=1, vehicle=2, with index 0 the "
                    "'unlabeled' slot).")
    ap.add_argument("--merge-iou-threshold", type=float, default=0.5,
                    help="Padded det IoU threshold for ROI merge (default: 0.5).")
    ap.add_argument("--merge-pad-frac", type=float, default=0.25,
                    help="Padding fraction applied to each bbox before IoU (default: 0.25).")
    ap.add_argument("--merge-union-inflate-max", type=float, default=1.5,
                    help="Max ratio union crop_w / max(crop_w_a, crop_w_b) (default: 1.5).")
    ap.add_argument("--discovery-grid", default=None,
                    help="Discovery grid density as WxH (e.g. 8x6 for dense "
                    "whole-frame detection on cadence). Default: scheduler's 3x2.")
    ap.add_argument("--dump-mot", type=Path, default=None,
                    help="After a --multi-target run, write per-track predictions "
                    "(MOT scorecard format) to this JSON path.")
    ap.add_argument("--dump-mot-classes", default="1",
                    help="Comma-separated class ids to keep in --dump-mot "
                    "(default: 1 = person-only).")
    args = ap.parse_args()
    discovery_grid = None
    if args.discovery_grid:
        _gx, _gy = args.discovery_grid.lower().split("x")
        discovery_grid = (int(_gx), int(_gy))

    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise SystemExit(f"cannot open video: {args.video}")
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    gt_doc = json.loads(args.gt.read_text())
    gt_traj = build_target_trajectory(gt_doc, label="person", anchor="largest")

    discovery_period = max(1, int(round(args.fps / args.discovery_fps)))
    # class_offset=1: unified person=1/vehicle=2 convention (Phase 0); without it
    # the raw 0-indexed decode made --target-classes 1,2 track vehicles.
    backend = HefBackend(args.hef, nms_score_threshold=args.nms_thresh, class_offset=1)
    meter = BudgetMeter(budget_inf_per_s=args.budget, fps=args.fps)

    if args.multi_target:
        target_classes = {int(c) for c in args.target_classes.split(",")}
        _disc_kw = {"discovery_grid": discovery_grid} if discovery_grid else {}
        scheduler = MultiTargetTileScheduler(src_w, src_h,
                                             discovery_period=discovery_period,
                                             max_zoom=args.max_zoom,
                                             target_model_h=args.target_model_h,
                                             merge_iou_threshold=args.merge_iou_threshold,
                                             merge_pad_frac=args.merge_pad_frac,
                                             merge_union_inflate_max=args.merge_union_inflate_max,
                                             **_disc_kw)
        lock = MultiTargetLock(target_classes=target_classes,
                               track_buffer=int(args.fps))
        try:
            res = run_multi(_frame_iter(cap, args.max_frames), src_w, src_h,
                            scheduler, lock, backend, meter, gt_traj,
                            gt_cls=PERSON)
        finally:
            backend.close()
            cap.release()
    else:
        _disc_kw = {"discovery_grid": discovery_grid} if discovery_grid else {}
        scheduler = TileScheduler(src_w, src_h, discovery_period=discovery_period,
                                  max_zoom=args.max_zoom,
                                  target_model_h=args.target_model_h, **_disc_kw)
        lock = TargetLock(frame_rate=int(args.fps))
        try:
            res = run(_frame_iter(cap, args.max_frames), src_w, src_h,
                      scheduler, lock, backend, meter, gt_traj)
        finally:
            backend.close()
            cap.release()

    # Score only over frames actually played (matters when --max-frames is set);
    # otherwise recall denominator includes frames we never had a chance to detect on.
    gt_for_score = {f: bb for f, bb in gt_traj.items() if f < res.n_frames}
    sc = score_run(gt_for_score, res.pred_traj, iou_thr=0.5)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    emit_frames_json(res, label=f"dynamic-b{int(args.budget)}", out_path=args.out)

    if args.dump_mot is not None:
        if not args.multi_target:
            raise SystemExit("--dump-mot requires --multi-target")
        dump_classes = {int(c) for c in args.dump_mot_classes.split(",")}
        by_track: dict = {}
        for f, d in res.multi_traj.items():
            for (cls, tid), bb in d.items():
                if cls not in dump_classes:
                    continue
                by_track.setdefault(tid, {})[f] = bb
        args.dump_mot.parent.mkdir(parents=True, exist_ok=True)
        args.dump_mot.write_text(json.dumps({
            "tracks": {str(t): {str(f): list(b) for f, b in traj.items()}
                       for t, traj in by_track.items()}}))
        print(f"mot dump         : {args.dump_mot}  "
              f"({len(by_track)} tracks, classes {sorted(dump_classes)})")

    print(f"\nframes processed : {res.n_frames}")
    if res.avg_tiles_per_frame:
        print(f"avg tiles/frame  : {res.avg_tiles_per_frame:.2f}  "
              f"(rt_factor {args.budget / (res.avg_tiles_per_frame * args.fps):.2f})")
    else:
        print("avg tiles/frame  : 0")
    print(f"GT target frames : {sc.n_gt_frames}")
    print(f"target recall    : {sc.recall:.3f}")
    print(f"target mean IoU  : {sc.mean_iou:.3f}")
    print(f"frames.json      : {args.out}")


if __name__ == "__main__":
    main()
