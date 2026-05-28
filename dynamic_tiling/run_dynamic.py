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

import dynamic_tiling  # noqa: F401
from .budget import BudgetMeter
from .scheduler import TileScheduler
from .target_lock import TargetLock
from .inference import HefBackend
from .gt_track import build_target_trajectory
from .score import score_run
from .replay import run, emit_frames_json


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
    ap.add_argument("--video", required=True)
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
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"cannot open video: {args.video}")
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    gt_doc = json.loads(args.gt.read_text())
    gt_traj = build_target_trajectory(gt_doc, label="person", anchor="largest")

    discovery_period = max(1, int(round(args.fps / args.discovery_fps)))
    scheduler = TileScheduler(src_w, src_h, discovery_period=discovery_period,
                              max_zoom=args.max_zoom,
                              target_model_h=args.target_model_h)
    lock = TargetLock(frame_rate=int(args.fps))
    backend = HefBackend(args.hef, nms_score_threshold=args.nms_thresh)
    meter = BudgetMeter(budget_inf_per_s=args.budget, fps=args.fps)

    try:
        res = run(_frame_iter(cap, args.max_frames), src_w, src_h,
                  scheduler, lock, backend, meter, gt_traj)
    finally:
        backend.close()
        cap.release()

    sc = score_run(gt_traj, res.pred_traj, iou_thr=0.5)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    emit_frames_json(res, label=f"dynamic-b{int(args.budget)}", out_path=args.out)

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
