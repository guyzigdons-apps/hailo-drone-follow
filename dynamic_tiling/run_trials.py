"""CLI: video + gt_tracks.json -> per-trial + aggregate tracking/recovery metrics.

    source setup_env.sh
    python -m dynamic_tiling.run_trials \
        --video /home/giladn/Videos/Drone/Training/DJI_..._0026_..._fov50.mp4 \
        --gt-tracks dynamic_tiling/runs/gt_tracks_0026_fov50.json \
        --budget 40 --fps 30 --discovery-fps 2 --discovery-grid 8x6
"""
from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import cv2

from .gt_clean import GtTrack
from .trials import run_all_trials, AggregateScore


def _load_tracks(path: Path):
    doc = json.loads(path.read_text())
    return [GtTrack(cls=t["cls"], track_id=t["track_id"],
                    frames={int(f): tuple(b) for f, b in t["frames"].items()})
            for t in doc["tracks"]]


def _frames_factory(video: Path, max_frames: int):
    def gen():
        cap = cv2.VideoCapture(str(video))
        n = 0
        try:
            while True:
                ok, fr = cap.read()
                if not ok or (max_frames > 0 and n >= max_frames):  # max_frames<=0 means no limit
                    break
                yield fr
                n += 1
        finally:
            cap.release()
    return gen


def format_aggregate(agg: AggregateScore, *, budget: float, fps: float) -> str:
    return (
        f"trials           : {agg.n_trials}\n"
        f"tiles/frame      : {agg.avg_tiles_per_frame:.3f}  (budget {budget} @ {fps}fps)\n"
        f"coverage         : {agg.mean_coverage:.3f}\n"
        f"mean IoU         : {agg.mean_iou:.3f}\n"
        f"drift rate       : {agg.mean_drift_rate:.3f}\n"
        f"loss events      : {agg.mean_loss_events:.2f}\n"
        f"time-to-recover  : {agg.mean_time_to_recover:.2f}\n"
        f"recovery success : {agg.mean_recovery_success:.3f}\n"
    )


def results_doc(agg: AggregateScore, *, person_track_ids, params) -> dict:
    """Machine-readable results: run params + aggregate + per-trial (one per
    person GT track, in run order — run_all_trials iterates person tracks in
    gt_tracks order, so person_track_ids must be that same order)."""
    aggregate = {k: v for k, v in asdict(agg).items() if k != "per_trial"}
    per_trial = [{"track_id": tid, **asdict(t)}
                 for tid, t in zip(person_track_ids, agg.per_trial, strict=True)]
    return {"params": params, "aggregate": aggregate, "per_trial": per_trial}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True, type=Path)
    ap.add_argument("--gt-tracks", required=True, type=Path)
    ap.add_argument("--hef",
                    default="/usr/local/hailo/resources/models/hailo10h/"
                            "hailo_yolov8n_4_classes_vga.hef")
    ap.add_argument("--budget", type=float, default=40.0)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--discovery-fps", type=float, default=2.0)
    ap.add_argument("--discovery-grid", default=None)
    ap.add_argument("--max-zoom", type=float, default=2.0)
    ap.add_argument("--discovery-overlap", type=float, default=0.25,
                    help="fraction of a grid cell shared with each neighbour "
                         "(matches the static-ablation 0.25 convention; 0 = edge-to-edge)")
    ap.add_argument("--target-model-h", type=float, default=40.0)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--nms-thresh", type=float, default=0.25)
    ap.add_argument("--out", type=Path, default=None,
                    help="write params + aggregate + per-trial results as JSON")
    ap.add_argument("--dump-frames", type=Path, default=None,
                    help="dir: write a replayable trial_<id>.frames.json per trial "
                         "(dets + tiles w/ categories + LOCK box) for the overlay viewer")
    ap.add_argument("--cache", type=Path, default=None,
                    help="SQLite tile-cache path; repeated (frame, crop) inferences are "
                         "served from it (a fully-warm cache needs no chip)")
    args = ap.parse_args()

    discovery_grid = None
    if args.discovery_grid:
        gx, gy = args.discovery_grid.lower().split("x")
        discovery_grid = (int(gx), int(gy))

    cap = cv2.VideoCapture(str(args.video))
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()

    tracks = _load_tracks(args.gt_tracks)

    def backend_factory():
        if args.cache:
            from .inference import CachedHefBackend
            return CachedHefBackend(
                args.hef, nms_score_threshold=args.nms_thresh, class_offset=1,
                cache_path=args.cache,
                meta={"video": args.video.name, "hef": args.hef,
                      "nms_thresh": args.nms_thresh, "class_offset": 1,
                      "det_coords": "crop_local"})
        from .inference import HefBackend
        return HefBackend(args.hef, nms_score_threshold=args.nms_thresh, class_offset=1)

    on_result = None
    if args.dump_frames:
        from .replay import emit_frames_json
        args.dump_frames.mkdir(parents=True, exist_ok=True)

        def on_result(track_id, res):
            out = args.dump_frames / f"trial_{track_id}.frames.json"
            emit_frames_json(res, label=f"trial-{track_id}-b{int(args.budget)}", out_path=out)
            print(f"frames -> {out}")

    agg = run_all_trials(
        frames_factory=_frames_factory(args.video, args.max_frames),
        src_w=src_w, src_h=src_h, gt_tracks=tracks,
        backend_factory=backend_factory,
        budget=args.budget, fps=args.fps, discovery_fps=args.discovery_fps,
        max_zoom=args.max_zoom, target_model_h=args.target_model_h,
        discovery_grid=discovery_grid, grid_overlap=args.discovery_overlap,
        on_result=on_result)

    print(format_aggregate(agg, budget=args.budget, fps=args.fps))

    if args.out:
        from hailo_tiling.classes import PERSON
        params = {"video": str(args.video), "gt_tracks": str(args.gt_tracks),
                  "hef": args.hef, "budget": args.budget, "fps": args.fps,
                  "discovery_fps": args.discovery_fps,
                  "discovery_grid": args.discovery_grid,
                  "discovery_overlap": args.discovery_overlap, "max_zoom": args.max_zoom,
                  "target_model_h": args.target_model_h,
                  "max_frames": args.max_frames, "nms_thresh": args.nms_thresh}
        person_ids = [t.track_id for t in tracks if t.cls == PERSON]
        args.out.write_text(json.dumps(
            results_doc(agg, person_track_ids=person_ids, params=params), indent=2))
        print(f"results -> {args.out}")


if __name__ == "__main__":
    main()
