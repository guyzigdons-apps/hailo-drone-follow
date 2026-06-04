"""CLI: score a --dump-mot predictions file against verified GT tracks.

    source setup_env.sh
    python -m dynamic_tiling.run_mot_eval \
        --gt-tracks dynamic_tiling/runs/gt_verify_0026_fov50/gt_tracks.verified.json \
        --pred dynamic_tiling/runs/mot/dyn_0026_fov50.json \
        --classes 1 --iou-thr 0.5 \
        --out dynamic_tiling/runs/mot/dyn_0026_fov50.report.json

GT format (same as run_trials._load_tracks):
    {"tracks": [{"cls": int, "track_id": int, "frames": {frame_str: [x,y,w,h]}}]}
Pred format (run_dynamic --dump-mot):
    {"tracks": {tid_str: {frame_str: [x,y,w,h]}}}

Both are loaded into MOT dicts {id: {frame_int: (x,y,w,h)}} and scored via
mot_metrics.score_mot (greedy IoU matching)."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .mot_metrics import score_mot

# Order metrics print in.
_METRIC_ORDER = [
    "MOTA", "IDF1", "IDsw", "FP", "FN", "Frag", "MT", "ML",
    "n_gt", "n_pred", "n_frames",
]
_FLOAT_KEYS = {"MOTA", "IDF1"}


def load_gt_as_mot(path: Path, classes: set[int]) -> dict:
    """Load verified GT tracks, keep only tracks whose `cls` is in `classes`,
    return {track_id: {frame_int: (x,y,w,h)}}."""
    doc = json.loads(Path(path).read_text())
    out: dict = {}
    for t in doc["tracks"]:
        if t["cls"] not in classes:
            continue
        out[t["track_id"]] = {int(f): tuple(b) for f, b in t["frames"].items()}
    return out


def load_pred(path: Path) -> dict:
    """Load a --dump-mot predictions file -> {track_id_int: {frame_int: (x,y,w,h)}}."""
    doc = json.loads(Path(path).read_text())
    return {int(tid): {int(f): tuple(b) for f, b in traj.items()}
            for tid, traj in doc["tracks"].items()}


def format_metrics(metrics: dict) -> str:
    """Aligned two-column metric table."""
    keys = [k for k in _METRIC_ORDER if k in metrics]
    keys += [k for k in metrics if k not in keys]
    width = max(len(k) for k in keys)
    lines = []
    for k in keys:
        v = metrics[k]
        sv = f"{v:.3f}" if k in _FLOAT_KEYS else str(v)
        lines.append(f"{k:<{width}} : {sv}")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gt-tracks", required=True, type=Path,
                    help="verified GT tracks JSON (run_trials format)")
    ap.add_argument("--pred", required=True, type=Path,
                    help="predictions JSON from run_dynamic --dump-mot")
    ap.add_argument("--classes", default="1",
                    help="comma-separated GT class ids to score (default 1 = person)")
    ap.add_argument("--iou-thr", type=float, default=0.5,
                    help="IoU threshold for per-frame matching (default 0.5)")
    ap.add_argument("--out", type=Path, default=None,
                    help="write metrics + input paths + params as JSON")
    args = ap.parse_args()

    classes = [int(c) for c in args.classes.split(",") if c.strip()]
    gt = load_gt_as_mot(args.gt_tracks, set(classes))
    pred = load_pred(args.pred)

    metrics = score_mot(gt, pred, iou_thr=args.iou_thr)

    print(f"gt-tracks : {args.gt_tracks}")
    print(f"pred      : {args.pred}")
    print(f"classes   : {classes}   iou-thr : {args.iou_thr}")
    print(format_metrics(metrics))

    if args.out is not None:
        doc = {
            "metrics": metrics,
            "gt_tracks": str(args.gt_tracks),
            "pred": str(args.pred),
            "params": {"classes": classes, "iou_thr": args.iou_thr},
        }
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(doc, indent=2))
        print(f"report    : {args.out}")


if __name__ == "__main__":
    main()
