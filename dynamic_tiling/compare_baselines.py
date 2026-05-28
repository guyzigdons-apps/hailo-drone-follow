"""Score static baseline frames.json files against the GT target trajectory and
tabulate them next to a dynamic run, all on the same single-target metric.

Static baselines have no track ids, so we reduce each frame to the single
detection best matching the GT target box that frame (the same single-target
question the dynamic harness answers)."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import dynamic_tiling  # noqa: F401
from .gt_track import build_target_trajectory
from .score import score_run, _iou


def baseline_pred_traj(frames_json: Path, gt_traj: dict, label="person") -> dict:
    doc = json.loads(frames_json.read_text())
    by_frame = {int(fr["frame"]): fr["detections"] for fr in doc["frames"]}
    pred = {}
    for f, gt_box in gt_traj.items():
        cands = [tuple(d["bbox"]) for d in by_frame.get(f, [])
                 if d.get("label") == label]
        if not cands:
            continue
        best = max(cands, key=lambda b: _iou(gt_box, b))
        if _iou(gt_box, best) > 0:
            pred[f] = best
    return pred


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True, type=Path)
    ap.add_argument("--dynamic", required=True, type=Path,
                    help="dynamic run frames.json")
    ap.add_argument("--baseline", nargs="+", type=Path, required=True,
                    help="static baseline frames.json files")
    args = ap.parse_args()

    gt_doc = json.loads(args.gt.read_text())
    gt_traj = build_target_trajectory(gt_doc, label="person", anchor="largest")

    rows = []
    for path in [args.dynamic, *args.baseline]:
        pred = baseline_pred_traj(path, gt_traj)
        sc = score_run(gt_traj, pred, iou_thr=0.5)
        rows.append((path.stem, sc.recall, sc.mean_iou, sc.n_hit, sc.n_gt_frames))

    print(f"{'run':40s} {'recall':>8s} {'mean_iou':>9s} {'hits':>6s}/{'gt':<6s}")
    for name, recall, miou, hits, ngt in rows:
        print(f"{name:40s} {recall:8.3f} {miou:9.3f} {hits:6d}/{ngt:<6d}")


if __name__ == "__main__":
    main()
