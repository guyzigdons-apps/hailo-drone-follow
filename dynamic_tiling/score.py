from __future__ import annotations

from dataclasses import dataclass


def _iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = a[0], a[1], a[0] + a[2], a[1] + a[3]
    bx1, by1, bx2, by2 = b[0], b[1], b[0] + b[2], b[1] + b[3]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = a[2] * a[3] + b[2] * b[3] - inter
    return inter / ua if ua > 0 else 0.0


@dataclass
class RunScore:
    n_gt_frames: int
    n_hit: int
    recall: float
    mean_iou: float
    per_frame_iou: dict[int, float | None]  # frame_idx -> IoU, or None if no prediction that frame


def score_run(gt_traj: dict, pred_traj: dict, iou_thr: float = 0.5) -> RunScore:
    """gt_traj/pred_traj: {frame_idx: (x,y,w,h) normalized}. A frame missing from pred_traj, or present with value None, counts as a miss (IoU contribution 0)."""
    per_frame = {}
    n_hit = 0
    iou_sum = 0.0
    for f, gt_box in gt_traj.items():
        pred_box = pred_traj.get(f)
        iou = _iou(gt_box, pred_box) if pred_box is not None else None
        per_frame[f] = iou
        if iou is not None and iou >= iou_thr:
            n_hit += 1
            iou_sum += iou
    n = len(gt_traj)
    return RunScore(
        n_gt_frames=n, n_hit=n_hit,
        recall=(n_hit / n) if n else 0.0,
        mean_iou=(iou_sum / n_hit) if n_hit else 0.0,  # mean IoU over HIT frames only (iou >= iou_thr)
        per_frame_iou=per_frame,
    )


def compare(name_a: str, a: RunScore, name_b: str, b: RunScore) -> dict:
    return {
        name_a: dict(recall=a.recall, mean_iou=a.mean_iou, hits=a.n_hit,
                     n_gt_frames=a.n_gt_frames),
        name_b: dict(recall=b.recall, mean_iou=b.mean_iou, hits=b.n_hit,
                     n_gt_frames=b.n_gt_frames),
    }
