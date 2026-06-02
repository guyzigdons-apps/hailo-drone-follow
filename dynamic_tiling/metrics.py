"""Tracking + recovery metric suite for single-target follow trials."""
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
class TrialScore:
    n_frames: int
    coverage: float          # fraction of GT-present frames the target is correctly followed
    mean_iou: float          # mean IoU over covered frames
    drift_rate: float        # fraction of GT-present frames the pred matched a DISTRACTOR
    # recovery fields filled by Task 6:
    loss_events: int = 0
    mean_time_to_recover: float = 0.0
    recovery_success_rate: float = 0.0


def _covered(gt_box, pred_box, iou_thr):
    return pred_box is not None and _iou(gt_box, pred_box) >= iou_thr


def score_trial(target_traj, pred_traj, *, distractors, iou_thr=0.5) -> TrialScore:
    """target_traj/pred_traj: {frame: (x,y,w,h)}. distractors: list of {frame: bbox}.
    Frames scored = frames where the target is present in GT."""
    frames = sorted(target_traj)
    n = len(frames)
    n_cov = 0
    n_drift = 0
    iou_sum = 0.0
    for f in frames:
        gt_box = target_traj[f]
        pred_box = pred_traj.get(f)
        if _covered(gt_box, pred_box, iou_thr):
            n_cov += 1
            iou_sum += _iou(gt_box, pred_box)
        elif pred_box is not None:
            # not on target — did we lock onto a different GT object?
            if any(_covered(d.get(f), pred_box, iou_thr) for d in distractors
                   if d.get(f) is not None):
                n_drift += 1
    # --- recovery: scan the covered/uncovered sequence over GT-present frames ---
    covered_seq = [_covered(target_traj[f], pred_traj.get(f), iou_thr) for f in frames]
    loss_events = 0
    recovered = 0
    ttr_sum = 0
    i = 0
    while i < n:
        if not covered_seq[i]:
            loss_events += 1
            j = i
            while j < n and not covered_seq[j]:
                j += 1
            # j == n  -> never recovered (clip ended lost); else recovered at j
            if j < n:
                recovered += 1
                ttr_sum += (j - i)
            i = j
        else:
            i += 1
    return TrialScore(
        n_frames=n,
        coverage=(n_cov / n) if n else 0.0,
        mean_iou=(iou_sum / n_cov) if n_cov else 0.0,
        drift_rate=(n_drift / n) if n else 0.0,
        loss_events=loss_events,
        mean_time_to_recover=(ttr_sum / recovered) if recovered else 0.0,
        recovery_success_rate=(recovered / loss_events) if loss_events else 0.0,
    )
