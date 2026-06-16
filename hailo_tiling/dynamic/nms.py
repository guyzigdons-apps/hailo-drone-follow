from __future__ import annotations

__all__ = ["nms_merge"]


def _iou(a, b):
    ax, ay, aw, ah = a; bx, by, bw, bh = b
    ix1, iy1 = max(ax, bx), max(ay, by)
    ix2, iy2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = aw * ah + bw * bh - inter
    return inter / ua if ua > 0 else 0.0


def _priority(d):
    return (1 if d.get("label") == "target" else 0,
            -d.get("age", 0),
            d.get("confidence", 0.0))


def nms_merge(detections, iou_thresh=0.3):
    """Greedy full-frame NMS over a union of detections (visualizer-schema dicts
    with an optional `age`). On overlap >= iou_thresh, keep the higher-priority
    box (target > freshest > most-confident) and drop the others."""
    kept = []
    for d in sorted(detections, key=_priority, reverse=True):
        if any(_iou(d["bbox"], k["bbox"]) >= iou_thresh for k in kept):
            continue
        kept.append(d)
    return kept
