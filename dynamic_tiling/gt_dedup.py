"""Per-frame fragment de-duplication of dense detections (offline GT prep).

Dense tiling fragments one object into several overlapping boxes. Greedy
same-class NMS collapses them: sort by confidence, keep a box, drop later
same-class boxes whose IoU with it exceeds the threshold.
"""
from __future__ import annotations


def _iomin(a, b) -> float:
    """Intersection over minimum area (IoMin).

    Standard IoU underestimates overlap when one tile-fragment is a cropped
    sub-region of the original box.  IoMin = inter / min(area_a, area_b)
    rises to 1.0 whenever the smaller box is fully contained in the larger
    one, making it the right metric for tile-fragment dedup.
    """
    ax1, ay1, ax2, ay2 = a[0], a[1], a[0] + a[2], a[1] + a[3]
    bx1, by1, bx2, by2 = b[0], b[1], b[0] + b[2], b[1] + b[3]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    min_area = min(a[2] * a[3], b[2] * b[3])
    return inter / min_area if min_area > 0 else 0.0


def dedup_frame(dets, *, iou_thr: float = 0.5):
    """Greedy same-class NMS. dets: list of {bbox:[x,y,w,h], confidence, cls}.
    Returns the kept detections (highest-confidence representative per cluster)."""
    order = sorted(dets, key=lambda d: -float(d.get("confidence", 0.0)))
    kept = []
    for d in order:
        b, c = d["bbox"], int(d.get("cls", -1))
        # IoMin treats a contained same-class box as a duplicate; a genuinely
        # distinct small object inside a larger one's box can be suppressed here
        # (recoverable downstream via keep_short flags + human review).
        if any(int(k.get("cls", -1)) == c and _iomin(b, k["bbox"]) > iou_thr for k in kept):
            continue
        kept.append(d)
    return kept


def dedup_doc(doc, *, iou_thr: float = 0.5) -> dict:
    """Return a new doc with each frame's detections de-duplicated."""
    return {**{k: v for k, v in doc.items() if k != "frames"},
            "frames": [{**fr, "detections": dedup_frame(fr.get("detections", []),
                                                         iou_thr=iou_thr)}
                       for fr in doc.get("frames", [])]}
