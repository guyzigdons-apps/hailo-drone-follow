from __future__ import annotations

from typing import Iterable, Sequence

from .types import CropRect, Det


def map_to_source(crop_dets: Iterable, crop: CropRect, src_w: int, src_h: int) -> list[Det]:
    """Map crop-local normalized detections into normalized full-frame Dets.

    `crop_dets` items expose .cls .x .y .w .h .score normalized within the crop
    (e.g. probe_phantom_hef.Detection)."""
    out: list[Det] = []
    for d in crop_dets:
        src_x = crop.x + d.x * crop.w
        src_y = crop.y + d.y * crop.h
        src_bw = d.w * crop.w
        src_bh = d.h * crop.h
        out.append(Det(
            cls=int(d.cls), score=float(d.score),
            x=src_x / src_w, y=src_y / src_h,
            w=src_bw / src_w, h=src_bh / src_h,
        ))
    return out


def _iou(a: Det, b: Det) -> float:
    ax1, ay1, ax2, ay2 = a.xyxy
    bx1, by1, bx2, by2 = b.xyxy
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = a.w * a.h + b.w * b.h - inter
    return inter / ua if ua > 0 else 0.0


def nms(dets: Sequence[Det], iou_thr: float = 0.5) -> list[Det]:
    """Greedy per-class NMS over normalized full-frame detections."""
    kept: list[Det] = []
    for d in sorted(dets, key=lambda x: x.score, reverse=True):
        if all(not (k.cls == d.cls and _iou(k, d) >= iou_thr) for k in kept):
            kept.append(d)
    return kept
