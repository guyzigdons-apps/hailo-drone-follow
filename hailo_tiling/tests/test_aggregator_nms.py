"""NMS + map_to_source unit tests.

These helpers were lifted verbatim from the legacy aggregator (now
``tiling_lab.harness.aggregator``); the cross-package identity-parity check was
dropped when the lab moved out of ``hailo_tiling``'s allowed import set
(tiling-lab restructure, 2026-06-07).
"""
from __future__ import annotations

from hailo_tiling.aggregator import map_to_source, nms
from hailo_tiling.types import CropRect, Det


class _CropLocalDet:
    """Mimics probe_phantom_hef.Detection (normalized in crop)."""
    def __init__(self, cls, x, y, w, h, score):
        self.cls, self.x, self.y, self.w, self.h, self.score = cls, x, y, w, h, score


def test_map_to_source_places_box_correctly():
    crop = CropRect(x=1000, y=500, w=640, h=480)
    d = _CropLocalDet(cls=0, x=0.25, y=0.25, w=0.5, h=0.5, score=0.9)
    out = map_to_source([d], crop, src_w=4000, src_h=3000)
    assert len(out) == 1
    o = out[0]
    assert abs(o.x - 1160 / 4000) < 1e-6
    assert abs(o.y - (500 + 0.25 * 480) / 3000) < 1e-6
    assert abs(o.w - (0.5 * 640) / 4000) < 1e-6
    assert abs(o.h - (0.5 * 480) / 3000) < 1e-6
    assert o.cls == 0
    assert abs(o.score - 0.9) < 1e-6


def test_nms_merges_overlapping_same_class():
    a = Det(cls=0, score=0.9, x=0.10, y=0.10, w=0.10, h=0.20)
    b = Det(cls=0, score=0.7, x=0.105, y=0.105, w=0.10, h=0.20)
    c = Det(cls=0, score=0.8, x=0.60, y=0.60, w=0.10, h=0.20)
    kept = nms([a, b, c], iou_thr=0.5)
    assert len(kept) == 2


def test_nms_keeps_different_classes():
    a = Det(cls=0, score=0.9, x=0.10, y=0.10, w=0.10, h=0.20)
    b = Det(cls=1, score=0.7, x=0.10, y=0.10, w=0.10, h=0.20)
    assert len(nms([a, b], iou_thr=0.5)) == 2
