"""Unit tests for hailo_tiling.types (the canonical home for the shared types).

The former dynamic-tiling ``types`` re-export shim was removed by the
tiling-lab restructure (2026-06-07); its identity-parity test went with it.
"""
from __future__ import annotations


def test_model_constants():
    from hailo_tiling.types import MODEL_W, MODEL_H, MODEL_ASPECT
    assert MODEL_W == 640
    assert MODEL_H == 480
    assert MODEL_ASPECT == 640 / 480


def test_croprect_clamp_keeps_w_h():
    from hailo_tiling.types import CropRect
    r = CropRect(x=-10, y=-5, w=100, h=75, mode="s").clamp(3840, 2160)
    assert r.x == 0 and r.y == 0
    assert r.w == 100 and r.h == 75
