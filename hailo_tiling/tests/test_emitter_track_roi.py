# hailo_tiling/tests/test_emitter_track_roi.py
"""TrackROIEmitter — predicted-bbox ROI tile while TRACKING."""
from __future__ import annotations

import pytest

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.emitters.track_roi import TrackROIEmitter
from hailo_tiling.types import LockState


def _meter():
    return BudgetMeter(budget_inf_per_s=1000.0, fps=30.0)


def test_emits_one_tile_when_tracking(src_dims, tracking_lock):
    src_w, src_h = src_dims
    e = TrackROIEmitter(max_zoom=2.0, target_model_h=40.0, roi_margin_frac=0.25)
    out = e.emit(src_w, src_h, tracking_lock, frame_idx=0, meter=_meter())
    assert len(out) == 1
    assert out[0].mode == "s"


def test_emits_empty_when_not_tracking(src_dims, searching_lock, lost_lock):
    src_w, src_h = src_dims
    e = TrackROIEmitter()
    assert e.emit(src_w, src_h, searching_lock, 0, _meter()) == []
    assert e.emit(src_w, src_h, lost_lock, 0, _meter()) == []


def test_matches_legacy_roi(src_dims, tracking_lock):
    """Exact output equality with the legacy `_roi` implementation."""
    from hailo_tiling.dynamic.scheduler import TileScheduler as LegacyScheduler
    src_w, src_h = src_dims
    legacy = LegacyScheduler(src_w, src_h, max_zoom=2.0, target_model_h=40.0,
                             roi_margin_frac=0.25)
    expected = legacy._roi(tracking_lock)

    e = TrackROIEmitter(max_zoom=2.0, target_model_h=40.0, roi_margin_frac=0.25)
    actual = e.emit(src_w, src_h, tracking_lock, 0, _meter())

    assert actual == [expected]
