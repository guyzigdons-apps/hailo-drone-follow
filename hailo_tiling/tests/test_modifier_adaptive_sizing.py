"""AdaptiveSliceSizingModifier (ASAHI) — reshape discovery grid by object scale."""
from __future__ import annotations

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.emitters.discovery_grid import _grid_full
from hailo_tiling.modifiers import AdaptiveSliceSizingModifier
from hailo_tiling.telemetry import NULL_SNAPSHOT, TelemetrySnapshot
from hailo_tiling.types import CropRect, LockState


def _meter():
    return BudgetMeter(budget_inf_per_s=1000.0, fps=30.0)


def _disc_tiles(src_w, src_h, gx, gy):
    return _grid_full(src_w, src_h, gx, gy, "m")


def test_no_track_returns_tiles_unchanged(src_dims, lost_lock):
    src_w, src_h = src_dims
    base = _disc_tiles(src_w, src_h, 3, 2)
    m = AdaptiveSliceSizingModifier()
    out = m.modify(base, src_w, src_h, lost_lock, 0, _meter(), NULL_SNAPSHOT)
    assert out == base


def test_no_telemetry_is_degraded_mode_still_uses_track_scale(src_dims):
    src_w, src_h = src_dims
    small_target = LockState(
        track_id=1, bbox_norm=(0.5, 0.5, 0.02, 0.04),
        status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0),
    )
    base = _disc_tiles(src_w, src_h, 3, 2)
    m = AdaptiveSliceSizingModifier(target_h_thresholds=(0.05, 0.15),
                                     small_grid=(6, 4), medium_grid=(3, 2),
                                     large_grid=(2, 1))
    out = m.modify(base, src_w, src_h, small_target, 0, _meter(), NULL_SNAPSHOT)
    assert len(out) == 24


def test_large_target_coarser_grid(src_dims):
    src_w, src_h = src_dims
    big_target = LockState(
        track_id=1, bbox_norm=(0.4, 0.3, 0.3, 0.4),
        status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0),
    )
    base = _disc_tiles(src_w, src_h, 3, 2)
    m = AdaptiveSliceSizingModifier(target_h_thresholds=(0.05, 0.15),
                                     small_grid=(6, 4), medium_grid=(3, 2),
                                     large_grid=(2, 1))
    out = m.modify(base, src_w, src_h, big_target, 0, _meter(), NULL_SNAPSHOT)
    assert len(out) == 2


def test_medium_target_keeps_default_grid(src_dims):
    src_w, src_h = src_dims
    med_target = LockState(
        track_id=1, bbox_norm=(0.4, 0.3, 0.05, 0.10),
        status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0),
    )
    base = _disc_tiles(src_w, src_h, 3, 2)
    m = AdaptiveSliceSizingModifier(target_h_thresholds=(0.05, 0.15),
                                     small_grid=(6, 4), medium_grid=(3, 2),
                                     large_grid=(2, 1))
    out = m.modify(base, src_w, src_h, med_target, 0, _meter(), NULL_SNAPSHOT)
    assert len(out) == 6


def test_does_not_touch_non_discovery_tiles(src_dims, tracking_lock):
    src_w, src_h = src_dims
    roi = CropRect(x=100, y=100, w=400, h=300, mode="s")
    disc = _disc_tiles(src_w, src_h, 3, 2)
    tiles_in = [roi] + disc

    m = AdaptiveSliceSizingModifier(target_h_thresholds=(0.05, 0.15),
                                     small_grid=(6, 4), medium_grid=(3, 2),
                                     large_grid=(2, 1))
    big = LockState(track_id=1, bbox_norm=(0.4, 0.3, 0.2, 0.20),
                     status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0))
    out = m.modify(tiles_in, src_w, src_h, big, 0, _meter(), NULL_SNAPSHOT)
    assert out[0] == roi
    assert len(out) == 1 + 2


def test_returns_empty_discovery_when_no_input_discovery_tiles(src_dims, tracking_lock):
    src_w, src_h = src_dims
    roi = CropRect(x=100, y=100, w=400, h=300, mode="s")
    m = AdaptiveSliceSizingModifier()
    out = m.modify([roi], src_w, src_h, tracking_lock, 0, _meter(), NULL_SNAPSHOT)
    assert out == [roi]
