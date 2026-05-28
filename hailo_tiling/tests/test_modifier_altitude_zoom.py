"""AltitudeZoomModifier — validates the telemetry data-flow end-to-end."""
from __future__ import annotations

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.emitters import (
    DiscoveryGridEmitter,
    RecoveryGridEmitter,
    TrackROIEmitter,
)
from hailo_tiling.modifiers import AltitudeZoomModifier, BudgetTrimModifier
from hailo_tiling.scheduler import TileScheduler
from hailo_tiling.telemetry import NULL_SNAPSHOT, TelemetrySnapshot
from hailo_tiling.types import CropRect, LockState


def _meter():
    return BudgetMeter(budget_inf_per_s=1000.0, fps=30.0)


def test_low_altitude_widens_roi(src_dims, tracking_lock):
    src_w, src_h = src_dims
    base_roi = TrackROIEmitter(max_zoom=2.0, target_model_h=40.0, roi_margin_frac=0.25)
    base_tiles = base_roi.emit(src_w, src_h, tracking_lock, 0, _meter())

    low = TelemetrySnapshot(altitude_agl_m=5.0)
    m = AltitudeZoomModifier(zoom_at_low_agl=1.0, zoom_at_high_agl=2.0,
                              low_agl_m=5.0, high_agl_m=40.0,
                              fallback_max_zoom=2.0)
    tiles_low = m.modify(list(base_tiles), src_w, src_h, tracking_lock, 0, _meter(), low)
    assert tiles_low[0].w >= base_tiles[0].w


def test_high_altitude_narrows_roi(src_dims, tracking_lock):
    src_w, src_h = src_dims
    base_roi = TrackROIEmitter(max_zoom=2.0, target_model_h=40.0, roi_margin_frac=0.25)
    base_tiles = base_roi.emit(src_w, src_h, tracking_lock, 0, _meter())

    high = TelemetrySnapshot(altitude_agl_m=40.0)
    m = AltitudeZoomModifier(zoom_at_low_agl=1.0, zoom_at_high_agl=2.0,
                              low_agl_m=5.0, high_agl_m=40.0,
                              fallback_max_zoom=2.0)
    tiles_high = m.modify(list(base_tiles), src_w, src_h, tracking_lock, 0, _meter(), high)
    assert tiles_high[0].w <= base_tiles[0].w


def test_null_telemetry_is_passthrough(src_dims, tracking_lock):
    src_w, src_h = src_dims
    base_roi = TrackROIEmitter()
    base_tiles = list(base_roi.emit(src_w, src_h, tracking_lock, 0, _meter()))

    m = AltitudeZoomModifier()
    out = m.modify(list(base_tiles), src_w, src_h, tracking_lock, 0, _meter(), NULL_SNAPSHOT)
    assert out == base_tiles


def test_only_roi_tiles_are_modified(src_dims, tracking_lock):
    src_w, src_h = src_dims
    roi = CropRect(x=100, y=100, w=400, h=300, mode="s")
    disc = CropRect(x=0, y=0, w=1280, h=960, mode="m")
    rec = CropRect(x=200, y=200, w=640, h=480, mode="s")

    high = TelemetrySnapshot(altitude_agl_m=40.0)
    m = AltitudeZoomModifier(zoom_at_low_agl=1.0, zoom_at_high_agl=2.0,
                              low_agl_m=5.0, high_agl_m=40.0)
    out = m.modify([roi, disc, rec], src_w, src_h, tracking_lock, 0, _meter(), high)
    assert out[1] == disc
    assert out[2] == rec


def test_end_to_end_with_scheduler(src_dims, tracking_lock,
                                    low_altitude_snapshot, high_altitude_snapshot):
    src_w, src_h = src_dims
    scheduler = TileScheduler(
        emitters=[
            TrackROIEmitter(max_zoom=2.0, target_model_h=40.0, roi_margin_frac=0.25),
            DiscoveryGridEmitter(grid=(3, 2), period=15, mode="m"),
            RecoveryGridEmitter(grid=(3, 3), span=0.4),
        ],
        modifiers=[
            AltitudeZoomModifier(zoom_at_low_agl=1.0, zoom_at_high_agl=2.0,
                                  low_agl_m=5.0, high_agl_m=40.0),
            BudgetTrimModifier(),
        ],
    )
    tiles_low = scheduler.decide(src_w, src_h, tracking_lock, 0, _meter(),
                                  telemetry=low_altitude_snapshot)
    tiles_high = scheduler.decide(src_w, src_h, tracking_lock, 0, _meter(),
                                   telemetry=high_altitude_snapshot)
    assert tiles_low[0].w >= tiles_high[0].w
    assert sum(1 for t in tiles_low if t.mode == "m") == 6
    assert sum(1 for t in tiles_high if t.mode == "m") == 6


def test_altitude_below_low_clamps_to_low_zoom(src_dims, tracking_lock):
    src_w, src_h = src_dims
    base = TrackROIEmitter().emit(src_w, src_h, tracking_lock, 0, _meter())

    very_low = TelemetrySnapshot(altitude_agl_m=0.5)
    at_low = TelemetrySnapshot(altitude_agl_m=5.0)

    m = AltitudeZoomModifier(zoom_at_low_agl=1.0, zoom_at_high_agl=2.0,
                              low_agl_m=5.0, high_agl_m=40.0)
    very_low_tiles = m.modify(list(base), src_w, src_h, tracking_lock, 0, _meter(), very_low)
    at_low_tiles = m.modify(list(base), src_w, src_h, tracking_lock, 0, _meter(), at_low)
    assert very_low_tiles[0].w == at_low_tiles[0].w
