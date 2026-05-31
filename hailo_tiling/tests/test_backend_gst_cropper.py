"""Tests for GstCropperBackend (Plan 6 Task B4).

No-chip: pipeline-string construction. The chip smoke lives in
tests/integration/test_gst_cropper_chip.py (gated on HAILO_CHIP=1).
"""
from __future__ import annotations

import pytest

from hailo_tiling.backends.gst_cropper import GstCropperBackend, crop_to_norm_tile
from hailo_tiling.cache.hashing import tile_norm_to_source_px
from hailo_tiling.types import CropRect

_W, _H = 3840, 2160


def test_gst_cropper_builds_pipeline():
    be = GstCropperBackend(hef="/m.hef", post_so="/p.so", source_w=_W, source_h=_H)
    # Two crops: a full-frame tile and a centred half tile.
    crops = [
        tile_norm_to_source_px(0.0, 0.0, 1.0, 1.0, _W, _H),
        tile_norm_to_source_px(0.25, 0.25, 0.5, 0.5, _W, _H),
    ]
    s = be.build_pipeline_string("/clip.mp4", crops)

    assert "hailotilecropper_dynamic" in s
    assert "hailonet" in s
    assert "hailofilter" in s
    assert "/m.hef" in s
    assert "/p.so" in s
    # The crops appear as a tiles-static string.
    assert 'tiles-static="' in s
    assert "0.000000,0.000000,1.000000,1.000000" in s
    assert "0.250000,0.250000,0.500000,0.500000" in s
    # No extra videoscale (the cropper keeps resize internally — Task-1 caps).
    assert "videoscale" not in s
    # Aggregator present (bypass + tiles rejoined).
    assert "hailotileaggregator" in s


def test_tiles_static_round_trips_crop_keys():
    """A normalized tile -> source-pixel crop -> normalized tile round-trips to
    the same string (so feeding these tiles reproduces the warmed crop key)."""
    for (x, y, w, h) in [(0.0, 0.0, 1.0, 1.0), (0.5, 0.0, 0.5, 0.5),
                         (0.25, 0.25, 0.5, 0.5)]:
        crop = tile_norm_to_source_px(x, y, w, h, _W, _H)
        s = crop_to_norm_tile(crop, _W, _H)
        # Re-deriving the source-pixel crop from the round-tripped tile must
        # yield the same key (the cropper applies the same truncate rule).
        parts = [float(v) for v in s.split(",")]
        crop2 = tile_norm_to_source_px(parts[0], parts[1], parts[2], parts[3], _W, _H)
        assert (crop2.x, crop2.y, crop2.w, crop2.h) == (crop.x, crop.y, crop.w, crop.h)


def test_tiles_static_requires_source_dims():
    be = GstCropperBackend(hef="/m.hef", source_w=0, source_h=0)
    with pytest.raises(ValueError):
        be.tiles_static([CropRect(x=0, y=0, w=10, h=10)])


def test_infer_empty_crops_returns_empty():
    be = GstCropperBackend(hef="/m.hef", source_w=_W, source_h=_H)
    assert be.infer(None, [], 0) == []
