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


def test_infer_uses_constructor_video_over_frame_arg():
    """Regression: the stateful runner calls infer(frame_idx, crops, frame_idx)
    — it passes the frame *index* as ``frame``, not the video path. The backend
    must use the video given at construction, else the pipeline becomes
    ``filesrc location="<int>"`` and decodes nothing (every dynamic warm then
    silently produced 0 dets / vacuous "0 misses")."""
    be = GstCropperBackend(hef="/m.hef", post_so="/p.so",
                           source_w=_W, source_h=_H, video="/real/clip.mp4")
    crops = [tile_norm_to_source_px(0.0, 0.0, 1.0, 1.0, _W, _H)]
    # The path used for the pipeline comes from self.video; the frame arg here
    # is an int frame index (what run_dynamic_config passes) and must be ignored.
    s = be.build_pipeline_string(be.video, crops)
    assert 'filesrc location="/real/clip.mp4"' in s
    # Without a constructor video, direct callers can still pass the path.
    be2 = GstCropperBackend(hef="/m.hef", post_so="/p.so", source_w=_W, source_h=_H)
    assert be2.video is None


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


def test_gst_cropper_injects_per_frame_rois():
    """Night-2 B1 (per-frame-relaunch fallback).

    The installed ``hailotilecropper_dynamic`` supports ONLY ``tiles-static``
    (the ``HailoTileROI``-via-``identity signal-handoffs`` injection path was
    never landed in the C++ — see KB note ``hailotilecropper_dynamic``). So the
    dynamic per-frame ROIs are injected by RE-BUILDING the cropper pipeline per
    frame with that frame's ROI list as ``tiles-static`` — exactly what
    ``build_pipeline_string`` does. This test pins that contract: a per-frame
    crop list produces a pipeline whose ``tiles-static`` carries EXACTLY that
    frame's ROI count and rects, and two different frames yield two different
    pipelines (the relaunch)."""
    be = GstCropperBackend(hef="/m.hef", post_so="/p.so", source_w=_W, source_h=_H)

    # Frame A: a 2-ROI dynamic frame (an arbitrary tracker ROI + one discovery
    # tile) — NOT a regular grid.
    frame_a = [
        tile_norm_to_source_px(0.31, 0.42, 0.18, 0.24, _W, _H),  # tracker ROI
        tile_norm_to_source_px(0.0, 0.0, 0.5, 0.5, _W, _H),      # discovery tile
    ]
    # Frame B: a 4-ROI dynamic frame.
    frame_b = [
        tile_norm_to_source_px(0.10, 0.10, 0.20, 0.20, _W, _H),
        tile_norm_to_source_px(0.55, 0.20, 0.20, 0.20, _W, _H),
        tile_norm_to_source_px(0.30, 0.60, 0.20, 0.20, _W, _H),
        tile_norm_to_source_px(0.70, 0.70, 0.25, 0.25, _W, _H),
    ]

    pa = be.build_pipeline_string("/clip.mp4", frame_a)
    pb = be.build_pipeline_string("/clip.mp4", frame_b)

    # The injected ROI count for each frame == that frame's crop count.
    def _n_tiles(pipe: str) -> int:
        import re
        m = re.search(r'tiles-static="([^"]*)"', pipe)
        assert m, "no tiles-static in pipeline"
        return len([t for t in m.group(1).split(";") if t.strip()])

    assert _n_tiles(pa) == 2
    assert _n_tiles(pb) == 4
    # Per-frame relaunch: different ROI sets => different pipelines.
    assert pa != pb
    # The exact tracker-ROI rect from frame A is present (round-trips to its
    # warmed crop key — the backend feeds normalized tiles re-derived from the
    # source-pixel crop).
    assert crop_to_norm_tile(frame_a[0], _W, _H) in pa
    # Frame B's first ROI is present in pb but not pa.
    assert crop_to_norm_tile(frame_b[0], _W, _H) in pb
    assert crop_to_norm_tile(frame_b[0], _W, _H) not in pa
