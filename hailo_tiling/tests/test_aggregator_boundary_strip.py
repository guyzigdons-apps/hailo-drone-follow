"""BoundaryStripFilter — Python port of C++ remove_exceeded_bboxes."""
from __future__ import annotations

from hailo_tiling.aggregator import BoundaryStripFilter
from hailo_tiling.types import CropRect, Det


def _tile_at(x, y, w, h, mode="m") -> CropRect:
    return CropRect(x=x, y=y, w=w, h=h, mode=mode)


SRC_W, SRC_H = 3840, 2160


def test_passes_dets_not_near_any_boundary():
    tile = _tile_at(0, 0, 1280, 960, mode="m")
    det = Det(cls=0, score=0.9, x=0.10, y=0.10, w=0.05, h=0.05)
    flt = BoundaryStripFilter(border_threshold=0.005)
    out = flt.filter([det], [tile], SRC_W, SRC_H)
    assert out == [det]


def test_drops_det_touching_tile_interior_right_edge():
    tile = _tile_at(0, 0, 1280, 960, mode="m")
    det = Det(cls=0, score=0.9, x=0.330, y=0.10, w=0.005, h=0.05)
    flt = BoundaryStripFilter(border_threshold=0.005)
    out = flt.filter([det], [tile], SRC_W, SRC_H)
    assert out == []


def test_keeps_det_touching_frame_edge_even_at_tile_boundary():
    tile = _tile_at(2560, 0, 1280, 960, mode="m")
    det = Det(cls=0, score=0.9, x=0.99, y=0.10, w=0.01, h=0.05)
    flt = BoundaryStripFilter(border_threshold=0.005)
    out = flt.filter([det], [tile], SRC_W, SRC_H)
    assert out == [det]


def test_single_scale_tiles_are_exempt_from_strip():
    tile = _tile_at(0, 0, 1280, 960, mode="s")
    det = Det(cls=0, score=0.9, x=0.330, y=0.10, w=0.005, h=0.05)
    flt = BoundaryStripFilter(border_threshold=0.005)
    out = flt.filter([det], [tile], SRC_W, SRC_H)
    assert out == [det]


def test_default_threshold_is_0_005():
    flt = BoundaryStripFilter()
    assert flt.border_threshold == 0.005


def test_threshold_zero_disables_strip():
    tile = _tile_at(0, 0, 1280, 960, mode="m")
    det = Det(cls=0, score=0.9, x=0.330, y=0.10, w=0.005, h=0.05)
    flt = BoundaryStripFilter(border_threshold=0.0)
    out = flt.filter([det], [tile], SRC_W, SRC_H)
    assert out == [det]


def test_multiple_tiles_only_strips_against_originating_tile_mode():
    tile_m = _tile_at(0, 0, 1280, 960, mode="m")
    tile_s = _tile_at(1280, 0, 1280, 960, mode="s")
    det = Det(cls=0, score=0.9, x=0.330, y=0.10, w=0.005, h=0.05)
    flt = BoundaryStripFilter(border_threshold=0.005)
    out = flt.filter([det], [tile_m, tile_s], SRC_W, SRC_H)
    assert out == []
