"""Tests for hailo_tiling.geometry — helpers absorbed from tiling_benchmark.

Parity-with-legacy assertions are the spec: the absorbed helpers must produce
byte-identical results to the frozen originals in ``tiling_benchmark``.
"""
import pytest

from hailo_tiling.geometry import grid_to_static_tiles, fov_to_crop_dims


def test_grid_to_static_tiles_matches_legacy():
    from tiling_benchmark.tiling_record import _grid_to_static_tiles as legacy
    assert grid_to_static_tiles(3, 2, 0.25, 0.25) == legacy(3, 2, 0.25, 0.25)


def test_grid_to_static_tiles_matches_legacy_with_mode():
    from tiling_benchmark.tiling_record import _grid_to_static_tiles as legacy
    assert grid_to_static_tiles(2, 2, 0.1, 0.1, "m") == legacy(2, 2, 0.1, 0.1, "m")


def test_grid_to_static_tiles_no_overlap_covers_unit_square():
    tiles = grid_to_static_tiles(2, 2, 0.0, 0.0)
    assert len(tiles) == 4
    parsed = [tuple(float(v) for v in t.split(",")) for t in tiles]
    xs = sorted({x for x, _, _, _ in parsed})
    ys = sorted({y for _, y, _, _ in parsed})
    assert xs == [0.0, 0.5]
    assert ys == [0.0, 0.5]
    assert all(w == 0.5 and h == 0.5 for _, _, w, h in parsed)


def test_fov_to_crop_dims_matches_legacy():
    from tiling_benchmark.prepare_video import fov_to_crop_dims as legacy
    for fov in (70, 60, 50):
        assert fov_to_crop_dims(fov) == legacy(fov)
