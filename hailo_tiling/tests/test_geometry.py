"""Tests for hailo_tiling.geometry — helpers absorbed from tiling_benchmark.

These helpers were copied verbatim from the frozen ``tiling_benchmark``
originals (provenance commit 7d9a8d9). The parity-with-legacy outputs are
pinned here directly as the spec: importing the frozen ``tiling_benchmark``
package from ``hailo_tiling`` would violate the package dependency rules
(see ``tests/test_architecture.py``), so the expected byte-identical results
are baked in as literals instead.
"""
import pytest

from hailo_tiling.geometry import grid_to_static_tiles, fov_to_crop_dims


def test_grid_to_static_tiles_matches_legacy():
    # Verbatim output of the frozen tiling_benchmark._grid_to_static_tiles(3,2,0.25,0.25).
    assert grid_to_static_tiles(3, 2, 0.25, 0.25) == [
        "0.000000,0.000000,0.400000,0.571429",
        "0.300000,0.000000,0.400000,0.571429",
        "0.600000,0.000000,0.400000,0.571429",
        "0.000000,0.428571,0.400000,0.571429",
        "0.300000,0.428571,0.400000,0.571429",
        "0.600000,0.428571,0.400000,0.571429",
    ]


def test_grid_to_static_tiles_matches_legacy_with_mode():
    # Verbatim output of the frozen legacy helper with the "m" mode suffix.
    assert grid_to_static_tiles(2, 2, 0.1, 0.1, "m") == [
        "0.000000,0.000000,0.526316,0.526316,m",
        "0.473684,0.000000,0.526316,0.526316,m",
        "0.000000,0.473684,0.526316,0.526316,m",
        "0.473684,0.473684,0.526316,0.526316,m",
    ]


def test_grid_to_static_tiles_no_overlap_covers_unit_square():
    tiles = grid_to_static_tiles(2, 2, 0.0, 0.0)
    assert len(tiles) == 4
    parsed = [tuple(float(v) for v in t.split(",")) for t in tiles]
    xs = sorted({x for x, _, _, _ in parsed})
    ys = sorted({y for _, y, _, _ in parsed})
    assert xs == [0.0, 0.5]
    assert ys == [0.0, 0.5]
    assert all(w == 0.5 and h == 0.5 for _, _, w, h in parsed)


def test_fov_to_crop_dims_known_spec_values():
    # Spec §8.2 table values (the former legacy parity target — prepare_video
    # now re-exports this very function as the single source of truth, so the
    # cross-module identity check became tautological after the tiling-lab
    # restructure; pin the published values directly instead).
    assert fov_to_crop_dims(70) == (6016, 3384)
    assert fov_to_crop_dims(60) == (4963, 2792)
    assert fov_to_crop_dims(50) == (4007, 2254)
