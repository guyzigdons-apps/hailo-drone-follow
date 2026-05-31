"""Tests for hailo_tiling.bench.metrics (matched-compute delta — Night-2 B3)."""
from __future__ import annotations

from hailo_tiling.bench.metrics import matched_compute_delta


_STATIC = [
    {"name": "1x1", "mean_tiles": 1.0, "recall": 0.28},
    {"name": "2x2", "mean_tiles": 4.0, "recall": 0.34},
    {"name": "6x4", "mean_tiles": 24.0, "recall": 0.44},
    {"name": "12x9", "mean_tiles": 108.0, "recall": 1.0},
]


def test_matched_compute_picks_closest_static_budget():
    # A dynamic row at 0.4 tiles/frame matches the 1x1 static grid (closest).
    name, delta = matched_compute_delta(0.4, 0.05, _STATIC)
    assert name == "1x1"
    assert delta == 0.05 - 0.28


def test_matched_compute_mid_budget():
    # 5 tiles/frame is closest to 2x2 (4.0) not 6x4 (24.0).
    name, delta = matched_compute_delta(5.0, 0.40, _STATIC)
    assert name == "2x2"
    assert abs(delta - (0.40 - 0.34)) < 1e-9


def test_matched_compute_empty_static():
    assert matched_compute_delta(1.0, 0.5, []) == (None, None)


def test_matched_compute_skips_none_recall():
    rows = [{"name": "x", "mean_tiles": 1.0, "recall": None},
            {"name": "y", "mean_tiles": 2.0, "recall": 0.3}]
    name, delta = matched_compute_delta(1.5, 0.4, rows)
    assert name == "y"  # the None-recall row is skipped
    assert abs(delta - 0.1) < 1e-9
