# hailo_tiling/tests/test_emitter_discovery_grid.py
"""DiscoveryGridEmitter — full-frame N×M grid on a cadence."""
from __future__ import annotations

import pytest

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.emitters import DiscoveryGridEmitter
from hailo_tiling.types import CropRect, LockState


def _meter():
    return BudgetMeter(budget_inf_per_s=1000.0, fps=30.0)


def test_emits_3x2_on_cadence(src_dims):
    src_w, src_h = src_dims
    e = DiscoveryGridEmitter(grid=(3, 2), period=15, mode="m")
    out = e.emit(src_w, src_h, LockState(), frame_idx=0, meter=_meter())
    assert len(out) == 6
    assert all(isinstance(c, CropRect) for c in out)
    assert all(c.mode == "m" for c in out)


def test_emits_empty_off_cadence(src_dims):
    src_w, src_h = src_dims
    e = DiscoveryGridEmitter(grid=(3, 2), period=15, mode="m")
    out = e.emit(src_w, src_h, LockState(), frame_idx=1, meter=_meter())
    assert out == []


def test_matches_legacy_grid(src_dims):
    """Exact output equality with the legacy `_grid` implementation."""
    from dynamic_tiling.scheduler import TileScheduler as LegacyScheduler
    src_w, src_h = src_dims
    legacy = LegacyScheduler(src_w, src_h, discovery_grid=(3, 2))
    expected = legacy._grid(3, 2, 0, 0, src_w, src_h, "m")

    e = DiscoveryGridEmitter(grid=(3, 2), period=15, mode="m")
    actual = e.emit(src_w, src_h, LockState(), frame_idx=0, meter=_meter())

    assert actual == expected
