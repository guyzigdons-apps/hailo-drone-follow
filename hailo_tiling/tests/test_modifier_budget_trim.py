"""BudgetTrimModifier — trims the tile list to fit the per-frame budget."""
from __future__ import annotations

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.modifiers import BudgetTrimModifier
from hailo_tiling.types import CropRect, LockState


def _tiles(n: int) -> list[CropRect]:
    return [CropRect(x=i, y=0, w=640, h=480, mode="s") for i in range(n)]


def test_no_trim_when_under_budget(src_dims):
    src_w, src_h = src_dims
    meter = BudgetMeter(budget_inf_per_s=1000.0, fps=30.0)  # 33 tiles/frame share
    m = BudgetTrimModifier()
    tiles = _tiles(5)
    out = m.modify(tiles, src_w, src_h, LockState(), 0, meter)
    assert out == tiles


def test_trims_from_tail(src_dims):
    src_w, src_h = src_dims
    meter = BudgetMeter(budget_inf_per_s=90.0, fps=30.0)  # 3 tiles/frame share
    m = BudgetTrimModifier()
    tiles = _tiles(10)
    out = m.modify(tiles, src_w, src_h, LockState(), 0, meter)
    assert out == tiles[:3]


def test_budget_zero_returns_empty(src_dims):
    src_w, src_h = src_dims
    meter = BudgetMeter(budget_inf_per_s=0.0, fps=30.0)
    m = BudgetTrimModifier()
    out = m.modify(_tiles(5), src_w, src_h, LockState(), 0, meter)
    assert out == []


def test_negative_budget_disabled_returns_all(src_dims):
    """A meter that returns a negative available count means 'unlimited'."""

    class UnlimitedMeter:
        def available(self, frame_idx):
            return -1.0

    src_w, src_h = src_dims
    m = BudgetTrimModifier()
    tiles = _tiles(5)
    out = m.modify(tiles, src_w, src_h, LockState(), 0, UnlimitedMeter())
    assert out == tiles
