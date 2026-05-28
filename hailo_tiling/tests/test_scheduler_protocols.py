"""Validate the Protocol shapes and TileScheduler composition contract."""
from __future__ import annotations

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.scheduler import (
    TileEmitter,
    TileModifier,
    TileScheduler,
)
from hailo_tiling.types import CropRect, LockState


class _DummyEmitter:
    """Test double: emits a fixed list of CropRects."""
    name = "dummy_emitter"

    def __init__(self, crops):
        self._crops = crops

    def emit(self, src_w, src_h, lock, frame_idx, meter, telemetry=None):
        return list(self._crops)


class _DummyModifier:
    """Test double: tags each crop's mode with a fixed string."""
    name = "dummy_modifier"

    def __init__(self, tag):
        self._tag = tag

    def modify(self, tiles, src_w, src_h, lock, frame_idx, meter, telemetry=None):
        return [CropRect(x=t.x, y=t.y, w=t.w, h=t.h, mode=self._tag) for t in tiles]


def test_protocol_runtime_compat():
    """Dummy classes structurally satisfy the Protocols."""
    e: TileEmitter = _DummyEmitter([])
    m: TileModifier = _DummyModifier("t")
    assert e.name == "dummy_emitter"
    assert m.name == "dummy_modifier"


def test_scheduler_runs_emitters_then_modifiers_in_order(src_dims, tracking_lock):
    src_w, src_h = src_dims
    meter = BudgetMeter(budget_inf_per_s=300.0, fps=30.0)
    a = CropRect(x=0, y=0, w=640, h=480, mode="s")
    b = CropRect(x=10, y=10, w=640, h=480, mode="s")
    scheduler = TileScheduler(
        emitters=[_DummyEmitter([a]), _DummyEmitter([b])],
        modifiers=[_DummyModifier("X")],
    )
    out = scheduler.decide(src_w, src_h, tracking_lock, frame_idx=0, meter=meter)
    # Emitters concatenated in order, then modifier rewrote each mode.
    assert [c.mode for c in out] == ["X", "X"]
    assert (out[0].x, out[0].y) == (0, 0)
    assert (out[1].x, out[1].y) == (10, 10)


def test_scheduler_empty_emitters_returns_empty():
    meter = BudgetMeter(budget_inf_per_s=300.0, fps=30.0)
    scheduler = TileScheduler(emitters=[], modifiers=[])
    out = scheduler.decide(3840, 2160, LockState(), 0, meter)
    assert out == []
