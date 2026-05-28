"""Public re-exports — fail fast if a class moves and we forget to update __init__."""
from __future__ import annotations


def test_top_level_imports():
    import hailo_tiling as ht
    # Types
    assert ht.CropRect is not None
    assert ht.Det is not None
    assert ht.LockState is not None
    # Scheduler & protocols
    assert ht.TileScheduler is not None
    assert ht.TileEmitter is not None
    assert ht.TileModifier is not None
    # Emitters
    assert ht.DiscoveryGridEmitter is not None
    assert ht.TrackROIEmitter is not None
    assert ht.RecoveryGridEmitter is not None
    # Modifiers
    assert ht.BudgetTrimModifier is not None
    # Budget
    assert ht.BudgetMeter is not None
