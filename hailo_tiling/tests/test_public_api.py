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


def test_plan2_top_level_imports():
    import hailo_tiling as ht
    # Telemetry
    assert ht.TelemetryProvider is not None
    assert ht.TelemetrySnapshot is not None
    assert ht.NULL_SNAPSHOT is not None
    assert ht.StaticTelemetry is not None
    assert ht.RecordedTelemetry is not None
    assert ht.MavsdkTelemetry is not None
    # New modifiers
    assert ht.AltitudeZoomModifier is not None
    assert ht.AdaptiveSliceSizingModifier is not None
    # Backends
    assert ht.InferenceBackend is not None
    assert ht.MockBackend is not None
    assert ht.HefBackend is not None
    # Aggregator
    assert ht.Aggregator is not None
    assert ht.BoundaryStripFilter is not None
    assert ht.DetectionMemory is not None
    assert ht.NoOpMemory is not None
    assert ht.nms is not None
    assert ht.map_to_source is not None
