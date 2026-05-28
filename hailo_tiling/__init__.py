"""hailo_tiling — reusable dynamic-tiling library for Hailo inference pipelines.

See docs/superpowers/specs/2026-05-28-tiling-library-design.md.
"""
__version__ = "0.1.0.dev0"

# --- Plan 1 surface ---
from .budget import BudgetMeter
from .emitters import DiscoveryGridEmitter, RecoveryGridEmitter, TrackROIEmitter
from .modifiers import (
    AdaptiveSliceSizingModifier,
    AltitudeZoomModifier,
    BudgetTrimModifier,
)
from .scheduler import TileEmitter, TileModifier, TileScheduler
from .types import (
    MODEL_ASPECT,
    MODEL_H,
    MODEL_W,
    CropRect,
    Det,
    LockState,
    ScheduledTile,
    TargetState,
)

# --- Plan 2 surface ---
from .telemetry import (
    MavsdkTelemetry,
    NULL_SNAPSHOT,
    RecordedTelemetry,
    StaticTelemetry,
    TelemetryProvider,
    TelemetrySnapshot,
)
from .backends import (
    CachingBackend,
    HefBackend,
    InferenceBackend,
    MockBackend,
    ReplayBackend,
)
from .cache import CacheMissError, SqliteCacheStore
from .aggregator import (
    Aggregator,
    BoundaryStripFilter,
    DetectionMemory,
    NoOpMemory,
    map_to_source,
    nms,
)

__all__ = [
    "__version__",
    # Plan 1
    "BudgetMeter",
    "CropRect",
    "Det",
    "LockState",
    "TargetState",
    "ScheduledTile",
    "MODEL_W",
    "MODEL_H",
    "MODEL_ASPECT",
    "TileScheduler",
    "TileEmitter",
    "TileModifier",
    "DiscoveryGridEmitter",
    "TrackROIEmitter",
    "RecoveryGridEmitter",
    "BudgetTrimModifier",
    # Plan 2 — telemetry
    "TelemetryProvider",
    "TelemetrySnapshot",
    "NULL_SNAPSHOT",
    "StaticTelemetry",
    "RecordedTelemetry",
    "MavsdkTelemetry",
    # Plan 2 — modifiers
    "AltitudeZoomModifier",
    "AdaptiveSliceSizingModifier",
    # Plan 2 — backends
    "InferenceBackend",
    "MockBackend",
    "HefBackend",
    # Plan 4 — cache + decorator/replay backends
    "CachingBackend",
    "ReplayBackend",
    "SqliteCacheStore",
    "CacheMissError",
    # Plan 2 — aggregator
    "Aggregator",
    "BoundaryStripFilter",
    "DetectionMemory",
    "NoOpMemory",
    "map_to_source",
    "nms",
]
