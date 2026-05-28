"""hailo_tiling — reusable dynamic-tiling library for Hailo inference pipelines.

See docs/superpowers/specs/2026-05-28-tiling-library-design.md.
"""
__version__ = "0.1.0.dev0"

from .budget import BudgetMeter
from .emitters import DiscoveryGridEmitter, RecoveryGridEmitter, TrackROIEmitter
from .modifiers import BudgetTrimModifier
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

__all__ = [
    "__version__",
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
]
