# dynamic_tiling/types.py
"""Compatibility shim: types live in hailo_tiling.types.

This module re-exports the public API so legacy `from dynamic_tiling.types import …`
imports keep working during the migration. Remove this shim in Plan 8
(drone-follow migration) once all callers move to hailo_tiling.
"""
from hailo_tiling.types import (  # noqa: F401
    MODEL_W,
    MODEL_H,
    MODEL_ASPECT,
    CropRect,
    Det,
    LockState,
    TargetState,
    ScheduledTile,
)
