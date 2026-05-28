"""Verify hailo_tiling.types and dynamic_tiling.types are the same objects."""
from __future__ import annotations


def test_dynamic_tiling_types_reexport_hailo_tiling():
    """The shim must re-export the same classes by identity."""
    from hailo_tiling.types import CropRect as HtCropRect
    from hailo_tiling.types import Det as HtDet
    from hailo_tiling.types import LockState as HtLockState
    from hailo_tiling.types import TargetState as HtTargetState
    from hailo_tiling.types import ScheduledTile as HtScheduledTile

    from dynamic_tiling.types import CropRect as DtCropRect
    from dynamic_tiling.types import Det as DtDet
    from dynamic_tiling.types import LockState as DtLockState
    from dynamic_tiling.types import TargetState as DtTargetState
    from dynamic_tiling.types import ScheduledTile as DtScheduledTile

    assert HtCropRect is DtCropRect
    assert HtDet is DtDet
    assert HtLockState is DtLockState
    assert HtTargetState is DtTargetState
    assert HtScheduledTile is DtScheduledTile


def test_model_constants():
    from hailo_tiling.types import MODEL_W, MODEL_H, MODEL_ASPECT
    assert MODEL_W == 640
    assert MODEL_H == 480
    assert MODEL_ASPECT == 640 / 480


def test_croprect_clamp_keeps_w_h():
    from hailo_tiling.types import CropRect
    r = CropRect(x=-10, y=-5, w=100, h=75, mode="s").clamp(3840, 2160)
    assert r.x == 0 and r.y == 0
    assert r.w == 100 and r.h == 75
