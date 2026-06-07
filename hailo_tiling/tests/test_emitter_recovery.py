# hailo_tiling/tests/test_emitter_recovery.py
"""RecoveryGridEmitter — search grid around the predicted lost-target position."""
from __future__ import annotations

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.emitters.recovery import RecoveryGridEmitter
from hailo_tiling.types import LockState


def _meter():
    return BudgetMeter(budget_inf_per_s=1000.0, fps=30.0)


def test_emits_3x3_when_searching(src_dims, searching_lock):
    src_w, src_h = src_dims
    e = RecoveryGridEmitter(grid=(3, 3), span=0.4)
    out = e.emit(src_w, src_h, searching_lock, frame_idx=0, meter=_meter())
    assert len(out) == 9
    assert all(c.mode == "s" for c in out)


def test_emits_empty_when_tracking(src_dims, tracking_lock):
    e = RecoveryGridEmitter()
    out = e.emit(src_dims[0], src_dims[1], tracking_lock, 0, _meter())
    assert out == []


def test_emits_empty_when_lost_without_track_id(src_dims, lost_lock):
    # lost_lock has track_id=None — recovery should not fire.
    e = RecoveryGridEmitter()
    out = e.emit(src_dims[0], src_dims[1], lost_lock, 0, _meter())
    assert out == []


def test_matches_legacy_recovery(src_dims, searching_lock):
    """Exact output equality with the legacy recovery branch."""
    from hailo_tiling.dynamic.scheduler import TileScheduler as LegacyScheduler
    from hailo_tiling.types import MODEL_ASPECT  # noqa: F401  (matches legacy import order)

    src_w, src_h = src_dims
    legacy = LegacyScheduler(src_w, src_h, recovery_grid=(3, 3), recovery_span=0.4)

    bx, by, bw, bh = searching_lock.bbox_norm
    ecx = bx + bw / 2 + searching_lock.last_velocity[0] * searching_lock.frames_since_seen
    ecy = by + bh / 2 + searching_lock.last_velocity[1] * searching_lock.frames_since_seen
    span = 0.4
    half = span / 2
    x0_n = max(0.0, min(1.0 - span, ecx - half))
    y0_n = max(0.0, min(1.0 - span, ecy - half))
    expected = legacy._grid(
        3, 3,
        x0_n * src_w, y0_n * src_h,
        span * src_w, span * src_h,
        "s",
    )

    e = RecoveryGridEmitter(grid=(3, 3), span=0.4)
    actual = e.emit(src_w, src_h, searching_lock, 0, _meter())

    assert actual == expected
