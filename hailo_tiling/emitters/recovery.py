# hailo_tiling/emitters/recovery.py
"""Search grid emitted when the locked target is SEARCHING or LOST."""
from __future__ import annotations

from ..types import CropRect, LockState, MODEL_ASPECT
from .discovery_grid import _grid_full as _full_frame_grid_unused  # noqa: F401


def _grid_region(src_w: int, src_h: int, gx: int, gy: int,
                 x0: float, y0: float, w: float, h: float, mode: str) -> list[CropRect]:
    """Build a gx×gy grid over the [x0, y0, w, h] src-pixel region.

    Equivalent to `dynamic_tiling.TileScheduler._grid(gx, gy, x0, y0, w, h, mode)`.
    Must produce byte-identical output.
    """
    out: list[CropRect] = []
    cw = w / gx
    ch = h / gy
    crop_w = max(cw, ch * MODEL_ASPECT)
    for j in range(gy):
        for i in range(gx):
            cx = x0 + (i + 0.5) * cw
            cy = y0 + (j + 0.5) * ch
            r = CropRect.from_center_width(cx, cy, int(round(crop_w)), mode=mode)
            out.append(r.clamp(src_w, src_h))
    return out


class RecoveryGridEmitter:
    """Emits a search grid around the predicted lost-target position.

    Fires only when `lock.status in {'SEARCHING', 'LOST'}` AND `lock.track_id is not None`.
    The grid is centered on the last-known bbox extrapolated by velocity * frames_since_seen
    (motion-predicted placement).
    """

    name = "recovery_grid"

    def __init__(self, grid: tuple[int, int] = (3, 3), span: float = 0.4, mode: str = "s"):
        self.gx, self.gy = grid
        self.span = span
        self.mode = mode

    def emit(self, src_w: int, src_h: int, lock: LockState,
             frame_idx: int, meter) -> list[CropRect]:
        if lock.status not in ("SEARCHING", "LOST"):
            return []
        if lock.track_id is None:
            return []
        bx, by, bw, bh = lock.bbox_norm
        ecx = bx + bw / 2 + lock.last_velocity[0] * lock.frames_since_seen
        ecy = by + bh / 2 + lock.last_velocity[1] * lock.frames_since_seen
        span = self.span
        half = span / 2
        x0_n = max(0.0, min(1.0 - span, ecx - half))
        y0_n = max(0.0, min(1.0 - span, ecy - half))
        return _grid_region(
            src_w, src_h, self.gx, self.gy,
            x0_n * src_w, y0_n * src_h,
            span * src_w, span * src_h,
            self.mode,
        )
