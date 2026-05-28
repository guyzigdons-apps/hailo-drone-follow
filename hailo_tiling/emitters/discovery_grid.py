# hailo_tiling/emitters/discovery_grid.py
"""Full-frame N×M discovery grid emitted on a fixed-period cadence."""
from __future__ import annotations

from ..types import CropRect, LockState, MODEL_ASPECT


def _grid_full(src_w: int, src_h: int, gx: int, gy: int, mode: str) -> list[CropRect]:
    """Build the N×M grid over the full frame.

    Each tile is 4:3 (model aspect). Tile width is grown to the larger of the
    cell width and the aspect-scaled cell height so cells are fully covered
    (tiles may overlap) rather than leaving vertical/horizontal gaps.

    This is the inlined equivalent of `dynamic_tiling.TileScheduler._grid(
    gx, gy, 0, 0, src_w, src_h, mode)` and must produce byte-identical output.
    """
    out: list[CropRect] = []
    cw = src_w / gx
    ch = src_h / gy
    crop_w = max(cw, ch * MODEL_ASPECT)
    for j in range(gy):
        for i in range(gx):
            cx = (i + 0.5) * cw
            cy = (j + 0.5) * ch
            r = CropRect.from_center_width(cx, cy, int(round(crop_w)), mode=mode)
            out.append(r.clamp(src_w, src_h))
    return out


class DiscoveryGridEmitter:
    """Emits an N×M grid covering the full frame every `period` frames."""

    name = "discovery_grid"

    def __init__(self, grid: tuple[int, int] = (3, 2), period: int = 15, mode: str = "m"):
        self.gx, self.gy = grid
        self.period = period
        self.mode = mode

    def emit(self, src_w, src_h, lock, frame_idx, meter) -> list[CropRect]:
        if frame_idx % self.period != 0:
            return []
        return _grid_full(src_w, src_h, self.gx, self.gy, self.mode)
