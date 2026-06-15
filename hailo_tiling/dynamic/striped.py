from __future__ import annotations

from .scheduler import TileScheduler
from ..types import CropRect, LockState

__all__ = ["StripedDenseScheduler"]


class StripedDenseScheduler:
    """Round-robin striped dense tiling.

    The dense grid (default 8x6, whole frame, multi-scale) is partitioned into
    K = round(fps / cadence_fps) interleaved stripes. Each frame emits exactly
    one stripe of dense tiles plus the wrapped v1 ROI/recovery tile(s), so the
    per-frame inference count is flat (no periodic discovery spike). A full
    dense refresh completes every K frames (~cadence_fps Hz).
    """

    def __init__(self, src_w: int, src_h: int, *,
                 dense_grid: tuple = (8, 6), fps: float = 60.0,
                 cadence_fps: float = 2.0, grid_overlap: float = 0.0,
                 **v1_kwargs):
        self.src_w = int(src_w)
        self.src_h = int(src_h)
        self.dense_grid = dense_grid
        self.K = max(1, int(round(fps / cadence_fps)))
        self._v1 = TileScheduler(self.src_w, self.src_h,
                                 grid_overlap=grid_overlap, **v1_kwargs)
        gx, gy = dense_grid
        # Row-major (j outer, i inner) => crop index == logical cell j*gx + i.
        self._dense = self._v1._grid(gx, gy, 0, 0,
                                     self.src_w, self.src_h, "m")
        self._stripes = [list(range(i, len(self._dense), self.K))
                         for i in range(self.K)]

    def stripe_indices(self, frame_idx: int) -> list[int]:
        """Logical dense-cell indices run on `frame_idx` (== crop indices)."""
        return self._stripes[frame_idx % self.K]
