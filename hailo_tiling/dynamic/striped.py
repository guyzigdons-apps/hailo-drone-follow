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

    def _target_crops(self, lock: LockState):
        """Return (crops, in_recovery). ROI when TRACKING; recovery grid when
        SEARCHING/LOST with a known track. Mirrors TileScheduler.decide minus
        the discovery grid (this class owns the dense pass)."""
        if lock.status in ("SEARCHING", "LOST") and lock.track_id is not None:
            gx, gy = self._v1.recovery_grid
            bx, by, bw, bh = lock.bbox_norm
            ecx = bx + bw / 2 + lock.last_velocity[0] * lock.frames_since_seen
            ecy = by + bh / 2 + lock.last_velocity[1] * lock.frames_since_seen
            span = self._v1.recovery_span
            half = span / 2
            x0_n = max(0.0, min(1.0 - span, ecx - half))
            y0_n = max(0.0, min(1.0 - span, ecy - half))
            crops = self._v1._grid(gx, gy, x0_n * self.src_w, y0_n * self.src_h,
                                   span * self.src_w, span * self.src_h, "s")
            return crops, True
        crops = []
        if lock.status == "TRACKING":
            crops.append(self._v1._roi(lock))
        return crops, False

    def decide(self, lock: LockState, frame_idx: int, meter) -> list[CropRect]:
        target_crops, in_recovery = self._target_crops(lock)
        if in_recovery:
            crops = target_crops          # recovery owns the frame
        else:
            stripe = [self._dense[i] for i in self.stripe_indices(frame_idx)]
            crops = target_crops + stripe  # ROI first, then dense stripe
        budget = int(meter.available(frame_idx))
        if budget >= 0 and len(crops) > budget:
            crops = crops[:max(0, budget)]
        return crops
