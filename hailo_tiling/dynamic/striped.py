from __future__ import annotations

from .scheduler import TileScheduler
from ..types import CropRect, LockState

__all__ = ["StripedDenseScheduler"]


class StripedDenseScheduler:
    """Flat-load dense tiling that sweeps the whole frame at a low effective rate.

    The dense grid (default 7x6 @ 0.15 overlap, whole frame, multi-scale) is laid
    out row-major and swept a few cells per frame so the per-frame dense cost is
    flat (no discovery spike) while any given cell is refreshed only ~once per
    sweep — the "lower-fps full-frame dense" pass. On a 3840x2160 source the
    default grid yields ~629x472 px crops (scale ~1.0 — the model's native
    640x480 with no upscaling) that overlap their neighbours by ~0.15, so objects
    on a tile boundary still appear whole in an adjacent tile. Off-sweep
    detections are carried forward by DetectionPersistence (the research's
    skipped-tile carry-forward). Two sweep orders:

      - ``rolling`` (default): emit ``dense_per_frame`` consecutive cells in
        row-major order, advancing each frame and wrapping — a row-band of dense
        tiles that rolls down the frame. Full-frame refresh every
        ceil(n_cells / dense_per_frame) frames. With 7x6 (42 cells) and
        dense_per_frame=2 that is 21 frames (~2.9 Hz/cell at 60 fps).
      - ``interleaved``: K = round(fps / cadence_fps) interleaved stripes,
        stripe ``frame % K`` per frame (legacy).

    decide() emits the locked-target ROI tile (every TRACKING frame) FIRST, then
    the dense cells. While SEARCHING/LOST there is no ROI, so the rolling dense
    sweep alone performs a flat whole-frame search (it re-detects a target that
    reappears anywhere, unlike a local recovery grid).
    """

    def __init__(self, src_w: int, src_h: int, *,
                 dense_grid: tuple = (7, 6), fps: float = 60.0,
                 cadence_fps: float = 2.0, grid_overlap: float = 0.15,
                 stripe_mode: str = "rolling", dense_per_frame: int = 2,
                 **v1_kwargs):
        self.src_w = int(src_w)
        self.src_h = int(src_h)
        self.dense_grid = dense_grid
        self.stripe_mode = stripe_mode
        self.dense_per_frame = max(1, int(dense_per_frame))
        self.K = max(1, int(round(fps / cadence_fps)))
        self._v1 = TileScheduler(self.src_w, self.src_h,
                                 grid_overlap=grid_overlap, **v1_kwargs)
        gx, gy = dense_grid
        # Row-major (j outer, i inner) => crop index == logical cell j*gx + i.
        # SINGLE-SCALE ("s"): each native ~640x480 cell is already model-sized, so
        # there is no pyramid to build. Tagging dense tiles multi-scale ("m") makes
        # hailotileaggregator apply cross-scale suppression with no companion layer
        # and SILENTLY DROP their detections (verified: a native crop detects the
        # cars at conf~0.88, but the "m" dense path recorded 0). Keep them "s".
        self._dense = self._v1._grid(gx, gy, 0, 0,
                                     self.src_w, self.src_h, "s")
        self._stripes = [list(range(i, len(self._dense), self.K))
                         for i in range(self.K)]

    def stripe_indices(self, frame_idx: int) -> list[int]:
        """Dense-cell indices swept on `frame_idx` (== crop indices). These are
        also the cells DetectionPersistence refreshes (carry-forward elsewhere)."""
        n = len(self._dense)
        if self.stripe_mode == "rolling":
            start = (frame_idx * self.dense_per_frame) % n
            return [(start + k) % n for k in range(min(self.dense_per_frame, n))]
        return self._stripes[frame_idx % self.K]

    def decide(self, lock: LockState, frame_idx: int, meter) -> list[CropRect]:
        # ROI tile only while actively TRACKING; otherwise the rolling dense
        # sweep alone is the (flat, whole-frame) search.
        crops: list[CropRect] = []
        if lock.status == "TRACKING":
            # bbox-proportional ROI: the dynamic window always sits a bit larger
            # than the target bbox and grows/shrinks with it (past native for a
            # near target, zoom-in for a far one). ROI first so budget keeps it.
            crops.append(self._v1._roi_proportional(lock))
        crops += [self._dense[i] for i in self.stripe_indices(frame_idx)]
        budget = int(meter.available(frame_idx))
        if budget >= 0 and len(crops) > budget:
            crops = crops[:max(0, budget)]
        return crops
