"""Per-frame dynamic-tiling controller for the live GStreamer pipeline.

Wraps the proven lab components:
  - TargetLock        (auto-locks the largest person, reuses prod ByteTracker)
  - TileScheduler     (discovery grid + ROI follow + recovery, budget-trimmed)
  - BudgetMeter       (sliding-window inference-rate cap)

`update(persons)` is called once per frame with the person detections from the
aggregator (normalized Det). It steps the lock, asks the scheduler for the next
crop set, charges the budget, and returns the crops as a `tiles-static` string
ready to push onto hailotilecropper_dynamic.
"""
from collections.abc import Sequence

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.dynamic.scheduler import TileScheduler
from hailo_tiling.types import Det
from tiling_lab.harness.target_lock import TargetLock

from tiling_lab.live.tiles_format import crops_to_tiles_static


class DynamicTilingController:
    def __init__(self, src_w: int, src_h: int, *, fps: float = 30.0,
                 budget_inf_per_s: float = 60.0, track_buffer: int = 90,
                 scheduler_kwargs: dict | None = None):
        self.src_w = int(src_w)
        self.src_h = int(src_h)
        self._sched = TileScheduler(self.src_w, self.src_h,
                                    **(scheduler_kwargs or {}))
        self._lock = TargetLock(track_buffer=track_buffer)
        self._meter = BudgetMeter(budget_inf_per_s=float(budget_inf_per_s),
                                  fps=float(fps))
        self._frame = 0
        self._total_tiles = 0

    def update(self, persons: Sequence[Det]) -> str:
        """Step one frame; return the tiles-static string for the next frame."""
        self._lock.step(list(persons), lock_if_unlocked=True)
        crops = self._sched.decide(self._lock.state, self._frame, self._meter)
        self._meter.charge(len(crops), self._frame)
        self._total_tiles += len(crops)
        self._frame += 1
        return crops_to_tiles_static(crops, self.src_w, self.src_h)

    @property
    def status(self) -> str:
        return self._lock.state.status

    @property
    def frame_count(self) -> int:
        return self._frame

    @property
    def mean_tiles_per_frame(self) -> float:
        return self._total_tiles / self._frame if self._frame else 0.0
