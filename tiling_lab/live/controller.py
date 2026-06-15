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
from hailo_tiling.dynamic.striped import StripedDenseScheduler
from hailo_tiling.dynamic.persistence import DetectionPersistence
from hailo_tiling.types import Det
from tiling_lab.harness.target_lock import TargetLock

from tiling_lab.live.tiles_format import crops_to_tiles_static


class DynamicTilingController:
    def __init__(self, src_w: int, src_h: int, *, fps: float = 30.0,
                 budget_inf_per_s: float = 60.0, track_buffer: int = 90,
                 scheduler_kwargs: dict | None = None,
                 striped: bool = False, persist: bool = False,
                 dense_grid: tuple = (8, 6), cadence_fps: float = 2.0):
        self.src_w = int(src_w)
        self.src_h = int(src_h)
        self.striped = striped
        if striped:
            self._sched = StripedDenseScheduler(
                self.src_w, self.src_h, dense_grid=dense_grid, fps=float(fps),
                cadence_fps=cadence_fps, **(scheduler_kwargs or {}))
        else:
            self._sched = TileScheduler(self.src_w, self.src_h,
                                        **(scheduler_kwargs or {}))
        self._lock = TargetLock(track_buffer=track_buffer)
        self._meter = BudgetMeter(budget_inf_per_s=float(budget_inf_per_s),
                                  fps=float(fps))
        self._persist = DetectionPersistence(dense_grid) if persist else None
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

    @staticmethod
    def _iou(a, b) -> float:
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        ix1 = max(ax, bx); iy1 = max(ay, by)
        ix2 = min(ax + aw, bx + bw); iy2 = min(ay + ah, by + bh)
        iw = max(0.0, ix2 - ix1); ih = max(0.0, iy2 - iy1)
        inter = iw * ih
        ua = aw * ah + bw * bh - inter
        return inter / ua if ua > 0 else 0.0

    def step_showcase(self, target_dets, all_dets):
        """Step one frame for the showcase runner.

        target_dets : Sequence[Det]   target-class detections (for the lock)
        all_dets    : Sequence[dict]  ALL detections, visualizer-schema dicts
        Returns (tiles_static_str, record_dets) where record_dets is the live
        SOT detection plus the persisted dense union (SOT box de-duplicated).
        """
        if not self.striped:
            raise RuntimeError("step_showcase requires striped=True")
        self._lock.step(list(target_dets), lock_if_unlocked=True)
        crops = self._sched.decide(self._lock.state, self._frame, self._meter)
        self._meter.charge(len(crops), self._frame)
        self._total_tiles += len(crops)

        records: list[dict] = []
        target_bbox = None
        st = self._lock.state
        if st.status == "TRACKING" and st.bbox_norm[2] > 0:
            target_bbox = tuple(st.bbox_norm)
            conf = max((d.score for d in target_dets), default=1.0)
            records.append({"label": "target", "confidence": float(conf),
                            "bbox": list(target_bbox)})

        if self._persist is not None:
            self._persist.update(self._sched.stripe_indices(self._frame),
                                 list(all_dets))
            for d in self._persist.published():
                if target_bbox is not None and \
                        self._iou(target_bbox, tuple(d["bbox"])) > 0.5:
                    continue
                records.append(d)

        tiles = crops_to_tiles_static(crops, self.src_w, self.src_h)
        self._frame += 1
        return tiles, records

    @property
    def status(self) -> str:
        return self._lock.state.status

    @property
    def frame_count(self) -> int:
        return self._frame

    @property
    def mean_tiles_per_frame(self) -> float:
        return self._total_tiles / self._frame if self._frame else 0.0
