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
from hailo_tiling.dynamic.acquisition import pyramid_crops
from hailo_tiling.dynamic.scheduler import TileScheduler
from hailo_tiling.dynamic.striped import StripedDenseScheduler
from hailo_tiling.dynamic.persistence import DetectionPersistence
from hailo_tiling.dynamic.nms import nms_merge
from hailo_tiling.types import Det
from tiling_lab.harness.target_lock import TargetLock, SeedTracker

from tiling_lab.live.tiles_format import crops_to_tiles_static


class DynamicTilingController:
    def __init__(self, src_w: int, src_h: int, *, fps: float = 30.0,
                 budget_inf_per_s: float = 600.0, track_buffer: int = 90,
                 scheduler_kwargs: dict | None = None,
                 striped: bool = False, persist: bool = False,
                 dense_grid: tuple = (8, 6), cadence_fps: float = 2.0,
                 stripe_mode: str = "rolling", dense_per_frame: int = 2,
                 acquire_mode: str = "largest", center_frac: float = 0.15,
                 central_frames: int = 10, grid_overlap: float = 0.0):
        self.src_w = int(src_w)
        self.src_h = int(src_h)
        self.striped = striped
        if striped:
            self._sched = StripedDenseScheduler(
                self.src_w, self.src_h, dense_grid=dense_grid, fps=float(fps),
                cadence_fps=cadence_fps, stripe_mode=stripe_mode,
                dense_per_frame=dense_per_frame, grid_overlap=grid_overlap,
                **(scheduler_kwargs or {}))
        else:
            self._sched = TileScheduler(self.src_w, self.src_h,
                                        grid_overlap=grid_overlap,
                                        **(scheduler_kwargs or {}))
        # "seed" acquisition follows exactly the operator-seeded target via a
        # direct nearest-detection associator (no ByteTracker "largest"/IoU path),
        # so the lock never jumps to a different object and only ever reports the
        # REAL detected bbox. Other modes keep the ByteTracker-backed TargetLock.
        self._seed_mode = (acquire_mode == "seed")
        if self._seed_mode:
            self._lock = SeedTracker(track_buffer=track_buffer)
        else:
            self._lock = TargetLock(track_buffer=track_buffer,
                                    acquire_mode=acquire_mode,
                                    center_frac=center_frac,
                                    central_frames=central_frames)
        self._meter = BudgetMeter(budget_inf_per_s=float(budget_inf_per_s),
                                  fps=float(fps))
        self._persist = DetectionPersistence(dense_grid) if persist else None
        self._frame = 0
        self._total_tiles = 0
        self._select_mode = "detection"
        self._click: tuple | None = None

    def update(self, persons: Sequence[Det]) -> str:
        """Step one frame; return the tiles-static string for the next frame."""
        if self._seed_mode:
            raise RuntimeError("seed acquire_mode is showcase-only; "
                               "use step_showcase(), not update()")
        self._lock.step(list(persons), lock_if_unlocked=True)
        crops = self._sched.decide(self._lock.state, self._frame, self._meter)
        self._meter.charge(len(crops), self._frame)
        self._total_tiles += len(crops)
        self._frame += 1
        return crops_to_tiles_static(crops, self.src_w, self.src_h)

    def seed(self, bbox_norm: tuple, mode: str = "detection") -> None:
        """Seed the target. mode='detection' snaps to an existing nearby
        detection (tight gate); mode='click' runs a zoom pyramid at the click
        until a detection is found (works before dense has detected it)."""
        self._select_mode = mode
        self._click = (bbox_norm[0] + bbox_norm[2] / 2.0,
                       bbox_norm[1] + bbox_norm[3] / 2.0)
        self._lock.seed(bbox_norm)

    def step_showcase(self, target_dets, all_dets):
        """Step one frame for the showcase runner.

        target_dets : Sequence[Det]   target-class detections (for the lock)
        all_dets    : Sequence[dict]  ALL detections, visualizer-schema dicts
        Returns (tiles_static_str, record_dets). With persist=True, record_dets
        is the live SOT detection plus the persisted dense union (SOT box
        de-duplicated); with persist disabled, only the live target record.
        """
        if not self.striped:
            raise RuntimeError("step_showcase requires striped=True")
        # Step the lock and decide whether a REAL target detection exists this
        # frame. In seed mode the associator returns that directly; in the
        # ByteTracker path it's TRACKING with a non-empty bbox (the filtered
        # track). Either way `detected` gates the drawn box, so we never emit
        # the synthetic seed position as a "target".
        if self._seed_mode:
            detected = self._lock.step(list(target_dets))
        else:
            self._lock.step(list(target_dets), lock_if_unlocked=True)
            detected = (self._lock.state.status == "TRACKING"
                        and self._lock.state.bbox_norm[2] > 0)
        st = self._lock.state
        crops = self._sched.decide(st, self._frame, self._meter)
        if (self._seed_mode and self._select_mode == "click"
                and self._click is not None and not self._lock.bound):
            dense = [c for c in crops if c.mode == "m"]
            crops = pyramid_crops(self._click[0], self._click[1],
                                  self.src_w, self.src_h) + dense
        self._meter.charge(len(crops), self._frame)
        self._total_tiles += len(crops)

        records: list[dict] = []
        target_bbox = None
        if detected and st.bbox_norm[2] > 0:
            target_bbox = tuple(st.bbox_norm)
            conf = max((d.score for d in target_dets), default=1.0)
            records.append({"label": "target", "confidence": float(conf),
                            "bbox": list(target_bbox)})

        if self._persist is not None:
            self._persist.update(self._sched.stripe_indices(self._frame),
                                 list(all_dets))
            self._persist.tick()
            union = list(records) + self._persist.published()   # target + carried
            records = nms_merge(union, iou_thresh=0.3)

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
