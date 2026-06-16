from __future__ import annotations

from typing import Sequence, TYPE_CHECKING

import numpy as np

from drone_follow.pipeline_adapter.tracker_factory import create_tracker

from hailo_tiling.types import Det, LockState, TargetState

_SCALE = 1000.0
_REACQ_IOU = 0.3  # min IoU vs last-known bbox to (re)acquire a track


def dets_to_array(person_dets: Sequence[Det]) -> np.ndarray:
    """Build the (N,5) [xmin,ymin,xmax,ymax,score] SCALE=1000 array ByteTracker
    expects (mirrors hailo_drone_detection_manager.py)."""
    arr = np.empty((len(person_dets), 5), dtype=np.float32)
    for i, d in enumerate(person_dets):
        arr[i, 0] = d.x * _SCALE
        arr[i, 1] = d.y * _SCALE
        arr[i, 2] = (d.x + d.w) * _SCALE
        arr[i, 3] = (d.y + d.h) * _SCALE
        arr[i, 4] = d.score
    return arr


def _iou_tlwh(a: tuple, b: tuple) -> float:
    ax1, ay1 = a[0], a[1]; ax2, ay2 = a[0] + a[2], a[1] + a[3]
    bx1, by1 = b[0], b[1]; bx2, by2 = b[0] + b[2], b[1] + b[3]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = a[2] * a[3] + b[2] * b[3] - inter
    return inter / ua if ua > 0 else 0.0


class TargetLock:
    """Follows one target across frames, wrapping the pipeline ByteTracker.

    ``track_id`` is a *stable* lock identity — it is set once when the target is
    first acquired and never changes, even if ByteTracker internally assigns the
    re-appearing person a new track id.  ``_bt_track_id`` is the *current*
    ByteTracker track id, which can change silently via IoU re-acquisition.

    Single-target offline scope — there is intentionally no give-up timeout;
    persistent re-id is the real pipeline's job.
    """

    def __init__(self, track_buffer: int = 90, reacq_motion: str = "frozen",
                 reacq_radius_growth: float = 0.0,
                 acquire_mode: str = "largest", center_frac: float = 0.2,
                 central_frames: int = 5, seed_timeout: int = 90,
                 **tracker_kwargs):
        self._tracker = create_tracker("byte", track_buffer=track_buffer,
                                       **tracker_kwargs)
        self.track_buffer = track_buffer
        # Public stable identity: set once on first lock, never changed.
        self.track_id: int | None = None
        # Internal: current ByteTracker track id (may diverge on re-acquisition).
        self._bt_track_id: int | None = None
        self.state = LockState()
        # Debug: the tracker output of the most recent step() (for replay dumps).
        self.last_tracks: list = []
        self.reacq_motion = reacq_motion
        self.reacq_radius_growth = reacq_radius_growth
        self._anchor: tuple | None = None   # (x, y, w, h) normalized, motion-updated
        # Acquisition policy. "largest": lock the biggest activated track (legacy).
        # "central": lock the track whose bbox centre sits within the central
        # region (|c-0.5| <= center_frac on both axes) and stays the most-central
        # candidate for `central_frames` consecutive frames (debounce). Lets the
        # operator pick the SOT by re-centring it in frame for a moment.
        self.acquire_mode = acquire_mode
        self.center_frac = center_frac
        self.central_frames = central_frames
        self.seed_timeout = seed_timeout
        self.acq_assoc_radius = 0.12   # max centre drift to count as same candidate
        self._acq_count = 0
        self._acq_misses = 0
        self._acq_cand_bbox: tuple | None = None    # running raw-det candidate bbox
        self._acq_cand_center: tuple | None = None  # running raw-det candidate centre
        # Pending seed: a bbox we want to lock onto. Set either by central
        # auto-acquisition or externally via seed(). While pending, the lock
        # holds an ROI on it (TRACKING state) so the dense tile samples it until
        # a real activated track appears there and is adopted.
        self._pending_seed: tuple | None = None
        self._seed_age = 0

    def seed(self, bbox_norm: tuple) -> None:
        """Externally seed the target by bbox (x, y, w, h) normalized — manual
        target selection / re-acquisition. Resets any existing binding so the
        seed takes over as a FRESH lock (e.g. re-acquiring a reappeared target at
        a new location). Cleared once a track is bound or it times out."""
        self._pending_seed = tuple(bbox_norm)
        self._seed_age = 0
        self._bt_track_id = None
        self.track_id = None
        self._anchor = None
        self._acq_count = 0
        self._acq_misses = 0
        self._acq_cand_bbox = None
        self._acq_cand_center = None

    def _most_central_det(self, dets: Sequence[Det]):
        """Raw detection within the central region nearest (0.5, 0.5); None if
        none is central. Used for auto-acquisition (no track id needed)."""
        lo, hi = 0.5 - self.center_frac, 0.5 + self.center_frac
        best, best_d = None, None
        for d in dets:
            cx, cy = d.x + d.w / 2.0, d.y + d.h / 2.0
            if not (lo <= cx <= hi and lo <= cy <= hi):
                continue
            dist = (cx - 0.5) ** 2 + (cy - 0.5) ** 2
            if best_d is None or dist < best_d:
                best_d, best = dist, d
        return best

    def _raw_central_acquire(self, dets: Sequence[Det]) -> None:
        """Accumulate central raw-detection observations of the same target
        (associated frame-to-frame by IoU), tolerating sampling gaps. On reaching
        central_frames, set a pending seed at that location."""
        cand = self._most_central_det(dets)
        if cand is not None:
            cb = (cand.x, cand.y, cand.w, cand.h)
            ccx, ccy = cand.x + cand.w / 2.0, cand.y + cand.h / 2.0
            # Associate by CENTRE DISTANCE, not IoU: targets here are tiny
            # (a few px wide) and wobble between sparse samples, so their boxes
            # rarely overlap (IoU~0) even when it's plainly the same object.
            if self._acq_cand_center is not None:
                pcx, pcy = self._acq_cand_center
                same = ((ccx - pcx) ** 2 + (ccy - pcy) ** 2) ** 0.5 < self.acq_assoc_radius
                self._acq_count = self._acq_count + 1 if same else 1
            else:
                self._acq_count = 1
            self._acq_cand_bbox = cb
            self._acq_cand_center = (ccx, ccy)
            self._acq_misses = 0
            if self._acq_count >= self.central_frames:
                self._pending_seed = cb
                self._seed_age = 0
                self._acq_count = 0
                self._acq_cand_bbox = None
                self._acq_cand_center = None
        else:
            self._acq_misses += 1
            if self._acq_misses > self.track_buffer:
                self._acq_cand_bbox = None
                self._acq_cand_center = None
                self._acq_count = 0
                self._acq_misses = 0

    @property
    def reacq_anchor(self) -> tuple | None:
        return self._anchor if self._anchor is not None else \
            (tuple(self.state.bbox_norm) if self.state.bbox_norm[2] > 0 else None)

    def _set_track(self, bt_track_id: int) -> None:
        """Lock onto a ByteTracker track.  Stable track_id is set only once."""
        self._bt_track_id = bt_track_id
        if self.track_id is None:
            self.track_id = bt_track_id

    def lock_from_gt(self, gt_bbox_norm: tuple, tracks) -> None:
        """Pick the activated track best matching a GT bbox (x,y,w,h)."""
        best, best_iou = None, 0.0
        for t in tracks:
            if not t.is_activated or not t.filtered_tlwh:
                continue
            i = _iou_tlwh(gt_bbox_norm, t.filtered_tlwh)
            if i > best_iou:
                best, best_iou = t, i
        if best is not None and best_iou > _REACQ_IOU:
            self._set_track(best.track_id)

    def adopt_overlapping(self, det) -> bool:
        """Re-point the lock at the activated track best overlapping `det`
        (ReID-confirmed recovery). Returns True if adopted."""
        best_iou, best = 0.1, None
        for t in self.last_tracks:
            if t.is_activated and t.filtered_tlwh:
                tl = t.filtered_tlwh
                iou = _iou_tlwh((det.x, det.y, det.w, det.h), tl)
                if iou > best_iou:
                    best_iou, best = iou, t
        if best is None:
            return False
        self._bt_track_id = best.track_id
        s = self.state
        s.bbox_norm = tuple(best.filtered_tlwh)
        s.status = "TRACKING"
        s.frames_since_seen = 0
        self._anchor = tuple(best.filtered_tlwh)
        return True

    def step(self, person_dets: Sequence[Det], *, lock_if_unlocked: bool = False,
             gt_bbox_norm: tuple | None = None) -> LockState:
        tracks = self._tracker.update(dets_to_array(person_dets))
        self.last_tracks = tracks

        if self._bt_track_id is None:
            if self._pending_seed is not None:
                # A seed is pending (from central auto-acquire or seed()): bind
                # to an activated track overlapping it as soon as one appears.
                self.lock_from_gt(self._pending_seed, tracks)
                if self._bt_track_id is not None:
                    self._pending_seed = None
            elif gt_bbox_norm is not None:
                self.lock_from_gt(gt_bbox_norm, tracks)
            elif lock_if_unlocked and self.acquire_mode == "central":
                # Auto-acquire from raw central DETECTIONS (not activated tracks):
                # a sparsely-sampled target never forms an activated track before
                # locking, so we seed on the detection and let the ROI densify it.
                self._raw_central_acquire(person_dets)
            elif lock_if_unlocked:
                acts = [t for t in tracks if t.is_activated and t.filtered_tlwh]
                if acts:
                    big = max(acts, key=lambda t: t.filtered_tlwh[2] * t.filtered_tlwh[3])
                    self._set_track(big.track_id)

        cur = next((t for t in tracks
                    if t.track_id == self._bt_track_id and t.is_activated
                    and t.filtered_tlwh), None)

        # IoU-based re-acquisition: if we're searching (lock set, but target
        # not found in current tracks) and a new activated track overlaps our
        # motion-updated anchor above threshold, silently re-point _bt_track_id
        # at it.  The public track_id (stable identity) is NOT changed.
        s = self.state

        # Advance the re-acquisition anchor once per lost frame (velocity mode),
        # BEFORE the reacq gate evaluates against it.  Frozen mode leaves it put.
        if cur is None and self._bt_track_id is not None and self._anchor is not None \
                and self.reacq_motion == "velocity":
            vx, vy = s.last_velocity
            ax, ay, aw, ah = self._anchor
            self._anchor = (ax + vx, ay + vy, aw, ah)

        if cur is None and self._bt_track_id is not None and self._anchor is not None:
            best_iou, best_track = _REACQ_IOU, None
            for t in tracks:
                if t.is_activated and t.filtered_tlwh and t.track_id != self._bt_track_id:
                    iou = _iou_tlwh(self._anchor, t.filtered_tlwh)
                    if iou > best_iou:
                        best_iou, best_track = iou, t
            if best_track is not None:
                self._bt_track_id = best_track.track_id  # internal update only
                cur = best_track

        # Time-growing centre-distance fallback: runs only when the IoU pass
        # above found nothing AND the radius-growth knob is enabled.  The search
        # radius widens with frames_since_seen so a target that moved far during
        # a long loss (IoU=0) can still be re-adopted by centre proximity.
        if cur is None and self._bt_track_id is not None and self._anchor is not None \
                and self.reacq_radius_growth > 0:
            ax, ay, aw, ah = self._anchor
            acx, acy = ax + aw / 2, ay + ah / 2
            r0 = max(aw, ah)
            r = r0 + self.reacq_radius_growth * s.frames_since_seen
            best_d, best_track = r, None
            for t in tracks:
                if t.is_activated and t.filtered_tlwh and t.track_id != self._bt_track_id:
                    tx, ty, tw, th = t.filtered_tlwh
                    d = ((tx + tw / 2 - acx) ** 2 + (ty + th / 2 - acy) ** 2) ** 0.5
                    if d < best_d:
                        best_d, best_track = d, t
            if best_track is not None:
                self._bt_track_id = best_track.track_id
                cur = best_track

        if cur is not None:
            had_prev = s.frames_since_seen == 0 and s.bbox_norm[2] > 0
            cx_old = s.bbox_norm[0] + s.bbox_norm[2] / 2 if had_prev else None
            cy_old = s.bbox_norm[1] + s.bbox_norm[3] / 2 if had_prev else None
            s.bbox_norm = tuple(cur.filtered_tlwh)
            cx_new = s.bbox_norm[0] + s.bbox_norm[2] / 2
            cy_new = s.bbox_norm[1] + s.bbox_norm[3] / 2
            if cx_old is not None:
                s.last_velocity = (cx_new - cx_old, cy_new - cy_old)
            self._anchor = tuple(cur.filtered_tlwh)
            s.status = "TRACKING"
            s.frames_since_seen = 0
            self._seed_age = 0
        elif self._pending_seed is not None:
            # Seeded but not yet bound to a track: report TRACKING at the seed so
            # the scheduler places an ROI tile there, densely sampling that spot
            # until a track activates and lock_from_gt (above) adopts it. Give up
            # after seed_timeout frames so a bad seed doesn't pin the ROI forever.
            s.bbox_norm = tuple(self._pending_seed)
            s.status = "TRACKING"
            s.frames_since_seen = 0
            self._anchor = tuple(self._pending_seed)
            self._seed_age += 1
            if self._seed_age > self.seed_timeout:
                self._pending_seed = None
                self._seed_age = 0
                s.status = "LOST"
        else:
            # last_velocity is deliberately retained during loss so the scheduler can extrapolate ROI placement.
            s.frames_since_seen += 1
            s.status = "SEARCHING" if s.frames_since_seen <= self.track_buffer else "LOST"
        s.track_id = self.track_id
        return s


class MultiTargetLock:
    """Tracks ALL activated ByteTracker tracks across multiple classes.

    One ByteTrackerAdapter per allowed class avoids cross-class IoU id
    collisions.  A single ``selected_key`` (composite key ``(cls, track_id)``)
    marks the user-selected target; it is set once via GT IoU matching and
    never cleared automatically.
    """

    def __init__(self, target_classes=frozenset({0, 1}),
                 track_buffer: int = 90, **tracker_kwargs):
        self._trackers = {c: create_tracker("byte", track_buffer=track_buffer,
                                            **tracker_kwargs)
                          for c in target_classes}
        self.target_classes: set = set(target_classes)
        self.track_buffer = track_buffer
        self.targets: dict = {}          # (cls, track_id) -> TargetState
        self.selected_key: tuple | None = None

    def step(self, dets: Sequence[Det], *,
             gt_bbox_norm: tuple | None = None,
             gt_cls: int | None = None) -> list:
        """Feed one frame of detections; return non-LOST TargetStates."""
        # --- 1. partition dets by class and run each tracker ---
        active_keys: set = set()
        for cls, tracker in self._trackers.items():
            cls_dets = [d for d in dets if d.cls == cls]
            tracks = tracker.update(dets_to_array(cls_dets))
            for t in tracks:
                if not t.is_activated or not t.filtered_tlwh:
                    continue
                key = (cls, t.track_id)
                active_keys.add(key)
                s = self.targets.get(key)
                if s is None:
                    s = TargetState(key=key, cls=cls)
                    self.targets[key] = s
                # velocity update (same rule as v1 TargetLock)
                had_prev = s.frames_since_seen == 0 and s.bbox_norm[2] > 0
                cx_old = (s.bbox_norm[0] + s.bbox_norm[2] / 2) if had_prev else None
                cy_old = (s.bbox_norm[1] + s.bbox_norm[3] / 2) if had_prev else None
                s.bbox_norm = tuple(t.filtered_tlwh)
                cx_new = s.bbox_norm[0] + s.bbox_norm[2] / 2
                cy_new = s.bbox_norm[1] + s.bbox_norm[3] / 2
                if cx_old is not None:
                    s.last_velocity = (cx_new - cx_old, cy_new - cy_old)
                s.status = "TRACKING"
                s.frames_since_seen = 0

        # --- 2. age inactive tracks ---
        for key, s in list(self.targets.items()):
            if key not in active_keys:
                s.frames_since_seen += 1
                if s.frames_since_seen > self.track_buffer:
                    s.status = "LOST"
                elif s.status != "LOST":
                    s.status = "SEARCHING"

        # --- 3. GT-seeded selection (once only) ---
        if self.selected_key is None and gt_bbox_norm is not None:
            best_key, best_iou = None, _REACQ_IOU
            for key, s in self.targets.items():
                if s.status == "TRACKING":
                    if gt_cls is not None and s.cls != gt_cls:
                        continue
                    iou = _iou_tlwh(gt_bbox_norm, s.bbox_norm)
                    if iou > best_iou:
                        best_iou, best_key = iou, key
            if best_key is not None:
                self.selected_key = best_key

        # --- 4. propagate selected flag ---
        for key, s in self.targets.items():
            s.selected = (key == self.selected_key)

        # --- 5. return non-LOST, sorted TRACKING first, selected first ---
        alive = [s for s in self.targets.values() if s.status != "LOST"]
        alive.sort(key=lambda s: (0 if s.status == "TRACKING" else 1,
                                  0 if s.selected else 1))
        return alive
