from __future__ import annotations

from typing import Sequence

import numpy as np

import dynamic_tiling  # noqa: F401  ensures _vendor_paths ran
from drone_follow.pipeline_adapter.tracker_factory import create_tracker

from .types import Det, LockState

_SCALE = 1000.0


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
    """

    def __init__(self, track_buffer: int = 90, **tracker_kwargs):
        self._tracker = create_tracker("byte", track_buffer=track_buffer,
                                       **tracker_kwargs)
        self.track_buffer = track_buffer
        # Public stable identity: set once on first lock, never changed.
        self.track_id: int | None = None
        # Internal: current ByteTracker track id (may diverge on re-acquisition).
        self._bt_track_id: int | None = None
        self.state = LockState()

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
        if best is not None and best_iou > 0.3:
            self._set_track(best.track_id)

    def step(self, person_dets: Sequence[Det], *, lock_if_unlocked: bool = False,
             gt_bbox_norm: tuple | None = None) -> LockState:
        tracks = self._tracker.update(dets_to_array(person_dets))

        if self._bt_track_id is None:
            if gt_bbox_norm is not None:
                self.lock_from_gt(gt_bbox_norm, tracks)
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
        # last known bbox above threshold, silently re-point _bt_track_id at it.
        # The public track_id (stable identity) is NOT changed.
        s = self.state
        if cur is None and self._bt_track_id is not None and s.bbox_norm[2] > 0:
            best_iou, best_track = 0.3, None
            for t in tracks:
                if t.is_activated and t.filtered_tlwh and t.track_id != self._bt_track_id:
                    iou = _iou_tlwh(s.bbox_norm, t.filtered_tlwh)
                    if iou > best_iou:
                        best_iou, best_track = iou, t
            if best_track is not None:
                self._bt_track_id = best_track.track_id  # internal update only
                cur = best_track

        if cur is not None:
            cx_old = s.bbox_norm[0] + s.bbox_norm[2] / 2 if s.frames_since_seen == 0 else None
            cy_old = s.bbox_norm[1] + s.bbox_norm[3] / 2 if s.frames_since_seen == 0 else None
            s.bbox_norm = tuple(cur.filtered_tlwh)
            cx_new = s.bbox_norm[0] + s.bbox_norm[2] / 2
            cy_new = s.bbox_norm[1] + s.bbox_norm[3] / 2
            if cx_old is not None:
                s.last_velocity = (cx_new - cx_old, cy_new - cy_old)
            s.status = "TRACKING"
            s.frames_since_seen = 0
        else:
            s.frames_since_seen += 1
            s.status = "SEARCHING" if s.frames_since_seen <= self.track_buffer else "LOST"
        s.track_id = self.track_id
        return s
