"""Thread-safe shared state — no third-party dependencies."""

import logging
import threading
import time
from typing import Optional

from .types import Detection

LOGGER = logging.getLogger(__name__)


class SharedDetectionState:
    """Thread-safe state for passing detections from the pipeline callback to the control loop."""

    def __init__(self):
        self._lock = threading.Lock()
        self._detection: Optional[Detection] = None
        self._frame_count: int = 0
        self._available_ids: set = set()
        # {track_id -> (x, y, w, h)} in normalized [0..1] coords. Atomic with
        # _available_ids so follow_server can match a client-supplied bbox
        # against the SAME snapshot the id check ran against — no ui_state race.
        self._available_bboxes: dict = {}

    def update(self, detection: Optional[Detection], available_ids: set,
               available_bboxes: Optional[dict] = None):
        with self._lock:
            self._detection = detection
            self._frame_count += 1
            self._available_ids = available_ids
            self._available_bboxes = dict(available_bboxes) if available_bboxes else {}

    def get_latest(self):
        with self._lock:
            return self._detection, self._frame_count

    def get_available_ids(self):
        """Get the set of currently visible detection IDs."""
        with self._lock:
            return self._available_ids.copy()

    def get_available_bboxes(self):
        """Get a {id -> (x, y, w, h)} snapshot of currently visible bboxes.

        Returned dict is a defensive copy under lock so the caller can iterate
        without holding the state lock; bboxes are tuples (immutable).
        """
        with self._lock:
            return dict(self._available_bboxes)


class FollowTargetState:
    """Thread-safe state for which detection ID to follow."""

    def __init__(self):
        self._lock = threading.Lock()
        self._target_id: Optional[int] = None
        self._last_seen: Optional[float] = None
        self._paused: bool = False
        self._explicit_lock: bool = False
        # Followee-change marker. Updated whenever set_target() or
        # enter_auto_mode() change the followee (None ↔ id ↔ different id).
        # Consumers: cairooverlay badge flash, UI toast, info log on switch.
        self._last_change_ts: Optional[float] = None

    def set_paused(self, paused: bool):
        """Pause or resume drone follow. When paused the control loop holds position."""
        with self._lock:
            self._paused = paused

    def is_paused(self) -> bool:
        """Return True if drone follow is paused (IDLE mode)."""
        with self._lock:
            return self._paused

    def set_explicit_lock(self, locked: bool):
        """Mark whether the current target was explicitly chosen by the operator."""
        with self._lock:
            self._explicit_lock = locked

    def is_explicit_lock(self) -> bool:
        """Return True if the current target was explicitly locked by the operator."""
        with self._lock:
            return self._explicit_lock

    def enter_auto_mode(self):
        """Atomically reset to AUTO mode: no target, not paused, not locked."""
        with self._lock:
            previous = self._target_id
            changed = previous is not None
            if changed:
                self._last_change_ts = time.monotonic()
            self._target_id = None
            self._paused = False
            self._explicit_lock = False
        if changed:
            LOGGER.info("[FOLLOW] cleared target (was ID %s) — back to AUTO", previous)

    def set_target(self, detection_id: Optional[int]):
        """Set the target detection ID to follow."""
        with self._lock:
            previous = self._target_id
            changed = detection_id != previous
            if changed:
                self._last_change_ts = time.monotonic()
            self._target_id = detection_id
            if detection_id is not None:
                self._last_seen = time.monotonic()
        if changed:
            if detection_id is None:
                LOGGER.info("[FOLLOW] cleared target (was ID %s)", previous)
            else:
                LOGGER.info(
                    "[FOLLOW] now following ID %s%s",
                    detection_id,
                    f" (was ID {previous})" if previous is not None else "",
                )

    def get_last_change_ts(self) -> Optional[float]:
        """Monotonic timestamp of the most recent followee transition.

        Used by the cairooverlay badge to flash a short notification when
        someone changes the followee (operator click, REID-DRIFT switch,
        AUTO re-acquire, Clear Target). ``None`` until the first switch.
        """
        with self._lock:
            return self._last_change_ts

    def get_target(self) -> Optional[int]:
        """Get the current target detection ID."""
        with self._lock:
            return self._target_id

    def update_last_seen(self):
        """Update the last seen timestamp for the current target."""
        with self._lock:
            if self._target_id is not None:
                self._last_seen = time.monotonic()

    def get_last_seen(self) -> Optional[float]:
        """Get the last seen timestamp (monotonic) for the current target."""
        with self._lock:
            return self._last_seen

    def get_status(self):
        """Get current status as a dict."""
        with self._lock:
            return {
                "following_id": self._target_id,
                "last_seen": self._last_seen
            }
