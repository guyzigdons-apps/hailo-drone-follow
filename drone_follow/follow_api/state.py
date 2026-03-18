"""Thread-safe shared state — no third-party dependencies."""

import logging
import threading
import time
from collections import deque
from typing import Optional

from .types import Detection, TrackingMode

LOGGER = logging.getLogger("drone_follow.state")


class SharedDetectionState:
    """Thread-safe state for passing detections from the pipeline callback to the control loop."""

    def __init__(self):
        self._lock = threading.Lock()
        self._detection: Optional[Detection] = None
        self._frame_count: int = 0
        self._available_ids: set = set()
        self.tracker_metrics = None  # set externally to a TrackerMetrics instance

    def update(self, detection: Optional[Detection], available_ids: set = None):
        with self._lock:
            self._detection = detection
            self._frame_count += 1
            if available_ids is not None:
                self._available_ids = available_ids

    def get_latest(self):
        with self._lock:
            return self._detection, self._frame_count

    def get_available_ids(self):
        """Get the set of currently visible detection IDs."""
        with self._lock:
            return self._available_ids.copy()


class FollowTargetState:
    """Thread-safe state for which detection ID to follow.

    Manages two tracking modes:
      MOT — all people tracked, drone waits for user to select a target.
      SOT — following a specific person by track ID.

    Also maintains a ReID embedding store: when the followed target is lost,
    nearby embeddings (up to ``max_reid_embeddings``) are saved so a future
    ReID module can attempt re-identification.
    """

    MAX_REID_EMBEDDINGS = 10
    REACQUIRE_TIMEOUT_S = 5.0  # seconds to attempt reacquisition before giving up

    def __init__(self):
        self._lock = threading.Lock()
        self._target_id: Optional[int] = None
        self._last_seen: Optional[float] = None
        self._mode: TrackingMode = TrackingMode.MOT
        # Last known normalized (0-1) center position of the followee
        self._last_known_position: Optional[tuple] = None  # (cx, cy)
        # Timestamp when target was lost — used for reacquisition timeout
        self._target_lost_at: Optional[float] = None
        # ReID embedding store — list of (track_id, embedding) tuples.
        # Populated when target is lost with embeddings of nearby detections.
        # ``embedding`` is currently None (placeholder for a future feature
        # extractor); the slot is reserved so the pipeline can be extended
        # without changing the state API.
        self._reid_embeddings: deque = deque(maxlen=self.MAX_REID_EMBEDDINGS)

    # ---- Mode management ----

    def get_mode(self) -> TrackingMode:
        with self._lock:
            return self._mode

    def set_target(self, detection_id: Optional[int]):
        """Set the target detection ID to follow.

        Setting a non-None ID switches to SOT mode.
        Setting None switches back to MOT mode (embeddings preserved
        for reacquisition if they were stored before this call).
        """
        with self._lock:
            self._target_id = detection_id
            if detection_id is not None:
                self._mode = TrackingMode.SOT
                self._last_seen = time.monotonic()
                self._target_lost_at = None
                self._reid_embeddings.clear()
                LOGGER.info("[MODE] SOT — following ID %d", detection_id)
            else:
                self._mode = TrackingMode.MOT
                self._target_lost_at = time.monotonic()
                LOGGER.info("[MODE] MOT — waiting for target selection")

    def get_target(self) -> Optional[int]:
        """Get the current target detection ID."""
        with self._lock:
            return self._target_id

    def update_last_seen(self, position: Optional[tuple] = None):
        """Update the last seen timestamp and optionally position for the current target.

        Args:
            position: (center_x, center_y) in normalized 0-1 coords, or None.
        """
        with self._lock:
            if self._target_id is not None:
                self._last_seen = time.monotonic()
                if position is not None:
                    self._last_known_position = position

    def get_last_seen(self) -> Optional[float]:
        """Get the last seen timestamp (monotonic) for the current target."""
        with self._lock:
            return self._last_seen

    def get_last_known_position(self) -> Optional[tuple]:
        """Get the last known (cx, cy) of the followee (normalized 0-1)."""
        with self._lock:
            return self._last_known_position

    # ---- ReID embedding store ----

    def store_reid_embeddings(self, nearby: list):
        """Store embeddings of detections near the last known target position.

        Called when the target is lost.  Each entry is a dict with at least
        ``track_id`` and ``embedding`` (currently None — placeholder for
        future feature extraction).  Only the closest ``MAX_REID_EMBEDDINGS``
        entries are kept.

        Args:
            nearby: list of dicts ``{"track_id": int, "embedding": Any,
                    "distance": float}`` sorted by ascending distance to last
                    known position.
        """
        with self._lock:
            self._reid_embeddings.clear()
            for entry in nearby[: self.MAX_REID_EMBEDDINGS]:
                self._reid_embeddings.append(entry)
            if nearby:
                LOGGER.debug("[REID] Stored %d nearby embeddings for re-identification",
                             len(self._reid_embeddings))

    def get_reid_embeddings(self) -> list:
        """Return stored ReID embeddings (list of dicts)."""
        with self._lock:
            return list(self._reid_embeddings)

    def has_reacquire_data(self) -> bool:
        """True if we have saved embeddings and are within the reacquisition timeout."""
        with self._lock:
            if not self._reid_embeddings or self._target_lost_at is None:
                return False
            elapsed = time.monotonic() - self._target_lost_at
            return elapsed < self.REACQUIRE_TIMEOUT_S

    def clear_reacquire(self):
        """Clear saved embeddings and lost timestamp (reacquisition gave up)."""
        with self._lock:
            self._reid_embeddings.clear()
            self._target_lost_at = None

    # ---- Status ----

    def get_status(self):
        """Get current status as a dict."""
        with self._lock:
            return {
                "following_id": self._target_id,
                "last_seen": self._last_seen,
                "tracking_mode": self._mode.value,
                "reid_embedding_count": len(self._reid_embeddings),
            }
