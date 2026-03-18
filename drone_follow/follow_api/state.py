"""Thread-safe shared state — no third-party dependencies."""

import logging
import threading
import time
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

    When ReID is enabled, the target's appearance embeddings are collected
    every few frames while following.  On target loss, these saved embeddings
    are used to re-identify the person in subsequent frames.
    """

    REACQUIRE_TIMEOUT_S = 5.0  # seconds to attempt reacquisition before giving up
    EMBEDDING_SAVE_INTERVAL = 5  # save one embedding every N frames
    MAX_TARGET_EMBEDDINGS = 20  # rolling window of saved embeddings

    def __init__(self):
        self._lock = threading.Lock()
        self._target_id: Optional[int] = None
        self._display_id: Optional[int] = None
        self._last_seen: Optional[float] = None
        self._mode: TrackingMode = TrackingMode.MOT
        # Last known normalized (0-1) center position of the followee
        self._last_known_position: Optional[tuple] = None  # (cx, cy)
        # Timestamp when target was lost — used for reacquisition timeout
        self._target_lost_at: Optional[float] = None
        # ReID target embeddings — appearance vectors of the followed person,
        # collected every EMBEDDING_SAVE_INTERVAL frames while in SOT mode.
        # Used for re-identification after target loss.
        self._target_embeddings: list = []
        self._embedding_frame_counter: int = 0

    # ---- Mode management ----

    def get_mode(self) -> TrackingMode:
        with self._lock:
            return self._mode

    def set_target(self, detection_id: Optional[int], display_id: Optional[int] = None,
                   reacquired: bool = False):
        """Set the target detection ID to follow.

        Setting a non-None ID switches to SOT mode.
        Setting None switches back to MOT mode (target embeddings preserved
        for ReID reacquisition).

        Args:
            detection_id: internal MOT track ID to follow.
            display_id: ID shown in the UI. Defaults to detection_id.
            reacquired: True when re-acquiring a lost target via ReID.
                        Preserves saved target embeddings and display_id.
        """
        with self._lock:
            self._target_id = detection_id
            self._display_id = display_id if display_id is not None else detection_id
            if detection_id is not None:
                self._mode = TrackingMode.SOT
                self._last_seen = time.monotonic()
                self._target_lost_at = None
                if not reacquired:
                    # Fresh target selection — start collecting new embeddings
                    self._target_embeddings = []
                self._embedding_frame_counter = 0
                LOGGER.info("[MODE] SOT — following ID %d (display %d)%s",
                            detection_id, self._display_id,
                            " (reacquired)" if reacquired else "")
            else:
                self._mode = TrackingMode.MOT
                # Keep _display_id and _target_embeddings for reacquisition
                self._target_lost_at = time.monotonic()
                LOGGER.info("[MODE] MOT — waiting for target selection"
                            " (%d embeddings saved)", len(self._target_embeddings))

    def get_target(self) -> Optional[int]:
        """Get the current target detection ID."""
        with self._lock:
            return self._target_id

    def get_display_id(self) -> Optional[int]:
        """Get the ID to show in the UI (original ID before any ID switches)."""
        with self._lock:
            return self._display_id

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

    # ---- ReID target embeddings ----

    def add_target_embedding(self, embedding):
        """Save a pre-normalized ReID embedding of the followed target.

        Called every frame while in SOT mode; internally saves only every
        ``EMBEDDING_SAVE_INTERVAL`` calls, keeping a rolling window of
        ``MAX_TARGET_EMBEDDINGS``.
        """
        with self._lock:
            self._embedding_frame_counter += 1
            if self._embedding_frame_counter > 1 and self._embedding_frame_counter % self.EMBEDDING_SAVE_INTERVAL != 0:
                return
            self._target_embeddings.append(embedding)
            if len(self._target_embeddings) > self.MAX_TARGET_EMBEDDINGS:
                self._target_embeddings.pop(0)

    def get_target_embeddings(self) -> list:
        """Return saved target embeddings (list of numpy arrays)."""
        with self._lock:
            return list(self._target_embeddings)

    def has_reacquire_data(self) -> bool:
        """True if we have saved target embeddings and are within the reacquisition timeout."""
        with self._lock:
            if not self._target_embeddings or self._target_lost_at is None:
                return False
            elapsed = time.monotonic() - self._target_lost_at
            return elapsed < self.REACQUIRE_TIMEOUT_S

    def clear_reacquire(self):
        """Clear saved embeddings and lost timestamp (reacquisition gave up)."""
        with self._lock:
            self._target_embeddings = []
            self._target_lost_at = None

    # ---- Status ----

    def get_status(self):
        """Get current status as a dict."""
        with self._lock:
            return {
                "following_id": self._target_id,
                "last_seen": self._last_seen,
                "tracking_mode": self._mode.value,
                "reid_embedding_count": len(self._target_embeddings),
            }
