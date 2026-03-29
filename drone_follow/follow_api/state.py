"""Thread-safe shared state — no third-party dependencies."""

import threading
import time
from typing import List, Optional

from .types import Detection, GestureDetection


class SharedDetectionState:
    """Thread-safe state for passing detections from the pipeline callback to the control loop."""

    def __init__(self):
        self._lock = threading.Lock()
        self._detection: Optional[Detection] = None
        self._frame_count: int = 0
        self._available_ids: set = set()

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


class SharedGestureState:
    """Thread-safe state for passing gesture detections from the pipeline to the control loop."""

    def __init__(self):
        self._lock = threading.Lock()
        self._gesture: Optional[GestureDetection] = None
        self._frame_count: int = 0

    def update(self, gesture: Optional[GestureDetection]):
        with self._lock:
            self._gesture = gesture
            self._frame_count += 1

    def get_latest(self):
        with self._lock:
            return self._gesture, self._frame_count


class FollowTargetState:
    """Thread-safe state for which detection ID to follow."""

    def __init__(self):
        self._lock = threading.Lock()
        self._target_id: Optional[int] = None
        self._last_seen: Optional[float] = None

    def set_target(self, detection_id: Optional[int]):
        """Set the target detection ID to follow."""
        with self._lock:
            self._target_id = detection_id
            if detection_id is not None:
                self._last_seen = time.monotonic()

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


class SharedGestureLateralState:
    """Thread-safe lateral velocity from gesture overlay.

    Producer: gesture_control_loop (computes lateral from hand-face offset).
    Consumer: live_control_loop (injects into follow command's right_m_s).
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._lateral: float = 0.0

    def update(self, lateral: float):
        with self._lock:
            self._lateral = lateral

    def get(self) -> float:
        with self._lock:
            return self._lateral


class SharedVelocityState:
    """Thread-safe state for passing velocity commands from control loop to pipeline callback."""

    def __init__(self):
        self._lock = threading.Lock()
        self._forward_m_s: float = 0.0
        self._right_m_s: float = 0.0
        self._down_m_s: float = 0.0
        self._yawspeed_deg_s: float = 0.0
        self._mode: str = "IDLE"

    def update(self, forward_m_s: float, right_m_s: float, down_m_s: float, yawspeed_deg_s: float, mode: str):
        with self._lock:
            self._forward_m_s = forward_m_s
            self._right_m_s = right_m_s
            self._down_m_s = down_m_s
            self._yawspeed_deg_s = yawspeed_deg_s
            self._mode = mode

    def get(self):
        """Returns (forward_m_s, right_m_s, down_m_s, yawspeed_deg_s, mode)."""
        with self._lock:
            return (self._forward_m_s, self._right_m_s, self._down_m_s,
                    self._yawspeed_deg_s, self._mode)
