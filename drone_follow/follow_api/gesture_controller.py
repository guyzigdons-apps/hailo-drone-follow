"""Gesture controller — pure domain logic for gesture-based drone control.

No MAVSDK, no Hailo, no GStreamer dependencies.
Only depends on standard library + the types/config from this package.
"""

import math
import time
from collections import deque
from typing import Optional

from .types import GestureDetection, VelocityCommand
from .config import ControllerConfig


def classify_hand_open(fingertip_positions: list, wrist_position: tuple) -> bool:
    """Classify hand as open or fist based on fingertip-to-wrist distances.

    Args:
        fingertip_positions: List of (x, y) tuples for each fingertip.
        wrist_position: (x, y) tuple of wrist landmark.

    Returns:
        True if open palm, False if fist.
    """
    if not fingertip_positions:
        return False
    wx, wy = wrist_position
    total_dist = 0.0
    for fx, fy in fingertip_positions:
        total_dist += math.sqrt((fx - wx) ** 2 + (fy - wy) ** 2)
    avg_dist = total_dist / len(fingertip_positions)
    # Threshold: average fingertip distance > 0.15 (normalized) means open palm
    return avg_dist > 0.15


class WaveDetector:
    """Detects a wave gesture by tracking hand_x oscillation over time.

    Counts direction reversals of hand_x in a sliding time window.
    A wave is detected when the reversal count reaches the configured threshold.
    """

    def __init__(self, reversals_needed: int = 3, window_s: float = 1.5):
        self._reversals_needed = reversals_needed
        self._window_s = window_s
        self._history: deque = deque()  # (timestamp, hand_x)
        self._last_direction: Optional[int] = None  # +1 or -1
        self._reversals: deque = deque()  # timestamps of reversals
        self._prev_x: Optional[float] = None
        self._detected = False

    def update(self, hand_x: float, timestamp: float) -> bool:
        """Update with new hand_x position. Returns True when wave detected."""
        if self._detected:
            return False

        # Prune old reversal timestamps
        cutoff = timestamp - self._window_s
        while self._reversals and self._reversals[0] < cutoff:
            self._reversals.popleft()

        if self._prev_x is not None:
            delta = hand_x - self._prev_x
            min_delta = 0.02  # minimum movement to count as direction
            if abs(delta) > min_delta:
                direction = 1 if delta > 0 else -1
                if self._last_direction is not None and direction != self._last_direction:
                    self._reversals.append(timestamp)
                self._last_direction = direction

        self._prev_x = hand_x

        if len(self._reversals) >= self._reversals_needed:
            self._detected = True
            return True
        return False

    def reset(self):
        """Reset detector for reuse."""
        self._history.clear()
        self._reversals.clear()
        self._last_direction = None
        self._prev_x = None
        self._detected = False

    @property
    def is_detected(self) -> bool:
        return self._detected


def compute_gesture_lateral(
    gesture: Optional[GestureDetection],
    config: ControllerConfig,
) -> float:
    """Compute lateral velocity from hand-face X offset for gesture overlay on follow mode.

    Returns right_m_s: positive = strafe right, negative = strafe left.
    Zero when no gesture, no hand, fist, or hand centered on face.
    """
    if gesture is None or gesture.hand is None or not gesture.hand.is_open:
        return 0.0

    hand_offset_x = gesture.hand.center_x - gesture.face.center_x
    offset_deg = hand_offset_x * config.hfov

    if abs(offset_deg) < config.gesture_hand_dead_zone_deg:
        return 0.0

    lateral = math.copysign(
        config.gesture_kp_lateral * math.sqrt(abs(offset_deg)),
        hand_offset_x,
    )
    return max(-config.gesture_max_lateral, min(config.gesture_max_lateral, lateral))


def compute_gesture_velocity_command(
    gesture: Optional[GestureDetection],
    config: ControllerConfig,
    last_gesture: Optional[GestureDetection] = None,
    search_active: bool = False,
) -> VelocityCommand:
    """Compute a velocity command from gesture detection.

    Phases:
    1. No detection -> search mode (slow spin) or hold position
    2. Face only (no hand) -> yaw to center face, hover
    3. Open palm -> yaw from hand-face offset, forward from arm extension
    4. Fist -> zero velocity, face-tracking yaw only

    Returns a pure VelocityCommand.
    """
    # No detection -> search or hold
    if gesture is None:
        if not search_active:
            return VelocityCommand(0.0, 0.0, 0.0, 0.0)
        search_direction = -1.0
        if last_gesture is not None:
            search_direction = -1.0 if last_gesture.face.center_x > 0.5 else 1.0
        return VelocityCommand(0.0, 0.0, 0.0, search_direction * config.search_yawspeed_slow)

    # Safety: person too close
    if gesture.person_bbox_height > config.max_bbox_height_safety:
        # Emergency backward + face-centering yaw
        face_yaw = _face_centering_yaw(gesture.face.center_x, config)
        return VelocityCommand(-config.max_backward, 0.0, 0.0, face_yaw)

    # Phase: no hand visible -> face centering only
    if gesture.hand is None:
        yaw = _face_centering_yaw(gesture.face.center_x, config)
        return VelocityCommand(0.0, 0.0, 0.0, yaw)

    # Phase: fist -> stop, face tracking yaw only
    if not gesture.hand.is_open:
        yaw = _face_centering_yaw(gesture.face.center_x, config)
        return VelocityCommand(0.0, 0.0, 0.0, yaw)

    # Phase: open palm -> hand-based yaw + arm-extension forward
    hand_offset_x = gesture.hand.center_x - gesture.face.center_x
    hand_offset_deg = hand_offset_x * config.hfov

    # Yaw from hand offset (hand right of face -> yaw right = negative in PX4)
    # Opposite sign to follow mode: hand is a joystick command, not tracking error
    if abs(hand_offset_deg) < config.gesture_hand_dead_zone_deg:
        yawspeed = 0.0
    else:
        yawspeed = -math.copysign(
            config.gesture_kp_hand_yaw * math.sqrt(abs(hand_offset_deg)),
            hand_offset_x
        )
    yawspeed = max(-config.gesture_max_yawspeed, min(config.gesture_max_yawspeed, yawspeed))

    # Forward from arm extension (wrist-to-face distance)
    arm_dx = gesture.hand.wrist_x - gesture.face.center_x
    arm_dy = gesture.hand.wrist_y - gesture.face.center_y
    arm_extension = math.sqrt(arm_dx ** 2 + arm_dy ** 2)

    if arm_extension < config.gesture_arm_dead_zone:
        forward = 0.0
    else:
        effective_ext = arm_extension - config.gesture_arm_dead_zone
        forward = config.gesture_kp_forward * effective_ext
    forward = min(forward, config.gesture_max_forward)

    return VelocityCommand(forward, 0.0, 0.0, yawspeed)


def _face_centering_yaw(face_center_x: float, config: ControllerConfig) -> float:
    """Compute yaw to center face in frame. Same signed-sqrt response as follow mode."""
    error_x_deg = (face_center_x - 0.5) * config.hfov
    if abs(error_x_deg) < config.dead_zone_deg:
        return 0.0
    yawspeed = -math.copysign(config.kp_yaw * math.sqrt(abs(error_x_deg)), error_x_deg)
    return max(-config.gesture_max_yawspeed, min(config.gesture_max_yawspeed, yawspeed))
