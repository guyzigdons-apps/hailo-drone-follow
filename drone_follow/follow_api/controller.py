"""Pure follow controller logic — no MAVSDK, no Hailo, no GStreamer dependencies.

Only depends on standard library + the types/config from this package.
"""

import math
import time
from typing import Optional

from .types import Detection, VelocityCommand
from .config import ControllerConfig

__all__ = [
    "compute_velocity_command",
    "ForwardSmoother",
    "reset_forward_dead_zone",
]


def _distance_to_bbox_height(
    altitude_m: float,
    horizontal_distance_m: float,
    vfov_deg: float,
    person_height_m: float = 1.7,
) -> float:
    """Convert desired horizontal distance to expected normalized bbox height (0-1).

    Uses perspective projection: at a given altitude and horizontal distance,
    compute what fraction of the vertical FOV an average person occupies.
    """
    slant_range = math.sqrt(horizontal_distance_m ** 2 + altitude_m ** 2)
    angular_height = 2.0 * math.atan(person_height_m / (2.0 * slant_range))
    vfov_rad = math.radians(vfov_deg)
    return angular_height / vfov_rad


class _ForwardDeadZone:
    """Hysteresis dead zone for forward/backward control.

    Uses two thresholds: a wider one to *exit* the dead zone (start moving)
    and a narrower one to *re-enter* the dead zone (stop moving).  This prevents
    the controller from flickering in and out of the dead zone when bbox height
    hovers near the boundary.
    """
    _in_dead_zone: bool = True

    def update(self, abs_delta: float, exit_threshold: float, reenter_threshold: float) -> bool:
        """Return True if the error is inside the dead zone."""
        if self._in_dead_zone:
            # Need a larger error to break out
            if abs_delta >= exit_threshold:
                self._in_dead_zone = False
        else:
            # Need error to drop below the tighter threshold to re-enter
            if abs_delta < reenter_threshold:
                self._in_dead_zone = True
        return self._in_dead_zone

    def reset(self):
        self._in_dead_zone = True


# Module-level dead zone state (reset when smoother is reset)
_fwd_dead_zone = _ForwardDeadZone()


def reset_forward_dead_zone():
    """Reset the forward dead zone hysteresis state. Call between flights or in tests."""
    _fwd_dead_zone.reset()


def _calculate_forward_speed(
    detection: Detection,
    config: ControllerConfig,
    target_bh: float,
) -> float:
    """Calculate forward/backward speed based on bbox height and bottom-of-frame position."""
    if config.yaw_only or config.kp_forward == 0:
        return 0.0

    if detection.bbox_height > config.max_bbox_height_safety:
        return -config.max_backward

    # Bottom-of-frame safety: if the bbox bottom edge is too low in the frame,
    # the person is directly beneath the drone — command backward to retreat.
    bbox_bottom = detection.center_y + detection.bbox_height / 2.0
    if bbox_bottom > config.bottom_y_threshold:
        overshoot = bbox_bottom - config.bottom_y_threshold
        return -config.kp_backward * math.sqrt(overshoot)

    height_delta = target_bh - detection.bbox_height
    exit_threshold = (config.dead_zone_height_percent / 100.0) * target_bh
    reenter_threshold = (config.dead_zone_reenter_percent / 100.0) * target_bh

    if _fwd_dead_zone.update(abs(height_delta), exit_threshold, reenter_threshold):
        return 0.0

    # Smooth ramp: linear blend from 0 to full sqrt response over the range
    # [reenter_threshold, 2*exit_threshold].  Avoids the discontinuous jump
    # at the dead zone edge.  With hysteresis we may be active at abs_delta
    # below exit_threshold, so ramp from the reenter boundary instead.
    abs_delta = abs(height_delta)
    ramp_end = 2.0 * exit_threshold
    if abs_delta < ramp_end:
        ramp = max(0.0, (abs_delta - reenter_threshold) / (ramp_end - reenter_threshold))
    else:
        ramp = 1.0

    if height_delta > 0:
        forward = ramp * config.kp_forward * math.sqrt(abs_delta)
    else:
        forward = -ramp * config.kp_backward * math.sqrt(abs_delta)

    return forward


class ForwardSmoother:
    """Estimates person approach/recede velocity and smooths forward commands.

    Tracks bbox_height over time to compute d(bbox_height)/dt, then uses that
    as a derivative feed-forward term. Also applies EMA to the final forward
    velocity to avoid big jumps.
    """

    def __init__(self):
        self._smoothed_forward: float = 0.0
        self._prev_bbox_h: Optional[float] = None
        self._prev_time: Optional[float] = None
        self._bbox_h_rate: float = 0.0  # EMA of d(bbox_height)/dt
        self._rate_alpha: float = 0.3   # smoothing for rate estimation

    def update(self, detection: Optional[Detection], raw_forward: float,
               config: ControllerConfig) -> float:
        """Return smoothed forward velocity."""
        now = time.monotonic()

        # Update bbox height rate estimate
        if detection is not None and self._prev_bbox_h is not None and self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 0.01:
                instant_rate = (detection.bbox_height - self._prev_bbox_h) / dt
                self._bbox_h_rate = (self._rate_alpha * instant_rate
                                     + (1.0 - self._rate_alpha) * self._bbox_h_rate)
        if detection is not None:
            self._prev_bbox_h = detection.bbox_height
            self._prev_time = now
        else:
            self._bbox_h_rate *= 0.9

        # Derivative feed-forward: positive rate means person is getting closer (bbox growing)
        # -> we should move backward (negative forward). Negative rate -> move forward.
        derivative_term = -config.kd_forward * self._bbox_h_rate

        target_forward = raw_forward + derivative_term

        # Clamp before smoothing
        target_forward = max(-config.max_backward, min(config.max_forward, target_forward))

        # EMA smoothing
        alpha = config.forward_alpha
        self._smoothed_forward = alpha * target_forward + (1.0 - alpha) * self._smoothed_forward

        return self._smoothed_forward

    def reset(self):
        self._smoothed_forward = 0.0
        self._prev_bbox_h = None
        self._prev_time = None
        self._bbox_h_rate = 0.0
        _fwd_dead_zone.reset()


def compute_velocity_command(
    detection: Optional[Detection],
    config: ControllerConfig,
    target_bbox_height_override: Optional[float] = None,
    last_detection: Optional[Detection] = None,
    search_active: bool = True,
    hold_velocity: Optional[VelocityCommand] = None,
) -> VelocityCommand:
    """Compute a velocity command from the current detection and config.

    Returns a pure VelocityCommand (no MAVSDK types).
    """
    target_bh = target_bbox_height_override if target_bbox_height_override is not None else config.target_bbox_height

    # --- Search mode: no current detection ---
    if detection is None:
        if not search_active:
            return hold_velocity if hold_velocity is not None else VelocityCommand(0.0, 0.0, 0.0, 0.0)
        # Derive search direction from last seen position.
        search_direction = 1.0
        if last_detection is not None:
            search_direction = 1.0 if last_detection.center_x > 0.5 else -1.0
        # Spin toward last seen direction with damped forward correction
        search_forward = 0.0
        if last_detection is not None:
            raw = _calculate_forward_speed(last_detection, config, target_bh)
            search_forward = raw * config.search_vel_damp
            search_forward = max(search_forward, 0)
        return VelocityCommand(search_forward, 0.0, 0.0, search_direction * config.search_yawspeed_slow)

    # --- Tracking mode ---
    error_x_deg = (detection.center_x - 0.5) * config.hfov
    error_y_deg = (detection.center_y - 0.5) * config.vfov

    # Yaw: signed square-root response
    if abs(error_x_deg) < config.dead_zone_deg:
        yawspeed = 0.0
    else:
        yawspeed = math.copysign(config.kp_yaw * math.sqrt(abs(error_x_deg)), error_x_deg)
    yawspeed = max(-config.max_yawspeed, min(config.max_yawspeed, yawspeed))

    # Altitude
    down = 0.0
    if not config.fixed_altitude and not config.yaw_only:
        down = 0.0 if abs(error_y_deg) < config.dead_zone_deg else config.kp_down * error_y_deg
        down = max(-config.max_down_speed, min(config.max_down_speed, down))

    forward = _calculate_forward_speed(detection, config, target_bh)

    right = config.orbit_speed_m_s * config.orbit_direction if config.follow_mode == "orbit" else 0.0
    return VelocityCommand(forward, right, down, yawspeed)


def _effective_target_bbox_height(
    config: ControllerConfig,
    current_altitude_m: float,
    min_altitude_m: float = 0.5,
    max_target: float = 0.9,
) -> float:
    """Compute effective target bbox height for the current altitude.

    If target_distance_m is set, use perspective geometry to derive bbox height
    from altitude + horizontal distance. Otherwise, scale target_bbox_height
    inversely with altitude relative to reference_altitude_m.
    """
    alt = max(current_altitude_m, min_altitude_m)
    if config.target_distance_m is not None and config.target_distance_m > 0:
        return min(_distance_to_bbox_height(
            alt, config.target_distance_m, config.vfov, config.person_height_m,
        ), max_target)
    effective = (config.reference_altitude_m * config.target_bbox_height) / alt
    return min(effective, max_target)
