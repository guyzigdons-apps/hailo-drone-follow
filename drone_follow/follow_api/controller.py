"""Pure follow controller logic — no MAVSDK, no Hailo, no GStreamer dependencies.

Only depends on standard library + the types/config from this package.
"""

import math
from typing import Optional

from .types import Detection, VelocityCommand
from .config import ControllerConfig

__all__ = [
    "compute_velocity_command",
    "ForwardSmoother",
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


def _calculate_forward_speed(
    detection: Detection,
    config: ControllerConfig,
    target_bh: float,
) -> float:
    """Plain P controller on bbox-height error, with a dead zone and safety bypasses.

    Pitch-induced oscillation (rigid camera → body pitch tilts image → false
    error signal ~1-2 Hz) is suppressed downstream by the low-pass EMA in
    ForwardSmoother, not here.  Keep this function stateless.
    """
    if config.yaw_only or config.kp_forward == 0:
        return 0.0

    # Safety bypass 1: bbox too large → full reverse.
    if detection.bbox_height > config.max_bbox_height_safety:
        return -config.max_backward

    # Safety bypass 2: person directly under the drone → retreat.
    bbox_bottom = detection.center_y + detection.bbox_height / 2.0
    if bbox_bottom > config.bottom_y_threshold:
        overshoot = bbox_bottom - config.bottom_y_threshold
        return -config.kp_backward * math.sqrt(overshoot)

    # Plain P with a single dead-zone threshold.
    height_delta = target_bh - detection.bbox_height
    dead_zone = (config.dead_zone_height_percent / 100.0) * target_bh
    if abs(height_delta) < dead_zone:
        return 0.0

    gain = config.kp_forward if height_delta > 0 else config.kp_backward
    raw = gain * height_delta  # sign preserved: +ve delta = approach, -ve = retreat
    return max(-config.max_backward, min(config.max_forward, raw))


class ForwardSmoother:
    """First-order low-pass (EMA) on the forward command.

    The pitch-induced oscillation sits at roughly 1-2 Hz.  With a 10 Hz loop
    and alpha ~0.07, the cutoff is ~0.11 Hz, which attenuates the oscillation
    by ~25 dB while leaving the sub-0.1 Hz tracking bandwidth intact.
    """

    def __init__(self):
        self._smoothed_forward: float = 0.0

    def update(self, detection: Optional[Detection], raw_forward: float,
               config: ControllerConfig) -> float:
        """Return smoothed forward velocity (EMA).  `detection` is accepted for
        API compatibility with the previous rate-aware smoother but is unused."""
        del detection  # no longer used; kept for signature stability
        target = max(-config.max_backward, min(config.max_forward, raw_forward))
        alpha = config.forward_alpha
        self._smoothed_forward = alpha * target + (1.0 - alpha) * self._smoothed_forward
        return self._smoothed_forward

    def reset(self):
        self._smoothed_forward = 0.0


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
