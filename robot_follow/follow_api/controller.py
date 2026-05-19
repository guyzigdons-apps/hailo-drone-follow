"""Pure follow controller — emit a RobotCommand from a Detection.

Only depends on standard library + the types/config from this package.

The controller writes ONLY the channels in caps.axes:
- forward_m_s: emitted only when Axis.FORWARD in caps.axes
- yaw_rate:    emitted in caps.yaw_unit (drone deg/s, rover rad/s)
- down_m_s:    ALWAYS 0.0 — altitude P correction is the adapter's job
               (see _apply_altitude_p in robot_api/adapters/mavsdk_drone.py).
               Today's pre-rewrite compute_velocity_command also returned
               down=0; live_control_loop applied altitude P externally.

Emergency safety: when bbox_height > max_bbox_height_safety AND
Axis.FORWARD in caps.axes, emit forward_m_s=-max_backward with
centering-yaw recovery (yaw stays active during the safety branch).

Frame-edge fade + safety-push gradient lives in the adapter
(_apply_retreat_from_tilt in robot_api/adapters/mavsdk_drone.py).
The controller only emits the raw clamped distance-P output; the
adapter applies the gradient and then the forward-velocity deadband
(post-fade, pre-smoothing, matching the legacy order). This keeps the
controller robot-agnostic (rover doesn't need tilt-induced retreat).

Per CONTEXT § Controller signature migration. Replaces the legacy
compute_velocity_command (last_detection / search_active / hold_velocity
concerns are now the orchestrator's responsibility).
"""

import math

from .types import Axis, Capabilities, Detection, RobotCommand
from .config import ControllerConfig

__all__ = [
    "compute",
]


def _compute_yaw(detection: Detection, config: ControllerConfig) -> float:
    """Signed square-root yaw P controller (horizontal centering).

    Math byte-identical to the legacy compute_velocity_command yaw block.
    """
    error_x_deg = (detection.center_x - 0.5) * config.hfov
    if abs(error_x_deg) < config.dead_zone_deg:
        return 0.0
    yawspeed = math.copysign(
        config.kp_yaw * math.sqrt(abs(error_x_deg)), error_x_deg
    )
    return max(-config.max_yawspeed, min(config.max_yawspeed, yawspeed))


def _compute_forward(detection: Detection, config: ControllerConfig) -> float:
    """Distance-error P controller → forward command.

    Math byte-identical to the legacy _calculate_distance_speed. Uses
    ``(target_bbox / bbox) - 1`` as the proportional distance error
    (scale-invariant; bbox ∝ 1/distance). Asymmetric retreat: factor<0
    uses kp_distance_back so retreat saturates max_backward before bbox
    reaches max_bbox_height_safety (the binary panic threshold).
    """
    if config.yaw_only:
        return 0.0
    if detection.bbox_height <= 0:
        return 0.0
    factor = (config.target_bbox_height / detection.bbox_height) - 1.0
    dead_zone = config.dead_zone_bbox_percent / 100.0
    if abs(factor) < dead_zone:
        return 0.0
    gain = config.kp_distance_back if factor < 0 else config.kp_distance
    if gain == 0:
        return 0.0
    raw = gain * factor
    return max(-config.max_backward, min(config.max_forward, raw))


def compute(
    detection: Detection,
    caps: Capabilities,
    config: ControllerConfig,
) -> RobotCommand:
    """Pure controller — emit a RobotCommand from a Detection.

    See module docstring for axis-emission rules and emergency-safety
    behavior. ``detection`` is NEVER None — the orchestrator handles
    target-lost via Robot.on_target_lost.

    Channels not in caps.axes are emitted as 0.0; the adapter ignores
    them anyway, but defaulting to zero keeps the wire setpoint quiet
    in case of misuse.
    """
    yaw_rate = _compute_yaw(detection, config)

    forward_m_s = 0.0
    if Axis.FORWARD in caps.axes:
        # Emergency safety branch (bbox too large) — overrides everything
        # except yaw. The yaw output above already centers; just emit
        # max-backward forward. Matches the legacy controller's behavior
        # (which returned VelocityCommand(-max_backward, 0, yawspeed) here).
        if (not config.yaw_only
                and detection.bbox_height > config.max_bbox_height_safety):
            forward_m_s = -config.max_backward
        else:
            # Distance-P (clamped to ±max_forward / -max_backward inside
            # _compute_forward). Frame-edge fade + safety-push gradient
            # and the forward-velocity deadband live in the adapter
            # (_apply_retreat_from_tilt + post-fade deadband) — the
            # controller stays robot-agnostic.
            forward_m_s = _compute_forward(detection, config)

    # Down: ALWAYS 0.0 — altitude P lives in the adapter
    # (see _apply_altitude_p). M4 invariant: baseline-captured down_m_s
    # is 0.0 for all 100 BaselineCases.
    down_m_s = 0.0

    return RobotCommand(
        forward_m_s=forward_m_s,
        yaw_rate=yaw_rate,
        down_m_s=down_m_s,
    )
