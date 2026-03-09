"""Controller configuration — pure dataclass, no third-party dependencies."""

import argparse
from dataclasses import dataclass
from typing import Optional


@dataclass
class ControllerConfig:
    hfov: float = 66.0
    vfov: float = 41.0
    # kp = proportional gain (P term of a PID controller).
    # Higher kp → faster response but more overshoot.
    kp_yaw: float = 5
    dead_zone_deg: float = 2.0
    max_yawspeed: float = 90.0
    kp_down: float = 0.08
    max_down_speed: float = 1.5
    target_bbox_height: float = 0.3
    target_distance_m: Optional[float] = None  # desired horizontal distance; overrides target_bbox_height when set
    person_height_m: float = 1.7               # assumed person height for distance calculation
    kp_forward: float = 3.0
    kp_backward: float = 5.0
    dead_zone_height_percent: float = 5.0  # dead zone as % of target_bbox_height (default 5%)
    max_forward: float = 2.0
    max_backward: float = 3.0
    detection_timeout_s: float = 0.5
    search_enter_delay_s: float = 2.0
    search_timeout_s: float = 60.0
    control_loop_hz: float = 10.0
    fixed_altitude: bool = True
    max_bbox_height_safety: float = 0.8  # Safety limit: if bbox height > 0.8, we are too close
    yaw_only: bool = True
    reference_altitude_m: float = 3.0  # target_bbox_height is defined at this altitude; scales by (ref_alt/current_alt)
    # Bottom-of-frame backward: bbox bottom edge beyond this triggers backward
    bottom_y_threshold: float = 0.7
    # Search mode
    search_yawspeed_slow: float = 10.0  # yaw speed during search (slower than tracking)
    search_vel_damp: float = 0.3        # dampening factor for forward/backward speed during search
    min_search_forward: float = 0.2     # minimum forward speed in search when last bbox was too small
    # Yaw smoothing
    smooth_yaw: bool = True             # enable low-pass smoothing on yaw command
    yaw_alpha: float = 0.3              # yaw EMA factor (0=very smooth, 1=no smoothing)
    # Forward smoothing: estimate person velocity and smooth commands
    smooth_forward: bool = True         # enable forward velocity smoothing
    forward_alpha: float = 0.1          # EMA smoothing factor (0=ignore new, 1=no smoothing)
    kd_forward: float = 2.0            # derivative gain: anticipate person movement

    follow_mode: str = "follow"       # "follow" or "orbit"
    orbit_speed_m_s: float = 1.0      # lateral velocity for orbit (m/s)
    orbit_direction: int = 1          # +1 = clockwise, -1 = counter-clockwise
    max_orbit_speed: float = 3.0      # max lateral speed limit

    target_altitude: float = 3.0
    log_verbosity: str = "normal"  # quiet | normal | debug

    def __post_init__(self):
        self.validate()

    def validate(self):
        """Raise ValueError if the configuration is internally inconsistent."""
        if self.target_distance_m is not None and not self.fixed_altitude:
            raise ValueError(
                "target_distance_m requires fixed_altitude=True; "
                "with variable altitude, use target_bbox_height instead"
            )

    @staticmethod
    def add_args(parser: argparse.ArgumentParser) -> None:
        """Register controller-related CLI flags on *parser*."""
        defaults = ControllerConfig()
        group = parser.add_argument_group("follow-controller")

        # Framing and target geometry
        group.add_argument("--hfov", type=float, default=defaults.hfov)
        group.add_argument("--vfov", type=float, default=defaults.vfov)
        group.add_argument("--target-bbox-height", type=float, default=None,
                           help=f"Target bbox height (0-1). Mutually exclusive with --target-distance. "
                                f"(default when no --target-distance: {defaults.target_bbox_height})")
        group.add_argument("--target-distance", type=float, default=None, metavar="M",
                           help="Desired horizontal distance to person in metres. Requires --fixed-altitude. "
                                "Mutually exclusive with --target-bbox-height. "
                                "(default: None, use target-bbox-height)")
        group.add_argument("--person-height", type=float, default=defaults.person_height_m, metavar="M",
                           help=f"Assumed person height for distance calculation (default: {defaults.person_height_m}m)")

        # Controller gains and loop behavior
        group.add_argument("--control-loop-hz", type=float, default=defaults.control_loop_hz)
        group.add_argument("--dead-zone-height-percent", type=float, default=defaults.dead_zone_height_percent,
                           help="Forward dead zone as %% of target bbox height (default: 5)")
        group.add_argument("--yaw-gain", dest="kp_yaw", type=float, default=defaults.kp_yaw)
        group.add_argument("--forward-gain", dest="kp_forward", type=float, default=defaults.kp_forward)
        group.add_argument("--backward-gain", dest="kp_backward", type=float, default=defaults.kp_backward,
                           help="Gain for backward movement when too close (default: 5.0)")
        group.add_argument("--pitch-gain", dest="kp_down", type=float, default=defaults.kp_down)

        # Flight mode
        group.add_argument("--fixed-altitude", action=argparse.BooleanOptionalAction, default=defaults.fixed_altitude,
                           help="Keep altitude fixed (default: True). Use --no-fixed-altitude for vertical following.")
        group.add_argument("--yaw-only", action=argparse.BooleanOptionalAction, default=defaults.yaw_only,
                           help="Yaw only mode: no forward/backward or altitude movement (default: True). Use --no-yaw-only for full follow.")

        # Search/follow behavior
        group.add_argument("--search-enter-delay", type=float, default=defaults.search_enter_delay_s,
                           help="Seconds without detection before active search starts (default: 2.0)")
        group.add_argument("--search-vel-damp", type=float, default=defaults.search_vel_damp,
                           help="Dampening factor for forward/backward speed during search based on last detection (default: 0.3)")
        group.add_argument("--search-timeout", type=float, default=defaults.search_timeout_s,
                           help="Seconds before landing if no person is found (default: 60.0)")

        # Smoothing
        group.add_argument("--smooth-forward", action=argparse.BooleanOptionalAction, default=defaults.smooth_forward,
                           help=f"Enable/disable forward velocity smoothing (default: {defaults.smooth_forward})")
        group.add_argument("--forward-alpha", type=float, default=defaults.forward_alpha,
                           help=f"EMA smoothing factor for forward velocity (0=sluggish, 1=no smoothing, default: {defaults.forward_alpha})")

        # Safety limits
        group.add_argument("--max-forward", type=float, default=defaults.max_forward,
                           help=f"Max forward speed in m/s (default: {defaults.max_forward})")
        group.add_argument("--max-backward", type=float, default=defaults.max_backward,
                           help=f"Max backward speed in m/s (default: {defaults.max_backward})")
        group.add_argument("--max-bbox-height-safety", type=float, default=defaults.max_bbox_height_safety,
                           help="Safety limit: stop/retreat if bbox height > limit (0.0-1.0) (default: 0.8)")

        # Orbit mode
        group.add_argument("--follow-mode", choices=["follow", "orbit"], default=defaults.follow_mode,
                           help="Follow mode: 'follow' (default) or 'orbit' (circle around target)")
        group.add_argument("--orbit-speed", type=float, default=defaults.orbit_speed_m_s,
                           help=f"Lateral velocity for orbit mode in m/s (default: {defaults.orbit_speed_m_s})")
        group.add_argument("--orbit-direction", type=int, choices=[1, -1], default=defaults.orbit_direction,
                           help="Orbit direction: 1=clockwise (default), -1=counter-clockwise")

        # Logging
        group.add_argument("--log-verbosity", choices=["quiet", "normal", "debug"], default=defaults.log_verbosity,
                           help="Console log verbosity (default: normal)")

    @classmethod
    def from_args(cls, args):
        # Single source of defaults: dataclass values.
        defaults = cls()

        def _arg(*names, default):
            for name in names:
                value = getattr(args, name, None)
                if value is not None:
                    return value
            return default

        # yaw_only: only True when user explicitly passed --yaw-only.
        yaw_only = _arg("yaw_only", default=defaults.yaw_only)
        if not isinstance(yaw_only, bool):
            yaw_only = bool(yaw_only)

        ref_alt = _arg("reference_altitude", "reference_altitude_m", default=defaults.reference_altitude_m)
        ref_alt = ref_alt if ref_alt and ref_alt > 0 else defaults.reference_altitude_m

        # --target-distance and --target-bbox-height are mutually exclusive.
        # Argparse defaults are None so we can detect user-explicit values.
        target_distance = getattr(args, "target_distance", None)
        target_bbox_height = getattr(args, "target_bbox_height", None)
        if target_distance is not None and target_bbox_height is not None:
            raise ValueError(
                "--target-distance and --target-bbox-height are mutually exclusive"
            )

        # --target-distance requires --fixed-altitude; reject invalid combination.
        fixed_alt = _arg("fixed_altitude", default=defaults.fixed_altitude)
        if target_distance is not None and not fixed_alt:
            raise ValueError(
                "--target-distance requires --fixed-altitude; "
                "use --fixed-altitude when setting a target distance, or omit --target-distance for using target-bbox-height parameter."
            )

        return cls(
            hfov=_arg("hfov", default=defaults.hfov),
            vfov=_arg("vfov", default=defaults.vfov),
            kp_yaw=_arg("kp_yaw", "yaw_gain", default=defaults.kp_yaw),
            kp_down=_arg("kp_down", "pitch_gain", default=defaults.kp_down),
            kp_forward=float(_arg("kp_forward", "forward_gain", default=defaults.kp_forward)),
            kp_backward=_arg("kp_backward", "backward_gain", default=defaults.kp_backward),
            target_bbox_height=_arg("target_bbox_height", default=defaults.target_bbox_height),
            target_distance_m=target_distance,
            person_height_m=_arg("person_height", "person_height_m", default=defaults.person_height_m),
            dead_zone_height_percent=_arg("dead_zone_height_percent", default=defaults.dead_zone_height_percent),
            reference_altitude_m=ref_alt,
            fixed_altitude=fixed_alt,
            yaw_only=yaw_only,
            detection_timeout_s=_arg("detection_timeout", "detection_timeout_s", default=defaults.detection_timeout_s),
            search_enter_delay_s=_arg("search_enter_delay", "search_enter_delay_s", default=defaults.search_enter_delay_s),
            control_loop_hz=_arg("control_loop_hz", default=defaults.control_loop_hz),
            max_forward=_arg("max_forward", default=defaults.max_forward),
            max_backward=_arg("max_backward", default=defaults.max_backward),
            max_bbox_height_safety=_arg("max_bbox_height_safety", default=defaults.max_bbox_height_safety),
            search_timeout_s=_arg("search_timeout", "search_timeout_s", default=defaults.search_timeout_s),
            search_vel_damp=_arg("search_vel_damp", default=defaults.search_vel_damp),
            smooth_yaw=_arg("smooth_yaw", default=defaults.smooth_yaw),
            yaw_alpha=_arg("yaw_alpha", default=defaults.yaw_alpha),
            smooth_forward=_arg("smooth_forward", default=defaults.smooth_forward),
            forward_alpha=_arg("forward_alpha", default=defaults.forward_alpha),
            kd_forward=_arg("kd_forward", default=defaults.kd_forward),
            follow_mode=_arg("follow_mode", default=defaults.follow_mode),
            orbit_speed_m_s=_arg("orbit_speed", "orbit_speed_m_s", default=defaults.orbit_speed_m_s),
            orbit_direction=_arg("orbit_direction", default=defaults.orbit_direction),
            max_orbit_speed=_arg("max_orbit_speed", default=defaults.max_orbit_speed),
            target_altitude=_arg("target_altitude", default=defaults.target_altitude),
            log_verbosity=_arg("log_verbosity", default=defaults.log_verbosity),
        )
