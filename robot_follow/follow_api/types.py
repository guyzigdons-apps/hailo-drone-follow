"""Pure domain types for robot follow — no third-party dependencies.

All actuator-boundary types live here so follow_api stays the
pure-leaf core. robot_api imports from here; never the other way.
Per Phase 3 R1 (adversarial review 2026-05-17).

RobotCommand is the controller's emit type (see controller.compute).
The legacy VelocityCommand was deleted in Phase 3 plan 03-07.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Literal, Optional


class Axis(Enum):
    """Body-frame velocity axes a Robot may support.

    Drone-shaped scope (velocity-control-in-body-frame, up to 3 axes).
    Pan-tilt, holonomic-rover (LATERAL), submarine (ROLL/PITCH), and
    robot-arm (position-control) are NOT modeled. See CONTEXT.md
    § domain scope note. Rename Robot → BodyVelocityRobot if v1.2
    surfaces a non-velocity actuator.
    """
    FORWARD = "forward"    # body x, m/s
    YAW = "yaw"            # body z rotation
    ALTITUDE = "altitude"  # body z linear, m/s (down positive per MAVSDK convention)


@dataclass(frozen=True)
class Capabilities:
    """Mechanical-only description of a Robot's actuator surface.

    Axes-only by design (decision 2026-05-14, ratified R1). NO
    behavioral policy flags — no `bottom_edge_policy`, no
    `yaw_spin_on_loss`. Robot-specific behaviors live INSIDE the
    adapter, never gated by a Capabilities flag.

    - `axes`: which body-frame velocity axes this robot actuates.
    - `yaw_unit`: unit the controller emits `RobotCommand.yaw_rate` in.
      Drone: "deg/s" (MAVSDK VelocityBodyYawspeed convention).
      Rover: "rad/s" (geometry_msgs/Twist convention).
      Adapter does NO unit conversion (Q5 lock from CONTEXT 2026-05-17).
    """
    axes: frozenset[Axis]
    yaw_unit: Literal["deg/s", "rad/s"]


@dataclass
class RobotCommand:
    """Per-tick actuator command emitted by controller, consumed by adapter.

    The controller writes ONLY the channels whose axis is in
    `caps.axes`; the adapter reads ONLY the channels in `caps.axes`.
    Replaces VelocityCommand in Phase 3 plan 03-07.

    - `forward_m_s`: body x velocity in m/s. Positive = forward.
    - `yaw_rate`: rotation about body z in `caps.yaw_unit`.
      Drone path: deg/s. Rover path: rad/s. NO adapter-side
      conversion (Q5 lock).
    - `down_m_s`: body z linear velocity in m/s, down positive
      (MAVSDK convention). Only written when Axis.ALTITUDE in caps.axes.
    """
    forward_m_s: float = 0.0
    yaw_rate: float = 0.0
    down_m_s: float = 0.0


@dataclass
class Detection:
    """A single person detection in normalized image coordinates."""
    label: str
    confidence: float
    center_x: float      # 0.0 to 1.0
    center_y: float      # 0.0 to 1.0
    bbox_height: float   # 0.0 to 1.0
    timestamp: float


@dataclass(frozen=True)
class SafetyContext:
    """Minimal struct the controller derives from Detection and passes
    to the adapter alongside RobotCommand.

    Decouples adapter from Detection-shape changes (v1.2 may add
    depth, multi-camera, multi-target). Per Phase 3 R4 (adversarial
    review 2026-05-17).

    Q6 lock (CONTEXT 2026-05-17): when `target_lost == True`, the
    adapter MUST early-return before reading any other field.
    The bbox/x values for the lost case are intentionally
    unspecified — adapters that respect the flag never see them.

    Fields:
    - `bbox_bottom_normalized`: cy + bh/2, for bottom-margin checks (0..1).
    - `bbox_size_normalized`: bbox_height, for size-based safety (0..1).
    - `target_lost`: True only when constructed via .lost().
    - `last_target_x`: for on_target_lost search direction hint.
    - `bbox_bottom_norm`: Optional[float]; rover-specific bottom-edge
      slow-down threshold input (Plan 06-04). Populated in
      from_detection from cy + bh/2; left None in lost() per Q6 lock.
      Optional (default None) so any caller that doesn't populate it
      remains backward-compatible. Coexists with bbox_bottom_normalized
      during migration — drone path reads the legacy field; rover
      adapter (06-04) will read this new one.
    """
    bbox_bottom_normalized: float
    bbox_size_normalized: float
    target_lost: bool
    last_target_x: Optional[float]
    # RINT-02 / Q1 lock (Phase 6 CONTEXT, 2026-05-20): the rover adapter
    # reads this and overrides forward_m_s=0.0 when >=0.85. Drone adapter
    # ignores this field; its retreat-from-tilt continues to read
    # bbox_bottom_normalized.
    bbox_bottom_norm: Optional[float] = None

    @classmethod
    def from_detection(cls, det: Detection) -> "SafetyContext":
        """Construct from a current real detection. target_lost=False."""
        return cls(
            bbox_bottom_normalized=det.center_y + det.bbox_height / 2,
            bbox_size_normalized=det.bbox_height,
            target_lost=False,
            last_target_x=det.center_x,
            bbox_bottom_norm=det.center_y + det.bbox_height / 2,
        )

    @classmethod
    def lost(cls, last_target_x: Optional[float] = None) -> "SafetyContext":
        """Construct for the orchestrator hold/lost path. target_lost=True.

        bbox fields are placeholder (0.5, 0.25) — center of frame,
        neutral bbox. Adapter MUST short-circuit on target_lost before
        reading them. Sentinel choice rationale: RESEARCH § SafetyContext
        derivation (lines 911-928); values are "outside any edge zone"
        so a misbehaving adapter that ignores target_lost won't trigger
        spurious retreat-from-tilt.

        `bbox_bottom_norm` is left at its None default per Q6 lock — the
        adapter must early-return on target_lost; the None sentinel is
        belt-and-braces so a misbehaving adapter that ignores
        target_lost will not see a "person at bottom edge" value.
        """
        return cls(
            bbox_bottom_normalized=0.5,
            bbox_size_normalized=0.25,
            target_lost=True,
            last_target_x=last_target_x,
        )
