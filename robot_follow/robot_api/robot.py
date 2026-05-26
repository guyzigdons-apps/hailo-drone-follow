"""Robot protocol — the unified actuator interface.

Types (Axis, Capabilities, RobotCommand, SafetyContext) live in
follow_api/types.py per Phase 3 R1; this module only defines the
Protocol class. Adapters (drone, rover) implement this interface.

Method semantics (locked from CONTEXT 2026-05-17):
- connect:        raise ConnectionError on hard failure. No retry in v1.1.
- start_session:  may block up to config.start_session_timeout_s
                   (default 30 s) waiting for offboard / rclpy init.
                   Honors asyncio.CancelledError for shutdown.
- send_command:   per-tick actuator call. Adapter applies its
                   robot-specific behaviors (drone: altitude P +
                   retreat-from-tilt + smoothing; rover: Twist publish).
                   MUST early-return if safety_ctx.target_lost is True
                   (Q6 lock).
- send_zero:      true actuator quiescent. Called once on shutdown.
                   NO last_detection arg (split from on_target_lost
                   per R2).
- on_target_lost: per-tick lost-target reaction. Drone yaw-spins;
                   rover sends Twist(0,0,0); pan-tilt holds.
- shutdown:       idempotent; always called in orchestrator finally.

The caps attribute (Capabilities) is launch-time fixed (no runtime
degradation path in v1.1).
"""

from typing import Optional, Protocol, runtime_checkable

from robot_follow.follow_api.types import (
    Capabilities,
    Detection,
    RobotCommand,
    SafetyContext,
)


@runtime_checkable
class Robot(Protocol):
    """Actuator-boundary protocol. Any object exposing these methods
    plus the caps attribute is structurally a Robot."""

    caps: Capabilities

    async def connect(self) -> None:
        """Open the wire (MAVSDK gRPC for drone, rclpy.init for rover).

        Raises ConnectionError on hard failure (TCP refused, ROS
        not sourced, etc.). Orchestrator catches and exits cleanly.
        """
        ...

    async def start_session(self) -> None:
        """Activate the actuator (offboard handshake for drone;
        node + publisher creation for rover).

        May block up to config.start_session_timeout_s for the drone's
        offboard handshake. Rover's implementation is sub-second.
        Honors asyncio.CancelledError if shutdown is signaled.
        """
        ...

    async def send_command(
        self,
        cmd: RobotCommand,
        safety_ctx: SafetyContext,
    ) -> None:
        """Forward a per-tick command to the wire.

        Adapter overlays its robot-specific behaviors here (drone:
        altitude P, retreat-from-tilt, smoothing; rover: optional
        bottom-edge slow). MUST early-return without wire-side
        effect if safety_ctx.target_lost is True (Q6 lock).

        yaw_rate units: cmd.yaw_rate is in self.caps.yaw_unit.
        Drone: deg/s. Rover: rad/s. NO adapter-side conversion
        (Q5 lock).
        """
        ...

    async def send_zero(self) -> None:
        """Send a quiescent zero setpoint.

        Called once in the orchestrator's finally block on shutdown.
        No last_detection arg — that's on_target_lost's job.
        """
        ...

    async def on_target_lost(
        self,
        last_detection: Optional[Detection],
    ) -> None:
        """Adapter's per-tick lost-target reaction.

        Called by the orchestrator each tick after the
        search_enter_delay has elapsed and no current detection
        exists. Drone: yaw-spin in last bbox direction.
        Rover: Twist(0,0,0). Pan-tilt: hold last pose.
        """
        ...

    async def shutdown(self) -> None:
        """Tear down all adapter-side resources.

        Idempotent (always called in orchestrator finally, even
        if connect/start_session failed mid-way). Drone: cancel
        telemetry tasks, land if armed, exit DetachedMavsdkServer.
        Rover: stop spin thread, destroy node, try_shutdown.
        Errors are logged, not raised.
        """
        ...
