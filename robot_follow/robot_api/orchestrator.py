"""robot_api.orchestrator — generic per-tick control loop.

Runs the shared loop for ALL robots (drone, rover). Reads from
shared_state, calls controller.compute, dispatches to
robot.send_command / robot.on_target_lost based on the state
machine documented in CONTEXT § Orchestrator state machine.

The body landed in Phase 3 plan 03-07 alongside the controller-signature
migration. Uses ``shared_state.get_latest()`` (M2 lock) — NOT
``shared_state.current_detection()`` (does not exist).

Mission-duration deadline (B1): NOT honored here. The orchestrator is
robot-agnostic and has no ``args`` in scope. The composition root
(robot_follow_app.run_drone) wraps this loop in
``asyncio.wait([loop, asyncio.sleep(args.mission_duration)], return_when=
FIRST_COMPLETED)`` so the watchdog stays a drone-side concern.
"""

from __future__ import annotations

import asyncio
import math
import time
from typing import TYPE_CHECKING, Optional, Any

from robot_follow.follow_api import controller
from robot_follow.follow_api.types import RobotCommand, SafetyContext

if TYPE_CHECKING:
    from robot_follow.follow_api.config import ControllerConfig
    from robot_follow.follow_api.state import SharedDetectionState
    from robot_follow.robot_api.robot import Robot


async def run_robot_loop(
    robot: "Robot",
    shared_state: "SharedDetectionState",
    config: "ControllerConfig",
    shutdown: asyncio.Event,
    ui_state: Optional[Any] = None,
) -> None:
    """Generic per-tick robot control loop.

    Lifecycle:
        await robot.connect()
        await robot.start_session()
        while not shutdown.is_set():
            tick: read detection → compute cmd → send_command
                                (or on_target_lost after search_enter_delay_s)
        finally: send_zero + shutdown

    State machine (CONTEXT lines 68-105, adjusted for actual get_latest API):
      - detection present → compute(detection, caps, config) → send_command
      - detection absent, within search_enter_delay_s → re-send last cmd
        (orchestrator-side hold, gated by SafetyContext.from_detection or .lost())
      - detection absent, past delay → on_target_lost (adapter-specific)

    ui_state (optional duck-typed):
        Any object that implements ``update_velocity(forward_m_s, down_m_s,
        yawspeed_deg_s, mode)`` — e.g. SharedUIState from web_server.py.
        Pass None (default) to suppress UI publishing (rover sim, unit tests
        without UI wiring).

    Published values reflect the controller's PRE-smoothing ``cmd``.
    Rationale: The drone adapter's ``_apply_smoothing`` is a drone-specific
    EMA + slew cap — publishing post-smoothing would (a) require ui_state
    inside the adapter, violating axes-only Capabilities (see CONTEXT
    D-axes-only contract / feedback_robot_abstraction_axes_only.md),
    (b) lag one tick behind the controller's actual command, (c) be
    undefined for rover (no smoother). Pre-Phase-3 behavior used
    pre-smoothing values from live_control_loop — this restores parity
    (gap closure 03-11, ABS-11).

    Mode strings are robot-agnostic (AUTO / LOCKED / LOST / IDLE) so the
    future Ros2RoverAdapter gets UI telemetry for free without any
    adapter-side changes.
    """
    await robot.connect()
    await robot.start_session()

    last_seen_t = None
    last_cmd = RobotCommand()
    last_detection = None
    tick_dt = 1.0 / config.control_loop_hz

    try:
        while not shutdown.is_set():
            tick_start = time.monotonic()
            # M2 lock: shared_state.get_latest() returns (detection, frame_count).
            # The pseudocode in CONTEXT used current_detection() which does NOT
            # exist on SharedDetectionState. The plan adapts pseudocode to reality.
            detection, _frame = shared_state.get_latest()
            if detection is not None:
                safety_ctx = SafetyContext.from_detection(detection)
                cmd = controller.compute(detection, robot.caps, config)
                await robot.send_command(cmd, safety_ctx)
                last_seen_t = time.monotonic()
                last_cmd = cmd
                last_detection = detection
                # Publish pre-smoothing cmd to UI. Mode="AUTO" here; LOCKED/AUTO
                # distinction via FollowTargetState is not wired in this plan
                # (run_robot_loop has no access to FollowTargetState) — deferred
                # to a future plan. The four allowed mode strings are:
                # AUTO, LOCKED, LOST, IDLE (symmetric for drone + rover).
                if ui_state is not None:
                    ui_state.update_velocity(cmd.forward_m_s, cmd.down_m_s, cmd.yaw_rate, "AUTO")
            else:
                lost_for = (time.monotonic() - last_seen_t) if last_seen_t else math.inf
                if lost_for < config.search_enter_delay_s:
                    safety_ctx = (
                        SafetyContext.from_detection(last_detection)
                        if last_detection is not None
                        else SafetyContext.lost()
                    )
                    await robot.send_command(last_cmd, safety_ctx)
                    # Hold phase: publish IDLE with zeros (hold is adapter-internal,
                    # no meaningful velocity to surface to the operator).
                    if ui_state is not None:
                        ui_state.update_velocity(0.0, 0.0, 0.0, "IDLE")
                else:
                    await robot.on_target_lost(last_detection)
                    # Lost phase: adapter executes its search behavior internally;
                    # yaw-spin is adapter-specific and NOT surfaced at the orchestrator.
                    if ui_state is not None:
                        ui_state.update_velocity(0.0, 0.0, 0.0, "LOST")
            elapsed = time.monotonic() - tick_start
            await asyncio.sleep(max(0.0, tick_dt - elapsed))
    finally:
        await robot.send_zero()
        # Publish clean shutdown indicator so the UI shows IDLE with zeros
        # rather than holding the last flying command after the loop exits.
        if ui_state is not None:
            ui_state.update_velocity(0.0, 0.0, 0.0, "IDLE")
        await robot.shutdown()
