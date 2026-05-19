"""robot_api.orchestrator — generic per-tick control loop.

Runs the shared loop for ALL robots (drone, rover). Reads from
shared_state, calls controller.compute, dispatches to
robot.send_command / robot.on_target_lost based on the state
machine documented in CONTEXT § Orchestrator state machine.

Body lands in Phase 3 plan 03-08 (depends on 03-06's
MavsdkDroneAdapter being instantiable + 03-07's compute() signature).
Until then, run_robot_loop raises NotImplementedError so any
accidental caller fails loudly.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot_follow.follow_api.config import ControllerConfig
    from robot_follow.follow_api.state import SharedDetectionState
    from robot_follow.robot_api.robot import Robot


async def run_robot_loop(
    robot: "Robot",
    shared_state: "SharedDetectionState",
    config: "ControllerConfig",
    shutdown: asyncio.Event,
) -> None:
    """Generic per-tick robot control loop.

    Lifecycle:
        await robot.connect()
        await robot.start_session()
        while not shutdown.is_set():
            tick: read detection → compute cmd → send_command
                                (or on_target_lost after delay)
        finally: send_zero + shutdown

    Full pseudocode in CONTEXT lines 68-105.
    Body deferred to Phase 3 plan 03-08; this signature locks
    the contract so callers compile against it now.
    """
    raise NotImplementedError(
        "run_robot_loop body lands in 03-08-PLAN; "
        "this is a Wave-3 scaffold (03-04-PLAN)"
    )
