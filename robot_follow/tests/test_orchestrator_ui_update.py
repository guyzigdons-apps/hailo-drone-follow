"""ABS-11 gap closure (03-11) — orchestrator publishes (forward, down, yawspeed, mode) to
ui_state after each tick branch and in shutdown finally.

Pins the pre-smoothing-cmd decision (gap 03-10-SUMMARY F1):
  The values published reflect the controller's PRE-smoothing ``cmd``. Publishing
  post-smoothed values would violate the axes-only Capabilities contract (ui_state
  cannot live inside the adapter) and would be undefined for the future rover adapter.

Five tests covering:
  1. Detection-present branch → "AUTO" + non-zero velocity from controller.compute
  2. Lost branch (past search_enter_delay_s) → "LOST" + zeros
  3. Shutdown finally block → "IDLE" + zeros as the LAST call
  4. ui_state=None safety → no AttributeError
  5. Pre-smoothing pin → spy forward matches controller.compute directly
"""
from __future__ import annotations

import asyncio
import time
from types import SimpleNamespace
from typing import List, Optional, Tuple

import pytest

from robot_follow.follow_api import controller
from robot_follow.follow_api.config import ControllerConfig
from robot_follow.follow_api.types import (
    Axis,
    Capabilities,
    Detection,
    RobotCommand,
    SafetyContext,
)
from robot_follow.robot_api.orchestrator import run_robot_loop

# ---------------------------------------------------------------------------
# Low-loop-hz config shared across all tests — keeps wall-clock times short.
# search_enter_delay_s=0.02 so test_publishes_lost_mode only needs ~5 ticks
# (at 200 Hz) to cross the threshold.
# ---------------------------------------------------------------------------
_TEST_CONFIG = ControllerConfig(
    control_loop_hz=200.0,
    search_enter_delay_s=0.02,
    yaw_only=True,           # simplifies forward=0 assertions for IDLE/LOST tests
)

_TEST_CONFIG_FORWARD = ControllerConfig(
    control_loop_hz=200.0,
    search_enter_delay_s=0.02,
    yaw_only=False,          # enables forward velocity so test 1 + 5 see non-zero values
    kp_yaw=10.0,             # strong yaw for center-offset detection to produce non-zero
    kp_distance=2.0,
    target_bbox_height=0.1,  # target smaller than detection → positive forward
    max_forward=2.0,
    max_backward=2.0,
)


# ---------------------------------------------------------------------------
# SpyUIState — records every update_velocity call
# ---------------------------------------------------------------------------

class SpyUIState:
    """Duck-typed ui_state spy. Records all update_velocity calls."""

    def __init__(self):
        self.calls: List[Tuple[float, float, float, str]] = []

    def update_velocity(
        self,
        forward_m_s: float,
        down_m_s: float,
        yawspeed_deg_s: float,
        mode: str,
    ) -> None:
        self.calls.append((float(forward_m_s), float(down_m_s), float(yawspeed_deg_s), str(mode)))


# ---------------------------------------------------------------------------
# FakeRobot — minimal Robot protocol surface consumed by run_robot_loop
# ---------------------------------------------------------------------------

_DRONE_CAPS = Capabilities(
    axes=frozenset({Axis.FORWARD, Axis.YAW, Axis.ALTITUDE}),
    yaw_unit="deg/s",
)


class FakeRobot:
    """Minimal Robot duck-type for orchestrator tests.

    Implements every async method used by run_robot_loop without any real
    hardware or MAVSDK dependency.
    """

    def __init__(self):
        self.caps: Capabilities = _DRONE_CAPS
        self.last_cmd: Optional[RobotCommand] = None
        self.send_zero_called: bool = False
        self.on_target_lost_count: int = 0

    async def connect(self) -> None:
        pass

    async def start_session(self) -> None:
        pass

    async def send_command(self, cmd: RobotCommand, safety_ctx: SafetyContext) -> None:
        self.last_cmd = cmd

    async def send_zero(self) -> None:
        self.send_zero_called = True

    async def on_target_lost(self, last_detection) -> None:
        self.on_target_lost_count += 1

    async def shutdown(self) -> None:
        pass


# ---------------------------------------------------------------------------
# StubSharedState — controllable detection source
# ---------------------------------------------------------------------------

class StubSharedState:
    """Returns detections from a provided sequence; repeats last entry when exhausted."""

    def __init__(self, detections: List[Optional[Detection]]):
        self._detections = list(detections)
        self._idx = 0
        self._frame = 0

    def get_latest(self) -> Tuple[Optional[Detection], int]:
        self._frame += 1
        det = self._detections[min(self._idx, len(self._detections) - 1)]
        if self._idx < len(self._detections) - 1:
            self._idx += 1
        return det, self._frame


def _make_detection(
    center_x: float = 0.3,
    center_y: float = 0.5,
    bbox_height: float = 0.4,
) -> Detection:
    """Create a Detection that produces non-zero yaw and non-zero forward (with yaw_only=False)."""
    return Detection(
        label="person",
        confidence=0.9,
        center_x=center_x,
        center_y=center_y,
        bbox_height=bbox_height,
        timestamp=time.monotonic(),
    )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

async def _run_loop_for_ticks(
    n_ticks: float,
    robot: FakeRobot,
    shared_state: StubSharedState,
    config: ControllerConfig,
    spy: Optional[SpyUIState],
) -> None:
    """Run run_robot_loop for approximately n_ticks * tick_dt seconds then shut down."""
    shutdown = asyncio.Event()
    tick_dt = 1.0 / config.control_loop_hz
    delay = n_ticks * tick_dt

    async def _stopper():
        await asyncio.sleep(delay)
        shutdown.set()

    stopper = asyncio.create_task(_stopper())
    try:
        await asyncio.wait_for(
            run_robot_loop(robot, shared_state, config, shutdown, ui_state=spy),
            timeout=2.0,
        )
    except asyncio.TimeoutError:
        shutdown.set()
        raise
    finally:
        stopper.cancel()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestOrchestratorUIUpdate:

    def test_publishes_auto_mode_with_non_zero_velocity_on_detection(self):
        """Detection-present branch → SpyUIState receives mode="AUTO" with non-zero velocity."""
        det = _make_detection(center_x=0.3, bbox_height=0.4)  # off-center → non-zero yaw
        shared = StubSharedState([det])
        robot = FakeRobot()
        spy = SpyUIState()
        config = _TEST_CONFIG_FORWARD

        asyncio.run(_run_loop_for_ticks(n_ticks=5, robot=robot, shared_state=shared, config=config, spy=spy))

        # There must be at least one AUTO publish
        auto_calls = [(f, d, y, m) for f, d, y, m in spy.calls if m == "AUTO"]
        assert auto_calls, f"No 'AUTO' mode calls in spy.calls: {spy.calls}"

        # At least one AUTO call must have non-zero yaw (center_x=0.3 → off-center)
        non_zero_yaw = [c for c in auto_calls if abs(c[2]) > 1e-9]
        assert non_zero_yaw, (
            f"All AUTO calls had yawspeed=0; expected non-zero yaw from off-center detection. "
            f"AUTO calls: {auto_calls}"
        )

    def test_publishes_lost_mode_after_search_enter_delay(self):
        """No detection from tick 0 → after search_enter_delay_s, mode='LOST' published."""
        shared = StubSharedState([None])  # always None
        robot = FakeRobot()
        spy = SpyUIState()
        config = _TEST_CONFIG  # search_enter_delay_s=0.02, 200 Hz

        # Run for enough ticks to cross search_enter_delay_s (0.02s at 200Hz = 4 ticks)
        # We run for 20 ticks to be safe
        asyncio.run(_run_loop_for_ticks(n_ticks=20, robot=robot, shared_state=shared, config=config, spy=spy))

        lost_calls = [c for c in spy.calls if c[3] == "LOST"]
        assert lost_calls, (
            f"No 'LOST' mode calls in spy.calls: {spy.calls}"
        )
        # LOST branch publishes zeros
        assert all(c[0] == 0.0 and c[1] == 0.0 and c[2] == 0.0 for c in lost_calls), (
            f"LOST calls should have (0.0, 0.0, 0.0), got: {lost_calls}"
        )

    def test_publishes_idle_on_shutdown(self):
        """After shutdown.set() and finally-block execution, LAST recorded call is (0,0,0,'IDLE')."""
        det = _make_detection()
        shared = StubSharedState([det])
        robot = FakeRobot()
        spy = SpyUIState()
        config = _TEST_CONFIG_FORWARD

        asyncio.run(_run_loop_for_ticks(n_ticks=5, robot=robot, shared_state=shared, config=config, spy=spy))

        assert spy.calls, "spy.calls is empty — loop never published anything"
        last_call = spy.calls[-1]
        assert last_call == (0.0, 0.0, 0.0, "IDLE"), (
            f"Last call was {last_call!r}, expected (0.0, 0.0, 0.0, 'IDLE'). "
            f"Full call log: {spy.calls}"
        )

    def test_no_publish_when_ui_state_is_none(self):
        """run_robot_loop with ui_state=None must not raise (no AttributeError on None)."""
        det = _make_detection()
        shared = StubSharedState([det])
        robot = FakeRobot()
        config = _TEST_CONFIG_FORWARD

        shutdown = asyncio.Event()

        async def _run():
            stopper = asyncio.create_task(_stop_after(shutdown, delay=3.0 / config.control_loop_hz))
            try:
                await asyncio.wait_for(
                    run_robot_loop(robot, shared, config, shutdown, ui_state=None),
                    timeout=2.0,
                )
            finally:
                stopper.cancel()

        # Must not raise
        asyncio.run(_run())

    def test_publish_uses_pre_smoothing_cmd(self):
        """Published forward_m_s MUST equal controller.compute(...).forward_m_s directly.

        This is the regression guard for the pre-smoothing architectural decision:
        if a future refactor moves the publish call to wrap _apply_smoothing's output,
        this test will catch it because the smoothed value would differ from the raw
        controller output (or would come from inside the adapter, violating axes-only).
        """
        det = _make_detection(center_x=0.3, bbox_height=0.4)
        shared = StubSharedState([det])
        robot = FakeRobot()
        spy = SpyUIState()
        config = _TEST_CONFIG_FORWARD

        asyncio.run(_run_loop_for_ticks(n_ticks=5, robot=robot, shared_state=shared, config=config, spy=spy))

        # Compute what controller.compute would produce for our detection + FakeRobot.caps
        expected_cmd = controller.compute(det, robot.caps, config)

        # Find the first AUTO call
        auto_calls = [(f, d, y, m) for f, d, y, m in spy.calls if m == "AUTO"]
        assert auto_calls, f"No AUTO calls in spy: {spy.calls}"

        first_auto = auto_calls[0]
        assert first_auto[0] == pytest.approx(expected_cmd.forward_m_s), (
            f"Published forward_m_s={first_auto[0]!r} != controller.compute forward={expected_cmd.forward_m_s!r}. "
            f"If these differ, the publish was moved inside the adapter (post-smoothing), "
            f"violating the pre-smoothing decision (03-11 ABS-11)."
        )
        assert first_auto[2] == pytest.approx(expected_cmd.yaw_rate), (
            f"Published yawspeed={first_auto[2]!r} != controller.compute yaw_rate={expected_cmd.yaw_rate!r}."
        )


# ---------------------------------------------------------------------------
# Helpers continued
# ---------------------------------------------------------------------------

async def _stop_after(shutdown: asyncio.Event, delay: float) -> None:
    await asyncio.sleep(delay)
    shutdown.set()
