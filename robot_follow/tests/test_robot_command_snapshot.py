"""Phase-3 snapshot gate.

Asserts the NEW pipeline (controller.compute + adapter pre-MAVSDK
transformations: _apply_altitude_p + _apply_retreat_from_tilt +
_apply_smoothing) produces the SAME numerical output as the captured
baseline values in cases/drone_command_baseline.py (those values were
captured in 03-07 Task 1 by running today's pre-rewrite
compute_velocity_command against each BaselineCase).

The xfail markers from the 03-01 scaffold were stripped here once 03-07
populated the baseline and migrated the controller signature.

Lifecycle note (R5 callback): this fixture is a Phase-3 transition
artifact. Recommended cleanup is **Option A — delete this file plus
cases/drone_command_baseline.py in a post-Phase-3 commit**, replacing
with property-based (Hypothesis) controller invariant tests. See
03-07-SUMMARY § Snapshot test archive plan.
"""

from __future__ import annotations

import pytest

from robot_follow.follow_api.config import ControllerConfig
from robot_follow.follow_api.controller import compute
from robot_follow.follow_api.types import RobotCommand, SafetyContext
from robot_follow.robot_api.adapters.mavsdk_drone import (
    DRONE_CAPS,
    _apply_altitude_p,
    _apply_retreat_from_tilt,
    _compute_search_yawspeed,
)
from robot_follow.tests.cases.drone_command_baseline import CASES, BaselineCase


@pytest.mark.parametrize("case", CASES, ids=lambda c: c.name)
def test_snapshot(case: BaselineCase) -> None:
    """Assert NEW pipeline output matches the captured baseline.

    The captured tuple is the **pre-smoothing** output of today's legacy
    ``compute_velocity_command`` (which applied yaw + forward + edge-safety
    + deadband, but NOT EMA / slew). For equivalence, the new pipeline
    here replicates only the corresponding steps:

      detection present:
          rc = compute(detection, DRONE_CAPS, config)
          down  = _apply_altitude_p(rc.down_m_s, {}, config)
          forward = _apply_retreat_from_tilt(rc.forward_m_s, safety_ctx, config)
          actual = (forward, down, rc.yaw_rate)

      detection absent (search-mode):
          actual = (0.0, 0.0, _compute_search_yawspeed(last_detection, config))

    ``_apply_smoothing`` is intentionally NOT in the snapshot chain — it is
    a stateful EMA / slew-rate filter that would inject one-tick lag the
    legacy single-call capture did not exercise. The smoother is covered
    by ``test_mavsdk_drone_adapter.py::TestApplySmoothing``.
    """
    config = ControllerConfig(**case.config_overrides)

    if case.detection is None:
        # Search-mode case (last_detection drives spin direction).
        yawspeed = _compute_search_yawspeed(case.last_detection, config)
        actual = (0.0, 0.0, yawspeed)
    else:
        rc = compute(case.detection, DRONE_CAPS, config)
        safety_ctx = SafetyContext.from_detection(case.detection)
        # Replicate the controller-plus-tilt portion of the adapter
        # send_command pipeline (pre-MAVSDK, pre-smoothing). altitude_cache
        # is empty → _apply_altitude_p passes through rc.down_m_s (0.0).
        down = _apply_altitude_p(rc.down_m_s, {}, config)
        forward = _apply_retreat_from_tilt(rc.forward_m_s, safety_ctx, config)
        actual = (forward, down, rc.yaw_rate)

    assert actual == pytest.approx(case.expected_velocity_command, abs=1e-6)
