"""Phase-3 snapshot gate.

Locks the OLD ``compute_velocity_command`` output for ~100 representative
detections so the refactor in Waves 3-5 cannot silently change behavior. All
tests in this module are marked ``@pytest.mark.xfail(strict=False)`` until
plan 03-07 captures the baseline tuple per case and strips the markers.

Why xfail and not skip:
    * Collection runs the parametrize machinery, so a typo in the case list
      surfaces as a collection error today (Wave 0 catches drift early).
    * ``strict=False`` matches the Phase 2 02-00 convention — a coincidental
      pre-fix pass is reported as ``xpass`` instead of breaking the suite.

Reference: ``.planning/phases/03-abstraction/03-CONTEXT.md`` § Regression
test strategy, ``.planning/phases/03-abstraction/03-RESEARCH.md`` § Snapshot
fixture design.
"""

from __future__ import annotations

import pytest

from robot_follow.follow_api.config import ControllerConfig
from robot_follow.follow_api.controller import compute_velocity_command
from robot_follow.tests.cases.drone_command_baseline import CASES, BaselineCase

# Strip markers: grep for these constants in 03-07 to find every xfail call
# site that has to come down with the baseline-capture commit.
XFAIL_REASON_CASES = "expected values captured + xfail stripped in 03-07-PLAN"
XFAIL_REASON_NEW = "new pipeline lands in 03-07; tests filled then"


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON_CASES)
@pytest.mark.parametrize("case", CASES, ids=lambda c: c.name)
def test_snapshot(case: BaselineCase) -> None:
    """Assert OLD compute_velocity_command output matches the captured baseline.

    Today this test is xfail across the board because
    ``expected_velocity_command`` is the placeholder ``(0.0, 0.0, 0.0)`` for
    every case. Wave 5 (plan 03-07) captures the real tuples by running a
    one-shot script against the OLD ``compute_velocity_command`` and then
    strips the xfail marker so the assertion becomes a hard gate.
    """
    config = ControllerConfig(**case.config_overrides)
    old_vc = compute_velocity_command(
        case.detection,
        config,
        last_detection=case.last_detection,
        search_active=(case.detection is None),
        hold_velocity=None,
    )
    actual = (old_vc.forward_m_s, old_vc.down_m_s, old_vc.yawspeed_deg_s)
    assert actual == pytest.approx(case.expected_velocity_command, abs=1e-6)


class TestNewPipelineEquivalence:
    """New ``compute(det, caps, config)`` + adapter pre-MAVSDK pipeline must
    equal OLD ``compute_velocity_command`` for every BaselineCase.

    Plan 03-07 fills these tests in with the full equivalence assertion. Until
    then the placeholder collects cleanly and skips if the new adapter module
    is not yet importable.
    """

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_NEW)
    def test_placeholder(self) -> None:
        try:
            # Lazy import — these symbols land in 03-06.
            from robot_follow.robot_api.adapters.mavsdk_drone import (  # noqa: F401
                MavsdkDroneAdapter,
                DRONE_CAPS,
                _apply_altitude_p,
                _apply_retreat_from_tilt,
            )
        except ImportError:
            pytest.skip(
                "robot_api.adapters.mavsdk_drone not yet present "
                "(lands in 03-06)"
            )
        assert False, "filled in 03-07"
