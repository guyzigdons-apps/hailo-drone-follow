"""R5 adapter pure-function unit-test scaffold.

Locks the per-axis unit-test surface for the four module-level helpers that
plan 03-06 extracts out of ``mavsdk_drone.py``:

    - ``_apply_altitude_p``
    - ``_apply_retreat_from_tilt``
    - ``_apply_smoothing``
    - ``_compute_search_yawspeed``

plus the orchestrated ``MavsdkDroneAdapter.send_command`` integration path.

Today none of those symbols exist on disk, so every test in this module is
``@pytest.mark.xfail(strict=False)`` and uses a lazy ``_load_adapter_module``
helper that skips cleanly when the implementation hasn't landed yet. Plan
03-06 lands the implementation, migrates the 46 cases from
``test_velocity_api_and_smoother.py``, expands each class with the full case
matrix, and strips ``XFAIL_REASON`` together with the markers.

Reference: ``.planning/phases/03-abstraction/03-CONTEXT.md`` § Adapter
unit-test plan; ``.planning/phases/03-abstraction/03-RESEARCH.md``
§ "Pure-function extracts (R5)".
"""

from __future__ import annotations

import pytest

from robot_follow.follow_api.config import ControllerConfig
from robot_follow.follow_api.types import Detection  # noqa: F401  (used by 03-06)

# grep XFAIL_REASON in plan 03-06 to find every strip site.
XFAIL_REASON = "pure-function extracts land in 03-06-PLAN"


def _load_adapter_module():
    """Lazy import of the adapter module.

    Returns the module if the implementation has landed; otherwise pytest-skips
    the calling test. Keeps collection clean today while letting 03-06's
    executor verify the placeholder skips flip to real passes when the module
    is imported normally.
    """
    try:
        from robot_follow.robot_api.adapters import mavsdk_drone as mod
    except ImportError:
        pytest.skip(
            "robot_api.adapters.mavsdk_drone not yet present (lands in 03-06)"
        )
    return mod


class TestApplyAltitudeP:
    """Unit tests for ``_apply_altitude_p`` (altitude-hold P correction).

    Plan 03-06 expands this class to ~8 cases (alt_cache empty passthrough,
    yaw_only passthrough, below/above target, floor/ceiling clamps,
    climb/descend saturation).
    """

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON)
    def test_altitude_cache_empty_passthrough(self) -> None:
        mod = _load_adapter_module()
        config = ControllerConfig()
        assert mod._apply_altitude_p(0.5, {}, config) == pytest.approx(0.5)


class TestApplyRetreatFromTilt:
    """Unit tests for ``_apply_retreat_from_tilt`` (frame-edge fade + safety).

    Plan 03-06 expands this class to ~10 cases mirroring today's
    ``TestFrameEdgeSafety`` in ``test_controller.py`` (lines 378-518).
    """

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON)
    def test_no_safety_when_outside_margin(self) -> None:
        mod = _load_adapter_module()
        config = ControllerConfig(yaw_only=False)
        # SafetyContext lands in 03-03; we build it lazily through the module.
        SafetyContext = getattr(mod, "SafetyContext", None)
        if SafetyContext is None:
            pytest.skip("SafetyContext not yet importable from adapter module")
        safety_ctx = SafetyContext.from_detection(
            Detection(
                label="person",
                confidence=0.9,
                center_x=0.5,
                center_y=0.5,
                bbox_height=0.20,
                timestamp=0.0,
            )
        )
        assert mod._apply_retreat_from_tilt(1.0, safety_ctx, config) == pytest.approx(1.0)


class TestApplySmoothing:
    """Unit tests for ``_apply_smoothing`` (clamp + EMA + slew-rate cap).

    Plan 03-06 migrates the 46 cases from
    ``test_velocity_api_and_smoother.py`` here, retargeting them at the
    extracted pure function with explicit ``SmoothingState`` instances.
    """

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON)
    def test_clamping_only_no_smoothing(self) -> None:
        # 46 cases from test_velocity_api_and_smoother.py migrate here in 03-06
        mod = _load_adapter_module()
        config = ControllerConfig(
            smooth_forward=False,
            smooth_down=False,
            smooth_yaw=False,
            max_forward_accel=0.0,
        )
        SmoothingState = mod.SmoothingState
        RobotCommand = mod.RobotCommand if hasattr(mod, "RobotCommand") else None
        if RobotCommand is None:
            pytest.skip("RobotCommand not yet importable from adapter module")
        state = SmoothingState()
        cmd = RobotCommand(forward_m_s=10.0, yaw_rate=0.0, down_m_s=0.0)
        out = mod._apply_smoothing(cmd, state, config)
        # Clamped to max_forward (default 1.5), no EMA contribution.
        assert out.forward_m_s == pytest.approx(config.max_forward)


class TestComputeSearchYawspeed:
    """Unit tests for ``_compute_search_yawspeed`` (last-seen-side spin).

    Plan 03-06 expands this class to 3 cases per RESEARCH § Pure-function
    extracts § ``_compute_search_yawspeed`` (None default, right-of-centre,
    left-of-centre).
    """

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON)
    def test_last_detection_none_returns_default(self) -> None:
        mod = _load_adapter_module()
        config = ControllerConfig()
        assert mod._compute_search_yawspeed(None, config) == pytest.approx(
            config.search_yawspeed_slow
        )


class TestMavsdkDroneAdapterIntegration:
    """Orchestrated ``send_command`` path: send_command(RobotCommand, SafetyContext)
    calls the four pure functions in order, translates to ``VelocityBodyYawspeed``,
    and dispatches via ``self._drone.offboard.set_velocity_body``.

    Plan 03-06 expands this class to cover the takeoff/land/RC-override
    branches and the offboard handshake. Wave 0 lands a single placeholder.
    """

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON)
    def test_send_command_orchestrates_pure_functions(self) -> None:
        mod = _load_adapter_module()
        # The real integration test in 03-06 instantiates MavsdkDroneAdapter
        # with a mock MAVSDK system and asserts set_velocity_body is called
        # once with the post-pipeline values. Today we just confirm the
        # class symbol is present on the module.
        assert hasattr(mod, "MavsdkDroneAdapter"), (
            "MavsdkDroneAdapter must be exposed on robot_api.adapters.mavsdk_drone"
        )
