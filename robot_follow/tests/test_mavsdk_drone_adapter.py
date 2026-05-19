"""R5 adapter pure-function + integration unit tests.

Covers the four module-level helpers extracted from ``mavsdk_drone.py``:

    - ``_apply_altitude_p``
    - ``_apply_retreat_from_tilt``
    - ``_apply_smoothing``
    - ``_compute_search_yawspeed``

plus the orchestrated ``MavsdkDroneAdapter.send_command`` /
``send_zero`` / ``on_target_lost`` integration paths.

The placeholder markers from the 03-01 scaffold are stripped; every
test is now a real, passing assertion. The VelocityCommandAPI-targeted
smoother tests in ``test_velocity_api_and_smoother.py`` stay green
until plan 03-07 deletes VelocityCommandAPI; equivalent coverage of
the extracted ``_apply_smoothing`` lives in ``TestApplySmoothing``
below.

Reference: ``.planning/phases/03-abstraction/03-CONTEXT.md`` § Adapter
unit-test plan; ``.planning/phases/03-abstraction/03-RESEARCH.md``
§ "Pure-function extracts (R5)".
"""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

import pytest

from robot_follow.follow_api.config import ControllerConfig
from robot_follow.follow_api.types import (
    Detection,
    RobotCommand,
    SafetyContext,
)
from robot_follow.robot_api.adapters.mavsdk_drone import (
    DRONE_CAPS,
    MavsdkDroneAdapter,
    SmoothingState,
    _apply_altitude_p,
    _apply_retreat_from_tilt,
    _apply_smoothing,
    _compute_search_yawspeed,
)


def _det(cx=0.5, cy=0.5, bh=0.3):
    return Detection(
        label="person",
        confidence=0.9,
        center_x=cx,
        center_y=cy,
        bbox_height=bh,
        timestamp=0.0,
    )


# ---------------------------------------------------------------------------
# TestApplyAltitudeP — altitude-hold P correction
# ---------------------------------------------------------------------------


class TestApplyAltitudeP:
    """Unit tests for ``_apply_altitude_p`` (altitude-hold P correction).

    Mirrors live_control_loop's altitude block (mavsdk_drone.py lines
    509-524). Pure: no I/O, no asyncio, no MAVSDK.
    """

    def test_altitude_cache_empty_passthrough(self) -> None:
        """No altitude reading available → pass input through unchanged."""
        config = ControllerConfig()
        assert _apply_altitude_p(0.5, {}, config) == pytest.approx(0.5)
        assert _apply_altitude_p(-0.7, {}, config) == pytest.approx(-0.7)
        assert _apply_altitude_p(0.0, {}, config) == pytest.approx(0.0)

    def test_yaw_only_passthrough_keeps_input_but_still_applies_clamps(self) -> None:
        """yaw_only=True: don't compute P term, but floor/ceiling clamps still active."""
        config = ControllerConfig(
            yaw_only=True, target_altitude=3.0,
            min_altitude=2.0, max_altitude=4.0,
        )
        # Mid-air, input passthrough.
        assert _apply_altitude_p(0.5, {"m": 3.0}, config) == pytest.approx(0.5)
        # At floor with positive (descend) → clamped to 0.
        assert _apply_altitude_p(0.5, {"m": 2.0}, config) == pytest.approx(0.0)
        # At ceiling with negative (climb) → clamped to 0.
        assert _apply_altitude_p(-0.3, {"m": 4.0}, config) == pytest.approx(0.0)

    def test_well_below_target_climbs(self) -> None:
        """Current alt below target → negative down (climb)."""
        config = ControllerConfig(
            yaw_only=False, target_altitude=3.0, kp_alt_hold=0.5,
            min_altitude=2.0, max_altitude=10.0,
            max_climb_speed=2.0, max_down_speed=2.0,
        )
        # err = 1.0 - 3.0 = -2.0; down = 0.5 * -2.0 = -1.0 (climb).
        out = _apply_altitude_p(0.0, {"m": 1.0}, config)
        # But min_altitude=2.0 and m=1.0 is BELOW floor. The current-alt <= min
        # branch fires only when down > 0; here down is negative (climb), so
        # no clamp. We expect the raw P value.
        assert out == pytest.approx(-1.0)

    def test_well_above_target_descends(self) -> None:
        """Current alt above target → positive down (descend)."""
        config = ControllerConfig(
            yaw_only=False, target_altitude=3.0, kp_alt_hold=0.5,
            min_altitude=0.5, max_altitude=10.0,
            max_climb_speed=2.0, max_down_speed=2.0,
        )
        # err = 5.0 - 3.0 = +2.0; down = 0.5 * 2.0 = 1.0 (descend).
        out = _apply_altitude_p(0.0, {"m": 5.0}, config)
        assert out == pytest.approx(1.0)

    def test_at_floor_descending_clamped_to_zero(self) -> None:
        """At min_altitude with positive down (descend) → clamped to 0.

        Construction quirk: ControllerConfig.validate requires
        ``min_altitude <= target_altitude <= max_altitude``. To force a
        descend command at the floor we set current_alt slightly ABOVE
        target so err > 0 → down > 0, then place the floor at the same
        current_alt value to trip the at-or-below-floor clamp.
        """
        config = ControllerConfig(
            yaw_only=False, target_altitude=2.0, kp_alt_hold=0.5,
            min_altitude=2.0, max_altitude=10.0,
        )
        # current_alt=2.0, target=2.0 → err=0 → down=0. At floor (m<=min)
        # AND down > 0 condition is False (down==0), so output is raw 0.
        # Use a slightly higher current_alt to get positive down:
        # current_alt=2.05, target=2.0 → err=+0.05 → down=+0.025 (positive).
        # BUT current_alt > min_altitude so floor clamp wouldn't fire.
        # Instead: raise min_altitude to match current_alt.
        config2 = ControllerConfig(
            yaw_only=False, target_altitude=2.05, kp_alt_hold=0.5,
            min_altitude=2.05, max_altitude=10.0,
        )
        # current_alt=2.5, target=2.05 → err=+0.45 → down=+0.225 (positive).
        # current_alt > min (2.5 > 2.05), so NOT at floor — clamp doesn't fire.
        # We need current_alt <= min. Force m=2.05 and min=2.05 (equal):
        out = _apply_altitude_p(0.0, {"m": 2.05}, config2)
        # err = 2.05 - 2.05 = 0 → down=0, not positive → no clamp activity.
        # Result is 0 either way; the goal here is to validate the
        # at-floor branch does NOT misbehave when down==0.
        assert out == pytest.approx(0.0)
        # Now construct a config where current_alt < min_altitude AND down > 0:
        # min_altitude must be > current_alt. Use target_altitude=2.0,
        # min_altitude=2.0, current_alt=1.0 — but that violates the
        # "current_alt below min_altitude" runtime case (which IS legal at
        # runtime — the cache can carry a reading below the floor while the
        # drone climbs). Use target=2.0, min=2.0, max=10.0, current=1.0:
        config3 = ControllerConfig(
            yaw_only=False, target_altitude=2.0, kp_alt_hold=0.5,
            min_altitude=2.0, max_altitude=10.0,
            max_climb_speed=1.0, max_down_speed=1.5,
        )
        # current=1.0, target=2.0 → err=-1.0 → down=-0.5 (climb, negative).
        # At-floor branch only fires when down > 0; here down is negative,
        # no clamp. So output is -0.5 (climb).
        assert _apply_altitude_p(0.0, {"m": 1.0}, config3) == pytest.approx(-0.5)
        # To get positive down with current_alt <= min, we need target < min.
        # That's forbidden by validate(). So the at-floor descending clamp is
        # only reachable when target_altitude is reduced AT RUNTIME below the
        # floor (UI live edit) — runtime mutation skips __post_init__.
        config4 = ControllerConfig(
            yaw_only=False, target_altitude=2.0, kp_alt_hold=0.5,
            min_altitude=2.0, max_altitude=10.0,
        )
        config4.target_altitude = 1.0  # bypass validate via direct mutation
        # current=2.0, target=1.0 → err=+1.0 → down=+0.5 (descend). At floor
        # (current <= min) AND down > 0 → clamp to 0.
        assert _apply_altitude_p(0.0, {"m": 2.0}, config4) == pytest.approx(0.0)

    def test_at_ceiling_climbing_clamped_to_zero(self) -> None:
        """At max_altitude with negative down (climb) → clamped to 0.

        Symmetric to floor case; target_altitude must satisfy
        ``min <= target <= max``, so we bypass validate via direct
        mutation to force a climb command at the ceiling.
        """
        config = ControllerConfig(
            yaw_only=False, target_altitude=4.0, kp_alt_hold=0.5,
            min_altitude=1.0, max_altitude=4.0,
        )
        config.target_altitude = 8.0  # bypass validate via direct mutation
        # current=4.0, target=8.0 → err=-4.0 → down=-2.0 (climb, negative).
        # At ceiling (current >= max) AND down < 0 → clamp to 0.
        out = _apply_altitude_p(0.0, {"m": 4.0}, config)
        assert out == pytest.approx(0.0)

    def test_saturate_max_climb_speed(self) -> None:
        """Very negative err → clamped to -max_climb_speed."""
        config = ControllerConfig(
            yaw_only=False, target_altitude=100.0, kp_alt_hold=2.0,
            min_altitude=0.5, max_altitude=200.0,
            max_climb_speed=1.0, max_down_speed=1.5,
        )
        # err = 1.0 - 100.0 = -99; raw = 2 * -99 = -198 → clamps to -1.0.
        out = _apply_altitude_p(0.0, {"m": 1.0}, config)
        assert out == pytest.approx(-1.0)

    def test_saturate_max_down_speed(self) -> None:
        """Very positive err → clamped to +max_down_speed."""
        config = ControllerConfig(
            yaw_only=False, target_altitude=0.5, kp_alt_hold=2.0,
            min_altitude=0.5, max_altitude=200.0,
            max_climb_speed=1.0, max_down_speed=1.5,
        )
        # err = 100.0 - 0.5 = +99.5; raw = +199 → clamps to +1.5.
        out = _apply_altitude_p(0.0, {"m": 100.0}, config)
        assert out == pytest.approx(1.5)


# ---------------------------------------------------------------------------
# TestApplyRetreatFromTilt — bottom/top frame-edge fade + safety push
# ---------------------------------------------------------------------------


class TestApplyRetreatFromTilt:
    """Unit tests for ``_apply_retreat_from_tilt``.

    Mirrors today's ``TestFrameEdgeSafety`` cases in
    ``test_controller.py`` lines 378-518, retargeted at the pure
    function with explicit SafetyContext inputs.

    Q6 lock: when ``safety_ctx.target_lost`` is True, returns
    ``forward_m_s`` unchanged regardless of bbox values.
    """

    def _cfg(self, **overrides):
        defaults = dict(
            yaw_only=False, target_bbox_height=0.3,
            top_margin_safety=0.05, bottom_margin_safety=0.05,
            kp_distance=1.0, kp_distance_back=1.0,
            max_forward=1.5, max_backward=1.5,
        )
        defaults.update(overrides)
        return ControllerConfig(**defaults)

    def test_target_lost_returns_input_unchanged(self) -> None:
        """Q6 lock: target_lost=True → passthrough; sentinel bbox values ignored."""
        cfg = self._cfg()
        lost_ctx = SafetyContext.lost()
        # The sentinel SafetyContext has bbox_bottom=0.5, size=0.25, which
        # would NOT trigger any edge safety, but the early-return path
        # must not even look at the values. Test with a synthetic
        # SafetyContext that WOULD trigger retreat if examined:
        evil_ctx = SafetyContext(
            bbox_bottom_normalized=1.0,  # would trigger bottom-edge push
            bbox_size_normalized=0.2,
            target_lost=True,            # but lost flag must win
            last_target_x=None,
        )
        assert _apply_retreat_from_tilt(0.5, lost_ctx, cfg) == pytest.approx(0.5)
        assert _apply_retreat_from_tilt(-0.7, lost_ctx, cfg) == pytest.approx(-0.7)
        assert _apply_retreat_from_tilt(0.3, evil_ctx, cfg) == pytest.approx(0.3)

    def test_yaw_only_passthrough(self) -> None:
        """yaw_only=True → return forward unchanged."""
        cfg = self._cfg(yaw_only=True)
        ctx = SafetyContext.from_detection(_det(cx=0.5, cy=0.95, bh=0.2))
        assert _apply_retreat_from_tilt(1.0, ctx, cfg) == pytest.approx(1.0)

    def test_no_breach_outside_all_zones(self) -> None:
        """Centered bbox of normal size → no fade, no push, return input."""
        cfg = self._cfg()
        ctx = SafetyContext.from_detection(_det(cx=0.5, cy=0.5, bh=0.2))
        # bbox_bottom=0.6, bbox_top=0.4 — well outside both margins (0.05).
        assert _apply_retreat_from_tilt(0.7, ctx, cfg) == pytest.approx(0.7)
        assert _apply_retreat_from_tilt(-0.3, ctx, cfg) == pytest.approx(-0.3)

    def test_bottom_edge_full_breach_max_backward(self) -> None:
        """Bottom edge fully past 1-margin → max_backward push."""
        cfg = self._cfg()
        # cy=0.95, bh=0.2 → bottom=1.05 (past 0.95). depth=0.10, ratio=1.0
        # → -max_backward push wins over any positive input.
        ctx = SafetyContext.from_detection(_det(cy=0.95, bh=0.2))
        assert _apply_retreat_from_tilt(1.0, ctx, cfg) == pytest.approx(-cfg.max_backward)

    def test_top_edge_full_breach_max_forward(self) -> None:
        """Top edge fully past margin → max_forward push."""
        cfg = self._cfg()
        # cy=0.05, bh=0.2 → top = -0.05 (past 0.05). depth=0.10, ratio=1.0
        # → +max_forward push.
        ctx = SafetyContext.from_detection(_det(cy=0.05, bh=0.2))
        # Start with a negative forward (retreat): the top push overrides via
        # max(forward, push).
        assert _apply_retreat_from_tilt(-0.5, ctx, cfg) == pytest.approx(cfg.max_forward)

    def test_bottom_partial_breach_proportional(self) -> None:
        """Bottom margin partially entered → proportional backward force."""
        cfg = self._cfg(kp_distance=2.0, max_forward=2.0, max_backward=3.0)
        # cy=0.93, bh=0.06 → bottom=0.96. depth=0.01, ratio=0.2, push=-0.6.
        # Input forward=2.0 (clamped natural). Result: min(2.0, -0.6) = -0.6.
        ctx = SafetyContext.from_detection(_det(cy=0.93, bh=0.06))
        # NOTE: input is the AFTER-bbox-controller forward. Pass 2.0 here.
        # But the fade-zone first scales 2.0 by fade-factor before pushing.
        # bbox_bottom=0.96, fade_depth=0.96-(1-2*0.05)=0.06, fade=1-0.06/0.05=-0.2 → 0
        # → fade scales forward to 0. Then push: min(0, -0.6) = -0.6.
        assert _apply_retreat_from_tilt(2.0, ctx, cfg) == pytest.approx(-0.6)

    def test_bottom_fade_at_margin_boundary_kills_approach(self) -> None:
        """At inner edge of bottom margin: natural forward fades to 0; no push yet."""
        cfg = self._cfg(bottom_margin_safety=0.1, kp_distance=2.0)
        # cy=0.85, bh=0.10 → bbox_bottom=0.90 (margin entry).
        # fade_depth = 0.90 - (1-0.2) = 0.10; fade = 1-0.10/0.10 = 0 → 0.
        # safety depth = 0.90 - 0.90 = 0 → no push.
        ctx = SafetyContext.from_detection(_det(cy=0.85, bh=0.10))
        assert _apply_retreat_from_tilt(1.0, ctx, cfg) == pytest.approx(0.0)

    def test_bottom_fade_does_not_touch_backward_natural(self) -> None:
        """Fade zone only damps the offending (positive) direction."""
        cfg = self._cfg(bottom_margin_safety=0.1, kp_distance=2.0)
        # cy=0.60, bh=0.50 → bbox_bottom=0.85 (in fade zone [0.80, 0.90]).
        # Negative natural forward is NOT faded; outside margin so no push.
        ctx = SafetyContext.from_detection(_det(cy=0.60, bh=0.50))
        assert _apply_retreat_from_tilt(-0.8, ctx, cfg) == pytest.approx(-0.8)

    def test_top_fade_outside_margin_scales_retreat(self) -> None:
        """Top fade zone: negative natural forward (retreat) scales toward 0."""
        cfg = self._cfg(top_margin_safety=0.1, kp_distance=2.0)
        # cy=0.40, bh=0.50 → bbox_top=0.15 (in fade zone [0.10, 0.20]).
        # fade_depth = 0.20 - 0.15 = 0.05; fade = 1 - 0.05/0.10 = 0.5.
        # Input -0.8 → -0.4. bbox_top > margin (=0.10), no push.
        ctx = SafetyContext.from_detection(_det(cy=0.40, bh=0.50))
        assert _apply_retreat_from_tilt(-0.8, ctx, cfg) == pytest.approx(-0.4)

    def test_top_fade_does_not_touch_forward_natural(self) -> None:
        """Top fade zone only damps backward natural; forward survives intact."""
        cfg = self._cfg(top_margin_safety=0.1, kp_distance=2.0, max_forward=2.0)
        # cy=0.20, bh=0.10 → bbox_top=0.15 (in fade zone).
        # Positive natural is NOT faded. No push (top > margin=0.10).
        ctx = SafetyContext.from_detection(_det(cy=0.20, bh=0.10))
        assert _apply_retreat_from_tilt(2.0, ctx, cfg) == pytest.approx(2.0)

    def test_disabled_when_both_margins_zero(self) -> None:
        """top_margin_safety=0 and bottom_margin_safety=0 → no override."""
        cfg = self._cfg(top_margin_safety=0.0, bottom_margin_safety=0.0)
        # bbox_bottom=1.05, would normally trigger bottom push — disabled.
        ctx = SafetyContext.from_detection(_det(cy=0.95, bh=0.2))
        assert _apply_retreat_from_tilt(0.5, ctx, cfg) == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# TestApplySmoothing — clamp + EMA + slew-rate cap
# ---------------------------------------------------------------------------


class TestApplySmoothing:
    """Unit tests for ``_apply_smoothing`` (clamp + EMA + slew cap).

    Representative coverage of the four behavior classes. The 46
    VelocityCommandAPI-targeted cases in
    ``test_velocity_api_and_smoother.py`` continue to cover the
    legacy production path until plan 03-07 deletes VelocityCommandAPI.
    """

    def test_clamping_only_no_smoothing(self) -> None:
        """No EMA / no slew → output is just clamped raw values."""
        config = ControllerConfig(
            smooth_forward=False, smooth_down=False, smooth_yaw=False,
            max_forward_accel=0.0,
            max_forward=1.5, max_backward=1.5,
            max_down_speed=1.5, max_yawspeed=90.0,
        )
        state = SmoothingState()
        cmd = RobotCommand(forward_m_s=999.0, yaw_rate=999.0, down_m_s=999.0)
        out = _apply_smoothing(cmd, state, config)
        assert out.forward_m_s == pytest.approx(config.max_forward)
        assert out.yaw_rate == pytest.approx(config.max_yawspeed)
        assert out.down_m_s == pytest.approx(config.max_down_speed)
        # State picked up the clamped values.
        assert state.filtered_forward == pytest.approx(config.max_forward)
        assert state.filtered_yaw == pytest.approx(config.max_yawspeed)
        assert state.filtered_down == pytest.approx(config.max_down_speed)

    def test_ema_first_call_approaches_alpha_times_raw(self) -> None:
        """First call with zero state → output = alpha * raw (other axes too)."""
        config = ControllerConfig(
            smooth_forward=True, forward_alpha=0.2,
            smooth_down=True, down_alpha=0.3,
            smooth_yaw=True, yaw_alpha=0.4,
            max_forward_accel=0.0,
        )
        state = SmoothingState()
        cmd = RobotCommand(forward_m_s=1.0, yaw_rate=10.0, down_m_s=0.5)
        out = _apply_smoothing(cmd, state, config)
        # 0.2 * 1.0 + 0.8 * 0.0 = 0.2
        assert out.forward_m_s == pytest.approx(0.2)
        assert out.down_m_s == pytest.approx(0.15)  # 0.3 * 0.5
        assert out.yaw_rate == pytest.approx(4.0)   # 0.4 * 10.0

    def test_ema_converges_with_repeated_calls(self) -> None:
        """Many repeated identical calls → output converges to raw value."""
        config = ControllerConfig(
            smooth_forward=True, forward_alpha=0.5,
            smooth_down=False, smooth_yaw=False,
            max_forward_accel=0.0,
        )
        state = SmoothingState()
        cmd = RobotCommand(forward_m_s=1.0, yaw_rate=0.0, down_m_s=0.0)
        out = None
        for _ in range(50):
            out = _apply_smoothing(cmd, state, config)
        assert out.forward_m_s == pytest.approx(1.0, abs=1e-6)

    def test_slew_cap_limits_forward_rise(self) -> None:
        """Forward steps capped by max_forward_accel / control_loop_hz."""
        config = ControllerConfig(
            smooth_forward=False, smooth_down=False, smooth_yaw=False,
            max_forward_accel=1.5, control_loop_hz=10.0,
            max_forward=5.0, max_backward=5.0,
        )
        state = SmoothingState()
        # prev_forward starts at 0; max_step = 1.5/10 = 0.15.
        # Request 2.0 in one tick → output clamped to +0.15.
        cmd = RobotCommand(forward_m_s=2.0, yaw_rate=0.0, down_m_s=0.0)
        out = _apply_smoothing(cmd, state, config)
        assert out.forward_m_s == pytest.approx(0.15)
        assert state.prev_forward == pytest.approx(0.15)

    def test_clamp_max_yawspeed(self) -> None:
        """Large yaw_rate clamped to max_yawspeed regardless of smoothing."""
        config = ControllerConfig(
            smooth_yaw=False,
            smooth_forward=False, smooth_down=False,
            max_forward_accel=0.0,
            max_yawspeed=60.0,
        )
        state = SmoothingState()
        cmd = RobotCommand(forward_m_s=0.0, yaw_rate=999.0, down_m_s=0.0)
        out = _apply_smoothing(cmd, state, config)
        assert out.yaw_rate == pytest.approx(60.0)

    def test_returns_new_robot_command_does_not_mutate_input(self) -> None:
        """Input RobotCommand is not mutated; output is a fresh instance."""
        config = ControllerConfig(
            smooth_forward=True, forward_alpha=0.5,
            smooth_down=False, smooth_yaw=False,
            max_forward_accel=0.0,
        )
        state = SmoothingState()
        cmd = RobotCommand(forward_m_s=1.0, yaw_rate=0.0, down_m_s=0.0)
        out = _apply_smoothing(cmd, state, config)
        assert out is not cmd
        assert cmd.forward_m_s == pytest.approx(1.0)  # input unchanged
        assert out.forward_m_s == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# TestComputeSearchYawspeed — last-seen-side spin
# ---------------------------------------------------------------------------


class TestComputeSearchYawspeed:
    """Unit tests for ``_compute_search_yawspeed``."""

    def test_no_last_detection_returns_default_positive(self) -> None:
        """No last detection → default to +search_yawspeed_slow (spin right)."""
        config = ControllerConfig(search_yawspeed_slow=12.5)
        assert _compute_search_yawspeed(None, config) == pytest.approx(12.5)

    def test_last_detection_right_returns_positive(self) -> None:
        """center_x > 0.5 → spin right (+search_yawspeed_slow)."""
        config = ControllerConfig(search_yawspeed_slow=10.0)
        det = _det(cx=0.8, cy=0.5, bh=0.2)
        assert _compute_search_yawspeed(det, config) == pytest.approx(10.0)

    def test_last_detection_left_returns_negative(self) -> None:
        """center_x < 0.5 → spin left (-search_yawspeed_slow)."""
        config = ControllerConfig(search_yawspeed_slow=10.0)
        det = _det(cx=0.2, cy=0.5, bh=0.2)
        assert _compute_search_yawspeed(det, config) == pytest.approx(-10.0)

    def test_last_detection_at_center_treated_as_left(self) -> None:
        """center_x == 0.5 → spin left (the > 0.5 check is strict)."""
        config = ControllerConfig(search_yawspeed_slow=10.0)
        det = _det(cx=0.5, cy=0.5, bh=0.2)
        assert _compute_search_yawspeed(det, config) == pytest.approx(-10.0)


# ---------------------------------------------------------------------------
# TestMavsdkDroneAdapterIntegration — orchestrated send_command path
# ---------------------------------------------------------------------------


class MockOffboard:
    def __init__(self):
        self.calls: list = []

    async def set_velocity_body(self, vbys):
        self.calls.append(vbys)


class MockDrone:
    def __init__(self):
        self.offboard = MockOffboard()


def _build_adapter(*, takeoff_landing: bool = False, config: ControllerConfig | None = None) -> MavsdkDroneAdapter:
    """Build a MavsdkDroneAdapter wired to a MockDrone (bypassing connect)."""
    cfg = config or ControllerConfig(yaw_only=False)
    args = SimpleNamespace(
        connection="udp://0:0",
        takeoff_landing=takeoff_landing,
        target_altitude=cfg.target_altitude,
    )
    adapter = MavsdkDroneAdapter(args, cfg)
    adapter._drone = MockDrone()
    return adapter


class TestMavsdkDroneAdapterIntegration:
    """Orchestrated ``send_command`` / ``send_zero`` / ``on_target_lost``."""

    def test_caps_is_drone_caps(self) -> None:
        """adapter.caps must equal DRONE_CAPS."""
        adapter = _build_adapter()
        assert adapter.caps == DRONE_CAPS
        assert adapter.caps.yaw_unit == "deg/s"

    def test_send_command_short_circuits_on_target_lost(self) -> None:
        """Q6 lock: target_lost=True → NO set_velocity_body call."""
        adapter = _build_adapter()
        cmd = RobotCommand(forward_m_s=1.0, yaw_rate=5.0, down_m_s=0.3)
        lost_ctx = SafetyContext.lost(last_target_x=0.7)
        asyncio.run(adapter.send_command(cmd, lost_ctx))
        assert adapter._drone.offboard.calls == []

    def test_send_command_orchestrates_pure_functions(self) -> None:
        """Non-lost path: altitude P + retreat-from-tilt + smoothing applied,
        then forwarded to set_velocity_body."""
        cfg = ControllerConfig(
            yaw_only=False,
            target_altitude=3.0, kp_alt_hold=0.5,
            min_altitude=2.0, max_altitude=4.0,
            max_climb_speed=1.0, max_down_speed=1.5,
            smooth_forward=False, smooth_down=False, smooth_yaw=False,
            max_forward_accel=0.0,
            top_margin_safety=0.0, bottom_margin_safety=0.0,  # disable edge safety
            max_yawspeed=90.0,
        )
        adapter = _build_adapter(config=cfg)
        adapter._altitude_cache["m"] = 5.0  # above max=4.0 → climb (down=-1.0) BUT at ceiling, clamped to 0
        # Use a controlled scenario: target=3.0, current=3.5 → err=0.5 → down=0.25
        adapter._altitude_cache["m"] = 3.5
        cmd = RobotCommand(forward_m_s=0.8, yaw_rate=20.0, down_m_s=0.0)
        ctx = SafetyContext.from_detection(_det(cx=0.5, cy=0.5, bh=0.2))
        asyncio.run(adapter.send_command(cmd, ctx))
        assert len(adapter._drone.offboard.calls) == 1
        vbys = adapter._drone.offboard.calls[0]
        assert vbys.forward_m_s == pytest.approx(0.8)
        assert vbys.down_m_s == pytest.approx(0.25)
        assert vbys.yawspeed_deg_s == pytest.approx(20.0)
        assert vbys.right_m_s == pytest.approx(0.0)

    def test_send_zero_resets_smoothing_and_sends_zero(self) -> None:
        """send_zero clears smoothing state + dispatches all-zero setpoint."""
        adapter = _build_adapter()
        adapter._smoothing.filtered_forward = 0.7
        adapter._smoothing.filtered_yaw = 30.0
        adapter._smoothing.filtered_down = 0.4
        adapter._smoothing.prev_forward = 0.7
        asyncio.run(adapter.send_zero())
        assert adapter._smoothing.filtered_forward == 0.0
        assert adapter._smoothing.filtered_yaw == 0.0
        assert adapter._smoothing.filtered_down == 0.0
        assert adapter._smoothing.prev_forward == 0.0
        assert len(adapter._drone.offboard.calls) == 1
        vbys = adapter._drone.offboard.calls[0]
        assert vbys.forward_m_s == 0.0
        assert vbys.right_m_s == 0.0
        assert vbys.down_m_s == 0.0
        assert vbys.yawspeed_deg_s == 0.0

    def test_on_target_lost_yaw_spin_direction(self) -> None:
        """on_target_lost dispatches yaw-only setpoint with sign per last bbox side."""
        cfg = ControllerConfig(search_yawspeed_slow=10.0)
        # Right side → +10
        adapter_right = _build_adapter(config=cfg)
        asyncio.run(adapter_right.on_target_lost(_det(cx=0.8, cy=0.5, bh=0.2)))
        assert len(adapter_right._drone.offboard.calls) == 1
        vbys_r = adapter_right._drone.offboard.calls[0]
        assert vbys_r.yawspeed_deg_s == pytest.approx(10.0)
        assert vbys_r.forward_m_s == 0.0
        assert vbys_r.down_m_s == 0.0
        # Left side → -10
        adapter_left = _build_adapter(config=cfg)
        asyncio.run(adapter_left.on_target_lost(_det(cx=0.2, cy=0.5, bh=0.2)))
        vbys_l = adapter_left._drone.offboard.calls[0]
        assert vbys_l.yawspeed_deg_s == pytest.approx(-10.0)

    def test_on_target_lost_none_detection_returns_default(self) -> None:
        """on_target_lost with None last_detection → default +search_yawspeed_slow."""
        cfg = ControllerConfig(search_yawspeed_slow=7.5)
        adapter = _build_adapter(config=cfg)
        asyncio.run(adapter.on_target_lost(None))
        vbys = adapter._drone.offboard.calls[0]
        assert vbys.yawspeed_deg_s == pytest.approx(7.5)

    def test_send_command_when_drone_none_is_silent(self) -> None:
        """send_command before connect() returns silently — mirrors today's behavior."""
        cfg = ControllerConfig(yaw_only=False)
        args = SimpleNamespace(connection="udp://0:0", takeoff_landing=False, target_altitude=3.0)
        adapter = MavsdkDroneAdapter(args, cfg)
        # adapter._drone is None (no connect() / no MockDrone wired).
        cmd = RobotCommand(forward_m_s=1.0, yaw_rate=5.0, down_m_s=0.0)
        ctx = SafetyContext.from_detection(_det())
        # Must not raise.
        asyncio.run(adapter.send_command(cmd, ctx))

    def test_isinstance_robot_protocol(self) -> None:
        """Adapter satisfies the runtime-checkable Robot protocol."""
        from robot_follow.robot_api.robot import Robot
        adapter = _build_adapter()
        assert isinstance(adapter, Robot)
