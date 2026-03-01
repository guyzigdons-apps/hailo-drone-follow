"""Tests for VelocityCommandAPI, ForwardSmoother, and _effective_target_bbox_height."""

import asyncio
import math
import time
from unittest.mock import patch

import pytest

from drone_follow.follow_api import (
    ControllerConfig,
    Detection,
    VelocityCommand,
    ForwardSmoother,
    _distance_to_bbox_height,
    _effective_target_bbox_height,
)
from drone_follow.drone_api import VelocityCommandAPI


def _det(cx=0.5, cy=0.5, bh=0.3):
    return Detection(
        label="test", confidence=0.9,
        center_x=cx, center_y=cy, bbox_height=bh,
        timestamp=time.monotonic(),
    )


# ---------------------------------------------------------------------------
# VelocityCommandAPI
# ---------------------------------------------------------------------------

class TestVelocityCommandAPIClamping:
    """send() should clamp each axis to configured maximums."""

    @pytest.fixture
    def api(self):
        cfg = ControllerConfig(
            max_forward=2.0, max_backward=3.0,
            max_down_speed=1.5, max_yawspeed=90.0,
            smooth_yaw=False,
        )
        return VelocityCommandAPI(drone=None, config=cfg)

    def test_forward_clamped_to_max(self, api):
        cmd = VelocityCommand(999.0, 0.0, 0.0, 0.0)
        result = asyncio.get_event_loop().run_until_complete(api.send(cmd))
        assert result.forward_m_s == pytest.approx(2.0)

    def test_backward_clamped_to_max(self, api):
        cmd = VelocityCommand(-999.0, 0.0, 0.0, 0.0)
        result = asyncio.get_event_loop().run_until_complete(api.send(cmd))
        assert result.forward_m_s == pytest.approx(-3.0)

    def test_down_clamped_both_directions(self, api):
        up = asyncio.get_event_loop().run_until_complete(
            api.send(VelocityCommand(0.0, 0.0, -999.0, 0.0)))
        down = asyncio.get_event_loop().run_until_complete(
            api.send(VelocityCommand(0.0, 0.0, 999.0, 0.0)))
        assert up.down_m_s == pytest.approx(-1.5)
        assert down.down_m_s == pytest.approx(1.5)

    def test_yaw_clamped(self, api):
        result = asyncio.get_event_loop().run_until_complete(
            api.send(VelocityCommand(0.0, 0.0, 0.0, 200.0)))
        assert result.yawspeed_deg_s == pytest.approx(90.0)

    def test_right_clamped_to_1(self, api):
        result = asyncio.get_event_loop().run_until_complete(
            api.send(VelocityCommand(0.0, 5.0, 0.0, 0.0)))
        assert result.right_m_s == pytest.approx(1.0)

    def test_within_limits_passes_through(self, api):
        cmd = VelocityCommand(1.0, 0.5, -0.5, 30.0)
        result = asyncio.get_event_loop().run_until_complete(api.send(cmd))
        assert result.forward_m_s == pytest.approx(1.0)
        assert result.right_m_s == pytest.approx(0.5)
        assert result.down_m_s == pytest.approx(-0.5)
        assert result.yawspeed_deg_s == pytest.approx(30.0)


class TestVelocityCommandAPIYawFilter:
    """Yaw low-pass filter behavior with smooth_yaw enabled."""

    def test_filter_smooths_step_input(self):
        cfg = ControllerConfig(smooth_yaw=True, yaw_alpha=0.3, max_yawspeed=200.0)
        api = VelocityCommandAPI(drone=None, config=cfg)
        loop = asyncio.get_event_loop()

        step = VelocityCommand(0.0, 0.0, 0.0, 100.0)
        r1 = loop.run_until_complete(api.send(step))
        # First sample: filtered = 0.3 * 100 + 0.7 * 0 = 30
        assert r1.yawspeed_deg_s == pytest.approx(30.0)

        r2 = loop.run_until_complete(api.send(step))
        # Second: filtered = 0.3 * 100 + 0.7 * 30 = 51
        assert r2.yawspeed_deg_s == pytest.approx(51.0)

    def test_filter_converges(self):
        cfg = ControllerConfig(smooth_yaw=True, yaw_alpha=0.5, max_yawspeed=200.0)
        api = VelocityCommandAPI(drone=None, config=cfg)
        loop = asyncio.get_event_loop()

        step = VelocityCommand(0.0, 0.0, 0.0, 60.0)
        result = None
        for _ in range(50):
            result = loop.run_until_complete(api.send(step))
        assert result.yawspeed_deg_s == pytest.approx(60.0, abs=0.1)

    def test_smooth_yaw_off_passes_through(self):
        cfg = ControllerConfig(smooth_yaw=False, max_yawspeed=200.0)
        api = VelocityCommandAPI(drone=None, config=cfg)
        loop = asyncio.get_event_loop()

        r = loop.run_until_complete(
            api.send(VelocityCommand(0.0, 0.0, 0.0, 100.0)))
        assert r.yawspeed_deg_s == pytest.approx(100.0)

    def test_custom_yaw_alpha_overrides_config(self):
        cfg = ControllerConfig(smooth_yaw=True, yaw_alpha=0.9, max_yawspeed=200.0)
        api = VelocityCommandAPI(drone=None, config=cfg, yaw_alpha=0.1)
        loop = asyncio.get_event_loop()

        r = loop.run_until_complete(
            api.send(VelocityCommand(0.0, 0.0, 0.0, 100.0)))
        # alpha=0.1: filtered = 0.1 * 100 = 10
        assert r.yawspeed_deg_s == pytest.approx(10.0)


class TestVelocityCommandAPISendZero:

    def test_send_zero_resets_filter(self):
        cfg = ControllerConfig(smooth_yaw=True, yaw_alpha=0.5, max_yawspeed=200.0)
        api = VelocityCommandAPI(drone=None, config=cfg)
        loop = asyncio.get_event_loop()

        # Build up filter state
        for _ in range(5):
            loop.run_until_complete(
                api.send(VelocityCommand(0.0, 0.0, 0.0, 80.0)))
        assert api._filtered_yaw != 0.0

        loop.run_until_complete(api.send_zero())
        assert api._filtered_yaw == 0.0

    def test_reset_filter_zeroes_state(self):
        cfg = ControllerConfig(smooth_yaw=True, yaw_alpha=0.5, max_yawspeed=200.0)
        api = VelocityCommandAPI(drone=None, config=cfg)
        loop = asyncio.get_event_loop()

        loop.run_until_complete(
            api.send(VelocityCommand(0.0, 0.0, 0.0, 50.0)))
        api.reset_filter()
        assert api._filtered_yaw == 0.0

        # After reset, first sample should start from 0 again
        r = loop.run_until_complete(
            api.send(VelocityCommand(0.0, 0.0, 0.0, 100.0)))
        assert r.yawspeed_deg_s == pytest.approx(50.0)  # 0.5 * 100 + 0.5 * 0


# ---------------------------------------------------------------------------
# ForwardSmoother
# ---------------------------------------------------------------------------

class TestForwardSmootherEMA:
    """EMA smoothing of forward velocity."""

    def test_first_call_with_no_detection_returns_smoothed_raw(self):
        s = ForwardSmoother()
        cfg = ControllerConfig(forward_alpha=0.5, kd_forward=0.0)
        result = s.update(None, 2.0, cfg)
        # No prior state, no detection: target = raw + 0 = 2.0
        # smoothed = 0.5 * 2.0 + 0.5 * 0.0 = 1.0
        assert result == pytest.approx(1.0)

    def test_ema_converges_to_constant_input(self):
        s = ForwardSmoother()
        cfg = ControllerConfig(forward_alpha=0.3, kd_forward=0.0)
        for _ in range(100):
            result = s.update(None, 1.5, cfg)
        assert result == pytest.approx(1.5, abs=0.01)

    def test_high_alpha_responds_faster(self):
        s_fast = ForwardSmoother()
        s_slow = ForwardSmoother()
        cfg_fast = ControllerConfig(forward_alpha=0.9, kd_forward=0.0)
        cfg_slow = ControllerConfig(forward_alpha=0.1, kd_forward=0.0)

        r_fast = s_fast.update(None, 2.0, cfg_fast)
        r_slow = s_slow.update(None, 2.0, cfg_slow)
        assert r_fast > r_slow

    def test_output_clamped_to_max_forward(self):
        s = ForwardSmoother()
        cfg = ControllerConfig(forward_alpha=1.0, kd_forward=0.0, max_forward=1.0)
        result = s.update(None, 5.0, cfg)
        assert result <= 1.0 + 0.01

    def test_output_clamped_to_max_backward(self):
        s = ForwardSmoother()
        cfg = ControllerConfig(forward_alpha=1.0, kd_forward=0.0, max_backward=2.0)
        result = s.update(None, -10.0, cfg)
        assert result >= -2.0 - 0.01


class TestForwardSmootherDerivative:
    """Derivative feed-forward based on bbox height rate of change."""

    def _det_at(self, bh, ts):
        return Detection(
            label="test", confidence=0.9,
            center_x=0.5, center_y=0.5, bbox_height=bh,
            timestamp=ts,
        )

    def test_growing_bbox_produces_negative_derivative(self):
        """Person approaching (bbox growing) -> derivative pushes backward."""
        s = ForwardSmoother()
        cfg = ControllerConfig(forward_alpha=1.0, kd_forward=2.0)

        t0 = time.monotonic()
        with patch("time.monotonic", return_value=t0):
            s.update(self._det_at(0.2, t0), 0.0, cfg)

        with patch("time.monotonic", return_value=t0 + 0.1):
            result = s.update(self._det_at(0.4, t0 + 0.1), 0.0, cfg)

        # bbox grew by 0.2 in 0.1s -> instant rate = 2.0
        # derivative = -kd * rate = -2.0 * (smoothed rate)
        # Should be negative (backward)
        assert result < 0.0

    def test_shrinking_bbox_produces_positive_derivative(self):
        """Person receding (bbox shrinking) -> derivative pushes forward."""
        s = ForwardSmoother()
        cfg = ControllerConfig(forward_alpha=1.0, kd_forward=2.0)

        t0 = time.monotonic()
        with patch("time.monotonic", return_value=t0):
            s.update(self._det_at(0.4, t0), 0.0, cfg)

        with patch("time.monotonic", return_value=t0 + 0.1):
            result = s.update(self._det_at(0.2, t0 + 0.1), 0.0, cfg)

        assert result > 0.0

    def test_zero_kd_disables_derivative(self):
        s = ForwardSmoother()
        cfg = ControllerConfig(forward_alpha=1.0, kd_forward=0.0)

        t0 = time.monotonic()
        with patch("time.monotonic", return_value=t0):
            s.update(self._det_at(0.2, t0), 1.0, cfg)

        with patch("time.monotonic", return_value=t0 + 0.1):
            result = s.update(self._det_at(0.5, t0 + 0.1), 1.0, cfg)

        # With alpha=1.0 and kd=0, output should equal raw
        assert result == pytest.approx(1.0)


class TestForwardSmootherDecayAndReset:

    def test_lost_detection_decays_rate(self):
        s = ForwardSmoother()
        cfg = ControllerConfig(forward_alpha=1.0, kd_forward=2.0)

        t0 = time.monotonic()
        with patch("time.monotonic", return_value=t0):
            s.update(self._det_at(0.2, t0), 0.0, cfg)
        with patch("time.monotonic", return_value=t0 + 0.1):
            s.update(self._det_at(0.4, t0 + 0.1), 0.0, cfg)

        rate_before = s._bbox_h_rate
        assert rate_before != 0.0

        # Lose detection: rate should decay by 0.9 each call
        with patch("time.monotonic", return_value=t0 + 0.2):
            s.update(None, 0.0, cfg)
        assert s._bbox_h_rate == pytest.approx(rate_before * 0.9)

    def test_reset_clears_all_state(self):
        s = ForwardSmoother()
        cfg = ControllerConfig(forward_alpha=0.5, kd_forward=2.0)
        s.update(None, 5.0, cfg)
        assert s._smoothed_forward != 0.0

        s.reset()
        assert s._smoothed_forward == 0.0
        assert s._prev_bbox_h is None
        assert s._prev_time is None
        assert s._bbox_h_rate == 0.0

    def _det_at(self, bh, ts):
        return Detection(
            label="test", confidence=0.9,
            center_x=0.5, center_y=0.5, bbox_height=bh,
            timestamp=ts,
        )


# ---------------------------------------------------------------------------
# _effective_target_bbox_height and _distance_to_bbox_height
# ---------------------------------------------------------------------------

class TestDistanceToBboxHeight:

    def test_closer_distance_gives_larger_bbox(self):
        near = _distance_to_bbox_height(3.0, 3.0, 41.0)
        far = _distance_to_bbox_height(3.0, 10.0, 41.0)
        assert near > far

    def test_higher_altitude_gives_smaller_bbox(self):
        low = _distance_to_bbox_height(2.0, 8.0, 41.0)
        high = _distance_to_bbox_height(10.0, 8.0, 41.0)
        assert low > high

    def test_taller_person_gives_larger_bbox(self):
        short = _distance_to_bbox_height(3.0, 8.0, 41.0, person_height_m=1.5)
        tall = _distance_to_bbox_height(3.0, 8.0, 41.0, person_height_m=2.0)
        assert tall > short

    def test_wider_fov_gives_smaller_bbox(self):
        narrow = _distance_to_bbox_height(3.0, 8.0, 30.0)
        wide = _distance_to_bbox_height(3.0, 8.0, 60.0)
        assert narrow > wide

    def test_result_is_positive_and_bounded(self):
        result = _distance_to_bbox_height(3.0, 8.0, 41.0)
        assert 0.0 < result < 1.0


class TestEffectiveTargetBboxHeight:

    def test_distance_mode_used_when_target_distance_set(self):
        cfg = ControllerConfig(target_distance_m=8.0, vfov=41.0, person_height_m=1.7, fixed_altitude=True)
        result = _effective_target_bbox_height(cfg, 3.0)
        expected = _distance_to_bbox_height(3.0, 8.0, 41.0, 1.7)
        assert result == pytest.approx(expected)

    def test_altitude_scaling_when_no_target_distance(self):
        cfg = ControllerConfig(
            target_distance_m=None,
            target_bbox_height=0.3,
            reference_altitude_m=3.0,
        )
        # At reference altitude: effective = (3.0 * 0.3) / 3.0 = 0.3
        assert _effective_target_bbox_height(cfg, 3.0) == pytest.approx(0.3)

        # At double altitude: effective = (3.0 * 0.3) / 6.0 = 0.15
        assert _effective_target_bbox_height(cfg, 6.0) == pytest.approx(0.15)

        # At half altitude: effective = (3.0 * 0.3) / 1.5 = 0.6
        assert _effective_target_bbox_height(cfg, 1.5) == pytest.approx(0.6)

    def test_max_target_clamp(self):
        cfg = ControllerConfig(
            target_distance_m=None,
            target_bbox_height=0.3,
            reference_altitude_m=3.0,
        )
        # Very low altitude -> would give huge bbox target, should be clamped
        result = _effective_target_bbox_height(cfg, 0.1)
        assert result <= 0.9

    def test_min_altitude_floor(self):
        cfg = ControllerConfig(
            target_distance_m=None,
            target_bbox_height=0.3,
            reference_altitude_m=3.0,
        )
        # Altitude 0 or negative should be floored to min_altitude_m (0.5)
        result_zero = _effective_target_bbox_height(cfg, 0.0)
        result_neg = _effective_target_bbox_height(cfg, -1.0)
        result_min = _effective_target_bbox_height(cfg, 0.5)
        assert result_zero == pytest.approx(result_min)
        assert result_neg == pytest.approx(result_min)

    def test_distance_mode_higher_altitude_smaller_target(self):
        cfg = ControllerConfig(target_distance_m=8.0, vfov=41.0, fixed_altitude=True)
        low = _effective_target_bbox_height(cfg, 2.0)
        high = _effective_target_bbox_height(cfg, 10.0)
        assert low > high

    def test_distance_mode_clamped_to_max_target(self):
        cfg = ControllerConfig(target_distance_m=0.5, vfov=41.0, person_height_m=1.7, fixed_altitude=True)
        result = _effective_target_bbox_height(cfg, 1.0, max_target=0.9)
        assert result <= 0.9

    def test_target_distance_zero_falls_through_to_altitude_scaling(self):
        cfg = ControllerConfig(
            target_distance_m=0,
            target_bbox_height=0.3,
            reference_altitude_m=3.0,
            fixed_altitude=True,
        )
        result = _effective_target_bbox_height(cfg, 3.0)
        # target_distance_m=0 -> condition is falsy -> altitude scaling
        assert result == pytest.approx(0.3)
