"""Tests for the FOV-aware proportional controller.

Migrated to the new ``controller.compute(detection, caps, config)`` signature
in Phase 3 plan 03-07. Search-mode + hold-velocity tests were removed —
those concerns are orchestrator-side now (covered in
test_robot_command_snapshot.py for behavioral equivalence).

The 7 VelocityCommandAPI tests (TestForwardLowPass) that exercised the
deleted legacy smoother were removed; equivalent coverage lives in
test_mavsdk_drone_adapter.py::TestApplySmoothing.

Edge-zone behavior tests (TestFrameEdgeSafety) were moved to
test_mavsdk_drone_adapter.py::TestApplyRetreatFromTilt because the
fade/push gradient now lives in the adapter (``_apply_retreat_from_tilt``).
The controller emits raw clamped distance-P only.
"""

import time
from types import SimpleNamespace

import pytest

from robot_follow.follow_api import (
    Detection,
    ControllerConfig,
    RobotCommand,
    compute,
)
from robot_follow.robot_api.adapters.mavsdk_drone import DRONE_CAPS


def _det(cx=0.5, cy=0.5, bh=0.3):
    """Helper to create a Detection at given normalized coords."""
    return Detection(
        label="test", confidence=0.9,
        center_x=cx, center_y=cy, bbox_height=bh,
        timestamp=time.monotonic(),
    )


@pytest.fixture
def config():
    """Default config with yaw_only=False so distance-mode forward + frame-edge
    safety can fire in the tests that exercise them."""
    return ControllerConfig(yaw_only=False)


# ---- Yaw (horizontal centering) ----

class TestYaw:
    def test_centered_within_dead_zone(self, config):
        """Detection near center -> zero yaw (dead zone)."""
        cmd = compute(_det(cx=0.51), DRONE_CAPS, config)
        assert cmd.yaw_rate == 0.0

    def test_target_right_positive_yaw(self, config):
        """Detection right of center -> positive yaw (clockwise)."""
        cmd = compute(_det(cx=0.75), DRONE_CAPS, config)
        assert cmd.yaw_rate > 0.0

    def test_target_left_negative_yaw(self, config):
        """Detection left of center -> negative yaw (counter-clockwise)."""
        cmd = compute(_det(cx=0.25), DRONE_CAPS, config)
        assert cmd.yaw_rate < 0.0

    def test_symmetry(self, config):
        """Equal offsets left and right should produce equal magnitude."""
        cmd_right = compute(_det(cx=0.7), DRONE_CAPS, config)
        cmd_left = compute(_det(cx=0.3), DRONE_CAPS, config)
        assert abs(cmd_right.yaw_rate + cmd_left.yaw_rate) < 0.01

    def test_yaw_saturation(self, config):
        """Extreme offset should be clamped to max_yawspeed."""
        cmd = compute(_det(cx=1.0), DRONE_CAPS, config)
        assert abs(cmd.yaw_rate) <= config.max_yawspeed + 0.01

    def test_fov_scaling(self):
        """Wider FOV with same pixel offset -> larger angular error -> larger yaw rate."""
        narrow = ControllerConfig(hfov=60.0)
        wide = ControllerConfig(hfov=120.0)
        det = _det(cx=0.7)
        cmd_narrow = compute(det, DRONE_CAPS, narrow)
        cmd_wide = compute(det, DRONE_CAPS, wide)
        assert abs(cmd_wide.yaw_rate) > abs(cmd_narrow.yaw_rate)

    def test_fov_proportional(self):
        """Double the FOV should double the angular error and thus the yaw rate
        (when not saturated)."""
        cfg_a = ControllerConfig(hfov=40.0, max_yawspeed=9999.0)
        cfg_b = ControllerConfig(hfov=80.0, max_yawspeed=9999.0)
        det = _det(cx=0.6)  # small offset to stay in linear region
        cmd_a = compute(det, DRONE_CAPS, cfg_a)
        cmd_b = compute(det, DRONE_CAPS, cfg_b)
        ratio = cmd_b.yaw_rate / cmd_a.yaw_rate
        # Yaw controller uses sqrt(|error_x_deg|), so doubling FOV scales by sqrt(2).
        assert abs(ratio - (2.0 ** 0.5)) < 0.01


# ---- Combined scenarios ----

class TestCombined:
    def test_perfectly_centered_at_target(self, config):
        """Target perfectly centered (cy=0.5) and at desired bbox height -> all zeros."""
        cmd = compute(
            _det(cx=0.5, cy=0.5, bh=config.target_bbox_height),
            DRONE_CAPS, config,
        )
        assert cmd.forward_m_s == 0.0
        assert cmd.down_m_s == 0.0
        assert cmd.yaw_rate == 0.0

    def test_yaw_and_forward_active_together(self):
        """Target off-center horizontally with bbox below target -> both yaw and forward active."""
        config = ControllerConfig(dead_zone_deg=0.0, yaw_only=False)
        # bh=0.15 is well below target_bbox_height=0.25 → factor>0 → forward
        cmd = compute(
            _det(cx=0.7, cy=0.5, bh=0.15), DRONE_CAPS, config,
        )
        assert cmd.yaw_rate > 0.0    # right -> positive yaw
        assert cmd.forward_m_s > 0.0  # bbox too small -> approach


class TestSafetyAndFollowing:
    def test_yaw_only_keeps_yaw_and_disables_forward_and_down(self):
        """Yaw-only mode still tracks yaw but zeroes forward and down commands."""
        cfg = ControllerConfig(yaw_only=True, dead_zone_deg=0.0)
        cmd = compute(_det(cx=0.8, cy=0.2, bh=0.1), DRONE_CAPS, cfg)
        assert cmd.yaw_rate > 0.0
        assert cmd.forward_m_s == 0.0
        assert cmd.down_m_s == 0.0


class TestConfigArgs:
    def test_log_verbosity_defaults_to_normal(self):
        cfg = ControllerConfig.from_args(SimpleNamespace())
        assert cfg.log_verbosity == "normal"

    def test_log_verbosity_is_read_from_args(self):
        cfg = ControllerConfig.from_args(SimpleNamespace(log_verbosity="debug"))
        assert cfg.log_verbosity == "debug"


# ---- Config validation ----

class TestConfigValidation:
    def test_default_config_is_valid(self):
        """Default config should pass validation."""
        ControllerConfig().validate()

    def test_min_altitude_must_be_less_than_max(self):
        """min_altitude >= max_altitude should raise ValueError."""
        with pytest.raises(ValueError, match="min_altitude"):
            ControllerConfig(min_altitude=25.0, max_altitude=20.0)

    def test_min_altitude_equals_max_is_invalid(self):
        with pytest.raises(ValueError, match="min_altitude"):
            ControllerConfig(min_altitude=10.0, max_altitude=10.0)

    def test_valid_altitude_range(self):
        """A valid altitude range should not raise."""
        ControllerConfig(min_altitude=1.0, max_altitude=50.0).validate()

    def test_target_altitude_must_be_at_most_max_altitude(self):
        with pytest.raises(ValueError, match="target_altitude"):
            ControllerConfig(target_altitude=5.0, max_altitude=4.0)

    def test_target_altitude_at_max_is_valid(self):
        ControllerConfig(target_altitude=4.0, max_altitude=4.0).validate()

    def test_target_altitude_below_min_raises(self):
        with pytest.raises(ValueError, match="target_altitude"):
            ControllerConfig(target_altitude=1.0, min_altitude=2.0, max_altitude=4.0)

    def test_validate_skips_altitude_when_axis_absent(self):
        """ABS-07: validate(caps) skips altitude checks when ALTITUDE not in caps.axes.

        A rover-style Capabilities with altitude=None fields would normally
        fail; with caps passed in, the relationship check is skipped.
        """
        from robot_follow.follow_api.types import Axis, Capabilities
        rover_caps = Capabilities(
            axes=frozenset({Axis.FORWARD, Axis.YAW}), yaw_unit="rad/s",
        )
        # Build a config with default altitudes (validates fine), then drop them
        # and re-validate with rover caps — should not raise.
        cfg = ControllerConfig()
        cfg.min_altitude = None
        cfg.max_altitude = None
        cfg.target_altitude = None
        cfg.validate(caps=rover_caps)  # no exception expected


class TestDistanceForward:
    """bbox_height drives forward speed (distance control). Altitude is held
    by the adapter (_apply_altitude_p), so the controller emits down=0."""

    def test_at_target_bbox_zero_forward(self, config):
        """Bbox at target -> no forward command."""
        cmd = compute(_det(bh=config.target_bbox_height), DRONE_CAPS, config)
        assert cmd.forward_m_s == 0.0
        assert cmd.down_m_s == 0.0

    def test_small_bbox_approaches(self, config):
        """Person far (bbox < target) -> forward (positive)."""
        cmd = compute(_det(bh=0.1), DRONE_CAPS, config)
        assert cmd.forward_m_s > 0.0
        assert cmd.down_m_s == 0.0

    def test_large_bbox_retreats(self, config):
        """Person close (bbox > target) -> backup (negative)."""
        cmd = compute(_det(bh=0.6), DRONE_CAPS, config)
        assert cmd.forward_m_s < 0.0
        assert cmd.down_m_s == 0.0

    def test_altitude_always_zero(self, config):
        """down_m_s must remain 0 across the bbox range (M4 invariant)."""
        for bh in (0.05, 0.2, config.target_bbox_height, 0.5, 0.7):
            cmd = compute(_det(bh=bh), DRONE_CAPS, config)
            assert cmd.down_m_s == 0.0, f"down should be 0 (bh={bh})"

    def test_emergency_bbox_no_climb(self, config):
        """Bbox > safety threshold -> max backward but no emergency climb."""
        cmd = compute(_det(bh=0.9), DRONE_CAPS, config)
        assert cmd.forward_m_s == -config.max_backward
        assert cmd.down_m_s == 0.0

    def test_clamped_to_max_forward(self):
        """Very small bbox with a high gain should saturate at max_forward."""
        # Edge-safety happens in the adapter now; here we only test the
        # controller's clamp behavior, so margins are zeroed.
        cfg = ControllerConfig(yaw_only=False, kp_distance=100.0,
                               top_margin_safety=0.0,
                               bottom_margin_safety=0.0)
        cmd = compute(_det(bh=0.001), DRONE_CAPS, cfg)
        assert cmd.forward_m_s == cfg.max_forward

    def test_yaw_only_overrides_mode(self):
        """yaw_only=True still wins -> all axes zero except yaw."""
        cfg = ControllerConfig(yaw_only=True, dead_zone_deg=0.0)
        cmd = compute(_det(cx=0.8, bh=0.1), DRONE_CAPS, cfg)
        assert cmd.forward_m_s == 0.0
        assert cmd.down_m_s == 0.0
        assert cmd.yaw_rate > 0.0

    def test_dead_zone_holds_zero(self, config):
        """Bbox within the bbox dead zone -> no forward command."""
        cmd = compute(
            _det(bh=config.target_bbox_height * 1.05), DRONE_CAPS, config,
        )
        assert cmd.forward_m_s == 0.0

    def test_asymmetric_retreat_uses_kp_distance_back(self):
        """Retreat (factor<0) uses kp_distance_back; approach uses kp_distance.

        With kp_distance=1.0, kp_distance_back=3.0 and target=0.3:
          bh=0.6 → factor=0.3/0.6-1 = -0.5 → raw = 3.0 * -0.5 = -1.5 (retreat)
          bh=0.2 → factor=0.3/0.2-1 = +0.5 → raw = 1.0 * +0.5 = +0.5 (approach)
        Same |factor|, different magnitudes thanks to the asymmetry.
        """
        cfg = ControllerConfig(
            yaw_only=False, target_bbox_height=0.3,
            kp_distance=1.0, kp_distance_back=3.0,
            max_forward=5.0, max_backward=5.0, dead_zone_bbox_percent=0.0,
            top_margin_safety=0.0, bottom_margin_safety=0.0,
        )
        retreat = compute(_det(bh=0.6), DRONE_CAPS, cfg)
        approach = compute(_det(bh=0.2), DRONE_CAPS, cfg)
        assert retreat.forward_m_s == pytest.approx(-1.5, abs=1e-6)
        assert approach.forward_m_s == pytest.approx(0.5, abs=1e-6)
        assert abs(retreat.forward_m_s) == pytest.approx(3.0 * approach.forward_m_s, abs=1e-6)

    def test_emergency_safety_branch(self):
        """The bbox > max_bbox_height_safety branch returns max_backward
        regardless of other config (yaw stays active)."""
        cfg = ControllerConfig(yaw_only=False, target_bbox_height=0.30)
        cmd = compute(_det(bh=0.95), DRONE_CAPS, cfg)  # > 0.8 safety
        assert cmd.forward_m_s == -cfg.max_backward


class TestRobotCommandShape:
    """Sanity: controller returns a RobotCommand, not a tuple or VelocityCommand."""

    def test_returns_robot_command(self, config):
        cmd = compute(_det(), DRONE_CAPS, config)
        assert isinstance(cmd, RobotCommand)
        assert hasattr(cmd, "forward_m_s")
        assert hasattr(cmd, "yaw_rate")
        assert hasattr(cmd, "down_m_s")
