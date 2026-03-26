"""Tests for the FOV-aware proportional controller."""

import time
from types import SimpleNamespace

import pytest

from drone_follow.follow_api import (
    Detection,
    ControllerConfig,
    compute_velocity_command,
    reset_forward_dead_zone,
)


def _det(cx=0.5, cy=0.5, bh=0.3):
    """Helper to create a Detection at given normalized coords."""
    return Detection(
        label="test", confidence=0.9,
        center_x=cx, center_y=cy, bbox_height=bh,
        timestamp=time.monotonic(),
    )


@pytest.fixture(autouse=True)
def _reset_dead_zone():
    """Reset hysteresis dead zone state before each test."""
    reset_forward_dead_zone()
    yield
    reset_forward_dead_zone()


@pytest.fixture
def config():
    """Default config with yaw_only=False for tests that need full movement."""
    return ControllerConfig(yaw_only=False)


# ---- No detection (search mode) ----

class TestSearchMode:
    def test_no_detection_returns_search_yaw(self, config):
        cmd = compute_velocity_command(None, config)
        assert cmd.yawspeed_deg_s == config.search_yawspeed_slow

    def test_no_detection_zero_velocity(self, config):
        cmd = compute_velocity_command(None, config)
        assert cmd.forward_m_s == 0.0
        assert cmd.right_m_s == 0.0
        assert cmd.down_m_s == 0.0


# ---- Yaw (horizontal centering) ----

class TestYaw:
    def test_centered_within_dead_zone(self, config):
        """Detection near center -> zero yaw (dead zone)."""
        cmd = compute_velocity_command(_det(cx=0.51), config)
        assert cmd.yawspeed_deg_s == 0.0

    def test_target_right_positive_yaw(self, config):
        """Detection right of center -> positive yaw (clockwise)."""
        cmd = compute_velocity_command(_det(cx=0.75), config)
        assert cmd.yawspeed_deg_s > 0.0

    def test_target_left_negative_yaw(self, config):
        """Detection left of center -> negative yaw (counter-clockwise)."""
        cmd = compute_velocity_command(_det(cx=0.25), config)
        assert cmd.yawspeed_deg_s < 0.0

    def test_symmetry(self, config):
        """Equal offsets left and right should produce equal magnitude."""
        cmd_right = compute_velocity_command(_det(cx=0.7), config)
        cmd_left = compute_velocity_command(_det(cx=0.3), config)
        assert abs(cmd_right.yawspeed_deg_s + cmd_left.yawspeed_deg_s) < 0.01

    def test_yaw_saturation(self, config):
        """Extreme offset should be clamped to max_yawspeed."""
        cmd = compute_velocity_command(_det(cx=1.0), config)
        assert abs(cmd.yawspeed_deg_s) <= config.max_yawspeed + 0.01

    def test_fov_scaling(self):
        """Wider FOV with same pixel offset -> larger angular error -> larger yaw rate."""
        narrow = ControllerConfig(hfov=60.0)
        wide = ControllerConfig(hfov=120.0)
        det = _det(cx=0.7)
        cmd_narrow = compute_velocity_command(det, narrow)
        cmd_wide = compute_velocity_command(det, wide)
        assert abs(cmd_wide.yawspeed_deg_s) > abs(cmd_narrow.yawspeed_deg_s)

    def test_fov_proportional(self):
        """Double the FOV should double the angular error and thus the yaw rate
        (when not saturated)."""
        cfg_a = ControllerConfig(hfov=40.0, max_yawspeed=9999.0)
        cfg_b = ControllerConfig(hfov=80.0, max_yawspeed=9999.0)
        det = _det(cx=0.6)  # small offset to stay in linear region
        cmd_a = compute_velocity_command(det, cfg_a)
        cmd_b = compute_velocity_command(det, cfg_b)
        ratio = cmd_b.yawspeed_deg_s / cmd_a.yawspeed_deg_s
        # Yaw controller uses sqrt(|error_x_deg|), so doubling FOV scales by sqrt(2).
        assert abs(ratio - (2.0 ** 0.5)) < 0.01


# ---- Altitude (vertical centering) ----
# Default config has fixed_altitude=True; tests below explicitly set False where needed.

class TestAltitude:
    def test_centered_within_dead_zone(self, config):
        cmd = compute_velocity_command(_det(cy=0.51), config)
        assert cmd.down_m_s == 0.0

    def test_target_below_positive_down(self):
        """Target below center -> fly down (positive down_m_s)."""
        config = ControllerConfig(fixed_altitude=False, target_distance_m=None, yaw_only=False)
        cmd = compute_velocity_command(_det(cy=0.75), config)
        assert cmd.down_m_s > 0.0

    def test_target_above_negative_down(self):
        """Target above center -> fly up (negative down_m_s)."""
        config = ControllerConfig(fixed_altitude=False, target_distance_m=None, yaw_only=False)
        cmd = compute_velocity_command(_det(cy=0.25), config)
        assert cmd.down_m_s < 0.0

    def test_altitude_saturation(self):
        config = ControllerConfig(fixed_altitude=False, target_distance_m=None, yaw_only=False)
        cmd = compute_velocity_command(_det(cy=1.0), config)
        assert abs(cmd.down_m_s) <= config.max_down_speed + 0.01

    def test_vfov_scaling(self):
        """Wider vertical FOV -> larger altitude command for same pixel offset."""
        narrow = ControllerConfig(vfov=30.0, fixed_altitude=False, target_distance_m=None, yaw_only=False)
        wide = ControllerConfig(vfov=90.0, fixed_altitude=False, target_distance_m=None, yaw_only=False)
        det = _det(cy=0.7)
        cmd_narrow = compute_velocity_command(det, narrow)
        cmd_wide = compute_velocity_command(det, wide)
        assert abs(cmd_wide.down_m_s) > abs(cmd_narrow.down_m_s)


# ---- Forward/backward (distance via bbox height) ----

class TestForward:
    def test_at_target_height_in_dead_zone(self, config):
        """Bbox height == target -> no forward movement (dead zone)."""
        cmd = compute_velocity_command(
            _det(bh=config.target_bbox_height), config
        )
        assert cmd.forward_m_s == 0.0

    def test_small_bbox_forward(self, config):
        """Small bbox (far away) -> fly forward."""
        cmd = compute_velocity_command(_det(bh=0.1), config)
        assert cmd.forward_m_s > 0.0

    def test_large_bbox_backward(self, config):
        """Large bbox (too close) -> fly backward."""
        cmd = compute_velocity_command(_det(bh=0.6), config)
        assert cmd.forward_m_s < 0.0

    def test_forward_saturation(self, config):
        """Very small bbox -> clamped to max_forward."""
        cmd = compute_velocity_command(_det(bh=0.01), config)
        assert cmd.forward_m_s <= config.max_forward + 0.01

    def test_backward_saturation(self, config):
        """Very large bbox -> clamped to max_backward."""
        cmd = compute_velocity_command(_det(bh=0.95), config)
        assert cmd.forward_m_s >= -config.max_backward - 0.01

    def test_height_dead_zone(self, config):
        """Bbox slightly off target but within dead zone -> zero forward."""
        dead_zone = (config.dead_zone_height_percent / 100.0) * config.target_bbox_height
        small_offset = dead_zone * 0.5
        cmd = compute_velocity_command(
            _det(bh=config.target_bbox_height + small_offset), config
        )
        assert cmd.forward_m_s == 0.0

    def test_right_always_zero_in_follow_mode(self, config):
        """right_m_s should always be zero in follow mode (no lateral movement)."""
        config.follow_mode = "follow"
        for cx in [0.1, 0.5, 0.9]:
            for cy in [0.1, 0.5, 0.9]:
                for bh in [0.1, 0.3, 0.8]:
                    cmd = compute_velocity_command(_det(cx=cx, cy=cy, bh=bh), config)
                    assert cmd.right_m_s == 0.0


# ---- Combined scenarios ----

class TestCombined:
    def test_perfectly_centered_at_target_distance(self, config):
        """Target perfectly centered and at desired distance -> all zeros."""
        cmd = compute_velocity_command(
            _det(cx=0.5, cy=0.5, bh=config.target_bbox_height), config
        )
        assert cmd.forward_m_s == 0.0
        assert cmd.right_m_s == 0.0
        assert cmd.down_m_s == 0.0
        assert cmd.yawspeed_deg_s == 0.0

    def test_all_axes_active(self):
        """Target off-center in all axes simultaneously."""
        config = ControllerConfig(dead_zone_deg=0.0, dead_zone_height_percent=0.0,
                                  fixed_altitude=False, target_distance_m=None, yaw_only=False)
        cmd = compute_velocity_command(
            _det(cx=0.7, cy=0.3, bh=0.15), config
        )
        assert cmd.yawspeed_deg_s > 0.0    # right -> positive yaw
        assert cmd.down_m_s < 0.0           # above center -> fly up
        assert cmd.forward_m_s > 0.0        # small bbox -> approach

    def test_custom_gains(self):
        """Custom gain values should scale the output proportionally."""
        cfg_low = ControllerConfig(
            kp_yaw=1.0, kp_down=0.04, kp_forward=1.5,
            dead_zone_deg=0.0, dead_zone_height_percent=0.0,
            fixed_altitude=False, target_distance_m=None, yaw_only=False,
            max_yawspeed=9999.0, max_down_speed=9999.0,
            max_forward=9999.0, max_backward=9999.0,
        )
        cfg_high = ControllerConfig(
            kp_yaw=2.0, kp_down=0.08, kp_forward=3.0,
            dead_zone_deg=0.0, dead_zone_height_percent=0.0,
            fixed_altitude=False, target_distance_m=None, yaw_only=False,
            max_yawspeed=9999.0, max_down_speed=9999.0,
            max_forward=9999.0, max_backward=9999.0,
        )
        # Keep bbox away from bottom-of-frame safety path so kp_forward scaling is isolated.
        det = _det(cx=0.65, cy=0.6, bh=0.15)
        cmd_low = compute_velocity_command(det, cfg_low)
        cmd_high = compute_velocity_command(det, cfg_high)

        assert abs(cmd_high.yawspeed_deg_s / cmd_low.yawspeed_deg_s - 2.0) < 0.01
        assert abs(cmd_high.down_m_s / cmd_low.down_m_s - 2.0) < 0.01
        assert abs(cmd_high.forward_m_s / cmd_low.forward_m_s - 2.0) < 0.01


class TestSafetyAndFollowing:
    def test_safety_retreat_overrides_forward_logic(self):
        """Over safety bbox threshold should force max backward retreat."""
        cfg = ControllerConfig(
            target_bbox_height=0.3,
            max_bbox_height_safety=0.6,
            dead_zone_height_percent=50.0,
            yaw_only=False,
        )
        cmd = compute_velocity_command(_det(bh=0.75), cfg)
        assert cmd.forward_m_s == -cfg.max_backward

    def test_bottom_of_frame_triggers_backward(self):
        """When bbox bottom edge exceeds bottom_y_threshold, drone should retreat."""
        cfg = ControllerConfig(
            target_bbox_height=0.3,
            dead_zone_height_percent=30.0,
            bottom_y_threshold=0.7,
            yaw_only=False,
        )
        # cy=0.8, bh=0.3 -> bbox_bottom = 0.95, well above 0.7 threshold
        cmd = compute_velocity_command(_det(cy=0.8, bh=0.3), cfg)
        assert cmd.forward_m_s < 0.0

    def test_bottom_of_frame_no_retreat_when_above_threshold(self):
        """When bbox bottom edge is above threshold, normal forward logic applies."""
        cfg = ControllerConfig(
            target_bbox_height=0.3,
            dead_zone_height_percent=30.0,
            bottom_y_threshold=0.7,
            yaw_only=False,
        )
        # cy=0.4, bh=0.3 -> bbox_bottom = 0.55, below 0.7 threshold
        # bh matches target within dead zone -> forward = 0
        cmd = compute_velocity_command(_det(cy=0.4, bh=0.3), cfg)
        assert cmd.forward_m_s == 0.0

    def test_bottom_of_frame_ignored_in_yaw_only(self):
        """Yaw-only mode should not trigger bottom-of-frame backward."""
        cfg = ControllerConfig(
            bottom_y_threshold=0.7,
            yaw_only=True,
        )
        # bbox_bottom = 0.95, but yaw_only -> forward stays 0
        cmd = compute_velocity_command(_det(cy=0.8, bh=0.3), cfg)
        assert cmd.forward_m_s == 0.0

    def test_yaw_only_keeps_yaw_and_disables_altitude_and_forward(self):
        """Yaw-only mode still tracks yaw but zeroes forward/down commands."""
        cfg = ControllerConfig(yaw_only=True, fixed_altitude=False, target_distance_m=None, dead_zone_deg=0.0)
        cmd = compute_velocity_command(_det(cx=0.8, cy=0.2, bh=0.1), cfg)
        assert cmd.yawspeed_deg_s > 0.0
        assert cmd.forward_m_s == 0.0
        assert cmd.down_m_s == 0.0

    def test_search_spins_toward_last_seen_side(self):
        """When target is lost, search yaw direction should follow last known side."""
        cfg = ControllerConfig()
        last_right = _det(cx=0.8, bh=0.2)
        last_left = _det(cx=0.2, bh=0.2)
        cmd_right = compute_velocity_command(None, cfg, last_detection=last_right)
        cmd_left = compute_velocity_command(None, cfg, last_detection=last_left)
        assert cmd_right.yawspeed_deg_s > 0.0
        assert cmd_left.yawspeed_deg_s < 0.0

    def test_search_wait_holds_previous_velocity(self):
        """Before active search, controller should hold last velocity."""
        cfg = ControllerConfig()
        hold = compute_velocity_command(_det(cx=0.7, cy=0.5, bh=0.3), cfg)
        cmd = compute_velocity_command(
            None,
            cfg,
            search_active=False,
            hold_velocity=hold,
        )
        assert cmd.forward_m_s == hold.forward_m_s
        assert cmd.down_m_s == hold.down_m_s
        assert cmd.yawspeed_deg_s == hold.yawspeed_deg_s


class TestConfigArgs:
    def test_log_verbosity_defaults_to_normal(self):
        cfg = ControllerConfig.from_args(SimpleNamespace())
        assert cfg.log_verbosity == "normal"

    def test_log_verbosity_is_read_from_args(self):
        cfg = ControllerConfig.from_args(SimpleNamespace(log_verbosity="debug"))
        assert cfg.log_verbosity == "debug"


# ---- Validation: target_distance_m vs fixed_altitude / target_bbox_height ----

class TestConfigValidation:
    def test_default_config_is_valid(self):
        """Default config (ysing target-bbox-height + variable altitude) should pass validation."""
        cfg = ControllerConfig()
        cfg.validate()

    def test_distance_with_fixed_altitude_is_valid(self):
        cfg = ControllerConfig(target_distance_m=10.0, fixed_altitude=True)
        cfg.validate()

    def test_bbox_height_mode_with_variable_altitude_is_valid(self):
        cfg = ControllerConfig(target_distance_m=None, fixed_altitude=False)
        cfg.validate()

    def test_distance_without_fixed_altitude_raises(self):
        with pytest.raises(ValueError, match="target_distance_m requires fixed_altitude"):
            ControllerConfig(target_distance_m=8.0, fixed_altitude=False)

    def test_validate_catches_invalid_after_mutation(self):
        cfg = ControllerConfig(target_distance_m=8.0, fixed_altitude=True)
        cfg.fixed_altitude = False
        with pytest.raises(ValueError, match="target_distance_m requires fixed_altitude"):
            cfg.validate()


class TestConfigFromArgsMutualExclusivity:
    def test_defaults_use_bbox_height_mode(self):
        """No explicit args -> defaults to target_distance_m=None, fixed_altitude=True."""
        cfg = ControllerConfig.from_args(SimpleNamespace())
        assert cfg.target_distance_m is None
        assert cfg.fixed_altitude is True

    def test_explicit_bbox_height_disables_distance(self):
        """Passing --target-bbox-height should set target_distance_m=None."""
        cfg = ControllerConfig.from_args(SimpleNamespace(target_bbox_height=0.4))
        assert cfg.target_distance_m is None
        assert cfg.target_bbox_height == 0.4

    def test_explicit_distance_without_fixed_altitude_raises(self):
        """--target-distance with --no-fixed-altitude raises (invalid combination)."""
        with pytest.raises(ValueError, match="--target-distance requires --fixed-altitude"):
            ControllerConfig.from_args(SimpleNamespace(target_distance=12.0, fixed_altitude=False))

    def test_explicit_distance_with_fixed_altitude_keeps_distance_mode(self):
        cfg = ControllerConfig.from_args(SimpleNamespace(target_distance=12.0, fixed_altitude=True))
        assert cfg.target_distance_m == 12.0
        assert cfg.fixed_altitude is True

    def test_both_distance_and_bbox_raises(self):
        with pytest.raises(ValueError, match="mutually exclusive"):
            ControllerConfig.from_args(SimpleNamespace(
                target_distance=8.0, target_bbox_height=0.3,
            ))


class TestForwardHysteresis:
    """Tests for the hysteresis dead zone on forward/backward control."""

    def test_stays_in_dead_zone_below_exit_threshold(self):
        """Error below exit threshold keeps controller in dead zone."""
        cfg = ControllerConfig(
            yaw_only=False, dead_zone_height_percent=15.0,
            dead_zone_reenter_percent=8.0, target_bbox_height=0.3,
        )
        # 10% error = below 15% exit threshold -> stays in dead zone
        bh = cfg.target_bbox_height * 0.90  # 10% smaller
        cmd = compute_velocity_command(_det(bh=bh), cfg)
        assert cmd.forward_m_s == 0.0

    def test_exits_dead_zone_above_exit_threshold(self):
        """Error above exit threshold breaks out of dead zone."""
        cfg = ControllerConfig(
            yaw_only=False, dead_zone_height_percent=15.0,
            dead_zone_reenter_percent=8.0, target_bbox_height=0.3,
        )
        # 20% error = above 15% exit threshold -> should move
        bh = cfg.target_bbox_height * 0.80  # 20% smaller (far away)
        cmd = compute_velocity_command(_det(bh=bh), cfg)
        assert cmd.forward_m_s > 0.0

    def test_hysteresis_stays_active_between_thresholds(self):
        """After exiting dead zone, error between reenter and exit keeps moving."""
        cfg = ControllerConfig(
            yaw_only=False, dead_zone_height_percent=15.0,
            dead_zone_reenter_percent=8.0, target_bbox_height=0.3,
        )
        # First: break out with large error
        bh_far = cfg.target_bbox_height * 0.80  # 20% error
        cmd1 = compute_velocity_command(_det(bh=bh_far), cfg)
        assert cmd1.forward_m_s > 0.0

        # Now: error drops to 10% (between 8% reenter and 15% exit) -> still moving
        bh_mid = cfg.target_bbox_height * 0.90  # 10% error
        cmd2 = compute_velocity_command(_det(bh=bh_mid), cfg)
        assert cmd2.forward_m_s > 0.0, "Should stay active due to hysteresis"

    def test_hysteresis_reenters_dead_zone_below_reenter_threshold(self):
        """After exiting dead zone, error below reenter threshold stops movement."""
        cfg = ControllerConfig(
            yaw_only=False, dead_zone_height_percent=15.0,
            dead_zone_reenter_percent=8.0, target_bbox_height=0.3,
        )
        # First: break out with large error
        bh_far = cfg.target_bbox_height * 0.80
        compute_velocity_command(_det(bh=bh_far), cfg)

        # Now: error drops to 5% (below 8% reenter) -> back to dead zone
        bh_close = cfg.target_bbox_height * 0.95  # 5% error
        cmd = compute_velocity_command(_det(bh=bh_close), cfg)
        assert cmd.forward_m_s == 0.0, "Should re-enter dead zone"

    def test_ramp_produces_smooth_transition(self):
        """Command right at exit threshold should be smaller than well beyond it."""
        cfg = ControllerConfig(
            yaw_only=False, dead_zone_height_percent=15.0,
            dead_zone_reenter_percent=8.0, target_bbox_height=0.3,
        )
        exit_dz = (cfg.dead_zone_height_percent / 100.0) * cfg.target_bbox_height
        # Just past exit threshold
        bh_edge = cfg.target_bbox_height - exit_dz * 1.05
        cmd_edge = compute_velocity_command(_det(bh=bh_edge), cfg)

        # Well past ramp region
        reset_forward_dead_zone()
        bh_far = cfg.target_bbox_height - exit_dz * 3.0
        cmd_far = compute_velocity_command(_det(bh=bh_far), cfg)

        assert 0.0 < cmd_edge.forward_m_s < cmd_far.forward_m_s


class TestOrbitMode:
    def test_orbit_adds_lateral_velocity(self):
        """In orbit mode, tracking a target should produce lateral velocity."""
        cfg = ControllerConfig(follow_mode="orbit", orbit_speed_m_s=1.5, orbit_direction=1, yaw_only=False)
        cmd = compute_velocity_command(_det(cx=0.5, cy=0.5, bh=0.3), cfg)
        assert cmd.right_m_s == 1.5

    def test_orbit_ccw_negative_lateral(self):
        """Counter-clockwise orbit should produce negative lateral velocity."""
        cfg = ControllerConfig(follow_mode="orbit", orbit_speed_m_s=1.0, orbit_direction=-1, yaw_only=False)
        cmd = compute_velocity_command(_det(cx=0.5, cy=0.5, bh=0.3), cfg)
        assert cmd.right_m_s == -1.0

    def test_follow_mode_no_lateral(self):
        """In follow mode, there should be no lateral velocity."""
        cfg = ControllerConfig(follow_mode="follow", orbit_speed_m_s=2.0)
        cmd = compute_velocity_command(_det(cx=0.5, cy=0.5, bh=0.3), cfg)
        assert cmd.right_m_s == 0.0

    def test_search_mode_no_lateral_in_orbit(self):
        """In orbit mode, search (no detection) should have no lateral velocity."""
        cfg = ControllerConfig(follow_mode="orbit", orbit_speed_m_s=1.5)
        cmd = compute_velocity_command(None, cfg)
        assert cmd.right_m_s == 0.0

    def test_orbit_preserves_yaw_and_forward(self):
        """Orbit mode should still compute yaw and forward normally."""
        cfg = ControllerConfig(follow_mode="orbit", orbit_speed_m_s=1.0,
                               dead_zone_deg=0.0, dead_zone_height_percent=0.0, yaw_only=False)
        cmd = compute_velocity_command(_det(cx=0.7, cy=0.5, bh=0.15), cfg)
        assert cmd.yawspeed_deg_s > 0.0  # target right of center
        assert cmd.forward_m_s > 0.0     # small bbox -> approach
        assert cmd.right_m_s == 1.0      # lateral orbit velocity


class TestDirectionHoldFilter:
    """Tests for temporal hysteresis on forward/backward direction changes."""

    def test_same_direction_passes_through(self):
        from drone_follow.follow_api.controller import _DirectionHoldFilter
        filt = _DirectionHoldFilter()
        # First call sets direction and passes through immediately
        assert filt.filter(0.5, 0.0) == 0.5
        # Subsequent same-direction calls pass through
        assert filt.filter(0.5, 0.1) == 0.5
        assert filt.filter(0.3, 0.2) == 0.3

    def test_direction_change_held_back(self):
        from drone_follow.follow_api.controller import _DirectionHoldFilter
        filt = _DirectionHoldFilter()
        filt._last_sign = 1  # was going forward
        # Switch to backward — should be held
        assert filt.filter(-0.5, 1.0) == 0.0  # held
        assert filt.filter(-0.5, 1.3) == 0.0  # still held (0.3s < 0.8s)

    def test_direction_change_allowed_after_hold(self):
        from drone_follow.follow_api.controller import _DirectionHoldFilter
        filt = _DirectionHoldFilter()
        filt._last_sign = 1  # was going forward
        filt.filter(-0.5, 1.0)   # start hold
        filt.filter(-0.5, 1.3)   # still holding
        result = filt.filter(-0.5, 1.9)  # 0.9s > 0.8s hold time
        assert result == -0.5  # now allowed

    def test_interrupted_direction_change_resets(self):
        from drone_follow.follow_api.controller import _DirectionHoldFilter
        filt = _DirectionHoldFilter()
        filt._last_sign = 1
        filt.filter(-0.5, 1.0)   # start backward hold
        filt.filter(-0.5, 1.3)   # 0.3s into hold
        filt.filter(0.5, 1.5)    # back to forward — interrupts
        # Forward should pass through (same as last_sign)
        assert filt.filter(0.5, 1.6) == 0.5

    def test_zero_passes_through(self):
        from drone_follow.follow_api.controller import _DirectionHoldFilter
        filt = _DirectionHoldFilter()
        filt._last_sign = 1
        assert filt.filter(0.0, 1.0) == 0.0
