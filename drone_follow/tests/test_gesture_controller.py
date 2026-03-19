"""Tests for gesture controller — compute_gesture_velocity_command, WaveDetector, classify_hand_open."""

import math
import time

import pytest

from drone_follow.follow_api.types import (
    FaceDetection, HandDetection, GestureDetection, VelocityCommand,
)
from drone_follow.follow_api.config import ControllerConfig
from drone_follow.follow_api.gesture_controller import (
    compute_gesture_velocity_command,
    compute_gesture_lateral,
    classify_hand_open,
    WaveDetector,
)


def _face(cx=0.5, cy=0.4):
    return FaceDetection(center_x=cx, center_y=cy, bbox_width=0.1, bbox_height=0.12,
                         confidence=0.9, timestamp=time.monotonic())


def _hand(cx=0.5, cy=0.5, wrist_x=0.5, wrist_y=0.6, is_open=True):
    return HandDetection(center_x=cx, center_y=cy, wrist_x=wrist_x, wrist_y=wrist_y,
                         is_open=is_open, confidence=0.8, timestamp=time.monotonic())


def _gesture(face=None, hand=None, person_bbox_height=0.3):
    f = face or _face()
    return GestureDetection(face=f, hand=hand, person_bbox_height=person_bbox_height,
                            timestamp=time.monotonic())


def _config(**kwargs):
    return ControllerConfig(**kwargs)


# --- compute_gesture_velocity_command ---

class TestGestureVelocityCommand:
    def test_no_detection_no_search(self):
        cmd = compute_gesture_velocity_command(None, _config(), search_active=False)
        assert cmd.forward_m_s == 0.0
        assert cmd.yawspeed_deg_s == 0.0

    def test_no_detection_search(self):
        cmd = compute_gesture_velocity_command(None, _config(), search_active=True)
        assert cmd.forward_m_s == 0.0
        assert cmd.yawspeed_deg_s != 0.0  # should be spinning

    def test_face_centered_no_hand(self):
        """Face centered, no hand -> near-zero yaw."""
        g = _gesture(face=_face(cx=0.5))
        cmd = compute_gesture_velocity_command(g, _config())
        assert cmd.forward_m_s == 0.0
        assert abs(cmd.yawspeed_deg_s) < 1.0  # within dead zone

    def test_face_off_center_no_hand(self):
        """Face off-center, no hand -> yaw to center face."""
        g = _gesture(face=_face(cx=0.7))
        cmd = compute_gesture_velocity_command(g, _config())
        assert cmd.forward_m_s == 0.0
        assert cmd.yawspeed_deg_s < 0  # person right of center -> yaw left (negative)

    def test_open_palm_right_of_face(self):
        """Open palm right of face -> negative yaw (yaw right in PX4 convention)."""
        g = _gesture(
            face=_face(cx=0.5),
            hand=_hand(cx=0.7, is_open=True),
        )
        cmd = compute_gesture_velocity_command(g, _config())
        assert cmd.yawspeed_deg_s < 0  # hand right -> yaw right (negative)

    def test_open_palm_left_of_face(self):
        """Open palm left of face -> positive yaw (yaw left)."""
        g = _gesture(
            face=_face(cx=0.5),
            hand=_hand(cx=0.3, is_open=True),
        )
        cmd = compute_gesture_velocity_command(g, _config())
        assert cmd.yawspeed_deg_s > 0  # hand left -> yaw left (positive)

    def test_fist_stops(self):
        """Fist detected -> zero forward, face-tracking yaw only."""
        g = _gesture(
            face=_face(cx=0.5),
            hand=_hand(cx=0.7, is_open=False),
        )
        cmd = compute_gesture_velocity_command(g, _config())
        assert cmd.forward_m_s == 0.0

    def test_arm_extended_forward(self):
        """Arm extended (wrist far from face) -> positive forward speed."""
        g = _gesture(
            face=_face(cx=0.5, cy=0.4),
            hand=_hand(cx=0.5, cy=0.7, wrist_x=0.5, wrist_y=0.8, is_open=True),
        )
        cmd = compute_gesture_velocity_command(g, _config())
        assert cmd.forward_m_s > 0.0

    def test_arm_close_no_forward(self):
        """Arm close to face (within dead zone) -> no forward."""
        g = _gesture(
            face=_face(cx=0.5, cy=0.4),
            hand=_hand(cx=0.5, cy=0.42, wrist_x=0.5, wrist_y=0.44, is_open=True),
        )
        cfg = _config(gesture_arm_dead_zone=0.1)
        cmd = compute_gesture_velocity_command(g, cfg)
        assert cmd.forward_m_s == 0.0

    def test_person_too_close_emergency_backward(self):
        """Person bbox > safety limit -> emergency backward."""
        g = _gesture(person_bbox_height=0.9)
        cmd = compute_gesture_velocity_command(g, _config())
        assert cmd.forward_m_s < 0  # backward

    def test_max_forward_clamped(self):
        """Forward speed is clamped to gesture_max_forward."""
        g = _gesture(
            face=_face(cx=0.5, cy=0.3),
            hand=_hand(cx=0.5, cy=0.9, wrist_x=0.5, wrist_y=1.0, is_open=True),
        )
        cfg = _config(gesture_max_forward=1.5)
        cmd = compute_gesture_velocity_command(g, cfg)
        assert cmd.forward_m_s <= cfg.gesture_max_forward

    def test_max_yawspeed_clamped(self):
        """Yaw speed is clamped to gesture_max_yawspeed."""
        g = _gesture(
            face=_face(cx=0.5),
            hand=_hand(cx=0.99, is_open=True),
        )
        cfg = _config(gesture_max_yawspeed=60.0)
        cmd = compute_gesture_velocity_command(g, cfg)
        assert abs(cmd.yawspeed_deg_s) <= cfg.gesture_max_yawspeed + 0.01

    def test_search_uses_last_gesture_direction(self):
        """Search mode uses last gesture face position for spin direction."""
        last = _gesture(face=_face(cx=0.8))
        cmd = compute_gesture_velocity_command(None, _config(), last_gesture=last, search_active=True)
        assert cmd.yawspeed_deg_s < 0  # last seen right -> spin right (negative)


# --- WaveDetector ---

class TestWaveDetector:
    def test_oscillation_detects_wave(self):
        """Oscillating hand_x detects wave after N reversals."""
        wd = WaveDetector(reversals_needed=3, window_s=2.0)
        t = 0.0
        positions = [0.3, 0.5, 0.3, 0.5, 0.3]
        detected = False
        for x in positions:
            t += 0.2
            if wd.update(x, t):
                detected = True
                break
        assert detected

    def test_steady_hand_no_wave(self):
        """Steady hand position -> no wave detected."""
        wd = WaveDetector(reversals_needed=3, window_s=2.0)
        t = 0.0
        for _ in range(20):
            t += 0.1
            assert not wd.update(0.5, t)

    def test_slow_oscillation_no_wave(self):
        """Oscillation too slow (outside window) -> no wave."""
        wd = WaveDetector(reversals_needed=3, window_s=0.5)
        t = 0.0
        # Oscillate but with 1s gaps between reversals
        positions = [0.3, 0.5, 0.3, 0.5]
        detected = False
        for x in positions:
            t += 1.0  # too slow
            if wd.update(x, t):
                detected = True
        assert not detected

    def test_reset(self):
        """After reset, detection state is cleared."""
        wd = WaveDetector(reversals_needed=3, window_s=2.0)
        t = 0.0
        for x in [0.3, 0.5, 0.3, 0.5, 0.3]:
            t += 0.2
            wd.update(x, t)
        assert wd.is_detected
        wd.reset()
        assert not wd.is_detected

    def test_no_double_detect(self):
        """After detection, further updates return False."""
        wd = WaveDetector(reversals_needed=3, window_s=2.0)
        t = 0.0
        for x in [0.3, 0.5, 0.3, 0.5, 0.3]:
            t += 0.2
            wd.update(x, t)
        assert wd.is_detected
        assert not wd.update(0.5, t + 0.1)


# --- compute_gesture_lateral ---

class TestGestureLateral:
    def test_no_gesture_returns_zero(self):
        assert compute_gesture_lateral(None, _config()) == 0.0

    def test_no_hand_returns_zero(self):
        g = _gesture(hand=None)
        assert compute_gesture_lateral(g, _config()) == 0.0

    def test_fist_returns_zero(self):
        g = _gesture(hand=_hand(cx=0.7, is_open=False))
        assert compute_gesture_lateral(g, _config()) == 0.0

    def test_hand_right_of_face_positive_lateral(self):
        """Hand right of face -> positive lateral (strafe right)."""
        g = _gesture(face=_face(cx=0.5), hand=_hand(cx=0.7, is_open=True))
        lateral = compute_gesture_lateral(g, _config())
        assert lateral > 0.0

    def test_hand_left_of_face_negative_lateral(self):
        """Hand left of face -> negative lateral (strafe left)."""
        g = _gesture(face=_face(cx=0.5), hand=_hand(cx=0.3, is_open=True))
        lateral = compute_gesture_lateral(g, _config())
        assert lateral < 0.0

    def test_hand_centered_dead_zone(self):
        """Hand near face center (within dead zone) -> zero lateral."""
        g = _gesture(face=_face(cx=0.5), hand=_hand(cx=0.51, is_open=True))
        cfg = _config(gesture_hand_dead_zone_deg=5.0)
        lateral = compute_gesture_lateral(g, cfg)
        assert lateral == 0.0

    def test_clamped_to_max(self):
        """Lateral is clamped to gesture_max_lateral."""
        g = _gesture(face=_face(cx=0.1), hand=_hand(cx=0.9, is_open=True))
        cfg = _config(gesture_max_lateral=1.5)
        lateral = compute_gesture_lateral(g, cfg)
        assert abs(lateral) <= cfg.gesture_max_lateral + 0.01

    def test_proportional_to_offset(self):
        """Larger offset -> larger lateral magnitude."""
        g_small = _gesture(face=_face(cx=0.5), hand=_hand(cx=0.55, is_open=True))
        g_large = _gesture(face=_face(cx=0.5), hand=_hand(cx=0.65, is_open=True))
        cfg = _config(gesture_max_lateral=10.0)  # high clamp so we see proportionality
        lat_small = compute_gesture_lateral(g_small, cfg)
        lat_large = compute_gesture_lateral(g_large, cfg)
        assert abs(lat_large) > abs(lat_small)


# --- classify_hand_open ---

class TestClassifyHandOpen:
    def test_spread_fingers_open(self):
        """Spread fingertips far from wrist -> open palm."""
        wrist = (0.5, 0.8)
        fingertips = [(0.4, 0.4), (0.45, 0.35), (0.5, 0.3), (0.55, 0.35), (0.6, 0.4)]
        assert classify_hand_open(fingertips, wrist) is True

    def test_closed_fist(self):
        """Fingertips close to wrist -> fist."""
        wrist = (0.5, 0.5)
        fingertips = [(0.5, 0.48), (0.51, 0.48), (0.5, 0.47), (0.49, 0.48), (0.5, 0.49)]
        assert classify_hand_open(fingertips, wrist) is False

    def test_empty_fingertips(self):
        """No fingertips -> fist."""
        assert classify_hand_open([], (0.5, 0.5)) is False
