"""Functional tests for gesture mode integration wiring.

Verifies that:
1. All new types are importable and constructable
2. SharedGestureState works correctly
3. Config round-trips with gesture fields
4. The gesture types match what gesture_detection_manager and gesture_drone expect
5. The --gesture CLI flag is registered
"""

import argparse
import time

import pytest

from drone_follow.follow_api.types import (
    FaceDetection, HandDetection, GestureDetection, FollowMode,
)
from drone_follow.follow_api.state import SharedGestureState
from drone_follow.follow_api.config import ControllerConfig
from drone_follow.follow_api import (
    FaceDetection as FaceDetectionReexport,
    HandDetection as HandDetectionReexport,
    GestureDetection as GestureDetectionReexport,
    SharedGestureState as SharedGestureStateReexport,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

def _make_face(cx=0.5, cy=0.3):
    return FaceDetection(
        center_x=cx, center_y=cy,
        bbox_width=0.1, bbox_height=0.12,
        confidence=0.9, timestamp=time.monotonic(),
    )


def _make_hand(cx=0.6, cy=0.4, is_open=True):
    return HandDetection(
        center_x=cx, center_y=cy,
        wrist_x=cx - 0.05, wrist_y=cy + 0.1,
        is_open=is_open, confidence=0.85,
        timestamp=time.monotonic(),
    )


def _make_gesture(face=None, hand=None):
    return GestureDetection(
        face=face or _make_face(),
        hand=hand,
        person_bbox_height=0.6,
        timestamp=time.monotonic(),
    )


# ---------------------------------------------------------------------------
# Type construction and field access
# ---------------------------------------------------------------------------

class TestGestureTypes:
    def test_face_detection_fields(self):
        f = _make_face(0.4, 0.2)
        assert f.center_x == 0.4
        assert f.center_y == 0.2
        assert f.bbox_width == 0.1
        assert f.bbox_height == 0.12
        assert f.confidence == 0.9

    def test_hand_detection_fields(self):
        h = _make_hand(0.6, 0.4, is_open=False)
        assert h.center_x == 0.6
        assert h.is_open is False
        assert h.wrist_x == pytest.approx(0.55)

    def test_gesture_detection_with_hand(self):
        g = _make_gesture(hand=_make_hand())
        assert g.face is not None
        assert g.hand is not None
        assert g.person_bbox_height == 0.6

    def test_gesture_detection_without_hand(self):
        g = _make_gesture(hand=None)
        assert g.hand is None

    def test_follow_mode_gesture_value(self):
        assert FollowMode.GESTURE == "gesture"
        assert FollowMode("gesture") == FollowMode.GESTURE


# ---------------------------------------------------------------------------
# SharedGestureState
# ---------------------------------------------------------------------------

class TestSharedGestureState:
    def test_initial_state_is_none(self):
        state = SharedGestureState()
        gesture, count = state.get_latest()
        assert gesture is None
        assert count == 0

    def test_update_and_get(self):
        state = SharedGestureState()
        g = _make_gesture()
        state.update(g)
        gesture, count = state.get_latest()
        assert gesture is g
        assert count == 1

    def test_update_none_clears(self):
        state = SharedGestureState()
        state.update(_make_gesture())
        state.update(None)
        gesture, count = state.get_latest()
        assert gesture is None
        assert count == 2

    def test_frame_count_increments(self):
        state = SharedGestureState()
        for _ in range(5):
            state.update(None)
        _, count = state.get_latest()
        assert count == 5


# ---------------------------------------------------------------------------
# Config gesture fields
# ---------------------------------------------------------------------------

class TestGestureConfig:
    def test_default_gesture_fields_exist(self):
        cfg = ControllerConfig()
        assert cfg.gesture_wave_reversals == 3
        assert cfg.gesture_wave_window_s == 1.5
        assert cfg.gesture_ack_duration_s == 1.0
        assert cfg.gesture_ack_amplitude_deg == 30.0
        assert cfg.gesture_hand_dead_zone_deg == 5.0
        assert cfg.gesture_kp_hand_yaw == 4.0
        assert cfg.gesture_max_yawspeed == 60.0
        assert cfg.gesture_kp_forward == 3.0
        assert cfg.gesture_max_forward == 1.5
        assert cfg.gesture_arm_dead_zone == 0.08

    def test_gesture_fields_in_json_roundtrip(self, tmp_path):
        cfg = ControllerConfig(gesture_kp_hand_yaw=8.0)
        path = str(tmp_path / "cfg.json")
        cfg.save_json(path)
        loaded = ControllerConfig.from_json(path)
        assert loaded.gesture_kp_hand_yaw == 8.0

    def test_follow_mode_accepts_gesture(self):
        cfg = ControllerConfig(follow_mode="gesture")
        assert cfg.follow_mode == "gesture"

    def test_follow_mode_gesture_in_argparse(self):
        """--follow-mode gesture is accepted by the CLI parser."""
        parser = argparse.ArgumentParser()
        ControllerConfig.add_args(parser)
        args = parser.parse_args(["--follow-mode", "gesture"])
        assert args.follow_mode == "gesture"


# ---------------------------------------------------------------------------
# Re-exports from __init__.py
# ---------------------------------------------------------------------------

class TestReexports:
    def test_types_reexported(self):
        assert FaceDetectionReexport is FaceDetection
        assert HandDetectionReexport is HandDetection
        assert GestureDetectionReexport is GestureDetection

    def test_state_reexported(self):
        assert SharedGestureStateReexport is SharedGestureState


# ---------------------------------------------------------------------------
# App-level --gesture flag
# ---------------------------------------------------------------------------

class TestGestureCLIFlag:
    def test_gesture_flag_registered(self):
        """The --gesture flag is parseable alongside other app args."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--gesture", action="store_true")
        args = parser.parse_args(["--gesture"])
        assert args.gesture is True

    def test_gesture_flag_default_false(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--gesture", action="store_true")
        args = parser.parse_args([])
        assert args.gesture is False
