"""Unit tests for pose gesture callback helpers.

Tests the pure logic that maps pose keypoints → WristDetection → GestureDetection.
No Hailo/GStreamer dependencies — uses plain data.
"""

import math
import time

import pytest

from drone_follow.follow_api.types import (
    FaceDetection, HandDetection, GestureDetection, WristDetection,
)
from drone_follow.follow_api.gesture_controller import WaveDetector


# --- Helper to simulate pose keypoints as a dict ---

def _make_keypoints(
    nose=(0.5, 0.3),
    left_eye=(0.48, 0.28), right_eye=(0.52, 0.28),
    left_shoulder=(0.4, 0.5), right_shoulder=(0.6, 0.5),
    left_wrist=(0.35, 0.7), right_wrist=(0.65, 0.7),
    left_elbow=(0.38, 0.6), right_elbow=(0.62, 0.6),
):
    """Build a dict of COCO keypoints as (x, y) tuples in frame-normalized coords."""
    return {
        0: nose, 1: left_eye, 2: right_eye,
        3: (0.45, 0.28), 4: (0.55, 0.28),  # ears
        5: left_shoulder, 6: right_shoulder,
        7: left_elbow, 8: right_elbow,
        9: left_wrist, 10: right_wrist,
        11: (0.45, 0.7), 12: (0.55, 0.7),  # hips
        13: (0.45, 0.9), 14: (0.55, 0.9),  # knees
        15: (0.45, 1.0), 16: (0.55, 1.0),  # ankles
    }


class TestFaceFromPose:
    """Test synthesizing FaceDetection from pose keypoints."""

    def test_face_from_nose_and_eyes(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import face_from_pose_keypoints
        kps = _make_keypoints(nose=(0.5, 0.3), left_eye=(0.48, 0.28), right_eye=(0.52, 0.28))
        face = face_from_pose_keypoints(kps, confidence=0.9, timestamp=1.0)
        assert face is not None
        assert face.center_x == pytest.approx(0.5, abs=0.01)
        assert face.center_y == pytest.approx(0.29, abs=0.02)  # avg of nose + eyes

    def test_face_none_when_no_nose(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import face_from_pose_keypoints
        kps = _make_keypoints()
        kps[0] = None  # no nose
        face = face_from_pose_keypoints(kps, confidence=0.9, timestamp=1.0)
        assert face is None


class TestWristFromPose:
    """Test extracting WristDetection from pose keypoints."""

    def test_left_wrist_extraction(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import wrist_from_pose_keypoints
        kps = _make_keypoints(left_wrist=(0.35, 0.7), left_shoulder=(0.4, 0.5))
        wrist = wrist_from_pose_keypoints(kps, side="left", timestamp=1.0)
        assert wrist is not None
        assert wrist.x == pytest.approx(0.35)
        assert wrist.y == pytest.approx(0.7)
        assert wrist.shoulder_x == pytest.approx(0.4)
        assert wrist.shoulder_y == pytest.approx(0.5)

    def test_right_wrist_extraction(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import wrist_from_pose_keypoints
        kps = _make_keypoints(right_wrist=(0.65, 0.4), right_shoulder=(0.6, 0.5))
        wrist = wrist_from_pose_keypoints(kps, side="right", timestamp=1.0)
        assert wrist is not None
        assert wrist.x == pytest.approx(0.65)
        assert wrist.y == pytest.approx(0.4)

    def test_wrist_none_when_missing(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import wrist_from_pose_keypoints
        kps = _make_keypoints()
        kps[9] = None  # no left wrist
        wrist = wrist_from_pose_keypoints(kps, side="left", timestamp=1.0)
        assert wrist is None


class TestWristToHandDetection:
    """Test mapping WristDetection → HandDetection for gesture controller compatibility."""

    def test_raised_wrist_maps_to_open_hand(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import wrist_to_hand_detection
        wrist = WristDetection(x=0.4, y=0.3, shoulder_x=0.4, shoulder_y=0.5,
                                confidence=0.9, timestamp=1.0)
        hand = wrist_to_hand_detection(wrist)
        assert hand is not None
        assert hand.is_open is True  # wrist above shoulder
        assert hand.center_x == pytest.approx(0.4)
        assert hand.wrist_x == pytest.approx(0.4)

    def test_lowered_wrist_maps_to_fist(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import wrist_to_hand_detection
        wrist = WristDetection(x=0.4, y=0.7, shoulder_x=0.4, shoulder_y=0.5,
                                confidence=0.9, timestamp=1.0)
        hand = wrist_to_hand_detection(wrist)
        assert hand is not None
        assert hand.is_open is False  # wrist below shoulder


class TestSelectActiveWrist:
    """Test selecting which wrist to use for gesture control."""

    def test_higher_wrist_selected(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import select_active_wrist
        left = WristDetection(x=0.35, y=0.3, shoulder_x=0.4, shoulder_y=0.5,
                               confidence=0.9, timestamp=1.0)
        right = WristDetection(x=0.65, y=0.7, shoulder_x=0.6, shoulder_y=0.5,
                                confidence=0.9, timestamp=1.0)
        active = select_active_wrist(left, right)
        assert active is left  # left is higher (lower y)

    def test_single_wrist_returned(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import select_active_wrist
        right = WristDetection(x=0.65, y=0.4, shoulder_x=0.6, shoulder_y=0.5,
                                confidence=0.9, timestamp=1.0)
        active = select_active_wrist(None, right)
        assert active is right

    def test_no_wrists_returns_none(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import select_active_wrist
        assert select_active_wrist(None, None) is None


class TestWaveDetectionWithWrist:
    """Verify WaveDetector works with wrist X coordinates (same as palm X)."""

    def test_wrist_oscillation_detects_wave(self):
        wd = WaveDetector(reversals_needed=3, window_s=2.0)
        t = 0.0
        # Simulate wrist oscillating left-right
        positions = [0.3, 0.5, 0.3, 0.5, 0.3]
        detected = False
        for x in positions:
            t += 0.2
            if wd.update(x, t):
                detected = True
                break
        assert detected
