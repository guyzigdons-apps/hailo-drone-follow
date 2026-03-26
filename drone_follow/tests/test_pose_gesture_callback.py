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
from drone_follow.pipeline_adapter.pose_gesture_manager import LEFT_WRIST, RIGHT_WRIST


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

    def test_wrist_near_shoulder_maps_to_open(self):
        """Wrist slightly below shoulder (within 10% margin) still counts as open."""
        from drone_follow.pipeline_adapter.pose_gesture_manager import wrist_to_hand_detection
        wrist = WristDetection(x=0.4, y=0.55, shoulder_x=0.4, shoulder_y=0.5,
                                confidence=0.9, timestamp=1.0)
        hand = wrist_to_hand_detection(wrist)
        assert hand is not None
        assert hand.is_open is True  # wrist 0.05 below shoulder, within 0.10 margin

    def test_lowered_wrist_maps_to_fist(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import wrist_to_hand_detection
        wrist = WristDetection(x=0.4, y=0.7, shoulder_x=0.4, shoulder_y=0.5,
                                confidence=0.9, timestamp=1.0)
        hand = wrist_to_hand_detection(wrist)
        assert hand is not None
        assert hand.is_open is False  # wrist 0.20 below shoulder, beyond 0.10 margin


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


class TestDetectTpose:
    """Test T-pose detection from keypoints.

    T-pose requires: wrists + elbows at shoulder height, elbows between
    shoulder and wrist, arms spread wide.
    """

    def test_tpose_detected(self):
        """Arms fully extended horizontal — classic T-pose."""
        from drone_follow.pipeline_adapter.pose_gesture_manager import detect_tpose
        kps = _make_keypoints(
            left_shoulder=(0.4, 0.5), right_shoulder=(0.6, 0.5),
            left_elbow=(0.25, 0.5), right_elbow=(0.75, 0.5),
            left_wrist=(0.1, 0.5), right_wrist=(0.9, 0.5),
        )
        assert detect_tpose(kps) is True

    def test_tpose_with_slight_y_offset(self):
        """Arms near shoulder height within tolerance."""
        from drone_follow.pipeline_adapter.pose_gesture_manager import detect_tpose
        kps = _make_keypoints(
            left_shoulder=(0.4, 0.5), right_shoulder=(0.6, 0.5),
            left_elbow=(0.25, 0.53), right_elbow=(0.75, 0.47),
            left_wrist=(0.1, 0.55), right_wrist=(0.9, 0.45),
        )
        assert detect_tpose(kps) is True

    def test_arms_down_not_tpose(self):
        """Arms at sides = not T-pose."""
        from drone_follow.pipeline_adapter.pose_gesture_manager import detect_tpose
        kps = _make_keypoints(
            left_shoulder=(0.4, 0.5), right_shoulder=(0.6, 0.5),
            left_elbow=(0.38, 0.65), right_elbow=(0.62, 0.65),
            left_wrist=(0.35, 0.8), right_wrist=(0.65, 0.8),
        )
        assert detect_tpose(kps) is False

    def test_arms_not_spread_not_tpose(self):
        """Arms at shoulder height but not spread wide."""
        from drone_follow.pipeline_adapter.pose_gesture_manager import detect_tpose
        kps = _make_keypoints(
            left_shoulder=(0.4, 0.5), right_shoulder=(0.6, 0.5),
            left_elbow=(0.41, 0.5), right_elbow=(0.59, 0.5),
            left_wrist=(0.42, 0.5), right_wrist=(0.58, 0.5),
        )
        assert detect_tpose(kps) is False

    def test_bent_arms_raised_not_tpose(self):
        """Hands raised to shoulder height but elbows bent (not extended).
        Elbows are below shoulder height — should NOT trigger T-pose."""
        from drone_follow.pipeline_adapter.pose_gesture_manager import detect_tpose
        kps = _make_keypoints(
            left_shoulder=(0.4, 0.5), right_shoulder=(0.6, 0.5),
            left_elbow=(0.38, 0.7), right_elbow=(0.62, 0.7),  # elbows low
            left_wrist=(0.3, 0.5), right_wrist=(0.7, 0.5),    # wrists at shoulder
        )
        assert detect_tpose(kps) is False

    def test_elbows_not_between_shoulder_and_wrist(self):
        """Elbows outside the shoulder-to-wrist span (e.g., behind body)."""
        from drone_follow.pipeline_adapter.pose_gesture_manager import detect_tpose
        kps = _make_keypoints(
            left_shoulder=(0.4, 0.5), right_shoulder=(0.6, 0.5),
            left_elbow=(0.45, 0.5), right_elbow=(0.55, 0.5),  # elbows INSIDE shoulders
            left_wrist=(0.1, 0.5), right_wrist=(0.9, 0.5),
        )
        # Elbows should be between shoulder and wrist X, not inside shoulder span
        # Left: shoulder=0.4, wrist=0.1, elbow=0.45 → 0.45 > 0.4 → not between
        assert detect_tpose(kps) is False

    def test_missing_elbow_not_tpose(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import detect_tpose
        kps = _make_keypoints(
            left_shoulder=(0.4, 0.5), right_shoulder=(0.6, 0.5),
            left_elbow=(0.25, 0.5), right_elbow=(0.75, 0.5),
            left_wrist=(0.1, 0.5), right_wrist=(0.9, 0.5),
        )
        kps[7] = None  # LEFT_ELBOW
        assert detect_tpose(kps) is False

    def test_missing_wrist_not_tpose(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import detect_tpose
        kps = _make_keypoints()
        kps[LEFT_WRIST] = None
        assert detect_tpose(kps) is False


class TestDetectXpose:
    """Test X-pose (arms crossed) detection."""

    def test_xpose_detected(self):
        """Arms crossed over chest — left wrist right of right wrist."""
        from drone_follow.pipeline_adapter.pose_gesture_manager import detect_xpose
        kps = _make_keypoints(
            left_shoulder=(0.4, 0.5), right_shoulder=(0.6, 0.5),
            left_wrist=(0.55, 0.55), right_wrist=(0.45, 0.55),  # crossed
        )
        assert detect_xpose(kps) is True

    def test_uncrossed_arms_not_xpose(self):
        """Normal arm position — not crossed."""
        from drone_follow.pipeline_adapter.pose_gesture_manager import detect_xpose
        kps = _make_keypoints(
            left_shoulder=(0.4, 0.5), right_shoulder=(0.6, 0.5),
            left_wrist=(0.35, 0.7), right_wrist=(0.65, 0.7),  # normal sides
        )
        assert detect_xpose(kps) is False

    def test_wrists_crossed_but_too_far_apart(self):
        """Crossed but wrists far apart — not a tight X."""
        from drone_follow.pipeline_adapter.pose_gesture_manager import detect_xpose
        kps = _make_keypoints(
            left_shoulder=(0.4, 0.5), right_shoulder=(0.6, 0.5),
            left_wrist=(0.8, 0.55), right_wrist=(0.2, 0.55),  # crossed but wide
        )
        assert detect_xpose(kps) is False

    def test_missing_keypoints_not_xpose(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import detect_xpose
        kps = _make_keypoints()
        kps[RIGHT_WRIST] = None
        assert detect_xpose(kps) is False


class TestPoseHoldTimer:
    """Test hold timer for sustained pose detection."""

    def test_triggers_after_hold_duration(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import PoseHoldTimer
        timer = PoseHoldTimer(hold_duration_s=2.0)
        t = 0.0
        # Hold for 2.5 seconds
        for _ in range(25):
            t += 0.1
            result = timer.update(True, t)
            if result:
                break
        assert result is True
        assert t >= 2.0

    def test_resets_on_interruption(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import PoseHoldTimer
        timer = PoseHoldTimer(hold_duration_s=2.0)
        # Hold for 1 second then break
        for i in range(10):
            timer.update(True, i * 0.1)
        timer.update(False, 1.1)  # break
        # Hold again for 1.5 seconds — should NOT trigger (restarted)
        triggered = False
        for i in range(15):
            if timer.update(True, 1.2 + i * 0.1):
                triggered = True
                break
        assert not triggered  # only 1.5s of the new hold, needs 2.0

    def test_no_double_trigger(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import PoseHoldTimer
        timer = PoseHoldTimer(hold_duration_s=1.0)
        # Hold for 2 seconds
        triggers = 0
        for i in range(20):
            if timer.update(True, i * 0.1):
                triggers += 1
        assert triggers == 1  # only fires once

    def test_reset_allows_retrigger(self):
        from drone_follow.pipeline_adapter.pose_gesture_manager import PoseHoldTimer
        timer = PoseHoldTimer(hold_duration_s=1.0)
        # First trigger
        for i in range(15):
            timer.update(True, i * 0.1)
        assert timer._triggered
        timer.reset()
        # Second trigger
        triggered = False
        for i in range(15):
            if timer.update(True, 2.0 + i * 0.1):
                triggered = True
        assert triggered
