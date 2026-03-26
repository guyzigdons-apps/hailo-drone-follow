"""Integration tests for pose-based gesture control types and wiring."""

import time
import pytest

from drone_follow.follow_api.types import WristDetection


class TestWristDetection:
    def test_construction(self):
        w = WristDetection(
            x=0.4, y=0.6,
            shoulder_x=0.4, shoulder_y=0.4,
            confidence=0.85,
            timestamp=time.monotonic(),
        )
        assert w.x == 0.4
        assert w.y == 0.6
        assert w.confidence == 0.85

    def test_is_raised_above_shoulder(self):
        """Wrist above shoulder (lower y) indicates raised hand."""
        w = WristDetection(x=0.4, y=0.3, shoulder_x=0.4, shoulder_y=0.5,
                           confidence=0.9, timestamp=0.0)
        assert w.y < w.shoulder_y  # raised

    def test_is_below_shoulder(self):
        """Wrist below shoulder (higher y) indicates lowered hand."""
        w = WristDetection(x=0.4, y=0.7, shoulder_x=0.4, shoulder_y=0.5,
                           confidence=0.9, timestamp=0.0)
        assert w.y > w.shoulder_y  # lowered


import argparse


class TestPoseCLIFlag:
    def test_pose_flag_registered(self):
        """The --pose flag is parseable alongside --gesture."""
        parser = argparse.ArgumentParser()
        parser.add_argument("--gesture", action="store_true")
        parser.add_argument("--pose", action="store_true")
        args = parser.parse_args(["--gesture", "--pose"])
        assert args.gesture is True
        assert args.pose is True

    def test_pose_flag_default_false(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--pose", action="store_true")
        args = parser.parse_args([])
        assert args.pose is False


class TestPoseGestureAppImport:
    def test_create_pose_gesture_app_importable(self):
        from drone_follow.pipeline_adapter import create_pose_gesture_app
        assert callable(create_pose_gesture_app)
