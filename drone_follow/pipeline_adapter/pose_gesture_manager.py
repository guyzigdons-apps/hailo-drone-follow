"""Pose-based gesture pipeline adapter — YOLOv8-Pose wrist keypoints for wave detection and drone control.

Uses a single YOLOv8-Pose model instead of the cascaded palm_detection + hand_landmark pipeline.
Wrist keypoints (COCO indices 9/10) drive wave detection and gesture control.
Face is synthesized from nose + eye keypoints.

Populates SharedGestureState (for gesture control) and SharedDetectionState (for distance control).
"""

import logging
import math
import os
import subprocess
import threading
import time
from typing import Dict, Optional, Tuple

from drone_follow.follow_api.types import (
    Detection, FaceDetection, HandDetection, GestureDetection, WristDetection,
)
from drone_follow.follow_api.gesture_controller import WaveDetector

LOGGER = logging.getLogger("drone_follow.pose_gesture")

# COCO keypoint indices
NOSE = 0
LEFT_EYE = 1
RIGHT_EYE = 2
LEFT_SHOULDER = 5
RIGHT_SHOULDER = 6
LEFT_ELBOW = 7
RIGHT_ELBOW = 8
LEFT_WRIST = 9
RIGHT_WRIST = 10

# Pose model / postprocess constants (resolved at runtime via hailo-apps helpers)
POSE_ESTIMATION_PIPELINE = "pose_estimation"


def face_from_pose_keypoints(
    keypoints: Dict[int, Optional[Tuple[float, float]]],
    confidence: float,
    timestamp: float,
) -> Optional[FaceDetection]:
    """Synthesize a FaceDetection from nose and eye keypoints.

    Args:
        keypoints: Dict mapping COCO index -> (x, y) in frame-normalized coords,
                   or None if keypoint is missing/low-confidence.
        confidence: Overall confidence for the face detection.
        timestamp: Timestamp of the detection.

    Returns:
        FaceDetection or None if nose keypoint is missing.
    """
    nose = keypoints.get(NOSE)
    if nose is None:
        return None

    left_eye = keypoints.get(LEFT_EYE)
    right_eye = keypoints.get(RIGHT_EYE)

    # Center = average of available face keypoints
    pts = [nose]
    if left_eye is not None:
        pts.append(left_eye)
    if right_eye is not None:
        pts.append(right_eye)

    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)

    # Estimate face bbox from eye separation (or fixed fraction if no eyes)
    if left_eye is not None and right_eye is not None:
        eye_dist = math.sqrt((left_eye[0] - right_eye[0]) ** 2 +
                             (left_eye[1] - right_eye[1]) ** 2)
        bbox_w = eye_dist * 2.5
        bbox_h = eye_dist * 3.0
    else:
        bbox_w = 0.08
        bbox_h = 0.10

    return FaceDetection(
        center_x=cx, center_y=cy,
        bbox_width=bbox_w, bbox_height=bbox_h,
        confidence=confidence, timestamp=timestamp,
    )


def wrist_from_pose_keypoints(
    keypoints: Dict[int, Optional[Tuple[float, float]]],
    side: str,
    timestamp: float,
) -> Optional[WristDetection]:
    """Extract a WristDetection from pose keypoints.

    Args:
        keypoints: Dict mapping COCO index -> (x, y) in frame-normalized coords,
                   or None if keypoint is missing/low-confidence.
        side: "left" or "right"
        timestamp: Timestamp of the detection.

    Returns:
        WristDetection or None if wrist or shoulder keypoint is missing.
    """
    wrist_idx = LEFT_WRIST if side == "left" else RIGHT_WRIST
    shoulder_idx = LEFT_SHOULDER if side == "left" else RIGHT_SHOULDER

    wrist_pt = keypoints.get(wrist_idx)
    shoulder_pt = keypoints.get(shoulder_idx)

    if wrist_pt is None or shoulder_pt is None:
        return None

    return WristDetection(
        x=wrist_pt[0], y=wrist_pt[1],
        shoulder_x=shoulder_pt[0], shoulder_y=shoulder_pt[1],
        confidence=1.0,  # pose model doesn't give per-keypoint confidence in TAPPAS
        timestamp=timestamp,
    )


def wrist_to_hand_detection(wrist: WristDetection) -> HandDetection:
    """Map a WristDetection to a HandDetection for gesture controller compatibility.

    is_open = wrist above shoulder (raised hand).
    center_x/y and wrist_x/y are both set to the wrist position.

    Args:
        wrist: WristDetection from pose model.

    Returns:
        HandDetection compatible with gesture controller.
    """
    is_open = wrist.y < wrist.shoulder_y  # lower y = higher in frame
    return HandDetection(
        center_x=wrist.x,
        center_y=wrist.y,
        wrist_x=wrist.x,
        wrist_y=wrist.y,
        is_open=is_open,
        confidence=wrist.confidence,
        timestamp=wrist.timestamp,
    )


def select_active_wrist(
    left: Optional[WristDetection],
    right: Optional[WristDetection],
) -> Optional[WristDetection]:
    """Select the active (highest / most raised) wrist for gesture control.

    Returns the wrist with lower y value (higher in frame), or whichever is available.

    Args:
        left: Left wrist detection or None.
        right: Right wrist detection or None.

    Returns:
        The selected wrist, or None if both are None.
    """
    if left is None and right is None:
        return None
    if left is None:
        return right
    if right is None:
        return left
    return left if left.y < right.y else right
