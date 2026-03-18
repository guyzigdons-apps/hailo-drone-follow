"""Pure domain types for drone follow — no third-party dependencies."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class FollowMode(str, Enum):
    FOLLOW = "follow"
    ORBIT = "orbit"
    GESTURE = "gesture"


@dataclass
class VelocityCommand:
    """Velocity command in the drone body frame.

    This is a pure domain type that replaces direct use of
    mavsdk.offboard.VelocityBodyYawspeed in the follow logic, keeping
    the follow layer free of MAVSDK dependencies.
    """
    forward_m_s: float
    right_m_s: float
    down_m_s: float
    yawspeed_deg_s: float


@dataclass
class Detection:
    """A single person detection in normalized image coordinates."""
    label: str
    confidence: float
    center_x: float      # 0.0 to 1.0
    center_y: float      # 0.0 to 1.0
    bbox_height: float   # 0.0 to 1.0
    timestamp: float


@dataclass
class FaceDetection:
    center_x: float       # 0.0 to 1.0
    center_y: float
    bbox_width: float
    bbox_height: float
    confidence: float
    timestamp: float


@dataclass
class HandDetection:
    center_x: float       # palm center, 0.0 to 1.0
    center_y: float
    wrist_x: float
    wrist_y: float
    is_open: bool
    confidence: float
    timestamp: float


@dataclass
class PalmDetection:
    """A tracked palm detection (from palm_detection model, no landmarks needed)."""
    track_id: int         # stable ID from PalmTracker
    center_x: float       # palm bbox center, 0.0 to 1.0
    center_y: float
    bbox_width: float
    bbox_height: float
    confidence: float
    timestamp: float


@dataclass
class GestureDetection:
    face: FaceDetection
    hand: Optional[HandDetection]   # None when no hand visible
    person_bbox_height: float
    timestamp: float
