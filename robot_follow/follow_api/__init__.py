"""follow_api — pure domain logic for drone follow.

No MAVSDK, Hailo, or GStreamer dependencies. Can be tested with
only standard library + numpy/scipy.
"""

from .types import (
    Axis,
    Capabilities,
    Detection,
    RobotCommand,
    SafetyContext,
)
from .config import ControllerConfig
from .state import SharedDetectionState, FollowTargetState
from .controller import compute

__all__ = [
    "Axis",
    "Capabilities",
    "Detection",
    "RobotCommand",
    "SafetyContext",
    "ControllerConfig",
    "SharedDetectionState",
    "FollowTargetState",
    "compute",
]
