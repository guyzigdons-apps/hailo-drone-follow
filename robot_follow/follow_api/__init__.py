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
    VelocityCommand,
)
from .config import ControllerConfig
from .state import SharedDetectionState, FollowTargetState
from .controller import compute_velocity_command

__all__ = [
    "Axis",
    "Capabilities",
    "Detection",
    "RobotCommand",
    "SafetyContext",
    "VelocityCommand",
    "ControllerConfig",
    "SharedDetectionState",
    "FollowTargetState",
    "compute_velocity_command",
]
