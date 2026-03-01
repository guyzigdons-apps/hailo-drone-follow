"""follow_api — pure domain logic for drone follow.

No MAVSDK, Hailo, or GStreamer dependencies. Can be tested with
only standard library + numpy/scipy.
"""

from .types import Detection, VelocityCommand
from .config import ControllerConfig
from .state import SharedDetectionState, FollowTargetState
from .controller import (
    compute_velocity_command,
    ForwardSmoother,
    _distance_to_bbox_height,
    _effective_target_bbox_height,
    _calculate_forward_speed,
)

__all__ = [
    "Detection",
    "VelocityCommand",
    "ControllerConfig",
    "SharedDetectionState",
    "FollowTargetState",
    "compute_velocity_command",
    "ForwardSmoother",
    "_distance_to_bbox_height",
    "_effective_target_bbox_height",
    "_calculate_forward_speed",
]
