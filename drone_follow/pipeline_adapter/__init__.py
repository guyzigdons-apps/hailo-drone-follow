"""pipeline_adapter — Hailo/GStreamer pipeline adapters.

All Hailo and GStreamer imports are confined to this package.
Other modules receive detections as pure Detection objects via callbacks.
"""

from .hailo_drone_detection_manager import create_app


def create_gesture_app(*args, **kwargs):
    """Lazy import to avoid loading gesture pipeline when not needed."""
    from .gesture_detection_manager import create_gesture_app as _create
    return _create(*args, **kwargs)


__all__ = [
    "create_app",
    "create_gesture_app",
]
