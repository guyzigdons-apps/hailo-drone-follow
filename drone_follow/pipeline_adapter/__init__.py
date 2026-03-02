"""pipeline_adapter — Hailo/GStreamer pipeline adapters.

All Hailo and GStreamer imports are confined to this package.
Other modules receive detections as pure Detection objects via callbacks.
ByteTracker (multi-object tracker) also lives here.
"""

from .byte_tracker import ByteTracker
from .hailo_drone_detection_manager import app_callback, create_app

__all__ = [
    "ByteTracker",
    "app_callback",
    "create_app",
]
