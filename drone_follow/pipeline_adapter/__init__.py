"""pipeline_adapter — Hailo/GStreamer pipeline adapters.

All Hailo and GStreamer imports are confined to this package.
Other modules receive detections as pure Detection objects via callbacks.
ByteTracker (multi-object tracker) also lives here.
"""

from .hailo_drone_detection_manager import create_app

__all__ = [
    "create_app",
]
