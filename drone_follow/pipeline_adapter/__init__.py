"""pipeline_adapter — Hailo/GStreamer pipeline adapters.

All Hailo and GStreamer imports are confined to this package.
Other modules receive detections as pure Detection objects via callbacks.

On RPi/x86: uses hailo_apps tiling pipeline (create_app)
On Hailo15:  uses native hailofrontendbinsrc + pyhailort (create_h15_app)
"""

try:
    from .hailo_drone_detection_manager import create_app
except ImportError:
    create_app = None

try:
    from .hailo15_pipeline import create_h15_app
except ImportError:
    create_h15_app = None

__all__ = [
    "create_app",
    "create_h15_app",
]
