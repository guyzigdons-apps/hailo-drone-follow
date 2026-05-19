"""Robot Follow — visual-servoing pipeline app for Hailo AI processors.

Architecture:
    follow_api/         Pure domain logic (types, config, state, controller math)
    robot_api/          Robot protocol + adapters (MAVSDK drone)
    pipeline_adapter/   Hailo/GStreamer pipeline adapter + ByteTracker
    servers/            HTTP servers (follow target API, web UI)
    tools/              Standalone utilities (video bridge)
    robot_follow_app.py Composition root and CLI entrypoint
"""

from .follow_api import (
    Detection,
    RobotCommand,
    SharedDetectionState,
    ControllerConfig,
    compute,
)

# Keep package import lightweight for tests/environments that don't have
# optional runtime deps (e.g. hailo, GStreamer).
try:
    from .pipeline_adapter import create_app
except ImportError:  # pragma: no cover - optional runtime dependencies
    create_app = None

try:
    from .servers import SharedUIState, WebServer
except ImportError:  # pragma: no cover - optional runtime dependencies
    SharedUIState = None
    WebServer = None

__all__ = [
    "Detection",
    "RobotCommand",
    "SharedDetectionState",
    "ControllerConfig",
    "compute",
    "create_app",
    "SharedUIState",
    "WebServer",
]
