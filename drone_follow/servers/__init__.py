"""servers — HTTP servers for drone follow application.

FollowServer: REST API for target selection.
WebServer: Web UI with MJPEG stream and interactive bounding boxes.
"""

from .follow_server import FollowServer, FollowServerHandler
from .video_recorder import VideoRecorder
from .web_server import SharedUIState, WebServer

__all__ = [
    "FollowServer",
    "FollowServerHandler",
    "VideoRecorder",
    "SharedUIState",
    "WebServer",
]
