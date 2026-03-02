#!/usr/bin/env python3
"""
Drone Follow — composition root and CLI entrypoint.

Wires together follow_api (pure domain logic), drone_api (MAVSDK adapter),
and pipeline_adapter (Hailo/GStreamer) into a running application.

The parser is assembled here from each domain's add_*_args() function,
so no module sees arguments it doesn't own.

Usage:
    python drone_follow.py --input rpi  # live mode with camera + drone

Pipeline options (--input, --input-codec, etc.) are passed through to the tiling pipeline.
"""

import argparse
import asyncio
import logging
import os
import signal
import threading

from follow_api import ControllerConfig, SharedDetectionState
from follow_api.state import FollowTargetState
from drone_api import run_live_drone
from drone_api.mavsdk_drone import add_drone_args

try:
    from servers import FollowServer
except ImportError:
    from .servers import FollowServer

LOGGER = logging.getLogger("drone_follow.app")


def _configure_logging(verbosity: str) -> None:
    level = {
        "quiet": logging.WARNING,
        "normal": logging.INFO,
        "debug": logging.DEBUG,
    }.get(verbosity, logging.INFO)
    logging.basicConfig(level=level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    logging.getLogger().setLevel(level)


def _resolve_serial_connection(args):
    """If --serial is given, override --connection with a serial:// URI."""
    if getattr(args, "serial", None) is not None:
        baud = getattr(args, "serial_baud", 115200)
        args.connection = f"serial://{args.serial}:{baud}"
        LOGGER.info("[drone] Serial mode: connection = %s", args.connection)


def _add_app_args(parser: argparse.ArgumentParser) -> None:
    """Register application-level CLI flags (servers, UI)."""
    group = parser.add_argument_group("app")
    group.add_argument("--follow-server-port", type=int, default=8080,
                       help="HTTP server port for target selection (only with --enable-tracking)")
    group.add_argument("--ui", action="store_true",
                       help="Enable web UI with live video and clickable bounding boxes")
    group.add_argument("--ui-port", type=int, default=5001,
                       help="Web UI server port (default: 5001)")
    group.add_argument("--ui-fps", type=int, default=10,
                       help="MJPEG stream frame rate (default: 10)")


def _build_parser() -> argparse.ArgumentParser:
    """Build the full CLI parser, assembling args from every domain.

    Each domain only registers arguments it owns:
      - follow_api:        controller gains, framing, search, smoothing, safety
      - drone_api:         MAVLink connection, flight lifecycle
      - pipeline_adapter:  tracking-lost-timeout
      - app (this file):   UI/server ports
    """
    from hailo_apps.python.core.common.core import get_pipeline_parser
    parser = get_pipeline_parser()

    ControllerConfig.add_args(parser)
    add_drone_args(parser)

    try:
        from pipeline_adapter import add_pipeline_args
    except ImportError:
        from .pipeline_adapter import add_pipeline_args
    add_pipeline_args(parser)

    _add_app_args(parser)
    return parser


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    shared_state = SharedDetectionState()
    shutdown = asyncio.Event()
    eos_reached = threading.Event()

    # Create target state for follow server
    target_state = FollowTargetState()

    # Pre-parse --ui flag to set up web UI before create_app parses all args
    ui_pre = argparse.ArgumentParser(add_help=False)
    ui_pre.add_argument("--ui", action="store_true")
    ui_pre.add_argument("--ui-port", type=int, default=5001)
    ui_pre.add_argument("--ui-fps", type=int, default=10)
    ui_pre.add_argument("--enable-tracking", action="store_true")
    ui_pre_args, _ = ui_pre.parse_known_args()

    ui_state = None
    web_server = None
    if ui_pre_args.ui:
        try:
            from servers import WebServer, SharedUIState
        except ImportError:
            from .servers import WebServer, SharedUIState
        ui_state = SharedUIState()
        # Check that the UI has been built
        _ui_build_index = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "ui", "build", "index.html")
        if not os.path.isfile(_ui_build_index):
            LOGGER.error("Web UI has not been built yet.")
            LOGGER.error("  cd hailo_apps/python/pipeline_apps/drone_follow/ui")
            LOGGER.error("  npm install")
            LOGGER.error("  npm run build")
            raise SystemExit(1)
        # Auto-enable tracking for UI (stable IDs needed for click-to-follow)
        if not ui_pre_args.enable_tracking:
            import sys as _sys
            _sys.argv.append("--enable-tracking")
            LOGGER.info("[ui] Auto-enabling tracking for UI mode")

    # Build the full parser from all domains, then pass to pipeline adapter
    parser = _build_parser()

    try:
        from pipeline_adapter import create_app
    except ImportError:
        from .pipeline_adapter import create_app

    app = create_app(shared_state, target_state=target_state, eos_reached=eos_reached,
                     ui_state=ui_state, ui_fps=ui_pre_args.ui_fps, parser=parser)
    args = app.options_menu
    _configure_logging(getattr(args, "log_verbosity", "normal"))
    _resolve_serial_connection(args)

    # Create controller config once so it can be shared (and mutated via web UI)
    controller_config = ControllerConfig.from_args(args)

    # Start follow server (always available)
    follow_server = FollowServer(target_state, shared_state, port=args.follow_server_port)
    follow_server.start()

    # Start web UI server
    if ui_state is not None:
        static_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui", "build")
        web_server = WebServer(ui_state, target_state, shared_state,
                               controller_config=controller_config,
                               port=args.ui_port, static_dir=static_dir,
                               follow_server_port=args.follow_server_port)
        web_server.start()

    def _eos_to_shutdown():
        eos_reached.wait()
        shutdown.set()
    threading.Thread(target=_eos_to_shutdown, daemon=True).start()

    def _quit_pipeline():
        """Tell GStreamer to quit (safe to call multiple times)."""
        try:
            app.loop.quit()
        except Exception:
            pass

    def run_pipeline():
        try:
            app.run()
        except SystemExit:
            pass
    pipeline_thread = threading.Thread(target=run_pipeline, daemon=False)
    pipeline_thread.start()

    def on_signal(*_):
        if not shutdown.is_set():
            shutdown.set()
            LOGGER.warning("[drone] Ctrl+C received, shutting down...")
            _quit_pipeline()

    # Register signal handlers at module level so they survive the asyncio loop
    signal.signal(signal.SIGINT, on_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, on_signal)

    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            for sig in (signal.SIGINT, signal.SIGTERM):
                loop.add_signal_handler(sig, on_signal)
        except NotImplementedError:
            pass  # already registered above
        loop.run_until_complete(
            run_live_drone(args, shared_state, shutdown,
                          config=controller_config, ui_state=ui_state))
    except KeyboardInterrupt:
        if not shutdown.is_set():
            shutdown.set()
        LOGGER.warning("[drone] Shutdown.")
        _quit_pipeline()
    except Exception:
        LOGGER.warning("[drone] Drone connection failed — pipeline continues without drone control.", exc_info=True)
    finally:
        if shutdown.is_set():
            _quit_pipeline()
        # Wait for pipeline; stay responsive to Ctrl+C
        try:
            while pipeline_thread.is_alive():
                pipeline_thread.join(timeout=1.0)
        except KeyboardInterrupt:
            if not shutdown.is_set():
                shutdown.set()
            LOGGER.warning("[drone] Ctrl+C received, shutting down...")
            _quit_pipeline()
            pipeline_thread.join(timeout=5.0)
        if web_server is not None:
            web_server.stop()
        follow_server.stop()


if __name__ == "__main__":
    main()
