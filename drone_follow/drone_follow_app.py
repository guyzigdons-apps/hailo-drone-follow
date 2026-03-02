#!/usr/bin/env python3
"""
Drone Follow — composition root and CLI entrypoint.

Wires together follow_api (pure domain logic), drone_api (MAVSDK adapter),
and pipeline_adapter (Hailo/GStreamer) into a running application.

The parser is assembled here from each domain's add_*_args() function,
so no module sees arguments it doesn't own.

Usage:
    python drone_follow_app.py --input rpi  # live mode with camera + drone

Pipeline options (--input, --input-codec, etc.) are passed through to the tiling pipeline.
"""

import argparse
import asyncio
import logging
import os
import signal
import threading

from drone_follow.follow_api import ControllerConfig, SharedDetectionState
from drone_follow.follow_api.state import FollowTargetState
from drone_follow.drone_api import run_live_drone
from drone_follow.drone_api.mavsdk_drone import add_drone_args
from drone_follow.servers import FollowServer

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
                       help="HTTP server port for target selection")
    group.add_argument("--ui", action="store_true",
                       help="Enable web UI with live video and clickable bounding boxes")
    group.add_argument("--ui-port", type=int, default=5001,
                       help="Web UI server port (default: 5001)")
    group.add_argument("--ui-fps", type=int, default=10,
                       help="MJPEG stream frame rate (default: 10)")
    group.add_argument("--record", action="store_true",
                       help="Record raw video + detections for the entire session (requires --ui)")


def _build_parser() -> argparse.ArgumentParser:
    """Build the full CLI parser, assembling args from every domain.

    Each domain only registers arguments it owns:
      - follow_api:        controller gains, framing, search, smoothing, safety
      - drone_api:         MAVLink connection, flight lifecycle
            - pipeline_adapter:  pipeline-specific integration args
      - app (this file):   UI/server ports
    """
    from hailo_apps.python.core.common.core import get_pipeline_parser
    parser = get_pipeline_parser()

    ControllerConfig.add_args(parser)
    add_drone_args(parser)

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
    ui_pre.add_argument("--record", action="store_true")
    ui_pre_args, _ = ui_pre.parse_known_args()

    ui_state = None
    web_server = None
    if ui_pre_args.ui:
        from drone_follow.servers import WebServer, SharedUIState
        ui_state = SharedUIState()
        # Check that the UI has been built
        _ui_build_index = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "ui", "build", "index.html")
        if not os.path.isfile(_ui_build_index):
            LOGGER.error("Web UI has not been built yet.")
            LOGGER.error("  cd drone_follow/ui")
            LOGGER.error("  npm install")
            LOGGER.error("  npm run build")
            raise SystemExit(1)
    # Build the full parser from all domains, then pass to pipeline adapter
    parser = _build_parser()

    from drone_follow.pipeline_adapter import create_app

    recordings_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
    app = create_app(shared_state, target_state=target_state, eos_reached=eos_reached,
                     ui_state=ui_state, ui_fps=ui_pre_args.ui_fps, parser=parser,
                     record_dir=recordings_dir)
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
                               follow_server_port=args.follow_server_port,
                               recording_ctl=app)

        web_server.start()

    def _quit_pipeline():
        """Tell GStreamer to quit (safe to call multiple times)."""
        try:
            app.loop.quit()
        except Exception:
            pass

    def _eos_to_shutdown():
        eos_reached.wait()
        shutdown.set()
        _quit_pipeline()
    threading.Thread(target=_eos_to_shutdown, daemon=True).start()

    def run_drone():
        """Run drone control in a background thread with its own asyncio loop."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(
                run_live_drone(args, shared_state, shutdown,
                              config=controller_config, ui_state=ui_state))
        except Exception:
            LOGGER.warning("[drone] Drone connection failed — pipeline continues without drone control.", exc_info=True)
        finally:
            loop.close()

    drone_thread = threading.Thread(target=run_drone, daemon=True)
    drone_thread.start()
    LOGGER.info("[app] Drone control started in background thread")

    def on_signal(*_):
        if not shutdown.is_set():
            shutdown.set()
            LOGGER.warning("[drone] Ctrl+C received, shutting down...")
            _quit_pipeline()

    signal.signal(signal.SIGINT, on_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, on_signal)

    # Start recording from CLI flag after pipeline is running
    if ui_pre_args.record and ui_state is not None:
        # Schedule recording start after pipeline enters PLAYING state
        def _start_recording_delayed():
            import time as _time
            _time.sleep(1.0)  # wait for pipeline to reach PLAYING
            app.start_recording()
        threading.Thread(target=_start_recording_delayed, daemon=True).start()

    # Run the GStreamer pipeline on the main thread (UI + Hailo start immediately)
    LOGGER.info("[app] Starting Hailo pipeline and UI on main thread")
    try:
        app.run()
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        if not shutdown.is_set():
            shutdown.set()
        if app.is_recording:
            app.stop_recording()
        # Wait for drone thread to finish cleanly
        drone_thread.join(timeout=5.0)
        if web_server is not None:
            web_server.stop()
        follow_server.stop()


if __name__ == "__main__":
    main()
