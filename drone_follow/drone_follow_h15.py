#!/usr/bin/env python3
"""
Drone Follow — Hailo15 entry point.

Same as drone_follow_app.py but uses the Hailo15 native pipeline adapter
instead of hailo_apps (which isn't available on the SoC).

Usage:
    python3 -m drone_follow.drone_follow_h15 --serial
"""

import argparse
import asyncio
import logging
import os
import signal
import threading
import time
import types

# Workaround: mavsdk's generated protobuf code expects runtime_version (protobuf 5.x).
# The Yocto image ships python3-protobuf 3.x which lacks it. Inject a stub so import succeeds.
def _ensure_protobuf_runtime_version():
    import google.protobuf
    if not hasattr(google.protobuf, 'runtime_version'):
        rv = types.ModuleType('runtime_version')
        rv.__version__ = (5, 0, 0)
        rv.check = lambda *a, **k: None
        rv.ValidateProtobufRuntimeVersion = lambda *a, **k: None
        rv.Domain = type('Domain', (), {'PUBLIC': 1})()
        google.protobuf.runtime_version = rv

_ensure_protobuf_runtime_version()

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


def _build_parser() -> argparse.ArgumentParser:
    """Build the CLI parser for H15 (no hailo_apps dependency)."""
    parser = argparse.ArgumentParser(description="Drone Follow — Hailo15")

    # Controller args
    ControllerConfig.add_args(parser)

    # Drone args
    add_drone_args(parser)

    # App args
    group = parser.add_argument_group("app")
    group.add_argument("--follow-server-port", type=int, default=8080,
                       help="HTTP server port for target selection")
    group.add_argument("--log-verbosity", choices=["quiet", "normal", "debug"],
                       default="normal", help="Log verbosity level")

    # Config file support
    group.add_argument("--config", type=str, default=None,
                       help="JSON config file for controller settings")
    group.add_argument("--save-config", type=str, default=None,
                       help="Save effective config to JSON and exit")

    return parser


def main():
    parser = _build_parser()
    args = parser.parse_args()
    _configure_logging(getattr(args, "log_verbosity", "normal"))
    _resolve_serial_connection(args)

    shared_state = SharedDetectionState()
    shutdown = asyncio.Event()
    eos_reached = threading.Event()
    target_state = FollowTargetState()

    # Load config from JSON file if provided
    if getattr(args, "config", None):
        ControllerConfig.load_json_into_args(args, args.config)

    controller_config = ControllerConfig.from_args(args)

    # --save-config: dump effective config to JSON and exit
    save_path = getattr(args, "save_config", None)
    if save_path:
        controller_config.save_json(save_path)
        LOGGER.info("[app] Config saved to %s", save_path)
        raise SystemExit(0)

    # Create H15 pipeline app
    from drone_follow.pipeline_adapter.hailo15_pipeline import create_h15_app

    app = create_h15_app(
        shared_state, target_state=target_state, eos_reached=eos_reached)

    # Start follow server
    follow_server = FollowServer(target_state, shared_state, port=args.follow_server_port)
    follow_server.start()

    def _quit_pipeline():
        try:
            app.stop()
        except Exception:
            pass

    def _eos_to_shutdown():
        eos_reached.wait()
        shutdown.set()
        _quit_pipeline()
    threading.Thread(target=_eos_to_shutdown, daemon=True).start()

    def run_drone():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(
                run_live_drone(args, shared_state, shutdown,
                               config=controller_config, ui_state=None))
        except Exception:
            LOGGER.warning("[drone] Drone connection failed — pipeline continues without drone control.",
                           exc_info=True)
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

    # Run the GStreamer pipeline on the main thread
    LOGGER.info("[app] Starting Hailo15 pipeline on main thread")
    LOGGER.info("[app] View stream: gst-launch-1.0 udpsrc port=5002 ! "
                "\"application/x-rtp,encoding-name=H264\" ! rtph264depay ! decodebin ! autovideosink")
    try:
        app.run()
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        if not shutdown.is_set():
            shutdown.set()
        drone_thread.join(timeout=5.0)
        follow_server.stop()


if __name__ == "__main__":
    main()
