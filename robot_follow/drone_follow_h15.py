#!/usr/bin/env python3
"""
Drone Follow — Hailo15 entry point.

Same as robot_follow_app.py but uses the Hailo15 native pipeline adapter
instead of hailo_apps (which isn't available on the SoC), and runs the
new robot_api.orchestrator.run_robot_loop in a background thread with
either a MavsdkDroneAdapter (live) or a _DryRunDroneAdapter (--dry-run).

Usage:
    python3 -m robot_follow.drone_follow_h15 --serial /dev/ttyACM0
"""

import argparse
import asyncio
import logging
import math
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

from robot_follow.follow_api import ControllerConfig, SharedDetectionState
from robot_follow.follow_api.state import FollowTargetState
from robot_follow.follow_api.types import RobotCommand, SafetyContext
from robot_follow.robot_api.adapters.mavsdk_drone import (
    DRONE_CAPS,
    MavsdkDroneAdapter,
    _reap_mavsdk_server,
    add_drone_args,
)
from robot_follow.robot_api.orchestrator import run_robot_loop
from robot_follow.servers import FollowServer
from robot_follow.servers.web_server import SharedUIState, WebServer, _WebHandler

LOGGER = logging.getLogger("robot_follow.app")


class _DryRunDroneAdapter:
    """Minimal Robot-protocol implementation that logs commands instead of
    sending them. Preserves the H15 --dry-run debugging behavior under the
    new run_robot_loop architecture (replaces the old VelocityCommandAPI(None,...)
    + live_control_loop(None,...) pattern from plan 03-07)."""

    def __init__(self, target_altitude_m: float = 0.0):
        self.caps = DRONE_CAPS
        self._target_altitude_m = target_altitude_m

    async def connect(self) -> None:
        LOGGER.info("[dry-run] connect (no wire)")

    async def start_session(self) -> None:
        LOGGER.info("[dry-run] start_session (no offboard)")

    async def send_command(self, cmd: RobotCommand, safety_ctx: SafetyContext) -> None:
        if safety_ctx.target_lost:
            return
        LOGGER.info("[dry-run] cmd forward=%.2f m/s yaw=%.2f deg/s down=%.2f m/s",
                    cmd.forward_m_s, cmd.yaw_rate, cmd.down_m_s)

    async def send_zero(self) -> None:
        LOGGER.info("[dry-run] send_zero (quiescent)")

    async def on_target_lost(self, last_detection) -> None:
        # Old behavior was yaw-spin to search; dry-run just notes the state.
        LOGGER.info("[dry-run] target lost — would search")

    async def shutdown(self) -> None:
        LOGGER.info("[dry-run] shutdown")


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


def _fix_udp_connection_url(args):
    """Rewrite udpin:// to udp:// for H15's mavsdk_server which doesn't support udpin://."""
    conn = getattr(args, "connection", "")
    if conn.startswith("udpin://"):
        fixed = conn.replace("udpin://", "udp://", 1)
        # udpin://0.0.0.0:14540 → udp://:14540 (listen mode)
        fixed = fixed.replace("udp://0.0.0.0:", "udp://:", 1)
        args.connection = fixed
        LOGGER.info("[drone] H15 UDP fix: connection = %s", args.connection)


def _disable_ipv6_loopback():
    """Disable IPv6 on the loopback interface — required for reliable mavsdk_server gRPC.

    With IPv6 enabled on lo, mavsdk_server v2.12.x's gRPC service registration
    is non-deterministic (~50% success). Disabling it makes the connection
    rock-solid (100%). Only affects loopback, not other network interfaces.
    """
    path = "/proc/sys/net/ipv6/conf/lo/disable_ipv6"
    try:
        with open(path, "r") as f:
            current = f.read().strip()
        if current == "1":
            return  # already disabled
        with open(path, "w") as f:
            f.write("1")
        LOGGER.info("[drone] Disabled IPv6 on loopback (was=%s) for mavsdk_server reliability", current)
    except (PermissionError, FileNotFoundError, OSError) as e:
        LOGGER.warning("[drone] Could not disable IPv6 loopback (%s) — mavsdk may be flaky", e)


def _fix_mavsdk_server_path():
    """Ensure mavsdk can find the server binary on H15 (Yocto installs to /usr/bin/)."""
    import shutil
    try:
        import mavsdk
        pip_path = os.path.join(os.path.dirname(mavsdk.__file__), 'bin', 'mavsdk_server')
        if not os.path.exists(pip_path):
            sys_path = shutil.which('mavsdk_server') or '/usr/bin/mavsdk_server'
            if os.path.exists(sys_path):
                os.makedirs(os.path.dirname(pip_path), exist_ok=True)
                os.symlink(sys_path, pip_path)
                LOGGER.info("[drone] Symlinked mavsdk_server: %s -> %s", pip_path, sys_path)
    except Exception as e:
        LOGGER.warning("[drone] Could not fix mavsdk_server path: %s", e)


def _build_parser() -> argparse.ArgumentParser:
    """Build the CLI parser for H15 (no hailo_apps dependency)."""
    parser = argparse.ArgumentParser(description="Drone Follow — Hailo15")

    # Controller args
    ControllerConfig.add_args(parser)

    # Drone args
    add_drone_args(parser)

    # App args — only add if not already registered by ControllerConfig/drone args
    existing = {s for a in parser._actions for s in a.option_strings}
    group = parser.add_argument_group("app")
    if "--follow-server-port" not in existing:
        group.add_argument("--follow-server-port", type=int, default=8080,
                           help="HTTP server port for target selection")
    group.add_argument("--dry-run", action="store_true",
                       help="Run control loop without drone connection — logs computed "
                            "velocity commands from live detections")
    group.add_argument("--record", nargs="?", const="auto", default=None,
                       metavar="PATH",
                       help="Record H264 video locally (Matroska container). "
                            "PATH may be a .mkv file, a directory (file auto-named "
                            "drone_<timestamp>.mkv inside it), or omitted "
                            "(defaults to /home/root/recordings/).")
    group.add_argument("-i", "--input", dest="replay", metavar="FILE",
                       help="Replay a recorded .mkv through inference + tracker instead of "
                            "the live camera. No recording, no UDP output. Implies --dry-run.")
    group.add_argument("--loop", action="store_true",
                       help="Loop the replay when it reaches EOS (only with -i/--input).")

    return parser


def main():
    parser = _build_parser()
    args = parser.parse_args()
    _configure_logging(getattr(args, "log_verbosity", "normal"))
    _resolve_serial_connection(args)
    _fix_udp_connection_url(args)
    _fix_mavsdk_server_path()
    _disable_ipv6_loopback()

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

    # Create shared UI state and H15 pipeline app
    ui_state = SharedUIState()

    # Replay mode forces dry-run (no drone control during replay)
    replay_path = getattr(args, "replay", None)
    if replay_path:
        args.dry_run = True
        LOGGER.info("[app] Replay mode (%s) — drone control disabled", replay_path)

    # Resolve record path. In replay mode, only the .jsonl detection log is
    # written (no encoder branch is built, so no .mkv video).
    record_path = None
    if getattr(args, "record", None) is not None:
        import datetime
        import shutil
        record_path = args.record
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        suffix = "_replay" if replay_path else ""
        filename = f"drone_{ts}{suffix}.mkv"
        fallback_dir = "/home/root/recordings"

        if record_path == "auto":
            record_dir = fallback_dir
        elif record_path.endswith(".mkv"):
            record_dir = os.path.dirname(record_path)
            filename = os.path.basename(record_path)
        else:
            # Treat as a directory; auto-name the file inside it
            record_dir = record_path

        # If the user asked for a path under /mnt or /media but nothing is
        # actually mounted there, fall back to internal storage. Otherwise
        # we'd silently fill the rootfs at the unmounted mount point.
        if record_dir.startswith(("/mnt/", "/media/")):
            mounted = False
            probe = record_dir
            while probe and probe != "/":
                if os.path.ismount(probe):
                    mounted = True
                    break
                probe = os.path.dirname(probe)
            if not mounted:
                LOGGER.warning("[app] %s is not mounted — recording to %s instead",
                               record_dir, fallback_dir)
                record_dir = fallback_dir

        record_path = os.path.join(record_dir, filename)
        os.makedirs(record_dir, exist_ok=True)
        # Warn (don't fail) if the target volume looks too small.
        free_gb = shutil.disk_usage(record_dir).free / (1024**3)
        if free_gb < 1.0:
            LOGGER.warning("[app] Only %.2f GB free at %s — recording may be truncated",
                           free_gb, record_dir)
        else:
            LOGGER.info("[app] Recording target has %.1f GB free", free_gb)
        if replay_path:
            LOGGER.info("[app] Recording detections to %s (video not re-encoded in replay mode)",
                        record_path.rsplit(".", 1)[0] + ".jsonl")
        else:
            LOGGER.info("[app] Recording video to %s", record_path)

    from robot_follow.pipeline_adapter.hailo15_pipeline import create_h15_app

    app = create_h15_app(
        shared_state, target_state=target_state, eos_reached=eos_reached,
        ui_state=ui_state, record_path=record_path, replay_path=replay_path,
        replay_loop=getattr(args, "loop", False))

    # Start follow server
    follow_server = FollowServer(target_state, shared_state, port=args.follow_server_port)
    follow_server.start()

    # Patch follow routes into the WebServer handler so everything goes through
    # port 5001 (avoids needing a second SSH tunnel for the FollowServer port).
    _orig_do_post = _WebHandler.do_POST

    def _patched_do_post(self):
        if self.path.startswith("/api/follow/") or self.path.startswith("/follow/"):
            suffix = self.path.split("/follow/", 1)[1]
            if suffix in ("clear", ""):
                target_state.set_target(None)
                self._send_json({"status": "success", "following_id": None})
            else:
                try:
                    det_id = int(suffix)
                except ValueError:
                    self.send_error(400, f"Invalid detection ID: {suffix}")
                    return
                available = shared_state.get_available_ids()
                if det_id not in available:
                    self._send_json({"status": "error",
                                     "available_ids": list(available)}, status=404)
                    return
                target_state.set_target(det_id)
                self._send_json({"status": "success", "following_id": det_id})
        else:
            _orig_do_post(self)

    _WebHandler.do_POST = _patched_do_post

    # Start web server (MJPEG + detections + config UI)
    ui_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui", "build")
    LOGGER.info("[app] UI static dir: %s (exists=%s)", ui_dir, os.path.isdir(ui_dir))
    web_server = WebServer(
        ui_state, target_state=target_state, shared_state=shared_state,
        controller_config=controller_config, port=5001, static_dir=ui_dir,
        follow_server_port=5001, recording_ctl=app)
    web_server.start()
    LOGGER.info("[app] Web UI at http://10.0.0.1:5001")

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

    def run_robot():
        """Run the new robot_api orchestrator in a background thread with
        its own asyncio loop. Mirrors robot_follow_app.py:run_robot, sans
        the rover branch (H15 is drone-only) and adapted for --dry-run."""
        if getattr(args, "dry_run", False):
            adapter = _DryRunDroneAdapter(
                target_altitude_m=getattr(args, "target_altitude", 0.0))
            LOGGER.info("[app] Dry-run control loop (no drone connection)")
        else:
            adapter = MavsdkDroneAdapter(args, controller_config)
            LOGGER.info("[drone] Thread started, connection=%s", args.connection)

        async def _main():
            duration = getattr(args, "mission_duration", math.inf)
            loop_task = asyncio.create_task(
                run_robot_loop(adapter, shared_state, controller_config,
                               shutdown, ui_state=ui_state))
            deadline_task = asyncio.create_task(asyncio.sleep(duration))
            try:
                await asyncio.wait(
                    [loop_task, deadline_task],
                    return_when=asyncio.FIRST_COMPLETED)
            finally:
                for t in (loop_task, deadline_task):
                    if not t.done():
                        t.cancel()
                        try:
                            await t
                        except asyncio.CancelledError:
                            pass
                        except Exception:
                            LOGGER.warning("[robot] background task raised on shutdown",
                                           exc_info=True)

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(_main())
        except Exception:
            LOGGER.warning("[robot] Control loop failed — pipeline continues without drone control.",
                           exc_info=True)
        finally:
            LOGGER.info("[robot] Thread exiting")
            loop.close()

    drone_thread = threading.Thread(target=run_robot, daemon=True)
    drone_thread.start()
    LOGGER.info("[app] Robot control started in background thread")

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
    try:
        app.run()
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        if not shutdown.is_set():
            shutdown.set()
        # Wait for robot thread to finish cleanly. _land_safely does an 8s
        # sleep after issuing land(), so allow generous time.
        drone_thread.join(timeout=20.0)
        if drone_thread.is_alive():
            # Robot thread is stuck (typically a MAVSDK land/offboard timeout).
            # Its `with DetachedMavsdkServer` __exit__ won't run, so the
            # mavsdk_server child process would survive us and keep UDP 14540
            # + TCP 50051 bound, blocking the next run. Reap by name.
            _reap_mavsdk_server()
        follow_server.stop()
        web_server.stop()


if __name__ == "__main__":
    main()
