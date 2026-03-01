"""MAVSDK drone controller — all MAVSDK imports are confined to this module.

Translates between the pure VelocityCommand domain type and MAVSDK's
VelocityBodyYawspeed internally. No other module needs to import mavsdk.
"""

import asyncio
import logging
import os
import signal
import subprocess
import sys
import time
from typing import Optional
from urllib.parse import urlparse

import mavsdk
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from mavsdk.telemetry import FlightMode

from drone_follow.follow_api.types import VelocityCommand
from drone_follow.follow_api.config import ControllerConfig
from drone_follow.follow_api.controller import (
    compute_velocity_command,
    ForwardSmoother,
    _effective_target_bbox_height,
)

LOGGER = logging.getLogger("drone_follow.control")


def add_drone_args(parser) -> None:
    """Register drone connection and flight-lifecycle CLI flags on *parser*."""
    group = parser.add_argument_group("drone-connection")

    group.add_argument("--connection", default="udpin://0.0.0.0:14540",
                       help="MAVLink connection string (default: udpin://0.0.0.0:14540)")
    group.add_argument("--serial", nargs="?", const="/dev/ttyACM0", default=None,
                       metavar="DEVICE",
                       help="Connect to CubeOrange via serial cable instead of UDP. "
                            "Optionally specify device path (default: /dev/ttyACM0)")
    group.add_argument("--serial-baud", type=int, default=57600,
                       help="Baud rate for serial connection (default: 57600)")
    group.add_argument("--no-takeoff-landing", action="store_true",
                       help="Do not take off or land; assume drone is already in offboard mode")
    group.add_argument("--takeoff-altitude", type=float, default=3.0)
    group.add_argument("--mission-duration", type=float, default=300.0)


# ---------------------------------------------------------------------------
# Velocity Command API – clamps maximums & low-pass filters yaw
# ---------------------------------------------------------------------------

def _to_mavsdk(cmd: VelocityCommand) -> VelocityBodyYawspeed:
    """Translate a pure VelocityCommand to MAVSDK's type."""
    return VelocityBodyYawspeed(cmd.forward_m_s, cmd.right_m_s, cmd.down_m_s, cmd.yawspeed_deg_s)


class VelocityCommandAPI:
    """Wrapper around drone.offboard.set_velocity_body that enforces max
    velocity limits and applies an exponential low-pass filter on the yaw axis.

    Usage:
        api = VelocityCommandAPI(drone, config)
        await api.send(cmd)          # clamped + filtered
        await api.send_zero()        # immediate zero (bypasses filter)
    """

    def __init__(self, drone, config: ControllerConfig, yaw_alpha: Optional[float] = None):
        """
        Args:
            drone: MAVSDK System (or None for print-only mode).
            config: ControllerConfig used to read max_* limits.
            yaw_alpha: Low-pass filter coefficient for yaw (0..1).
                       Smaller = more smoothing, larger = faster response.
        """
        self._drone = drone
        self._config = config
        self._yaw_alpha = config.yaw_alpha if yaw_alpha is None else yaw_alpha
        self._filtered_yaw: float = 0.0

    async def send(self, cmd: VelocityCommand) -> VelocityCommand:
        """Clamp velocity components, apply yaw low-pass filter, and send.

        Returns the command that was actually sent (after clamping/filtering).
        """
        # Clamp each axis to configured maximums
        forward = max(-self._config.max_backward, min(self._config.max_forward, cmd.forward_m_s))
        right = max(-1.0, min(1.0, cmd.right_m_s))  # lateral not heavily used; keep bounded
        down = max(-self._config.max_down_speed, min(self._config.max_down_speed, cmd.down_m_s))
        yaw_raw = max(-self._config.max_yawspeed, min(self._config.max_yawspeed, cmd.yawspeed_deg_s))

        if self._config.smooth_yaw:
            # Low-pass filter on yaw: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
            self._filtered_yaw = (self._yaw_alpha * yaw_raw
                                  + (1.0 - self._yaw_alpha) * self._filtered_yaw)
            yaw_out = self._filtered_yaw
        else:
            self._filtered_yaw = yaw_raw
            yaw_out = yaw_raw

        clamped = VelocityCommand(forward, right, down, yaw_out)

        if self._drone is not None:
            await self._drone.offboard.set_velocity_body(_to_mavsdk(clamped))

        return clamped

    async def send_zero(self) -> None:
        """Send an immediate zero-velocity command and reset the yaw filter."""
        self._filtered_yaw = 0.0
        zero = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        if self._drone is not None:
            await self._drone.offboard.set_velocity_body(zero)

    async def send_raw(self, cmd: VelocityCommand) -> None:
        """Send a command without clamping or filtering (for pre-offboard setpoints)."""
        if self._drone is not None:
            await self._drone.offboard.set_velocity_body(_to_mavsdk(cmd))

    def reset_filter(self) -> None:
        """Reset the yaw low-pass filter state."""
        self._filtered_yaw = 0.0


# ---------------------------------------------------------------------------
# Detached MAVSDK Server (for graceful shutdown)
# ---------------------------------------------------------------------------

class DetachedMavsdkServer:
    """
    Manages a mavsdk_server process that is detached from the current session,
    so it doesn't die on Ctrl+C (SIGINT). This allows the Python script to
    catch SIGINT and perform a graceful landing sequence using the server.
    """
    def __init__(self, connection_url, port=50051):
        self.connection_url = connection_url
        self.port = port
        self.process = None

    def _grpc_address_from_connection(self):
        """Derive gRPC address from connection URL (host from connection, port from self.port)."""
        try:
            parsed = urlparse(self.connection_url)
            host = (parsed.hostname or "127.0.0.1").strip() or "127.0.0.1"
            if host == "0.0.0.0":
                host = "127.0.0.1"
            return f"grpc://{host}:{self.port}"
        except Exception:
            return f"grpc://127.0.0.1:{self.port}"

    def __enter__(self):
        # If already using grpc, no need to start a server
        if self.connection_url.startswith("grpc://"):
            return self.connection_url

        # Try to find mavsdk_server binary
        try:
            server_path = os.path.join(os.path.dirname(mavsdk.__file__), 'bin', 'mavsdk_server')
        except Exception:
            server_path = None

        if not server_path or not os.path.exists(server_path):
            LOGGER.warning("[drone] mavsdk_server not found at %s, using default System() behavior", server_path)
            return self.connection_url  # Fallback to default behavior

        cmd = [server_path, "-u", self.connection_url, "-p", str(self.port)]
        LOGGER.info("[drone] Starting detached mavsdk_server: %s", " ".join(cmd))

        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

        # Give server a moment to start before returning
        time.sleep(0.5)
        return self._grpc_address_from_connection()

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill()


# ---------------------------------------------------------------------------
# Offboard mode helpers
# ---------------------------------------------------------------------------

def _exit_if_not_offboard(reason: str) -> None:
    """Exit the process immediately. Use when --no-takeoff-landing and drone must be OFFBOARD."""
    LOGGER.error("[drone] %s", reason)
    sys.stderr.flush()
    os._exit(1)


async def _wait_for_offboard_mode(drone: System, shutdown: asyncio.Event) -> None:
    """Block until the drone enters OFFBOARD mode, streaming zero setpoints as keep-alive.

    In --no-takeoff-landing mode the user switches to OFFBOARD externally (e.g. via
    a GCS).  We stream zero-velocity setpoints so PX4 accepts the transition, and
    wait patiently instead of killing the process.
    """
    zero = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    setpoint_period = 0.05

    async def _stream_setpoints():
        while not shutdown.is_set():
            try:
                await drone.offboard.set_velocity_body(zero)
            except Exception:
                pass
            await asyncio.sleep(setpoint_period)

    async def _watch_for_offboard():
        async for mode in drone.telemetry.flight_mode():
            if shutdown.is_set():
                return
            if mode == FlightMode.OFFBOARD:
                LOGGER.info("[drone] OFFBOARD mode detected.")
                return
            LOGGER.info("[drone] Current mode: %s -- waiting for OFFBOARD...", mode.name)

    setpoint_task = asyncio.create_task(_stream_setpoints())
    watch_task = asyncio.create_task(_watch_for_offboard())
    shutdown_task = asyncio.create_task(shutdown.wait())
    try:
        LOGGER.info("[drone] Waiting for OFFBOARD mode (switch via GCS)...")
        done, pending = await asyncio.wait(
            [watch_task, shutdown_task],
            return_when=asyncio.FIRST_COMPLETED,
        )
        for t in pending:
            await _cancel_task(t)
    finally:
        await _cancel_task(setpoint_task)


async def _watch_offboard_mode(drone: System, shutdown: asyncio.Event) -> None:
    """Background task: if flight mode ever leaves OFFBOARD, kill the app."""
    async for mode in drone.telemetry.flight_mode():
        if shutdown.is_set():
            return
        if mode != FlightMode.OFFBOARD:
            _exit_if_not_offboard(
                f"Drone left OFFBOARD mode (current: {mode.name}). Exiting."
            )


def _print_connection_error(prefix: str, e: Exception, hint: bool = False) -> None:
    """Print a short message when failure is due to lost connection (e.g. sim closed)."""
    msg = str(e).lower()
    if "unavailable" in msg or "connection refused" in msg or "connection reset" in msg:
        LOGGER.warning("%s: connection lost (sim or MAVSDK backend closed).", prefix)
        if hint:
            LOGGER.warning("[drone] Tip: press Ctrl+C once and wait for landing before closing the sim.")
    else:
        LOGGER.warning("%s: %s", prefix, e)


def _ignore_sigint_during_landing(ignore: bool) -> None:
    """Ignore or restore SIGINT so a second Ctrl+C does not kill the process during landing."""
    try:
        if ignore:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        else:
            signal.signal(signal.SIGINT, signal.SIG_DFL)
    except (ValueError, OSError):
        pass  # signal only works in main thread; ignore


# ---------------------------------------------------------------------------
# Live Control Loop
# ---------------------------------------------------------------------------

async def _telemetry_altitude_task(drone, altitude_cache: dict, shutdown: asyncio.Event) -> None:
    """Background task: stream position and store relative altitude (m) in altitude_cache['m']."""
    try:
        async for position in drone.telemetry.position():
            if shutdown.is_set():
                return
            altitude_cache["m"] = position.relative_altitude_m
    except Exception:
        pass


async def live_control_loop(drone, shared_state, config, shutdown, altitude_cache: Optional[dict] = None, ui_state=None):
    """Control loop for Hailo modes.

    Reads detections from shared_state, computes velocity commands.
    When config.reference_altitude_m is set and altitude_cache is provided, target bbox height is scaled by altitude.
    If ui_state is provided, logs are also pushed to the web UI.
    """
    vel_api = VelocityCommandAPI(drone, config)

    def _log(msg: str, level: int = logging.INFO):
        if not LOGGER.isEnabledFor(level):
            return
        LOGGER.log(level, msg)
        if ui_state is not None:
            ui_state.push_log(msg)

    period = 1.0 / max(0.1, config.control_loop_hz)
    last_detection_time = time.monotonic()
    last_valid_detection: Optional[VelocityCommand] = None
    _prev_takeoff_alt = config.takeoff_altitude
    _goto_altitude = None
    _prev_cmd: Optional[VelocityCommand] = None
    _fwd_smoother = ForwardSmoother()

    # Constants
    _GOTO_KP = 0.5
    _GOTO_MAX_SPEED = 1.5
    _GOTO_TOLERANCE = 0.3
    _LOG_INTERVAL = 1.0
    _FWD_LOG_INTERVAL = 0.5

    # Throttle timers
    _last_log_time = 0.0
    _last_fwd_log_time = 0.0

    try:
        while not shutdown.is_set():
            now = time.monotonic()
            detection, _ = shared_state.get_latest()

            if detection is not None:
                age = now - detection.timestamp
                if age > config.detection_timeout_s:
                    detection = None
                else:
                    last_detection_time = now
                    last_valid_detection = detection

            # Check search timeout
            time_since_detection = now - last_detection_time
            if time_since_detection > config.search_timeout_s:
                _log(f"[drone] Search timeout ({config.search_timeout_s}s) exceeded - no person found. Landing...", level=logging.WARNING)
                shutdown.set()
                break

            # Detect takeoff_altitude changes and start goto
            if config.takeoff_altitude != _prev_takeoff_alt:
                _goto_altitude = config.takeoff_altitude
                _log(f"[drone] Altitude changed: going to {_goto_altitude:.1f}m", level=logging.INFO)
                _prev_takeoff_alt = config.takeoff_altitude

            target_override = None
            if altitude_cache and altitude_cache.get("m") is not None:
                target_override = _effective_target_bbox_height(config, altitude_cache["m"])
            cmd = compute_velocity_command(
                detection, config,
                target_bbox_height_override=target_override,
                last_detection=last_valid_detection,
                search_active=(time_since_detection >= config.search_enter_delay_s),
                hold_velocity=_prev_cmd,
            )

            if config.smooth_forward and not config.yaw_only:
                smoothed_fwd = _fwd_smoother.update(detection, cmd.forward_m_s, config)
                cmd = VelocityCommand(smoothed_fwd, cmd.right_m_s, cmd.down_m_s, cmd.yawspeed_deg_s)

            # Override vertical velocity when going to a new altitude
            if _goto_altitude is not None and altitude_cache.get("m") is not None:
                alt_error = _goto_altitude - altitude_cache["m"]
                if abs(alt_error) < _GOTO_TOLERANCE:
                    _log(f"[drone] Reached target altitude {_goto_altitude:.1f}m", level=logging.INFO)
                    _goto_altitude = None
                else:
                    down_speed = max(-_GOTO_MAX_SPEED, min(_GOTO_MAX_SPEED, -_GOTO_KP * alt_error))
                    cmd = VelocityCommand(cmd.forward_m_s, cmd.right_m_s, down_speed, cmd.yawspeed_deg_s)

            # Forward-velocity log (throttled)
            if now - _last_fwd_log_time >= _FWD_LOG_INTERVAL and detection is not None:
                target_bh = target_override if target_override is not None else config.target_bbox_height
                _log(f"[FWD] target={target_bh:.2f} bbox={detection.bbox_height:.2f} "
                     f"final={cmd.forward_m_s:+.2f}", level=logging.DEBUG)
                _last_fwd_log_time = now

            cmd = await vel_api.send(cmd)
            if drone is None:
                tag = "TRACK" if detection is not None else "SEARCH"
                _log(f"[{tag}] Yaw:{cmd.yawspeed_deg_s:+6.1f}\u00b0/s  "
                     f"Fwd:{cmd.forward_m_s:+5.2f}m/s  "
                     f"Down:{cmd.down_m_s:+5.2f}m/s", level=logging.INFO)
            if ui_state is not None:
                mode = "TRACK" if detection is not None else ("SEARCH" if time_since_detection >= config.search_enter_delay_s else "SEARCH-WAIT")
                ui_state.update_velocity(cmd.forward_m_s, cmd.down_m_s, cmd.yawspeed_deg_s, mode)
            _prev_cmd = cmd

            # Periodic status log to UI
            if now - _last_log_time >= _LOG_INTERVAL:
                _last_log_time = now
                alt_str = f" alt={altitude_cache['m']:.1f}m" if altitude_cache and altitude_cache.get("m") is not None else ""
                eff_str = f" eff_target={target_override:.2f}" if target_override is not None else ""
                if detection is not None:
                    _log(f"[TRACK] Yaw:{cmd.yawspeed_deg_s:+5.1f} Fwd:{cmd.forward_m_s:+5.2f} Down:{cmd.down_m_s:+5.2f}"
                         f" pos=({detection.center_x:.2f},{detection.center_y:.2f}) bbox_h={detection.bbox_height:.2f}"
                         f"{eff_str}{alt_str}", level=logging.INFO)
                elif time_since_detection < config.search_enter_delay_s:
                    _log(f"[SEARCH-WAIT] entering search in {config.search_enter_delay_s - time_since_detection:.1f}s{alt_str}", level=logging.INFO)
                else:
                    search_dir = "right" if cmd.yawspeed_deg_s > 0 else "left"
                    _log(f"[SEARCH] Spinning {search_dir} at {abs(cmd.yawspeed_deg_s):.1f} deg/s{alt_str}", level=logging.INFO)

            await asyncio.sleep(period)
    except asyncio.CancelledError:
        try:
            await vel_api.send_zero()
        except Exception:
            pass
        raise


# ---------------------------------------------------------------------------
# Offboard start / land / cancel helpers
# ---------------------------------------------------------------------------

async def _start_offboard(drone, vel_api: VelocityCommandAPI, shutdown: asyncio.Event) -> None:
    """Stream zero setpoints then start offboard mode with retries.

    PX4 requires setpoints to be streamed before offboard.start()
    (NO_SETPOINT_SET otherwise). Streams at ~20 Hz for 2 s, then
    retries offboard.start() up to 3 times.
    """
    zero = VelocityCommand(0.0, 0.0, 0.0, 0.0)
    setpoint_period_s = 0.05

    for _ in range(int(2.0 / setpoint_period_s)):
        if shutdown.is_set():
            return
        await vel_api.send_raw(zero)
        await asyncio.sleep(setpoint_period_s)

    max_retries = 3
    for attempt in range(max_retries):
        try:
            await drone.offboard.start()
            return
        except OffboardError as e:
            if attempt == max_retries - 1:
                raise
            LOGGER.warning("[drone] Failed to start offboard (%s), retrying...", e)
            for _ in range(int(1.0 / setpoint_period_s)):
                if shutdown.is_set():
                    return
                await vel_api.send_raw(zero)
                await asyncio.sleep(setpoint_period_s)


async def _land_safely(drone, vel_api: VelocityCommandAPI) -> None:
    """Stop offboard mode and land, ignoring SIGINT during the sequence."""
    try:
        await vel_api.send_zero()
        await drone.offboard.stop()
    except Exception as e:
        _print_connection_error("[drone] Offboard stop", e)

    LOGGER.warning("[drone] Landing safely - please wait (ignoring further Ctrl+C until done)...")
    try:
        _ignore_sigint_during_landing(ignore=True)
        LOGGER.info("[drone] Landing...")
        try:
            await drone.action.land()
            await asyncio.sleep(8)
        except Exception as e:
            _print_connection_error("[drone] Land", e)
    finally:
        _ignore_sigint_during_landing(ignore=False)


async def _cancel_task(task: asyncio.Task) -> None:
    """Cancel an asyncio task and suppress CancelledError."""
    task.cancel()
    try:
        await task
    except (asyncio.CancelledError, Exception):
        pass


# ---------------------------------------------------------------------------
# Main drone lifecycle
# ---------------------------------------------------------------------------

async def run_live_drone(args, shared_state, shutdown, shutdown_read_fd=None,
                         config=None, ui_state=None):
    """Connect to drone and run live control loop with Hailo detections.

    If config is provided, use it directly (allows live mutation from web UI).
    If ui_state is provided, logs are pushed to the web UI.
    """
    if config is None:
        config = ControllerConfig.from_args(args)

    if shutdown_read_fd is not None:
        loop = asyncio.get_running_loop()
        def _on_shutdown_pipe():
            try:
                os.read(shutdown_read_fd, 1)
            except (OSError, BlockingIOError):
                pass
            try:
                loop.remove_reader(shutdown_read_fd)
            except (OSError, ValueError):
                pass
            shutdown.set()
        loop.add_reader(shutdown_read_fd, _on_shutdown_pipe)

    manage_takeoff_landing = not getattr(args, 'no_takeoff_landing', False)

    with DetachedMavsdkServer(args.connection) as connection_url:
        drone = System()
        await drone.connect(system_address=connection_url)

        if manage_takeoff_landing:
            LOGGER.info("[drone] Connecting and taking off...")
        else:
            LOGGER.info("[drone] Connecting (drone must already be in OFFBOARD)...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                break

        if manage_takeoff_landing:
            await drone.action.set_takeoff_altitude(args.takeoff_altitude)
            await drone.action.arm()
            await drone.action.takeoff()
            await asyncio.sleep(15)
        else:
            await _wait_for_offboard_mode(drone, shutdown)
            if shutdown.is_set():
                return

        vel_api = VelocityCommandAPI(drone, config)
        await _start_offboard(drone, vel_api, shutdown)
        if shutdown.is_set():
            return

        if manage_takeoff_landing:
            await asyncio.sleep(3)

        altitude_cache: dict = {}
        alt_task = asyncio.create_task(_telemetry_altitude_task(drone, altitude_cache, shutdown))
        control_task = asyncio.create_task(
            live_control_loop(drone, shared_state, config, shutdown, altitude_cache, ui_state=ui_state))
        watch_task = None
        if not manage_takeoff_landing:
            watch_task = asyncio.create_task(_watch_offboard_mode(drone, shutdown))

        try:
            done, pending = await asyncio.wait(
                [
                    asyncio.create_task(shutdown.wait()),
                    asyncio.create_task(asyncio.sleep(args.mission_duration)),
                ],
                return_when=asyncio.FIRST_COMPLETED,
            )
            for t in pending:
                await _cancel_task(t)
            if shutdown.is_set():
                if manage_takeoff_landing:
                    LOGGER.warning("[drone] Shutdown requested, landing...")
                else:
                    LOGGER.warning("[drone] Shutdown requested, stopping control loop...")
        except asyncio.CancelledError:
            if manage_takeoff_landing:
                LOGGER.warning("[drone] Shutdown requested, landing...")
            else:
                LOGGER.warning("[drone] Shutdown requested, stopping control loop...")
        finally:
            await _cancel_task(alt_task)
            if watch_task is not None:
                await _cancel_task(watch_task)
            if manage_takeoff_landing:
                await _land_safely(drone, vel_api)
            await _cancel_task(control_task)
        LOGGER.info("[drone] Done.")
