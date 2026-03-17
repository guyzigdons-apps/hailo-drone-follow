"""Gesture-mode drone controller and unified drone runner.

Imports VelocityCommandAPI and connection helpers from mavsdk_drone but keeps
all gesture logic self-contained. The main mavsdk_drone module is NOT modified.

The unified runner (run_unified_drone) supports runtime switching between
follow and gesture control loops based on config.follow_mode.
"""

import asyncio
import logging
import math
import time
from typing import Optional

from drone_follow.follow_api.types import VelocityCommand
from drone_follow.follow_api.config import ControllerConfig
from drone_follow.follow_api.gesture_controller import (
    compute_gesture_velocity_command,
    WaveDetector,
    _face_centering_yaw,
)

from .mavsdk_drone import (
    VelocityCommandAPI,
    live_control_loop,
    _telemetry_altitude_task,
    _wait_for_offboard_mode,
    _watch_offboard_mode,
    _start_offboard,
    _land_safely,
    _cancel_task,
    DetachedMavsdkServer,
    _wait_for_connection,
    _CONNECTION_TIMEOUT_S,
    _ARM_MAX_ATTEMPTS,
    _ARM_RETRY_DELAY_S,
    _ARM_SHUTDOWN_POLL_S,
)

import mavsdk
from mavsdk import System
from urllib.parse import urlparse

LOGGER = logging.getLogger("drone_follow.gesture")


async def gesture_control_loop(drone, gesture_state, config, shutdown,
                                altitude_cache=None, ui_state=None,
                                mode_changed=None, velocity_state=None):
    """Control loop for gesture mode.

    Handles wave detection for lock-on, acknowledgment yaw oscillation,
    and continuous gesture-based velocity commands.

    If mode_changed is provided, the loop exits when it is set.
    """
    vel_api = VelocityCommandAPI(drone, config)
    wave_detector = WaveDetector(
        reversals_needed=config.gesture_wave_reversals,
        window_s=config.gesture_wave_window_s,
    )

    def _log(msg: str, level: int = logging.INFO):
        if not LOGGER.isEnabledFor(level):
            return
        LOGGER.log(level, msg)
        if ui_state is not None:
            ui_state.push_log(msg)

    period = 1.0 / max(0.1, config.control_loop_hz)
    last_gesture_time = time.monotonic()
    last_gesture = None
    locked_on = False
    ack_start_time = None  # None = not in ack phase
    _LOG_INTERVAL = 1.0
    _last_log_time = 0.0

    try:
        while not shutdown.is_set():
            if mode_changed is not None and mode_changed.is_set():
                LOGGER.info("[gesture] Mode changed, exiting gesture control loop")
                return
            now = time.monotonic()
            gesture, _ = gesture_state.get_latest()

            if gesture is not None:
                age = now - gesture.timestamp
                if age > config.detection_timeout_s:
                    gesture = None
                else:
                    last_gesture_time = now
                    last_gesture = gesture

            time_since_gesture = now - last_gesture_time

            # Search timeout — hold position (no auto-land from gesture mode)
            if time_since_gesture > config.search_timeout_s:
                _log(f"[gesture] Search timeout ({config.search_timeout_s}s) exceeded. Holding position.",
                     level=logging.WARNING)
                await vel_api.send_zero()
                await asyncio.sleep(period)
                continue

            # Wave detection phase (before lock-on)
            if not locked_on:
                if gesture is not None and gesture.hand is not None:
                    if wave_detector.update(gesture.hand.center_x, now):
                        locked_on = True
                        ack_start_time = now
                        _log("[gesture] Wave detected! Acknowledging...", level=logging.INFO)

                # Before lock-on, just do face centering
                if gesture is not None and not locked_on:
                    yaw = _face_centering_yaw(gesture.face.center_x, config)
                    cmd = VelocityCommand(0.0, 0.0, 0.0, yaw)
                    cmd = await vel_api.send(cmd)
                elif not locked_on:
                    await vel_api.send_zero()

                if not locked_on:
                    if ui_state is not None:
                        ui_state.update_velocity(0.0, 0.0, 0.0, "WAVE-WAIT")
                    if velocity_state is not None:
                        velocity_state.update(0.0, 0.0, 0.0, 0.0, "WAVE-WAIT")
                    await asyncio.sleep(period)
                    continue

            # Acknowledgment phase: yaw oscillation
            if ack_start_time is not None:
                elapsed = now - ack_start_time
                if elapsed < config.gesture_ack_duration_s:
                    osc_yaw = config.gesture_ack_amplitude_deg * math.sin(
                        2.0 * math.pi * 4.0 * elapsed)
                    cmd = VelocityCommand(0.0, 0.0, 0.0, osc_yaw)
                    cmd = await vel_api.send(cmd)
                    if ui_state is not None:
                        ui_state.update_velocity(0.0, 0.0, osc_yaw, "ACK")
                    if velocity_state is not None:
                        velocity_state.update(0.0, 0.0, 0.0, osc_yaw, "ACK")
                    await asyncio.sleep(period)
                    continue
                else:
                    ack_start_time = None
                    _log("[gesture] Gesture control active!", level=logging.INFO)

            # Main gesture control (after lock-on + ack)
            cmd = compute_gesture_velocity_command(
                gesture, config,
                last_gesture=last_gesture,
                search_active=(time_since_gesture >= config.search_enter_delay_s),
            )

            cmd = await vel_api.send(cmd)

            # Determine mode string for UI
            if gesture is None:
                if time_since_gesture >= config.search_enter_delay_s:
                    mode = "SEARCH"
                else:
                    mode = "SEARCH-WAIT"
            elif gesture.hand is None:
                mode = "FACE-TRACK"
            elif not gesture.hand.is_open:
                mode = "FIST-STOP"
            else:
                mode = "GESTURE"

            if ui_state is not None:
                ui_state.update_velocity(cmd.forward_m_s, cmd.down_m_s, cmd.yawspeed_deg_s, mode)
            if velocity_state is not None:
                velocity_state.update(cmd.forward_m_s, cmd.right_m_s, cmd.down_m_s, cmd.yawspeed_deg_s, mode)

            # Periodic logging
            if now - _last_log_time >= _LOG_INTERVAL:
                _last_log_time = now
                if gesture is not None:
                    hand_str = ("open" if (gesture.hand and gesture.hand.is_open)
                                else "fist" if gesture.hand else "none")
                    _log(f"[{mode}] Yaw:{cmd.yawspeed_deg_s:+5.1f} Fwd:{cmd.forward_m_s:+5.2f} "
                         f"hand={hand_str} face=({gesture.face.center_x:.2f},{gesture.face.center_y:.2f})",
                         level=logging.INFO)
                else:
                    _log(f"[{mode}] No gesture detection", level=logging.INFO)

            await asyncio.sleep(period)
    except asyncio.CancelledError:
        try:
            await vel_api.send_zero()
        except Exception:
            pass
        raise


async def run_gesture_drone(args, gesture_state, shutdown, shutdown_read_fd=None,
                            config=None, ui_state=None, on_connected_cb=None):
    """Connect to drone and run gesture control loop.

    Mirrors the structure of run_live_drone() but uses gesture_control_loop.
    The original run_live_drone() is not modified.
    """
    import os

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

    manage_takeoff_landing = getattr(args, 'takeoff_landing', False)

    with DetachedMavsdkServer(args.connection) as connection_url:
        if connection_url.startswith("grpc://"):
            parsed = urlparse(connection_url)
            drone = System(mavsdk_server_address=parsed.hostname or "127.0.0.1",
                           port=parsed.port or 50051)
            await drone.connect()
        else:
            drone = System()
            await drone.connect(system_address=connection_url)

        if manage_takeoff_landing:
            LOGGER.info("[drone] Connecting and taking off...")
        else:
            LOGGER.info("[drone] Connecting (switch to OFFBOARD via GCS when ready)...")

        connected = False
        try:
            connected = await asyncio.wait_for(
                _wait_for_connection(drone), timeout=_CONNECTION_TIMEOUT_S)
        except asyncio.TimeoutError:
            pass
        if not connected:
            raise ConnectionError(
                f"No drone detected on {args.connection} after {_CONNECTION_TIMEOUT_S}s. "
                "Pipeline continues without drone control.")

        if on_connected_cb is not None:
            on_connected_cb()

        armed = False
        vel_api = VelocityCommandAPI(drone, config)
        alt_task = None
        control_task = None
        watch_task = None
        try:
            if manage_takeoff_landing:
                await drone.action.set_takeoff_altitude(args.target_altitude)
                for attempt in range(_ARM_MAX_ATTEMPTS):
                    if shutdown.is_set():
                        return
                    try:
                        await drone.action.arm()
                        armed = True
                        break
                    except mavsdk.action.ActionError as e:
                        if attempt == _ARM_MAX_ATTEMPTS - 1:
                            raise
                        LOGGER.warning(
                            "[drone] arm() failed (%s), retrying in %ds... (%d/%d)",
                            e, _ARM_RETRY_DELAY_S, attempt + 1, _ARM_MAX_ATTEMPTS - 1)
                        for _ in range(int(_ARM_RETRY_DELAY_S / _ARM_SHUTDOWN_POLL_S)):
                            if shutdown.is_set():
                                return
                            await asyncio.sleep(_ARM_SHUTDOWN_POLL_S)
                await drone.action.takeoff()
                await asyncio.sleep(15)

                await _start_offboard(drone, vel_api, shutdown)
                if shutdown.is_set():
                    return
                await asyncio.sleep(3)

                altitude_cache: dict = {}
                alt_task = asyncio.create_task(
                    _telemetry_altitude_task(drone, altitude_cache, shutdown))
                control_task = asyncio.create_task(
                    gesture_control_loop(drone, gesture_state, config, shutdown,
                                         altitude_cache, ui_state=ui_state,
                                         velocity_state=velocity_state))

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
                    LOGGER.warning("[drone] Shutdown requested, landing...")
            else:
                altitude_cache: dict = {}
                alt_task = asyncio.create_task(
                    _telemetry_altitude_task(drone, altitude_cache, shutdown))

                while not shutdown.is_set():
                    await _wait_for_offboard_mode(drone, shutdown)
                    if shutdown.is_set():
                        break

                    offboard_lost = asyncio.Event()
                    vel_api.reset_filter()
                    control_task = asyncio.create_task(
                        gesture_control_loop(drone, gesture_state, config, shutdown,
                                             altitude_cache, ui_state=ui_state,
                                             velocity_state=velocity_state))
                    watch_task = asyncio.create_task(
                        _watch_offboard_mode(drone, shutdown, offboard_lost))

                    done, pending = await asyncio.wait(
                        [
                            asyncio.create_task(shutdown.wait()),
                            asyncio.create_task(offboard_lost.wait()),
                            asyncio.create_task(asyncio.sleep(args.mission_duration)),
                        ],
                        return_when=asyncio.FIRST_COMPLETED,
                    )
                    for t in pending:
                        await _cancel_task(t)

                    await _cancel_task(control_task)
                    control_task = None
                    await _cancel_task(watch_task)
                    watch_task = None

                    try:
                        await vel_api.send_zero()
                    except Exception:
                        pass
                    try:
                        await drone.offboard.stop()
                    except Exception:
                        pass

                    if shutdown.is_set():
                        LOGGER.warning("[drone] Shutdown requested, stopping control loop...")
                        break

                    if offboard_lost.is_set():
                        LOGGER.info("[drone] Control loop paused. Waiting for OFFBOARD again...")

        except asyncio.CancelledError:
            LOGGER.warning("[drone] Shutdown requested...")
        finally:
            if alt_task is not None:
                await _cancel_task(alt_task)
            if watch_task is not None:
                await _cancel_task(watch_task)
            if control_task is not None:
                await _cancel_task(control_task)
            if manage_takeoff_landing and armed:
                await _land_safely(drone, vel_api)
        LOGGER.info("[drone] Done.")


# ---------------------------------------------------------------------------
# Unified drone runner — supports runtime mode switching
# ---------------------------------------------------------------------------

async def _mode_switching_control_loop(drone, shared_state, gesture_state, config,
                                        shutdown, altitude_cache, ui_state,
                                        pipeline_app):
    """Control loop wrapper that switches between follow and gesture based on config.follow_mode.

    When the mode changes (via web UI config update), the current inner loop is cancelled,
    the pipeline valve is toggled, and the new loop starts.
    """
    vel_api = VelocityCommandAPI(drone, config)

    first_iteration = True
    while not shutdown.is_set():
        current_mode = config.follow_mode

        # Toggle pipeline gesture inference (skip on first iteration —
        # pipeline starts with all hailonets active to avoid preroll deadlock;
        # disable_gesture is only safe once frames are flowing)
        if pipeline_app is not None:
            if current_mode == "gesture":
                pipeline_app.enable_gesture()
            elif not first_iteration:
                pipeline_app.disable_gesture()
            else:
                # Schedule deferred disable after a few frames have flowed
                async def _deferred_disable():
                    await asyncio.sleep(3.0)
                    if not shutdown.is_set() and pipeline_app is not None:
                        if config.follow_mode != "gesture":
                            pipeline_app.disable_gesture()
                asyncio.create_task(_deferred_disable())
        first_iteration = False

        # Start the appropriate control loop as a task
        if current_mode == "gesture":
            LOGGER.info("[drone] Starting GESTURE control loop")
            loop_task = asyncio.create_task(
                gesture_control_loop(
                    drone, gesture_state, config, shutdown,
                    altitude_cache=altitude_cache, ui_state=ui_state,
                    velocity_state=velocity_state))
        else:
            LOGGER.info("[drone] Starting FOLLOW control loop (mode=%s)", current_mode)
            loop_task = asyncio.create_task(
                live_control_loop(
                    drone, shared_state, config, shutdown,
                    altitude_cache=altitude_cache, ui_state=ui_state,
                    velocity_state=velocity_state))

        # Poll for mode change while the control loop runs
        try:
            while not shutdown.is_set():
                if loop_task.done():
                    # Inner loop exited on its own (e.g. search timeout)
                    break
                if config.follow_mode != current_mode:
                    LOGGER.info("[drone] Switching from %s to %s",
                                current_mode, config.follow_mode)
                    break
                await asyncio.sleep(0.25)
        except asyncio.CancelledError:
            await _cancel_task(loop_task)
            raise

        # Cancel the inner loop and send zero before switching
        await _cancel_task(loop_task)
        try:
            await vel_api.send_zero()
        except Exception:
            pass


async def run_unified_drone(args, shared_state, gesture_state, shutdown,
                             shutdown_read_fd=None, config=None, ui_state=None,
                             on_connected_cb=None, pipeline_app=None,
                             velocity_state=None):
    """Connect to drone and run the mode-switching control loop.

    Replaces both run_live_drone and run_gesture_drone. Supports runtime
    switching between follow and gesture modes via config.follow_mode.
    """
    import os

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

    manage_takeoff_landing = getattr(args, 'takeoff_landing', False)

    with DetachedMavsdkServer(args.connection) as connection_url:
        if connection_url.startswith("grpc://"):
            parsed = urlparse(connection_url)
            drone = System(mavsdk_server_address=parsed.hostname or "127.0.0.1",
                           port=parsed.port or 50051)
            await drone.connect()
        else:
            drone = System()
            await drone.connect(system_address=connection_url)

        if manage_takeoff_landing:
            LOGGER.info("[drone] Connecting and taking off...")
        else:
            LOGGER.info("[drone] Connecting (switch to OFFBOARD via GCS when ready)...")

        connected = False
        try:
            connected = await asyncio.wait_for(
                _wait_for_connection(drone), timeout=_CONNECTION_TIMEOUT_S)
        except asyncio.TimeoutError:
            pass
        if not connected:
            raise ConnectionError(
                f"No drone detected on {args.connection} after {_CONNECTION_TIMEOUT_S}s. "
                "Pipeline continues without drone control.")

        if on_connected_cb is not None:
            on_connected_cb()

        armed = False
        vel_api = VelocityCommandAPI(drone, config)
        alt_task = None
        control_task = None
        watch_task = None
        try:
            if manage_takeoff_landing:
                await drone.action.set_takeoff_altitude(args.target_altitude)
                for attempt in range(_ARM_MAX_ATTEMPTS):
                    if shutdown.is_set():
                        return
                    try:
                        await drone.action.arm()
                        armed = True
                        break
                    except mavsdk.action.ActionError as e:
                        if attempt == _ARM_MAX_ATTEMPTS - 1:
                            raise
                        LOGGER.warning(
                            "[drone] arm() failed (%s), retrying in %ds... (%d/%d)",
                            e, _ARM_RETRY_DELAY_S, attempt + 1, _ARM_MAX_ATTEMPTS - 1)
                        for _ in range(int(_ARM_RETRY_DELAY_S / _ARM_SHUTDOWN_POLL_S)):
                            if shutdown.is_set():
                                return
                            await asyncio.sleep(_ARM_SHUTDOWN_POLL_S)
                await drone.action.takeoff()
                await asyncio.sleep(15)

                await _start_offboard(drone, vel_api, shutdown)
                if shutdown.is_set():
                    return
                await asyncio.sleep(3)

                altitude_cache: dict = {}
                alt_task = asyncio.create_task(
                    _telemetry_altitude_task(drone, altitude_cache, shutdown))
                control_task = asyncio.create_task(
                    _mode_switching_control_loop(
                        drone, shared_state, gesture_state, config, shutdown,
                        altitude_cache, ui_state, pipeline_app))

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
                    LOGGER.warning("[drone] Shutdown requested, landing...")
            else:
                altitude_cache: dict = {}
                alt_task = asyncio.create_task(
                    _telemetry_altitude_task(drone, altitude_cache, shutdown))

                while not shutdown.is_set():
                    await _wait_for_offboard_mode(drone, shutdown)
                    if shutdown.is_set():
                        break

                    offboard_lost = asyncio.Event()
                    vel_api.reset_filter()
                    control_task = asyncio.create_task(
                        _mode_switching_control_loop(
                            drone, shared_state, gesture_state, config, shutdown,
                            altitude_cache, ui_state, pipeline_app))
                    watch_task = asyncio.create_task(
                        _watch_offboard_mode(drone, shutdown, offboard_lost))

                    done, pending = await asyncio.wait(
                        [
                            asyncio.create_task(shutdown.wait()),
                            asyncio.create_task(offboard_lost.wait()),
                            asyncio.create_task(asyncio.sleep(args.mission_duration)),
                        ],
                        return_when=asyncio.FIRST_COMPLETED,
                    )
                    for t in pending:
                        await _cancel_task(t)

                    await _cancel_task(control_task)
                    control_task = None
                    await _cancel_task(watch_task)
                    watch_task = None

                    try:
                        await vel_api.send_zero()
                    except Exception:
                        pass
                    try:
                        await drone.offboard.stop()
                    except Exception:
                        pass

                    if shutdown.is_set():
                        LOGGER.warning("[drone] Shutdown requested, stopping control loop...")
                        break

                    if offboard_lost.is_set():
                        LOGGER.info("[drone] Control loop paused. Waiting for OFFBOARD again...")

        except asyncio.CancelledError:
            LOGGER.warning("[drone] Shutdown requested...")
        finally:
            if alt_task is not None:
                await _cancel_task(alt_task)
            if watch_task is not None:
                await _cancel_task(watch_task)
            if control_task is not None:
                await _cancel_task(control_task)
            if manage_takeoff_landing and armed:
                await _land_safely(drone, vel_api)
        LOGGER.info("[drone] Done.")
