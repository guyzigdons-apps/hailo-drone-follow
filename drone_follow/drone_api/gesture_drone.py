"""Gesture-mode drone controller and unified drone runner.

Imports VelocityCommandAPI and connection helpers from mavsdk_drone but keeps
all gesture logic self-contained. The main mavsdk_drone module is NOT modified.

The unified runner (run_unified_drone) supports runtime switching between
follow and gesture control loops based on config.follow_mode.
"""

import asyncio
import logging
import time

from drone_follow.follow_api.config import ControllerConfig
from drone_follow.follow_api.gesture_controller import compute_gesture_lateral

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
                                mode_changed=None, velocity_state=None,
                                palm_state=None, palm_lock=None,
                                pipeline_app=None,
                                gesture_lateral_state=None):
    """Gesture lateral overlay loop — produces lateral velocity from active palm.

    Wave detection is handled in the pipeline callback. This loop reads the gesture
    state (only populated when an active palm exists) and computes lateral velocity
    from hand-face X offset, writing it to gesture_lateral_state.

    If mode_changed is provided, the loop exits when it is set.
    """
    def _set_lateral(value: float):
        if gesture_lateral_state is not None:
            gesture_lateral_state.update(value)

    period = 1.0 / max(0.1, config.control_loop_hz)
    _LOG_INTERVAL = 1.0
    _last_log_time = 0.0

    try:
        while not shutdown.is_set():
            if mode_changed is not None and mode_changed.is_set():
                LOGGER.info("[gesture] Mode changed, exiting gesture control loop")
                _set_lateral(0.0)
                return
            now = time.monotonic()

            # Read gesture (only has hand data when callback has an active palm)
            gesture, _ = gesture_state.get_latest()
            if gesture is not None:
                age = now - gesture.timestamp
                if age > config.detection_timeout_s:
                    gesture = None

            # Compute lateral from hand position
            lateral = compute_gesture_lateral(gesture, config)
            _set_lateral(lateral)

            # Periodic logging
            if now - _last_log_time >= _LOG_INTERVAL:
                _last_log_time = now
                active_id = palm_lock.get_locked_palm_id() if palm_lock else None
                if gesture is not None and gesture.hand is not None:
                    hand_str = "open" if gesture.hand.is_open else "fist"
                    mode = "GESTURE" if gesture.hand.is_open else "FIST-STOP"
                    LOGGER.info("[%s] Lateral:%+5.2f hand=%s active=%s",
                                mode, lateral, hand_str, active_id)
                else:
                    LOGGER.info("[GESTURE-WAIT] Lateral:%+5.2f active=%s",
                                lateral, active_id)

            await asyncio.sleep(period)
    except asyncio.CancelledError:
        _set_lateral(0.0)
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
                                        pipeline_app, velocity_state=None,
                                        palm_state=None, palm_lock=None,
                                        gesture_lateral_state=None):
    """Control loop wrapper: always runs follow, optionally adds gesture lateral overlay.

    Follow mode's live_control_loop always runs and handles all drone commands (yaw,
    forward, altitude, safety). When gesture mode is active, the gesture_control_loop
    runs alongside and writes a lateral velocity to gesture_lateral_state, which
    live_control_loop reads and injects into the command's right_m_s axis.

    When the mode changes (via web UI config update), the gesture overlay task is
    started or stopped. The follow loop continues uninterrupted.
    """
    vel_api = VelocityCommandAPI(drone, config)

    first_iteration = True
    gesture_task = None

    while not shutdown.is_set():
        current_mode = config.follow_mode

        # Toggle pipeline gesture inference
        if pipeline_app is not None:
            if current_mode == "gesture":
                pipeline_app.enable_gesture()
            elif not first_iteration:
                pipeline_app.disable_gesture()
            else:
                async def _deferred_disable():
                    await asyncio.sleep(3.0)
                    if not shutdown.is_set() and pipeline_app is not None:
                        if config.follow_mode != "gesture":
                            pipeline_app.disable_gesture()
                asyncio.create_task(_deferred_disable())
        first_iteration = False

        # Always start follow loop
        LOGGER.info("[drone] Starting FOLLOW control loop (mode=%s)", current_mode)
        follow_task = asyncio.create_task(
            live_control_loop(
                drone, shared_state, config, shutdown,
                altitude_cache=altitude_cache, ui_state=ui_state,
                velocity_state=velocity_state,
                gesture_lateral_state=gesture_lateral_state if current_mode == "gesture" else None))

        # Start gesture overlay if in gesture mode
        if current_mode == "gesture":
            LOGGER.info("[drone] Starting GESTURE lateral overlay")
            gesture_task = asyncio.create_task(
                gesture_control_loop(
                    drone, gesture_state, config, shutdown,
                    altitude_cache=altitude_cache, ui_state=ui_state,
                    velocity_state=velocity_state,
                    palm_state=palm_state, palm_lock=palm_lock,
                    pipeline_app=pipeline_app,
                    gesture_lateral_state=gesture_lateral_state))
        else:
            gesture_task = None
            # Clear lateral when leaving gesture mode
            if gesture_lateral_state is not None:
                gesture_lateral_state.update(0.0)

        # Poll for mode change while the loops run
        try:
            while not shutdown.is_set():
                if follow_task.done():
                    break
                if config.follow_mode != current_mode:
                    LOGGER.info("[drone] Switching from %s to %s",
                                current_mode, config.follow_mode)
                    break
                await asyncio.sleep(0.25)
        except asyncio.CancelledError:
            await _cancel_task(follow_task)
            if gesture_task is not None:
                await _cancel_task(gesture_task)
            raise

        # Cancel loops and send zero before switching
        await _cancel_task(follow_task)
        if gesture_task is not None:
            await _cancel_task(gesture_task)
            gesture_task = None
        if gesture_lateral_state is not None:
            gesture_lateral_state.update(0.0)
        try:
            await vel_api.send_zero()
        except Exception:
            pass


async def run_unified_drone(args, shared_state, gesture_state, shutdown,
                             shutdown_read_fd=None, config=None, ui_state=None,
                             on_connected_cb=None, pipeline_app=None,
                             velocity_state=None,
                             palm_state=None, palm_lock=None,
                             dry_run=False,
                             gesture_lateral_state=None):
    """Connect to drone and run the mode-switching control loop.

    Replaces both run_live_drone and run_gesture_drone. Supports runtime
    switching between follow and gesture modes via config.follow_mode.

    If dry_run=True, skips drone connection entirely and runs the control
    loop with drone=None (VelocityCommandAPI already handles this).
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

    if dry_run:
        LOGGER.info("[drone] DRY-RUN mode — no drone connection, commands are no-ops")
        altitude_cache: dict = {}
        control_task = asyncio.create_task(
            _mode_switching_control_loop(
                None, shared_state, gesture_state, config, shutdown,
                altitude_cache, ui_state, pipeline_app,
                velocity_state=velocity_state,
                palm_state=palm_state, palm_lock=palm_lock,
                gesture_lateral_state=gesture_lateral_state))
        try:
            await shutdown.wait()
        except asyncio.CancelledError:
            pass
        finally:
            await _cancel_task(control_task)
        LOGGER.info("[drone] DRY-RUN done.")
        return

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
                        altitude_cache, ui_state, pipeline_app,
                        velocity_state=velocity_state,
                        palm_state=palm_state, palm_lock=palm_lock,
                        gesture_lateral_state=gesture_lateral_state))

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
                            altitude_cache, ui_state, pipeline_app,
                            velocity_state=velocity_state,
                            palm_state=palm_state, palm_lock=palm_lock,
                            gesture_lateral_state=gesture_lateral_state))
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
