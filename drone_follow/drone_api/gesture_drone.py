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
    compute_gesture_lateral,
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
                                mode_changed=None, velocity_state=None,
                                palm_state=None, palm_lock=None,
                                pipeline_app=None,
                                gesture_lateral_state=None):
    """Gesture lateral overlay loop — produces lateral velocity, not full commands.

    Handles palm tracking, wave detection for lock-on, hand landmark toggling,
    and acknowledgment yaw oscillation. After lock-on, computes lateral velocity
    from hand-face X offset and writes it to gesture_lateral_state. The follow
    loop (live_control_loop) reads this lateral value and injects it into the
    follow command's right_m_s axis.

    This loop does NOT send velocity commands to the drone — follow mode handles
    all drone commands. The only exception is the acknowledgment yaw oscillation
    which is written as a lateral impulse (brief zero lateral during ack).

    If mode_changed is provided, the loop exits when it is set.
    """
    vel_api = VelocityCommandAPI(drone, config)

    # Per-palm wave detectors: {palm_track_id: WaveDetector}
    wave_detectors: dict = {}

    def _get_wave_detector(palm_id):
        if palm_id not in wave_detectors:
            wave_detectors[palm_id] = WaveDetector(
                reversals_needed=config.gesture_wave_reversals,
                window_s=config.gesture_wave_window_s,
            )
        return wave_detectors[palm_id]

    def _log(msg: str, level: int = logging.INFO):
        if not LOGGER.isEnabledFor(level):
            return
        LOGGER.log(level, msg)
        if ui_state is not None:
            ui_state.push_log(msg)

    def _set_lateral(value: float):
        if gesture_lateral_state is not None:
            gesture_lateral_state.update(value)

    period = 1.0 / max(0.1, config.control_loop_hz)
    last_gesture_time = time.monotonic()
    last_gesture = None
    locked_on = False
    locked_palm_id = None
    palm_lost_time = None  # when the locked palm was last seen
    _PALM_LOST_TIMEOUT_S = 2.0  # unlock after palm missing this long
    ack_start_time = None  # None = not in ack phase
    _LOG_INTERVAL = 1.0
    _last_log_time = 0.0

    def _unlock_palm():
        nonlocal locked_on, locked_palm_id, palm_lost_time, ack_start_time
        locked_on = False
        locked_palm_id = None
        palm_lost_time = None
        ack_start_time = None
        _set_lateral(0.0)
        if palm_lock is not None:
            palm_lock.unlock()
        if pipeline_app is not None:
            pipeline_app.disable_hand_landmarks()
        # Reset all wave detectors for fresh start
        wave_detectors.clear()
        _log("[gesture] Palm unlocked — hand landmarks disabled", level=logging.INFO)

    try:
        while not shutdown.is_set():
            if mode_changed is not None and mode_changed.is_set():
                LOGGER.info("[gesture] Mode changed, exiting gesture control loop")
                _set_lateral(0.0)
                return
            now = time.monotonic()

            # Read tracked palms
            palms = []
            if palm_state is not None:
                palms, _ = palm_state.get_latest()

            # Read gesture (only meaningful when locked + hand landmarks on)
            gesture, _ = gesture_state.get_latest()
            if gesture is not None:
                age = now - gesture.timestamp
                if age > config.detection_timeout_s:
                    gesture = None
                else:
                    last_gesture_time = now
                    last_gesture = gesture

            time_since_gesture = now - last_gesture_time

            # === WAVE DETECTION PHASE (before lock-on) ===
            if not locked_on:
                # Track which palm IDs are currently visible
                visible_palm_ids = {p.track_id for p in palms}

                # Clean up wave detectors for palms no longer visible
                stale = [pid for pid in wave_detectors if pid not in visible_palm_ids]
                for pid in stale:
                    del wave_detectors[pid]

                # Run wave detection on each visible palm
                wave_palm_id = None
                for palm in palms:
                    wd = _get_wave_detector(palm.track_id)
                    if wd.update(palm.center_x, now):
                        wave_palm_id = palm.track_id
                        break

                if wave_palm_id is not None:
                    locked_on = True
                    locked_palm_id = wave_palm_id
                    ack_start_time = now
                    if palm_lock is not None:
                        palm_lock.lock(wave_palm_id)
                    if pipeline_app is not None:
                        pipeline_app.enable_hand_landmarks()
                    _log(f"[gesture] Wave detected on palm {wave_palm_id}! "
                         f"Enabling hand landmarks...", level=logging.INFO)

                if not locked_on:
                    # No lateral before lock-on; follow loop handles everything
                    _set_lateral(0.0)
                    if velocity_state is not None:
                        velocity_state.update(0.0, 0.0, 0.0, 0.0, "WAVE-WAIT")
                    await asyncio.sleep(period)
                    continue

            # === LOCKED PHASE ===

            # Check if locked palm is still visible
            locked_palm_visible = any(p.track_id == locked_palm_id for p in palms)
            if locked_palm_visible:
                palm_lost_time = None
            elif palm_lost_time is None:
                palm_lost_time = now

            # Unlock if locked palm lost for too long
            if palm_lost_time is not None and (now - palm_lost_time) > _PALM_LOST_TIMEOUT_S:
                _unlock_palm()
                await asyncio.sleep(period)
                continue

            # Acknowledgment phase: zero lateral during ack
            if ack_start_time is not None:
                elapsed = now - ack_start_time
                if elapsed < config.gesture_ack_duration_s:
                    _set_lateral(0.0)
                    if velocity_state is not None:
                        velocity_state.update(0.0, 0.0, 0.0, 0.0, "ACK")
                    await asyncio.sleep(period)
                    continue
                else:
                    ack_start_time = None
                    _log("[gesture] Gesture control active!", level=logging.INFO)

            # Main gesture lateral (after lock-on + ack)
            lateral = compute_gesture_lateral(gesture, config)
            _set_lateral(lateral)

            # Determine mode string for logging
            if gesture is None:
                mode = "GESTURE-WAIT"
            elif gesture.hand is None:
                mode = "GESTURE-NOHAND"
            elif not gesture.hand.is_open:
                mode = "FIST-STOP"
            else:
                mode = "GESTURE"

            # Periodic logging
            if now - _last_log_time >= _LOG_INTERVAL:
                _last_log_time = now
                palm_count = len(palms)
                if gesture is not None:
                    hand_str = ("open" if (gesture.hand and gesture.hand.is_open)
                                else "fist" if gesture.hand else "none")
                    _log(f"[{mode}] Lateral:{lateral:+5.2f} "
                         f"hand={hand_str} palm={locked_palm_id} palms={palm_count} "
                         f"face=({gesture.face.center_x:.2f},{gesture.face.center_y:.2f})",
                         level=logging.INFO)
                else:
                    _log(f"[{mode}] Lateral:{lateral:+5.2f} palm={locked_palm_id} palms={palm_count} "
                         f"No gesture detection", level=logging.INFO)

            await asyncio.sleep(period)
    except asyncio.CancelledError:
        _set_lateral(0.0)
        # Clean up lock state on exit
        if palm_lock is not None and palm_lock.is_locked:
            palm_lock.unlock()
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
