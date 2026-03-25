"""Hailo tiling pipeline adapter — all Hailo/GStreamer imports are confined here.

Translates Hailo detection objects into the pure Detection domain type.
No other module needs to import hailo or gi.repository.
"""

import argparse
import logging
import os
import subprocess
import threading
import time
from datetime import datetime
from typing import Optional

import hailo
import numpy as np

from drone_follow.follow_api.types import Detection, VelocityCommand
from drone_follow.follow_api.controller import compute_velocity_command

from .byte_tracker import ByteTracker

LOGGER = logging.getLogger("drone_follow.app")

_EMPTY_DET_ARRAY = np.empty((0, 5), dtype=np.float32)

_gst_module = None


def _get_gst():
    """Import and cache GStreamer bindings (deferred to avoid import at module level)."""
    global _gst_module
    if _gst_module is None:
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        _gst_module = Gst
    return _gst_module


# ---------------------------------------------------------------------------
# Callback helpers
# ---------------------------------------------------------------------------

def _build_det_info(person, track_id=None):
    """Build a UI detection dict from a Hailo detection object."""
    pbbox = person.get_bbox()
    det_info = {
        "label": "person",
        "confidence": round(person.get_confidence(), 3),
        "bbox": {
            "x": round(pbbox.xmin(), 4),
            "y": round(pbbox.ymin(), 4),
            "w": round(pbbox.width(), 4),
            "h": round(pbbox.height(), 4),
        },
    }
    if track_id is not None:
        det_info["id"] = track_id
    return det_info


def _update_ui(ui_state, persons, person_to_id, following_id):
    """Push detection metadata to the web UI if enabled."""
    if ui_state is None:
        return
    all_dets = [_build_det_info(p, person_to_id.get(id(p))) for p in persons]
    ui_state.update_detections(all_dets, following_id)


def _run_tracker(byte_tracker, persons):
    """Run ByteTracker and return (available_ids, person_by_id, person_to_id).

    person_by_id:  {track_id -> person detection}
    person_to_id:  {id(person) -> track_id}  (reverse lookup)
    """
    available_ids = set()
    person_by_id = {}

    SCALE = 1000.0
    det_array = np.empty((len(persons), 5), dtype=np.float32)
    for i, person in enumerate(persons):
        bbox = person.get_bbox()
        det_array[i, 0] = bbox.xmin() * SCALE
        det_array[i, 1] = bbox.ymin() * SCALE
        det_array[i, 2] = (bbox.xmin() + bbox.width()) * SCALE
        det_array[i, 3] = (bbox.ymin() + bbox.height()) * SCALE
        det_array[i, 4] = person.get_confidence()

    all_tracks = byte_tracker.update(det_array)

    for t in all_tracks:
        if t.is_activated and 0 <= t.input_index < len(persons):
            available_ids.add(t.track_id)
            person_by_id[t.track_id] = persons[t.input_index]
        elif t.is_activated:
            available_ids.add(t.track_id)

    person_to_id = {id(p): tid for tid, p in person_by_id.items()}
    return available_ids, person_by_id, person_to_id


# ---------------------------------------------------------------------------
# HUD overlay helpers
# ---------------------------------------------------------------------------

def _get_velocity(velocity_state, detection, config):
    """Get current velocity command, computing locally if the control loop isn't running.

    Priority:
      1. velocity_state from the drone control loop (if mode != IDLE)
      2. Local computation from detection + config (pure math, no drone needed)
    """
    if velocity_state is not None:
        fwd, right, down, yaw, mode = velocity_state.get()
        if mode != "IDLE":
            return fwd, right, down, yaw, mode

    # Control loop not running — compute locally from detection
    if detection is not None and config is not None:
        cmd = compute_velocity_command(detection, config)
        return cmd.forward_m_s, cmd.right_m_s, cmd.down_m_s, cmd.yawspeed_deg_s, "TRACK"

    if config is not None:
        # No detection, compute search command
        cmd = compute_velocity_command(None, config)
        if cmd.yawspeed_deg_s != 0:
            return cmd.forward_m_s, cmd.right_m_s, cmd.down_m_s, cmd.yawspeed_deg_s, "SEARCH"

    return 0.0, 0.0, 0.0, 0.0, "IDLE"


def _attach_velocity_arrows(roi, velocity_state, detection=None, config=None):
    """Attach overlay_arrow and overlay_text metadata to ROI for HUD visualization.

    RC stick layout (bottom of frame):
      Left stick:  yaw (horizontal) + altitude (vertical)
      Right stick: lateral/roll (horizontal) + forward/backward (vertical)

    Works in three modes:
      - With drone connected: reads actual commands from velocity_state
      - Without drone: computes expected commands locally from detection + config
      - No detection: shows search/idle state
    """
    fwd, right, down, yaw, mode = _get_velocity(velocity_state, detection, config)

    # Stick positions (bottom of frame, spaced apart like RC sticks)
    left_cx, left_cy = 0.35, 0.85    # left stick
    right_cx, right_cy = 0.65, 0.85  # right stick

    # Scaling: MAX values = full-length arrow. Set to typical operating range,
    # not theoretical max, so arrows are clearly visible and proportional.
    MAX_ARROW_LEN = 0.15
    MAX_YAW = 25.0       # deg/s — typical tracking range is 5-25°/s
    MAX_FWD = 2.0         # m/s
    MAX_DOWN = 1.5        # m/s
    MAX_LAT = 1.5         # m/s — matches gesture_max_lateral

    def _arrow(cx, cy, angle, length, r, g, b, thickness=3):
        label = f"x:{cx},y:{cy},angle:{angle},len:{length:.4f},r:{r},g:{g},b:{b},t:{thickness}"
        roi.add_object(hailo.HailoClassification("overlay_arrow", 0, label, 0.0))

    # --- Left stick: yaw (horizontal) + altitude (vertical) ---
    # Yaw: 0 = right, 180 = left — cyan
    if abs(yaw) > 1.0:
        yaw_len = min(abs(yaw) / MAX_YAW, 1.0) * MAX_ARROW_LEN
        _arrow(left_cx, left_cy, 0.0 if yaw > 0 else 180.0, yaw_len, 0, 220, 255, 4)

    # Altitude: 90 = up (climb), 270 = down (descend) — yellow
    if abs(down) > 0.05:
        alt_len = min(abs(down) / MAX_DOWN, 1.0) * MAX_ARROW_LEN
        _arrow(left_cx, left_cy, 90.0 if down < 0 else 270.0, alt_len, 255, 220, 0, 3)

    # Left stick crosshair
    _arrow(left_cx, left_cy, 0, 0.002, 255, 255, 255, 4)

    # --- Right stick: forward/backward (vertical) + lateral (horizontal) ---
    # Forward: 90 = up, 270 = down — green/orange
    if abs(fwd) > 0.05:
        fwd_len = min(abs(fwd) / MAX_FWD, 1.0) * MAX_ARROW_LEN
        if fwd > 0:
            _arrow(right_cx, right_cy, 90.0, fwd_len, 0, 255, 100, 4)
        else:
            _arrow(right_cx, right_cy, 270.0, fwd_len, 255, 80, 0, 4)

    # Lateral: 0 = right, 180 = left — magenta
    if abs(right) > 0.05:
        lat_len = min(abs(right) / MAX_LAT, 1.0) * MAX_ARROW_LEN
        _arrow(right_cx, right_cy, 0.0 if right > 0 else 180.0, lat_len, 255, 0, 255, 4)

    # Right stick crosshair
    _arrow(right_cx, right_cy, 0, 0.002, 255, 255, 255, 4)

    # Mode text (centered between sticks)
    if mode:
        if mode == "TRACK" or mode == "ORBIT" or mode == "GESTURE":
            color = "r:0,g:255,b:100"
        elif mode.startswith("SEARCH"):
            color = "r:255,g:220,b:0"
        elif mode == "IDLE":
            color = "r:128,g:128,b:128"
        else:
            color = "r:200,g:200,b:200"
        label = f"x:0.47,y:{left_cy + 0.05},text:{mode},{color},scale:0.6,bg:1"
        roi.add_object(hailo.HailoClassification("overlay_text", 0, label, 0.0))


# ---------------------------------------------------------------------------
# Main app callback
# ---------------------------------------------------------------------------

def app_callback(element, buffer, user_data):
    """Tiling pipeline callback: pick largest person (or specific tracked person), update shared state.

    ByteTracker runs synchronously in the callback:
    1. Convert detections to Nx5 array, run tracker.update() synchronously
    2. Each returned track has input_index pointing to the matched detection
    3. Build person_by_id directly -- no cross-frame IoU re-matching needed
    """
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    persons = [d for d in detections if d.get_label() == "person"]

    target_state = user_data.target_state
    ui_state = user_data.ui_state
    # selected_detection will be set if we find a person to follow
    selected_detection = None

    if not persons:
        user_data.byte_tracker.update(_EMPTY_DET_ARRAY)
        user_data.shared_state.update(None, available_ids=set())
        if target_state is not None and target_state.get_target() is not None:
            target_state.set_target(None)
        _update_ui(ui_state, [], {}, target_state.get_target() if target_state else None)
        if target_state is None or target_state.get_target() is None:
            LOGGER.debug("[SEARCH MODE] No person detected in frame - follow state cleared")
    else:
        available_ids, person_by_id, person_to_id = _run_tracker(
            user_data.byte_tracker, persons)

        # --- Target selection ---
        target_id = target_state.get_target() if target_state is not None else None

        best = None
        follow_mode = ""
        if target_id is not None:
            best = person_by_id.get(target_id)

            if best is not None:
                target_state.update_last_seen()
                follow_mode = f"ID {target_id}"
            else:
                user_data.shared_state.update(None, available_ids=available_ids)
                if target_state.get_target() is not None:
                    target_state.set_target(None)
                _update_ui(ui_state, persons, person_to_id, target_state.get_target())
                if target_state.get_target() is None:
                    LOGGER.debug("[SEARCH MODE] Target ID %s not in frame. Available: %s - follow state cleared",
                                target_id, sorted(available_ids) if available_ids else "none")
                best = None  # signal: no valid target this frame
        else:
            best = max(persons, key=lambda d: d.get_bbox().width() * d.get_bbox().height())
            best_tid = person_to_id.get(id(best))
            if best_tid is not None and target_state is not None:
                target_state.set_target(best_tid)
                follow_mode = f"locked ID {best_tid}"
            elif best_tid is not None:
                follow_mode = f"largest (ID {best_tid})"
            else:
                follow_mode = "largest (no tracking)"

        if best is not None:
            bbox = best.get_bbox()
            cx = bbox.xmin() + bbox.width() / 2
            cy = bbox.ymin() + bbox.height() / 2
            selected_detection = Detection(
                label="person",
                confidence=best.get_confidence(),
                center_x=cx,
                center_y=cy,
                bbox_height=bbox.height(),
                timestamp=time.monotonic(),
            )
            user_data.shared_state.update(selected_detection, available_ids=available_ids)

            _update_ui(ui_state, persons, person_to_id,
                       target_state.get_target() if target_state else None)

            available_str = f"Available: {sorted(available_ids)}" if available_ids else ""
            LOGGER.debug("[FOLLOWING %s] conf=%.2f center=(%.2f,%.2f) h=%.2f %s",
                        follow_mode, best.get_confidence(), cx, cy, bbox.height(), available_str)

    # Attach flight command HUD arrows (runs every frame, with or without drone connection)
    velocity_state = getattr(user_data, 'velocity_state', None)
    config = getattr(user_data, 'controller_config', None)
    _attach_velocity_arrows(roi, velocity_state, detection=selected_detection, config=config)


# ---------------------------------------------------------------------------
# Pipeline app factory
# ---------------------------------------------------------------------------


def create_app(shared_state, target_state=None, eos_reached=None, ui_state=None, ui_fps=10,
               parser: Optional[argparse.ArgumentParser] = None, record_dir=None,
               velocity_state=None, controller_config=None, no_overlay=False):
    """Create the tiling pipeline app with drone-follow callback.

    Follows the hailo-app pattern: build parser, create user_data,
    instantiate GStreamerTilingApp. If eos_reached is a threading.Event,
    EOS will set it instead of calling GStreamer shutdown (so we can land first).

    Args:
        shared_state: SharedDetectionState for passing detections to control loop
        target_state: FollowTargetState for tracking-based target selection (optional)
        eos_reached: threading.Event to signal EOS instead of shutdown (optional)
        ui_state: SharedUIState for web UI (optional)
        ui_fps: MJPEG stream frame rate (default: 10)
        parser: Pre-built argparse parser with all domain args already registered.
                If None, a bare pipeline parser is created (for backward compat).
        record_dir: Directory for recording output files (optional)
    """
    from hailo_apps.python.pipeline_apps.tiling.tiling_pipeline import (
        GStreamerTilingApp,
    )
    from hailo_apps.python.core.gstreamer.gstreamer_app import app_callback_class
    from hailo_apps.python.core.common.core import get_pipeline_parser
    from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
        QUEUE, DISPLAY_PIPELINE,
        INFERENCE_PIPELINE, USER_CALLBACK_PIPELINE,
        TILE_CROPPER_PIPELINE, SOURCE_PIPELINE,
    )

    if parser is None:
        parser = get_pipeline_parser()

    class DroneFollowUserData(app_callback_class):
        def __init__(self, shared_state, target_state=None, ui_state=None,
                     byte_tracker=None, velocity_state=None, controller_config=None):
            super().__init__()
            self.shared_state = shared_state
            self.target_state = target_state
            self.ui_state = ui_state
            self.byte_tracker = byte_tracker
            self.velocity_state = velocity_state
            self.controller_config = controller_config

    class DroneFollowTilingApp(GStreamerTilingApp):
        """Tiling app with EOS handling and optional MJPEG appsink for web UI."""
        def __init__(self, app_callback, user_data, parser=None, eos_reached=None,
                     ui_enabled=False, ui_state=None, ui_fps=30, record_dir=None,
                     no_overlay=False):
            self._eos_reached = eos_reached
            self._ui_enabled = ui_enabled
            self._ui_state = ui_state
            self._ui_fps = ui_fps
            self._no_overlay = no_overlay
            self._recording = False
            self._record_dir = record_dir or os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "..", "recordings")
            self._record_lock = threading.Lock()
            self._ffmpeg_proc = None
            super().__init__(app_callback, user_data, parser=parser)
            # Connect appsink after pipeline is created by super().__init__
            if self._ui_enabled:
                self._connect_mjpeg_sink()

        def _connect_mjpeg_sink(self):
            """Connect the MJPEG appsink and record appsink new-sample signals."""
            self._Gst = _get_gst()
            mjpeg_sink = self.pipeline.get_by_name("mjpeg_sink")
            if mjpeg_sink:
                mjpeg_sink.connect("new-sample", self._on_mjpeg_sample)
            record_sink = self.pipeline.get_by_name("record_appsink")
            if record_sink:
                record_sink.connect("new-sample", self._on_record_sample)

        def _on_mjpeg_sample(self, appsink):
            """appsink callback: extract pre-encoded JPEG bytes."""
            Gst = self._Gst
            sample = appsink.emit("pull-sample")
            if sample:
                buf = sample.get_buffer()
                success, map_info = buf.map(Gst.MapFlags.READ)
                if success:
                    self._ui_state.update_frame(bytes(map_info.data))
                    buf.unmap(map_info)
            return Gst.FlowReturn.OK

        def _on_record_sample(self, appsink):
            """appsink callback: pipe raw RGB frames to ffmpeg subprocess."""
            Gst = self._Gst
            sample = appsink.emit("pull-sample")
            if sample and self._recording and self._ffmpeg_proc:
                buf = sample.get_buffer()
                ok, mapinfo = buf.map(Gst.MapFlags.READ)
                if ok:
                    try:
                        self._ffmpeg_proc.stdin.write(mapinfo.data)
                    except (BrokenPipeError, OSError):
                        pass
                    buf.unmap(mapinfo)
            return Gst.FlowReturn.OK

        def on_eos(self):
            if self._eos_reached is not None:
                self._eos_reached.set()
            else:
                super().on_eos()

        def _on_pipeline_rebuilt(self):
            super()._on_pipeline_rebuilt()
            if self._ui_enabled:
                self._connect_mjpeg_sink()

        # ---- Recording control ----

        @property
        def is_recording(self):
            return self._recording

        def _generate_record_path(self):
            os.makedirs(self._record_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            return os.path.join(self._record_dir, f"rec_{ts}.mkv")

        def start_recording(self, path=None):
            """Spawn ffmpeg subprocess and open valve. Returns the output file path."""
            with self._record_lock:
                if self._recording:
                    LOGGER.warning("[record] Already recording")
                    return None
                if not self._ui_enabled:
                    LOGGER.error("[record] Recording requires UI pipeline (--ui)")
                    return None

                valve = self.pipeline.get_by_name("record_valve")
                if valve is None:
                    LOGGER.error("[record] record_valve not found in pipeline")
                    return None

                record_path = path or self._generate_record_path()
                width, height = self.video_width, self.video_height
                fps = self.frame_rate

                self._ffmpeg_proc = subprocess.Popen([
                    "ffmpeg", "-y", "-nostdin",
                    "-f", "rawvideo", "-pix_fmt", "rgb24",
                    "-s", f"{width}x{height}", "-r", str(fps),
                    "-i", "pipe:0",
                    "-c:v", "libx264", "-preset", "ultrafast",
                    "-tune", "zerolatency", "-b:v", "5000k",
                    record_path,
                ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                valve.set_property("drop", False)
                self._recording = True
                self._current_record_path = record_path
                LOGGER.info("[record] Started recording to %s", record_path)
                return record_path

        def stop_recording(self):
            """Close valve and finalize ffmpeg in background. Non-blocking."""
            with self._record_lock:
                if not self._recording:
                    return None

                valve = self.pipeline.get_by_name("record_valve")
                if valve:
                    valve.set_property("drop", True)

                self._recording = False
                path = self._current_record_path
                proc = self._ffmpeg_proc
                self._ffmpeg_proc = None

                def _finalize():
                    try:
                        if proc and proc.stdin:
                            proc.stdin.close()
                        if proc:
                            proc.wait(timeout=5)
                    except Exception:
                        LOGGER.exception("[record] ffmpeg finalize error")
                    LOGGER.info("[record] Finalized: %s", path)

                threading.Thread(target=_finalize, daemon=True).start()

                LOGGER.info("[record] Stopped recording: %s", path)
                return path

        def get_pipeline_string(self):
            if not self._ui_enabled:
                return super().get_pipeline_string()

            # Build pipeline with tee: one branch for display, one for MJPEG appsink
            source_pipeline = SOURCE_PIPELINE(
                video_source=self.video_source,
                video_width=self.video_width,
                video_height=self.video_height,
                frame_rate=self.frame_rate,
                sync=self.sync,
                mirror_image=False,
            )

            detection_pipeline = INFERENCE_PIPELINE(
                hef_path=self.hef_path,
                post_process_so=self.post_process_so,
                post_function_name=self.post_function,
                batch_size=self.batch_size,
                config_json=self.labels_json,
            )

            tiling_mode = 1 if self.use_multi_scale else 0
            scale_level = self.scale_level if self.use_multi_scale else 0
            tile_cropper_pipeline = TILE_CROPPER_PIPELINE(
                detection_pipeline,
                name='tile_cropper_wrapper',
                internal_offset=True,
                scale_level=scale_level,
                tiling_mode=tiling_mode,
                tiles_along_x_axis=self.tiles_x,
                tiles_along_y_axis=self.tiles_y,
                overlap_x_axis=self.overlap_x,
                overlap_y_axis=self.overlap_y,
                iou_threshold=self.iou_threshold,
                border_threshold=self.border_threshold,
            )

            user_callback_pipeline = USER_CALLBACK_PIPELINE()

            # MJPEG branch (no overlay — React draws bboxes client-side)
            mjpeg_branch = (
                f"videoconvert n-threads=2 ! "
                f"videorate max-rate={self._ui_fps} ! "
                f"video/x-raw,framerate={self._ui_fps}/1 ! "
                f"jpegenc quality=70 ! "
                f"appsink name=mjpeg_sink sync=false drop=true emit-signals=true"
            )

            # Shared overlay, then split into display + record
            # Inline the overlay element (no OVERLAY_PIPELINE helper) to avoid double-queue
            overlay_element = "" if self._no_overlay else "hailooverlay_community name=hailo_overlay hud-overlay=true ! "
            overlay_branch = (
                f"{overlay_element}"
                f"tee name=overlay_tee "
                f"overlay_tee. ! {QUEUE(name='display_q')} ! "
                f"videoconvert n-threads=2 ! "
                f"fpsdisplaysink name=hailo_display video-sink={self.video_sink} "
                f"sync={self.sync} text-overlay={self.show_fps} signal-fps-measurements=true "
                f"overlay_tee. ! {QUEUE(name='record_q', max_size_buffers=1)} ! "
                f"valve name=record_valve drop=true ! "
                f"videoconvert n-threads=2 ! video/x-raw,format=RGB ! "
                f"appsink name=record_appsink emit-signals=true drop=true sync=false async=false max-buffers=1"
            )

            output_pipeline = (
                f"tee name=ui_tee "
                f"ui_tee. ! {QUEUE(name='overlay_q', leaky='downstream')} ! {overlay_branch} "
                f"ui_tee. ! {QUEUE(name='mjpeg_q', leaky='downstream')} ! {mjpeg_branch}"
            )

            pipeline_parts = [source_pipeline, tile_cropper_pipeline]
            pipeline_parts.extend([user_callback_pipeline, output_pipeline])

            return ' ! '.join(pipeline_parts)

    tracker = ByteTracker(
        track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30,
    )
    LOGGER.info("[tracking] ByteTracker running synchronously in callback")

    user_data = DroneFollowUserData(
        shared_state, target_state, ui_state=ui_state, byte_tracker=tracker,
        velocity_state=velocity_state, controller_config=controller_config,
    )
    app = DroneFollowTilingApp(
        app_callback, user_data, parser=parser, eos_reached=eos_reached,
        ui_enabled=(ui_state is not None), ui_state=ui_state, ui_fps=ui_fps,
        record_dir=record_dir, no_overlay=no_overlay,
    )
    return app
