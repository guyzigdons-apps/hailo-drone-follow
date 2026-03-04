"""Hailo tiling pipeline adapter — all Hailo/GStreamer imports are confined here.

Translates Hailo detection objects into the pure Detection domain type.
No other module needs to import hailo or gi.repository.
"""

import argparse
import logging
import os
import threading
import time
from datetime import datetime
from typing import Optional

import hailo
import numpy as np

from drone_follow.follow_api.types import Detection

from .byte_tracker import ByteTracker

LOGGER = logging.getLogger("drone_follow.app")

_EMPTY_DET_ARRAY = np.empty((0, 5), dtype=np.float32)


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

    if not persons:
        user_data.byte_tracker.update(_EMPTY_DET_ARRAY)
        user_data.shared_state.update(None, available_ids=set())
        if target_state is not None and target_state.get_target() is not None:
            target_state.set_target(None)
        _update_ui(ui_state, [], {}, target_state.get_target() if target_state else None)
        if target_state is None or target_state.get_target() is None:
            LOGGER.debug("[SEARCH MODE] No person detected in frame - follow state cleared")
        return

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
            return
    else:
        # IDLE mode: hold position, do not select any target
        if target_state is not None and target_state.is_paused():
            user_data.shared_state.update(None, available_ids=available_ids)
            _update_ui(ui_state, persons, person_to_id, None)
            return
        best = max(persons, key=lambda d: d.get_bbox().width() * d.get_bbox().height())
        best_tid = person_to_id.get(id(best))
        if best_tid is not None and target_state is not None:
            target_state.set_target(best_tid)
            follow_mode = f"locked ID {best_tid}"
        elif best_tid is not None:
            follow_mode = f"largest (ID {best_tid})"
        else:
            follow_mode = "largest (no tracking)"

    bbox = best.get_bbox()
    cx = bbox.xmin() + bbox.width() / 2
    cy = bbox.ymin() + bbox.height() / 2
    user_data.shared_state.update(Detection(
        label="person",
        confidence=best.get_confidence(),
        center_x=cx,
        center_y=cy,
        bbox_height=bbox.height(),
        timestamp=time.monotonic(),
    ), available_ids=available_ids)

    _update_ui(ui_state, persons, person_to_id,
               target_state.get_target() if target_state else None)

    available_str = f"Available: {sorted(available_ids)}" if available_ids else ""
    LOGGER.debug("[FOLLOWING %s] conf=%.2f center=(%.2f,%.2f) h=%.2f %s",
                follow_mode, best.get_confidence(), cx, cy, bbox.height(), available_str)


# ---------------------------------------------------------------------------
# Pipeline app factory
# ---------------------------------------------------------------------------


def create_app(shared_state, target_state=None, eos_reached=None, ui_state=None, ui_fps=10,
               parser: Optional[argparse.ArgumentParser] = None, record_dir=None):
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
        QUEUE, DISPLAY_PIPELINE, OVERLAY_PIPELINE,
    )

    if parser is None:
        parser = get_pipeline_parser()

    class DroneFollowUserData(app_callback_class):
        def __init__(self, shared_state, target_state=None, ui_state=None,
                     byte_tracker=None):
            super().__init__()
            self.shared_state = shared_state
            self.target_state = target_state
            self.ui_state = ui_state
            self.byte_tracker = byte_tracker

    class DroneFollowTilingApp(GStreamerTilingApp):
        """Tiling app with EOS handling and optional MJPEG appsink for web UI."""
        def __init__(self, app_callback, user_data, parser=None, eos_reached=None,
                     ui_enabled=False, ui_state=None, ui_fps=30, record_dir=None):
            self._eos_reached = eos_reached
            self._ui_enabled = ui_enabled
            self._ui_state = ui_state
            self._ui_fps = ui_fps
            self._recording = False
            self._record_dir = record_dir or os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "..", "recordings")
            self._record_lock = threading.Lock()
            super().__init__(app_callback, user_data, parser=parser)
            # Connect appsink after pipeline is created by super().__init__
            if self._ui_enabled:
                self._connect_mjpeg_sink()

        def _connect_mjpeg_sink(self):
            """Connect the MJPEG appsink's new-sample signal."""
            import gi
            gi.require_version("Gst", "1.0")
            from gi.repository import Gst
            self._Gst = Gst
            mjpeg_sink = self.pipeline.get_by_name("mjpeg_sink")
            if mjpeg_sink:
                mjpeg_sink.connect("new-sample", self._on_mjpeg_sample)

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
            """Start GStreamer-native recording. Returns the output file path."""
            import gi
            gi.require_version("Gst", "1.0")
            from gi.repository import Gst

            with self._record_lock:
                if self._recording:
                    LOGGER.warning("[record] Already recording")
                    return None
                if not self._ui_enabled:
                    LOGGER.error("[record] Recording requires UI pipeline (--ui)")
                    return None

                valve = self.pipeline.get_by_name("record_valve")
                encoder = self.pipeline.get_by_name("record_enc")
                muxer = self.pipeline.get_by_name("record_mux")
                filesink = self.pipeline.get_by_name("record_sink")
                if valve is None or filesink is None:
                    LOGGER.error("[record] Recording elements not found in pipeline")
                    return None

                record_path = path or self._generate_record_path()

                # Cycle encoder, muxer, and filesink through NULL to clear
                # any leftover EOS state from a previous recording session
                for el in (filesink, muxer, encoder):
                    if el is not None:
                        el.set_state(Gst.State.NULL)
                filesink.set_property("location", record_path)
                for el in (encoder, muxer, filesink):
                    if el is not None:
                        el.sync_state_with_parent()

                valve.set_property("drop", False)
                self._recording = True
                self._current_record_path = record_path
                LOGGER.info("[record] Started recording to %s", record_path)
                return record_path

        def stop_recording(self):
            """Stop recording and finalize the file.

            Closes the valve, then transitions the recording elements
            (encoder → muxer → filesink) to NULL.  matroskamux writes its
            seek index and duration during its PLAYING→NULL transition,
            so no EOS is sent — this keeps the main pipeline untouched.
            """
            import gi
            gi.require_version("Gst", "1.0")
            from gi.repository import Gst

            with self._record_lock:
                if not self._recording:
                    return None

                valve = self.pipeline.get_by_name("record_valve")
                encoder = self.pipeline.get_by_name("record_enc")
                muxer = self.pipeline.get_by_name("record_mux")
                filesink = self.pipeline.get_by_name("record_sink")
                if valve is None:
                    self._recording = False
                    return None

                self._recording = False
                path = getattr(self, "_current_record_path", None)

                # Close the valve first to stop new buffers entering
                valve.set_property("drop", True)

                # Transition encoder → muxer → filesink to NULL.
                # matroskamux finalises the file (writes Cues/SeekHead/duration)
                # during its state change — no EOS needed.
                for el in (encoder, muxer, filesink):
                    if el is not None:
                        el.set_state(Gst.State.NULL)

                LOGGER.info("[record] Stopped recording: %s", path)
                return path

        def get_pipeline_string(self):
            openhd_stream = getattr(self.options_menu, 'openhd_stream', False)

            # If no custom output needed, delegate to parent
            if not self._ui_enabled and not openhd_stream:
                return super().get_pipeline_string()

            # Build pipeline with custom output (OpenHD stream and/or MJPEG UI)
            from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
                SOURCE_PIPELINE, INFERENCE_PIPELINE, USER_CALLBACK_PIPELINE,
                TILE_CROPPER_PIPELINE, OPENHD_STREAM_PIPELINE,
            )

            source_pipeline = SOURCE_PIPELINE(
                video_source=self.video_source,
                video_width=self.video_width,
                video_height=self.video_height,
                frame_rate=self.frame_rate,
                sync=self.sync,
                mirror_image=self.options_menu.horizontal_mirror,
                vertical_mirror=self.options_menu.vertical_mirror,
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

            # Primary output branch: OpenHD stream, display, or fakesink
            if openhd_stream:
                openhd_port = getattr(self.options_menu, 'openhd_port', 5500)
                openhd_bitrate = getattr(self.options_menu, 'openhd_bitrate', 5000)
                primary_branch = OPENHD_STREAM_PIPELINE(
                    port=openhd_port, bitrate=openhd_bitrate,
                )
            else:
                primary_branch = DISPLAY_PIPELINE(
                    video_sink=self.video_sink, sync=self.sync, show_fps=self.show_fps,
                )

            if self._ui_enabled:
                # MJPEG branch (raw video, no overlay — React draws bboxes)
                mjpeg_branch = (
                    f"videoconvert n-threads=2 ! "
                    f"videorate max-rate={self._ui_fps} ! "
                    f"video/x-raw,framerate={self._ui_fps}/1 ! "
                    f"jpegenc quality=70 ! "
                    f"appsink name=mjpeg_sink sync=false drop=true emit-signals=true"
                )

                # Recording branch (valve drops by default; toggled at runtime)
                record_branch = (
                    f"valve name=record_valve drop=true ! "
                    f"{OVERLAY_PIPELINE(name='record_overlay')} ! "
                    f"videoconvert n-threads=2 ! "
                    f"x264enc name=record_enc tune=zerolatency bitrate=5000 speed-preset=ultrafast ! "
                    f"matroskamux name=record_mux ! filesink name=record_sink async=false location=/dev/null"
                )

                # Tee splits into primary + MJPEG + recording
                # All branches use leaky queues so a slow branch never stalls the others
                output_pipeline = (
                    f"tee name=ui_tee "
                    f"ui_tee. ! {QUEUE(name='primary_branch_q', leaky='downstream')} ! {primary_branch} "
                    f"ui_tee. ! {QUEUE(name='mjpeg_branch_q', leaky='downstream')} ! {mjpeg_branch} "
                    f"ui_tee. ! {QUEUE(name='record_branch_q', max_size_buffers=1, leaky='downstream')} ! {record_branch}"
                )
            else:
                output_pipeline = primary_branch

            pipeline_parts = [source_pipeline, tile_cropper_pipeline]
            pipeline_parts.extend([user_callback_pipeline, output_pipeline])

            return ' ! '.join(pipeline_parts)

    tracker = ByteTracker(
        track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30,
    )
    LOGGER.info("[tracking] ByteTracker running synchronously in callback")

    user_data = DroneFollowUserData(
        shared_state, target_state, ui_state=ui_state, byte_tracker=tracker,
    )
    app = DroneFollowTilingApp(
        app_callback, user_data, parser=parser, eos_reached=eos_reached,
        ui_enabled=(ui_state is not None), ui_state=ui_state, ui_fps=ui_fps,
        record_dir=record_dir,
    )
    return app
