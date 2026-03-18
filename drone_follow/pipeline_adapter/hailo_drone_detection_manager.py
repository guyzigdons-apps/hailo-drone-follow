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

import cv2
import hailo
import numpy as np

from drone_follow.follow_api.types import Detection, TrackingMode

from .sot_tracker import SOTracker
from .tracker import MetricsTracker
from .tracker_factory import create_tracker

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
# Frame extraction for SOT tracker
# ---------------------------------------------------------------------------

_cached_caps = None  # (format, width, height) — stable after negotiation


def _get_frame_from_buffer(element, buffer):
    """Extract a BGR numpy array from the GStreamer buffer.

    Uses cached caps after the first successful extraction.
    Returns None if caps aren't negotiated yet or format is unsupported.
    """
    global _cached_caps
    from hailo_apps.python.core.common.buffer_utils import (
        get_caps_from_pad, get_numpy_from_buffer,
    )

    if _cached_caps is None:
        pad = element.get_static_pad("src")
        fmt, w, h = get_caps_from_pad(pad)
        if fmt is None or w is None or h is None:
            return None
        _cached_caps = (fmt, w, h)

    fmt, w, h = _cached_caps
    frame = get_numpy_from_buffer(buffer, fmt, w, h)

    if fmt == "RGB":
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame


def _hailo_bbox_to_pixel_xywh(person, frame_w, frame_h):
    """Convert a Hailo normalized bbox to pixel (x, y, w, h) tuple."""
    bbox = person.get_bbox()
    x = int(bbox.xmin() * frame_w)
    y = int(bbox.ymin() * frame_h)
    w = int(bbox.width() * frame_w)
    h = int(bbox.height() * frame_h)
    return (x, y, max(w, 1), max(h, 1))


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


def _update_ui(ui_state, persons, person_to_id, following_id,
               tracking_mode="mot", id_remap=None):
    """Push detection metadata to the web UI if enabled.

    Args:
        id_remap: optional dict {internal_id: display_id} to override
                  IDs shown in the UI (e.g. after an ID switch).
    """
    if ui_state is None:
        return
    all_dets = []
    for p in persons:
        tid = person_to_id.get(id(p))
        if id_remap and tid in id_remap:
            tid = id_remap[tid]
        all_dets.append(_build_det_info(p, tid))
    ui_state.update_detections(all_dets, following_id, tracking_mode=tracking_mode)


def _try_reacquire_reid(target_state, persons):
    """Attempt to reacquire a lost target by comparing saved target embeddings
    against ReID embeddings on raw YOLO detections.

    Returns the matched person detection on success, None otherwise.
    """
    target_embs = target_state.get_target_embeddings()
    if not target_embs:
        return None

    best_person = None
    best_sim = -1.0
    threshold = 0.5  # cosine similarity threshold for re-identification

    for person in persons:
        matrices = person.get_objects_typed(hailo.HAILO_MATRIX)
        if not matrices:
            continue
        det_emb = np.array(matrices[0].get_data())
        norm = np.linalg.norm(det_emb)
        if norm < 1e-6:
            continue
        det_emb = det_emb / norm

        # Compare against all saved target embeddings, take best similarity
        for saved in target_embs:
            sim = float(np.dot(det_emb, saved))
            if sim > best_sim:
                best_sim = sim
                best_person = person

    if best_sim >= threshold and best_person is not None:
        LOGGER.info("[REID] Reacquire match (sim=%.3f)", best_sim)
        return best_person
    return None


def _find_overlapping_detection(sot_result, persons, frame_h, frame_w):
    """Find the YOLO detection that best overlaps NanoTrack's bbox.

    Returns the person detection with highest IoU, or None if below threshold.
    """
    if sot_result is None or not sot_result.ok or sot_result.bbox is None:
        return None

    sx, sy, sw, sh = sot_result.bbox
    best_iou = 0.0
    best_person = None

    for person in persons:
        bbox = person.get_bbox()
        # Convert normalized bbox to pixels
        px = int(bbox.xmin() * frame_w)
        py = int(bbox.ymin() * frame_h)
        pw = int(bbox.width() * frame_w)
        ph = int(bbox.height() * frame_h)

        # Compute IoU
        x1 = max(sx, px)
        y1 = max(sy, py)
        x2 = min(sx + sw, px + pw)
        y2 = min(sy + sh, py + ph)
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        union = sw * sh + pw * ph - inter
        iou = inter / union if union > 0 else 0

        if iou > best_iou:
            best_iou = iou
            best_person = person

    return best_person if best_iou >= 0.3 else None


def _run_tracker(tracker, persons, embeddings=None):
    """Run tracker and return (available_ids, person_by_id, person_to_id).

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

    all_tracks = tracker.update(det_array, embeddings=embeddings)

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
    """Tiling pipeline callback with MOT/SOT mode support.

    MOT mode: ByteTracker tracks all people, UI shows IDs, drone waits
        for user to select a target.
    SOT mode: NanoTrack visually tracks the selected person.  MOT does
        NOT run.  ReID embeddings are collected from the YOLO detection
        that overlaps NanoTrack's bbox.  When NanoTrack loses the target,
        ReID matching is attempted against all YOLO detections in the
        frame.  If a match is found, NanoTrack is re-initialized on it.
        Otherwise, reverts to MOT mode.
    """
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    persons = [d for d in detections if d.get_label() == "person"]

    target_state = user_data.target_state
    ui_state = user_data.ui_state
    sot = user_data.sot_tracker
    mode = target_state.get_mode() if target_state is not None else TrackingMode.MOT

    # =======================================================================
    # MOT MODE
    # =======================================================================
    if mode == TrackingMode.MOT:
        if not persons:
            user_data.tracker.update(_EMPTY_DET_ARRAY)
            user_data.shared_state.update(None, available_ids=set())
            _update_ui(ui_state, [], {}, None, tracking_mode="mot")
            return

        # Extract ReID embeddings for MOT tracker
        embeddings = []
        for person in persons:
            matrices = person.get_objects_typed(hailo.HAILO_MATRIX)
            if matrices:
                embeddings.append(np.array(matrices[0].get_data()))
            else:
                embeddings.append(None)
        if not any(e is not None for e in embeddings):
            embeddings = None

        available_ids, person_by_id, person_to_id = _run_tracker(
            user_data.tracker, persons, embeddings=embeddings)

        # Attach tracking IDs for hailooverlay
        for person in persons:
            track_id = person_to_id.get(id(person))
            if track_id is not None:
                person.add_object(hailo.HailoUniqueID(track_id, hailo.TRACKING_ID))

        # ReID reacquisition: try to re-identify a previously lost target
        if target_state is not None and target_state.has_reacquire_data():
            matched_person = _try_reacquire_reid(target_state, persons)
            if matched_person is not None:
                display_id = target_state.get_display_id()
                target_state.set_target(
                    id(matched_person), display_id=display_id, reacquired=True)
                bbox = matched_person.get_bbox()
                cx = bbox.xmin() + bbox.width() / 2
                cy = bbox.ymin() + bbox.height() / 2
                target_state.update_last_seen(position=(cx, cy))

                # Init NanoTrack on the reacquired person
                frame = _get_frame_from_buffer(element, buffer)
                if frame is not None:
                    h, w = frame.shape[:2]
                    sot.init(frame, _hailo_bbox_to_pixel_xywh(matched_person, w, h))

                user_data.shared_state.update(Detection(
                    label="person",
                    confidence=matched_person.get_confidence(),
                    center_x=cx, center_y=cy,
                    bbox_height=bbox.height(),
                    timestamp=time.monotonic(),
                ), available_ids=available_ids)
                _update_ui(ui_state, persons, person_to_id,
                           display_id, tracking_mode="sot")
                _log_tracker_metrics(user_data.tracker)
                return
            LOGGER.debug("[MOT] reacquire: no ReID match (%d target embeddings)",
                         len(target_state.get_target_embeddings()))
        elif target_state is not None:
            target_state.clear_reacquire()

        user_data.shared_state.update(None, available_ids=available_ids)
        _update_ui(ui_state, persons, person_to_id, None, tracking_mode="mot")
        LOGGER.debug("[MOT] %d tracked ids=%s",
                     len(available_ids), sorted(available_ids))
        _log_tracker_metrics(user_data.tracker)
        return

    # =======================================================================
    # SOT MODE — NanoTrack only, no MOT
    # =======================================================================
    display_id = target_state.get_display_id()
    frame = _get_frame_from_buffer(element, buffer)

    # Initialize NanoTrack on first SOT frame (target just selected from MOT)
    if not sot.is_initialized:
        target_id = target_state.get_target()
        # Run MOT once to resolve which detection is our target_id
        embeddings = []
        for person in persons:
            matrices = person.get_objects_typed(hailo.HAILO_MATRIX)
            if matrices:
                embeddings.append(np.array(matrices[0].get_data()))
            else:
                embeddings.append(None)
        if not any(e is not None for e in embeddings):
            embeddings = None
        _, person_by_id, _ = _run_tracker(
            user_data.tracker, persons, embeddings=embeddings)
        init_person = person_by_id.get(target_id)

        if init_person is None:
            # Target not found in detections — can't init NanoTrack
            LOGGER.info("[SOT→MOT] Can't find ID %d in detections to init NanoTrack",
                        target_id)
            sot.reset()
            target_state.set_target(None)
            user_data.shared_state.update(None, available_ids=set())
            _update_ui(ui_state, persons, {}, None, tracking_mode="mot")
            return

        if frame is not None:
            h, w = frame.shape[:2]
            sot.init(frame, _hailo_bbox_to_pixel_xywh(init_person, w, h))
            # Save the first ReID embedding
            matrices = init_person.get_objects_typed(hailo.HAILO_MATRIX)
            if matrices:
                emb = np.array(matrices[0].get_data())
                norm = np.linalg.norm(emb)
                if norm > 1e-6:
                    target_state.add_target_embedding(emb / norm)

    # --- Run NanoTrack ---
    if frame is not None and sot.is_initialized:
        sot_result = sot.update(frame)
    else:
        sot_result = None

    if sot_result is not None and sot_result.ok:
        # NanoTrack is tracking — use its position for the drone
        target_state.update_last_seen(position=sot_result.center_norm)

        # Find overlapping YOLO detection for ReID + display overlay
        sot_person_to_id = {}
        overlap_person = None
        if persons and frame is not None:
            h, w = frame.shape[:2]
            overlap_person = _find_overlapping_detection(sot_result, persons, h, w)
            if overlap_person is not None:
                # Tag for hailooverlay green box + ID label
                overlap_person.add_object(
                    hailo.HailoUniqueID(display_id, hailo.TRACKING_ID))
                sot_person_to_id[id(overlap_person)] = display_id
                # Collect ReID embedding
                matrices = overlap_person.get_objects_typed(hailo.HAILO_MATRIX)
                if matrices:
                    emb = np.array(matrices[0].get_data())
                    norm = np.linalg.norm(emb)
                    if norm > 1e-6:
                        target_state.add_target_embedding(emb / norm)

        user_data.shared_state.update(Detection(
            label="person",
            confidence=sot_result.confidence,
            center_x=sot_result.center_norm[0],
            center_y=sot_result.center_norm[1],
            bbox_height=sot_result.bbox_height_norm,
            timestamp=time.monotonic(),
        ), available_ids=set())
        _update_ui(ui_state, persons, sot_person_to_id, display_id, tracking_mode="sot")
        LOGGER.debug("[SOT] NanoTrack ok score=%.2f c=(%.2f,%.2f)",
                     sot_result.confidence,
                     sot_result.center_norm[0], sot_result.center_norm[1])
    else:
        # NanoTrack lost the target — try ReID against current detections
        if persons:
            matched_person = _try_reacquire_reid(target_state, persons)
            if matched_person is not None and frame is not None:
                # Found via ReID — re-init NanoTrack, stay in SOT
                bbox = matched_person.get_bbox()
                cx = bbox.xmin() + bbox.width() / 2
                cy = bbox.ymin() + bbox.height() / 2
                target_state.update_last_seen(position=(cx, cy))
                h, w = frame.shape[:2]
                sot.init(frame, _hailo_bbox_to_pixel_xywh(matched_person, w, h))
                # Tag recovered person for overlay
                matched_person.add_object(
                    hailo.HailoUniqueID(display_id, hailo.TRACKING_ID))
                user_data.shared_state.update(Detection(
                    label="person",
                    confidence=matched_person.get_confidence(),
                    center_x=cx, center_y=cy,
                    bbox_height=bbox.height(),
                    timestamp=time.monotonic(),
                ), available_ids=set())
                _update_ui(ui_state, persons, {id(matched_person): display_id},
                           display_id, tracking_mode="sot")
                LOGGER.info("[SOT] NanoTrack lost → ReID recovered (sim match)")
                return

        # ReID also failed — revert to MOT
        n_embs = len(target_state.get_target_embeddings())
        LOGGER.info("[SOT→MOT] NanoTrack + ReID failed. %d embeddings saved.", n_embs)
        sot.reset()
        target_state.set_target(None)
        user_data.shared_state.update(None, available_ids=set())
        _update_ui(ui_state, persons, {}, None, tracking_mode="mot")


def _log_tracker_metrics(tracker):
    """Periodic tracker metrics log (~every 10 s at 30 fps)."""
    metrics = getattr(tracker, "metrics", None)
    if metrics is not None and metrics.total_frames % 300 == 0:
        LOGGER.info("[tracker] fps=%.1f  update=%.1fms  match=%.0f%%  tracks=%d  id_sw=%d",
                    metrics.fps, metrics.update_ms, metrics.match_ratio * 100,
                    metrics.active_tracks, metrics.id_switches)


# ---------------------------------------------------------------------------
# Pipeline app factory
# ---------------------------------------------------------------------------


def create_app(shared_state, target_state=None, eos_reached=None, ui_state=None, ui_fps=10,
               parser: Optional[argparse.ArgumentParser] = None,
               tracker_name: Optional[str] = None,
               record_enabled=False, record_dir=None):
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
    from hailo_apps.python.core.common.core import get_pipeline_parser, get_resource_path
    from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
        QUEUE, DISPLAY_PIPELINE, OVERLAY_PIPELINE,
        INFERENCE_PIPELINE, USER_CALLBACK_PIPELINE,
        TILE_CROPPER_PIPELINE, SOURCE_PIPELINE,
        CROPPER_PIPELINE,
    )
    from hailo_apps.python.core.common.defines import (
        RESOURCES_SO_DIR_NAME,
        RESOURCES_MODELS_DIR_NAME,
        REID_POSTPROCESS_SO_FILENAME,
        ALL_DETECTIONS_CROPPER_POSTPROCESS_SO_FILENAME,
        REID_CROPPER_POSTPROCESS_FUNCTION,
        REID_POSTPROCESS_FUNCTION,
    )

    if parser is None:
        parser = get_pipeline_parser()

    class DroneFollowUserData(app_callback_class):
        def __init__(self, shared_state, target_state=None, ui_state=None,
                     tracker=None, sot_tracker=None):
            super().__init__()
            self.shared_state = shared_state
            self.target_state = target_state
            self.ui_state = ui_state
            self.tracker = tracker
            self.sot_tracker = sot_tracker

    class DroneFollowTilingApp(GStreamerTilingApp):
        """Tiling app with EOS handling and optional MJPEG appsink for web UI."""
        def __init__(self, app_callback, user_data, parser=None, eos_reached=None,
                     ui_enabled=False, ui_state=None, ui_fps=30,
                     record_enabled=False, record_dir=None):
            self._eos_reached = eos_reached
            self._ui_enabled = ui_enabled
            self._record_enabled = record_enabled
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
            self._Gst = _get_gst()
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
            return os.path.join(self._record_dir, f"rec_{ts}.mp4")

        def start_recording(self, path=None):
            """Start GStreamer-native recording. Returns the output file path."""
            Gst = _get_gst()

            with self._record_lock:
                if self._recording:
                    LOGGER.warning("[record] Already recording")
                    return None
                if not self._record_enabled:
                    LOGGER.error("[record] Recording requires --record flag")
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
            """Stop recording and finalize the file."""
            Gst = _get_gst()

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

        def cleanup_recording_branch(self):
            """Force recording branch elements to NULL so they don't block pipeline shutdown."""
            if not self._record_enabled:
                return
            Gst = _get_gst()
            with self._record_lock:
                for name in ("record_enc", "record_mux", "record_sink"):
                    el = self.pipeline.get_by_name(name)
                    if el is not None:
                        el.set_state(Gst.State.NULL)

        def _build_reid_cropper_pipeline(self):
            """Build the RepVGG body-ReID cropper+inference sub-pipeline.

            Returns the pipeline string or None if ReID is disabled or the HEF is not found.
            """
            if not getattr(self.options_menu, 'reid', True):
                LOGGER.info("[reid] ReID disabled via --no-reid")
                return None

            reid_hef = get_resource_path(
                pipeline_name=None,
                resource_type=RESOURCES_MODELS_DIR_NAME,
                arch=self.arch,
                model='repvgg_a0_person_reid_512',
            )
            if reid_hef is None or not reid_hef.exists():
                LOGGER.warning("[reid] RepVGG HEF not found (%s) — running without ReID", reid_hef)
                return None

            reid_so = get_resource_path(
                pipeline_name=None,
                resource_type=RESOURCES_SO_DIR_NAME,
                model=REID_POSTPROCESS_SO_FILENAME,
            )
            cropper_so = get_resource_path(
                pipeline_name=None,
                resource_type=RESOURCES_SO_DIR_NAME,
                model=ALL_DETECTIONS_CROPPER_POSTPROCESS_SO_FILENAME,
            )

            reid_inference = INFERENCE_PIPELINE(
                hef_path=str(reid_hef),
                post_process_so=str(reid_so),
                post_function_name=REID_POSTPROCESS_FUNCTION,
                batch_size=self.batch_size,
                config_json=None,
                name='reid_inference',
            )

            reid_cropper = CROPPER_PIPELINE(
                inner_pipeline=reid_inference,
                so_path=str(cropper_so),
                function_name=REID_CROPPER_POSTPROCESS_FUNCTION,
                internal_offset=True,
                name='reid_cropper',
            )

            LOGGER.info("[reid] RepVGG body-ReID pipeline enabled (hef=%s)", reid_hef)
            return reid_cropper

        def get_pipeline_string(self):
            reid_cropper_pipeline = self._build_reid_cropper_pipeline()

            if not self._ui_enabled and not self._record_enabled:
                if reid_cropper_pipeline is None:
                    return super().get_pipeline_string()

                # Non-UI/non-record path with ReID: build minimal pipeline
                source_pipeline = SOURCE_PIPELINE(
                    video_source=self.video_source,
                    video_width=self.video_width,
                    video_height=self.video_height,
                    frame_rate=self.frame_rate,
                    sync=self.sync,
                    horizontal_mirror=self.horizontal_mirror,
                    vertical_mirror=self.vertical_mirror,
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
                display_pipeline = DISPLAY_PIPELINE(
                    video_sink=self.video_sink, sync=self.sync, show_fps=self.show_fps,
                )

                return (
                    f'{source_pipeline} ! '
                    f'{tile_cropper_pipeline} ! '
                    f'{reid_cropper_pipeline} ! '
                    f'{user_callback_pipeline} ! '
                    f'{display_pipeline}'
                )

            # Build custom pipeline with optional tee branches
            source_pipeline = SOURCE_PIPELINE(
                video_source=self.video_source,
                video_width=self.video_width,
                video_height=self.video_height,
                frame_rate=self.frame_rate,
                sync=self.sync,
                horizontal_mirror=self.horizontal_mirror,
                vertical_mirror=self.vertical_mirror,
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

            display_branch = DISPLAY_PIPELINE(
                video_sink=self.video_sink, sync=self.sync, show_fps=self.show_fps,
            )

            # Build extra branches beyond display
            extra_branches = []

            if self._ui_enabled:
                mjpeg_branch = (
                    f"videoconvert n-threads=2 ! "
                    f"videorate max-rate={self._ui_fps} ! "
                    f"video/x-raw,framerate={self._ui_fps}/1 ! "
                    f"jpegenc quality=70 ! "
                    f"appsink name=mjpeg_sink sync=false drop=true emit-signals=true"
                )
                extra_branches.append(
                    f"t. ! {QUEUE(name='mjpeg_branch_q', leaky='downstream')} ! {mjpeg_branch}"
                )

            if self._record_enabled:
                record_branch = (
                    f"valve name=record_valve drop=true ! "
                    f"{OVERLAY_PIPELINE(name='record_overlay')} ! "
                    f"videoconvert n-threads=2 ! "
                    f"x264enc name=record_enc tune=zerolatency bitrate=5000 speed-preset=ultrafast ! "
                    f"mp4mux name=record_mux faststart=true ! filesink name=record_sink async=false location=/dev/null"
                )
                extra_branches.append(
                    f"t. ! {QUEUE(name='record_branch_q', max_size_buffers=1, leaky='downstream')} ! {record_branch}"
                )

            # Use tee when there are extra branches beyond display
            output_pipeline = (
                f"tee name=t "
                f"t. ! {QUEUE(name='display_branch_q', leaky='downstream')} ! {display_branch} "
                + " ".join(extra_branches)
            )

            pipeline_parts = [source_pipeline, tile_cropper_pipeline]
            if reid_cropper_pipeline is not None:
                pipeline_parts.append(reid_cropper_pipeline)
            pipeline_parts.extend([user_callback_pipeline, output_pipeline])

            return ' ! '.join(pipeline_parts)

    args = parser.parse_known_args()[0] if parser is not None else None
    if tracker_name is None:
        tracker_name = getattr(args, "tracker", "byte") if args is not None else "byte"

    t0 = time.monotonic()
    inner_tracker = create_tracker(
        tracker_name, track_thresh=0.4, track_buffer=90,
        match_thresh=0.5, frame_rate=30,
    )
    init_ms = (time.monotonic() - t0) * 1000.0
    tracker = MetricsTracker(inner_tracker, init_time_ms=init_ms)
    LOGGER.info("[tracking] %s ready (init %.1f ms), running synchronously in callback",
                tracker_name, init_ms)

    shared_state.tracker_metrics = tracker.metrics

    # SOT tracker (NanoTrack) — initialized lazily when a target is selected
    sot = SOTracker()
    LOGGER.info("[tracking] NanoTrack SOT ready (lazy init on target selection)")

    user_data = DroneFollowUserData(
        shared_state, target_state, ui_state=ui_state, tracker=tracker,
        sot_tracker=sot,
    )
    app = DroneFollowTilingApp(
        app_callback, user_data, parser=parser, eos_reached=eos_reached,
        ui_enabled=(ui_state is not None), ui_state=ui_state, ui_fps=ui_fps,
        record_enabled=record_enabled, record_dir=record_dir,
    )
    return app
