"""Hailo tiling pipeline adapter — all Hailo/GStreamer imports are confined here.

Translates Hailo detection objects into the pure Detection domain type.
No other module needs to import hailo or gi.repository.
"""

import argparse
import logging
import math
import os
import threading
import time
from datetime import datetime
from typing import Optional

import cv2
import hailo
import numpy as np

from drone_follow.follow_api.types import Detection, TrackingMode

from .sot_tracker import SOTracker, SOTResult
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

    # NanoTrack expects BGR; GStreamer RGB needs conversion
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


def _find_closest_mot_detection(sot_result, person_by_id, iou_threshold=0.3):
    """Find the MOT track whose bbox best overlaps the SOT result.

    Returns (track_id, person) or (None, None) if no match above threshold.
    """
    if sot_result.bbox is None or not sot_result.ok:
        return None, None

    sx, sy, sw, sh = sot_result.bbox
    best_iou = 0.0
    best_tid = None
    best_person = None

    # Need frame dimensions to convert normalized MOT bboxes to pixels.
    # We use the SOT result's normalized center to get approximate frame size.
    # Actually, we'll compare in normalized space using sot_result.center_norm
    # and bbox_height_norm.
    if sot_result.center_norm is None:
        return None, None

    scx, scy = sot_result.center_norm
    s_half_h = sot_result.bbox_height_norm / 2
    # Approximate SOT bbox width ratio (assume aspect from pixel bbox)
    s_half_w = (sw / max(sh, 1)) * s_half_h / 2 if sh > 0 else s_half_h

    for tid, person in person_by_id.items():
        bbox = person.get_bbox()
        mcx = bbox.xmin() + bbox.width() / 2
        mcy = bbox.ymin() + bbox.height() / 2
        m_half_w = bbox.width() / 2
        m_half_h = bbox.height() / 2

        # IoU in normalized space
        x1 = max(scx - s_half_w, mcx - m_half_w)
        y1 = max(scy - s_half_h, mcy - m_half_h)
        x2 = min(scx + s_half_w, mcx + m_half_w)
        y2 = min(scy + s_half_h, mcy + m_half_h)

        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area_s = (2 * s_half_w) * (2 * s_half_h)
        area_m = bbox.width() * bbox.height()
        union = area_s + area_m - inter

        iou = inter / union if union > 0 else 0
        if iou > best_iou:
            best_iou = iou
            best_tid = tid
            best_person = person

    if best_iou >= iou_threshold:
        return best_tid, best_person
    return None, None


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


def _update_ui(ui_state, persons, person_to_id, following_id, tracking_mode="mot"):
    """Push detection metadata to the web UI if enabled."""
    if ui_state is None:
        return
    all_dets = [_build_det_info(p, person_to_id.get(id(p))) for p in persons]
    ui_state.update_detections(all_dets, following_id, tracking_mode=tracking_mode)


def _try_reacquire(target_state, person_by_id):
    """Attempt to reacquire a lost target using saved ReID embeddings.

    Compares stored embeddings against current detections.  Returns the
    matched track_id on success, None otherwise.

    Currently a no-op — embeddings are None placeholders.  When a ReID
    feature extractor is added, this function will perform actual
    appearance-based matching.
    """
    embeddings = target_state.get_reid_embeddings()
    if not embeddings:
        return None

    # Only attempt matching if we have real embeddings (not None placeholders)
    has_real = any(e.get("embedding") is not None for e in embeddings)
    if not has_real:
        return None

    # TODO: Compare saved embeddings against current detections' embeddings.
    # For each person in person_by_id, extract an embedding and compute
    # cosine similarity against our saved embeddings.  Return the track_id
    # of the best match above a threshold.
    return None


def _collect_nearby_embeddings(persons, person_by_id, last_pos):
    """Build a list of nearby-detection dicts sorted by distance to *last_pos*.

    Each entry: {"track_id": int, "embedding": None, "distance": float}.
    ``embedding`` is a placeholder — a future ReID feature extractor will
    populate it with an actual appearance vector.

    Args:
        persons: list of Hailo detection objects in the current frame.
        person_by_id: {track_id -> detection} mapping from the tracker.
        last_pos: (cx, cy) normalized 0-1 — last known target position.

    Returns:
        List sorted by ascending Euclidean distance to *last_pos*.
    """
    if last_pos is None:
        return []
    lx, ly = last_pos
    nearby = []
    for tid, person in person_by_id.items():
        bbox = person.get_bbox()
        cx = bbox.xmin() + bbox.width() / 2
        cy = bbox.ymin() + bbox.height() / 2
        dist = math.hypot(cx - lx, cy - ly)
        nearby.append({"track_id": tid, "embedding": None, "distance": dist})
    nearby.sort(key=lambda e: e["distance"])
    return nearby


def _run_tracker(tracker, persons):
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

    all_tracks = tracker.update(det_array)

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

    MOT mode (default): All people are tracked and shown in the UI.
        The drone does NOT auto-follow anyone — it waits for the user
        to select a target.
    SOT mode: The drone follows a specific person using a hybrid approach:
        1. MOT (ByteTrack) provides track ID continuity.
        2. NanoTrack SOT provides visual tracking as fallback when MOT
           loses the track ID (e.g., ID switch, brief occlusion).
        3. If both lose the target, nearby embeddings are saved for
           future ReID and the pipeline reverts to MOT mode.

    ByteTracker + NanoTrack both run synchronously in the callback.
    """
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    persons = [d for d in detections if d.get_label() == "person"]

    target_state = user_data.target_state
    ui_state = user_data.ui_state
    sot = user_data.sot_tracker
    mode = target_state.get_mode() if target_state is not None else TrackingMode.MOT

    # --- No persons in frame ---
    if not persons:
        user_data.tracker.update(_EMPTY_DET_ARRAY)

        # In SOT mode, try NanoTrack even with no detections (visual tracking)
        if mode == TrackingMode.SOT and sot.is_initialized:
            frame = _get_frame_from_buffer(element, buffer)
            if frame is not None:
                sot_result = sot.update(frame)
                if sot_result.ok:
                    # NanoTrack still sees the target even though YOLO doesn't
                    target_state.update_last_seen(position=sot_result.center_norm)
                    user_data.shared_state.update(Detection(
                        label="person",
                        confidence=sot_result.confidence,
                        center_x=sot_result.center_norm[0],
                        center_y=sot_result.center_norm[1],
                        bbox_height=sot_result.bbox_height_norm,
                        timestamp=time.monotonic(),
                    ), available_ids=set())
                    _update_ui(ui_state, [], {}, target_state.get_target(),
                               tracking_mode="sot")
                    LOGGER.debug("[SOT/NanoTrack] No YOLO dets but NanoTrack ok "
                                 "score=%.3f", sot_result.confidence)
                    return

        # Truly lost
        user_data.shared_state.update(None, available_ids=set())
        if target_state is not None and mode == TrackingMode.SOT:
            sot.reset()
            target_state.set_target(None)
            LOGGER.debug("[SOT→MOT] No person detected — reverting to MOT")
        _update_ui(ui_state, [], {}, None, tracking_mode="mot")
        return

    # --- Run MOT tracker (always, both modes) ---
    available_ids, person_by_id, person_to_id = _run_tracker(
        user_data.tracker, persons)

    # === MOT MODE: show all, follow none ===
    if mode == TrackingMode.MOT:
        sot.reset()  # ensure SOT is clean

        # --- Reacquisition: try to re-identify the lost target ---
        if target_state is not None and target_state.has_reacquire_data():
            matched_id = _try_reacquire(target_state, person_by_id)
            if matched_id is not None and matched_id in person_by_id:
                LOGGER.info("[MOT→SOT] Reacquired target as ID %d via ReID",
                            matched_id)
                target_state.set_target(matched_id)
                best = person_by_id[matched_id]
                bbox = best.get_bbox()
                cx = bbox.xmin() + bbox.width() / 2
                cy = bbox.ymin() + bbox.height() / 2
                target_state.update_last_seen(position=(cx, cy))

                # Init NanoTrack on the reacquired target
                frame = _get_frame_from_buffer(element, buffer)
                if frame is not None:
                    h, w = frame.shape[:2]
                    sot.init(frame, _hailo_bbox_to_pixel_xywh(best, w, h))

                user_data.shared_state.update(Detection(
                    label="person",
                    confidence=best.get_confidence(),
                    center_x=cx, center_y=cy,
                    bbox_height=bbox.height(),
                    timestamp=time.monotonic(),
                ), available_ids=available_ids)
                _update_ui(ui_state, persons, person_to_id, matched_id,
                           tracking_mode="sot")
                _log_tracker_metrics(user_data.tracker)
                return
            # No match yet — stay in MOT, will retry next frame
            LOGGER.debug("[MOT/reacquire] No ReID match, retrying... (%d embeddings saved)",
                         len(target_state.get_reid_embeddings()))
        elif target_state is not None:
            # Timeout expired or no embeddings — stop trying
            target_state.clear_reacquire()

        user_data.shared_state.update(None, available_ids=available_ids)
        _update_ui(ui_state, persons, person_to_id, None, tracking_mode="mot")
        LOGGER.debug("[MOT] %d persons tracked: %s",
                     len(available_ids), sorted(available_ids))
        _log_tracker_metrics(user_data.tracker)
        return

    # === SOT MODE: follow the selected target ===
    target_id = target_state.get_target()
    best = person_by_id.get(target_id)

    # Extract frame for NanoTrack (needed for init or update)
    frame = _get_frame_from_buffer(element, buffer)

    if best is not None:
        # --- MOT found the target ---
        bbox = best.get_bbox()
        cx = bbox.xmin() + bbox.width() / 2
        cy = bbox.ymin() + bbox.height() / 2
        target_state.update_last_seen(position=(cx, cy))

        # (Re-)init NanoTrack with MOT's bbox for drift correction
        if frame is not None:
            h, w = frame.shape[:2]
            pixel_bbox = _hailo_bbox_to_pixel_xywh(best, w, h)
            if not sot.is_initialized:
                sot.init(frame, pixel_bbox)
            else:
                # Re-init periodically to prevent drift (every frame MOT
                # confirms the target, we reset NanoTrack's template)
                sot.init(frame, pixel_bbox)

        user_data.shared_state.update(Detection(
            label="person",
            confidence=best.get_confidence(),
            center_x=cx,
            center_y=cy,
            bbox_height=bbox.height(),
            timestamp=time.monotonic(),
        ), available_ids=available_ids)

        _update_ui(ui_state, persons, person_to_id, target_id, tracking_mode="sot")
        LOGGER.debug("[SOT/MOT] Following ID %d  conf=%.2f center=(%.2f,%.2f) h=%.2f",
                     target_id, best.get_confidence(), cx, cy, bbox.height())

    elif sot.is_initialized and frame is not None:
        # --- MOT lost the track ID, try NanoTrack fallback ---
        sot_result = sot.update(frame)

        if sot_result.ok:
            # NanoTrack still tracks visually — try to adopt a new MOT ID
            fallback_tid, fallback_person = _find_closest_mot_detection(
                sot_result, person_by_id)

            if fallback_tid is not None:
                # Found a matching MOT detection — adopt its ID
                LOGGER.info("[SOT] MOT ID %d lost, NanoTrack matched MOT ID %d "
                            "(ID switch recovery)", target_id, fallback_tid)
                target_state.set_target(fallback_tid)
                fbbox = fallback_person.get_bbox()
                cx = fbbox.xmin() + fbbox.width() / 2
                cy = fbbox.ymin() + fbbox.height() / 2
                target_state.update_last_seen(position=(cx, cy))

                # Re-init NanoTrack on the corrected bbox
                h, w = frame.shape[:2]
                sot.init(frame, _hailo_bbox_to_pixel_xywh(fallback_person, w, h))

                user_data.shared_state.update(Detection(
                    label="person",
                    confidence=fallback_person.get_confidence(),
                    center_x=cx,
                    center_y=cy,
                    bbox_height=fbbox.height(),
                    timestamp=time.monotonic(),
                ), available_ids=available_ids)
                _update_ui(ui_state, persons, person_to_id, fallback_tid,
                           tracking_mode="sot")
            else:
                # NanoTrack tracks but no MOT match — use NanoTrack position
                target_state.update_last_seen(position=sot_result.center_norm)
                user_data.shared_state.update(Detection(
                    label="person",
                    confidence=sot_result.confidence,
                    center_x=sot_result.center_norm[0],
                    center_y=sot_result.center_norm[1],
                    bbox_height=sot_result.bbox_height_norm,
                    timestamp=time.monotonic(),
                ), available_ids=available_ids)
                _update_ui(ui_state, persons, person_to_id, target_id,
                           tracking_mode="sot")
                LOGGER.debug("[SOT/NanoTrack] MOT ID %d lost, NanoTrack fallback "
                             "score=%.3f", target_id, sot_result.confidence)
        else:
            # Both MOT and NanoTrack lost the target — revert to MOT
            _handle_target_lost(target_state, sot, persons, person_by_id,
                                available_ids, user_data, ui_state, person_to_id,
                                target_id)
    else:
        # MOT lost and NanoTrack not available — revert to MOT
        _handle_target_lost(target_state, sot, persons, person_by_id,
                            available_ids, user_data, ui_state, person_to_id,
                            target_id)

    _log_tracker_metrics(user_data.tracker)


def _handle_target_lost(target_state, sot, persons, person_by_id,
                        available_ids, user_data, ui_state, person_to_id,
                        target_id):
    """Handle target loss: save ReID embeddings, reset SOT, revert to MOT."""
    last_pos = target_state.get_last_known_position()
    nearby = _collect_nearby_embeddings(persons, person_by_id, last_pos)
    target_state.store_reid_embeddings(nearby)

    LOGGER.info("[SOT→MOT] Target ID %d lost (MOT+NanoTrack). "
                "Saved %d nearby embeddings. Available: %s — reverting to MOT",
                target_id, len(nearby),
                sorted(available_ids) if available_ids else "none")

    sot.reset()
    target_state.set_target(None)  # switches to MOT
    user_data.shared_state.update(None, available_ids=available_ids)
    _update_ui(ui_state, persons, person_to_id, None, tracking_mode="mot")


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
               parser: Optional[argparse.ArgumentParser] = None, record_dir=None,
               tracker_name: Optional[str] = None):
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
        INFERENCE_PIPELINE, USER_CALLBACK_PIPELINE,
        TILE_CROPPER_PIPELINE, SOURCE_PIPELINE,
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
            return os.path.join(self._record_dir, f"rec_{ts}.mkv")

        def start_recording(self, path=None):
            """Start GStreamer-native recording. Returns the output file path."""
            Gst = _get_gst()

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

            display_branch = DISPLAY_PIPELINE(
                video_sink=self.video_sink, sync=self.sync, show_fps=self.show_fps,
            )

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

            # Tee splits into display + MJPEG + recording
            # All branches use leaky queues so a slow branch never stalls the others
            output_pipeline = (
                f"tee name=ui_tee "
                f"ui_tee. ! {QUEUE(name='display_branch_q', leaky='downstream')} ! {display_branch} "
                f"ui_tee. ! {QUEUE(name='mjpeg_branch_q', leaky='downstream')} ! {mjpeg_branch} "
                f"ui_tee. ! {QUEUE(name='record_branch_q', max_size_buffers=1, leaky='downstream')} ! {record_branch}"
            )

            pipeline_parts = [source_pipeline, tile_cropper_pipeline,
                              user_callback_pipeline, output_pipeline]

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
        record_dir=record_dir,
    )
    return app
