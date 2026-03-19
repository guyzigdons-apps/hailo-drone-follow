"""Hailo gesture detection pipeline adapter — tiling (person/face) + palm/hand landmark (C++ NPU).

Extends the tiling pipeline from hailo_drone_detection_manager by appending the C++ gesture
detection stages: palm_detection → palm_croppers → hand_affine_warp → hand_landmark → gesture_classification.

All inference runs on the Hailo NPU. The Python callback only reads metadata attached by the
C++ postprocess shared libraries:
  - HailoDetection("hand") with tight bounding box
  - HailoLandmarks("hand_landmarks") with 21 keypoints
  - HailoClassification("gesture") with gesture label (FIST, OPEN_HAND, etc.)

Populates SharedGestureState (for gesture control) and SharedDetectionState (for distance control).
"""

import logging
import math
import os
import time
from typing import Optional

import hailo
import numpy as np

from drone_follow.follow_api.types import Detection, FaceDetection, HandDetection, GestureDetection, PalmDetection

from .byte_tracker import ByteTracker
from .hailo_drone_detection_manager import (
    _get_gst, _build_det_info, _run_tracker, _update_ui,
    _EMPTY_DET_ARRAY, _attach_velocity_arrows,
)

LOGGER = logging.getLogger("drone_follow.gesture")

# ---------------------------------------------------------------------------
# C++ postprocess shared libraries
# ---------------------------------------------------------------------------

SO_DIR = "/usr/local/hailo/resources/so"
PALM_DETECTION_POST_SO = os.path.join(SO_DIR, "libpalm_detection_postprocess.so")
PALM_CROPPERS_SO = os.path.join(SO_DIR, "libpalm_croppers.so")
HAND_AFFINE_WARP_SO = os.path.join(SO_DIR, "libhand_affine_warp.so")
HAND_LANDMARK_POST_SO = os.path.join(SO_DIR, "libhand_landmark_postprocess.so")
GESTURE_CLASSIFICATION_SO = os.path.join(SO_DIR, "libgesture_classification.so")
PERSON_PALM_CROPPERS_SO = os.path.join(SO_DIR, "libperson_palm_croppers.so")

# Model paths
_MODELS_DIR = os.path.join(
    os.path.dirname(__file__), "..", "..", "hailo-apps",
    "hailo_apps", "python", "pipeline_apps", "gesture_detection", "models")
DEFAULT_PALM_HEF = os.path.join(_MODELS_DIR, "palm_detection_lite.hef")
DEFAULT_HAND_HEF = os.path.join(_MODELS_DIR, "hand_landmark_lite.hef")


# ---------------------------------------------------------------------------
# Metadata extraction helpers
# ---------------------------------------------------------------------------

def _extract_hand_from_roi(roi):
    """Extract hand detection, landmarks, and gesture from Hailo ROI metadata.

    After the C++ pipeline runs, the ROI contains:
      - HailoDetection("hand") with tight bbox and:
        - HailoLandmarks("hand_landmarks") with 21 keypoints (bbox-relative)
        - HailoClassification("gesture") with label

    Returns:
        (HandDetection, gesture_label) or (None, None)
    """
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    hand_dets = [d for d in detections if d.get_label() == "hand"]
    if not hand_dets:
        return None, None

    # Pick highest-confidence hand
    hand_det = max(hand_dets, key=lambda d: d.get_confidence())
    hbbox = hand_det.get_bbox()

    # Get gesture classification
    gesture_label = None
    for obj in hand_det.get_objects():
        if isinstance(obj, hailo.HailoClassification):
            if obj.get_classification_type() == "gesture":
                gesture_label = obj.get_label()

    # Get hand landmarks (21 points, bbox-relative [0,1])
    landmarks = None
    for obj in hand_det.get_objects():
        if isinstance(obj, hailo.HailoLandmarks):
            if obj.get_landmarks_type() == "hand_landmarks":
                landmarks = obj

    if landmarks is None:
        return None, gesture_label

    points = landmarks.get_points()
    if len(points) < 21:
        return None, gesture_label

    # Convert landmark coords from hand-bbox-relative to frame-normalized
    hx0 = hbbox.xmin()
    hy0 = hbbox.ymin()
    hw = hbbox.width()
    hh = hbbox.height()

    def to_frame(pt):
        return (hx0 + pt.x() * hw, hy0 + pt.y() * hh)

    # Wrist = landmark 0
    wrist_x, wrist_y = to_frame(points[0])

    # Palm center = average of landmarks 0, 5, 9, 13, 17
    palm_indices = [0, 5, 9, 13, 17]
    palm_x = sum(to_frame(points[i])[0] for i in palm_indices) / len(palm_indices)
    palm_y = sum(to_frame(points[i])[1] for i in palm_indices) / len(palm_indices)

    # Hand tilt: angle from wrist (0) to middle finger MCP (9)
    mid_x, mid_y = to_frame(points[9])
    tilt_rad = math.atan2(mid_x - wrist_x, -(mid_y - wrist_y))  # 0 = fingers up
    tilt_deg = math.degrees(tilt_rad)

    is_open = gesture_label not in ("FIST", None)

    hand = HandDetection(
        center_x=palm_x,
        center_y=palm_y,
        wrist_x=wrist_x,
        wrist_y=wrist_y,
        is_open=is_open,
        confidence=hand_det.get_confidence(),
        timestamp=time.monotonic(),
    )

    return hand, gesture_label


def _match_face_to_person(faces, person_bbox):
    """Find the face detection whose center falls inside person_bbox."""
    px0 = person_bbox.xmin()
    py0 = person_bbox.ymin()
    px1 = px0 + person_bbox.width()
    py1 = py0 + person_bbox.height()

    best = None
    best_conf = -1.0
    for f in faces:
        fb = f.get_bbox()
        fcx = fb.xmin() + fb.width() / 2
        fcy = fb.ymin() + fb.height() / 2
        if px0 <= fcx <= px1 and py0 <= fcy <= py1:
            if f.get_confidence() > best_conf:
                best = f
                best_conf = f.get_confidence()
    return best


def _extract_palms_from_roi(roi):
    """Extract tracked palm detections from ROI.

    After hailotracker (class_id=100), each palm detection has a HailoUniqueID.
    Only extracts "palm" label detections — the "hand" label is the expanded
    affine warp crop for the hand landmark model and must be excluded.

    Returns list of PalmDetection with stable track IDs from the GStreamer tracker.
    """
    now = time.monotonic()
    palms = []

    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    for det in detections:
        if det.get_label() != "palm":
            continue

        bbox = det.get_bbox()
        cx = bbox.xmin() + bbox.width() / 2
        cy = bbox.ymin() + bbox.height() / 2

        # Extract tracking ID from HailoUniqueID (attached by hailotracker)
        track_id = -1
        ids = det.get_objects_typed(hailo.HAILO_UNIQUE_ID)
        if ids:
            track_id = ids[0].get_id()

        palms.append(PalmDetection(
            track_id=track_id,
            center_x=cx,
            center_y=cy,
            bbox_width=bbox.width(),
            bbox_height=bbox.height(),
            confidence=det.get_confidence(),
            timestamp=now,
        ))

    return palms


def _find_hand_near_palm(roi, palm_cx, palm_cy, max_dist=0.15):
    """Find the hand detection closest to a tracked palm position.

    Used after hand landmarks are enabled to match the locked palm's
    hand detection from the full gesture pipeline output.
    """
    best_hand = None
    best_label = None
    best_dist = max_dist

    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    for det in detections:
        if det.get_label() != "hand":
            continue
        bbox = det.get_bbox()
        cx = bbox.xmin() + bbox.width() / 2
        cy = bbox.ymin() + bbox.height() / 2
        dist = math.sqrt((cx - palm_cx) ** 2 + (cy - palm_cy) ** 2)
        if dist < best_dist:
            best_dist = dist
            best_hand = det
            # Extract hand + gesture from this specific detection
            best_label = None
            for obj in det.get_objects():
                if isinstance(obj, hailo.HailoClassification):
                    if obj.get_classification_type() == "gesture":
                        best_label = obj.get_label()

    if best_hand is None:
        return None, None

    # Extract landmarks from matched hand
    hbbox = best_hand.get_bbox()
    landmarks = None
    for obj in best_hand.get_objects():
        if isinstance(obj, hailo.HailoLandmarks):
            if obj.get_landmarks_type() == "hand_landmarks":
                landmarks = obj

    if landmarks is None:
        return None, best_label

    points = landmarks.get_points()
    if len(points) < 21:
        return None, best_label

    hx0 = hbbox.xmin()
    hy0 = hbbox.ymin()
    hw = hbbox.width()
    hh = hbbox.height()

    def to_frame(pt):
        return (hx0 + pt.x() * hw, hy0 + pt.y() * hh)

    wrist_x, wrist_y = to_frame(points[0])
    palm_indices = [0, 5, 9, 13, 17]
    px = sum(to_frame(points[i])[0] for i in palm_indices) / len(palm_indices)
    py = sum(to_frame(points[i])[1] for i in palm_indices) / len(palm_indices)

    is_open = best_label not in ("FIST", None)

    hand = HandDetection(
        center_x=px, center_y=py,
        wrist_x=wrist_x, wrist_y=wrist_y,
        is_open=is_open,
        confidence=best_hand.get_confidence(),
        timestamp=time.monotonic(),
    )
    return hand, best_label


# ---------------------------------------------------------------------------
# Main callback
# ---------------------------------------------------------------------------

_gesture_log_interval = 1.0
_gesture_last_log_time = 0.0


def _get_gesture_mode(user_data):
    """Read current gesture control mode from velocity_state (set by drone control loop)."""
    velocity_state = getattr(user_data, 'velocity_state', None)
    if velocity_state is None:
        return None
    _, _, _, _, mode = velocity_state.get()
    return mode


def _colorize_hand_detections(roi, gesture_mode):
    """Set hand detection bbox to green when gesture control is active (post-wave lock-on)."""
    active_modes = ("GESTURE", "FIST-STOP", "FACE-TRACK", "ACK")
    if gesture_mode not in active_modes:
        return
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    for det in detections:
        if det.get_label() == "hand":
            # Green = 0x00FF00 packed as 0xRRGGBB
            color_cls = hailo.HailoClassification("overlay_color", 0x00FF00, "", 0.0)
            det.add_object(color_cls)


def gesture_app_callback(element, buffer, user_data):
    """Pipeline callback: extracts person, face, palm, and hand metadata.

    Person + face come from the tiling YOLOv8n.
    Palm detections come from the palm_detection model (always on in gesture mode).
    Hand landmarks + gesture come from the hand_landmark model (only when locked on a palm).

    Palm tracking runs every frame. Hand landmarks are only extracted for the
    locked palm to reduce NPU load.
    """
    global _gesture_last_log_time

    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    persons = [d for d in detections if d.get_label() == "person"]
    faces = [d for d in detections if d.get_label() == "face"]

    target_state = user_data.target_state
    ui_state = user_data.ui_state
    gesture_state = user_data.gesture_state
    shared_state = user_data.shared_state
    palm_state = user_data.palm_state
    palm_lock = user_data.palm_lock
    selected_detection = None
    hand = None
    gesture_label = None
    now = time.monotonic()

    # --- Palm tracking (via GStreamer hailotracker, class_id=100) ---
    locked_id = palm_lock.get_locked_palm_id() if palm_lock is not None else None
    hand_landmarks_on = palm_lock.hand_landmarks_enabled if palm_lock is not None else True

    tracked_palms = _extract_palms_from_roi(roi)
    if palm_state is not None:
        palm_state.update(tracked_palms)

    if not persons:
        user_data.byte_tracker.update(_EMPTY_DET_ARRAY)
        shared_state.update(None, available_ids=set())
        gesture_state.update(None)
        if target_state is not None and target_state.get_target() is not None:
            target_state.set_target(None)
        _update_ui(ui_state, [], {}, None)
    else:
        available_ids, person_by_id, person_to_id = _run_tracker(
            user_data.byte_tracker, persons)

        # --- Target selection (same as follow mode) ---
        target_id = target_state.get_target() if target_state is not None else None
        best = None
        if target_id is not None:
            best = person_by_id.get(target_id)
            if best is not None:
                target_state.update_last_seen()
            else:
                shared_state.update(None, available_ids=available_ids)
                gesture_state.update(None)
                if target_state.get_target() is not None:
                    target_state.set_target(None)
                _update_ui(ui_state, persons, person_to_id, None)
                best = None
        else:
            best = max(persons, key=lambda d: d.get_bbox().width() * d.get_bbox().height())
            best_tid = person_to_id.get(id(best))
            if best_tid is not None and target_state is not None:
                target_state.set_target(best_tid)

        if best is not None:
            pbbox = best.get_bbox()
            cx = pbbox.xmin() + pbbox.width() / 2
            cy = pbbox.ymin() + pbbox.height() / 2

            selected_detection = Detection(
                label="person",
                confidence=best.get_confidence(),
                center_x=cx,
                center_y=cy,
                bbox_height=pbbox.height(),
                timestamp=now,
            )
            shared_state.update(selected_detection, available_ids=available_ids)

            # --- Face: match from sibling detections ---
            matched_face_det = _match_face_to_person(faces, pbbox)
            face = None
            if matched_face_det is not None:
                fb = matched_face_det.get_bbox()
                face = FaceDetection(
                    center_x=fb.xmin() + fb.width() / 2,
                    center_y=fb.ymin() + fb.height() / 2,
                    bbox_width=fb.width(),
                    bbox_height=fb.height(),
                    confidence=matched_face_det.get_confidence(),
                    timestamp=now,
                )

            # --- Hand: only extract for locked palm ---
            # To reduce NPU load, hand landmarks run every Nth frame.
            # The callback toggles hand_landmark_hailonet pass-through and
            # caches the last good HandDetection for skip frames.
            if locked_id is not None and hand_landmarks_on:
                locked_palm = None
                for p in tracked_palms:
                    if p.track_id == locked_id:
                        locked_palm = p
                        break
                if locked_palm is not None:
                    # Determine if this frame had landmarks (active on previous toggle)
                    hand, gesture_label = _find_hand_near_palm(
                        roi, locked_palm.center_x, locked_palm.center_y)
                    if hand is not None:
                        # Fresh landmarks — cache them
                        user_data.cached_hand[locked_id] = hand
                        user_data.cached_gesture[locked_id] = gesture_label
                    else:
                        # Skip frame — reuse cached hand with updated palm position
                        cached = user_data.cached_hand.get(locked_id)
                        if cached is not None:
                            hand = HandDetection(
                                center_x=locked_palm.center_x,
                                center_y=locked_palm.center_y,
                                wrist_x=cached.wrist_x,
                                wrist_y=cached.wrist_y,
                                is_open=cached.is_open,
                                confidence=cached.confidence,
                                timestamp=now,
                            )
                            gesture_label = user_data.cached_gesture.get(locked_id)

                # Toggle hand_landmark_hailonet for NEXT frame
                user_data.hand_landmark_frame_count += 1
                hailonet = user_data._hand_landmark_hailonet
                if hailonet is not None:
                    should_run = (user_data.hand_landmark_frame_count % user_data.hand_landmark_period) == 0
                    if should_run and not user_data._landmarks_active:
                        hailonet.set_property("pass-through", False)
                        user_data._landmarks_active = True
                    elif not should_run and user_data._landmarks_active:
                        hailonet.set_property("pass-through", True)
                        user_data._landmarks_active = False

            elif locked_id is None and not hand_landmarks_on:
                # Not locked, hand landmarks off — no hand data (expected)
                user_data.cached_hand.clear()
                user_data.cached_gesture.clear()
                user_data.hand_landmark_frame_count = 0
                if user_data._landmarks_active:
                    hailonet = user_data._hand_landmark_hailonet
                    if hailonet is not None:
                        hailonet.set_property("pass-through", True)
                    user_data._landmarks_active = False
            else:
                # Fallback: extract any hand (e.g. during transition)
                hand, gesture_label = _extract_hand_from_roi(roi)

            if face is not None:
                gesture_state.update(GestureDetection(
                    face=face,
                    hand=hand,
                    person_bbox_height=pbbox.height(),
                    timestamp=now,
                ))
            else:
                gesture_state.update(None)

            # Update UI
            following_id = target_state.get_target() if target_state else None
            _update_ui(ui_state, persons, person_to_id, following_id)

    # --- Gesture mode overlay coloring and logging ---
    gesture_mode = _get_gesture_mode(user_data)
    _colorize_hand_detections(roi, gesture_mode)

    # Periodic logging (terminal + UI)
    if now - _gesture_last_log_time >= _gesture_log_interval:
        _gesture_last_log_time = now
        hand_str = "none"
        if hand is not None:
            hand_str = f"{'open' if hand.is_open else 'fist'} ({hand.center_x:.2f},{hand.center_y:.2f})"
        face_str = "none"
        if selected_detection is not None:
            face_str = f"({selected_detection.center_x:.2f},{selected_detection.center_y:.2f})"
        mode_str = gesture_mode or "NO-DRONE"
        palm_ids = [p.track_id for p in tracked_palms]
        palm_str = f"palms={palm_ids}"
        lock_str = f"lock={locked_id}" if locked_id is not None else "lock=none"
        log_msg = (f"[gesture] mode={mode_str} hand={hand_str} "
                   f"gesture={gesture_label} person={face_str} "
                   f"{palm_str} {lock_str} "
                   f"persons={len(persons)} faces={len(faces)}")
        LOGGER.info(log_msg)
        if ui_state is not None:
            ui_state.push_log(log_msg)

    # Attach flight command HUD arrows (runs every frame, with or without drone)
    velocity_state = getattr(user_data, 'velocity_state', None)
    config = getattr(user_data, 'controller_config', None)
    _attach_velocity_arrows(roi, velocity_state, detection=selected_detection, config=config)


# ---------------------------------------------------------------------------
# Pipeline app factory
# ---------------------------------------------------------------------------

def create_gesture_app(shared_state, gesture_state, target_state=None, eos_reached=None,
                       ui_state=None, ui_fps=10, parser=None, record_dir=None,
                       initial_follow_mode="follow", velocity_state=None,
                       palm_state=None, palm_lock=None):
    """Create the gesture pipeline app.

    Extends the tiling pipeline (YOLOv8n for person + face) with C++ palm detection,
    hand landmark, and gesture classification — all running on the Hailo NPU.
    """
    from hailo_apps.python.pipeline_apps.tiling.tiling_pipeline import (
        GStreamerTilingApp,
    )
    from hailo_apps.python.core.gstreamer.gstreamer_app import app_callback_class
    from hailo_apps.python.core.common.core import get_pipeline_parser
    from hailo_apps.python.core.common.defines import SHARED_VDEVICE_GROUP_ID
    from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
        QUEUE, DISPLAY_PIPELINE, OVERLAY_PIPELINE,
        INFERENCE_PIPELINE, USER_CALLBACK_PIPELINE,
        TILE_CROPPER_PIPELINE, SOURCE_PIPELINE,
        TRACKER_PIPELINE,
    )

    if parser is None:
        parser = get_pipeline_parser()

    class GestureUserData(app_callback_class):
        def __init__(self, shared_state, gesture_state, target_state=None,
                     ui_state=None, byte_tracker=None, velocity_state=None,
                     controller_config=None, palm_state=None, palm_lock=None):
            super().__init__()
            self.shared_state = shared_state
            self.gesture_state = gesture_state
            self.target_state = target_state
            self.ui_state = ui_state
            self.byte_tracker = byte_tracker
            self.velocity_state = velocity_state
            self.controller_config = controller_config
            self.palm_state = palm_state
            self.palm_lock = palm_lock
            # Cache last HandDetection per palm track ID for landmark skip frames
            self.cached_hand = {}        # {palm_track_id: HandDetection}
            self.cached_gesture = {}     # {palm_track_id: str or None}
            # Frame counter for hand landmark skip: run every Nth frame
            self.hand_landmark_frame_count = 0
            self.hand_landmark_period = 3  # run landmarks every 3rd frame
            self._landmarks_active = False  # current pass-through state
            self._hand_landmark_hailonet = None  # set after pipeline creation

    class GestureTilingApp(GStreamerTilingApp):
        """Tiling + gesture pipeline with EOS handling and optional MJPEG appsink."""
        def __init__(self, app_callback, user_data, parser=None, eos_reached=None,
                     ui_enabled=False, ui_state=None, ui_fps=30, record_dir=None,
                     initial_follow_mode="follow"):
            self._eos_reached = eos_reached
            self._ui_enabled = ui_enabled
            self._ui_state = ui_state
            self._ui_fps = ui_fps
            self._initial_follow_mode = initial_follow_mode
            self._gesture_enabled = (initial_follow_mode == "gesture")
            self._hand_landmarks_enabled = False  # pipeline starts with pass-through=true
            self._recording = False
            self._record_dir = record_dir or os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "..", "recordings")
            self._record_lock = __import__("threading").Lock()
            super().__init__(app_callback, user_data, parser=parser)
            if self._ui_enabled:
                self._connect_mjpeg_sink()

        def _connect_mjpeg_sink(self):
            self._Gst = _get_gst()
            mjpeg_sink = self.pipeline.get_by_name("mjpeg_sink")
            if mjpeg_sink:
                mjpeg_sink.connect("new-sample", self._on_mjpeg_sample)

        def _on_mjpeg_sample(self, appsink):
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

        @property
        def is_recording(self):
            return self._recording

        def _generate_record_path(self):
            from datetime import datetime
            os.makedirs(self._record_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            return os.path.join(self._record_dir, f"rec_{ts}.mkv")

        def start_recording(self, path=None):
            Gst = _get_gst()
            with self._record_lock:
                if self._recording or not self._ui_enabled:
                    return None
                valve = self.pipeline.get_by_name("record_valve")
                filesink = self.pipeline.get_by_name("record_sink")
                if valve is None or filesink is None:
                    return None
                encoder = self.pipeline.get_by_name("record_enc")
                muxer = self.pipeline.get_by_name("record_mux")
                record_path = path or self._generate_record_path()
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
            Gst = _get_gst()
            with self._record_lock:
                if not self._recording:
                    return None
                valve = self.pipeline.get_by_name("record_valve")
                if valve is None:
                    self._recording = False
                    return None
                encoder = self.pipeline.get_by_name("record_enc")
                muxer = self.pipeline.get_by_name("record_mux")
                filesink = self.pipeline.get_by_name("record_sink")
                self._recording = False
                path = getattr(self, "_current_record_path", None)
                valve.set_property("drop", True)
                for el in (encoder, muxer, filesink):
                    if el is not None:
                        el.set_state(Gst.State.NULL)
                LOGGER.info("[record] Stopped recording: %s", path)
                return path

        _GESTURE_HAILONET_NAMES = ("palm_detection_hailonet", "hand_landmark_hailonet")
        _HAND_LANDMARK_NAMES = ("hand_landmark_hailonet",)

        def _set_gesture_passthrough(self, passthrough: bool):
            """Set pass-through on gesture hailonets. Only safe after pipeline is PLAYING."""
            for name in self._GESTURE_HAILONET_NAMES:
                el = self.pipeline.get_by_name(name)
                if el is not None:
                    el.set_property("pass-through", passthrough)
                else:
                    LOGGER.warning("[gesture] hailonet '%s' not found", name)

        def enable_gesture(self):
            """Enable palm/hand/gesture NPU inference (for gesture mode).

            Enables palm detection but keeps hand landmarks off until a palm is locked.
            """
            if self._gesture_enabled:
                return
            # Enable palm detection
            el = self.pipeline.get_by_name("palm_detection_hailonet")
            if el is not None:
                el.set_property("pass-through", False)
            # Hand landmarks start disabled (already pass-through=true in pipeline string)
            self._gesture_enabled = True
            self._hand_landmarks_enabled = False
            LOGGER.info("[gesture] Gesture mode ENABLED (palm detection on, hand landmarks off)")

        def disable_gesture(self):
            """Disable palm/hand/gesture NPU inference (for follow mode, saves compute).

            Only call after the pipeline is in PLAYING state — setting pass-through
            during preroll can cause deadlocks.
            """
            if not self._gesture_enabled:
                return
            self._set_gesture_passthrough(True)
            self._gesture_enabled = False
            self._hand_landmarks_enabled = False
            LOGGER.info("[gesture] Gesture NPU inference DISABLED (pass-through)")

        def _set_hand_landmarks_passthrough(self, passthrough: bool):
            """Toggle hand_landmark_hailonet pass-through only."""
            for name in self._HAND_LANDMARK_NAMES:
                el = self.pipeline.get_by_name(name)
                if el is not None:
                    el.set_property("pass-through", passthrough)

        def enable_hand_landmarks(self):
            """Enable hand landmark inference (after palm lock-on)."""
            if self._hand_landmarks_enabled:
                return
            self._set_hand_landmarks_passthrough(False)
            self._hand_landmarks_enabled = True
            LOGGER.info("[gesture] Hand landmark NPU inference ENABLED (palm locked)")

        def disable_hand_landmarks(self):
            """Disable hand landmark inference (palm unlocked, saves compute)."""
            if not self._hand_landmarks_enabled:
                return
            self._set_hand_landmarks_passthrough(True)
            self._hand_landmarks_enabled = False
            LOGGER.info("[gesture] Hand landmark NPU inference DISABLED (palm unlocked)")

        @property
        def gesture_enabled(self):
            return self._gesture_enabled

        @property
        def hand_landmarks_enabled(self):
            return getattr(self, '_hand_landmarks_enabled', False)

        def get_pipeline_string(self):
            # --- 1. Source ---
            source_pipeline = SOURCE_PIPELINE(
                video_source=self.video_source,
                video_width=self.video_width,
                video_height=self.video_height,
                frame_rate=self.frame_rate,
                sync=self.sync,
            )

            # --- 2. Tiling (person + face detection, YOLOv8n) ---
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

            # --- 3. Person-crop palm detection ---
            # Crops per-person square regions for palm detection, giving the palm
            # model a zoomed-in view of each person instead of the whole frame.
            # After aggregation, palm detections are nested inside person_palm_crop
            # sub-ROIs; palm_to_hand_crop promotes them to the main ROI.
            palm_detection_pipeline = INFERENCE_PIPELINE(
                hef_path=DEFAULT_PALM_HEF,
                post_process_so=PALM_DETECTION_POST_SO,
                batch_size=1,
                name="palm_detection",
                scheduler_timeout_ms=33,
            )

            palm_detection_wrapper = (
                f"{QUEUE(name='palm_wrapper_input_q')} ! "
                f"hailocropper name=palm_wrapper_crop "
                f"so-path={PERSON_PALM_CROPPERS_SO} "
                f"function-name=person_palm_crop "
                f"use-letterbox=false "
                f"internal-offset=true "
                f"hailoaggregator name=palm_wrapper_agg "
                f"palm_wrapper_crop. ! {QUEUE(name='palm_wrapper_bypass_q', max_size_buffers=20)} ! palm_wrapper_agg.sink_0 "
                f"palm_wrapper_crop. ! {palm_detection_pipeline} ! palm_wrapper_agg.sink_1 "
                f"palm_wrapper_agg. ! {QUEUE(name='palm_wrapper_output_q')} "
            )

            # --- 3b. Palm tracker (Hailo GStreamer tracker for stable palm IDs) ---
            # class_id=100: matches PALM_CLASS_ID set in palm_detection_postprocess.cpp,
            # so hailotracker only tracks palms — not persons, faces, or other detections.
            palm_tracker_pipeline = TRACKER_PIPELINE(
                class_id=100,
                kalman_dist_thr=0.7,
                iou_thr=0.8,
                init_iou_thr=0.6,
                keep_new_frames=2,
                keep_tracked_frames=30,
                keep_lost_frames=5,
                name="palm_tracker",
            )

            # --- 4. Hand landmark inner pipeline (inside palm cropper) ---
            # Starts with pass-through=true: hand landmarks are only enabled
            # after a wave gesture locks onto a specific palm (saves NPU).
            inner_pipeline = (
                f"{QUEUE(name='hand_scale_q')} ! "
                f"videoscale name=hand_videoscale n-threads=2 qos=false ! "
                f"video/x-raw, width=224, height=224, pixel-aspect-ratio=1/1 ! "
                f"videoconvert name=hand_videoconvert n-threads=2 ! "
                f"hailofilter so-path={HAND_AFFINE_WARP_SO} "
                f"name=hand_affine_warp use-gst-buffer=true qos=false ! "
                f"{QUEUE(name='hand_hailonet_q')} ! "
                f"hailonet name=hand_landmark_hailonet "
                f"hef-path={DEFAULT_HAND_HEF} "
                f"batch-size=1 "
                f"vdevice-group-id={SHARED_VDEVICE_GROUP_ID} "
                f"scheduler-timeout-ms=33 "
                f"pass-through=true "
                f"force-writable=true ! "
                f"{QUEUE(name='hand_postproc_q')} ! "
                f"hailofilter name=hand_landmark_postproc "
                f"so-path={HAND_LANDMARK_POST_SO} qos=false ! "
                f"{QUEUE(name='hand_output_q')} "
            )

            # --- 5. Palm cropper (palm → rotated hand crop → inner pipeline) ---
            # Both this cropper and the person crop wrapper above operate on the
            # full 720p frame buffer — all crops are taken at full resolution.
            palm_cropper_pipeline = (
                f"{QUEUE(name='palm_cropper_input_q')} ! "
                f"hailocropper name=palm_cropper "
                f"so-path={PALM_CROPPERS_SO} "
                f"function-name=palm_to_hand_crop "
                f"use-letterbox=false "
                f"no-scaling-bbox=true "
                f"internal-offset=true "
                f"hailoaggregator name=palm_agg "
                f"palm_cropper. ! "
                f"{QUEUE(name='palm_bypass_q', max_size_buffers=20)} ! palm_agg.sink_0 "
                f"palm_cropper. ! {inner_pipeline} ! palm_agg.sink_1 "
                f"palm_agg. ! {QUEUE(name='palm_cropper_output_q')} "
            )

            # --- 6. Gesture classification (removes palm/person_palm_crop detections, adds gesture label) ---
            # Toggled to pass-through at runtime (after PLAYING) together with
            # hand_landmark_hailonet so raw palm detections survive to the callback.
            gesture_filter = (
                f"{QUEUE(name='gesture_filter_q')} ! "
                f"hailofilter so-path={GESTURE_CLASSIFICATION_SO} "
                f"name=gesture_classification qos=false "
            )

            # --- 7. User callback ---
            user_callback_pipeline = USER_CALLBACK_PIPELINE()

            # --- 8. Display / UI output ---
            if not self._ui_enabled:
                display_pipeline = DISPLAY_PIPELINE(
                    video_sink=self.video_sink, sync=self.sync, show_fps=self.show_fps,
                )
                pipeline_string = (
                    f"{source_pipeline} ! "
                    f"{tile_cropper_pipeline} ! "
                    f"{palm_detection_wrapper} ! "
                    f"{palm_cropper_pipeline} ! "
                    f"{palm_tracker_pipeline} ! "
                    f"{gesture_filter} ! "
                    f"{user_callback_pipeline} ! "
                    f"{display_pipeline}"
                )
            else:
                display_branch = DISPLAY_PIPELINE(
                    video_sink=self.video_sink, sync=self.sync, show_fps=self.show_fps,
                    community_overlay=True,
                    overlay_props={"hud_overlay": True, "use_custom_colors": True},
                )
                mjpeg_branch = (
                    f"videoconvert n-threads=2 ! "
                    f"videorate max-rate={self._ui_fps} ! "
                    f"video/x-raw,framerate={self._ui_fps}/1 ! "
                    f"jpegenc quality=70 ! "
                    f"appsink name=mjpeg_sink sync=false drop=true emit-signals=true"
                )
                record_branch = (
                    f"valve name=record_valve drop=true ! "
                    f"{OVERLAY_PIPELINE(name='record_overlay', community=True, hud_overlay=True, use_custom_colors=True)} ! "
                    f"videoconvert n-threads=2 ! "
                    f"x264enc name=record_enc tune=zerolatency bitrate=5000 speed-preset=ultrafast ! "
                    f"matroskamux name=record_mux ! filesink name=record_sink async=false location=/dev/null"
                )
                output_pipeline = (
                    f"tee name=ui_tee "
                    f"ui_tee. ! {QUEUE(name='display_branch_q', leaky='downstream')} ! {display_branch} "
                    f"ui_tee. ! {QUEUE(name='mjpeg_branch_q', leaky='downstream')} ! {mjpeg_branch} "
                    f"ui_tee. ! {QUEUE(name='record_branch_q', max_size_buffers=1, leaky='downstream')} ! {record_branch}"
                )
                pipeline_string = (
                    f"{source_pipeline} ! "
                    f"{tile_cropper_pipeline} ! "
                    f"{palm_detection_wrapper} ! "
                    f"{palm_cropper_pipeline} ! "
                    f"{palm_tracker_pipeline} ! "
                    f"{gesture_filter} ! "
                    f"{user_callback_pipeline} ! "
                    f"{output_pipeline}"
                )

            LOGGER.info("[gesture] Pipeline: tiling + palm_detection + hand_landmark + gesture_classification")
            return pipeline_string

    tracker = ByteTracker(
        track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30,
    )
    LOGGER.info("[gesture] ByteTracker (persons) + hailotracker (palms, class_id=100)")

    user_data = GestureUserData(
        shared_state, gesture_state, target_state, ui_state=ui_state,
        byte_tracker=tracker, velocity_state=velocity_state,
        palm_state=palm_state, palm_lock=palm_lock,
    )
    app = GestureTilingApp(
        gesture_app_callback, user_data, parser=parser, eos_reached=eos_reached,
        ui_enabled=(ui_state is not None), ui_state=ui_state, ui_fps=ui_fps,
        record_dir=record_dir, initial_follow_mode=initial_follow_mode,
    )
    # Store hailonet element reference for callback-driven pass-through toggling
    el = app.pipeline.get_by_name("hand_landmark_hailonet")
    if el is not None:
        user_data._hand_landmark_hailonet = el
    return app
