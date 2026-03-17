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

from drone_follow.follow_api.types import Detection, FaceDetection, HandDetection, GestureDetection

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


# ---------------------------------------------------------------------------
# Main callback
# ---------------------------------------------------------------------------

def gesture_app_callback(element, buffer, user_data):
    """Pipeline callback: extracts person, face, and hand metadata from the C++ pipeline.

    Person + face come from the tiling YOLOv8n.
    Hand + landmarks + gesture come from the palm/hand C++ stages.
    """
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    persons = [d for d in detections if d.get_label() == "person"]
    faces = [d for d in detections if d.get_label() == "face"]

    target_state = user_data.target_state
    ui_state = user_data.ui_state
    gesture_state = user_data.gesture_state
    shared_state = user_data.shared_state
    selected_detection = None

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
            now = time.monotonic()

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

            # --- Hand: extract from C++ pipeline metadata ---
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

            if hand is not None:
                LOGGER.debug("[gesture] hand=(%s, %.2f,%.2f) gesture=%s",
                             "open" if hand.is_open else "fist",
                             hand.center_x, hand.center_y, gesture_label)

    # Attach flight command HUD arrows (runs every frame, with or without drone)
    velocity_state = getattr(user_data, 'velocity_state', None)
    config = getattr(user_data, 'controller_config', None)
    _attach_velocity_arrows(roi, velocity_state, detection=selected_detection, config=config)


# ---------------------------------------------------------------------------
# Pipeline app factory
# ---------------------------------------------------------------------------

def create_gesture_app(shared_state, gesture_state, target_state=None, eos_reached=None,
                       ui_state=None, ui_fps=10, parser=None, record_dir=None,
                       initial_follow_mode="follow", velocity_state=None):
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
    )

    if parser is None:
        parser = get_pipeline_parser()

    class GestureUserData(app_callback_class):
        def __init__(self, shared_state, gesture_state, target_state=None,
                     ui_state=None, byte_tracker=None, velocity_state=None,
                     controller_config=None):
            super().__init__()
            self.shared_state = shared_state
            self.gesture_state = gesture_state
            self.target_state = target_state
            self.ui_state = ui_state
            self.byte_tracker = byte_tracker
            self.velocity_state = velocity_state
            self.controller_config = controller_config

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

        def _set_gesture_passthrough(self, passthrough: bool):
            """Set pass-through on gesture hailonets. Only safe after pipeline is PLAYING."""
            for name in self._GESTURE_HAILONET_NAMES:
                el = self.pipeline.get_by_name(name)
                if el is not None:
                    el.set_property("pass-through", passthrough)
                else:
                    LOGGER.warning("[gesture] hailonet '%s' not found", name)

        def enable_gesture(self):
            """Enable palm/hand/gesture NPU inference (for gesture mode)."""
            if self._gesture_enabled:
                return
            self._set_gesture_passthrough(False)
            self._gesture_enabled = True
            LOGGER.info("[gesture] Gesture NPU inference ENABLED")

        def disable_gesture(self):
            """Disable palm/hand/gesture NPU inference (for follow mode, saves compute).

            Only call after the pipeline is in PLAYING state — setting pass-through
            during preroll can cause deadlocks.
            """
            if not self._gesture_enabled:
                return
            self._set_gesture_passthrough(True)
            self._gesture_enabled = False
            LOGGER.info("[gesture] Gesture NPU inference DISABLED (pass-through)")

        @property
        def gesture_enabled(self):
            return self._gesture_enabled

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

            # --- 4. Hand landmark inner pipeline (inside palm cropper) ---
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
                    f"{gesture_filter} ! "
                    f"{user_callback_pipeline} ! "
                    f"{display_pipeline}"
                )
            else:
                display_branch = DISPLAY_PIPELINE(
                    video_sink=self.video_sink, sync=self.sync, show_fps=self.show_fps,
                    community_overlay=True, overlay_props={"hud_overlay": True},
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
                    f"{OVERLAY_PIPELINE(name='record_overlay', community=True, hud_overlay=True)} ! "
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
                    f"{gesture_filter} ! "
                    f"{user_callback_pipeline} ! "
                    f"{output_pipeline}"
                )

            LOGGER.info("[gesture] Pipeline: tiling + palm_detection + hand_landmark + gesture_classification")
            return pipeline_string

    tracker = ByteTracker(
        track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30,
    )
    LOGGER.info("[gesture] ByteTracker running synchronously in callback")

    user_data = GestureUserData(
        shared_state, gesture_state, target_state, ui_state=ui_state,
        byte_tracker=tracker, velocity_state=velocity_state,
    )
    app = GestureTilingApp(
        gesture_app_callback, user_data, parser=parser, eos_reached=eos_reached,
        ui_enabled=(ui_state is not None), ui_state=ui_state, ui_fps=ui_fps,
        record_dir=record_dir, initial_follow_mode=initial_follow_mode,
    )
    return app
