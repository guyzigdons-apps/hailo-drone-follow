"""Pose-based gesture pipeline adapter — YOLOv8-Pose wrist keypoints for wave detection and drone control.

Uses a single YOLOv8-Pose model instead of the cascaded palm_detection + hand_landmark pipeline.
Wrist keypoints (COCO indices 9/10) drive wave detection and gesture control.
Face is synthesized from nose + eye keypoints.

Populates SharedGestureState (for gesture control) and SharedDetectionState (for distance control).
"""

import logging
import math
import os
import subprocess
import threading
import time
from typing import Dict, Optional, Tuple

from drone_follow.follow_api.types import (
    Detection, FaceDetection, HandDetection, GestureDetection, WristDetection,
)
from drone_follow.follow_api.gesture_controller import WaveDetector

LOGGER = logging.getLogger("drone_follow.pose_gesture")

# COCO keypoint indices
NOSE = 0
LEFT_EYE = 1
RIGHT_EYE = 2
LEFT_SHOULDER = 5
RIGHT_SHOULDER = 6
LEFT_ELBOW = 7
RIGHT_ELBOW = 8
LEFT_WRIST = 9
RIGHT_WRIST = 10

# Pose model / postprocess constants (resolved at runtime via hailo-apps helpers)
POSE_ESTIMATION_PIPELINE = "pose_estimation"


def face_from_pose_keypoints(
    keypoints: Dict[int, Optional[Tuple[float, float]]],
    confidence: float,
    timestamp: float,
) -> Optional[FaceDetection]:
    """Synthesize a FaceDetection from nose and eye keypoints.

    Args:
        keypoints: Dict mapping COCO index -> (x, y) in frame-normalized coords,
                   or None if keypoint is missing/low-confidence.
        confidence: Overall confidence for the face detection.
        timestamp: Timestamp of the detection.

    Returns:
        FaceDetection or None if nose keypoint is missing.
    """
    nose = keypoints.get(NOSE)
    if nose is None:
        return None

    left_eye = keypoints.get(LEFT_EYE)
    right_eye = keypoints.get(RIGHT_EYE)

    # Center = average of available face keypoints
    pts = [nose]
    if left_eye is not None:
        pts.append(left_eye)
    if right_eye is not None:
        pts.append(right_eye)

    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)

    # Estimate face bbox from eye separation (or fixed fraction if no eyes)
    if left_eye is not None and right_eye is not None:
        eye_dist = math.sqrt((left_eye[0] - right_eye[0]) ** 2 +
                             (left_eye[1] - right_eye[1]) ** 2)
        bbox_w = eye_dist * 2.5
        bbox_h = eye_dist * 3.0
    else:
        bbox_w = 0.08
        bbox_h = 0.10

    return FaceDetection(
        center_x=cx, center_y=cy,
        bbox_width=bbox_w, bbox_height=bbox_h,
        confidence=confidence, timestamp=timestamp,
    )


def wrist_from_pose_keypoints(
    keypoints: Dict[int, Optional[Tuple[float, float]]],
    side: str,
    timestamp: float,
) -> Optional[WristDetection]:
    """Extract a WristDetection from pose keypoints.

    Args:
        keypoints: Dict mapping COCO index -> (x, y) in frame-normalized coords,
                   or None if keypoint is missing/low-confidence.
        side: "left" or "right"
        timestamp: Timestamp of the detection.

    Returns:
        WristDetection or None if wrist or shoulder keypoint is missing.
    """
    wrist_idx = LEFT_WRIST if side == "left" else RIGHT_WRIST
    shoulder_idx = LEFT_SHOULDER if side == "left" else RIGHT_SHOULDER

    wrist_pt = keypoints.get(wrist_idx)
    shoulder_pt = keypoints.get(shoulder_idx)

    if wrist_pt is None or shoulder_pt is None:
        return None

    return WristDetection(
        x=wrist_pt[0], y=wrist_pt[1],
        shoulder_x=shoulder_pt[0], shoulder_y=shoulder_pt[1],
        confidence=1.0,  # pose model doesn't give per-keypoint confidence in TAPPAS
        timestamp=timestamp,
    )


def wrist_to_hand_detection(wrist: WristDetection) -> HandDetection:
    """Map a WristDetection to a HandDetection for gesture controller compatibility.

    is_open = wrist above shoulder (raised hand).
    center_x/y and wrist_x/y are both set to the wrist position.

    Args:
        wrist: WristDetection from pose model.

    Returns:
        HandDetection compatible with gesture controller.
    """
    is_open = wrist.y < wrist.shoulder_y  # lower y = higher in frame
    return HandDetection(
        center_x=wrist.x,
        center_y=wrist.y,
        wrist_x=wrist.x,
        wrist_y=wrist.y,
        is_open=is_open,
        confidence=wrist.confidence,
        timestamp=wrist.timestamp,
    )


def select_active_wrist(
    left: Optional[WristDetection],
    right: Optional[WristDetection],
) -> Optional[WristDetection]:
    """Select the active (highest / most raised) wrist for gesture control.

    Returns the wrist with lower y value (higher in frame), or whichever is available.

    Args:
        left: Left wrist detection or None.
        right: Right wrist detection or None.

    Returns:
        The selected wrist, or None if both are None.
    """
    if left is None and right is None:
        return None
    if left is None:
        return right
    if right is None:
        return left
    return left if left.y < right.y else right


# ---------------------------------------------------------------------------
# Task 3: GStreamer callback helpers and callback
# ---------------------------------------------------------------------------

def _extract_pose_keypoints(detection, bbox) -> Dict[int, Tuple[float, float]]:
    """Extract COCO keypoints from a Hailo pose detection.

    Converts bbox-relative [0,1] coords to frame-normalized [0,1]:
        x = bbox.xmin() + pt.x() * bbox.width()
        y = bbox.ymin() + pt.y() * bbox.height()

    Args:
        detection: Hailo detection object with HAILO_LANDMARKS metadata.
        bbox: Hailo bounding box of the detection.

    Returns:
        Dict mapping COCO keypoint index -> (x, y) in frame-normalized coords.
        Returns empty dict if landmarks are missing or fewer than 17 points.
    """
    import hailo  # local import to avoid breaking tests

    landmarks_list = detection.get_objects_typed(hailo.HAILO_LANDMARKS)
    if not landmarks_list:
        return {}

    landmarks = landmarks_list[0]
    points = landmarks.get_points()
    if len(points) < 17:
        return {}

    result = {}
    for idx, pt in enumerate(points):
        x = bbox.xmin() + pt.x() * bbox.width()
        y = bbox.ymin() + pt.y() * bbox.height()
        result[idx] = (x, y)
    return result


_pose_last_log_time: float = 0.0
_pose_log_interval: float = 2.0


def pose_gesture_app_callback(element, buffer, user_data):
    """GStreamer pipeline callback for pose-based gesture detection.

    Extracts person detections from YOLOv8-Pose, runs ByteTracker for person
    tracking, selects the target person, extracts wrist keypoints, runs wave
    detection on all visible persons, and updates gesture_state.

    Follows the same pattern as gesture_app_callback in gesture_detection_manager.py.
    """
    import hailo  # local import to avoid breaking tests
    from .byte_tracker import ByteTracker  # noqa: F401 — used via user_data
    from .hailo_drone_detection_manager import (
        _get_gst, _build_det_info, _run_tracker, _update_ui,
        _EMPTY_DET_ARRAY, _attach_velocity_arrows,
    )

    global _pose_last_log_time

    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    persons = [d for d in detections if d.get_label() == "person"]

    target_state = user_data.target_state
    ui_state = user_data.ui_state
    gesture_state = user_data.gesture_state
    shared_state = user_data.shared_state
    selected_detection = None
    face = None
    hand = None
    now = time.monotonic()

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

        # --- Target selection (largest or previously tracked) ---
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

            # --- Extract pose keypoints and synthesize face ---
            keypoints = _extract_pose_keypoints(best, pbbox)
            if keypoints:
                face = face_from_pose_keypoints(keypoints, best.get_confidence(), now)

            # --- Wave detection: runs on all visible persons ---
            # Build map of track_id -> (person, keypoints) for wave candidates
            visible_ids = set(person_to_id.values())

            # Clean up wave detectors for persons no longer visible
            stale = [pid for pid in user_data.wave_detectors if pid not in visible_ids]
            for pid in stale:
                del user_data.wave_detectors[pid]

            # If active person lost its track, clear it
            if (user_data.active_person_id is not None
                    and user_data.active_person_id not in visible_ids):
                user_data.active_person_id = None

            # Run wave detection on each visible person (only if no active person yet)
            if user_data.active_person_id is None:
                cfg = getattr(user_data, 'controller_config', None)
                for person in persons:
                    p_tid = person_to_id.get(id(person))
                    if p_tid is None:
                        continue
                    p_bbox = person.get_bbox()
                    p_kps = _extract_pose_keypoints(person, p_bbox)
                    if not p_kps:
                        continue
                    left_wrist = wrist_from_pose_keypoints(p_kps, "left", now)
                    right_wrist = wrist_from_pose_keypoints(p_kps, "right", now)
                    active_wrist = select_active_wrist(left_wrist, right_wrist)
                    if active_wrist is None:
                        continue
                    if p_tid not in user_data.wave_detectors:
                        user_data.wave_detectors[p_tid] = WaveDetector(
                            reversals_needed=cfg.gesture_wave_reversals if cfg else 3,
                            window_s=cfg.gesture_wave_window_s if cfg else 1.5,
                        )
                    wd = user_data.wave_detectors[p_tid]
                    if wd.update(active_wrist.x, now):
                        user_data.active_person_id = p_tid
                        LOGGER.info("[pose-gesture] Wave detected on person %d — active", p_tid)
                        break

            # --- Map active wrist to HandDetection after wave ---
            active_id = user_data.active_person_id
            if active_id is not None:
                active_person = person_by_id.get(active_id)
                if active_person is not None:
                    ap_bbox = active_person.get_bbox()
                    ap_kps = _extract_pose_keypoints(active_person, ap_bbox)
                    if ap_kps:
                        left_wrist = wrist_from_pose_keypoints(ap_kps, "left", now)
                        right_wrist = wrist_from_pose_keypoints(ap_kps, "right", now)
                        active_wrist = select_active_wrist(left_wrist, right_wrist)
                        if active_wrist is not None:
                            hand = wrist_to_hand_detection(active_wrist)

            # --- Update gesture_state ---
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

    # --- Periodic logging ---
    if now - _pose_last_log_time >= _pose_log_interval:
        _pose_last_log_time = now
        hand_str = "none"
        if hand is not None:
            hand_str = f"{'open' if hand.is_open else 'fist'} ({hand.center_x:.2f},{hand.center_y:.2f})"
        face_str = "none"
        if selected_detection is not None:
            face_str = f"({selected_detection.center_x:.2f},{selected_detection.center_y:.2f})"
        active_id = user_data.active_person_id
        lock_str = f"active={active_id}" if active_id is not None else "active=none"
        log_msg = (f"[pose-gesture] hand={hand_str} person={face_str} "
                   f"{lock_str} persons={len(persons)}")
        LOGGER.info(log_msg)
        if ui_state is not None:
            ui_state.push_log(log_msg)

    # --- HUD velocity arrows ---
    velocity_state = getattr(user_data, 'velocity_state', None)
    config = getattr(user_data, 'controller_config', None)
    _attach_velocity_arrows(roi, velocity_state, detection=selected_detection, config=config)


# ---------------------------------------------------------------------------
# Task 4: Pipeline factory
# ---------------------------------------------------------------------------

def create_pose_gesture_app(shared_state, gesture_state, target_state=None, eos_reached=None,
                             ui_state=None, ui_fps=10, parser=None, record_dir=None,
                             velocity_state=None, no_overlay=False):
    """Create the pose gesture pipeline app.

    Uses a single YOLOv8-Pose model for both person detection and wrist keypoint
    extraction. Simpler than the tiling+palm pipeline — no cascaded models.
    """
    from hailo_apps.python.core.gstreamer.gstreamer_app import GStreamerApp, app_callback_class
    from hailo_apps.python.core.common.core import get_pipeline_parser, get_resource_path, resolve_hef_path
    from hailo_apps.python.core.common.defines import (
        POSE_ESTIMATION_PIPELINE, POSE_ESTIMATION_POSTPROCESS_FUNCTION,
        POSE_ESTIMATION_POSTPROCESS_SO_FILENAME, RESOURCES_SO_DIR_NAME,
    )
    from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
        QUEUE, DISPLAY_PIPELINE, INFERENCE_PIPELINE, INFERENCE_PIPELINE_WRAPPER,
        SOURCE_PIPELINE, TRACKER_PIPELINE, USER_CALLBACK_PIPELINE,
    )
    from .byte_tracker import ByteTracker
    from .hailo_drone_detection_manager import _get_gst

    if parser is None:
        parser = get_pipeline_parser()

    class PoseGestureUserData(app_callback_class):
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
            # Wave detection state (runs in callback, not in drone control loop)
            self.wave_detectors = {}       # {person_track_id: WaveDetector}
            self.active_person_id = None   # track ID of the wave-qualified person

    class PoseGestureApp(GStreamerApp):
        """Pose estimation pipeline with EOS handling and optional MJPEG appsink."""

        def __init__(self, app_callback, user_data, parser, eos_reached=None,
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
            self.app_callback = app_callback

            # GStreamerApp takes (args, user_data) — args is the parsed namespace
            super().__init__(parser, user_data)

            # Override HEF path using hailo-apps resolver
            self.hef_path = resolve_hef_path(
                self.hef_path, app_name=POSE_ESTIMATION_PIPELINE, arch=self.arch)

            # Postprocess SO
            self.post_process_so = get_resource_path(
                POSE_ESTIMATION_PIPELINE, RESOURCES_SO_DIR_NAME,
                self.arch, POSE_ESTIMATION_POSTPROCESS_SO_FILENAME)
            self.post_process_function = POSE_ESTIMATION_POSTPROCESS_FUNCTION

            self.batch_size = 2
            self.create_pipeline()

            if self._ui_enabled:
                self._connect_mjpeg_sink()

        def _connect_mjpeg_sink(self):
            self._Gst = _get_gst()
            mjpeg_sink = self.pipeline.get_by_name("mjpeg_sink")
            if mjpeg_sink:
                mjpeg_sink.connect("new-sample", self._on_mjpeg_sample)
            record_sink = self.pipeline.get_by_name("record_appsink")
            if record_sink:
                record_sink.connect("new-sample", self._on_record_sample)

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

        @property
        def is_recording(self):
            return self._recording

        def _generate_record_path(self):
            from datetime import datetime
            os.makedirs(self._record_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            return os.path.join(self._record_dir, f"rec_{ts}.mkv")

        def start_recording(self, path=None):
            """Spawn ffmpeg subprocess and open valve. Returns the output file path."""
            with self._record_lock:
                if self._recording or not self._ui_enabled:
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

        # --- Gesture enable/disable: no-ops (pose pipeline always runs) ---

        def enable_gesture(self):
            """No-op: pose pipeline always has gesture enabled."""
            pass

        def disable_gesture(self):
            """No-op: pose pipeline always has gesture enabled."""
            pass

        @property
        def gesture_enabled(self):
            return True

        def get_pipeline_string(self):
            # --- 1. Source ---
            source_pipeline = SOURCE_PIPELINE(
                video_source=self.video_source,
                video_width=self.video_width,
                video_height=self.video_height,
                frame_rate=self.frame_rate,
                sync=self.sync,
                mirror_image=False,
            )

            # --- 2. Pose inference (YOLOv8-Pose: person detection + keypoints) ---
            inference_pipeline = INFERENCE_PIPELINE(
                hef_path=self.hef_path,
                post_process_so=self.post_process_so,
                post_function_name=self.post_process_function,
                batch_size=self.batch_size,
            )
            inference_wrapper = INFERENCE_PIPELINE_WRAPPER(inference_pipeline)

            # --- 3. Person tracker ---
            tracker_pipeline = TRACKER_PIPELINE(class_id=0)

            # --- 4. User callback ---
            user_callback_pipeline = USER_CALLBACK_PIPELINE()

            # --- 5. Display / UI output ---
            if not self._ui_enabled:
                overlay_element = "" if self._no_overlay else "hailooverlay name=hailo_overlay ! "
                display_pipeline = (
                    f"{overlay_element}"
                    f"videoconvert n-threads=2 ! "
                    f"fpsdisplaysink name=hailo_display video-sink={self.video_sink} "
                    f"sync={self.sync} text-overlay={self.show_fps} signal-fps-measurements=true"
                )
                pipeline_string = (
                    f"{source_pipeline} ! "
                    f"{inference_wrapper} ! "
                    f"{tracker_pipeline} ! "
                    f"{user_callback_pipeline} ! "
                    f"{display_pipeline}"
                )
            else:
                # MJPEG branch for web UI (React draws bboxes client-side)
                mjpeg_branch = (
                    f"videoconvert n-threads=2 ! "
                    f"videorate max-rate={self._ui_fps} ! "
                    f"video/x-raw,framerate={self._ui_fps}/1 ! "
                    f"jpegenc quality=70 ! "
                    f"appsink name=mjpeg_sink sync=false drop=true emit-signals=true"
                )

                # Overlay, then tee into display + record paths
                overlay_element = "" if self._no_overlay else "hailooverlay name=hailo_overlay ! "
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
                pipeline_string = (
                    f"{source_pipeline} ! "
                    f"{inference_wrapper} ! "
                    f"{tracker_pipeline} ! "
                    f"{user_callback_pipeline} ! "
                    f"{output_pipeline}"
                )

            LOGGER.info("[pose-gesture] Pipeline: YOLOv8-Pose person detection + wrist keypoints")
            return pipeline_string

    from .byte_tracker import ByteTracker as _ByteTracker
    tracker = _ByteTracker(
        track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30,
    )
    LOGGER.info("[pose-gesture] ByteTracker (persons) initialized")

    user_data = PoseGestureUserData(
        shared_state, gesture_state, target_state, ui_state=ui_state,
        byte_tracker=tracker, velocity_state=velocity_state,
    )
    app = PoseGestureApp(
        pose_gesture_app_callback, user_data, parser=parser,
        eos_reached=eos_reached,
        ui_enabled=(ui_state is not None), ui_state=ui_state, ui_fps=ui_fps,
        record_dir=record_dir, no_overlay=no_overlay,
    )
    return app
