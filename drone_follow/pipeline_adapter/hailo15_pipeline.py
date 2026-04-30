"""Hailo15 pipeline adapter — camera + inference using native H15 GStreamer + pyhailort.

Replaces the RPi/x86 hailo_apps tiling pipeline for Hailo15H/15L SoCs.

Pipeline architecture:
  hailofrontendbinsrc (ISP camera) →
    sink0 (4K)        → fakesink (unused)
    sink1 (720p@30fps) → hailoencodebin → H264 → UDP (for viewing on PC)
    sink2 (FHD@15fps)  → appsink → pyhailort inference → detections → SharedDetectionState
"""

import json
import logging
import os
import tempfile
import threading
import time
from typing import Optional

import numpy as np

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

from drone_follow.follow_api.types import Detection
from drone_follow.follow_api.state import SharedDetectionState, FollowTargetState
from .yolo_postprocess import extract_person_detections

try:
    from .byte_tracker import ByteTracker
except ImportError:
    ByteTracker = None

LOGGER = logging.getLogger("drone_follow.app")

# ---------------------------------------------------------------------------
# Configuration defaults
# ---------------------------------------------------------------------------
MEDIALIB_TOP_CONFIG_CANDIDATES = [
    "/etc/imaging/cfg/medialib_configs/face_landmarks_medialib_config.json",
    "/etc/imaging/cfg/medialib_configs/ai_example_medialib_config.json",
]
YOLO_HEF_CANDIDATES = [
    "/home/root/apps/face_landmarks/resources/hailo_yolov8s_384_640.hef",
    "/home/root/apps/face_landmarks/resources/hailo_yolov8n_384_640.hef",
    "/home/root/apps/ai_example_app/resources/hailo_yolov8s_384_640.hef",
    "/home/root/apps/ai_example_app/resources/hailo_yolov8n_384_640.hef",
]
UDP_HOST = "10.0.0.2"
UDP_PORT = 5002
DETECTION_CONFIDENCE_THRESHOLD = 0.4

# Defaults for frontend config fields that hailofrontendbinsrc requires
_FRONTEND_DEFAULTS = {
    "dewarp": {
        "enabled": False,
        "color_interpolation": "INTERPOLATION_TYPE_BILINEAR",
        "sensor_calib_path": "",
        "camera_type": "CAMERA_TYPE_PINHOLE",
    },
    "dis": {
        "enabled": False,
        "minimun_coefficient_filter": 0.1,
        "decrement_coefficient_threshold": 0.001,
        "increment_coefficient_threshold": 0.01,
        "running_average_coefficient": 0.033,
        "std_multiplier": 3.0,
        "black_corners_correction_enabled": True,
        "black_corners_threshold": 0.5,
        "average_luminance_threshold": 0,
        "camera_fov_factor": 0.85,
        "angular_dis": {
            "enabled": False,
            "vsm": {"hoffset": 0, "voffset": 0, "width": 1920, "height": 1080, "max_displacement": 64},
        },
        "debug": {
            "generate_resize_grid": False, "fix_stabilization": False,
            "fix_stabilization_longitude": 0.0, "fix_stabilization_latitude": 0.0,
        },
    },
    "eis": {
        "enabled": False, "stabilize": True, "eis_config_path": "", "window_size": 10,
        "rotational_smoothing_coefficient": 0.0, "iir_hpf_coefficient": 0.997,
        "camera_fov_factor": 0.85, "line_readout_time": 7410, "hdr_exposure_ratio": 0.2,
        "min_angle_deg": 0.005, "max_angle_deg": 6.0,
    },
    "gyro": {"enabled": False, "sensor_name": "", "sensor_frequency": "0", "scale": 0.0},
    "isp": {"isp_config_files_path": "/usr/bin"},
    "hdr": {"enabled": False, "dol": 2},
    "denoise": {
        "enabled": False, "sensor": "imx675", "method": "HIGH_QUALITY", "loopback-count": 1,
        "network": {
            "network_path": "", "y_channel": "", "uv_channel": "",
            "feedback_y_channel": "", "feedback_uv_channel": "",
            "output_y_channel": "", "output_uv_channel": "",
        },
    },
}


# ---------------------------------------------------------------------------
# Config resolution
# ---------------------------------------------------------------------------

def _resolve_frontend_config():
    """Resolve the medialib config chain to a full frontend config file.

    Returns (frontend_config_path, encoder_config_path, n_streams, app_settings).
    The frontend config is a temp file that must be cleaned up by the caller.
    """
    medialib_config = None
    for path in MEDIALIB_TOP_CONFIG_CANDIDATES:
        if os.path.isfile(path):
            medialib_config = path
            break
    if medialib_config is None:
        raise FileNotFoundError(
            f"No medialib config found. Checked: {MEDIALIB_TOP_CONFIG_CANDIDATES}. "
            "Run setup_hailo_sensor first.")

    with open(medialib_config) as f:
        top_config = json.load(f)

    default_profile = top_config.get("default_profile", "Daylight")
    profile_config_path = None
    for p in top_config.get("profiles", []):
        if p["name"] == default_profile:
            profile_config_path = p["config_file"]
            break

    if not profile_config_path or not os.path.isfile(profile_config_path):
        raise FileNotFoundError(f"Profile config not found: {profile_config_path}")

    with open(profile_config_path) as f:
        profile_config = json.load(f)

    app_settings_path = profile_config.get("application_settings")
    if not app_settings_path or not os.path.isfile(app_settings_path):
        raise FileNotFoundError(f"application_settings not found: {app_settings_path}")

    with open(app_settings_path) as f:
        app_settings = json.load(f)

    # Find encoder config for sink1
    encoder_config_path = None
    for es in profile_config.get("encoded_output_streams", []):
        if es.get("stream_id") == "sink1":
            encoder_config_path = es.get("encoding")
            break

    # Build full frontend config
    first_stream = app_settings["application_input_streams"]["resolutions"][0]
    frontend_full = {
        **_FRONTEND_DEFAULTS,
        **app_settings,
        "input_video": {
            "resolution": {
                "width": first_stream["width"],
                "height": first_stream["height"],
                "framerate": first_stream["framerate"],
            }
        },
    }
    frontend_full.pop("metadata", None)
    frontend_full.pop("version", None)

    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".json", prefix="frontend_", delete=False)
    json.dump(frontend_full, tmp, indent=2)
    tmp.close()

    n_streams = len(app_settings["application_input_streams"]["resolutions"])
    return tmp.name, encoder_config_path, n_streams, app_settings


def _find_yolo_hef():
    for path in YOLO_HEF_CANDIDATES:
        if os.path.isfile(path):
            return path
    raise FileNotFoundError("No YOLOv8 HEF found. Checked: " + ", ".join(YOLO_HEF_CANDIDATES))


# ---------------------------------------------------------------------------
# Hailo15 Pipeline App
# ---------------------------------------------------------------------------

class Hailo15PipelineApp:
    """GStreamer pipeline app for Hailo15 with pyhailort inference.

    Provides the same interface as GStreamerTilingApp so drone_follow_app.py
    can use it as a drop-in replacement.
    """

    def __init__(self, shared_state, target_state=None, ui_state=None,
                 eos_reached=None, options_menu=None):
        self.shared_state = shared_state
        self.target_state = target_state
        self.ui_state = ui_state
        self._eos_reached = eos_reached
        self.options_menu = options_menu

        self.loop = None
        self.pipeline = None
        self._frontend_config_path = None
        self._frame_count = 0
        self._recording = False

        # ByteTracker for consistent IDs (optional — requires scipy)
        self._byte_tracker = None
        if ByteTracker is not None:
            self._byte_tracker = ByteTracker(
                track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=15,
            )


        # pyhailort inference objects (initialized in _setup_inference)
        self._vdevice = None
        self._configured_model = None
        self._infer_model = None

        # UDP socket for sending detection data to PC viewer
        self._det_sock = None

        # Persistent JPEG encoder pipeline for web UI
        self._jpeg_enc = None
        self._jpeg_src = None
        self._jpeg_out = None

    # -- Recording stubs (not supported on H15 yet) --
    @property
    def is_recording(self):
        return False

    def start_recording(self, path=None):
        return None

    def stop_recording(self):
        return None

    def cleanup_recording_branch(self):
        pass

    # -- Setup --

    def _setup_inference(self):
        """Initialize pyhailort VDevice and load the YOLOv8 HEF."""
        from hailo_platform import VDevice

        hef_path = _find_yolo_hef()
        LOGGER.info("[h15] Loading HEF: %s", hef_path)

        self._vdevice = VDevice()
        self._infer_model = self._vdevice.create_infer_model(hef_path)

        # Don't override format types — let NMS output use its native format
        self._configured_model = self._infer_model.configure()

        input_info = self._infer_model.input()
        LOGGER.info("[h15] Model input: name=%s shape=%s", input_info.name, input_info.shape)
        for out in self._infer_model.outputs:
            LOGGER.info("[h15] Model output: name=%s shape=%s is_nms=%s",
                        out.name, out.shape, out.is_nms)

    def _build_pipeline(self):
        """Build the GStreamer pipeline string and create it."""
        frontend_path, encoder_path, n_streams, app_settings = _resolve_frontend_config()
        self._frontend_config_path = frontend_path

        LOGGER.info("[h15] Frontend config: %s", frontend_path)
        LOGGER.info("[h15] Encoder config: %s", encoder_path)
        LOGGER.info("[h15] Streams: %d", n_streams)

        Gst.init(None)

        sink_parts = []
        for i in range(n_streams):
            if i == 1 and encoder_path:
                # 720p → H264 encode → UDP for viewing
                sink_parts.append(
                    f"src.src_{i} ! queue leaky=downstream max-size-buffers=3 ! "
                    f"hailoencodebin name=enc config-file-path={encoder_path} ! "
                    f"h264parse config-interval=-1 ! "
                    f"rtph264pay ! "
                    f"udpsink host={UDP_HOST} port={UDP_PORT} sync=false"
                )
            elif i == 2:
                if self.ui_state is not None:
                    # FHD@15fps → tee: inference + raw NV12 for web UI (boxes drawn in Python)
                    sink_parts.append(
                        f"src.src_{i} ! queue leaky=downstream max-size-buffers=3 ! "
                        f"tee name=t "
                        f"t. ! queue leaky=downstream max-size-buffers=3 ! "
                        f"appsink name=frame_sink emit-signals=true sync=false drop=true "
                        f"t. ! queue leaky=downstream max-size-buffers=1 ! "
                        f"videoscale ! video/x-raw,width=640,height=360 ! "
                        f"appsink name=raw_sink emit-signals=true sync=false drop=true max-buffers=1"
                    )
                else:
                    # FHD@15fps → appsink for detection only
                    sink_parts.append(
                        f"src.src_{i} ! queue leaky=downstream max-size-buffers=3 ! "
                        f"appsink name=frame_sink emit-signals=true sync=false drop=true"
                    )
            else:
                sink_parts.append(
                    f"src.src_{i} ! queue leaky=downstream max-size-buffers=1 ! "
                    f"fakesink sync=false"
                )

        pipeline_str = (
            f"hailofrontendbinsrc name=src "
            f"config-file-path={frontend_path} "
            + " ".join(sink_parts)
        )
        LOGGER.info("[h15] Pipeline: %s", pipeline_str)
        self.pipeline = Gst.parse_launch(pipeline_str)

        # Connect appsink for inference
        appsink = self.pipeline.get_by_name("frame_sink")
        if appsink:
            appsink.connect("new-sample", self._on_new_sample)

        # Connect raw NV12 appsink for web UI (box drawing + JPEG encode in Python)
        raw_sink = self.pipeline.get_by_name("raw_sink")
        if raw_sink:
            raw_sink.connect("new-sample", self._on_raw_web_sample)

    def _ensure_jpeg_encoder(self, width, height):
        """Create a persistent GStreamer pipeline for NV12→JPEG encoding."""
        if self._jpeg_enc is not None:
            return
        caps = f"video/x-raw,format=NV12,width={width},height={height},framerate=0/1"
        self._jpeg_enc = Gst.parse_launch(
            f"appsrc name=src caps={caps} is-live=true format=time ! "
            f"videoconvert ! jpegenc quality=70 ! "
            f"appsink name=out emit-signals=false sync=false drop=true max-buffers=1"
        )
        self._jpeg_src = self._jpeg_enc.get_by_name("src")
        self._jpeg_out = self._jpeg_enc.get_by_name("out")
        self._jpeg_enc.set_state(Gst.State.PLAYING)
        LOGGER.info("[h15] JPEG encoder pipeline started (%dx%d)", width, height)

    def _on_raw_web_sample(self, appsink):
        """Raw NV12 appsink callback: draw boxes, encode JPEG, push to web UI."""
        sample = appsink.emit("pull-sample")
        if sample is None or self.ui_state is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        caps = sample.get_caps()
        struct = caps.get_structure(0)
        w = struct.get_int("width")[1]
        h = struct.get_int("height")[1]

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK
        try:
            frame = np.frombuffer(map_info.data, dtype=np.uint8).copy()
        finally:
            buf.unmap(map_info)

        # Draw detection boxes on Y plane
        det, _ = self.shared_state.get_latest()
        if det is not None:
            y_plane = frame[:w * h].reshape(h, w)
            bw = int(det.bbox_width * w) if det.bbox_width > 0 else int(det.bbox_height * w * 0.5)
            bh = int(det.bbox_height * h)
            cx, cy = int(det.center_x * w), int(det.center_y * h)
            x1, y1 = max(0, cx - bw // 2), max(0, cy - bh // 2)
            x2, y2 = min(w - 1, cx + bw // 2), min(h - 1, cy + bh // 2)
            t = 2
            y_plane[y1:y1 + t, x1:x2] = 235
            y_plane[max(0, y2 - t):y2, x1:x2] = 235
            y_plane[y1:y2, x1:x1 + t] = 235
            y_plane[y1:y2, max(0, x2 - t):x2] = 235

        # Encode to JPEG via persistent pipeline
        self._ensure_jpeg_encoder(w, h)
        enc_buf = Gst.Buffer.new_allocate(None, len(frame), None)
        enc_buf.fill(0, frame.tobytes())
        self._jpeg_src.emit("push-buffer", enc_buf)

        out_sample = self._jpeg_out.emit("try-pull-sample", int(200 * 1e6))
        if out_sample is not None:
            out_buf = out_sample.get_buffer()
            ok, out_map = out_buf.map(Gst.MapFlags.READ)
            if ok:
                self.ui_state.update_frame(bytes(out_map.data))
                out_buf.unmap(out_map)

        return Gst.FlowReturn.OK

    def _send_detections_udp(self, detections):
        """Send detection data to PC viewer via UDP (port UDP_PORT + 1)."""
        import json as _json
        import socket
        if self._det_sock is None:
            self._det_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        payload = _json.dumps([{
            "cx": d.center_x, "cy": d.center_y,
            "bh": d.bbox_height, "bw": d.bbox_width,
            "conf": d.confidence,
        } for d in detections]).encode()
        try:
            self._det_sock.sendto(payload, (UDP_HOST, UDP_PORT + 1))
        except OSError:
            pass

    def _on_new_sample(self, appsink):
        """Appsink callback: grab frame, run inference, update detections."""
        sample = appsink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        caps = sample.get_caps()
        self._frame_count += 1

        if self._configured_model is None:
            return Gst.FlowReturn.OK

        try:
            self._run_inference(buf, caps)
        except Exception as e:
            if self._frame_count % 30 == 0:
                LOGGER.warning("[h15] Inference error: %s", e)

        return Gst.FlowReturn.OK

    def _run_inference(self, gst_buf, caps):
        """Run YOLOv8 inference on a GStreamer buffer."""
        # Extract frame data
        success, map_info = gst_buf.map(Gst.MapFlags.READ)
        if not success:
            return
        try:
            frame_data = np.frombuffer(map_info.data, dtype=np.uint8).copy()
        finally:
            gst_buf.unmap(map_info)

        # Get model input shape
        input_info = self._infer_model.input()
        input_shape = input_info.shape  # e.g. (384, 640, 3) for RGB or (576, 1024) for NV12

        # Log shapes on first frame for debugging
        if self._frame_count == 1:
            struct = caps.get_structure(0)
            cap_width = struct.get_int("width")[1] if struct.has_field("width") else 0
            cap_height = struct.get_int("height")[1] if struct.has_field("height") else 0
            cap_format = struct.get_string("format") if struct.has_field("format") else "unknown"
            LOGGER.info("[h15] Frame: %d bytes, caps=%dx%d format=%s",
                        len(frame_data), cap_width, cap_height, cap_format)
            LOGGER.info("[h15] Model input: shape=%s (total=%d), format=%s",
                        input_shape, np.prod(input_shape), input_info.format)

        # The appsink frame is NV12 (H*1.5*W bytes). The model expects a specific
        # resolution. We need to extract and resize the frame to match.
        # Parse frame dimensions from caps
        struct = caps.get_structure(0)
        frame_w = struct.get_int("width")[1]
        frame_h = struct.get_int("height")[1]

        # Model input shape is [192, 640, 3]. The HEF name says 384x640 — the "192"
        # is likely 384*1.5/3 = 192, meaning the model expects raw NV12 data reshaped
        # as [H*1.5, W] then viewed as [H*1.5/C, W, C]. Or it may just want the
        # raw NV12 bytes resized to fit.
        #
        # Strategy: resize the NV12 frame to match the model's total byte count,
        # then reshape to the expected input_shape.
        model_bytes = int(np.prod(input_shape))

        # Model expects 192*640*3 = 368640 bytes.
        # For NV12 at 640x384: 640*384*1.5 = 368640 — exact match!
        # So the model wants NV12 at 640x384, stored as [192, 640, 3].
        # NV12 height equivalent = 192 * 3 / 1.5 = 384, width = 640.
        model_w = input_shape[1]  # 640
        model_nv12_h = input_shape[0] * input_shape[2] * 2 // 3  # 192*3*2/3 = 384

        # Resize NV12: resize Y and UV planes separately using nearest-neighbor
        y_h, y_w = frame_h, frame_w
        uv_h, uv_w = frame_h // 2, frame_w

        y_plane = frame_data[:y_h * y_w].reshape(y_h, y_w)
        uv_plane = frame_data[y_h * y_w:].reshape(uv_h, uv_w)

        # Nearest-neighbor resize Y plane
        row_idx_y = (np.arange(model_nv12_h) * y_h // model_nv12_h).astype(int)
        col_idx_y = (np.arange(model_w) * y_w // model_w).astype(int)
        y_resized = y_plane[row_idx_y][:, col_idx_y]

        # Nearest-neighbor resize UV plane
        uv_target_h = model_nv12_h // 2
        row_idx_uv = (np.arange(uv_target_h) * uv_h // uv_target_h).astype(int)
        col_idx_uv = (np.arange(model_w) * uv_w // model_w).astype(int)
        uv_resized = uv_plane[row_idx_uv][:, col_idx_uv]

        # Concatenate Y + UV and reshape to model input shape
        nv12_resized = np.concatenate([y_resized.ravel(), uv_resized.ravel()])
        frame = np.ascontiguousarray(nv12_resized.reshape(input_shape))

        # Create bindings with pre-allocated output buffers
        bindings = self._configured_model.create_bindings()
        bindings.input().set_buffer(frame)

        output_buffers = {}
        for name in self._infer_model.output_names:
            out_shape = self._infer_model.output(name).shape
            output_buffers[name] = np.empty(out_shape, dtype=np.float32)
            bindings.output(name).set_buffer(output_buffers[name])

        self._configured_model.run([bindings], 1000)

        # Extract detections from NMS output
        all_detections = []
        for name in self._infer_model.output_names:
            output = self._infer_model.output(name)
            if output.is_nms:
                # NMS output: parse the flat buffer as per-class detections
                nms_data = output_buffers[name]
                if self._frame_count == 1:
                    LOGGER.info("[h15] NMS output: shape=%s, format=HAILO_NMS_BY_CLASS",
                                nms_data.shape)
                person_dets = extract_person_detections(
                    nms_data, confidence_threshold=DETECTION_CONFIDENCE_THRESHOLD)
                all_detections.extend(person_dets)
            else:
                LOGGER.debug("[h15] Non-NMS output %s — skipping", name)

        # Send detections to PC viewer overlay
        self._send_detections_udp(all_detections)

        # Run tracker for stable IDs
        ui_dets = []
        tracked_ids = set()
        track_by_id = {}  # track_id -> Detection

        if self._byte_tracker is not None and all_detections:
            tracks = self._byte_tracker.update(
                np.array([[d.center_x - d.bbox_width / 2,
                           d.center_y - d.bbox_height / 2,
                           d.center_x + d.bbox_width / 2,
                           d.center_y + d.bbox_height / 2,
                           d.confidence] for d in all_detections]),
                (1, 1),
            )
            for t in tracks:
                x1, y1, x2, y2 = t.tlbr
                tid = int(t.track_id)
                tracked_ids.add(tid)
                det = Detection(
                    label="person",
                    confidence=float(t.score),
                    center_x=(x1 + x2) / 2,
                    center_y=(y1 + y2) / 2,
                    bbox_height=y2 - y1,
                    bbox_width=x2 - x1,
                    timestamp=time.monotonic(),
                )
                track_by_id[tid] = det
                ui_dets.append({
                    "id": tid,
                    "bbox": {"x": x1, "y": y1, "w": x2 - x1, "h": y2 - y1},
                    "confidence": float(t.score),
                    "label": "person",
                })
        else:
            for i, d in enumerate(all_detections):
                tracked_ids.add(i)
                track_by_id[i] = d
                ui_dets.append({
                    "id": i,
                    "bbox": {
                        "x": d.center_x - d.bbox_width / 2,
                        "y": d.center_y - d.bbox_height / 2,
                        "w": d.bbox_width,
                        "h": d.bbox_height,
                    },
                    "confidence": d.confidence,
                    "label": d.label,
                })

        # Pick the detection to follow: selected target > largest
        target_id = self.target_state.get_target() if self.target_state else None
        if target_id is not None and target_id in track_by_id:
            follow_det = track_by_id[target_id]
        elif track_by_id:
            follow_det = max(track_by_id.values(),
                             key=lambda d: d.bbox_height * d.confidence)
        else:
            follow_det = None

        self.shared_state.update(follow_det, available_ids=tracked_ids)

        # Push to web UI
        if self.ui_state is not None:
            following_id = self.target_state.get_target() if self.target_state else None
            self.ui_state.update_detections(ui_dets, following_id=following_id)

        if self._frame_count % 30 == 0:
            LOGGER.info("[h15] frame=%d detections=%d", self._frame_count, len(all_detections))

    # -- Run --

    def run(self):
        """Start the pipeline and block on the GLib main loop."""
        self._setup_inference()
        self._build_pipeline()

        self.loop = GLib.MainLoop()

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        LOGGER.info("[h15] Starting pipeline (UDP stream to %s:%d)", UDP_HOST, UDP_PORT)
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Failed to start H15 pipeline")

        LOGGER.info("[h15] Pipeline running")
        self.loop.run()

    def _on_bus_message(self, bus, msg):
        t = msg.type
        if t == Gst.MessageType.EOS:
            LOGGER.info("[h15] End of stream")
            if self._eos_reached:
                self._eos_reached.set()
            else:
                self.pipeline.set_state(Gst.State.NULL)
                self.loop.quit()
        elif t == Gst.MessageType.ERROR:
            err, debug = msg.parse_error()
            LOGGER.error("[h15] Pipeline error: %s\n  Debug: %s", err.message, debug)
            self.pipeline.set_state(Gst.State.NULL)
            self.loop.quit()

    def stop(self):
        """Stop the pipeline and clean up."""
        if self._jpeg_enc:
            self._jpeg_enc.set_state(Gst.State.NULL)
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        if self.loop:
            self.loop.quit()
        if self._configured_model:
            self._configured_model.__exit__(None, None, None)
        if self._vdevice:
            self._vdevice.release()
        if self._frontend_config_path and self._frontend_config_path.startswith("/tmp"):
            try:
                os.unlink(self._frontend_config_path)
            except OSError:
                pass


# ---------------------------------------------------------------------------
# Factory (matches create_app interface for drone_follow_app.py)
# ---------------------------------------------------------------------------

def create_h15_app(shared_state, target_state=None, eos_reached=None,
                   ui_state=None, parser=None, **kwargs):
    """Create a Hailo15 pipeline app.

    Provides the same interface as pipeline_adapter.create_app() so
    drone_follow_app.py can use it as a drop-in replacement.
    """
    # Parse args from the provided parser (or create a minimal one)
    if parser is not None:
        args = parser.parse_args()
    else:
        import argparse
        args = argparse.Namespace()

    app = Hailo15PipelineApp(
        shared_state=shared_state,
        target_state=target_state,
        ui_state=ui_state,
        eos_reached=eos_reached,
        options_menu=args,
    )
    return app
