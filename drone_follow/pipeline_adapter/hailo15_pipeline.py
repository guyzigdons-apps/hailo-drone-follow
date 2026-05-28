"""Hailo15 pipeline adapter — camera + inference using native H15 GStreamer + pyhailort.

Replaces the RPi/x86 hailo_apps tiling pipeline for Hailo15H/15L SoCs.

Pipeline architecture:
  hailofrontendbinsrc (ISP camera) →
    sink0 (4K@30fps)   → hailoencodebin → H264 → matroskamux → file (when --record)
                       → fakesink                                  (otherwise)
    sink1 (720p)       → fakesink (unused)
    sink2 (FHD@15fps)  → appsink → pyhailort inference → detections → SharedDetectionState
                                 → JPEG (640x360, 15fps) → web UI MJPEG stream

The web UI (port 5001) is the live monitoring surface. There's no UDP H264
stream out anymore — the VPU encoder budget is reserved for the 4K recording.
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

    # Find encoder config for sink0 (4K recording)
    encoder_config_path = None
    for es in profile_config.get("encoded_output_streams", []):
        if es.get("stream_id") == "sink0":
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
                 eos_reached=None, options_menu=None, record_path=None,
                 replay_path=None, replay_loop=False):
        self.shared_state = shared_state
        self.target_state = target_state
        self._record_path = record_path
        self._replay_path = replay_path  # if set, use file source instead of ISP
        self._replay_loop = replay_loop  # seek back to start on EOS
        self._rec_first_pts = None  # for PTS normalization on recording branch
        self._det_log = None         # detections sidecar file (jsonl)
        # Diagnostic: track which source PTS each branch processed
        self._last_inference_pts = None
        self._last_jpeg_pts = None
        self._det_log_t0 = None      # wall-clock time of first detection write
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
            # ByteTracker hardcodes det_thresh = track_thresh + 0.1 internally,
            # so new tracks need confidence ≥ 0.5 to be created. On replay (H264
            # decoded source) confidences run lower than live ISP frames, leaving
            # the tracker unable to re-establish a lost track from raw NMS hits
            # in the 0.4–0.5 range. Drop the +0.1 buffer so new tracks can be
            # created at the same threshold we accept for NMS.
            self._byte_tracker.det_thresh = 0.4


        # pyhailort inference objects (initialized in _setup_inference)
        self._vdevice = None
        self._configured_model = None
        self._infer_model = None

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
        Gst.init(None)
        if self._replay_path:
            self._build_replay_pipeline()
            return
        frontend_path, encoder_path, n_streams, app_settings = _resolve_frontend_config()
        self._frontend_config_path = frontend_path

        LOGGER.info("[h15] Frontend config: %s", frontend_path)
        LOGGER.info("[h15] Encoder config: %s", encoder_path)
        LOGGER.info("[h15] Streams: %d", n_streams)

        sink_parts = []
        for i in range(n_streams):
            if i == 0 and self._record_path and encoder_path:
                # 4K → H264 encode → matroskamux → file. The h264parse is named
                # so we can attach a pad probe to normalize PTS; hailoencodebin
                # emits buffers with absolute system PTS and players would see
                # invalid durations if we wrote that to the container directly.
                sink_parts.append(
                    f"src.src_{i} ! queue leaky=downstream max-size-buffers=3 ! "
                    f"hailoencodebin name=enc config-file-path={encoder_path} ! "
                    f"queue leaky=downstream max-size-buffers=200 ! "
                    f"h264parse name=rec_h264parse config-interval=1 ! "
                    f"matroskamux ! "
                    f"filesink location={self._record_path} sync=false"
                )
                LOGGER.info("[h15] Recording 4K H264 to %s", self._record_path)
            elif i == 2:
                # FHD@15fps → appsink for detection. JPEG for web UI is encoded
                # inside the inference callback so the JPEG and detections come
                # from the same frame (atomic pairing).
                sink_parts.append(
                    f"src.src_{i} ! queue leaky=downstream max-size-buffers=3 ! "
                    f"appsink name=frame_sink emit-signals=true sync=false drop=true"
                )
            else:
                # Unused streams (sink_0 when not recording, sink_1 always) still
                # need a downstream sink because the ISP frontend emits them.
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

        # Note: web UI JPEG is now encoded inside the inference callback,
        # not in a separate raw_sink branch, to keep JPEG and detections
        # atomically paired (same source frame).

        # Attach PTS-normalization pad probe to the recording branch
        if self._record_path:
            rec_parse = self.pipeline.get_by_name("rec_h264parse")
            if rec_parse:
                self._rec_first_pts = None
                src_pad = rec_parse.get_static_pad("src")
                src_pad.add_probe(Gst.PadProbeType.BUFFER, self._on_record_probe)

    def _build_replay_pipeline(self):
        """Build a pipeline that replays a recorded .mkv through inference.

        No ISP, no encoder, no UDP, no recording branches. Just decode the file,
        feed NV12 frames into the inference appsink (and optionally the web UI
        raw appsink), and let the existing callbacks do the rest.
        """
        if not os.path.isfile(self._replay_path):
            raise FileNotFoundError(f"Replay file not found: {self._replay_path}")
        LOGGER.info("[h15] Replay mode: %s", self._replay_path)

        # Single sink — inference. JPEG for web UI is encoded inside the
        # inference callback (atomic pairing with detections).
        # An `identity sync=true` element throttles the pipeline to the
        # source PTS rate so replay runs at native speed instead of as
        # fast as the decoder can pump.
        pipeline_str = (
            f"filesrc location={self._replay_path} ! "
            f"matroskademux ! h264parse ! avdec_h264 ! "
            f"videoconvert ! video/x-raw,format=NV12 ! "
            f"identity sync=true ! "
            f"queue leaky=downstream max-size-buffers=3 ! "
            f"appsink name=frame_sink emit-signals=true sync=false drop=true"
        )
        LOGGER.info("[h15] Replay pipeline: %s", pipeline_str)
        self.pipeline = Gst.parse_launch(pipeline_str)

        appsink = self.pipeline.get_by_name("frame_sink")
        if appsink:
            appsink.connect("new-sample", self._on_new_sample)

    def _write_detection_log(self, detections, following_id, raw_count=None, raw_max_conf=None):
        """Append a detection record (JSONL) with timestamp relative to record start."""
        import json as _json
        if self._det_log is None:
            log_path = self._record_path.rsplit(".", 1)[0] + ".jsonl"
            self._det_log = open(log_path, "w", buffering=1)  # line-buffered
            self._det_log_t0 = time.monotonic()
            LOGGER.info("[h15] Detection log: %s", log_path)
        t_rel = time.monotonic() - self._det_log_t0
        rec = {"t": round(t_rel, 4), "following_id": following_id, "detections": detections}
        if raw_count is not None:
            rec["raw_count"] = raw_count  # how many NMS detections before tracker
        if raw_max_conf is not None:
            rec["raw_max_conf"] = round(raw_max_conf, 3)  # peak pre-tracker confidence
        self._det_log.write(_json.dumps(rec) + "\n")

    def _on_record_probe(self, pad, info):
        """Pad probe: rewrite buffer PTS/DTS so the recording starts at 0.

        hailoencodebin emits buffers timestamped with absolute system time.
        We use the first DTS as the base (DTS is monotonic in encoded order)
        and clamp any value that would go negative — never drop buffers,
        since dropping reference frames breaks the H264 decode chain.
        """
        buf = info.get_buffer()
        if buf is None:
            return Gst.PadProbeReturn.OK
        pts = buf.pts
        dts = buf.dts
        # Prefer DTS as the base (monotonic). Fall back to PTS if no DTS.
        ref = dts if dts != Gst.CLOCK_TIME_NONE else pts
        if ref == Gst.CLOCK_TIME_NONE:
            return Gst.PadProbeReturn.OK

        if self._rec_first_pts is None:
            self._rec_first_pts = ref
            LOGGER.info("[h15] Recording timestamp base = %d ns", ref)
        base = self._rec_first_pts

        # Subtract base; clamp to 0 (don't drop, to preserve reference chain)
        if pts != Gst.CLOCK_TIME_NONE:
            buf.pts = max(0, pts - base)
        if dts != Gst.CLOCK_TIME_NONE:
            buf.dts = max(0, dts - base)
        return Gst.PadProbeReturn.OK

    _UI_JPEG_W = 640
    _UI_JPEG_H = 360

    def _encode_jpeg_for_ui(self, frame_data, src_w, src_h):
        """Downscale source NV12 to 640x360 and JPEG-encode it.

        Called from the inference callback so the JPEG is from the same frame
        as the detections we just computed — pairs them atomically.
        """
        y_plane = frame_data[:src_h * src_w].reshape(src_h, src_w)
        uv_plane = frame_data[src_h * src_w:src_h * src_w + (src_h // 2) * src_w].reshape(src_h // 2, src_w)

        dst_w, dst_h = self._UI_JPEG_W, self._UI_JPEG_H
        # Nearest-neighbor NV12 resize (same approach as model-input resize)
        row_idx_y = (np.arange(dst_h) * src_h // dst_h).astype(int)
        col_idx_y = (np.arange(dst_w) * src_w // dst_w).astype(int)
        y_resized = y_plane[row_idx_y][:, col_idx_y]
        uv_target_h = dst_h // 2
        row_idx_uv = (np.arange(uv_target_h) * (src_h // 2) // uv_target_h).astype(int)
        col_idx_uv = (np.arange(dst_w) * src_w // dst_w).astype(int)
        uv_resized = uv_plane[row_idx_uv][:, col_idx_uv]
        nv12_resized = np.concatenate([y_resized.ravel(), uv_resized.ravel()])

        self._ensure_jpeg_encoder(dst_w, dst_h)
        enc_buf = Gst.Buffer.new_allocate(None, len(nv12_resized), None)
        enc_buf.fill(0, nv12_resized.tobytes())
        self._jpeg_src.emit("push-buffer", enc_buf)
        out_sample = self._jpeg_out.emit("try-pull-sample", int(200 * 1e6))
        if out_sample is None:
            return None
        out_buf = out_sample.get_buffer()
        ok, out_map = out_buf.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            return bytes(out_map.data)
        finally:
            out_buf.unmap(out_map)

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
        # Diagnostic: record the source PTS of this frame
        if buf.pts != Gst.CLOCK_TIME_NONE:
            self._last_jpeg_pts = buf.pts
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

        # Note: no server-side box drawing — the React SVG overlay handles
        # all detection boxes/crosses. Drawing here would duplicate them.

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
        # Diagnostic: record the source PTS of this frame
        if gst_buf.pts != Gst.CLOCK_TIME_NONE:
            self._last_inference_pts = gst_buf.pts
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
                # Tracker's tlbr is a Kalman-filtered (predicted) position.
                # For UI display we use the raw NMS detection bbox (matches
                # the video pixels exactly), but keep the tracker's stable ID.
                # Match by best IoU between the tracker's tlbr and raw detections.
                tx1, ty1, tx2, ty2 = t.tlbr
                best_iou, best_det = 0.0, None
                for d in all_detections:
                    dx1 = d.center_x - d.bbox_width / 2
                    dy1 = d.center_y - d.bbox_height / 2
                    dx2 = d.center_x + d.bbox_width / 2
                    dy2 = d.center_y + d.bbox_height / 2
                    ix1 = max(tx1, dx1); iy1 = max(ty1, dy1)
                    ix2 = min(tx2, dx2); iy2 = min(ty2, dy2)
                    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
                    union = ((tx2 - tx1) * (ty2 - ty1) +
                             (dx2 - dx1) * (dy2 - dy1) - inter)
                    iou = inter / union if union > 0 else 0.0
                    if iou > best_iou:
                        best_iou, best_det = iou, d
                tid = int(t.track_id)
                tracked_ids.add(tid)
                # Use the matched raw detection bbox if IoU is reasonable;
                # otherwise fall back to the tracker's predicted bbox (the
                # observation was lost this frame, prediction is all we have).
                if best_det is not None and best_iou > 0.2:
                    raw_d = best_det
                    x1 = raw_d.center_x - raw_d.bbox_width / 2
                    y1 = raw_d.center_y - raw_d.bbox_height / 2
                    bw = raw_d.bbox_width
                    bh = raw_d.bbox_height
                else:
                    x1, y1 = tx1, ty1
                    bw, bh = tx2 - tx1, ty2 - ty1
                det = Detection(
                    label="person",
                    confidence=float(t.score),
                    center_x=x1 + bw / 2,
                    center_y=y1 + bh / 2,
                    bbox_height=bh,
                    bbox_width=bw,
                    timestamp=time.monotonic(),
                )
                track_by_id[tid] = det
                ui_dets.append({
                    "id": tid,
                    "bbox": {"x": x1, "y": y1, "w": bw, "h": bh},
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
            # Encode JPEG from THIS frame and push atomically with detections,
            # so the bbox always matches the displayed video frame.
            jpeg = self._encode_jpeg_for_ui(frame_data, frame_w, frame_h)
            # Diagnostic: tag detections with the PTS of both branches
            self._last_jpeg_pts = self._last_inference_pts  # now same frame
            ui_dets.append({
                "_diag": True,
                "inference_pts_ns": self._last_inference_pts,
                "jpeg_pts_ns": self._last_jpeg_pts,
            })
            self.ui_state.update_detections(ui_dets, following_id=following_id)
            if jpeg is not None:
                self.ui_state.update_frame(jpeg)

        # Write detections to sidecar file for post-flight overlay.
        # Strip the _diag entry — it has no bbox and would crash overlay scripts.
        # Include the raw-NMS detection count so we can tell whether a missing
        # entry is "model didn't detect" vs "tracker dropped it".
        if self._record_path:
            real_dets = [d for d in ui_dets if not d.get("_diag")]
            raw_max_conf = max((d.confidence for d in all_detections), default=0.0)
            self._write_detection_log(
                real_dets,
                self.target_state.get_target() if self.target_state else None,
                raw_count=len(all_detections),
                raw_max_conf=raw_max_conf,
            )

        if self._frame_count % 30 == 0:
            LOGGER.info("[h15] frame=%d raw=%d tracked=%d", self._frame_count,
                        len(all_detections), len(tracked_ids))

    # -- Run --

    def run(self):
        """Start the pipeline and block on the GLib main loop."""
        self._setup_inference()
        self._build_pipeline()

        self.loop = GLib.MainLoop()

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        LOGGER.info("[h15] Starting pipeline")
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Failed to start H15 pipeline")

        LOGGER.info("[h15] Pipeline running")
        self.loop.run()

    def _on_bus_message(self, bus, msg):
        t = msg.type
        if t == Gst.MessageType.EOS:
            if self._replay_path and self._replay_loop:
                LOGGER.info("[h15] End of replay — rebuilding pipeline for loop")
                # Full rebuild — seek/state-cycle approaches don't reliably
                # recover avdec_h264 from EOS state. A fresh pipeline always
                # works.
                self.pipeline.set_state(Gst.State.NULL)
                self._build_replay_pipeline()
                bus = self.pipeline.get_bus()
                bus.add_signal_watch()
                bus.connect("message", self._on_bus_message)
                self.pipeline.set_state(Gst.State.PLAYING)
                return
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
        """Stop the pipeline and clean up. Sends EOS first if recording so the
        muxer finalizes; otherwise tears down immediately so the drone thread
        has full time to land.
        """
        if self.pipeline:
            if self._record_path:
                # Recording: need EOS for matroskamux to write Duration/Cues
                LOGGER.info("[h15] Sending EOS, waiting for pipeline to flush...")
                self.pipeline.send_event(Gst.Event.new_eos())
                bus = self.pipeline.get_bus()
                msg = bus.timed_pop_filtered(
                    3 * Gst.SECOND,
                    Gst.MessageType.EOS | Gst.MessageType.ERROR
                )
                if msg is None:
                    LOGGER.warning("[h15] EOS not received within 3s — recording may be incomplete")
                else:
                    LOGGER.info("[h15] Pipeline flushed cleanly")
            self.pipeline.set_state(Gst.State.NULL)
        if self._jpeg_enc:
            self._jpeg_enc.set_state(Gst.State.NULL)
        if self._det_log:
            try:
                self._det_log.close()
            except OSError:
                pass
            self._det_log = None
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
                   ui_state=None, parser=None, record_path=None,
                   replay_path=None, replay_loop=False, **kwargs):
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
        record_path=record_path,
        replay_path=replay_path,
        replay_loop=replay_loop,
        options_menu=args,
    )
    return app
