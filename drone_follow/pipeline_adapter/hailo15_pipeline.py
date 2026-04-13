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
from .byte_tracker import ByteTracker

LOGGER = logging.getLogger("drone_follow.app")

# ---------------------------------------------------------------------------
# Configuration defaults
# ---------------------------------------------------------------------------
MEDIALIB_TOP_CONFIG = "/etc/imaging/cfg/medialib_configs/ai_example_medialib_config.json"
YOLO_HEF_CANDIDATES = [
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
    if not os.path.isfile(MEDIALIB_TOP_CONFIG):
        raise FileNotFoundError(
            f"{MEDIALIB_TOP_CONFIG} not found. Run setup_hailo_sensor first.")

    with open(MEDIALIB_TOP_CONFIG) as f:
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

        # ByteTracker for consistent IDs
        self._byte_tracker = ByteTracker(
            track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=15,
        )

        # pyhailort inference objects (initialized in _setup_inference)
        self._vdevice = None
        self._configured_model = None
        self._infer_model = None

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
        from hailo_platform import VDevice, FormatType

        hef_path = _find_yolo_hef()
        LOGGER.info("[h15] Loading HEF: %s", hef_path)

        self._vdevice = VDevice()
        self._infer_model = self._vdevice.create_infer_model(hef_path)

        # Request float32 output for easy post-processing
        for output in self._infer_model.outputs:
            output.set_format_type(FormatType.FLOAT32)

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
                # FHD@15fps → appsink for detection
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

        Gst.init(None)
        self.pipeline = Gst.parse_launch(pipeline_str)

        # Connect appsink
        appsink = self.pipeline.get_by_name("frame_sink")
        if appsink:
            appsink.connect("new-sample", self._on_new_sample)

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
            frame_data = np.frombuffer(bytes(map_info.data), dtype=np.uint8)
        finally:
            gst_buf.unmap(map_info)

        # Get model input shape and resize/reshape frame
        input_info = self._infer_model.input()
        input_shape = input_info.shape  # e.g. (384, 640, 3) or similar

        # The frame from appsink is NV12. The model may expect a different format.
        # For now, resize the NV12 Y plane or convert as needed.
        # pyhailort handles format conversion internally if input format matches.
        frame = np.ascontiguousarray(frame_data[:np.prod(input_shape)].reshape(input_shape))

        # Create bindings and run
        bindings = self._configured_model.create_bindings()
        bindings.input().set_buffer(frame)

        output_buffers = {}
        for name in self._infer_model.output_names:
            out_shape = self._infer_model.output(name).shape
            output_buffers[name] = np.empty(out_shape, dtype=np.float32)
            bindings.output(name).set_buffer(output_buffers[name])

        self._configured_model.run([bindings], timeout_ms=1000)

        # Extract detections from NMS output
        all_detections = []
        for name in self._infer_model.output_names:
            output = self._infer_model.output(name)
            if output.is_nms:
                nms_result = bindings.output(name).get_buffer()
                person_dets = extract_person_detections(
                    nms_result, confidence_threshold=DETECTION_CONFIDENCE_THRESHOLD)
                all_detections.extend(person_dets)
            else:
                LOGGER.debug("[h15] Non-NMS output %s, shape %s — skipping",
                             name, output_buffers[name].shape)

        # Update shared state with best detection
        if all_detections:
            best = max(all_detections, key=lambda d: d.bbox_height * d.confidence)
            self.shared_state.update(best, available_ids=set())
        else:
            self.shared_state.update(None, available_ids=set())

        if self._frame_count % 30 == 0:
            LOGGER.debug("[h15] frame=%d detections=%d", self._frame_count, len(all_detections))

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
