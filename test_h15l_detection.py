#!/usr/bin/env python3
"""
Proof-of-concept: camera pipeline on Hailo15H/15L using GStreamer.

Run setup_hailo_sensor first, then:
    python3 test_h15l_detection.py

View on PC (10.0.0.2):
    gst-launch-1.0 udpsrc port=5002 ! application/x-rtp,encoding-name=H264 ! rtph264depay ! decodebin ! autovideosink

Phase 1: Verify camera frames flow through the GStreamer pipeline.
Phase 2 (after pyhailort is in the image): Add NPU inference.
"""

import json
import os
import sys
import signal
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
MEDIALIB_TOP_CONFIG = "/etc/imaging/cfg/medialib_configs/ai_example_medialib_config.json"
YOLO_HEF_CANDIDATES = [
    "/home/root/apps/ai_example_app/resources/hailo_yolov8s_384_640.hef",
    "/home/root/apps/ai_example_app/resources/hailo_yolov8n_384_640.hef",
]

# ---------------------------------------------------------------------------
# Callbacks
# ---------------------------------------------------------------------------
frame_count = 0

def on_new_sample(appsink):
    """Pull a sample from appsink and report frame info."""
    global frame_count
    sample = appsink.emit("pull-sample")
    if sample is None:
        return Gst.FlowReturn.OK

    buf = sample.get_buffer()
    caps = sample.get_caps()
    frame_count += 1

    if frame_count == 1:
        print(f"\n=== First frame received! ===")
        print(f"Caps: {caps.to_string()}")
        print(f"Buffer size: {buf.get_size()} bytes")
        print(f"PTS: {buf.pts}\n")

    if frame_count % 30 == 0:
        print(f"[frame {frame_count}] size={buf.get_size()}, pts={buf.pts}")

    return Gst.FlowReturn.OK


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    Gst.init(None)

    # List available Hailo GStreamer elements
    print("Registered Hailo GStreamer elements:")
    registry = Gst.Registry.get()
    hailo_elements = []
    for plugin_name in ["hailo", "medialib", "hailotools"]:
        features = registry.get_feature_list_by_plugin(plugin_name)
        if features:
            print(f"  Plugin '{plugin_name}':")
            for feat in features:
                print(f"    - {feat.get_name()}")
                hailo_elements.append(feat.get_name())

    if "hailofrontendbinsrc" not in hailo_elements:
        print("\nERROR: hailofrontendbinsrc not available. Is libmedialib installed?")
        sys.exit(1)

    # Resolve the frontend config from the medialib config chain:
    #   medialib_config.json -> profile -> application_settings.json
    # hailofrontendbinsrc needs application_settings.json (has application_input_streams)
    if not os.path.isfile(MEDIALIB_TOP_CONFIG):
        print(f"ERROR: {MEDIALIB_TOP_CONFIG} not found.")
        print("Run setup_hailo_sensor first.")
        sys.exit(1)

    print(f"\nMedialib config: {MEDIALIB_TOP_CONFIG}")
    frontend_config_path = None
    try:
        with open(MEDIALIB_TOP_CONFIG) as f:
            top_config = json.load(f)
        arch = top_config.get("metadata", {}).get("architecture", "unknown")
        default_profile = top_config.get("default_profile", "Daylight")
        print(f"  Architecture: {arch}")
        print(f"  Default profile: {default_profile}")

        # Find the default profile's config file
        profile_config_path = None
        for p in top_config.get("profiles", []):
            print(f"  Profile: {p['name']} -> {p['config_file']}")
            if p["name"] == default_profile:
                profile_config_path = p["config_file"]

        if profile_config_path and os.path.isfile(profile_config_path):
            with open(profile_config_path) as f:
                profile_config = json.load(f)
            app_settings_path = profile_config.get("application_settings")
            if app_settings_path and os.path.isfile(app_settings_path):
                print(f"  Application settings: {app_settings_path}")
            else:
                print(f"  WARNING: application_settings not found: {app_settings_path}")
                app_settings_path = None

            # Find encoder config for sink1 (720p stream)
            encoder_config_path = None
            for es in profile_config.get("encoded_output_streams", []):
                if es.get("stream_id") == "sink1":
                    encoder_config_path = es.get("encoding")
                    break
            if encoder_config_path and os.path.isfile(encoder_config_path):
                print(f"  Encoder config (sink1): {encoder_config_path}")
            else:
                print(f"  WARNING: encoder config for sink1 not found")
                encoder_config_path = None
        else:
            print(f"  WARNING: profile config not found: {profile_config_path}")
            app_settings_path = None
    except Exception as e:
        print(f"  Error reading config chain: {e}")
        app_settings_path = None

    if not app_settings_path:
        print("ERROR: Could not resolve application_settings.json")
        sys.exit(1)

    # hailofrontendbinsrc needs the full frontend_config_t which includes
    # dewarp, denoise, isp, hdr, input_video, etc. The application_settings.json
    # only has app-level fields. Merge with required defaults.
    with open(app_settings_path) as f:
        app_settings = json.load(f)

    # Get input resolution from first stream for input_video config
    first_stream = app_settings["application_input_streams"]["resolutions"][0]

    # hailofrontendbinsrc validates ALL sub-fields even when features are disabled.
    # Provide complete defaults matching the frontend_config_t schema.
    FRONTEND_DEFAULTS = {
        "input_video": {
            "resolution": {
                "width": first_stream["width"],
                "height": first_stream["height"],
                "framerate": first_stream["framerate"],
            }
        },
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
                "vsm": {
                    "hoffset": 0, "voffset": 0,
                    "width": 1920, "height": 1080,
                    "max_displacement": 64,
                },
            },
            "debug": {
                "generate_resize_grid": False,
                "fix_stabilization": False,
                "fix_stabilization_longitude": 0.0,
                "fix_stabilization_latitude": 0.0,
            },
        },
        "eis": {
            "enabled": False,
            "stabilize": True,
            "eis_config_path": "",
            "window_size": 10,
            "rotational_smoothing_coefficient": 0.0,
            "iir_hpf_coefficient": 0.997,
            "camera_fov_factor": 0.85,
            "line_readout_time": 7410,
            "hdr_exposure_ratio": 0.2,
            "min_angle_deg": 0.005,
            "max_angle_deg": 6.0,
        },
        "gyro": {
            "enabled": False,
            "sensor_name": "",
            "sensor_frequency": "0",
            "scale": 0.0,
        },
        "isp": {"isp_config_files_path": "/usr/bin"},
        "hdr": {"enabled": False, "dol": 2},
        "denoise": {
            "enabled": False,
            "sensor": "imx675",
            "method": "HIGH_QUALITY",
            "loopback-count": 1,
            "network": {
                "network_path": "",
                "y_channel": "", "uv_channel": "",
                "feedback_y_channel": "", "feedback_uv_channel": "",
                "output_y_channel": "", "output_uv_channel": "",
            },
        },
    }

    # Merge: app_settings fields take priority, defaults fill in the rest
    frontend_full = {**FRONTEND_DEFAULTS, **app_settings}
    # Remove metadata/version fields that aren't part of frontend_config_t
    frontend_full.pop("metadata", None)
    frontend_full.pop("version", None)

    # Write merged config to a temp file
    import tempfile
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".json", prefix="frontend_", delete=False)
    json.dump(frontend_full, tmp, indent=2)
    tmp.close()
    frontend_config_path = tmp.name
    print(f"  Generated frontend config: {frontend_config_path}")

    # Find HEF model
    hef_path = None
    for c in YOLO_HEF_CANDIDATES:
        if os.path.isfile(c):
            hef_path = c
            break
    if hef_path:
        print(f"YOLOv8 HEF: {hef_path}")
    else:
        print("Warning: No YOLOv8 HEF found (inference will be added later)")

    # Build pipeline (camera only — no hailonet on SoC)
    # hailofrontendbinsrc produces multiple source pads (one per stream in config).
    # We must link ALL pads or the element blocks. Use fakesink for unused streams
    # and appsink for the stream we want (sink1 = 1280x720 is good for detection).
    n_streams = len(app_settings["application_input_streams"]["resolutions"])
    print(f"  Streams configured: {n_streams}")
    for i, s in enumerate(app_settings["application_input_streams"]["resolutions"]):
        print(f"    [{i}] {s.get('stream_id', f'sink{i}')}: "
              f"{s['width']}x{s['height']}@{s['framerate']}fps")

    # Stream layout (matching ai_example_app):
    #   sink0 (full res) → fakesink (unused)
    #   sink1 (720p)     → tee → appsink (frame callback) + H264 encode → UDP
    #   sink2+ (others)  → fakesink (unused)
    if not encoder_config_path:
        print("ERROR: Cannot stream without encoder config for sink1")
        sys.exit(1)

    sink_parts = []
    for i in range(n_streams):
        if i == 1:  # 720p stream: encode → UDP + appsink for detection
            sink_parts.append(
                f"src.src_{i} ! queue leaky=downstream max-size-buffers=3 ! "
                f"hailoencodebin name=enc config-file-path={encoder_config_path} ! "
                f"h264parse config-interval=-1 ! "
                f"rtph264pay ! "
                f"udpsink host=10.0.0.2 port=5002 sync=false"
            )
        elif i == 2:  # Use another stream for detection appsink
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
        f"config-file-path={frontend_config_path} "
        + " ".join(sink_parts)
    )

    print(f"\nStreaming H264 to 10.0.0.2:5002")
    print(f"View on PC:")
    print(f"  gst-launch-1.0 udpsrc port=5002 ! "
          f"\"application/x-rtp,encoding-name=H264\" ! "
          f"rtph264depay ! decodebin ! autovideosink")
    print(f"\nPipeline:\n  {pipeline_str}\n")

    try:
        pipeline = Gst.parse_launch(pipeline_str)
    except GLib.Error as e:
        print(f"Failed to create pipeline: {e}")
        sys.exit(1)

    # Connect appsink
    appsink = pipeline.get_by_name("frame_sink")
    appsink.connect("new-sample", on_new_sample)

    # Main loop
    loop = GLib.MainLoop()

    def on_signal(*_):
        print(f"\nStopping... ({frame_count} frames received)")
        pipeline.set_state(Gst.State.NULL)
        loop.quit()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_message(bus, msg):
        t = msg.type
        if t == Gst.MessageType.EOS:
            print("End of stream")
            pipeline.set_state(Gst.State.NULL)
            loop.quit()
        elif t == Gst.MessageType.ERROR:
            err, debug = msg.parse_error()
            print(f"Pipeline error: {err.message}")
            if debug:
                print(f"  Debug: {debug}")
            pipeline.set_state(Gst.State.NULL)
            loop.quit()
        elif t == Gst.MessageType.STATE_CHANGED:
            if msg.src == pipeline:
                old, new, pending = msg.parse_state_changed()
                print(f"Pipeline state: {old.value_nick} -> {new.value_nick}")

    bus.connect("message", on_message)

    print("Starting pipeline...")
    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        print("Failed to start pipeline")
        sys.exit(1)

    print("Pipeline running. Press Ctrl+C to stop.\n")
    loop.run()
    # Clean up temp config
    if frontend_config_path and frontend_config_path.startswith("/tmp"):
        try:
            os.unlink(frontend_config_path)
        except OSError:
            pass

    print(f"Done. Total frames: {frame_count}")


if __name__ == "__main__":
    main()
