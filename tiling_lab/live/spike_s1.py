"""Spike S1: does hailotilecropper_dynamic honour per-frame tile changes?

Run on chip:
    source setup_env.sh
    python -m tiling_lab.live.spike_s1 --video <GT.mp4> --frames 240

Alternates the cropper's `tiles-static` between two disjoint regions every 30
frames and logs, per buffer, the x-centroid of aggregated person detections.
If the centroid follows the active tile region (low while LEFT, high while
RIGHT), mechanism (A) — runtime property re-read — works. The script watches
the bus for caps-renegotiation errors / stalls.

If (A) fails (centroid does not move), this is the place to add mechanism (B):
attach hailo.HailoTileROI objects to each buffer's HailoROI from an upstream
`identity signal-handoffs=true` / pad probe (the plugin doc states static tiles
are *appended to dynamic tiles read from the buffer's HailoROI on every frame*).
"""
import argparse
import subprocess
import sys

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib  # noqa: E402
import hailo  # noqa: E402

from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
    SOURCE_PIPELINE, INFERENCE_PIPELINE,
)

# Concrete, on-disk resources (TILING_* defaults in hailo_apps defines.py).
DEFAULT_HEF = "/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef"
DEFAULT_SO = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"
DEFAULT_FUNC = "filter"
DEFAULT_LABELS = "/usr/local/hailo/resources/json/hailo_4_classes.json"

TILE_LEFT = "0.0,0.0,0.4,0.3,s"
TILE_RIGHT = "0.6,0.7,0.4,0.3,s"


def probe_dims(video: str) -> tuple[int, int]:
    """Return (width, height) of the video via ffprobe."""
    out = subprocess.check_output([
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=width,height", "-of", "csv=p=0:s=x", video,
    ]).decode().strip()
    w, h = out.split("x")
    return int(w), int(h)


def build_pipeline(video, hef, post_so, func, labels, w, h):
    inner = INFERENCE_PIPELINE(hef_path=hef, post_process_so=post_so,
                               post_function_name=func, batch_size=1,
                               config_json=labels, name="s1_infer")
    src = SOURCE_PIPELINE(video_source=video, video_width=w, video_height=h,
                          frame_rate=30, sync=False)
    return (
        f"{src} ! queue name=s1_in_q ! "
        f"hailotilecropper_dynamic name=tc internal-offset=true "
        f"tiling-mode=single-scale tiles-static=\"{TILE_LEFT}\" "
        f"hailotileaggregator name=agg flatten-detections=true iou-threshold=0.3 "
        f"tc. ! queue name=s1_bypass_q ! agg.sink_0 "
        f"tc. ! video/x-raw,format=RGB ! {inner} ! agg.sink_1 "
        f"agg. ! queue name=s1_out_q ! fakesink sync=false name=s1_sink"
    )


def main():
    global TILE_LEFT, TILE_RIGHT
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--frames", type=int, default=240)
    ap.add_argument("--hef", default=DEFAULT_HEF)
    ap.add_argument("--post-so", default=DEFAULT_SO)
    ap.add_argument("--func", default=DEFAULT_FUNC)
    ap.add_argument("--labels", default=DEFAULT_LABELS)
    ap.add_argument("--left", default=TILE_LEFT, help="tiles-static for region L")
    ap.add_argument("--right", default=TILE_RIGHT, help="tiles-static for region R")
    ap.add_argument("--all-labels", action="store_true",
                    help="print a histogram of ALL detection labels (debug)")
    args = ap.parse_args()
    TILE_LEFT, TILE_RIGHT = args.left, args.right

    w, h = probe_dims(args.video)
    print(f"[S1] video dims {w}x{h}", flush=True)

    Gst.init(None)
    pipeline = Gst.parse_launch(
        build_pipeline(args.video, args.hef, args.post_so, args.func,
                       args.labels, w, h))
    cropper = pipeline.get_by_name("tc")
    agg = pipeline.get_by_name("agg")

    state = {"frame": 0, "region": "L"}
    loop = GLib.MainLoop()

    def probe(pad, info):
        buf = info.get_buffer()
        roi = hailo.get_roi_from_buffer(buf)
        dets = roi.get_objects_typed(hailo.HAILO_DETECTION)
        cxs = []
        hist = {}
        for d in dets:
            lbl = d.get_label()
            hist[lbl] = hist.get(lbl, 0) + 1
            b = d.get_bbox()
            cxs.append(b.xmin() + b.width() / 2.0)  # centroid over ALL dets
        f = state["frame"]
        if args.all_labels and f < 10:
            print(f"[S1] frame {f} ALL labels={hist} total_dets={len(dets)}",
                  flush=True)
        if f > 0 and f % 30 == 0:
            state["region"] = "R" if state["region"] == "L" else "L"
            tiles = TILE_RIGHT if state["region"] == "R" else TILE_LEFT
            cropper.set_property("tiles-static", tiles)  # mechanism (A)
            print(f"[S1] frame {f}: set tiles-static -> {state['region']} ({tiles})",
                  flush=True)
        mean_cx = sum(cxs) / len(cxs) if cxs else float("nan")
        print(f"[S1] frame {f} region={state['region']} "
              f"persons={len(cxs)} mean_cx={mean_cx:.3f}", flush=True)
        state["frame"] += 1
        if state["frame"] >= args.frames:
            loop.quit()
        return Gst.PadProbeReturn.OK

    agg.get_static_pad("src").add_probe(Gst.PadProbeType.BUFFER, probe)

    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_msg(_bus, msg):
        t = msg.type
        if t == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print(f"[S1][BUS-ERROR] {err}: {dbg}", flush=True)
            loop.quit()
        elif t == Gst.MessageType.EOS:
            print("[S1] EOS", flush=True)
            loop.quit()
        return True

    bus.connect("message", on_msg)
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    sys.exit(main())
