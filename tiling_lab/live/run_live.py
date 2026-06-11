"""Run dynamic tiling live on a video through the real GStreamer pipeline.

    source setup_env.sh
    python -m tiling_lab.live.run_live \
        --video tiling_visualizer_site/dist/data/videos/0025_fov50.mp4 \
        --out tiling_lab/runs/live_0025_fov50

Pipeline: filesrc -> decodebin -> hailotilecropper_dynamic -> hailonet infer
-> hailotileaggregator -> overlay -> x264 -> mkv. A buffer probe on the
aggregator src steps the DynamicTilingController with this frame's person
detections and pushes the returned tiles-static string onto the cropper for the
NEXT frame (one cropping-period latency, confirmed by spike S1). Saves an
overlay .mkv and a per-frame tiles JSONL so the run is verifiable headless.
"""
import argparse
import json
import os
import subprocess
import sys

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib  # noqa: E402
import hailo  # noqa: E402

from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
    SOURCE_PIPELINE, INFERENCE_PIPELINE,
)

from hailo_tiling.types import Det
from tiling_lab.live.controller import DynamicTilingController

# Concrete on-disk resources (project TILING defaults; proven by spike S1).
DEFAULT_HEF = "/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef"
DEFAULT_SO = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"
DEFAULT_FUNC = "filter"
DEFAULT_LABELS = "/usr/local/hailo/resources/json/hailo_4_classes.json"

# Discovery seed for frame 0, before any detection exists (2x2 with overlap).
INITIAL_TILES = ("0.0,0.0,0.55,0.6,s;0.45,0.0,0.55,0.6,s;"
                 "0.0,0.4,0.55,0.6,s;0.45,0.4,0.55,0.6,s")


def probe_dims(video: str) -> tuple[int, int]:
    out = subprocess.check_output([
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=width,height", "-of", "csv=p=0:s=x", video,
    ]).decode().strip()
    w, h = out.split("x")
    return int(w), int(h)


def build_pipeline(video, hef, post_so, func, labels, w, h, fps, out_mkv):
    inner = INFERENCE_PIPELINE(hef_path=hef, post_process_so=post_so,
                               post_function_name=func, batch_size=1,
                               config_json=labels, name="live_infer")
    src = SOURCE_PIPELINE(video_source=video, video_width=w, video_height=h,
                          frame_rate=fps, sync=False)
    return (
        f"{src} ! queue name=live_in_q ! "
        f"hailotilecropper_dynamic name=tc internal-offset=true "
        f"tiling-mode=single-scale tiles-static=\"{INITIAL_TILES}\" "
        f"hailotileaggregator name=agg flatten-detections=true iou-threshold=0.3 "
        f"tc. ! queue name=live_bypass_q ! agg.sink_0 "
        f"tc. ! video/x-raw,format=RGB ! {inner} ! agg.sink_1 "
        f"agg. ! queue name=live_out_q ! hailooverlay_community ! "
        f"videoconvert ! x264enc tune=zerolatency bitrate=5000 ! "
        f"matroskamux ! filesink location={out_mkv} sync=false"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--out", required=True, help="output run dir")
    ap.add_argument("--frames", type=int, default=0, help="0 = whole clip")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--budget", type=float, default=300.0,
                    help="inference budget, tiles/sec. Must be high enough that "
                         "the discovery grid can fire while searching — at 60 "
                         "(~2/frame) discovery is starved and a small aerial "
                         "target is rarely acquired (7.8%% detected); at 300 "
                         "(~10/frame cap, ~3.5 used) acquisition jumps to ~65%%.")
    ap.add_argument("--hef", default=DEFAULT_HEF)
    ap.add_argument("--post-so", default=DEFAULT_SO)
    ap.add_argument("--func", default=DEFAULT_FUNC)
    ap.add_argument("--labels", default=DEFAULT_LABELS)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    out_mkv = os.path.join(args.out, "overlay.mkv")
    tiles_jsonl = open(os.path.join(args.out, "tiles.jsonl"), "w")

    w, h = probe_dims(args.video)
    print(f"[live] video dims {w}x{h}", flush=True)

    Gst.init(None)
    pipeline = Gst.parse_launch(
        build_pipeline(args.video, args.hef, args.post_so, args.func,
                       args.labels, w, h, args.fps, out_mkv))
    cropper = pipeline.get_by_name("tc")
    agg = pipeline.get_by_name("agg")

    ctrl = DynamicTilingController(src_w=w, src_h=h, fps=args.fps,
                                   budget_inf_per_s=args.budget)
    state = {"frame": 0}
    loop = GLib.MainLoop()

    def probe(pad, info):
        buf = info.get_buffer()
        roi = hailo.get_roi_from_buffer(buf)
        dets = roi.get_objects_typed(hailo.HAILO_DETECTION)
        persons = []
        for d in dets:
            if d.get_label() != "person":
                continue
            b = d.get_bbox()
            persons.append(Det(cls=0, score=d.get_confidence(),
                               x=b.xmin(), y=b.ymin(),
                               w=b.width(), h=b.height()))
        tiles = ctrl.update(persons)
        cropper.set_property("tiles-static", tiles or INITIAL_TILES)
        f = state["frame"]
        tiles_jsonl.write(json.dumps({
            "frame": f, "status": ctrl.status, "n_persons": len(persons),
            "n_tiles": (tiles.count(";") + 1) if tiles else 0, "tiles": tiles,
        }) + "\n")
        if f % 30 == 0:
            print(f"[live] frame {f} status={ctrl.status} "
                  f"persons={len(persons)} mean_t/f={ctrl.mean_tiles_per_frame:.2f}",
                  flush=True)
        state["frame"] += 1
        if args.frames and state["frame"] >= args.frames:
            loop.quit()
        return Gst.PadProbeReturn.OK

    agg.get_static_pad("src").add_probe(Gst.PadProbeType.BUFFER, probe)

    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_msg(_bus, msg):
        if msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print(f"[live][BUS-ERROR] {err}: {dbg}", flush=True)
            loop.quit()
        elif msg.type == Gst.MessageType.EOS:
            print("[live] EOS", flush=True)
            loop.quit()
        return True

    bus.connect("message", on_msg)
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)
        tiles_jsonl.close()
        print(f"[live] wrote {out_mkv} and tiles.jsonl in {args.out}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
