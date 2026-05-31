"""Resize envelope-vs-stretch detection-quality probe (Task 7).

Runs the canonical detection pipeline and taps POST-aggregator (final
source-frame) detections — exactly what the drone-follow app consumes.

For one FOV clip it runs three configs:
  - whole : untiled whole-frame inference (pseudo-reference)
  - stretch : 3x2 tiled, cropper resize-mode=stretch
  - letterbox : 3x2 tiled, cropper resize-mode=letterbox

and dumps per-frame detections to JSON. A separate analysis step
(resize_quality_analyze.py) computes counts / confidence / per-class /
recall-vs-reference.

This reuses the canonical cropper subgraph from tiling_record.py and the
post-aggregator detection-extraction approach from
scripts/cache_gst_replay_gate.py.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

# Disable VAAPI (QoS storms on this host) — mirror tiling_record.py.
_VAAPI = ",".join(f"{f}:NONE" for f in (
    "vaapidecodebin", "vaapidecode", "vaapih264dec", "vaapih265dec",
    "vaapimpeg2dec", "vaapivc1dec", "vaapijpegdec", "vaapipostproc"))
_ex = os.environ.get("GST_PLUGIN_FEATURE_RANK", "")
os.environ["GST_PLUGIN_FEATURE_RANK"] = f"{_ex},{_VAAPI}" if _ex else _VAAPI

import gi
gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402
import hailo  # noqa: E402

POST_SO = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"
POST_FN = "filter"
HEF = ("/usr/local/hailo/resources/models/hailo10h/"
       "hailo_yolov8n_4_classes_vga.hef")


def grid_static(nx: int, ny: int) -> str:
    w, h = 1.0 / nx, 1.0 / ny
    return ";".join(
        f"{rx*w:.6f},{ry*h:.6f},{w:.6f},{h:.6f}"
        for ry in range(ny) for rx in range(nx))


def dets_from_buffer(buf) -> list[dict]:
    roi = hailo.get_roi_from_buffer(buf)
    out = []
    for d in roi.get_objects_typed(hailo.HAILO_DETECTION):
        bb = d.get_bbox()
        out.append({
            "cls": int(d.get_class_id()),
            "score": float(d.get_confidence()),
            "x": float(bb.xmin()), "y": float(bb.ymin()),
            "w": float(bb.width()), "h": float(bb.height()),
        })
    return out


def _inference(score_thr: float) -> str:
    return (
        "videoconvert ! "
        f"hailonet name=hn hef-path={HEF} batch-size=1 force-writable=true "
        f"nms-score-threshold={score_thr} ! "
        f"hailofilter so-path={POST_SO} function-name={POST_FN} qos=false"
    )


def build_tiled(video: str, mode: str, tiles_static: str,
                score_thr: float) -> str:
    inner = _inference(score_thr)
    return (
        f'filesrc location="{video}" ! decodebin ! '
        "videoconvert ! video/x-raw,format=RGB ! "
        "queue name=in_q ! "
        f"hailotilecropper_dynamic name=tc internal-offset=true "
        f"resize-mode={mode} "
        f'tiles-static="{tiles_static}" '
        "hailotileaggregator name=agg flatten-detections=true iou-threshold=0.3 "
        "tc. ! queue name=bypass_q ! agg.sink_0 "
        f"tc. ! video/x-raw,format=RGB ! {inner} ! queue ! agg.sink_1 "
        "agg. ! queue name=out_q ! identity name=tap ! "
        "fakesink name=sink sync=false"
    )


def build_whole(video: str, score_thr: float) -> str:
    # Untiled reference: scale the whole frame to the network input (640x480)
    # the same way hailonet expects. The cropper does this implicitly in the
    # tiled paths; here we do it explicitly with a single videoscale. This
    # mirrors the standard non-tiled detection pipeline.
    inner = _inference(score_thr)
    return (
        f'filesrc location="{video}" ! decodebin ! '
        "videoscale ! video/x-raw,width=640,height=480 ! "
        f"videoconvert ! video/x-raw,format=RGB ! "
        f"queue ! {inner} ! "
        "queue ! identity name=tap ! fakesink name=sink sync=false"
    )


def run_pass(pipeline_str: str, max_frames: int) -> tuple[list, str | None]:
    Gst.init(None)
    pipeline = Gst.parse_launch(pipeline_str)
    tap = pipeline.get_by_name("tap")
    srcpad = tap.get_static_pad("src")
    records: list = []
    err = {"msg": None}
    loop = GLib.MainLoop()

    def probe(pad, info):
        buf = info.get_buffer()
        if buf is None:
            return Gst.PadProbeReturn.OK
        idx = len(records)
        if idx >= max_frames:
            loop.quit()
            return Gst.PadProbeReturn.OK
        records.append({"frame": idx, "dets": dets_from_buffer(buf)})
        return Gst.PadProbeReturn.OK

    srcpad.add_probe(Gst.PadProbeType.BUFFER, probe)
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_msg(_b, msg):
        if msg.type == Gst.MessageType.EOS:
            loop.quit()
        elif msg.type == Gst.MessageType.ERROR:
            e, dbg = msg.parse_error()
            err["msg"] = f"{e.message} | {dbg}"
            loop.quit()

    bus.connect("message", on_msg)
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)
    return records, err["msg"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--out-dir", required=True, type=Path)
    ap.add_argument("--tag", required=True, help="fov tag for filenames")
    ap.add_argument("--max-frames", type=int, default=120)
    ap.add_argument("--score-threshold", type=float, default=0.3)
    ap.add_argument("--nx", type=int, default=3)
    ap.add_argument("--ny", type=int, default=2)
    ap.add_argument("--ref-nx", type=int, default=6)
    ap.add_argument("--ref-ny", type=int, default=4)
    args = ap.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    tiles = grid_static(args.nx, args.ny)

    # Dense-tile pseudo-reference: a finer grid (default 6x4) where small /
    # distant people stay resolvable after the per-tile resize. This is a much
    # stronger reference than a whole-frame downscale (which destroys small
    # targets at 3840x2160 -> 640x480). Uses the same HEF; resize-mode=stretch
    # for the reference (status quo). The dense grid is what tiling_benchmark
    # uses as pseudo-GT (12x9 there; we use a lighter grid for runtime).
    dense = grid_static(args.ref_nx, args.ref_ny)
    configs = {
        "whole": build_whole(args.video, args.score_threshold),
        "dense_ref": build_tiled(args.video, "stretch", dense, args.score_threshold),
        "stretch": build_tiled(args.video, "stretch", tiles, args.score_threshold),
        "letterbox": build_tiled(args.video, "letterbox", tiles, args.score_threshold),
    }
    for name, pipe in configs.items():
        print(f"[{args.tag}] running {name} ...", flush=True)
        records, err = run_pass(pipe, args.max_frames)
        if err:
            print(f"[{args.tag}] {name} ERROR: {err}", file=sys.stderr)
            return 1
        n = sum(len(r["dets"]) for r in records)
        print(f"[{args.tag}]   {name}: {len(records)} frames, {n} dets",
              flush=True)
        out = args.out_dir / f"{args.tag}__{name}.json"
        out.write_text(json.dumps(records))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
