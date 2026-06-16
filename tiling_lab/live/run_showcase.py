"""Run striped dynamic tiling live, NO drawing / NO saved video.

Validates that decode + tile-crop + track + infer sustains the source fps and
captures per-frame data for the offline visualizer.

    source setup_env.sh
    python -m tiling_lab.live.run_showcase \
        --video /home/giladn/Videos/Drone/Training/Car/DJI_20260614161040_0013_D.MP4 \
        --out tiling_lab/runs/showcase_0013 --fps 60 --target-class vehicle

Pipeline: filesrc -> decodebin -> hailotilecropper_dynamic -> hailonet infer
-> hailotileaggregator -> fakesink. A probe on the aggregator's OUTPUT QUEUE
(not its src pad — see below) steps the striped controller and pushes
next-frame tiles onto the cropper. Outputs
frames.json (visualizer schema) and metrics.json (achieved fps + tile stats).
"""
import argparse
import json
import os
import statistics
import subprocess
import sys
import time

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib  # noqa: E402
import hailo  # noqa: E402

from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
    SOURCE_PIPELINE, INFERENCE_PIPELINE,
)

from hailo_tiling.types import Det
from tiling_lab.live.controller import DynamicTilingController

DEFAULT_HEF = "/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef"
DEFAULT_SO = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"
DEFAULT_FUNC = "filter"
DEFAULT_LABELS = "/usr/local/hailo/resources/json/hailo_4_classes.json"

# Small 4-tile multi-scale seed for frame 0, before any lock/stripe exists.
# (The full 8x6 dense grid is owned by the controller via --dense-grid.)
INITIAL_TILES = ("0.0,0.0,0.18,0.18,m;0.40,0.0,0.18,0.18,m;"
                 "0.80,0.0,0.18,0.18,m;0.40,0.40,0.18,0.18,m")


def tiles_static_to_dicts(s):
    """Parse a 'x,y,w,h,mode;...' tiles-static string into the viewer's tile
    schema: a list of {x,y,w,h,category} dicts (overlay_viewer.load_frames_indexed).
    mode 'm' (dense discovery) -> 'multi-scale'; 's' (ROI/recovery) -> 'dynamic'.

    Input is trusted internal data (crops_to_tiles_static output); segments with
    fewer than 4 fields are skipped, numeric fields are assumed well-formed."""
    out = []
    for seg in s.split(";"):
        if not seg:
            continue
        parts = seg.split(",")
        if len(parts) < 4:
            continue
        x, y, w, h = (float(v) for v in parts[:4])
        mode = parts[4] if len(parts) > 4 else "s"
        out.append({"x": x, "y": y, "w": w, "h": h,
                    "category": "multi-scale" if mode == "m" else "dynamic"})
    return out


def build_frame_record(frame, detections, applied_tiles):
    """Pair a frame's detections with the tiles that ACTUALLY produced them
    (the tiles applied to this frame = the ones pushed on the previous probe),
    so the viewer draws each box inside the tile that detected it.

    `applied_tiles` is a list of categorised tile dicts ({x,y,w,h,category})."""
    return {"frame": frame, "detections": detections,
            "tiles": list(applied_tiles)}


def probe_dims(video: str) -> tuple[int, int]:
    out = subprocess.check_output([
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=width,height", "-of", "csv=p=0:s=x", video,
    ]).decode().strip()
    w, h = out.split("x")
    return int(w), int(h)


def build_pipeline(video, hef, post_so, func, labels, w, h, fps):
    # Dedicated vdevice group, NOT the default "SHARED": this is a standalone
    # measurement tool, and because we os._exit() to dodge the Hailo NULL-teardown
    # abort, network-groups aren't released cleanly — pinning them to a private
    # group keeps that from polluting the SHARED group other apps use (a polluted
    # SHARED group was observed to halve throughput after many runs).
    inner = INFERENCE_PIPELINE(hef_path=hef, post_process_so=post_so,
                               post_function_name=func, batch_size=1,
                               config_json=labels, name="show_infer",
                               vdevice_group_id="showcase_solo")
    src = SOURCE_PIPELINE(video_source=video, video_width=w, video_height=h,
                          frame_rate=fps, sync=False)
    return (
        f"{src} ! queue name=show_in_q ! "
        f"hailotilecropper_dynamic name=tc internal-offset=true "
        f"tiling-mode=single-scale tiles-static=\"{INITIAL_TILES}\" "
        f"hailotileaggregator name=agg flatten-detections=true iou-threshold=0.3 "
        f"tc. ! queue name=show_bypass_q ! agg.sink_0 "
        f"tc. ! video/x-raw,format=RGB ! {inner} ! agg.sink_1 "
        # fakesink MUST set enable-last-sample=false: by default fakesink keeps a
        # ref to the last buffer, pinning one from the aggregator/cropper buffer
        # pool, so the cropper blocks waiting to reuse it and the pipeline
        # deadlocks after frame 0. async=false avoids a preroll wait on this
        # non-live, no-draw drain. (hailotileaggregator's src advertises no caps,
        # so caps-querying sinks need care — see .claude/memory.)
        f"agg. ! queue name=show_out_q ! "
        f"fakesink sync=false async=false enable-last-sample=false"
    )


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=0, help="0 = whole clip")
    ap.add_argument("--fps", type=float, default=60.0)
    ap.add_argument("--budget", type=float, default=600.0,
                    help="inference budget (tiles/s, sliding-window). The showcase "
                         "load is flat: ROI(1) + --dense-per-frame tiles per frame. "
                         "budget/fps must exceed 2*(1 + dense_per_frame) so the meter "
                         "never trims it; 600 @ 60 fps leaves headroom for recovery "
                         "bursts (steady-state share ≈ 7 tiles/frame vs a load of 3).")
    ap.add_argument("--target-class", default="vehicle")
    ap.add_argument("--dense-grid", default="7x6",
                    help="full-frame dense grid (cols x rows). 7x6 on a 4K "
                         "source yields ~native 640x480 crops.")
    ap.add_argument("--grid-overlap", type=float, default=0.15,
                    help="fraction of each dense cell shared with its neighbour "
                         "so boundary objects appear whole in an adjacent tile.")
    ap.add_argument("--cadence-fps", type=float, default=2.0,
                    help="(interleaved mode only) full-frame refresh rate.")
    ap.add_argument("--stripe-mode", default="rolling",
                    choices=["rolling", "interleaved"],
                    help="rolling: flat --dense-per-frame cells/frame swept "
                         "row-major (rolling rows window); interleaved: legacy.")
    ap.add_argument("--dense-per-frame", type=int, default=2,
                    help="(rolling) dense inferences allocated per frame; the "
                         "rest of the budget serves the tracking ROI tile.")
    ap.add_argument("--acquire-mode", default="seed",
                    choices=["seed", "central", "largest"],
                    help="target policy: 'seed' (default) follows exactly the "
                         "--seed/--init-bbox position via a direct nearest-"
                         "detection associator and shows the REAL detected bbox "
                         "(never jumps to another object); 'central' locks the "
                         "target held nearest frame centre for --central-frames "
                         "frames; 'largest' locks the biggest detection (legacy).")
    ap.add_argument("--select-mode", default="click", choices=["click", "detection"],
                    help="how a --seed acts: 'click' runs a zoom pyramid at the "
                         "location until the target is found (works before dense "
                         "detects it); 'detection' snaps to an existing nearby "
                         "detection (tight gate, no zoom).")
    ap.add_argument("--center-frac", type=float, default=0.15,
                    help="half-width of the central acquisition region "
                         "(centre in [0.5-f, 0.5+f] on both axes).")
    ap.add_argument("--central-frames", type=int, default=5,
                    help="number of central *observations* (detections, which "
                         "arrive ~cadence-fps/s for an unlocked target) the same "
                         "track must accumulate before it is locked.")
    ap.add_argument("--init-bbox", default=None, metavar="x,y,w,h",
                    help="initial-location seed: lock the target at this "
                         "normalized bbox (0-1), bypassing auto-acquisition. "
                         "Manual target selection (see tiling_lab.live.pick_target).")
    ap.add_argument("--init-frame", type=int, default=0,
                    help="frame at which to apply --init-bbox (default 0).")
    ap.add_argument("--seed", action="append", default=[], metavar="FRAME:x,y,w,h",
                    help="timed manual seed: at FRAME, (re)lock the target at the "
                         "normalized bbox. Repeatable for multi-trail SOT "
                         "(each re-seed is a fresh acquisition).")
    ap.add_argument("--label", default="showcase")
    ap.add_argument("--hef", default=DEFAULT_HEF)
    ap.add_argument("--post-so", default=DEFAULT_SO)
    ap.add_argument("--func", default=DEFAULT_FUNC)
    ap.add_argument("--labels", default=DEFAULT_LABELS)
    return ap


def main():
    args = build_arg_parser().parse_args()

    gx, gy = (int(v) for v in args.dense_grid.lower().split("x"))
    os.makedirs(args.out, exist_ok=True)
    w, h = probe_dims(args.video)
    print(f"[showcase] dims {w}x{h} fps={args.fps} target={args.target_class}",
          flush=True)

    Gst.init(None)
    pipeline = Gst.parse_launch(build_pipeline(
        args.video, args.hef, args.post_so, args.func, args.labels,
        w, h, args.fps))
    cropper = pipeline.get_by_name("tc")

    ctrl = DynamicTilingController(
        src_w=w, src_h=h, fps=args.fps, budget_inf_per_s=args.budget,
        striped=True, persist=True, dense_grid=(gx, gy),
        cadence_fps=args.cadence_fps, stripe_mode=args.stripe_mode,
        dense_per_frame=args.dense_per_frame, acquire_mode=args.acquire_mode,
        center_frac=args.center_frac, central_frames=args.central_frames,
        grid_overlap=args.grid_overlap)

    # Build the timed-seed map {frame -> bbox} from --init-bbox/--init-frame and
    # any --seed FRAME:x,y,w,h entries. Applied in the probe when the frame hits.
    seeds_by_frame = {}
    if args.init_bbox:
        seeds_by_frame[args.init_frame] = tuple(
            float(v) for v in args.init_bbox.split(","))
    for spec in args.seed:
        fr_str, bbox_str = spec.split(":")
        seeds_by_frame[int(fr_str)] = tuple(float(v) for v in bbox_str.split(","))
    if args.acquire_mode == "seed" and not seeds_by_frame:
        print("[showcase][WARN] acquire-mode=seed but no --seed/--init-bbox "
              "given; the target will never be acquired.", flush=True)

    frame_records = []
    tile_counts = []
    state = {"frame": 0, "t0": None, "t_last": None,
             "applied": tiles_static_to_dicts(INITIAL_TILES)}
    loop = GLib.MainLoop()

    def probe(_pad, info):
        if state["t0"] is None:
            state["t0"] = time.perf_counter()
        if state["frame"] in seeds_by_frame:
            ctrl.seed(seeds_by_frame[state["frame"]], mode=args.select_mode)
            print(f"[showcase] seeded target @frame {state['frame']} "
                  f"bbox={seeds_by_frame[state['frame']]} mode={args.select_mode}",
                  flush=True)
        buf = info.get_buffer()
        roi = hailo.get_roi_from_buffer(buf)
        dets = roi.get_objects_typed(hailo.HAILO_DETECTION)
        all_dets = []
        target_dets = []
        for d in dets:
            b = d.get_bbox()
            rec = {"label": d.get_label(), "confidence": d.get_confidence(),
                   "bbox": [b.xmin(), b.ymin(), b.width(), b.height()]}
            all_dets.append(rec)
            if d.get_label() == args.target_class:
                target_dets.append(Det(cls=0, score=d.get_confidence(),
                                       x=b.xmin(), y=b.ymin(),
                                       w=b.width(), h=b.height()))
        tiles, records = ctrl.step_showcase(target_dets, all_dets)
        tile_dicts = ctrl.last_tiles                  # categorised (ROI/dense/pyramid)
        pushed = tiles or INITIAL_TILES
        # Record THIS frame's detections against the tiles that produced them
        # (applied on the PREVIOUS probe), then push `pushed` for the next frame.
        frame_records.append(build_frame_record(state["frame"], records,
                                                state["applied"]))
        cropper.set_property("tiles-static", pushed)
        state["applied"] = (tile_dicts if tiles
                            else tiles_static_to_dicts(INITIAL_TILES))
        n_tiles = pushed.count(";") + 1 if pushed else 0
        tile_counts.append(n_tiles)
        f = state["frame"]
        if f % 60 == 0:
            print(f"[showcase] frame {f} status={ctrl.status} "
                  f"n_tiles={n_tiles}", flush=True)
        state["frame"] += 1
        state["t_last"] = time.perf_counter()
        if args.frames and state["frame"] >= args.frames:
            loop.quit()
        return Gst.PadProbeReturn.OK

    # Attach the probe DOWNSTREAM of show_out_q, not on agg.src. On agg.src the
    # probe runs on the aggregator's streaming thread, so stepping the controller
    # and pushing tiles-static back to the upstream cropper stalls aggregation and
    # serialises the whole pipeline (~2 fps). Probing after the queue decouples the
    # probe onto the queue's downstream thread, letting the aggregator keep
    # producing -> sustains 60 fps. (Verified 2026-06-15: 63 fps vs 2 fps.)
    out_q = pipeline.get_by_name("show_out_q")
    out_q.get_static_pad("src").add_probe(Gst.PadProbeType.BUFFER, probe)

    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_msg(_bus, msg):
        if msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print(f"[showcase][BUS-ERROR] {err}: {dbg}", flush=True)
            loop.quit()
        elif msg.type == Gst.MessageType.EOS:
            print("[showcase] EOS", flush=True)
            loop.quit()
        return True

    bus.connect("message", on_msg)
    pipeline.set_state(Gst.State.PLAYING)
    loop.run()

    # Compute + write outputs FIRST, while all data is in Python memory and the
    # files can be flushed/closed cleanly. Only then tear the pipeline down.
    n = len(frame_records)
    wall_s = (state["t_last"] - state["t0"]) if (state["t0"] and n > 1) else 0.0
    achieved = (n / wall_s) if wall_s > 0 else 0.0
    counts = tile_counts or [0]
    metrics = {
        "frames": n,
        "wall_s": round(wall_s, 3),
        "achieved_fps": round(achieved, 2),
        "source_fps": args.fps,
        "sustains_60fps": bool(achieved >= 60.0),
        "mean_tiles_per_frame": round(statistics.mean(counts), 3),
        "max_tiles_per_frame": max(counts),
        "p95_tiles_per_frame": sorted(counts)[int(0.95 * (len(counts) - 1))],
    }
    with open(os.path.join(args.out, "frames.json"), "w", encoding="utf-8") as fp:
        json.dump({"label": args.label, "frames": frame_records}, fp)
    with open(os.path.join(args.out, "metrics.json"), "w", encoding="utf-8") as fp:
        json.dump(metrics, fp, indent=2)
    print(f"[showcase] {n} frames, achieved_fps={achieved:.2f} "
          f"sustains_60fps={metrics['sustains_60fps']} "
          f"mean_t/f={metrics['mean_tiles_per_frame']} "
          f"max_t/f={metrics['max_tiles_per_frame']}", flush=True)

    # Hard-exit, bypassing GStreamer's NULL-state teardown. Some Hailo elements
    # abort with a std::logic_error ("basic_string::_M_construct null not valid")
    # during the PLAYING->NULL transition in headless / non-TTY runs (e.g. under
    # subprocess capture), which would kill the process with SIGABRT *after* the
    # work is done and corrupt the exit code. Outputs are already flushed above,
    # so a clean os._exit(0) is the robust choice for this measurement tool; the
    # OS reclaims the device handle on exit (verified: back-to-back runs reuse it).
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
