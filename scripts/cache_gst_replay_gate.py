"""Per-tile live-vs-cached bit-exact replay gate (GStreamer, on-chip).

Plan `2026-05-31-gst-cache-source-pixel-provenance.md` — Task 6.

Proves that replaying the `gst-hailo-cache` SQLite cache reproduces live
Hailo inference EXACTLY, at the per-tile point (BEFORE the aggregator's
NMS), using the SAME canonical GStreamer pipeline.

Two passes through the canonical tiled pipeline
(`DYNAMIC_TILE_CROPPER_PIPELINE`, cropper on sink_0 bypass + tiles on
sink_1):

  Pass 1 (LIVE):
    cropper ! [ video/x-raw,format=RGB ! videoconvert
                ! hailonet hef=<HEF>
                ! hailofilter so=<POST_SO> function=filter
                ! hailocachewriter mode=tile_cache
                      source-width=W source-height=H output-file=<cache> ]
            ! agg
    A pad probe AFTER `hailofilter` reads each tile buffer's HailoROI
    detections, keyed by (buffer_idx, source-pixel crop rect). The writer
    records the per-tile crop provenance AND the serialized detections.

  Pass 2 (CACHED):
    cropper ! [ video/x-raw,format=RGB ! videoconvert
                ! hailocachereader cache-file=<cache>
                      source-width=W source-height=H on-miss=error
                ! hailocachebypass ]
            ! agg
    The reader serves cached detections WITHOUT touching the chip (there
    is literally no `hailonet` in the pass-2 graph — the strongest possible
    proof the chip was not consulted). It restores the cached detections
    into each tile buffer's HailoROI; the same pad probe (now after
    `hailocachebypass`) reads them back.

The two per-tile detection streams, keyed by (buffer_idx, crop_rect), must
be byte-identical. Writes pass1.json, pass2.json, diff_report.json.

Run:
    source setup_env.sh
    GST_PLUGIN_PATH=gst-hailo-cache/build/src \
      python scripts/cache_gst_replay_gate.py \
        --video <clip.mp4> --hef <model.hef> \
        --out-dir /tmp/gate --max-frames 16 --tiles-static 3x2
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Sequence

import gi

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402

import hailo  # noqa: E402

# Canonical post-process .so + function for the 4-class yolov8 VGA HEF
# (yolov8 → libyolo_hailortpp_postprocess.so + 'filter'; see
# tiling_benchmark/run_pxt_yolov8m.py and the HEF auto-detect convention).
_DEFAULT_POST_SO = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"
_DEFAULT_POST_FN = "filter"


# --------------------------------------------------------------------------
# tiles-static helpers
# --------------------------------------------------------------------------

def _grid_static_tiles(nx: int, ny: int) -> str:
    """Return a `tiles-static` string for an nx-by-ny equal grid (no overlap).

    Each rect is "x,y,w,h" with w=1/nx, h=1/ny. 6 tiles for 3x2. The mode
    suffix is left off (cropper default). Deterministic ordering: row-major.
    """
    w = 1.0 / nx
    h = 1.0 / ny
    rects = []
    for ry in range(ny):
        for rx in range(nx):
            rects.append(f"{rx * w:.6f},{ry * h:.6f},{w:.6f},{h:.6f}")
    return ";".join(rects)


def _parse_tiles_arg(s: str) -> str:
    """Accept either "NxM" (e.g. "3x2") or a raw tiles-static string."""
    s = s.strip()
    if "x" in s and ";" not in s and "," not in s:
        nx, ny = s.lower().split("x")
        return _grid_static_tiles(int(nx), int(ny))
    return s


# --------------------------------------------------------------------------
# detection extraction (the per-tile tap)
# --------------------------------------------------------------------------

def _round(v: float) -> float:
    # Match the writer's %.9g serialization precision so live and cached
    # streams compare equal after the float -> text -> float round-trip the
    # cache performs. 9 significant digits is lossless for float32.
    return float(f"{float(v):.9g}")


def _dets_from_buffer(buf) -> list[dict]:
    """Read per-tile HailoROI detections from a GstBuffer.

    Returns a list of {cls, score, x, y, w, h} dicts in HailoROI insertion
    order (the order hailofilter added them / the reader restored them),
    plus the tile's normalized crop bbox so we can key by source pixels.
    """
    roi = hailo.get_roi_from_buffer(buf)
    dets = roi.get_objects_typed(hailo.HAILO_DETECTION)
    out = []
    for d in dets:
        bb = d.get_bbox()
        out.append({
            "cls": int(d.get_class_id()),
            "score": _round(d.get_confidence()),
            "x": _round(bb.xmin()),
            "y": _round(bb.ymin()),
            "w": _round(bb.width()),
            "h": _round(bb.height()),
        })
    return out


def _tile_crop_px(buf, src_w: int, src_h: int):
    """Source-pixel crop key for a tile buffer, matching the writer/reader
    truncate-then-clamp rule (cache_keys::tile_crop_to_source_px)."""
    roi = hailo.get_roi_from_buffer(buf)
    bb = roi.get_bbox()
    xmin, ymin, w, h = bb.xmin(), bb.ymin(), bb.width(), bb.height()
    # Whole-frame ROI (0,0,1,1) => no per-tile provenance.
    if abs(xmin) < 1e-6 and abs(ymin) < 1e-6 and abs(w - 1.0) < 1e-6 and abs(h - 1.0) < 1e-6:
        return None
    cx = int(xmin * src_w)
    cy = int(ymin * src_h)
    cw = int(w * src_w)
    ch = int(h * src_h)
    # clamp to residual extent (matches the C++ clamp)
    cw = max(0, min(cw, src_w - cx))
    ch = max(0, min(ch, src_h - cy))
    return (cx, cy, cw, ch)


# --------------------------------------------------------------------------
# pipeline construction
# --------------------------------------------------------------------------

def _inner_live(hef: str, post_so: str, post_fn: str, cache: str,
                src_w: int, src_h: int) -> str:
    """Pass-1 inner (per-tile) branch: live hailonet + hailofilter + writer.

    NOTE: the cropper subgraph already pins `video/x-raw,format=RGB` on the
    cropped branch, so the inner string starts at `videoconvert` (no extra
    capsfilter — the canonical Task-1 pipeline keeps resize in the cropper).
    """
    return (
        "videoconvert ! "
        f"hailonet name=gate_hailonet hef-path={hef} batch-size=1 force-writable=true ! "
        f"hailofilter name=gate_tap so-path={post_so} function-name={post_fn} qos=false ! "
        f"hailocachewriter name=gate_writer mode=tile_cache "
        f"source-width={src_w} source-height={src_h} record-empty=true "
        f"output-file={cache}"
    )


def _inner_cached(cache: str, src_w: int, src_h: int) -> str:
    """Pass-2 inner branch: cache reader + bypass. NO hailonet (chip-free)."""
    return (
        "videoconvert ! "
        f"hailocachereader name=gate_reader cache-file={cache} "
        f"source-width={src_w} source-height={src_h} on-miss=error ! "
        "hailocachebypass name=gate_tap"
    )


def _cropper_subgraph(inner: str, tiles_static: str) -> str:
    """DYNAMIC_TILE_CROPPER_PIPELINE, inlined (no hailo_apps import needed).

    bypass branch on sink_0, cropped tiles through `inner` on sink_1,
    rejoined at hailotileaggregator. RGB capsfilter pins the cropped branch
    format. Mirrors tiling_benchmark/tiling_record.py:152.
    """
    return (
        "queue name=tc_in_q ! "
        f"hailotilecropper_dynamic name=tc internal-offset=true "
        f'tiles-static="{tiles_static}" '
        "hailotileaggregator name=agg flatten-detections=true iou-threshold=0.3 "
        "tc. ! queue name=tc_bypass_q ! agg.sink_0 "
        f"tc. ! video/x-raw,format=RGB ! {inner} ! agg.sink_1 "
        "agg. ! queue name=tc_out_q"
    )


def _build_pipeline(video: str, cropper_sub: str) -> str:
    return (
        f'filesrc location="{video}" ! decodebin ! '
        "videoconvert ! video/x-raw,format=RGB ! "
        f"{cropper_sub} ! "
        "fakesink name=gate_sink sync=false"
    )


# --------------------------------------------------------------------------
# run one pass
# --------------------------------------------------------------------------

class _PassResult:
    def __init__(self):
        # ordered list of records: each is dict(buffer_idx, crop, dets)
        self.records: list[dict] = []
        self.error: str | None = None


def _run_pass(pipeline_str: str, src_w: int, src_h: int,
              max_frames: int, n_tiles: int) -> _PassResult:
    result = _PassResult()
    pipeline = Gst.parse_launch(pipeline_str)
    tap = pipeline.get_by_name("gate_tap")
    if tap is None:
        result.error = "gate_tap element not found"
        return result
    srcpad = tap.get_static_pad("src")

    counter = {"n": 0}
    max_buffers = max_frames * n_tiles

    def probe(pad, info):
        buf = info.get_buffer()
        if buf is None:
            return Gst.PadProbeReturn.OK
        idx = counter["n"]
        counter["n"] += 1
        if idx >= max_buffers:
            # signal EOS-ish: stop the loop once we have enough tiles.
            loop.quit()
            return Gst.PadProbeReturn.OK
        crop = _tile_crop_px(buf, src_w, src_h)
        dets = _dets_from_buffer(buf)
        result.records.append({
            "buffer_idx": idx,
            "crop": list(crop) if crop else None,
            "dets": dets,
        })
        return Gst.PadProbeReturn.OK

    srcpad.add_probe(Gst.PadProbeType.BUFFER, probe)

    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_msg(_bus, msg):
        t = msg.type
        if t == Gst.MessageType.EOS:
            loop.quit()
        elif t == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            result.error = f"{err.message} | {dbg}"
            loop.quit()

    bus.connect("message", on_msg)

    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)
    return result


# --------------------------------------------------------------------------
# diff
# --------------------------------------------------------------------------

def _key(rec: dict) -> tuple:
    return (rec["buffer_idx"], tuple(rec["crop"]) if rec["crop"] else None)


def _diff(p1: list[dict], p2: list[dict]) -> list[dict]:
    deviations: list[dict] = []
    m1 = {_key(r): r for r in p1}
    m2 = {_key(r): r for r in p2}
    all_keys = sorted(set(m1) | set(m2), key=lambda k: (k[0], k[1] or (0, 0, 0, 0)))
    for k in all_keys:
        r1 = m1.get(k)
        r2 = m2.get(k)
        if r1 is None:
            deviations.append({"key": list(k[1]) if k[1] else None,
                               "buffer_idx": k[0],
                               "reason": "present in pass2 only"})
            continue
        if r2 is None:
            deviations.append({"key": list(k[1]) if k[1] else None,
                               "buffer_idx": k[0],
                               "reason": "present in pass1 only"})
            continue
        if r1["dets"] != r2["dets"]:
            deviations.append({
                "key": list(k[1]) if k[1] else None,
                "buffer_idx": k[0],
                "reason": "detections differ",
                "pass1": r1["dets"],
                "pass2": r2["dets"],
            })
    return deviations


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------

def _probe_src_dims(video: str) -> tuple[int, int]:
    import cv2
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        raise SystemExit(f"cannot open video: {video}")
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()
    return w, h


def run(video: str, hef: str, post_so: str, post_fn: str, out_dir: Path,
        max_frames: int, tiles_static: str) -> int:
    Gst.init(None)
    out_dir.mkdir(parents=True, exist_ok=True)

    src_w, src_h = _probe_src_dims(video)
    n_tiles = len([r for r in tiles_static.split(";") if r.strip()])
    cache = str(out_dir / "gate_cache.sqlite3")
    if os.path.exists(cache):
        os.unlink(cache)

    print(f"[gate] src={src_w}x{src_h} n_tiles={n_tiles} "
          f"max_frames={max_frames} cache={cache}", flush=True)

    # ---- Pass 1: live -----------------------------------------------------
    print("[gate] pass 1 (LIVE: hailonet + hailofilter + writer) ...", flush=True)
    inner1 = _inner_live(hef, post_so, post_fn, cache, src_w, src_h)
    pipe1 = _build_pipeline(video, _cropper_subgraph(inner1, tiles_static))
    r1 = _run_pass(pipe1, src_w, src_h, max_frames, n_tiles)
    if r1.error:
        print(f"[gate] pass 1 ERROR: {r1.error}", file=sys.stderr)
        report = {"status": "DIFF", "deviations": [{"reason": f"pass1 error: {r1.error}"}],
                  "n_tiles_compared": 0, "pass2_served_from_cache": False}
        (out_dir / "diff_report.json").write_text(json.dumps(report, indent=2))
        return 1
    (out_dir / "pass1.json").write_text(json.dumps(r1.records, indent=2))
    n1_dets = sum(len(r["dets"]) for r in r1.records)
    print(f"[gate]   pass1 tiles={len(r1.records)} total_dets={n1_dets}", flush=True)

    if not os.path.exists(cache):
        print("[gate] no cache produced by pass 1", file=sys.stderr)
        report = {"status": "DIFF", "deviations": [{"reason": "pass1 produced no cache"}],
                  "n_tiles_compared": 0, "pass2_served_from_cache": False}
        (out_dir / "diff_report.json").write_text(json.dumps(report, indent=2))
        return 1

    # ---- Pass 2: cached (chip-free) --------------------------------------
    print("[gate] pass 2 (CACHED: reader + bypass, NO hailonet) ...", flush=True)
    inner2 = _inner_cached(cache, src_w, src_h)
    # Strong proof: assert there is literally no hailonet in the pass-2 graph.
    pass2_has_hailonet = "hailonet" in inner2
    pipe2 = _build_pipeline(video, _cropper_subgraph(inner2, tiles_static))
    r2 = _run_pass(pipe2, src_w, src_h, max_frames, n_tiles)
    if r2.error:
        print(f"[gate] pass 2 ERROR: {r2.error}", file=sys.stderr)
        report = {"status": "DIFF", "deviations": [{"reason": f"pass2 error: {r2.error}"}],
                  "n_tiles_compared": 0,
                  "pass2_served_from_cache": (not pass2_has_hailonet)}
        (out_dir / "diff_report.json").write_text(json.dumps(report, indent=2))
        return 1
    (out_dir / "pass2.json").write_text(json.dumps(r2.records, indent=2))
    n2_dets = sum(len(r["dets"]) for r in r2.records)
    print(f"[gate]   pass2 tiles={len(r2.records)} total_dets={n2_dets}", flush=True)

    # ---- diff -------------------------------------------------------------
    deviations = _diff(r1.records, r2.records)
    n_compared = len(r1.records)
    status = "OK" if not deviations else "DIFF"

    report = {
        "status": status,
        "video": video,
        "hef": hef,
        "src_w": src_w,
        "src_h": src_h,
        "n_tiles_per_frame": n_tiles,
        "max_frames": max_frames,
        "n_tiles_compared": n_compared,
        "pass1_total_dets": n1_dets,
        "pass2_total_dets": n2_dets,
        # The pass-2 inner pipeline has no hailonet at all — the chip is
        # never opened. This is the strongest proof pass 2 was chip-free.
        "pass2_served_from_cache": (not pass2_has_hailonet),
        "pass2_pipeline_has_hailonet": pass2_has_hailonet,
        "deviations": deviations[:50],  # cap for readability
        "n_deviations": len(deviations),
    }
    (out_dir / "diff_report.json").write_text(json.dumps(report, indent=2))

    if deviations:
        print(f"[gate] DIFF — {len(deviations)} deviating tiles", file=sys.stderr)
        for d in deviations[:5]:
            print(f"   - {d}", file=sys.stderr)
        return 1
    print(f"[gate] OK — {n_compared} tiles bit-exact, pass2 chip-free "
          f"(no hailonet in graph)")
    return 0


def _build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="cache_gst_replay_gate")
    ap.add_argument("--video", required=True)
    ap.add_argument(
        "--hef",
        default="/usr/local/hailo/resources/models/hailo10h/"
                "hailo_yolov8n_4_classes_vga.hef",
    )
    ap.add_argument("--post-so", default=_DEFAULT_POST_SO)
    ap.add_argument("--post-function", default=_DEFAULT_POST_FN)
    ap.add_argument("--out-dir", required=True, type=Path)
    ap.add_argument("--max-frames", type=int, default=16)
    ap.add_argument(
        "--tiles-static", default="3x2",
        help='Either "NxM" (equal grid, e.g. 3x2 = 6 tiles) or a raw '
             'tiles-static string "x,y,w,h;...".',
    )
    return ap


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_argparser().parse_args(argv)
    tiles_static = _parse_tiles_arg(args.tiles_static)
    return run(
        video=args.video, hef=args.hef, post_so=args.post_so,
        post_fn=args.post_function, out_dir=args.out_dir,
        max_frames=args.max_frames, tiles_static=tiles_static,
    )


if __name__ == "__main__":
    raise SystemExit(main())
