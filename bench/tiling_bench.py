"""Detection-throughput benchmark for tiling configurations.

Single-config runner. Drops the post-overlay video into a fakesink, counts
detections per second from the user callback, and writes a JSON summary on EOS.

Modeled on tiling_record.py: same VAAPI-disable trick, same dynamic
cropper (`hailotilecropper_dynamic`) wrapping the inference + community
overlay (with `show-tiles=false`). HEF / postprocess resolution is inherited
from `GStreamerTilingApp`.

Example:
  python tiling_bench.py --input /path/to/video.mp4 \
      --tiles-x 3 --tiles-y 2 --overlap-x 0.5 --overlap-y 0.5 \
      --bench-output /tmp/tiling_bench_3x2.json --bench-label 3x2
"""

import os
import sys

# Disable VAAPI plugins before any GStreamer import so decodebin falls back to
# software decoders (avdec_h264, etc).
_VAAPI_DISABLE = ",".join(
    f"{f}:NONE"
    for f in (
        "vaapidecodebin",
        "vaapidecode",
        "vaapih264dec",
        "vaapih265dec",
        "vaapimpeg2dec",
        "vaapivc1dec",
        "vaapijpegdec",
        "vaapipostproc",
    )
)
_existing = os.environ.get("GST_PLUGIN_FEATURE_RANK", "")
os.environ["GST_PLUGIN_FEATURE_RANK"] = (
    f"{_existing},{_VAAPI_DISABLE}" if _existing else _VAAPI_DISABLE
)

import json
import time
from pathlib import Path

import gi
gi.require_version("Gst", "1.0")

import hailo  # noqa: E402

from hailo_apps.python.core.common.core import get_pipeline_parser  # noqa: E402
from hailo_apps.python.core.gstreamer.gstreamer_app import app_callback_class  # noqa: E402
from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (  # noqa: E402
    INFERENCE_PIPELINE,
    OVERLAY_PIPELINE,
    QUEUE,
    TILE_CROPPER_PIPELINE,
    USER_CALLBACK_PIPELINE,
)
from hailo_apps.python.core.common.hailo_logger import get_logger  # noqa: E402
from hailo_apps.python.pipeline_apps.tiling.tiling_pipeline import GStreamerTilingApp  # noqa: E402

# Reuse the cropper subgraph + grid->static-tile string conversion from the
# sibling tiling_record.py — same directory, no __init__.py, so add the parent
# dir to sys.path and import it as a top-level module.
_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))
from tiling_record import (  # noqa: E402
    DYNAMIC_TILE_CROPPER_PIPELINE,
    _center_tile_static,
)

# Probe whether the python bindings expose get_class_id() on detections —
# decide once at import time so we don't pay a try/except per detection.
_HAS_CLASS_ID = hasattr(hailo.HailoDetection, "get_class_id") if hasattr(hailo, "HailoDetection") else True

hailo_logger = get_logger(__name__)

_GST_CLOCK_TIME_NONE = 0xFFFFFFFFFFFFFFFF


class BenchData(app_callback_class):
    def __init__(self):
        super().__init__()
        self.per_second: dict[int, dict] = {}
        # Per-frame detection list — one entry per buffer arrived at the
        # callback. Normalized bbox coords (0-1). Written to <output>.frames.json
        # so the headline summary stays small.
        self.per_frame: list[dict] = []
        self.frames_seen = 0
        self.fallback_fps = 100.0
        self.first_pts_seen = False

    def reset(self):
        self.per_second.clear()
        self.per_frame.clear()
        self.frames_seen = 0
        self.first_pts_seen = False


def bench_callback(_element, buffer, user_data: BenchData):
    """Called via the framework's identity-handoff signal — `buffer` is a Gst.Buffer."""
    if buffer is None:
        return
    user_data.frames_seen += 1

    pts = buffer.pts
    if pts == _GST_CLOCK_TIME_NONE or pts is None:
        second = int((user_data.frames_seen - 1) // user_data.fallback_fps)
    else:
        second = int(pts // 1_000_000_000)
        user_data.first_pts_seen = True

    try:
        roi = hailo.get_roi_from_buffer(buffer)
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    except Exception:
        detections = []

    bucket = user_data.per_second.get(second)
    if bucket is None:
        bucket = {
            "frames": 0,
            "frames_with_det": 0,
            "total_det": 0,
            "max_det_in_frame": 0,
            "_conf_sum": 0.0,
            "_conf_n": 0,
            "max_conf": 0.0,
        }
        user_data.per_second[second] = bucket

    bucket["frames"] += 1
    n = len(detections)

    # Build per-frame detection list (always, even when n == 0 — useful for
    # the analyzer to know empty frames were observed).
    frame_dets: list[dict] = []
    for det in detections:
        try:
            bbox = det.get_bbox()
            x = float(bbox.xmin())
            y = float(bbox.ymin())
            w = float(bbox.width())
            h = float(bbox.height())
        except Exception:
            continue
        c = float(det.get_confidence())
        try:
            label = det.get_label()
        except Exception:
            label = ""
        entry = {
            "bbox": [x, y, w, h],
            "confidence": c,
            "label": label,
        }
        if _HAS_CLASS_ID:
            try:
                entry["class_id"] = int(det.get_class_id())
            except Exception:
                pass
        frame_dets.append(entry)

    user_data.per_frame.append({
        "frame": user_data.frames_seen - 1,
        "second": second,
        "pts_ns": int(pts) if pts not in (None, _GST_CLOCK_TIME_NONE) else -1,
        "detections": frame_dets,
    })

    if n > 0:
        bucket["frames_with_det"] += 1
        bucket["total_det"] += n
        if n > bucket["max_det_in_frame"]:
            bucket["max_det_in_frame"] = n
        for det_entry in frame_dets:
            c = det_entry["confidence"]
            bucket["_conf_sum"] += c
            bucket["_conf_n"] += 1
            if c > bucket["max_conf"]:
                bucket["max_conf"] = c


class GStreamerTilingBenchApp(GStreamerTilingApp):
    def __init__(self, app_callback, user_data, parser=None):
        self._summary_written = False
        super().__init__(app_callback, user_data, parser=parser)

    def _add_tiling_arguments(self, parser):
        super()._add_tiling_arguments(parser)
        parser.add_argument(
            "--score-threshold",
            type=float,
            default=None,
            help="If set, passes nms-score-threshold=<val> to hailonet. "
                 "Only valid for HEFs with on-chip NMS metadata (e.g. YOLO HEFs). "
                 "Leave unset for HEFs whose post-process happens in the .so "
                 "(e.g. ssd_mobilenet_v1_visdrone — threshold lives in the labels JSON).",
        )
        parser.add_argument(
            "--overlap-x",
            type=float,
            default=None,
            help="Manual overlap fraction along X axis (0.0-0.5). Overrides auto config.",
        )
        parser.add_argument(
            "--overlap-y",
            type=float,
            default=None,
            help="Manual overlap fraction along Y axis (0.0-0.5). Overrides auto config.",
        )
        parser.add_argument(
            "--bench-output",
            type=str,
            required=True,
            help="Path to write the benchmark JSON summary.",
        )
        parser.add_argument(
            "--bench-label",
            type=str,
            default="",
            help="Free-form label for this run (echoed in JSON + stdout summary).",
        )
        parser.add_argument(
            "--post-process-so",
            type=str,
            default=None,
            help="Override the post-process .so path (otherwise auto-detected from HEF, "
                 "which assumes YOLO). For ssd_mobilenet_v1_visdrone, pass "
                 "/usr/local/hailo/resources/so/libmobilenet_ssd_postprocess.so",
        )
        parser.add_argument(
            "--post-function",
            type=str,
            default=None,
            help="Override the post-process function name (otherwise auto-detected from "
                 "HEF, which assumes YOLO 'filter'). For ssd_mobilenet_v1_visdrone pass "
                 "'mobilenet_ssd_visdrone'.",
        )
        parser.add_argument(
            "--include-full-frame",
            action="store_true",
            help="Prepend a whole-frame (0,0,1,1) tile to tiles-static so large objects "
                 "are seen intact by at least one tile. Pairs with multi-tile grids to "
                 "rescue objects larger than a single tile.",
        )
        parser.add_argument(
            "--include-center-tile",
            action="store_true",
            help="Append a centered square static tile to the grid. Combines with "
                 "--include-full-frame and the regular grid — all three lists are "
                 "concatenated and fed to hailotilecropper_dynamic's tiles-static.",
        )
        parser.add_argument(
            "--center-tile-size",
            type=float,
            default=0.4,
            help="Side length of the centered tile as a fraction of the frame (default 0.4).",
        )
        parser.add_argument(
            "--show-tiles",
            action="store_true",
            help="Set show-tiles=true on the community overlay (no visible effect with "
                 "the bench fakesink, but the property is forwarded for consistency).",
        )

    def on_eos(self):
        hailo_logger.info("EOS received; writing bench summary and shutting down")
        try:
            self._write_summary()
        except Exception as exc:
            hailo_logger.error("Failed to write bench summary: %s", exc)
        self.shutdown()

    def get_pipeline_string(self) -> str:
        # Apply post-process overrides here so they take effect before the
        # parent's create_pipeline() (called from super().__init__) reads them.
        if self.options_menu.post_process_so:
            self.post_process_so = self.options_menu.post_process_so
            hailo_logger.info("Overriding post_process_so -> %s", self.post_process_so)
        if self.options_menu.post_function:
            self.post_function = self.options_menu.post_function
            hailo_logger.info("Overriding post_function -> %s", self.post_function)

        source_pipeline = self.get_source_pipeline()

        score_threshold = self.options_menu.score_threshold
        additional_params = ""
        if score_threshold is not None:
            additional_params = f"nms-score-threshold={score_threshold}"
            hailo_logger.info("Detection on-chip NMS score threshold set to %.2f", score_threshold)
        else:
            hailo_logger.info("No on-chip NMS threshold passed; relying on .so post-process internal threshold")
        detection_pipeline = INFERENCE_PIPELINE(
            hef_path=self.hef_path,
            post_process_so=self.post_process_so,
            post_function_name=self.post_function,
            batch_size=self.batch_size,
            config_json=self.labels_json,
            additional_params=additional_params,
        )

        tiles_x = self.tiles_x
        tiles_y = self.tiles_y
        overlap_x = self.options_menu.overlap_x if self.options_menu.overlap_x is not None else self.overlap_x
        overlap_y = self.options_menu.overlap_y if self.options_menu.overlap_y is not None else self.overlap_y

        if self.use_multi_scale:
            # GT path: use upstream `hailotilecropper` (NOT the dynamic variant)
            # because only the regular cropper supports tiling-mode=1 + scale-level=N.
            # With tiles_x=tiles_y=1, scale_level=3 → batch_size = 1 (custom 1x1) +
            # 14 (predefined 1x1+2x2+3x3) = 15 inferences/frame.
            # border-threshold is required (>0) for multi-scale — already set
            # by the configuration module.
            hailo_logger.info(
                "GT multi-scale: custom %dx%d + predefined scale-level=%d "
                "(predefined grids 1x1%s%s) = %d total tiles/frame",
                tiles_x, tiles_y, self.scale_level,
                "+2x2" if self.scale_level >= 2 else "",
                "+3x3" if self.scale_level >= 3 else "",
                self.batch_size,
            )
            tile_cropper_pipeline = TILE_CROPPER_PIPELINE(
                detection_pipeline,
                name="tile_cropper_wrapper",
                internal_offset=True,
                scale_level=self.scale_level,
                tiling_mode=1,
                tiles_along_x_axis=tiles_x,
                tiles_along_y_axis=tiles_y,
                overlap_x_axis=overlap_x,
                overlap_y_axis=overlap_y,
                iou_threshold=self.iou_threshold,
                border_threshold=self.border_threshold,
            )
        else:
            # Grid is configured directly on the plugin via tiles-along-x/y-axis +
            # overlap-x/y-axis (matches upstream hailotilecropper). Extras like
            # full-frame and center tile go through tiles-static.
            extras: list[str] = []
            if self.options_menu.include_full_frame and not (tiles_x == 1 and tiles_y == 1):
                extras.append("0.000000,0.000000,1.000000,1.000000")
            if self.options_menu.include_center_tile:
                extras.append(_center_tile_static(self.options_menu.center_tile_size))
            tiles_static = ";".join(extras)
            hailo_logger.info(
                "Bench grid: %dx%d, overlap (x=%.2f, y=%.2f), full_frame=%s, "
                "center_tile=%s (size=%.2f), tiles_static=%s",
                tiles_x, tiles_y, overlap_x, overlap_y,
                self.options_menu.include_full_frame,
                self.options_menu.include_center_tile,
                self.options_menu.center_tile_size,
                tiles_static,
            )

            tile_cropper_pipeline = DYNAMIC_TILE_CROPPER_PIPELINE(
                detection_pipeline,
                name="tile_cropper_wrapper",
                internal_offset=True,
                iou_threshold=self.iou_threshold,
                border_threshold=self.border_threshold,
                tiles_static=tiles_static,
                tiles_x=tiles_x,
                tiles_y=tiles_y,
                overlap_x=overlap_x,
                overlap_y=overlap_y,
            )

        user_callback_pipeline = USER_CALLBACK_PIPELINE()
        overlay_pipeline = OVERLAY_PIPELINE(
            name="hailo_overlay", community=True,
            show_tiles=bool(self.options_menu.show_tiles),
        )

        # Pure throughput sink: never sync to clock, never wait on preroll.
        sink_branch = (
            f"{QUEUE(name='bench_sink_q')} ! "
            f"fakesink name=hailo_display sync=false async=false silent=true"
        )

        pipeline_string = (
            f"{source_pipeline} ! "
            f"{tile_cropper_pipeline} ! "
            f"{user_callback_pipeline} ! "
            f"{overlay_pipeline} ! "
            f"{sink_branch}"
        )

        hailo_logger.debug("Pipeline string: %s", pipeline_string)
        return pipeline_string

    def _write_summary(self):
        if self._summary_written:
            return
        self._summary_written = True

        ud: BenchData = self.user_data
        per_second_list = []
        total_frames = 0
        frames_with_det = 0
        total_det = 0
        global_conf_sum = 0.0
        global_conf_n = 0
        seconds_with_any = 0

        for second in sorted(ud.per_second.keys()):
            b = ud.per_second[second]
            mean_conf = (b["_conf_sum"] / b["_conf_n"]) if b["_conf_n"] else 0.0
            per_second_list.append({
                "second": second,
                "frames": b["frames"],
                "frames_with_det": b["frames_with_det"],
                "total_det": b["total_det"],
                "max_det_in_frame": b["max_det_in_frame"],
                "mean_conf": round(mean_conf, 4),
                "max_conf": round(b["max_conf"], 4),
            })
            total_frames += b["frames"]
            frames_with_det += b["frames_with_det"]
            total_det += b["total_det"]
            global_conf_sum += b["_conf_sum"]
            global_conf_n += b["_conf_n"]
            if b["total_det"] > 0:
                seconds_with_any += 1

        # Use the actual buffer-counted total when present; ud.frames_seen is the
        # callback-side total which should match.
        if total_frames == 0:
            total_frames = ud.frames_seen

        frames_with_det_pct = (100.0 * frames_with_det / total_frames) if total_frames else 0.0
        mean_det_per_frame = (total_det / total_frames) if total_frames else 0.0
        mean_conf = (global_conf_sum / global_conf_n) if global_conf_n else 0.0
        n_seconds = len(per_second_list)
        seconds_with_det_pct = (100.0 * seconds_with_any / n_seconds) if n_seconds else 0.0

        overlap_x = self.options_menu.overlap_x if self.options_menu.overlap_x is not None else self.overlap_x
        overlap_y = self.options_menu.overlap_y if self.options_menu.overlap_y is not None else self.overlap_y

        summary = {
            "label": self.options_menu.bench_label,
            "config": {
                "tiles_x": self.tiles_x,
                "tiles_y": self.tiles_y,
                "overlap_x": float(overlap_x),
                "overlap_y": float(overlap_y),
                "score_threshold": (float(self.options_menu.score_threshold)
                                    if self.options_menu.score_threshold is not None
                                    else None),
                "multi_scale": bool(self.use_multi_scale),
                "scale_level": int(self.scale_level) if self.use_multi_scale else 0,
                "batch_size": int(self.batch_size),
                "include_full_frame": bool(self.options_menu.include_full_frame),
                "include_center_tile": bool(self.options_menu.include_center_tile),
                "center_tile_size": (float(self.options_menu.center_tile_size)
                                     if self.options_menu.include_center_tile else 0.0),
            },
            "video": {
                "path": self.video_source,
                "fps": float(ud.fallback_fps),
                "duration_s": float(n_seconds),
                "frames_seen": ud.frames_seen,
            },
            "per_second": per_second_list,
            "summary": {
                "total_frames": total_frames,
                "frames_with_det": frames_with_det,
                "frames_with_det_pct": round(frames_with_det_pct, 2),
                "total_det": total_det,
                "mean_det_per_frame": round(mean_det_per_frame, 4),
                "mean_conf": round(mean_conf, 4),
                "seconds_with_any_det": seconds_with_any,
                "seconds_with_any_det_pct": round(seconds_with_det_pct, 2),
            },
        }

        out_path = Path(self.options_menu.bench_output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with out_path.open("w") as f:
            json.dump(summary, f, indent=2)

        # Sidecar per-frame detections file. Kept separate from the headline
        # summary so the summary stays small. Used by analyze_bench.py for
        # mAP-style comparison against a pseudo-GT run.
        if out_path.suffix == ".json":
            frames_out = out_path.with_name(out_path.stem + ".frames.json")
        else:
            frames_out = out_path.with_name(out_path.name + ".frames.json")
        frames_doc = {
            "label": summary["label"],
            "config": summary["config"],
            "video": summary["video"],
            "frames": ud.per_frame,
        }
        with frames_out.open("w") as f:
            json.dump(frames_doc, f)

        total_dets_in_frames = sum(len(fr["detections"]) for fr in ud.per_frame)

        print(
            f"BENCH RESULT label={summary['label']!r} "
            f"frames={total_frames} "
            f"frames_with_det_pct={summary['summary']['frames_with_det_pct']:.2f} "
            f"mean_det_per_frame={summary['summary']['mean_det_per_frame']:.3f} "
            f"mean_conf={summary['summary']['mean_conf']:.3f} "
            f"per_frame_total_det={total_dets_in_frames} "
            f"json={out_path} frames_json={frames_out}",
            flush=True,
        )


def _strip_file_uri_in_argv():
    """Pre-process argv so `--input file:///path` becomes `--input /path`.

    The pipeline `filesrc` element wants a filesystem path, not a URI. The
    driver passes file:// URIs for clarity; strip the scheme here.
    """
    new_argv = []
    i = 0
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg in ("--input", "-i") and i + 1 < len(sys.argv):
            val = sys.argv[i + 1]
            if val.startswith("file://"):
                val = val[len("file://"):]
            new_argv.append(arg)
            new_argv.append(val)
            i += 2
        elif arg.startswith("--input=") and arg[len("--input="):].startswith("file://"):
            new_argv.append("--input=" + arg[len("--input=") + len("file://"):])
            i += 1
        else:
            new_argv.append(arg)
            i += 1
    sys.argv = new_argv


def main():
    _strip_file_uri_in_argv()
    parser = get_pipeline_parser()
    parser.set_defaults(width=1280, height=800)

    user_data = BenchData()
    app = GStreamerTilingBenchApp(bench_callback, user_data, parser=parser)

    t0 = time.perf_counter()
    try:
        app.run()
    except SystemExit:
        wall = time.perf_counter() - t0
        print(f"BENCH WALL_CLOCK_S={wall:.2f}", flush=True)
        raise


if __name__ == "__main__":
    main()
