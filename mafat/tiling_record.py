"""Temporary tiling pipeline with file recording.

Throwaway script — DO NOT COMMIT. Lives in /tmp/ to keep the repo clean.

Subclasses GStreamerTilingApp. Display branch is always on; pass --record
to additionally tee the post-overlay video to an x264enc -> matroskamux ->
filesink (recorded .mkv).

Defaults to 1280x800 input. Use --output to set the recording path.

Example:
  python /tmp/tiling_record.py --input usb
  python /tmp/tiling_record.py --input usb --record --output /tmp/tiling.mkv
  python /tmp/tiling_record.py --input rpi --width 1280 --height 800 --record

After recording, fix the matroska header:
  ffmpeg -i /tmp/tiling.mkv -c copy /tmp/tiling_fixed.mkv
"""

import os
import sys

# Disable VAAPI plugins before any GStreamer import so decodebin falls back to
# software decoders (avdec_h264, etc). VAAPI was producing QoS message storms
# on this host, likely due to driver / sync mismatches.
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

import hailo

from hailo_apps.python.core.common.core import get_pipeline_parser, handle_list_models_flag
from hailo_apps.python.core.common.defines import TILING_PIPELINE
from hailo_apps.python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
    FILE_SINK_PIPELINE,
    INFERENCE_PIPELINE,
    OVERLAY_PIPELINE,
    QUEUE,
    USER_CALLBACK_PIPELINE,
)
from hailo_apps.python.core.common.hailo_logger import get_logger
from hailo_apps.python.pipeline_apps.tiling.tiling_pipeline import GStreamerTilingApp

hailo_logger = get_logger(__name__)


# ============================================================================
# Manual tile configuration
# ============================================================================
# This file uses `hailotilecropper_dynamic`, which accepts an arbitrary list of
# normalized rectangles via its `tiles-static` property (and can also accept
# per-frame tiles attached as HailoTileROI objects via an upstream
# `identity signal-handoffs=true` element — not used here). It is NOT limited
# to regular grids; multi-scale auto-tile generation from the upstream
# `hailotilecropper` (`scale-level`, `tiling-mode`) has no equivalent here, so
# `--multi-scale` falls back to the configured single-scale grid.
#
# Set MANUAL_TILES = None to use the upstream auto/manual logic (default).
# Set MANUAL_TILES = dict(...) to override tile grid & overlap directly.
#
# overlap_x/_y are fractions of TILE size (not frame). Range: 0.0 - 0.5.
#
# The values below are what auto-mode computes for the current pipeline
# (1280x800 source, 640x640 model, --min-overlap 0.1) — copy and edit:
#
# MANUAL_TILES = {
#     "tiles_x": 3,
#     "tiles_y": 2,
#     "overlap_x": 0.5,
#     "overlap_y": 0.5,
# }
MANUAL_TILES = None


def passthrough_callback(pad, buffer, user_data):
    return


def _center_tile_static(size: float) -> str:
    """Return a single static-tile rectangle string for a centered square tile of
    side `size` (fraction of frame). Useful as an extra fixed tile to give the
    detector a higher-resolution look at the middle of the frame on top of a
    regular grid."""
    if not (0.0 < size <= 1.0):
        raise ValueError(f"center-tile-size must be in (0, 1], got {size}")
    x = (1.0 - size) / 2.0
    return f"{x:.6f},{x:.6f},{size:.6f},{size:.6f}"


def _grid_to_static_tiles(tiles_x: int, tiles_y: int,
                          overlap_x: float, overlap_y: float) -> list[str]:
    """Convert a regular tiles_x*tiles_y grid into a list of normalized
    'x,y,w,h' rectangles. Math: T = 1/(N - (N-1)*o); step = T*(1-o).
    For N==1 returns a single full-axis tile.
    """
    if tiles_x < 1 or tiles_y < 1:
        return []
    def axis(n: int, o: float):
        if n == 1:
            return [(0.0, 1.0)]
        T = 1.0 / (n - (n - 1) * o)
        S = T * (1.0 - o)
        return [(i * S, T) for i in range(n)]
    rects = []
    for (y, h) in axis(tiles_y, overlap_y):
        for (x, w) in axis(tiles_x, overlap_x):
            rects.append(f"{x:.6f},{y:.6f},{w:.6f},{h:.6f}")
    return rects


def DYNAMIC_TILE_CROPPER_PIPELINE(inner_pipeline: str, name: str,
                                  internal_offset: bool,
                                  iou_threshold: float,
                                  border_threshold: float,
                                  tiles_static: str = "",
                                  tiles_x: int = 0,
                                  tiles_y: int = 0,
                                  overlap_x: float = 0.0,
                                  overlap_y: float = 0.0) -> str:
    """Cropper+aggregator subgraph using `hailotilecropper_dynamic`.

    Same I/O contract as the upstream `TILE_CROPPER_PIPELINE` helper: bypass
    branch on `src_0`, cropped tiles on `src_1` fed through `inner_pipeline`,
    rejoined at `hailotileaggregator`. The capsfilter on the cropped branch
    pins format=RGB so the cropper's src_1 caps don't fixate on a YUV format
    mismatching the bypass/sink format (per the dynamic cropper demo).

    The installed `hailotilecropper_dynamic` only exposes `tiles-static`
    (the grid props were never landed in the C++ source), so we enumerate
    the requested grid as static rectangles in Python and concatenate them
    onto any extra `tiles_static` rectangles passed in. Pass
    `tiles_x=tiles_y=0` to disable the grid.
    """
    border = (
        f"border-threshold={str(border_threshold).lower()} "
        if border_threshold else ""
    )
    grid_rects = _grid_to_static_tiles(tiles_x, tiles_y, overlap_x, overlap_y) \
        if (tiles_x > 0 and tiles_y > 0) else []
    extra_rects = [r for r in tiles_static.split(";") if r.strip()] if tiles_static else []
    all_rects = grid_rects + extra_rects
    static_str = ";".join(all_rects)
    static_prop = f'tiles-static="{static_str}" ' if static_str else ""
    return (
        f"{QUEUE(name=f'{name}_input_q')} ! "
        f"hailotilecropper_dynamic name={name}_cropper "
        f"internal-offset={str(internal_offset).lower()} {static_prop}"
        f"hailotileaggregator name={name}_agg "
        f"flatten-detections=true iou-threshold={iou_threshold} {border}"
        f"{name}_cropper. ! {QUEUE(name=f'{name}_bypass_q')} ! {name}_agg.sink_0 "
        f"{name}_cropper. ! video/x-raw,format=RGB ! {inner_pipeline} ! {name}_agg.sink_1 "
        f"{name}_agg. ! {QUEUE(name=f'{name}_output_q')}"
    )


class GStreamerTilingRecordApp(GStreamerTilingApp):
    def on_eos(self):
        # Override the base class behavior of rebuilding the pipeline on EOS
        # for file sources. We want a single pass through the input.
        hailo_logger.info("EOS received; shutting down (looping disabled)")
        self.shutdown()

    def _add_tiling_arguments(self, parser):
        super()._add_tiling_arguments(parser)
        parser.add_argument(
            "--record",
            action="store_true",
            help="Enable recording branch (tee output to .mkv file). Off by default.",
        )
        parser.add_argument(
            "--output",
            default="/tmp/tiling_record.mkv",
            help="Path to the output .mkv recording (default: /tmp/tiling_record.mkv). Only used with --record.",
        )
        parser.add_argument(
            "--record-bitrate",
            type=int,
            default=5000,
            help="x264enc bitrate (kbps) for the recording branch (default: 5000). Only used with --record.",
        )
        parser.add_argument(
            "--score-threshold",
            type=float,
            default=0.3,
            help="Detection confidence threshold (on-chip nms-score-threshold). Default: 0.3",
        )
        parser.add_argument(
            "--include-center-tile",
            action="store_true",
            help="Append a centered square static tile to the grid. Useful for "
                 "drone-follow scenarios where the target is biased to the frame center.",
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
            help="Draw tile rectangles on the displayed frame (sets show-tiles=true on hailooverlay_community).",
        )

    def get_pipeline_string(self) -> str:
        source_pipeline = self.get_source_pipeline()

        score_threshold = self.options_menu.score_threshold
        detection_pipeline = INFERENCE_PIPELINE(
            hef_path=self.hef_path,
            post_process_so=self.post_process_so,
            post_function_name=self.post_function,
            batch_size=self.batch_size,
            config_json=self.labels_json,
            additional_params=f"nms-score-threshold={score_threshold}",
        )
        hailo_logger.info("Detection score threshold set to %.2f", score_threshold)

        if MANUAL_TILES is not None:
            tiles_x = MANUAL_TILES["tiles_x"]
            tiles_y = MANUAL_TILES["tiles_y"]
            overlap_x = MANUAL_TILES["overlap_x"]
            overlap_y = MANUAL_TILES["overlap_y"]
            hailo_logger.info(
                "MANUAL_TILES override: %dx%d grid, overlap (x=%.2f, y=%.2f)",
                tiles_x, tiles_y, overlap_x, overlap_y,
            )
        else:
            tiles_x = self.tiles_x
            tiles_y = self.tiles_y
            overlap_x = self.overlap_x
            overlap_y = self.overlap_y

        if self.use_multi_scale:
            hailo_logger.warning(
                "--multi-scale has no equivalent in hailotilecropper_dynamic; "
                "falling back to the %dx%d single-scale grid.", tiles_x, tiles_y,
            )

        # Grid is now driven by the plugin's own tiles-along-x/y-axis +
        # overlap-x/y-axis properties (mirroring upstream hailotilecropper).
        # Extras like the optional center tile go through tiles-static.
        tiles_static = ""
        if self.options_menu.include_center_tile:
            tiles_static = _center_tile_static(self.options_menu.center_tile_size)
            hailo_logger.info("Appending center tile (size=%.2f) -> %s",
                              self.options_menu.center_tile_size, tiles_static)

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

        display_branch = (
            f"{QUEUE(name='display_branch_q')} ! "
            f"videoconvert n-threads=2 name=display_convert qos=false ! "
            f"{QUEUE(name='display_sink_q')} ! "
            f"fpsdisplaysink name=hailo_display video-sink={self.video_sink} "
            f"sync={self.sync} text-overlay={self.show_fps} signal-fps-measurements=true"
        )

        head = (
            f"{source_pipeline} ! "
            f"{tile_cropper_pipeline} ! "
            f"{user_callback_pipeline} ! "
            f"{overlay_pipeline}"
        )

        if self.options_menu.record:
            output = self.options_menu.output
            bitrate = self.options_menu.record_bitrate
            # Generous pre-encoder queue absorbs x264enc bursts so the recording
            # branch doesn't backpressure the source / tee.
            file_branch = (
                f"queue name=record_pre_enc_q max-size-buffers=200 max-size-bytes=0 "
                f"max-size-time=0 leaky=no ! "
                f"{FILE_SINK_PIPELINE(output_file=output, bitrate=bitrate)}"
            )
            pipeline_string = (
                f"{head} ! "
                f"tee name=record_tee "
                f"record_tee. ! {display_branch} "
                f"record_tee. ! {file_branch}"
            )
            hailo_logger.info("Recording tiling output to %s (bitrate %d kbps)", output, bitrate)
        else:
            pipeline_string = f"{head} ! {display_branch}"
            hailo_logger.info("Recording disabled (pass --record to enable)")

        hailo_logger.debug("Pipeline string: %s", pipeline_string)
        return pipeline_string


def main() -> None:
    parser = get_pipeline_parser()
    parser.set_defaults(width=1280, height=800)

    user_data = app_callback_class()
    app = GStreamerTilingRecordApp(passthrough_callback, user_data, parser=parser)
    app.run()


if __name__ == "__main__":
    print("Starting Hailo Tiling Record App (1280x800; pass --record to enable recording)...")
    main()
