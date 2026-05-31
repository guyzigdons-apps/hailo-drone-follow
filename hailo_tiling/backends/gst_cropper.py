"""GstCropperBackend — canonical GStreamer research path (Plan 6 Task B4).

Drives the canonical tiled pipeline
(``hailotilecropper_dynamic -> hailonet -> hailofilter``) with an arbitrary set
of crops (converted to normalized ``tiles-static``) and returns per-crop
tile-local-normalized detections via the same pad-probe extraction as
``scripts/cache_gst_replay_gate.py``.

Wrapped in ``CachingBackend`` this is the warm-and-run path for **dynamic /
lever** configs whose ROI tiles cannot be pre-warmed — so every paper row,
including the live ones, runs through GStreamer.

Pipeline-string construction (which crops -> which ``tiles-static``, element
order) is pure and chip-free; ``infer`` requires a Hailo chip.
"""
from __future__ import annotations

from typing import Any, Sequence

from ..types import CropRect, Det
from .backend import InferenceBackend

# Canonical postprocess .so + function for the 4-class yolov8 VGA HEF (same as
# the replay gate / warmer).
_DEFAULT_POST_SO = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"
_DEFAULT_POST_FN = "filter"


def crop_to_norm_tile(crop: CropRect, src_w: int, src_h: int) -> str:
    """Convert a source-pixel CropRect back to a normalized ``x,y,w,h`` tile
    string for ``tiles-static``. Inverse of ``tile_norm_to_source_px``'s scale
    (the cropper re-derives the source-pixel rect via the same truncate rule,
    so feeding these normalized tiles reproduces the crop key)."""
    x = crop.x / src_w
    y = crop.y / src_h
    w = crop.w / src_w
    h = crop.h / src_h
    return f"{x:.6f},{y:.6f},{w:.6f},{h:.6f}"


class GstCropperBackend(InferenceBackend):
    def __init__(
        self,
        hef: str,
        post_so: str | None = None,
        source_w: int = 0,
        source_h: int = 0,
        post_fn: str | None = None,
    ):
        self.hef = hef
        self.post_so = post_so or _DEFAULT_POST_SO
        self.post_fn = post_fn or _DEFAULT_POST_FN
        self.source_w = int(source_w)
        self.source_h = int(source_h)

    # -- pipeline construction (pure / chip-free) --------------------------

    def tiles_static(self, crops: Sequence[CropRect]) -> str:
        """The ``tiles-static`` string for an ordered crop batch."""
        if self.source_w <= 0 or self.source_h <= 0:
            raise ValueError("source_w/source_h must be set to build tiles-static")
        return ";".join(
            crop_to_norm_tile(c, self.source_w, self.source_h) for c in crops
        )

    def build_pipeline_string(self, video: str, crops: Sequence[CropRect]) -> str:
        """Build the canonical cropper pipeline string for ``crops``.

        Mirrors ``scripts/cache_gst_replay_gate.py``'s pass-1 construction
        (cropper bypass on sink_0, tiles on sink_1 through hailonet +
        hailofilter), with NO extra ``videoscale`` (the cropper keeps the
        resize internally — the Task-1 caps finding).
        """
        tiles = self.tiles_static(crops)
        inner = (
            "videoconvert ! "
            f"hailonet name=gcb_hailonet hef-path={self.hef} "
            "batch-size=1 force-writable=true ! "
            f"hailofilter name=gcb_tap so-path={self.post_so} "
            f"function-name={self.post_fn} qos=false"
        )
        cropper = (
            "queue name=gcb_in_q ! "
            f'hailotilecropper_dynamic name=gcb_tc internal-offset=true '
            f'tiles-static="{tiles}" '
            "hailotileaggregator name=gcb_agg flatten-detections=true "
            "iou-threshold=0.3 "
            "gcb_tc. ! queue name=gcb_bypass_q ! gcb_agg.sink_0 "
            f"gcb_tc. ! video/x-raw,format=RGB ! {inner} ! gcb_agg.sink_1 "
            "gcb_agg. ! queue name=gcb_out_q"
        )
        return (
            f'filesrc location="{video}" ! decodebin ! '
            "videoconvert ! video/x-raw,format=RGB ! "
            f"{cropper} ! fakesink name=gcb_sink sync=false"
        )

    # -- inference (chip) ---------------------------------------------------

    def infer(
        self,
        frame: Any,
        crops: Sequence[CropRect],
        frame_idx: int,
    ) -> list[list[Det]]:
        """Run the cropper pipeline for ``crops`` and return per-crop tile-local
        normalized dets in input order.

        Requires a Hailo chip. ``frame`` is the source video path (the cropper
        decodes the frame itself); ``frame_idx`` selects which decoded frame to
        score. Reuses the replay gate's pad-probe extraction.
        """
        if not crops:
            return []
        if self.source_w <= 0 or self.source_h <= 0:
            raise ValueError("source_w/source_h must be set for infer")

        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import GLib, Gst

        import hailo  # noqa: F401

        Gst.init(None)
        n_tiles = len(crops)
        pipe_str = self.build_pipeline_string(str(frame), crops)
        pipeline = Gst.parse_launch(pipe_str)
        tap = pipeline.get_by_name("gcb_tap")
        srcpad = tap.get_static_pad("src")

        per_crop: list[list[Det]] = []
        counter = {"n": 0}
        loop = GLib.MainLoop()

        def _dets_from_buffer(buf) -> list[Det]:
            roi = hailo.get_roi_from_buffer(buf)
            objs = roi.get_objects_typed(hailo.HAILO_DETECTION)
            out: list[Det] = []
            for d in objs:
                bb = d.get_bbox()
                out.append(
                    Det(
                        cls=int(d.get_class_id()),
                        score=float(d.get_confidence()),
                        x=float(bb.xmin()),
                        y=float(bb.ymin()),
                        w=float(bb.width()),
                        h=float(bb.height()),
                    )
                )
            return out

        # Score only the requested frame_idx's tiles (n_tiles buffers per frame
        # at the tap, in cropper tile order).
        first_buf = frame_idx * n_tiles

        def probe(pad, info):
            buf = info.get_buffer()
            if buf is None:
                return Gst.PadProbeReturn.OK
            idx = counter["n"]
            counter["n"] += 1
            if first_buf <= idx < first_buf + n_tiles:
                per_crop.append(_dets_from_buffer(buf))
                if len(per_crop) >= n_tiles:
                    loop.quit()
            return Gst.PadProbeReturn.OK

        srcpad.add_probe(Gst.PadProbeType.BUFFER, probe)
        bus = pipeline.get_bus()
        bus.add_signal_watch()

        def on_msg(_bus, msg):
            if msg.type in (Gst.MessageType.EOS, Gst.MessageType.ERROR):
                loop.quit()

        bus.connect("message", on_msg)
        pipeline.set_state(Gst.State.PLAYING)
        try:
            loop.run()
        finally:
            pipeline.set_state(Gst.State.NULL)

        # Pad/truncate to exactly n_tiles (defensive).
        while len(per_crop) < n_tiles:
            per_crop.append([])
        return per_crop[:n_tiles]
