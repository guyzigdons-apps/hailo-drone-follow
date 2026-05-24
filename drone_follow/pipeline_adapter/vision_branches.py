"""Pipeline output branches for drone-follow.

After source -> tile_cropper(detection) -> user_callback, the pipeline tees
into up to four branches. Only the ones the operator opts into are built::

    tee output_tee
        ├─ [--webui]   : MJPEG appsink (clean frames; client renders bbox)
        ├─ [--openhd]  : RTP H.264 over UDP to an OpenHD ground station
        └─ [--display / --record]
                       : identity local_meta_id  (pad probe: replace the
                                                  target's HailoDetection
                                                  with one whose class_id
                                                  is TARGET_OVERLAY_CLASS_ID
                                                  so the per-class YAML
                                                  style applies)
                       : hailooverlay_community show-tiles=false
                                                style-config=<yaml>
                       : tee local_tee
                            ├─ [--display] : videoconvert + fpsdisplaysink
                            └─ [--record]  : valve + H.264 + matroskamux + filesink

All vision work uses GStreamer-native elements + Hailo metadata APIs;
no pad probe maps the buffer pixels.

Tile-bbox suppression is handled at the element level
(``show-tiles=false``). Per-detection emphasis (color + line thickness)
is delegated to the community overlay's YAML style-config: the pad probe
just retags the target detection with a sentinel class_id; the YAML rule
under ``styles.<TARGET_OVERLAY_CLASS_ID>`` defines what that class looks
like (green, thicker line, etc.).
"""

from __future__ import annotations

import logging
import os
import shutil
import subprocess
import threading
from typing import Optional, Tuple

import hailo

LOGGER = logging.getLogger(__name__)


class TargetCrossState:
    """Thread-safe holder for the target's bbox center + mode.

    Populated by the ``highlight_target`` pad probe on each buffer (running
    on the GStreamer streaming thread). Read by the ``cairooverlay`` draw
    callback (running on the same thread, but the lock keeps it correct
    if rendering ever moves off-thread).

    ``cx``/``cy`` are normalized [0..1] image coords (matching the UI's
    convention). ``mode`` is one of ``"LOCKED"`` / ``"AUTO"`` / ``None``
    so the cairo overlay can draw a small state badge alongside the cross.
    ``None`` everywhere ⇒ no target visible this frame ⇒ no cross drawn.
    """

    __slots__ = ("_lock", "_cx", "_cy", "_mode")

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cx: Optional[float] = None
        self._cy: Optional[float] = None
        self._mode: Optional[str] = None

    def set(self, cx: Optional[float], cy: Optional[float],
            mode: Optional[str]) -> None:
        with self._lock:
            self._cx, self._cy, self._mode = cx, cy, mode

    def clear(self) -> None:
        self.set(None, None, None)

    def get(self) -> Tuple[Optional[float], Optional[float], Optional[str]]:
        with self._lock:
            return self._cx, self._cy, self._mode

# Sentinel class_id used to retag the locked/auto target detection on
# the local branch. The YAML style-config maps this to a thicker green
# bbox; all other detections keep their default class_id and inherit
# the element-level (thin) line thickness.
TARGET_OVERLAY_CLASS_ID = 99

# Packed 0xRRGGBB colour drawn around every non-target detection on the
# local branch. Attached via an ``overlay_color`` HailoClassification
# (read by hailooverlay_community when ``use-custom-colors=true``).
NON_TARGET_BBOX_COLOR_RGB = 0xFFFFFF  # white

# Path to the bundled overlay style YAML. Resolved relative to this file
# so the module works from a checkout, an editable install, or a copied
# tree without env-var fiddling.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_STYLE_CONFIG = os.path.normpath(
    os.path.join(_THIS_DIR, "..", "..", "configs", "overlay_style.yaml")
)
DEFAULT_OVERLAY_STYLE_CONFIG = (
    _DEFAULT_STYLE_CONFIG if os.path.isfile(_DEFAULT_STYLE_CONFIG) else ""
)


# ---------------------------------------------------------------------------
# H.264 encoder selection
# ---------------------------------------------------------------------------

_H264_ENCODER_TEMPLATE: Optional[str] = None


def select_h264_encoder(bitrate_bps: int, bitrate_kbps: int) -> str:
    """Return a launch fragment for the best available H.264 SW encoder.

    Tries ``x264enc`` first (gst-plugins-ugly — better quality / latency,
    historically preferred on RPi for OpenHD), then falls back to
    ``openh264enc`` (gst-plugins-bad — pre-installed on most RPi images).
    The result is cached so repeated calls don't re-shell out to
    ``gst-inspect-1.0``.

    Both encoders accept a ``bitrate`` property but with different units;
    we render the right value for whichever encoder is chosen, and
    callers that update bitrate at runtime (the OpenHD bridge) need to
    handle the unit difference themselves via ``encoder.get_factory().get_name()``.
    """
    global _H264_ENCODER_TEMPLATE
    if _H264_ENCODER_TEMPLATE is None:
        _H264_ENCODER_TEMPLATE = _detect_h264_encoder()
    return _H264_ENCODER_TEMPLATE.format(bps=bitrate_bps, kbps=bitrate_kbps)


def _detect_h264_encoder() -> str:
    def _has(elem: str) -> bool:
        if not shutil.which("gst-inspect-1.0"):
            return False
        rc = subprocess.run(["gst-inspect-1.0", elem],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL).returncode
        return rc == 0

    if _has("x264enc"):
        return ("x264enc tune=zerolatency speed-preset=ultrafast "
                "bitrate={kbps}")
    if _has("openh264enc"):
        return ("openh264enc bitrate={bps} complexity=low "
                "rate-control=bitrate gop-size=15")
    raise RuntimeError(
        "No H.264 encoder found. Install one of:\n"
        "  sudo apt install gstreamer1.0-plugins-ugly   # provides x264enc\n"
        "  sudo apt install gstreamer1.0-plugins-bad    # provides openh264enc"
    )


# ---------------------------------------------------------------------------
# Branch fragments
# ---------------------------------------------------------------------------

def webui_branch(ui_fps: int) -> str:
    """MJPEG appsink for the web UI. Clean frames (no overlay) so the
    browser can render its own interactive SVG bboxes on top.
    """
    return (
        "queue name=mjpeg_branch_q leaky=downstream max-size-buffers=3 ! "
        "videoconvert n-threads=2 ! "
        f"videorate max-rate={ui_fps} ! "
        f"video/x-raw,framerate={ui_fps}/1 ! "
        "jpegenc quality=70 ! "
        "appsink name=mjpeg_sink sync=false drop=true emit-signals=true"
    )


def openhd_branch(port: int, bitrate_kbps: int) -> str:
    """OpenHD branch — H.264 SW encode + RTP + UDP sink. Element name
    ``openhd_stream_encoder`` is preserved for the OpenHD bridge's
    bitrate-update probe.
    """
    bitrate_bps = bitrate_kbps * 1000
    encoder = select_h264_encoder(bitrate_bps, bitrate_kbps)
    factory, props = encoder.split(" ", 1)
    # Streaming-only tuning. Three load-bearing properties on x264enc:
    #
    # * ``key-int-max=5`` — IDR every 5 frames so a single lost UDP packet
    #   only corrupts ~0.2 s of video before the next IDR refresh, vs.
    #   the ~10 s default GOP.
    # * ``threads=2`` — explicit two-thread encoding so x264 doesn't run
    #   single-threaded on RPi5 at the bitrates QOpenHD asks for.
    # * ``sliced-threads=false`` — **load-bearing, do not remove**. With
    #   ``tune=zerolatency``, libx264 internally enables sliced-threads
    #   (i_sliced_threads=1), splitting each frame into 2 slices when
    #   threads=2. On a lossy radio link this produces visible
    #   "half-frame corruption" (one slice survives, the other turns
    #   into blocky garbage) instead of clean whole-frame drops; we
    #   want the latter. Confirmed by NAL-unit count regression: NEW
    #   produced 2x slices/frame vs OLD until this was restored.
    #
    # Not applied to the recording branch — slice-loss artifacts don't
    # apply to a local file.
    if factory == "x264enc":
        stream_props = "threads=2 key-int-max=5 sliced-threads=false"
    else:  # openh264enc — no threads knob, gop-size already set in template
        stream_props = ""
    # All queues in this branch are pinned to max-size-buffers=3 and
    # explicitly disable the byte/time limits (defaults are 10 MB / 1 s
    # in GStreamer). Without these explicit caps the convert/enc queues
    # would default to ``max-size-buffers=200`` and ``max-size-time=1s``;
    # under CPU pressure x264 falls behind, ~1 s of frames pile up, the
    # queue blocks, then drains in a burst — visible on the radio link
    # as the periodic "resetting every second" stutter. The pre-refactor
    # code used the project ``QUEUE()`` helper which already set these
    # caps; we restore the same shape inline.
    return (
        "queue name=openhd_branch_q leaky=downstream max-size-buffers=3 "
        "max-size-bytes=0 max-size-time=0 ! "
        "queue name=openhd_stream_convert_q max-size-buffers=3 "
        "max-size-bytes=0 max-size-time=0 ! "
        "videoconvert n-threads=2 ! video/x-raw,format=I420 ! "
        "queue name=openhd_stream_enc_q max-size-buffers=3 "
        "max-size-bytes=0 max-size-time=0 ! "
        f"{factory} name=openhd_stream_encoder {props} {stream_props} ! "
        "rtph264pay config-interval=1 pt=96 mtu=1440 ! "
        f"udpsink host=127.0.0.1 port={port} sync=false async=false"
    )


def _display_subbranch(video_sink: str, sync: str, show_fps: str) -> str:
    return (
        "queue name=display_branch_q leaky=downstream max-size-buffers=3 ! "
        "videoconvert name=hailo_display_videoconvert n-threads=2 qos=false ! "
        "queue name=hailo_display_q ! "
        f"fpsdisplaysink name=hailo_display video-sink={video_sink} "
        f"sync={sync} text-overlay={show_fps} signal-fps-measurements=true"
    )


def _record_subbranch(record_output: str, record_bitrate_kbps: int) -> str:
    bitrate_bps = record_bitrate_kbps * 1000
    encoder = select_h264_encoder(bitrate_bps, record_bitrate_kbps)
    # splitmuxsink writes a fresh .mkv per recording session: the
    # ``valve`` gates frames, and ``split-now`` is emitted on stop to
    # finalise the current file. ``max-size-time=0 max-size-bytes=0``
    # disables automatic splitting, so file rotation is purely operator
    # driven via the start/stop callbacks. The ``location`` here is just
    # the template default; the actual per-fragment path is generated by
    # the ``format-location-full`` signal handler (see
    # ``DroneFollowTilingApp._on_record_format_location``).
    #
    # async-handling/async-finalize=true and the inner sink-properties
    # (``async=false sync=false``) keep splitmuxsink out of pipeline
    # preroll synchronisation — required because the valve is closed at
    # launch, so no buffer would reach the file sink to preroll, which
    # would otherwise stall the entire pipeline including the webui
    # branch.
    return (
        "queue name=record_branch_q leaky=downstream max-size-buffers=3 ! "
        "valve name=record_valve drop=true ! "
        "videoconvert n-threads=2 ! "
        f"{encoder} ! "
        "h264parse config-interval=1 ! "
        "splitmuxsink name=file_sink "
        f'location="{record_output}" '
        "max-size-time=0 max-size-bytes=0 "
        "async-handling=true async-finalize=true "
        "muxer-factory=matroskamux "
        'sink-properties="sink,async=false,sync=false"'
    )


def local_branch(*, display: bool, record: bool, record_output: Optional[str],
                 record_bitrate_kbps: int, video_sink: str, sync: str,
                 show_fps: str) -> str:
    """Display + record local branch.

    Both sub-branches share an upstream ``identity name=local_meta_id``
    (pad-probe attachment point for :func:`highlight_target`) and a
    single ``hailooverlay_community``.
    """
    if not display and not record:
        raise ValueError("local_branch requires display or record (or both)")

    overlay_props = (
        "show-tiles=false line-thickness=2 "
        "font-thickness=1 text-background=true "
        "use-custom-colors=true"
    )
    if DEFAULT_OVERLAY_STYLE_CONFIG:
        overlay_props += f' style-config="{DEFAULT_OVERLAY_STYLE_CONFIG}"'
    # cairooverlay draws the target cross + state badge on top of the bbox
    # overlay. videoconvert wrappers around it because cairooverlay needs
    # BGRA/ARGB (it can't write into Hailo's NV12 native format).
    head = (
        "queue name=local_branch_q leaky=downstream max-size-buffers=3 ! "
        "identity name=local_meta_id ! "
        "queue name=hailo_overlay_q ! "
        f"hailooverlay_community name=hailo_overlay {overlay_props} ! "
        "videoconvert n-threads=2 ! video/x-raw,format=BGRA ! "
        "cairooverlay name=target_cross_overlay"
    )

    subs = []
    if display:
        subs.append(_display_subbranch(video_sink, sync, show_fps))
    if record:
        if record_output is None:
            raise ValueError("record=True requires a record_output path")
        subs.append(_record_subbranch(record_output, record_bitrate_kbps))

    if len(subs) == 1:
        return f"{head} ! {subs[0]}"
    return (f"{head} ! tee name=local_tee "
            + " ".join(f"local_tee. ! {sub}" for sub in subs))


def assemble_output_stage(*, display: bool, record: bool, openhd: bool,
                          webui: bool, openhd_port: int = 5500,
                          openhd_bitrate_kbps: int = 3917,
                          ui_fps: int = 10, record_output: Optional[str] = None,
                          record_bitrate_kbps: int = 5000,
                          video_sink: str = "autovideosink", sync: str = "false",
                          show_fps: str = "false",
                          fakesink_sync: str = "false") -> str:
    """Build the full output stage (everything after user_callback).

    Returns a single launch-string. If exactly one output is requested,
    returns just that branch. If multiple, wraps them in a top-level
    ``tee output_tee``. If none, returns a fakesink so the pipeline is
    still well-formed.
    """
    branches = []
    if webui:
        branches.append(webui_branch(ui_fps))
    if openhd:
        branches.append(openhd_branch(openhd_port, openhd_bitrate_kbps))
    if display or record:
        branches.append(local_branch(
            display=display, record=record,
            record_output=record_output,
            record_bitrate_kbps=record_bitrate_kbps,
            video_sink=video_sink, sync=sync, show_fps=show_fps,
        ))

    if not branches:
        return f"fakesink name=fake_sink sync={fakesink_sync}"
    if len(branches) == 1:
        return branches[0]
    return ("tee name=output_tee "
            + " ".join(f"output_tee. ! {b}" for b in branches))


# ---------------------------------------------------------------------------
# Metadata pad probe (display/record branch)
# ---------------------------------------------------------------------------

def _tag_white(det):
    """Attach an ``overlay_color`` classification (white) so the local
    overlay renders this detection with a white bbox. Idempotent.
    """
    for cls in det.get_objects_typed(hailo.HAILO_CLASSIFICATION):
        if cls.get_classification_type() == "overlay_color":
            return
    det.add_object(hailo.HailoClassification(
        "overlay_color",            # classification type read by overlay
        NON_TARGET_BBOX_COLOR_RGB,  # class_id = packed 0xRRGGBB (white)
        "",                         # label (unused on packed-int path)
        0.0,                        # confidence
    ))


def highlight_target(pad, info, target_state, cross_state=None):
    """Style the local-branch detections so the operator can tell the
    locked/auto target apart from everyone else at a glance:

    * **Target**: bbox center captured into ``cross_state`` for the
      downstream cairo overlay to draw a target cross at, and the target's
      HailoDetection is REMOVED from the ROI so ``hailooverlay_community``
      doesn't draw a bbox over the cross.
    * **Non-target detections**: an ``overlay_color`` classification of
      :data:`NON_TARGET_BBOX_COLOR_RGB` (white) is attached so they render
      in white at the element-level (thin) line thickness.

    Pure metadata work — no pixel buffers are mapped.

    Backwards compat: ``cross_state`` is optional. With ``cross_state=None``
    the probe falls back to the previous behaviour (retag target with
    :data:`TARGET_OVERLAY_CLASS_ID` so the YAML green-bbox rule fires).
    """
    import gi  # noqa: F401 — local import keeps gi out of module-load time
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst

    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    target_id = target_state.get_target() if target_state is not None else None
    roi = hailo.get_roi_from_buffer(buffer)
    target_det = None
    others = []

    for det in roi.get_objects_typed(hailo.HAILO_DETECTION):
        is_target = False
        if target_id is not None and target_id > 0:
            for uid in det.get_objects_typed(hailo.HAILO_UNIQUE_ID):
                if uid.get_id() == target_id:
                    is_target = True
                    break
        if is_target and det.get_class_id() != TARGET_OVERLAY_CLASS_ID:
            target_det = det
        elif is_target:  # already retagged on a previous pass
            target_det = det
        else:
            others.append(det)

    # Tag every non-target detection white so it stands apart from the
    # target's marker. Idempotent across probe re-runs.
    for det in others:
        _tag_white(det)

    if target_det is not None:
        bbox = target_det.get_bbox()
        cx = bbox.xmin() + bbox.width() / 2.0
        cy = bbox.ymin() + bbox.height() / 2.0

        if cross_state is not None:
            # Capture target center for the downstream cairooverlay draw
            # callback. Mode hint follows whether this is an explicit lock
            # vs the auto-acquired largest person.
            try:
                is_locked = bool(target_state.is_explicit_lock())
            except Exception:  # noqa: BLE001 — older FollowTargetState shapes
                is_locked = False
            cross_state.set(cx, cy, "LOCKED" if is_locked else "AUTO")
            # Drop the target detection so hailooverlay doesn't draw a bbox
            # over the cross. The other branches' shared metadata is fine —
            # the orchestrator's view of state was populated upstream in
            # user_callback, before the output_tee.
            roi.remove_object(target_det)
        else:
            # Legacy path: retag with class_id 99 so the YAML thick-green
            # rule applies (preserves callers that don't pass cross_state).
            new_det = hailo.HailoDetection(
                bbox, TARGET_OVERLAY_CLASS_ID, target_det.get_label(),
                target_det.get_confidence(),
            )
            for child in list(target_det.get_objects()):
                if (isinstance(child, hailo.HailoClassification)
                        and child.get_classification_type() == "overlay_color"):
                    continue
                try:
                    new_det.add_object(child)
                except Exception:  # noqa: BLE001 — best-effort re-attach
                    pass
            roi.remove_object(target_det)
            roi.add_object(new_det)
    elif cross_state is not None:
        # No target visible this frame ⇒ stop drawing a stale cross.
        cross_state.clear()

    return Gst.PadProbeReturn.OK


# Backwards-compat alias — older callers may still import the old name.
strip_tiles_and_highlight_target = highlight_target


def wire_local_meta_probe(pipeline, target_state, cross_state=None) -> bool:
    """Attach :func:`highlight_target` to the ``local_meta_id`` identity
    element if it exists in the pipeline.

    With ``cross_state`` provided, also connects the ``target_cross_overlay``
    ``cairooverlay``'s ``draw`` signal to :func:`draw_target_cross` so the
    captured target center renders as a cross + small state badge on the
    display/record video.

    Returns True if a probe was attached; False if the element is absent
    (no display/record branch was built). Safe to call after every
    pipeline rebuild.
    """
    ident = pipeline.get_by_name("local_meta_id")
    if ident is None:
        return False
    src_pad = ident.get_static_pad("src")
    if src_pad is None:
        return False
    import gi  # noqa: F401
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    src_pad.add_probe(
        Gst.PadProbeType.BUFFER,
        highlight_target,
        target_state,
        cross_state,
    )

    if cross_state is not None:
        overlay = pipeline.get_by_name("target_cross_overlay")
        if overlay is not None:
            overlay.connect("draw", draw_target_cross, cross_state)
        else:
            LOGGER.warning(
                "[overlay] target_cross_overlay element not found in pipeline "
                "— cross will not be drawn on display/record video."
            )
    return True


# Cross geometry — matched to the UI's SVG cross in App.jsx for visual
# parity. Arm length scales loosely with video size so it stays readable
# at any resolution; floor + ceiling keep it visible at 480p and
# unobtrusive at 4K.
_CROSS_ARM_FRACTION = 0.025      # 2.5 % of min(width, height) per arm half
_CROSS_ARM_MIN_PX = 12
_CROSS_ARM_MAX_PX = 32
_CROSS_HALO_WIDTH_PX = 6
_CROSS_LINE_WIDTH_PX = 3

# Brand colours — match the SVG cross in the React UI (App.jsx).
_CROSS_GREEN = (0x80 / 255, 0xF0 / 255, 0x60 / 255, 1.0)  # locked / auto
_CROSS_HALO = (0.0, 0.0, 0.0, 0.75)                      # behind every stroke
_BADGE_BG = (0.0, 0.0, 0.0, 0.55)                        # state badge bg


def draw_target_cross(overlay, context, timestamp, duration, cross_state):
    """``cairooverlay::draw`` handler — renders the target cross + a small
    state badge.

    Reads ``cross_state`` (populated upstream by :func:`highlight_target`).
    No draw when the state is empty (no target visible) — the underlying
    video frame passes through untouched.
    """
    cx_norm, cy_norm, mode = cross_state.get()
    if cx_norm is None or cy_norm is None:
        return

    # cairooverlay::draw doesn't supply width/height directly; read them
    # off the element's sink pad caps. Cached on the element on first draw.
    width, height = _cached_overlay_dims(overlay)
    if width is None or height is None:
        return

    cx = cx_norm * width
    cy = cy_norm * height
    arm = max(
        _CROSS_ARM_MIN_PX,
        min(_CROSS_ARM_MAX_PX,
            int(_CROSS_ARM_FRACTION * min(width, height))),
    )

    context.set_line_cap(2)  # cairo.LINE_CAP_ROUND
    # Black halo so the cross stays readable on any background.
    context.set_source_rgba(*_CROSS_HALO)
    context.set_line_width(_CROSS_HALO_WIDTH_PX)
    context.move_to(cx - arm, cy)
    context.line_to(cx + arm, cy)
    context.stroke()
    context.move_to(cx, cy - arm)
    context.line_to(cx, cy + arm)
    context.stroke()
    # Green cross on top.
    context.set_source_rgba(*_CROSS_GREEN)
    context.set_line_width(_CROSS_LINE_WIDTH_PX)
    context.move_to(cx - arm, cy)
    context.line_to(cx + arm, cy)
    context.stroke()
    context.move_to(cx, cy - arm)
    context.line_to(cx, cy + arm)
    context.stroke()

    # State badge — small dark rectangle with the current mode label.
    if mode:
        _draw_state_badge(context, mode, width, height)


def _draw_state_badge(context, mode, width, height) -> None:
    """Top-left badge: dark pill with mode text."""
    text = mode.upper()
    pad_x = 8
    pad_y = 4
    font_size = max(12, int(height * 0.022))
    context.select_font_face("sans-serif", 0, 1)  # NORMAL, BOLD
    context.set_font_size(font_size)
    _, _, text_w, text_h, _, _ = context.text_extents(text)
    box_w = int(text_w + 2 * pad_x)
    box_h = int(text_h + 2 * pad_y)
    box_x = 16
    box_y = 16
    # Background pill.
    context.set_source_rgba(*_BADGE_BG)
    context.rectangle(box_x, box_y, box_w, box_h)
    context.fill()
    # Text — green to match the cross.
    context.set_source_rgba(*_CROSS_GREEN)
    context.move_to(box_x + pad_x, box_y + pad_y + text_h)
    context.show_text(text)


def _cached_overlay_dims(overlay):
    """Read width/height off the cairooverlay's negotiated sink caps once
    and cache on the element. Returns ``(None, None)`` until caps are set.
    """
    cached = getattr(overlay, "_cached_dims", None)
    if cached is not None:
        return cached
    pad = overlay.get_static_pad("sink")
    if pad is None:
        return None, None
    caps = pad.get_current_caps()
    if caps is None:
        return None, None
    s = caps.get_structure(0)
    ok_w, w = s.get_int("width")
    ok_h, h = s.get_int("height")
    if not (ok_w and ok_h):
        return None, None
    overlay._cached_dims = (w, h)
    return overlay._cached_dims
