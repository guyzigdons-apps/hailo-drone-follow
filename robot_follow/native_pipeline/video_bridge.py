"""In-process GStreamer bridge: multicast H264 → JPEG → web UI.

Closes the "no live video in the web UI" gap in native-pipeline mode.
The C++ binary emits sink1 (720p H264) to a multicast group; this
bridge joins the group, decodes, re-encodes as JPEG, and pushes each
frame into ``SharedUIState.update_frame()`` — the same hook the legacy
Python pipeline uses, so ``App.jsx`` and the SSE handler don't know
the difference.

Why sink1 and not sink0:
    sink0 is 4K (3840×2160). ``avdec_h264`` is the only H264 decoder on
    the H15 (no hw decode element exposed), and software-decoding 4K at
    30 fps on the A53 cores is hot. sink1 is 720p, much cheaper, and
    plenty for the UI preview.

Why in-process Gst (not gst-launch subprocess):
    Subprocess stdout-piping of JPEGs needs custom framing (SOI/EOI
    scan, length prefix). With ``Gst.appsink`` we get one buffer per
    frame straight from GStreamer — no framing, no parsing, no
    subprocess lifecycle.
"""

from __future__ import annotations

import logging
import threading
from typing import Optional

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib  # noqa: E402

LOGGER = logging.getLogger(__name__)

DEFAULT_MULTICAST_GROUP = "239.255.0.50"
# sink1's port. Must match the C++ side's port_from_stream_id formula
# (base_port + stream_num*2 → 5000 + 1*2 = 5002).
DEFAULT_UI_VIDEO_PORT = 5002
DEFAULT_MULTICAST_IFACE = "eth0"


_gst_init_lock = threading.Lock()
_gst_initialized = False


def _ensure_gst_init() -> None:
    """Init Gst exactly once across the process.

    Safe to call from anywhere — the lock makes parallel-init races
    a no-op, and Gst.init itself is idempotent but the bookkeeping
    flag keeps the log noise down.
    """
    global _gst_initialized
    with _gst_init_lock:
        if not _gst_initialized:
            Gst.init(None)
            _gst_initialized = True


class NativeVideoBridge:
    """Run a GStreamer pipeline that turns multicast H264 into UI JPEGs.

    Lifetime is bounded by ``start()`` / ``stop()`` — the GLib MainLoop
    runs on a daemon thread so it never blocks shutdown. Each decoded
    frame becomes a JPEG and is handed to ``ui_state.update_frame()``,
    which is the same path the Python pipeline uses; the web UI's MJPEG
    element and SSE consumers see frames identically in both modes.

    Frame size is downscaled to 960×540 by default. The UI doesn't need
    more — App.jsx fits-to-container anyway — and the smaller payload
    keeps MJPEG bandwidth and jpegenc cost low.
    """

    def __init__(
        self,
        ui_state,
        *,
        multicast_group: str = DEFAULT_MULTICAST_GROUP,
        port: int = DEFAULT_UI_VIDEO_PORT,
        multicast_iface: str = DEFAULT_MULTICAST_IFACE,
        target_width: int = 640,
        target_height: int = 360,
        target_framerate: int = 15,
        jpeg_quality: int = 70,
    ) -> None:
        self._ui_state = ui_state
        self._multicast_group = multicast_group
        self._port = port
        self._multicast_iface = multicast_iface
        self._target_width = target_width
        self._target_height = target_height
        self._target_framerate = target_framerate
        self._jpeg_quality = jpeg_quality
        self._pipeline: Optional[Gst.Pipeline] = None
        self._loop: Optional[GLib.MainLoop] = None
        self._loop_thread: Optional[threading.Thread] = None
        self._frame_count = 0

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    def start(self) -> None:
        if self._pipeline is not None:
            return
        _ensure_gst_init()

        # Build pipeline as a string — easier to read and debug than the
        # equivalent Gst.ElementFactory.make chain. Caps on udpsrc tell
        # rtph264depay how to interpret the RTP payload (otherwise it
        # refuses to negotiate and the pipeline never reaches PLAYING).
        # rtpjitterbuffer with 100 ms latency tolerates UDP reorder/loss
        # without adding much wall-clock delay. appsink with
        # max-buffers=1 + drop=true means the UI gets the *latest* frame
        # at whatever rate update_frame can keep up; older frames are
        # dropped pre-callback so we never queue stale video.
        # clockoverlay burns the H15 wall clock into each frame so you can
        # eyeball end-to-end video latency against the browser's clock:
        # `video says 14:30:42, browser says 14:30:48` → 6 s of lag, no
        # external stopwatch needed. Position it after videoscale so the
        # font scales nicely against the downscaled output (rendering on
        # full-res would be larger than needed). Pango font size keeps
        # the overlay readable on a 960×540 frame.
        # Two anti-backlog measures, since avdec_h264 (sw) on the A53
        # can't keep up with the 30 fps 720p source:
        #
        # 1. buffer-size=524288 caps the kernel UDP socket buffer at
        #    ~512 KB. When full, the kernel drops the OLDEST packets
        #    (FIFO-tail drop). At ~5 Mbps that's ~0.8 s of buffering
        #    max — bounded latency, no minutes-deep queue.
        #
        # 2. leaky=downstream max-size-buffers=2 on the queue right
        #    before avdec_h264 makes the decode stage non-blocking:
        #    if it lags, h264 nal units get dropped (well, leaked) at
        #    the queue boundary instead of backpressuring the
        #    jitterbuffer. Two-buffer slack avoids killing the next
        #    keyframe pair. max-size-time/bytes=0 means buffers is
        #    the only cap.
        pipeline_desc = (
            f"udpsrc address={self._multicast_group} port={self._port} "
            f"auto-multicast=true multicast-iface={self._multicast_iface} "
            f"buffer-size=524288 "
            f"caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264\" "
            f"! rtpjitterbuffer latency=100 "
            f"! rtph264depay "
            f"! h264parse "
            f"! queue leaky=downstream max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
            f"! avdec_h264 "
            f"! videoconvert "
            # videorate post-decode caps the rate going downstream to the
            # UI. The decoder still receives every frame (H264 temporal
            # deps make pre-decode drops painful), but videoconvert /
            # videoscale / clockoverlay / jpegenc and the MJPEG ship-out
            # all do half the work at 15 fps. This frees CPU for the
            # decoder. The 4K recorder reads multicast :5000 directly and
            # is untouched — it still records 4K @ 30 fps verbatim.
            f"! videorate drop-only=true "
            f"! video/x-raw,framerate={self._target_framerate}/1 "
            f"! videoscale "
            f"! video/x-raw,width={self._target_width},height={self._target_height} "
            f"! clockoverlay halignment=left valignment=top "
            f"time-format=\"%H:%M:%S\" font-desc=\"Sans, 18\" "
            f"shaded-background=true "
            f"! jpegenc quality={self._jpeg_quality} "
            f"! appsink name=ui_sink emit-signals=true max-buffers=1 drop=true sync=false"
        )
        LOGGER.info("[video-bridge] Pipeline: %s", pipeline_desc)
        self._pipeline = Gst.parse_launch(pipeline_desc)

        appsink = self._pipeline.get_by_name("ui_sink")
        appsink.connect("new-sample", self._on_new_sample)

        # Bus watch surfaces decoder errors instead of silently parking
        # in PAUSED. Routed through GLib so it runs on the MainLoop
        # thread (Gst.Bus.add_watch needs a MainContext).
        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_bus_error)
        bus.connect("message::eos", self._on_bus_eos)

        self._loop = GLib.MainLoop()
        self._loop_thread = threading.Thread(
            target=self._loop.run, name="native-video-bridge", daemon=True,
        )
        self._loop_thread.start()

        rc = self._pipeline.set_state(Gst.State.PLAYING)
        if rc == Gst.StateChangeReturn.FAILURE:
            self.stop()
            raise RuntimeError(
                "video bridge: failed to enter PLAYING. Check that the C++ "
                "pipeline is running with --multicast and that sink1 is "
                "publishing on " + f"{self._multicast_group}:{self._port}",
            )
        LOGGER.info(
            "[video-bridge] Joined %s:%d on %s → web UI MJPEG",
            self._multicast_group, self._port, self._multicast_iface,
        )

    def stop(self) -> None:
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None
        if self._loop is not None:
            self._loop.quit()
            self._loop = None
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=2.0)
            self._loop_thread = None
        LOGGER.info("[video-bridge] Stopped after %d frames", self._frame_count)

    # ------------------------------------------------------------------ #
    # GStreamer callbacks
    # ------------------------------------------------------------------ #
    def _on_new_sample(self, sink) -> Gst.FlowReturn:
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK
        buf = sample.get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        try:
            jpeg = bytes(info.data)
        finally:
            buf.unmap(info)
        try:
            self._ui_state.update_frame(jpeg)
        except Exception:
            # Don't let a UI-side bug stall the GStreamer pipeline.
            LOGGER.exception("[video-bridge] update_frame raised")
        self._frame_count += 1
        return Gst.FlowReturn.OK

    def _on_bus_error(self, _bus, message) -> None:
        err, dbg = message.parse_error()
        LOGGER.error("[video-bridge] %s: %s", err.message, dbg or "")

    def _on_bus_eos(self, _bus, _message) -> None:
        LOGGER.warning("[video-bridge] EOS — C++ pipeline likely stopped")
