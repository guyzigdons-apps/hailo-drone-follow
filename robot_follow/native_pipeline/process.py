"""Subprocess wrapper for the C++ df_native_pipeline binary.

Mirrors the ``DetachedMavsdkServer`` pattern: spawn + manage + clean
shutdown + name-based reap if a previous run hung. Unlike mavsdk_server,
we want the C++ binary to die together with our Python parent (no
``start_new_session``) so a Python crash doesn't leave the camera/NPU
busy on the H15.
"""

from __future__ import annotations

import logging
import os
import signal
import subprocess
import time
from typing import Optional

LOGGER = logging.getLogger(__name__)

# Renamed from drone_follow_native_pipeline to df_native_pipeline. The old
# name was getting SIGKILL'd during MediaLibrary::initialize() by a
# pkill -f matching the substring. The shorter name evades whatever is
# doing that matching. (Verified empirically: a byte-identical binary
# at /tmp/myapp runs to detection, while one at any path with the old
# name dies inside init().)
DEFAULT_BINARY = "/home/root/native_pipeline/df_native_pipeline"
DEFAULT_ZMQ_PORT = 7000
# Must match DEFAULT_MULTICAST_GROUP in native_pipeline/main.cpp. Both ends
# (the C++ binary's udp_sink0 and the recorder's gst-launch udpsrc) need to
# use the same group.
DEFAULT_MULTICAST_GROUP = "239.255.0.50"
DEFAULT_VIDEO_PORT = 5000


def _ensure_multicast_route(iface: str = "eth0", group: str = "239.0.0.0/8") -> None:
    """Make sure admin-scoped multicast traffic egresses on ``iface``.

    Without an explicit route the H15 kernel rejects sends to 239.x.x.x
    with EADDRNOTAVAIL ("Could not get/set settings from/on resource" at the
    udpsink layer) — there's no default multicast route on this stripped-
    down Yocto image. ``ip route replace`` is idempotent: safe to call on
    every start; survives until the next reboot. The route is local to
    this device, so it has no effect on multicast-unrelated networking.
    """
    try:
        subprocess.run(
            ["ip", "route", "replace", group, "dev", iface],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        LOGGER.warning(
            "[native] Could not install multicast route %s dev %s: %s — "
            "udpsink may fail to start in multicast mode",
            group, iface, exc,
        )


def _reap_native_pipeline() -> None:
    """Kill any stragglers from a prior crashed run.

    The binary holds the camera and the NPU; leaking it blocks the next
    launch. We use ``-x`` (exact comm match) instead of ``-f`` (substring
    on argv) so the pkill can't accidentally match a different process
    that merely *mentions* the binary's name (e.g. an strace command line
    that includes it as an argument).
    """
    try:
        subprocess.run(
            ["pkill", "-9", "-x", "df_native_pipel"],  # comm truncates at 15 chars
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        pass


class NativePipelineProcess:
    """Managed subprocess for the C++ pipeline binary.

    Usage::

        with NativePipelineProcess(zmq_port=7000) as proc:
            # … run subscriber + control loop …
            # subprocess gets terminated on context exit
    """

    def __init__(
        self,
        *,
        binary_path: str = DEFAULT_BINARY,
        zmq_port: int = DEFAULT_ZMQ_PORT,
        medialib_config: Optional[str] = None,
        startup_timeout_s: float = 3.0,
        multicast: bool = False,
        tiles_spec: Optional[str] = None,
    ) -> None:
        self.binary_path = binary_path
        self.zmq_port = zmq_port
        self.medialib_config = medialib_config
        self.startup_timeout_s = startup_timeout_s
        self.multicast = multicast
        # None → C++ binary uses framework DEFAULT_TILES.
        # Non-empty string → forwarded as ``--tiles "x,y,w,h;...;x,y,w,h"``.
        self.tiles_spec = tiles_spec
        self.process: Optional[subprocess.Popen] = None

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    def __enter__(self) -> "NativePipelineProcess":
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()

    def start(self) -> None:
        if self.process is not None:
            return
        if not os.path.isfile(self.binary_path):
            raise FileNotFoundError(
                f"native pipeline binary not found at {self.binary_path}; "
                "did you run scripts/h15_boot/install_app.sh + scp the binary?"
            )

        # Clean up any stale instance from a previous crashed run before
        # we try to grab the camera / NPU.
        _reap_native_pipeline()
        time.sleep(0.3)

        if self.multicast:
            _ensure_multicast_route()

        cmd = [self.binary_path, "--zmq-port", str(self.zmq_port)]
        if self.medialib_config:
            cmd += ["--config-file-path", self.medialib_config]
        if self.multicast:
            cmd += ["--multicast"]
        if self.tiles_spec:
            cmd += ["--tiles", self.tiles_spec]
        LOGGER.info("[native] Starting %s", " ".join(cmd))

        # No start_new_session — we want the child to inherit our process
        # group so a SIGTERM/SIGKILL of our parent cascades to it. This
        # matters more than mavsdk_server's "survive Ctrl+C to land safely"
        # because the camera/NPU need to be released cleanly even if our
        # Python control loop dies unexpectedly.
        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        # Wait until the binary actually publishes its first detection frame
        # (true readiness) rather than sleeping a fixed startup_timeout_s.
        # This also catches an immediate crash (bad config / device busy /
        # HEF load failure). Returns as soon as the pipeline is live — cuts
        # several seconds off the apply/rebuild latency vs the old blind sleep.
        self._await_first_frame()
        LOGGER.info("[native] Pipeline running (pid %d, zmq port %d)",
                    self.process.pid, self.zmq_port)

    def _await_first_frame(self, max_wait_s: float = 12.0) -> None:
        """Block until the native binary publishes its first ZMQ frame, it
        exits, or max_wait_s elapses. A read-only SUB probe on the same port
        the real subscriber uses — PUB fans out per-subscriber, so this does
        not steal frames from the downstream subscriber that connects later.
        """
        import zmq  # local import: keep zmq optional for non-native callers

        ctx = zmq.Context.instance()
        sub = ctx.socket(zmq.SUB)
        sub.setsockopt(zmq.SUBSCRIBE, b"")
        sub.setsockopt(zmq.RCVTIMEO, 200)
        sub.connect(f"tcp://127.0.0.1:{self.zmq_port}")
        t0 = time.time()
        try:
            while time.time() - t0 < max_wait_s:
                rc = self.process.poll()
                if rc is not None:
                    self.process = None
                    raise RuntimeError(
                        f"native pipeline exited immediately with code {rc}. "
                        "Check /home/root/hailo_analytics.log on the H15 for details."
                    )
                try:
                    sub.recv()
                    return  # first frame arrived → pipeline is live
                except zmq.Again:
                    continue
            LOGGER.warning(
                "[native] no frames within %.1fs of start; proceeding anyway",
                max_wait_s)
        finally:
            sub.close(0)

    def stop(self, timeout: float = 5.0) -> None:
        if self.process is None:
            return
        proc = self.process
        self.process = None
        if proc.poll() is not None:
            return  # already exited
        LOGGER.info("[native] Stopping pipeline (pid %d)", proc.pid)
        try:
            proc.send_signal(signal.SIGINT)  # let main.cpp's SignalHandler stop cleanly
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            LOGGER.warning("[native] Pipeline did not exit on SIGINT — escalating to SIGKILL")
            proc.kill()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass
            _reap_native_pipeline()


def _reap_recorder() -> None:
    """Kill any leftover gst-launch recorder processes."""
    try:
        subprocess.run(
            ["pkill", "-INT", "-f", "matroskamux.*filesink"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        pass


class MulticastRecorderProcess:
    """gst-launch subprocess that joins the multicast group and writes mkv.

    Sized to pair with ``NativePipelineProcess(multicast=True)``. Both
    must be on the same multicast group + port.

    The pipeline keeps the H264 untouched (no decode/re-encode): just
    depays the RTP, parses, muxes into matroska, writes. Lifetime is
    bounded by ``start()`` / ``stop()``; SIGINT first so matroskamux gets
    an EOS and writes a clean index, escalate to SIGKILL on timeout.
    """

    def __init__(
        self,
        record_path: str,
        *,
        multicast_group: str = DEFAULT_MULTICAST_GROUP,
        port: int = DEFAULT_VIDEO_PORT,
        multicast_iface: str = "eth0",
    ) -> None:
        self.record_path = record_path
        self.multicast_group = multicast_group
        self.port = port
        # H15 multicast joins fail with "No such device" if the kernel can't
        # pick a default route to the multicast group. Pinning to eth0 (the
        # H15 ↔ host ethernet interface) makes the join deterministic.
        self.multicast_iface = multicast_iface
        self.process: Optional[subprocess.Popen] = None

    def __enter__(self) -> "MulticastRecorderProcess":
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()

    def start(self) -> None:
        if self.process is not None:
            return
        os.makedirs(os.path.dirname(self.record_path) or ".", exist_ok=True)
        _reap_recorder()
        time.sleep(0.2)

        # rtpjitterbuffer smooths over UDP reorder/loss; h264parse normalizes
        # the stream-format before matroskamux. filesink with sync=false because
        # we want to write as fast as packets arrive (no real-time pacing).
        # Note: caps value uses no internal spaces — passing each space-
        # separated token as a list arg means an unquoted caps with spaces
        # would split into multiple args. Comma-only form is valid GStreamer.
        pipeline_args = [
            "udpsrc",
            f"address={self.multicast_group}",
            f"port={self.port}",
            "auto-multicast=true",
            # Without multicast-iface the kernel rejects the join with
            # "No such device": it can't pick an interface for an
            # admin-scoped group with no default route. eth0 is the H15's
            # ethernet interface holding 10.0.0.1.
            f"multicast-iface={self.multicast_iface}",
            "buffer-size=2097152",
            "caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264",
            "!",
            "rtpjitterbuffer", "latency=200",
            "!",
            "rtph264depay",
            "!",
            "h264parse", "config-interval=1",
            "!",
            "matroskamux",
            "!",
            "filesink", f"location={self.record_path}", "sync=false",
        ]
        cmd = ["gst-launch-1.0", "-e"] + pipeline_args
        # The -e flag tells gst-launch to send EOS on SIGINT so matroskamux
        # finalizes the file.
        LOGGER.info("[recorder] Starting gst-launch → %s", self.record_path)
        LOGGER.debug("[recorder] Pipeline args: %s", " ".join(cmd))

        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        # Brief sanity check: gst-launch immediately exits on a bad pipeline.
        time.sleep(0.5)
        if self.process.poll() is not None:
            rc = self.process.returncode
            self.process = None
            raise RuntimeError(
                f"gst-launch recorder exited immediately with code {rc}. "
                "Pipeline may be malformed or filesink path unwritable."
            )

    def stop(self, timeout: float = 5.0) -> None:
        if self.process is None:
            return
        proc = self.process
        self.process = None
        if proc.poll() is not None:
            return
        LOGGER.info("[recorder] Stopping (pid %d)", proc.pid)
        try:
            # SIGINT triggers EOS in gst-launch -e, which lets matroskamux
            # write the index/duration tags. Without this the file is
            # playable but missing seekability metadata.
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            LOGGER.warning("[recorder] gst-launch did not exit on SIGINT — SIGKILL")
            proc.kill()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass
