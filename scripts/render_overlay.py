#!/usr/bin/env python3
"""Offline overlay renderer.

Reads a recording bundle (``clean.mkv`` + ``frames.jsonl``) and produces
an ``overlay.rendered.mkv`` with the bbox + cross + state badge painted
on top — same visual contract as the live recording's ``overlay.mkv``,
but rendered from the clean source so the overlay can be tweaked,
redrawn, or reproduced any time after the flight.

Bundle layout consumed
----------------------
::

    recordings/<YYYY-MM-DD_HH-MM-SS>/
    ├── clean.mkv         (source video — no overlay)
    ├── frames.jsonl      (per-frame metadata, PTS-aligned to clean.mkv)
    ├── manifest.json     (optional — encoder bitrate)
    └── overlay.rendered.mkv   (THIS SCRIPT WRITES THIS)

Usage::

    python scripts/render_overlay.py <bundle_dir>

How it works
------------
ffmpeg decodes ``clean.mkv`` into raw BGRA frames on stdout. Each frame
becomes a writable ``cairo.ImageSurface``. The renderer indexes
``frames.jsonl`` by ``pts_ns`` (matching the buffer PTS stamped by the
live pipeline) and on each frame:

  * Drives a stub :class:`FollowTargetState` from the JSONL so the badge
    flash window is correct in *video* time (independent of render
    speed).
  * Paints non-target detection bboxes inline (matching
    ``hailooverlay_community`` style).
  * Calls the production :func:`draw_target_cross` from
    ``vision_branches`` to paint the cross + state badge + change toast,
    so the offline render stays visually in lockstep with whatever the
    live pipeline draws.

The painted BGRA frame is then piped to a second ffmpeg that encodes
``overlay.rendered.mkv``.

Why not GStreamer for the offline path?
---------------------------------------
``hailo.add_detection`` requires a writable ``GstBuffer``; PyGObject's
buffer wrapping holds an extra ref that breaks ``gst_buffer_is_writable``
from Python pad probes, so metadata attached this way silently doesn't
stick. Writing a custom GstElement in Python sidesteps the issue but
adds ~150 lines of element-registration boilerplate. ffmpeg + cairo is
simpler and uses the *same* draw callback as the live pipeline, so
visual parity tracks the live path without drift.
"""

from __future__ import annotations

import argparse
import bisect
import json
import logging
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

import cairo

# Hush the [FOLLOW] info lines that fire on every replayed transition —
# the live state log makes sense in flight, but it's noise here.
logging.getLogger("robot_follow.follow_api.state").setLevel(logging.WARNING)

from robot_follow.follow_api.state import FollowTargetState  # noqa: E402
from robot_follow.pipeline_adapter.vision_branches import (  # noqa: E402
    NON_TARGET_BBOX_COLOR_RGB,
    TargetCrossState,
    draw_target_cross,
)

LOG = logging.getLogger("render_overlay")


# ---------------------------------------------------------------------------
# Event log — follow_change cause attribution for the toast
# ---------------------------------------------------------------------------

# Mapping from ``cause`` strings emitted by the live drone-follow code
# (see drone_follow/follow_api/event_log.py callers) to the toast text
# we paint on the rendered overlay. Falls back to "TARGET CHANGED" when
# events.jsonl is missing or a cause is unrecognised.
def _toast_for(cause: str, to_id: Optional[int]) -> str:
    if cause == "USER" and to_id is not None:
        return f"USER LOCKED ID {to_id}"
    if cause == "USER":
        return "USER IDLE"
    if cause == "CLEAR":
        return "TARGET CLEARED"
    if cause == "REID" and to_id is not None:
        return f"REID RE-ACQUIRED ID {to_id}"
    if cause == "REID-DRIFT" and to_id is not None:
        return f"REID-DRIFT → ID {to_id}"
    if cause == "AUTO" and to_id is not None:
        return f"AUTO → ID {to_id}"
    if cause == "AUTO":
        return "TARGET LOST"
    if cause == "TIMEOUT":
        return "REID TIMEOUT"
    return "TARGET CHANGED"


class EventIndex:
    """Sequential reader over ``events.jsonl`` filtered to ``follow_change``.

    The renderer walks frames in PTS order; at each detected transition
    it pulls the next ``follow_change`` event whose wall-clock ``t`` is
    at-or-after the current frame's ``t`` field. If no events.jsonl is
    present, every call returns ``None`` and the renderer falls back to
    the default "TARGET CHANGED" toast.
    """

    def __init__(self, path: Optional[Path]) -> None:
        self._events: list[dict] = []
        self._cursor = 0
        if path is None or not path.is_file():
            return
        with open(path) as f:
            for raw in f:
                raw = raw.strip()
                if not raw:
                    continue
                try:
                    row = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if row.get("kind") == "follow_change":
                    self._events.append(row)

    def __len__(self) -> int:
        return len(self._events)

    def next_at_or_after(self, wall_t: float,
                        expected_to: Optional[int]) -> Optional[dict]:
        """Return the next follow_change event with t >= wall_t, advancing
        the cursor past it. Only returns an event whose ``to`` matches the
        renderer's observed target transition — a defence against drift
        between frames.jsonl and events.jsonl timestamps.
        """
        # Tolerance window: events.jsonl is wall-clock, frames.jsonl is
        # wall-clock too, both stamped on the same machine, but they
        # cross-thread races mean the event can land a few ms before or
        # after the matching frame. Search a ~250 ms window centred on
        # wall_t.
        lo = wall_t - 0.25
        while self._cursor < len(self._events):
            ev = self._events[self._cursor]
            t = ev.get("t", 0.0)
            if t < lo:
                self._cursor += 1
                continue
            if t > wall_t + 0.25:
                # Future event — leave for the next call.
                return None
            self._cursor += 1
            if ev.get("to") == expected_to:
                return ev
            # Mismatched ``to`` — skip and keep searching the window for
            # one that does match.
        return None


# ---------------------------------------------------------------------------
# Frame log
# ---------------------------------------------------------------------------

class FrameLog:
    """In-memory index of ``frames.jsonl`` keyed by ``pts_ns``."""

    def __init__(self, path: Path) -> None:
        rows: list[dict] = []
        with open(path) as f:
            for raw in f:
                raw = raw.strip()
                if not raw:
                    continue
                try:
                    rows.append(json.loads(raw))
                except json.JSONDecodeError:
                    continue
        rows = [r for r in rows if r.get("pts_ns") is not None]
        rows.sort(key=lambda r: r["pts_ns"])
        self._rows: list[dict] = rows
        self._keys: list[int] = [int(r["pts_ns"]) for r in rows]

    def __len__(self) -> int:
        return len(self._rows)

    def lookup(self, pts_ns: int) -> Optional[dict]:
        """Return the most-recent row at-or-before ``pts_ns`` (the live
        overlay is stable across a frame, so a one-frame miss caused by
        PTS rounding is visually fine).
        """
        if not self._rows:
            return None
        idx = bisect.bisect_right(self._keys, pts_ns) - 1
        if idx < 0:
            return None
        return self._rows[idx]


# ---------------------------------------------------------------------------
# Replay-aware FollowTargetState
# ---------------------------------------------------------------------------

class ReplayTargetState(FollowTargetState):
    """:class:`FollowTargetState` driven by video PTS instead of wall clock.

    The cairo badge flashes for two seconds after a followee change. The
    real state stamps ``_last_change_ts`` with ``time.monotonic()`` on
    each transition; the cairo callback computes ``monotonic() -
    _last_change_ts < 2.0``. In offline replay the renderer runs as fast
    as ffmpeg allows — typically faster than realtime — so the
    wall-clock window would shrink in the output video. We override
    ``_last_change_ts`` per frame so the same subtraction in the cairo
    callback yields *video-time* elapsed.
    """

    def __init__(self) -> None:
        super().__init__()
        self._virtual_now: float = 0.0
        self._virtual_change: Optional[float] = None

    def set_virtual_now(self, t_seconds: float) -> None:
        self._virtual_now = t_seconds

    def mark_change_at_virtual_time(self, t_seconds: float) -> None:
        self._virtual_change = t_seconds

    def refresh_flash_window(self) -> None:
        if self._virtual_change is None:
            return
        video_elapsed = self._virtual_now - self._virtual_change
        self._last_change_ts = time.monotonic() - video_elapsed


# ---------------------------------------------------------------------------
# Bbox drawing — replicates hailooverlay_community's appearance
# ---------------------------------------------------------------------------

def _rgb_from_packed(packed: int) -> tuple[float, float, float]:
    return (
        ((packed >> 16) & 0xFF) / 255.0,
        ((packed >> 8) & 0xFF) / 255.0,
        (packed & 0xFF) / 255.0,
    )


_NON_TARGET_COLOR = _rgb_from_packed(NON_TARGET_BBOX_COLOR_RGB)


def _draw_bboxes(ctx: cairo.Context, detections: list[dict],
                 width: int, height: int, target_id: Optional[int]) -> None:
    """Paint white bboxes around every non-target detection.

    Mirrors the live ``hailooverlay_community`` element when
    ``use-custom-colors=true`` is set with the project's overlay style
    YAML: non-target bboxes are white at thin line thickness, with a
    small ID label inside the top-left corner. The target detection
    itself is omitted here because the cairo cross + badge already
    represent it; this matches the live pipeline where ``highlight_target``
    removes the target detection before ``hailooverlay_community`` runs.
    """
    ctx.set_line_width(2.0)
    ctx.select_font_face("sans-serif", cairo.FONT_SLANT_NORMAL,
                         cairo.FONT_WEIGHT_BOLD)
    font_size = max(10, int(height * 0.018))
    ctx.set_font_size(font_size)

    for det in detections:
        det_id = det.get("id")
        if det_id is not None and target_id is not None and det_id == target_id:
            continue  # target is represented by the cross; skip bbox
        bbox = det.get("bbox") or (0.0, 0.0, 0.0, 0.0)
        x_n, y_n, w_n, h_n = (float(v) for v in bbox[:4])
        x = x_n * width
        y = y_n * height
        w = w_n * width
        h = h_n * height
        # Stroke
        ctx.set_source_rgba(*_NON_TARGET_COLOR, 1.0)
        ctx.rectangle(x, y, w, h)
        ctx.stroke()
        # ID label (top-left corner, inside the box)
        if det_id is not None:
            label = f"ID {det_id}"
            # Black halo
            ctx.set_source_rgba(0.0, 0.0, 0.0, 0.65)
            ctx.move_to(x + 4, y + font_size + 2)
            ctx.show_text(label)
            # White text on top, slightly offset for halo effect
            ctx.set_source_rgba(*_NON_TARGET_COLOR, 1.0)
            ctx.move_to(x + 4, y + font_size + 2)
            ctx.show_text(label)


# ---------------------------------------------------------------------------
# Cross-state helper — derives what the live highlight_target probe
# would have written, from the JSONL row.
# ---------------------------------------------------------------------------

def _resolve_target_detection(row: dict, target_id: int) -> Optional[dict]:
    """Find the detection that the cross should land on.

    Primary path: detection.id == followed_id. This always works for
    recordings made with the followed_id-is-target_id fix (anything
    >= commit fb05… on the bundle-layout branch).

    Legacy fallback: bundles recorded before the fix stored the stable
    ReID original_id in followed_id while detection.id was the live
    tracker id — so a direct match is impossible after a ReID swap.
    When that happens AND the frame contains exactly one detection,
    use it (single-person scenes, by far the most common case for
    locked follow). Multi-detection legacy rows still fall back to
    SEARCH; we'd rather show no cross than the wrong one.
    """
    dets = row.get("detections") or ()
    for det in dets:
        if det.get("id") == target_id:
            return det
    # Single-detection fallback for legacy bundles
    if len(dets) == 1:
        return dets[0]
    return None


def _cross_state_from_row(row: dict, target_state: ReplayTargetState,
                          cross_state: TargetCrossState) -> None:
    target_id = target_state.get_target()
    explicit_lock = target_state.is_explicit_lock()
    if target_id is None or target_id <= 0:
        cross_state.clear()
        return
    target_det = _resolve_target_detection(row, target_id)
    if target_det is None:
        # Target known but not visible — SEARCH badge (no cross)
        cross_state.set(None, None, "SEARCH")
        return
    bbox = target_det.get("bbox")
    if bbox is None:
        cross_state.set(None, None, "SEARCH")
        return
    x, y, w, h = (float(v) for v in bbox[:4])
    cx = x + w / 2.0
    cy = y + h / 2.0
    cross_state.set(cx, cy, "LOCKED" if explicit_lock else "AUTO")


# ---------------------------------------------------------------------------
# ffmpeg helpers
# ---------------------------------------------------------------------------

def _ffprobe(path: Path) -> dict:
    """Fall-back source of video dimensions when ``manifest.json`` doesn't
    have them (e.g. recordings made before Phase 4 of the bundle layout).
    """
    out = subprocess.check_output(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries",
         "stream=width,height,r_frame_rate,nb_frames,duration",
         "-of", "json", str(path)],
        timeout=10.0)
    data = json.loads(out)
    stream = data["streams"][0]
    fps_str = stream.get("r_frame_rate", "30/1")
    if "/" in fps_str:
        num, den = fps_str.split("/")
        fps = float(num) / float(den) if float(den) != 0 else 30.0
    else:
        fps = float(fps_str)
    return {
        "width":    int(stream["width"]),
        "height":   int(stream["height"]),
        "fps":      fps,
        "duration": float(stream.get("duration", 0.0) or 0.0),
    }


def _ffprobe_first_pts_ns(clean: Path) -> int:
    """Read clean.mkv's first frame's PTS, in nanoseconds.

    The live pipeline tees a buffer at ``output_tee`` and the recording
    branch's ``splitmuxsink``+``matroskamux`` preserves the original
    ``buffer.pts`` on the way into the file. ``frames.jsonl`` was
    written from the same upstream callback, so its ``pts_ns`` values
    use the same epoch. Anchoring our lookup on clean.mkv's first
    frame is what lines the two timelines up; computing PTS from
    ``frame_index / fps`` alone produces a multi-second lag because
    JSONL starts at the live-pipeline clock value, not at zero.
    """
    out = subprocess.check_output([
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_frames", "-read_intervals", "%+#1",
        "-of", "json", str(clean),
    ], timeout=10.0)
    data = json.loads(out)
    frames_arr = data.get("frames") or []
    if not frames_arr:
        return 0
    f = frames_arr[0]
    # ``best_effort_timestamp_time`` is the most reliable across muxers;
    # falls back to pts_time when the muxer didn't stamp one.
    t = f.get("best_effort_timestamp_time") or f.get("pts_time")
    if t is None:
        return 0
    return int(float(t) * 1e9)


def _video_info_from_manifest(manifest: dict, clean: Path) -> dict:
    """Prefer manifest dims (cheap, no subprocess) and only ffprobe when
    a field is missing. Falls back to the full ffprobe path cleanly.

    Returns a dict with width / height / fps / source ("manifest" or
    "ffprobe") so the caller can log which path was used.
    """
    video = manifest.get("video") if isinstance(manifest, dict) else None
    if isinstance(video, dict):
        w = video.get("width")
        h = video.get("height")
        fps = video.get("fps")
        if w and h and fps:
            return {"width": int(w), "height": int(h), "fps": float(fps),
                    "source": "manifest"}
    info = _ffprobe(clean)
    info["source"] = "ffprobe"
    return info


def _decode_proc(clean: Path):
    return subprocess.Popen(
        ["ffmpeg", "-hide_banner", "-loglevel", "error",
         "-i", str(clean),
         "-f", "rawvideo", "-pix_fmt", "bgra", "-"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )


def _encode_proc(output: Path, width: int, height: int,
                fps: float, bitrate_kbps: int):
    return subprocess.Popen(
        ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
         "-f", "rawvideo", "-pix_fmt", "bgra",
         "-s", f"{width}x{height}", "-r", f"{fps:.6f}",
         "-i", "-",
         "-c:v", "libx264", "-preset", "ultrafast",
         "-b:v", f"{bitrate_kbps}k", "-pix_fmt", "yuv420p",
         str(output)],
        stdin=subprocess.PIPE, stderr=subprocess.PIPE,
    )


# ---------------------------------------------------------------------------
# Top-level render
# ---------------------------------------------------------------------------

def render(bundle_dir: str | os.PathLike,
           output_name: str = "overlay.rendered.mkv") -> Path:
    bundle = Path(bundle_dir).resolve()
    clean = bundle / "clean.mkv"
    frames_path = bundle / "frames.jsonl"
    events_path = bundle / "events.jsonl"
    manifest_path = bundle / "manifest.json"
    output = bundle / output_name

    if not bundle.is_dir():
        raise SystemExit(f"bundle directory not found: {bundle}")
    if not clean.is_file():
        raise SystemExit(f"missing clean.mkv in bundle: {clean}")
    if not frames_path.is_file():
        raise SystemExit(f"missing frames.jsonl in bundle: {frames_path}")

    manifest: dict = {}
    if manifest_path.is_file():
        try:
            with open(manifest_path) as f:
                manifest = json.load(f)
        except (OSError, json.JSONDecodeError):
            LOG.warning("manifest.json present but unparseable; continuing")
    bitrate = int(manifest.get("video", {}).get("encoder_bitrate_kbps") or 5000)

    frame_log = FrameLog(frames_path)
    if not frame_log:
        raise SystemExit(
            f"frames.jsonl has no rows with pts_ns — recording probably "
            f"predates the overlay-replay feature: {frames_path}")
    LOG.info("loaded %d frame rows", len(frame_log))

    # follow_change events drive the change-toast attribution. Missing
    # events.jsonl (recordings predating Phase 3) is handled: every
    # transition falls back to the default "TARGET CHANGED" text.
    event_index = EventIndex(events_path if events_path.is_file() else None)
    if len(event_index):
        LOG.info("loaded %d follow_change events", len(event_index))
    else:
        LOG.info("no events.jsonl (or no follow_change events) — "
                 "using default 'TARGET CHANGED' toast")

    # Prefer manifest dims (cheap) over ffprobe (subprocess).
    info = _video_info_from_manifest(manifest, clean)
    width, height, fps = info["width"], info["height"], info["fps"]
    frame_bytes_size = width * height * 4  # BGRA = 4 bytes/pixel
    LOG.info("clean.mkv: %dx%d @ %.3f fps (from %s)",
             width, height, fps, info["source"])

    # Anchor the JSONL lookup on clean.mkv's actual first-frame PTS so
    # the two timelines line up. Without this, the renderer's
    # frame_index/fps queries start at 0 while JSONL pts_ns starts at
    # the live pipeline's buffer.pts (typically several seconds in),
    # producing a multi-second overlay lag in the rendered video.
    clean_first_pts_ns = _ffprobe_first_pts_ns(clean)
    if frame_log._keys:
        jsonl_first_pts_ns = frame_log._keys[0]
        offset_ms = (jsonl_first_pts_ns - clean_first_pts_ns) / 1e6
        LOG.info("PTS anchor: clean.mkv first=%dns, frames.jsonl first=%dns "
                 "(skew %.2fms)",
                 clean_first_pts_ns, jsonl_first_pts_ns, offset_ms)

    target_state = ReplayTargetState()
    cross_state = TargetCrossState()

    # Stub overlay object so the production ``draw_target_cross`` can
    # pull cached dims without touching a real Gst element. The function
    # checks ``dims`` first when supplied, so the attribute below is
    # belt-and-braces.
    class _StubOverlay:
        _cached_dims = (width, height)

        def get_static_pad(self, _name):
            return None

    stub_overlay = _StubOverlay()

    decode = _decode_proc(clean)
    encode = _encode_proc(output, width, height, fps, bitrate)

    prev_followed: Optional[int] = None
    frame_index = 0
    t0 = time.monotonic()
    LOG.info("rendering %s -> %s", clean.name, output.name)
    try:
        while True:
            raw = decode.stdout.read(frame_bytes_size)
            if len(raw) < frame_bytes_size:
                break
            buf = bytearray(raw)
            # cairo expects ARGB32 little-endian = BGRA bytes; the
            # rawvideo we asked ffmpeg for is BGRA, so this matches.
            surface = cairo.ImageSurface.create_for_data(
                memoryview(buf), cairo.FORMAT_ARGB32, width, height)
            try:
                ctx = cairo.Context(surface)

                # PTS for this clean.mkv frame in the *live pipeline's*
                # timeline. ``clean_first_pts_ns`` anchors us; the per-
                # frame increment is constant at the recording's fps.
                # JSONL's pts_ns lives in the same timeline so the
                # lookup matches the row that was logged at this
                # exact buffer in the live pipeline.
                pts_seconds = frame_index / fps
                pts_ns = clean_first_pts_ns + int(pts_seconds * 1e9)
                row = frame_log.lookup(pts_ns)

                if row is not None:
                    followed_id = row.get("followed_id")
                    explicit_lock = bool(row.get("explicit_lock") or False)

                    target_state.set_virtual_now(pts_seconds)
                    if followed_id != prev_followed:
                        target_state.mark_change_at_virtual_time(pts_seconds)
                        # Resolve the toast text from events.jsonl. We
                        # search a 250 ms window around the frame's
                        # wall-clock ``t``; a matching ``to`` confirms it
                        # really is the same transition the live pipeline
                        # logged.
                        frame_wall_t = float(row.get("t") or 0.0)
                        ev = event_index.next_at_or_after(
                            frame_wall_t, followed_id)
                        cause = ev.get("cause") if ev else None
                        cross_state.set_toast(_toast_for(cause or "", followed_id))
                        prev_followed = followed_id
                    if followed_id is None:
                        target_state.enter_auto_mode()
                    else:
                        target_state.set_target(int(followed_id))
                    target_state.set_explicit_lock(explicit_lock)
                    target_state.refresh_flash_window()

                    # Resolve which detection (if any) the cross should
                    # cover; pass its id to _draw_bboxes so we skip
                    # painting its bbox over the cross. Falls back to
                    # single-detection heuristic for legacy bundles
                    # whose followed_id was the stable ReID label
                    # rather than the live tracker id.
                    tid_for_render = target_state.get_target()
                    skip_id = None
                    if tid_for_render is not None:
                        td = _resolve_target_detection(row, tid_for_render)
                        if td is not None:
                            skip_id = td.get("id")

                    # Paint non-target bboxes first so the cross sits on top.
                    _draw_bboxes(ctx, row.get("detections") or (),
                                 width, height, skip_id)

                    # Compute cross state and call the production draw fn.
                    _cross_state_from_row(row, target_state, cross_state)
                    draw_target_cross(
                        stub_overlay, ctx, 0, 0,
                        cross_state, target_state,
                        dims=(width, height),
                    )
                surface.flush()
            finally:
                surface.finish()

            encode.stdin.write(bytes(buf))
            frame_index += 1
    finally:
        # Close the encoder's stdin so it sees EOF and flushes; then
        # block on both subprocesses without an artificial timeout.
        # ffmpeg can be slow on long inputs (multi-GB recordings on a
        # Pi5 take minutes), and an arbitrary cap just produces flaky
        # SIGKILLs that lose the final fragment.
        if encode.stdin is not None:
            try:
                encode.stdin.close()
            except BrokenPipeError:
                pass
        decode.wait()
        encode.wait()

    # Failure reporting: read stderr only on non-zero exit. ``read()`` is
    # safe here because both subprocesses have terminated.
    if decode.returncode not in (0, None):
        err = decode.stderr.read().decode("utf-8", errors="replace").strip()
        LOG.error("ffmpeg decode failed (rc=%s):\n%s",
                  decode.returncode, err or "<no stderr>")
        raise SystemExit("decode failed")
    if encode.returncode not in (0, None):
        err = encode.stderr.read().decode("utf-8", errors="replace").strip()
        LOG.error("ffmpeg encode failed (rc=%s):\n%s",
                  encode.returncode, err or "<no stderr>")
        raise SystemExit("encode failed")

    dt = time.monotonic() - t0
    size_mb = output.stat().st_size / 1e6 if output.exists() else 0.0
    LOG.info("done: %d frames in %.1fs (%.0f fps render) -> %s (%.1f MB)",
             frame_index, dt, frame_index / dt if dt > 0 else 0.0,
             output, size_mb)
    return output


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bundle", help="path to a recording bundle directory")
    ap.add_argument("-o", "--output", default="overlay.rendered.mkv",
                    help="output filename inside the bundle "
                         "(default: overlay.rendered.mkv)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    render(args.bundle, output_name=args.output)


if __name__ == "__main__":
    main()
