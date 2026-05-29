"""hailo-tiling-visualize — render an ffmpeg+ASS telemetry overlay on a video.

Reads a JSONL telemetry timeline (from ``hailo-tiling-import-telemetry``)
plus a source video and produces an annotated MP4 with an ASS-burned HUD
showing altitude / ground-speed / yaw-rate / lat-lon / frame number.

Plan 7 Task 7 ships the pure-Python ASS generator; Task 8 wires the
ffmpeg invocation. See
``docs/superpowers/plans/2026-05-28-telemetry-import-visualizer.md``.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Sequence


def _build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="hailo-tiling-visualize",
        description=(
            "Render a telemetry HUD overlay onto a source video using "
            "ffmpeg + ASS subtitles. Reads JSONL timelines produced by "
            "hailo-tiling-import-telemetry."
        ),
    )
    p.add_argument("--video", type=Path, required=True,
                   help="Source video file.")
    p.add_argument("--telemetry", type=Path, required=True,
                   help="JSONL timeline from hailo-tiling-import-telemetry.")
    p.add_argument("--output", type=Path, required=True,
                   help="Output annotated MP4 path.")
    p.add_argument("--fps", type=float, default=None,
                   help="Override video frame rate. Default: probed via ffprobe.")
    p.add_argument("--font", type=int, default=24,
                   help="ASS font size (points). Default: 24.")
    p.add_argument("--ffmpeg-path", type=Path, default=None,
                   help="Path to ffmpeg binary. Default: 'ffmpeg' on PATH.")
    p.add_argument("--dry-run", action="store_true",
                   help="Build the ASS overlay and print it; skip ffmpeg.")
    return p


# ---------------------------------------------------------------------------
# Task 7: ASS overlay generator (pure-Python)
# ---------------------------------------------------------------------------

# ASS uses '{...}' for inline override codes and '\' as a control prefix.
# Cue text must escape these so a stray brace in (future) field labels does
# not turn into a style override. lat/lon strings won't trigger this, but the
# helper exists as a defensive measure (per spec).
def _escape_ass_text(s: str) -> str:
    """Escape ASS specials (`{`, `}`, `\\`) in a piece of cue text."""
    return (
        s.replace("\\", "\\\\")
         .replace("{", "\\{")
         .replace("}", "\\}")
    )


def _fmt_float(value, width: int, precision: int) -> str:
    """Format a float at fixed width/precision; render ``None`` as dashes."""
    if value is None:
        return "-" * width
    try:
        return f"{float(value):>{width}.{precision}f}"
    except (TypeError, ValueError):
        return "-" * width


def _fmt_geo(value, width: int, precision: int) -> str:
    """Format a geo value at fixed width.

    - ``None`` -> all dashes.
    - Numeric -> fixed-precision float, like :func:`_fmt_float`.
    - String -> truncated/padded to ``width`` characters (no float conversion).
      Used as a defensive escape hatch so non-numeric geo entries (e.g.
      future labels) still align in the HUD; the caller is expected to
      pass the result through :func:`_escape_ass_text` for ASS safety.
    """
    if value is None:
        return "-" * width
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return f"{float(value):>{width}.{precision}f}"
    if isinstance(value, str):
        if len(value) >= width:
            return value[:width]
        return value.rjust(width)
    try:
        return f"{float(value):>{width}.{precision}f}"
    except (TypeError, ValueError):
        return "-" * width


def _ground_speed(row: dict):
    """Derive ground speed in m/s from a telemetry row.

    Uses ``sqrt(vx^2 + vy^2)`` when ``velocity_world`` is present, otherwise
    falls back to a per-row precomputed scalar (``ground_speed_m_s``). Returns
    ``None`` if neither is available.
    """
    v = row.get("velocity_world")
    if v is not None:
        try:
            vx = float(v[0])
            vy = float(v[1])
            return math.sqrt(vx * vx + vy * vy)
        except (TypeError, ValueError, IndexError):
            pass
    gs = row.get("ground_speed_m_s")
    if gs is not None:
        try:
            return float(gs)
        except (TypeError, ValueError):
            pass
    return None


def _format_overlay_text(row: dict) -> str:
    """Format one telemetry row as a fixed-width single-line HUD string.

    Columns:
      ``ALT  12.3 m  GS   4.5 m/s  YAW  0.12 rad/s  LAT  0.000123  LON  0.000456  F    1234``

    Missing fields render as ``--`` of the same column width so lines stay
    aligned. The frame number is derived by the caller (it depends on fps);
    here we expect the row dict may carry ``_frame_idx`` injected by
    :func:`build_ass`. If absent, the frame column renders as dashes.
    """
    alt = _fmt_float(row.get("altitude_agl_m"), width=5, precision=1)
    gs = _fmt_float(_ground_speed(row), width=5, precision=1)
    yaw = _fmt_float(row.get("yaw_rate_rad_s"), width=5, precision=2)

    geo = row.get("_geo") or {}
    lat_raw = geo.get("lat") if isinstance(geo, dict) else None
    lon_raw = geo.get("lon") if isinstance(geo, dict) else None
    lat = _fmt_geo(lat_raw, width=10, precision=6)
    lon = _fmt_geo(lon_raw, width=10, precision=6)

    frame_idx = row.get("_frame_idx")
    if frame_idx is None:
        frame_str = "-" * 6
    else:
        try:
            frame_str = f"{int(frame_idx):>6d}"
        except (TypeError, ValueError):
            frame_str = "-" * 6

    # Escape every dynamic segment defensively.
    parts = [
        f"ALT  {_escape_ass_text(alt)} m",
        f"GS   {_escape_ass_text(gs)} m/s",
        f"YAW  {_escape_ass_text(yaw)} rad/s",
        f"LAT  {_escape_ass_text(lat)}",
        f"LON  {_escape_ass_text(lon)}",
        f"F    {_escape_ass_text(frame_str)}",
    ]
    return "  ".join(parts)


def _ass_time(seconds: float) -> str:
    """Format ``seconds`` as ASS ``H:MM:SS.cc`` (centiseconds, no leading zero on hour)."""
    if seconds < 0:
        seconds = 0.0
    total_cs = int(round(seconds * 100))
    hours, rem_cs = divmod(total_cs, 360_000)
    minutes, rem_cs = divmod(rem_cs, 6_000)
    secs, cs = divmod(rem_cs, 100)
    return f"{hours:d}:{minutes:02d}:{secs:02d}.{cs:02d}"


_ASS_HEADER = (
    "[Script Info]\n"
    "ScriptType: v4.00+\n"
    "Collisions: Normal\n"
    "PlayResX: 1920\n"
    "PlayResY: 1080\n"
    "WrapStyle: 2\n"
    "ScaledBorderAndShadow: yes\n"
    "\n"
    "[V4+ Styles]\n"
    "Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, "
    "OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, "
    "ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, "
    "Alignment, MarginL, MarginR, MarginV, Encoding\n"
    "Style: Default,Consolas,24,&H00FFFFFF,&H000000FF,&H00000000,&H00000000,"
    "0,0,0,0,100,100,0,0,1,0.5,0,1,20,20,20,1\n"
    "\n"
    "[Events]\n"
    "Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, "
    "Effect, Text\n"
)


def build_ass(rows: list[dict], fps: float, duration_s: float) -> str:
    """Render an ASS subtitle file string with one Dialogue per video frame.

    Last-value-carried-forward: the cue for frame ``i`` (covering
    ``[i/fps, (i+1)/fps)``) reflects the row with the largest
    ``timestamp <= i / fps``. Frames before the first telemetry row render
    with the empty row (everything as ``--``).
    """
    if fps <= 0:
        raise ValueError(f"fps must be positive, got {fps!r}")
    if duration_s < 0:
        raise ValueError(f"duration_s must be non-negative, got {duration_s!r}")

    sorted_rows = sorted(rows, key=lambda r: float(r.get("timestamp", 0.0)))
    timestamps = [float(r.get("timestamp", 0.0)) for r in sorted_rows]

    n_frames = int(duration_s * fps)
    lines = [_ASS_HEADER]
    empty_row: dict = {}

    for frame_idx in range(n_frames):
        start = frame_idx / fps
        end = (frame_idx + 1) / fps
        idx = bisect.bisect_right(timestamps, start) - 1
        if idx < 0:
            base = empty_row
        else:
            base = sorted_rows[idx]
        row_view = dict(base)
        row_view["_frame_idx"] = frame_idx
        text = _format_overlay_text(row_view)
        lines.append(
            f"Dialogue: 0,{_ass_time(start)},{_ass_time(end)},Default,,0,0,0,,{text}\n"
        )
    return "".join(lines)


def _load_jsonl(path: Path) -> list[dict]:
    """Load a JSONL telemetry timeline produced by ``import_telemetry``."""
    rows: list[dict] = []
    with path.open("r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    return rows


def _parse_avg_frame_rate(value: str) -> float | None:
    """Parse an ffprobe ``avg_frame_rate`` field (e.g. ``30000/1001``).

    Returns ``None`` on any parse failure (caller falls back to the default).
    """
    if not value:
        return None
    value = value.strip()
    try:
        if "/" in value:
            num, den = value.split("/", 1)
            num_f = float(num)
            den_f = float(den)
            if den_f == 0.0:
                return None
            return num_f / den_f
        return float(value)
    except (TypeError, ValueError):
        return None


def _probe_fps(video: Path, default: float = 30.0) -> float:
    """Probe the video's average frame rate via ``ffprobe``.

    Falls back to ``default`` if ffprobe is missing, the call fails, or the
    output cannot be parsed.
    """
    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        return default
    try:
        result = subprocess.run(
            [
                ffprobe,
                "-v", "error",
                "-select_streams", "v:0",
                "-show_entries", "stream=avg_frame_rate",
                "-of", "default=nw=1:nk=1",
                str(video),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return default
    if result.returncode != 0:
        return default
    parsed = _parse_avg_frame_rate(result.stdout)
    if parsed is None or parsed <= 0.0:
        return default
    return parsed


def _probe_duration(video: Path, default: float | None = None) -> float | None:
    """Probe the video's duration in seconds via ``ffprobe``.

    Returns ``default`` on any failure so the caller can compute duration
    from row count if needed.
    """
    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        return default
    try:
        result = subprocess.run(
            [
                ffprobe,
                "-v", "error",
                "-select_streams", "v:0",
                "-show_entries", "stream=duration",
                "-of", "default=nw=1:nk=1",
                str(video),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return default
    if result.returncode != 0:
        return default
    try:
        return float(result.stdout.strip())
    except (TypeError, ValueError):
        return default


def _invoke_ffmpeg(
    ffmpeg_path: str,
    video: Path,
    ass_file: Path,
    output: Path,
) -> tuple[int, str]:
    """Run ffmpeg to burn the ASS overlay onto ``video`` and write ``output``.

    Returns ``(returncode, stderr_text)``. The caller is responsible for
    reporting failure to the user.
    """
    cmd = [
        ffmpeg_path,
        "-y",
        "-i", str(video),
        "-vf", f"ass={ass_file}",
        "-c:v", "libx264",
        "-preset", "fast",
        "-crf", "20",
        "-c:a", "copy",
        str(output),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)
    return result.returncode, result.stderr


def main(argv: Sequence[str] | None = None) -> int:
    """Entry point for the ``hailo-tiling-visualize`` console script."""
    args = _build_argparser().parse_args(argv)

    # Resolve ffmpeg binary: explicit --ffmpeg-path wins, else search PATH.
    if args.ffmpeg_path is not None:
        ffmpeg_bin: str | None = str(args.ffmpeg_path)
    else:
        ffmpeg_bin = shutil.which("ffmpeg")

    # If we'll actually invoke ffmpeg, fail fast and clearly when missing.
    if not args.dry_run and ffmpeg_bin is None:
        print(
            "error: ffmpeg not found on PATH. Install ffmpeg (e.g. "
            "`sudo apt install ffmpeg`) or pass --ffmpeg-path.",
            file=sys.stderr,
        )
        return 2

    rows = _load_jsonl(args.telemetry)

    fps = args.fps if args.fps is not None else _probe_fps(args.video)
    if fps <= 0:
        print(
            f"error: invalid fps {fps!r}; pass --fps explicitly.",
            file=sys.stderr,
        )
        return 2

    # In --dry-run we never shell out; the duration falls back to the last
    # telemetry-row timestamp (plenty for inspecting the ASS payload). When
    # actually rendering, probe the video so the ASS spans the real clip.
    if args.dry_run:
        duration = None
    else:
        duration = _probe_duration(args.video)
    if duration is None or duration <= 0:
        # Fall back to the last telemetry-row timestamp; if even that is
        # missing, render a single-frame ASS.
        if rows:
            last_ts = max(float(r.get("timestamp", 0.0)) for r in rows)
            duration = max(last_ts, 1.0 / fps)
        else:
            duration = 1.0 / fps

    ass_text = build_ass(rows=rows, fps=fps, duration_s=duration)

    if args.dry_run:
        sys.stdout.write(ass_text)
        sys.stdout.flush()
        print(
            f"dry-run: built ASS for {len(rows)} telemetry rows "
            f"at {fps:.3f} fps over {duration:.3f} s",
            file=sys.stderr,
        )
        return 0

    args.output.parent.mkdir(parents=True, exist_ok=True)

    # NamedTemporaryFile inside a TemporaryDirectory: ffmpeg's ass= filter
    # takes a file path, and the surrounding directory tears down on exit.
    with tempfile.TemporaryDirectory(prefix="hailo-tiling-vis-") as tmpdir:
        ass_path = Path(tmpdir) / "overlay.ass"
        ass_path.write_text(ass_text, encoding="utf-8")
        assert ffmpeg_bin is not None  # guarded above
        rc, stderr = _invoke_ffmpeg(
            ffmpeg_bin, args.video, ass_path, args.output,
        )

    if rc != 0:
        sys.stderr.write(stderr)
        print(
            f"error: ffmpeg exited with code {rc}",
            file=sys.stderr,
        )
        return rc if rc != 0 else 1

    print(
        f"wrote annotated video to {args.output} "
        f"({len(rows)} telemetry rows, {fps:.3f} fps, {duration:.3f} s)",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
