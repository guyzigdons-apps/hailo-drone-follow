"""hailo-tiling-visualize — render an ffmpeg+ASS telemetry overlay on a video.

Reads a JSONL telemetry timeline (from ``hailo-tiling-import-telemetry``)
plus a source video and produces an annotated MP4 with an ASS-burned HUD
showing altitude / ground-speed / yaw-rate / lat-lon / frame number.

Plan 7 Task 1 ships only the argparse skeleton; Task 7 lands the ASS
generator and Task 8 wires the ffmpeg invocation. See
``docs/superpowers/plans/2026-05-28-telemetry-import-visualizer.md``.
"""
from __future__ import annotations

import argparse
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


def main(argv: Sequence[str] | None = None) -> int:
    """Entry point for the ``hailo-tiling-visualize`` console script.

    Stub: implemented in Plan 7 Tasks 7 + 8.
    """
    _build_argparser().parse_args(argv)
    raise SystemExit("not implemented")


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
