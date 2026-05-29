"""hailo-tiling-import-telemetry — import ULG / SRT into a JSONL timeline.

Reads a PX4 ULog (``--ulg``) or DJI SRT sidecar (``--srt``), aligns it to the
source video's timebase, and writes a ``RecordedTelemetry``-compatible JSONL
timeline to ``--output``.

See ``docs/superpowers/plans/2026-05-28-telemetry-import-visualizer.md``.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Sequence

from .. import __version__ as _PKG_VERSION
from ..telemetry.align import align_to_video
from ..telemetry.srt import parse_srt
from ..telemetry.ulg import parse_ulg


def _build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="hailo-tiling-import-telemetry",
        description=(
            "Import a PX4 ULog or DJI SRT sidecar into a RecordedTelemetry-"
            "compatible JSONL timeline. To ingest into flight_record.sqlite3, "
            "pipe through `hailo-tiling-bench --ingest-telemetry` once Plan 6 "
            "lands."
        ),
    )
    p.add_argument(
        "--version",
        action="version",
        version=f"%(prog)s {_PKG_VERSION}",
        help="Print the hailo_tiling package version and exit.",
    )
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--ulg", type=Path, default=None,
                     help="PX4 ULog (.ulg) input file.")
    src.add_argument("--srt", type=Path, default=None,
                     help="DJI SRT sidecar input file.")
    p.add_argument("--video", type=Path, default=None,
                   help="Source video (required for --align video-creation).")
    p.add_argument(
        "--align", default="video-start",
        help=("Time-alignment strategy: 'video-start' (default), "
              "'offset:<seconds>', or 'video-creation'."),
    )
    p.add_argument("--output", type=Path, default=None,
                   help="Output JSONL path. Default: <input>.jsonl.")
    p.add_argument("--limit", type=int, default=None,
                   help="Keep only the first N rows (debug knob).")
    p.add_argument("--strip-geo", action="store_true",
                   help="Drop the _geo sidecar from each row.")
    return p


def _default_output(source: Path) -> Path:
    """Default output path: '<source>.jsonl' alongside the input.

    Matches the spec wording ("<input>.jsonl alongside the input"). We
    append rather than replace the suffix so that ``foo.srt`` becomes
    ``foo.srt.jsonl`` and ``foo.ulg`` becomes ``foo.ulg.jsonl`` -- this
    avoids collisions if both an SRT and a ULG share a stem.
    """
    return source.with_suffix(source.suffix + ".jsonl")


def main(argv: Sequence[str] | None = None) -> int:
    """Entry point for the ``hailo-tiling-import-telemetry`` console script."""
    args = _build_argparser().parse_args(argv)

    # Pick the source and dispatch to the matching parser.
    if args.ulg is not None:
        source: Path = args.ulg
        rows = parse_ulg(source)
    else:
        source = args.srt
        rows = parse_srt(source)

    # video-creation requires --video; the other strategies tolerate a
    # missing path (align_to_video only reads it for video-creation).
    if args.align == "video-creation" and args.video is None:
        print(
            "error: --align video-creation requires --video PATH",
            file=sys.stderr,
        )
        return 2

    # Non-video-creation strategies ignore the path; pass a cross-platform
    # empty Path() rather than '/dev/null' (which doesn't exist on Windows
    # and was a Linux-specific placeholder).
    video_path = args.video if args.video is not None else Path()
    rows = align_to_video(rows, video_path, args.align)

    if args.strip_geo:
        rows = [{k: v for k, v in r.items() if k != "_geo"} for r in rows]

    if args.limit is not None:
        rows = rows[: args.limit]

    output = args.output if args.output is not None else _default_output(source)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as fh:
        for row in rows:
            # Deterministic encoding (sort_keys + compact separators) so two
            # runs over the same input produce byte-identical output. This
            # is Plan 7 Task 9's reproducibility requirement, applied here so
            # the contract holds from day one.
            fh.write(json.dumps(row, sort_keys=True, separators=(",", ":")))
            fh.write("\n")

    print(
        f"wrote {len(rows)} rows from {source} to {output}",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
