"""hailo-tiling-import-telemetry — import ULG / SRT into a JSONL timeline.

Reads a PX4 ULog (``--ulg``) or DJI SRT sidecar (``--srt``), aligns it to the
source video's timebase, and writes a ``RecordedTelemetry``-compatible JSONL
timeline to ``--output``.

Plan 7 Task 1 ships only the argparse skeleton; Task 5 wires the parsers +
aligner into a working CLI. See
``docs/superpowers/plans/2026-05-28-telemetry-import-visualizer.md``.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence


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


def main(argv: Sequence[str] | None = None) -> int:
    """Entry point for the ``hailo-tiling-import-telemetry`` console script.

    Stub: implemented in Plan 7 Task 5.
    """
    _build_argparser().parse_args(argv)
    raise SystemExit("not implemented")


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
