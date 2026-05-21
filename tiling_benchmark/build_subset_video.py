"""Build a subset MP4 from a source video using a list of frame indices.

Reads frame_indices from a selected_frames.json (the output of select_frames.py),
constructs an ffmpeg `select` filter expression with those indices, and writes
out a new contiguous MP4 re-clocked to a target fps.

Usage::

  python tiling_benchmark/build_subset_video.py \\
      --input /path/to/source.mp4 \\
      --selected /path/to/selected_frames.json \\
      --out /path/to/subset_video.mp4 \\
      --fps 30
"""

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", type=Path, required=True)
    ap.add_argument("--selected", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--crf", type=int, default=22)
    ap.add_argument("--preset", default="fast")
    args = ap.parse_args()

    if shutil.which("ffmpeg") is None:
        print("ERROR: ffmpeg not found on PATH", file=sys.stderr)
        return 1
    if not args.input.is_file():
        print(f"ERROR: input not found: {args.input}", file=sys.stderr)
        return 1
    if not args.selected.is_file():
        print(f"ERROR: selected JSON not found: {args.selected}", file=sys.stderr)
        return 1

    with args.selected.open() as f:
        sel = json.load(f)
    frame_indices = sel.get("frame_indices") or []
    if not frame_indices:
        print("ERROR: no frame_indices in selected JSON", file=sys.stderr)
        return 1

    # Build the select filter expression. Each term is `eq(n,IDX)`, OR'd
    # together. Then setpts to re-clock the output as contiguous at target fps.
    eq_terms = "+".join(f"eq(n\\,{i})" for i in frame_indices)
    vf_expr = f"select='{eq_terms}',setpts=N/{args.fps}/TB"

    args.out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ffmpeg", "-hide_banner", "-y",
        "-i", str(args.input),
        "-vf", vf_expr,
        "-vsync", "vfr",
        "-r", str(args.fps),
        "-c:v", "libx265",
        "-crf", str(args.crf),
        "-preset", args.preset,
        "-an",
        str(args.out),
    ]
    print("ffmpeg command:")
    print(" ".join(cmd))
    proc = subprocess.run(cmd)
    if proc.returncode != 0:
        print(f"ERROR: ffmpeg exited with code {proc.returncode}", file=sys.stderr)
        return proc.returncode

    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
