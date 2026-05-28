"""Run the GT-12x9-25-multi config over many prepared clips.

Wrapper around `run_pxt_yolov8m.py` that iterates over a clip set and routes
each clip's output into its own subdirectory under `pxt_runs_yolov8m/<clip_id>/`
so the 11 May-28 clips' outputs don't collide.

Defaults to the 11 prepared clips from the 2026-05-28 shoot. Pass `--clips`
to override.

GT-only by default: passes `--only GT-12x9-25-multi --skip-analyze` to the
inner driver so each invocation runs ONE config (the dense-tiling pseudo-GT)
and doesn't bother with the analyzer (no baselines to compare to per-clip).

Usage:
  python tiling_benchmark/run_gt_batch.py
  python tiling_benchmark/run_gt_batch.py --skip-existing
  python tiling_benchmark/run_gt_batch.py --clips 0028 0030 0031 0035
  python tiling_benchmark/run_gt_batch.py --clips-dir /path/to/clips
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
INNER = HERE / "run_pxt_yolov8m.py"

# Default clip set: the 2026-05-28 shoot.
# Main-camera 6K clips are referenced via their _prepared.MP4 (rotation-stripped).
# 4K landscape clips are referenced directly.
DEFAULT_CLIPS_DIR = Path("/home/giladn/Videos/Drone/Training")
DEFAULT_STEMS = [
    # Main 1x (28mm) — 6K, prepared (rotation-stripped)
    "DJI_20260528155151_0025_D_prepared",
    "DJI_20260528155239_0026_D_prepared",
    "DJI_20260528155332_0027_D_prepared",
    "DJI_20260528155741_0029_D_prepared",
    "DJI_20260528160047_0032_D_prepared",
    "DJI_20260528160134_0033_D_prepared",
    "DJI_20260528160300_0034_D_prepared",
    # Med-tele 2.5x (70mm) — 4K landscape
    "DJI_20260528155445_0028_D",
    "DJI_20260528155813_0030_D",
    # Tele 6x (168mm) — 4K landscape; DJI-TELE-12 main matrix
    "DJI_20260528155917_0031_D",
    "DJI_20260528160410_0035_D",
]

CLIP_ID_RE = re.compile(r"_(\d{4})_D(?:_prepared)?$")


def clip_id(stem: str) -> str:
    """Extract the 4-digit clip id from a DJI stem."""
    m = CLIP_ID_RE.search(stem)
    if not m:
        raise ValueError(f"cannot parse clip id from stem: {stem}")
    return m.group(1)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clips-dir", type=Path, default=DEFAULT_CLIPS_DIR,
                    help="Directory containing the .MP4 clips.")
    ap.add_argument("--clips", nargs="+", default=None,
                    help="Subset of clip IDs (e.g., '0025 0030') to process. "
                         "Default: all 11 from the May 28 shoot.")
    ap.add_argument("--out-base", type=Path,
                    default=HERE / "pxt_runs_yolov8m",
                    help="Base directory; per-clip subdirs created underneath.")
    ap.add_argument("--skip-existing", action="store_true",
                    help="Skip clips whose GT frames.json is already present.")
    ap.add_argument("--dry-run", action="store_true",
                    help="Print the per-clip commands without running.")
    args, extra = ap.parse_known_args()

    if not args.clips_dir.is_dir():
        print(f"ERROR: clips dir not found: {args.clips_dir}", file=sys.stderr)
        return 1

    stems = DEFAULT_STEMS
    if args.clips:
        keep = set(args.clips)
        stems = [s for s in DEFAULT_STEMS if clip_id(s) in keep]
        missing = keep - {clip_id(s) for s in DEFAULT_STEMS}
        if missing:
            print(f"ERROR: unknown clip ids: {sorted(missing)}", file=sys.stderr)
            return 1

    results: list[tuple[str, float, int, Path]] = []
    args.out_base.mkdir(parents=True, exist_ok=True)

    for stem in stems:
        cid = clip_id(stem)
        video = args.clips_dir / f"{stem}.MP4"
        if not video.is_file():
            print(f"WARNING: {video} missing; skipping clip {cid}")
            continue
        out_dir = args.out_base / cid
        out_dir.mkdir(parents=True, exist_ok=True)
        gt_frames = out_dir / "pxt_GT-12x9-25-multi.frames.json"
        if args.skip_existing and gt_frames.is_file():
            print(f"[{cid}] skip — GT already at {gt_frames}")
            continue

        cmd = [
            sys.executable, str(INNER),
            "--video", str(video),
            "--out-dir", str(out_dir),
            "--only", "GT-12x9-25-multi",
            "--skip-analyze",
        ] + extra

        if args.dry_run:
            print(f"[{cid}] DRY RUN: {' '.join(cmd)}")
            continue

        print(f"\n{'=' * 78}\n[{cid}] {video.name}\n    out -> {out_dir}\n{'=' * 78}",
              flush=True)
        t0 = time.perf_counter()
        proc = subprocess.run(cmd)
        wall = time.perf_counter() - t0
        results.append((cid, wall, proc.returncode, gt_frames))

    print("\n" + "=" * 78)
    print(f"{'clip':>6} {'wall_s':>10} {'frames':>8} {'rc':>4}  frames_json")
    print("-" * 78)
    for cid, wall, rc, gt_frames in results:
        n_frames = "?"
        if gt_frames.is_file():
            try:
                doc = json.loads(gt_frames.read_text())
                n_frames = str(len(doc) if isinstance(doc, list) else doc.get("frames", "?"))
            except Exception:
                n_frames = "?"
        print(f"{cid:>6} {wall:>10.1f} {n_frames:>8} {rc:>4}  {gt_frames}")
    print("=" * 78)

    return 0


if __name__ == "__main__":
    sys.exit(main())
