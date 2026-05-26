"""Upscale-hypothesis sweep driver — yolov8m variant.

Mirror of run_upscale_bench.py but pointed at the COCO 80-class yolov8m HEF
(on-chip NMS, 640x640 native input). Tile grids identical to the 4-class
upscale subset experiment so the two model families can be compared on the
same 50-frame subset video.

Output per config (in --out-dir):
  upscale_<label>.json         — headline summary
  upscale_<label>.frames.json  — per-frame normalized detections

Usage::

  python tiling_benchmark/run_upscale_yolov8m.py \\
      --video tiling_benchmark/pxt_runs/upscale_subset/subset_video.mp4 \\
      --out-dir tiling_benchmark/pxt_runs_yolov8m/upscale_subset \\
      --skip-analyze
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

# Reuse the ffprobe helper from the main pxt driver.
HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
from run_pxt_bench import probe_video_dims  # noqa: E402

BENCH_SCRIPT = HERE / "tiling_bench.py"

# Default subset video built by build_subset_video.py — same subset used by
# the 4-class upscale experiment.
DEFAULT_VIDEO = HERE / "pxt_runs" / "upscale_subset" / "subset_video.mp4"
DEFAULT_OUT_DIR = HERE / "pxt_runs_yolov8m" / "upscale_subset"

# yolov8m HEF + COCO 80-class labels. tiling_bench.py auto-detects the
# post-process .so / function from the HEF.
HEF_PATH = "/usr/local/hailo/resources/models/hailo10h/yolov8m.hef"
LABELS_JSON = "/usr/local/hailo/resources/json/coco.json"

# Same 5 grids as the 4-class upscale subset experiment, all at native
# source dims with 0.25 overlap.
CONFIGS = [
    {"label": "baseline-12x9",  "tiles_x": 12, "tiles_y": 9},
    {"label": "upscale-14x10",  "tiles_x": 14, "tiles_y": 10},
    {"label": "upscale-16x12",  "tiles_x": 16, "tiles_y": 12},
    {"label": "upscale-18x13",  "tiles_x": 18, "tiles_y": 13},
    {"label": "upscale-20x15",  "tiles_x": 20, "tiles_y": 15},
]
OVERLAP_X = 0.25
OVERLAP_Y = 0.25


def run_one(cfg: dict, video: str, out_dir: Path, source_w: int, source_h: int,
            extra_args: list[str]) -> tuple[Path, float, int]:
    out_path = out_dir / f"upscale_{cfg['label']}.json"
    cmd = [
        sys.executable,
        str(BENCH_SCRIPT),
        "--input", f"file://{video}",
        "--tiles-x", str(cfg["tiles_x"]),
        "--tiles-y", str(cfg["tiles_y"]),
        "--overlap-x", str(OVERLAP_X),
        "--overlap-y", str(OVERLAP_Y),
        "--width", str(source_w),
        "--height", str(source_h),
        "--bench-output", str(out_path),
        "--bench-label", cfg["label"],
        "--hef-path", HEF_PATH,
        "--labels-json", LABELS_JSON,
        "--disable-sync",
    ]
    cmd += extra_args
    print(f"\n=== Running config '{cfg['label']}' ===")
    print(" ".join(cmd), flush=True)
    t0 = time.perf_counter()
    proc = subprocess.run(cmd)
    wall = time.perf_counter() - t0
    return out_path, wall, proc.returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", default=str(DEFAULT_VIDEO))
    ap.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR))
    ap.add_argument("--skip-existing", action="store_true",
                    help="Skip configs whose JSON output already exists in --out-dir")
    ap.add_argument("--skip-analyze", action="store_true",
                    help="Skip any post-run analysis (we run analyze_pxt.py "
                         "manually for this experiment).")
    args, extra = ap.parse_known_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if not Path(args.video).is_file():
        print(f"ERROR: video not found: {args.video}", file=sys.stderr)
        sys.exit(1)
    if not Path(HEF_PATH).is_file():
        print(f"ERROR: HEF not found: {HEF_PATH}", file=sys.stderr)
        sys.exit(1)
    if not Path(LABELS_JSON).is_file():
        print(f"ERROR: labels JSON not found: {LABELS_JSON}", file=sys.stderr)
        sys.exit(1)

    source_w, source_h = probe_video_dims(args.video)

    print("=" * 78)
    print(f"video:        {args.video}")
    print(f"native dims:  {source_w}x{source_h}")
    print(f"hef:          {HEF_PATH}")
    print(f"labels:       {LABELS_JSON}")
    print(f"out dir:      {out_dir}")
    print("configs:")
    for cfg in CONFIGS:
        tile_w = source_w / cfg["tiles_x"]
        tile_h = source_h / cfg["tiles_y"]
        print(f"  {cfg['label']:>18}  grid={cfg['tiles_x']}x{cfg['tiles_y']}"
              f"  per-tile~{tile_w:.0f}x{tile_h:.0f} src-px")
    print("=" * 78)

    results = []
    for cfg in CONFIGS:
        out_path = out_dir / f"upscale_{cfg['label']}.json"
        if args.skip_existing and out_path.is_file():
            print(f"\n=== Skip '{cfg['label']}' (output exists) ===", flush=True)
            with out_path.open() as f:
                results.append((cfg, json.load(f), 0.0, 0))
            continue
        out_path, wall, rc = run_one(cfg, args.video, out_dir, source_w, source_h, extra)
        if rc != 0:
            print(f"  config '{cfg['label']}' exited with code {rc}", flush=True)
        if not out_path.is_file():
            print(f"  WARNING: bench output {out_path} missing — pipeline likely didn't reach EOS")
            results.append((cfg, None, wall, rc))
            continue
        with out_path.open() as f:
            results.append((cfg, json.load(f), wall, rc))

    # Comparison table
    print("\n" + "=" * 110)
    print(f"{'label':>18} {'grid':>7} {'frames':>7} {'fwd_pct':>8} "
          f"{'det/f':>7} {'conf':>6} {'wall_s':>8} {'rc':>4}")
    print("-" * 110)
    for cfg, data, wall, rc in results:
        grid = f"{cfg['tiles_x']}x{cfg['tiles_y']}"
        if data is None:
            print(f"{cfg['label']:>18} {grid:>7} {'?':>7} {'?':>8} {'?':>7} {'?':>6} "
                  f"{wall:>8.2f} {rc:>4}")
            continue
        s = data["summary"]
        print(f"{cfg['label']:>18} {grid:>7} {s['total_frames']:>7} "
              f"{s['frames_with_det_pct']:>8.2f} {s['mean_det_per_frame']:>7.3f} "
              f"{s['mean_conf']:>6.3f} {wall:>8.2f} {rc:>4}")
    print("=" * 110)


if __name__ == "__main__":
    main()
