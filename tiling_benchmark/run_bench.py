"""Driver for tiling_bench.py — runs a list of configs back-to-back, prints
a comparison table, and runs analyze_bench.py for an mAP-style comparison
treating the multi-scale config as pseudo-ground-truth.

Usage:
  python run_bench.py [--video PATH]

The configs list is hardcoded for now; edit the CONFIGS constant to change
what gets run. The first entry must be the GT (multi-scale) — the analyzer
is run with `--gt` set to that run's frames.json.
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
BENCH_SCRIPT = HERE / "tiling_bench.py"
ANALYZE_SCRIPT = HERE / "analyze_bench.py"
DEFAULT_VIDEO = "/home/giladn/Videos/Drone/mafat_car.mp4"
DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "pxt_runs"

# All runs go through the visdrone SSD-MobileNet model so detections from
# different tile configs use the same class space — required for mAP to be
# meaningful across rows.
HEF_PATH = "/usr/local/hailo/resources/models/hailo8/ssd_mobilenet_v1_visdrone.hef"
LABELS_JSON = "/usr/local/hailo/resources/json/visdrone.json"
# The upstream tiling pipeline auto-detects post-process from the HEF and
# defaults to YOLO (libyolo_hailortpp_postprocess.so + 'filter'). That's wrong
# for SSD MobileNet — pass these explicitly so tiling_bench's override kicks in.
POST_PROCESS_SO = "/usr/local/hailo/resources/so/libmobilenet_ssd_postprocess.so"
POST_FUNCTION = "mobilenet_ssd_visdrone"

CONFIGS = [
    # GT run: upstream hailotilecropper with tiling-mode=1, scale-level=3.
    # Yields 1 (custom 1x1) + 14 (predefined 1x1+2x2+3x3) = 15 inferences/frame.
    # Slow but highest recall — used as pseudo-ground-truth for mAP.
    {
        "label": "GT-multi-scale-3",
        "tiles_x": 1, "tiles_y": 1,
        "overlap_x": 0.0, "overlap_y": 0.0,
        "multi_scale": True, "scale_level": 3,
    },
    {
        "label": "1x1",
        "tiles_x": 1, "tiles_y": 1,
        "overlap_x": 0.0, "overlap_y": 0.0,
    },
    {
        "label": "3x2",
        "tiles_x": 3, "tiles_y": 2,
        "overlap_x": 0.5, "overlap_y": 0.5,
    },
    # Adds a whole-frame tile so large objects are seen intact alongside the
    # 3x2 grid — should rescue truck/van AP that 3x2 alone fragments.
    {
        "label": "3x2+1x1",
        "tiles_x": 3, "tiles_y": 2,
        "overlap_x": 0.5, "overlap_y": 0.5,
        "include_full_frame": True,
    },
    # Adds a centered ~0.4x0.4 tile on top of the 3x2 grid — gives a higher
    # pixel-density look at the frame center where drone-follow targets bias.
    {
        "label": "3x2+center",
        "tiles_x": 3, "tiles_y": 2,
        "overlap_x": 0.5, "overlap_y": 0.5,
        "include_center_tile": True,
        "center_tile_size": 0.4,
    },
    # Same total tile count as multi-scale's 3x3 layer; tile size ~280px is
    # closer to the model's native 300px input — should help small objects
    # (person, motor) where 3x2's ~430px tiles undersample.
    {
        "label": "3x3",
        "tiles_x": 3, "tiles_y": 3,
        "overlap_x": 0.0, "overlap_y": 0.0,
    },
    # Cheap recall ceiling: large tiles + whole-frame fallback. Five tiles
    # total — between 1x1 and 3x2 in cost.
    {
        "label": "2x2+1x1",
        "tiles_x": 2, "tiles_y": 2,
        "overlap_x": 0.5, "overlap_y": 0.5,
        "include_full_frame": True,
    },
]


def run_one(cfg: dict, video: str, out_dir: Path, extra_args: list[str]) -> tuple[Path, float, int]:
    out_path = out_dir / f"tiling_bench_{cfg['label']}.json"
    cmd = [
        sys.executable,
        str(BENCH_SCRIPT),
        "--input", f"file://{video}",
        "--tiles-x", str(cfg["tiles_x"]),
        "--tiles-y", str(cfg["tiles_y"]),
        "--overlap-x", str(cfg["overlap_x"]),
        "--overlap-y", str(cfg["overlap_y"]),
        "--bench-output", str(out_path),
        "--bench-label", cfg["label"],
        "--hef-path", HEF_PATH,
        "--labels-json", LABELS_JSON,
        "--post-process-so", POST_PROCESS_SO,
        "--post-function", POST_FUNCTION,
        "--disable-sync",
    ]
    if cfg.get("multi_scale"):
        cmd += ["--multi-scale", "--scale-levels", str(cfg.get("scale_level", 3))]
    if cfg.get("include_full_frame"):
        cmd += ["--include-full-frame"]
    if cfg.get("include_center_tile"):
        cmd += ["--include-center-tile"]
        if "center_tile_size" in cfg:
            cmd += ["--center-tile-size", str(cfg["center_tile_size"])]
    cmd += extra_args
    print(f"\n=== Running config '{cfg['label']}' ===")
    print(" ".join(cmd), flush=True)
    t0 = time.perf_counter()
    proc = subprocess.run(cmd)
    wall = time.perf_counter() - t0
    return out_path, wall, proc.returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", default=DEFAULT_VIDEO, help="Path to input video")
    ap.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR), help="JSON output directory")
    ap.add_argument("--skip-analyze", action="store_true",
                    help="Skip the post-run mAP analysis step")
    ap.add_argument("--skip-existing", action="store_true",
                    help="Skip configs whose JSON output already exists in --out-dir")
    args, extra = ap.parse_known_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if not Path(args.video).is_file():
        print(f"ERROR: video not found: {args.video}", file=sys.stderr)
        sys.exit(1)

    results = []
    for cfg in CONFIGS:
        out_path = out_dir / f"tiling_bench_{cfg['label']}.json"
        if args.skip_existing and out_path.is_file():
            print(f"\n=== Skipping config '{cfg['label']}' (output already exists) ===", flush=True)
            with out_path.open() as f:
                data = json.load(f)
            results.append((cfg, data, 0.0))
            continue
        out_path, wall, rc = run_one(cfg, args.video, out_dir, extra)
        if rc != 0:
            print(f"  config '{cfg['label']}' exited with code {rc}", flush=True)
        if not out_path.is_file():
            print(f"  WARNING: bench output {out_path} missing — pipeline likely didn't reach EOS")
            results.append((cfg, None, wall))
            continue
        with out_path.open() as f:
            data = json.load(f)
        results.append((cfg, data, wall))

    print("\n" + "=" * 100)
    print(f"{'label':>20}  {'tot_frames':>10}  {'fwd_pct':>8}  {'mean_det/f':>10}  "
          f"{'mean_conf':>9}  {'sec_w/det':>9}  {'wall_s':>8}")
    print("-" * 100)
    for cfg, data, wall in results:
        if data is None:
            print(f"{cfg['label']:>20}  {'?':>10}  {'?':>8}  {'?':>10}  {'?':>9}  {'?':>9}  {wall:>8.2f}")
            continue
        s = data["summary"]
        print(f"{cfg['label']:>20}  "
              f"{s['total_frames']:>10}  "
              f"{s['frames_with_det_pct']:>8.2f}  "
              f"{s['mean_det_per_frame']:>10.3f}  "
              f"{s['mean_conf']:>9.3f}  "
              f"{s['seconds_with_any_det']:>9}  "
              f"{wall:>8.2f}")
    print("=" * 100)

    if args.skip_analyze:
        return

    # Drive analyze_bench.py: GT is the first config (must be the multi-scale run).
    gt_cfg = CONFIGS[0]
    gt_frames = out_dir / f"tiling_bench_{gt_cfg['label']}.frames.json"
    pred_frames = []
    for cfg in CONFIGS[1:]:
        p = out_dir / f"tiling_bench_{cfg['label']}.frames.json"
        if p.is_file():
            pred_frames.append(p)
        else:
            print(f"  WARNING: missing frames file for '{cfg['label']}': {p}")

    if not gt_frames.is_file():
        print(f"\nWARNING: GT frames file missing — skipping analysis: {gt_frames}")
        return
    if not pred_frames:
        print("\nWARNING: no pred frames files — skipping analysis")
        return

    analyze_cmd = [
        sys.executable,
        str(ANALYZE_SCRIPT),
        "--gt", str(gt_frames),
    ]
    for p in pred_frames:
        analyze_cmd += ["--pred", str(p)]
    print("\n=== Running analyze_bench.py ===")
    print(" ".join(analyze_cmd), flush=True)
    subprocess.run(analyze_cmd)


if __name__ == "__main__":
    main()
