"""Driver for the pixels-per-target tiling sweep — yolov8m variant.

Sibling of run_pxt_bench.py targeting the COCO 80-class yolov8m HEF (on-chip
NMS, 640x640 native input). Output goes to its own dir
(`tiling_benchmark/pxt_runs_yolov8m/`) to avoid colliding with the 4-class
runs.

The downstream analyzer is invoked with `--filter-classes person car` so the
report only counts the two classes we actually care about for drone-follow
benchmarking, even though the model emits 80.

Usage:
  python tiling_benchmark/run_pxt_yolov8m.py
  python tiling_benchmark/run_pxt_yolov8m.py --skip-existing
  python tiling_benchmark/run_pxt_yolov8m.py --only GT-12x9-25-multi 1x1-native
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path


def probe_video_dims(video_path: str | Path) -> tuple[int, int]:
    """Return (width, height) of the first video stream via ffprobe.

    Exits non-zero with a clear error if ffprobe fails or no video stream
    is found.
    """
    cmd = ["ffprobe", "-v", "error", "-of", "json",
           "-show_streams", str(video_path)]
    try:
        proc = subprocess.run(cmd, check=False, capture_output=True, text=True)
    except FileNotFoundError:
        print("ERROR: ffprobe not found on PATH", file=sys.stderr)
        sys.exit(1)
    if proc.returncode != 0:
        print(f"ERROR: ffprobe failed for {video_path}: "
              f"{proc.stderr.strip()}", file=sys.stderr)
        sys.exit(1)
    try:
        data = json.loads(proc.stdout)
    except json.JSONDecodeError as e:
        print(f"ERROR: ffprobe JSON parse failed for {video_path}: {e}",
              file=sys.stderr)
        sys.exit(1)
    for s in data.get("streams", []):
        if s.get("codec_type") == "video":
            return int(s["width"]), int(s["height"])
    print(f"ERROR: no video stream found in {video_path}", file=sys.stderr)
    sys.exit(1)


HERE = Path(__file__).resolve().parent
BENCH_SCRIPT = HERE / "tiling_bench.py"
ANALYZE_SCRIPT = HERE / "analyze_pxt.py"

# Same pre-rotated MP4 as run_pxt_bench.py — GStreamer's decodebin does NOT
# apply the original file's rotate=90 side-data so we feed the transposed file.
DEFAULT_VIDEO = "/home/giladn/Videos/Drone/Training/DJI_20260430103421_0010_D_rotated.MP4"
DEFAULT_OUT_DIR = HERE / "pxt_runs_yolov8m"

# COCO yolov8m: 640x640 input, on-chip yolov8_nms_postprocess metadata in the
# HEF, 80 classes. tiling_bench.py auto-detects post_process_so + post_function
# from the HEF (yolov8 → libyolo_hailortpp_postprocess.so + 'filter'); do NOT
# override here.
HEF_PATH = "/usr/local/hailo/resources/models/hailo10h/yolov8m.hef"
# coco.json lays out the 80 COCO classes with "unlabeled" at index 0
# (matching the hailo_4_classes.json sentinel pattern), so labels[1]=="person"
# and labels[3]=="car". detection_threshold lives inside this JSON.
LABELS_JSON = "/usr/local/hailo/resources/json/coco.json"
# Downstream analyzer: only report the two classes we care about for the
# drone-follow comparison. Class IDs match labels[] indices in coco.json.
FILTER_CLASSES = ["person", "car"]

# Each entry produces:
#   <out_dir>/pxt_<label>.json         — headline summary
#   <out_dir>/pxt_<label>.frames.json  — per-frame normalized detections
#
# `source_w` / `source_h` of None means "use the video's native dims, probed
# via ffprobe at startup". The `1x1-640x640` baseline keeps the source pinned
# to yolov8m's native input resolution (vs the 640x480 baseline used by the
# 4-class runs).
CONFIGS = [
    # Pseudo-ground-truth: small tiles with the multi-scale extras.
    {"label": "GT-12x9-25-multi", "tiles_x": 12, "tiles_y": 9, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": None, "source_h": None, "is_gt": True,
     "extra_grids": [(1, 1, 0.0, 0.0), (3, 2, 0.25, 0.25)]},

    # No-tiling baselines.
    {"label": "1x1-native",  "tiles_x": 1, "tiles_y": 1, "overlap_x": 0.0, "overlap_y": 0.0,
     "source_w": None, "source_h": None},
    # Source pinned to yolov8m's native 640x640 input — sanity-check what the
    # model sees with no upscaling at all.
    {"label": "1x1-640x640", "tiles_x": 1, "tiles_y": 1, "overlap_x": 0.0, "overlap_y": 0.0,
     "source_w": 640, "source_h": 640},

    # Sweep at native source resolution, increasing tile count.
    {"label": "2x2-native",  "tiles_x": 2, "tiles_y": 2, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": None, "source_h": None},
    {"label": "3x2-native",  "tiles_x": 3, "tiles_y": 2, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": None, "source_h": None},
    {"label": "3x3-native",  "tiles_x": 3, "tiles_y": 3, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": None, "source_h": None},
    {"label": "4x3-native",  "tiles_x": 4, "tiles_y": 3, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": None, "source_h": None},
    {"label": "6x4-native",  "tiles_x": 6, "tiles_y": 4, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": None, "source_h": None},
    {"label": "8x6-native",  "tiles_x": 8, "tiles_y": 6, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": None, "source_h": None},

    # Hybrid: 4x3 (4:3-aspect tiles) + whole-frame rescue for large objects.
    {"label": "4x3-native+full", "tiles_x": 4, "tiles_y": 3, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": None, "source_h": None, "include_full_frame": True},
]


def run_one(cfg: dict, video: str, out_dir: Path, extra_args: list[str]) -> tuple[Path, float, int]:
    out_path = out_dir / f"pxt_{cfg['label']}.json"
    cmd = [
        sys.executable,
        str(BENCH_SCRIPT),
        "--input", f"file://{video}",
        "--tiles-x", str(cfg["tiles_x"]),
        "--tiles-y", str(cfg["tiles_y"]),
        "--overlap-x", str(cfg["overlap_x"]),
        "--overlap-y", str(cfg["overlap_y"]),
        "--width", str(cfg["source_w"]),
        "--height", str(cfg["source_h"]),
        "--bench-output", str(out_path),
        "--bench-label", cfg["label"],
        "--hef-path", HEF_PATH,
        "--labels-json", LABELS_JSON,
        "--disable-sync",
    ]
    if cfg.get("include_full_frame"):
        cmd.append("--include-full-frame")
    for (gx, gy, gox, goy) in cfg.get("extra_grids", []):
        cmd += ["--extra-grid", f"{gx},{gy},{gox},{goy}"]
    cmd += extra_args
    print(f"\n=== Running config '{cfg['label']}' ===")
    print(" ".join(cmd), flush=True)
    t0 = time.perf_counter()
    proc = subprocess.run(cmd)
    wall = time.perf_counter() - t0
    return out_path, wall, proc.returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", default=DEFAULT_VIDEO)
    ap.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR))
    ap.add_argument("--skip-existing", action="store_true",
                    help="Skip configs whose JSON output already exists in --out-dir")
    ap.add_argument("--only", nargs="+", default=None,
                    help="Run only configs whose labels appear in this list")
    ap.add_argument("--skip-analyze", action="store_true",
                    help="Skip the post-run pxt analysis step")
    args, extra = ap.parse_known_args()

    print("Model: yolov8m (640x640, COCO 80 classes, filtered downstream to person+car)",
          flush=True)

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

    # Probe the input video for native dims and substitute into every config
    # whose source_w/source_h is None.
    native_w, native_h = probe_video_dims(args.video)
    for cfg in CONFIGS:
        if cfg.get("source_w") is None:
            cfg["source_w"] = native_w
        if cfg.get("source_h") is None:
            cfg["source_h"] = native_h

    selected = CONFIGS
    if args.only:
        keep = set(args.only)
        selected = [c for c in CONFIGS if c["label"] in keep]
        missing = keep - {c["label"] for c in CONFIGS}
        if missing:
            print(f"ERROR: unknown labels in --only: {sorted(missing)}", file=sys.stderr)
            sys.exit(1)

    print("=" * 78)
    print(f"video:        {args.video}")
    print(f"native dims:  {native_w}x{native_h}")
    print(f"out dir:      {out_dir}")
    print(f"hef:          {HEF_PATH}")
    print(f"labels:       {LABELS_JSON}")
    print(f"filter:       {' '.join(FILTER_CLASSES)}")
    print("configs:")
    for cfg in selected:
        print(f"  {cfg['label']:>18}  source={cfg['source_w']}x{cfg['source_h']}"
              f"  grid={cfg['tiles_x']}x{cfg['tiles_y']}"
              f"  overlap={cfg['overlap_x']:.2f}/{cfg['overlap_y']:.2f}"
              f"{'  +full' if cfg.get('include_full_frame') else ''}")
    print("=" * 78)

    results = []
    for cfg in selected:
        out_path = out_dir / f"pxt_{cfg['label']}.json"
        if args.skip_existing and out_path.is_file():
            print(f"\n=== Skip '{cfg['label']}' (output exists) ===", flush=True)
            with out_path.open() as f:
                results.append((cfg, json.load(f), 0.0))
            continue
        out_path, wall, rc = run_one(cfg, args.video, out_dir, extra)
        if rc != 0:
            print(f"  config '{cfg['label']}' exited with code {rc}", flush=True)
        if not out_path.is_file():
            print(f"  WARNING: bench output {out_path} missing — pipeline likely didn't reach EOS")
            results.append((cfg, None, wall))
            continue
        with out_path.open() as f:
            results.append((cfg, json.load(f), wall))

    # Comparison table
    print("\n" + "=" * 110)
    print(f"{'label':>20} {'src':>11} {'grid':>7} {'overlap':>9} "
          f"{'frames':>7} {'fwd_pct':>8} {'det/f':>7} {'conf':>6} {'wall_s':>8}")
    print("-" * 110)
    for cfg, data, wall in results:
        src = f"{cfg['source_w']}x{cfg['source_h']}"
        grid = f"{cfg['tiles_x']}x{cfg['tiles_y']}"
        ov = f"{cfg['overlap_x']:.2f}/{cfg['overlap_y']:.2f}"
        if data is None:
            print(f"{cfg['label']:>20} {src:>11} {grid:>7} {ov:>9} {'?':>7} {'?':>8} {'?':>7} {'?':>6} {wall:>8.2f}")
            continue
        s = data["summary"]
        print(f"{cfg['label']:>20} {src:>11} {grid:>7} {ov:>9} "
              f"{s['total_frames']:>7} {s['frames_with_det_pct']:>8.2f} "
              f"{s['mean_det_per_frame']:>7.3f} {s['mean_conf']:>6.3f} {wall:>8.2f}")
    print("=" * 110)

    if args.skip_analyze:
        return

    gt_cfg = next((c for c in selected if c.get("is_gt")), None)
    if gt_cfg is None:
        print("\nNo config flagged is_gt=True in this run; skipping analysis.")
        return
    gt_frames = out_dir / f"pxt_{gt_cfg['label']}.frames.json"
    pred_frames = [out_dir / f"pxt_{c['label']}.frames.json"
                   for c in selected if c is not gt_cfg]
    pred_frames = [p for p in pred_frames if p.is_file()]
    if not (gt_frames.is_file() and pred_frames):
        print("\nMissing GT or pred frames files; skipping analysis.")
        return

    analyze_cmd = [sys.executable, str(ANALYZE_SCRIPT),
                   "--gt", str(gt_frames),
                   "--video-w", str(gt_cfg["source_w"]),
                   "--video-h", str(gt_cfg["source_h"]),
                   "--out", str(out_dir / "pxt_analysis.json"),
                   "--filter-classes", *FILTER_CLASSES]
    for p in pred_frames:
        analyze_cmd += ["--pred", str(p)]
    print("\n=== Running analyze_pxt.py ===")
    print(" ".join(analyze_cmd), flush=True)
    subprocess.run(analyze_cmd)


if __name__ == "__main__":
    main()
