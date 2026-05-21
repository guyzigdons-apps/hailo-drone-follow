# Tiling Accuracy & Pixels-Per-Target Sweep Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Quantify the effective "pixels per target" the Hailo detection network needs to reliably find people and vehicles in DJI drone footage, by running an inference sweep across multiple tiling layouts and source resolutions against a high-recall pseudo-ground-truth.

**Architecture:** Reuses the existing `tiling_benchmark/tiling_bench.py` GStreamer benchmark harness (it already dumps normalized per-frame detections to `<run>.frames.json`). A new driver `run_pxt_bench.py` defines the experiment list (a GT config + a sweep matrix) and dispatches `tiling_bench.py` once per config. A new analyzer `analyze_pxt.py` joins each candidate's detections to the GT detections frame-by-frame, then bins the GT objects by their height in **source pixels** to produce a recall-vs-size curve per class. An optional overlay viewer `overlay_dets.py` renders the saved per-frame JSON on top of the source video on the fly (no annotated video file written).

**Tech Stack:**
- GStreamer + `hailotilecropper_dynamic` (already wired up in `tiling_benchmark/tiling_bench.py` / `tiling_record.py`)
- Hailo-10H + `hailo_yolov8n_4_classes_vga.hef` — the **default HEF** used by the upstream tiling app. Input shape **640×480 (W×H)**, 4 classes via on-chip NMS, label strings from `hailo_4_classes.json` (`person`, `vehicle`, `face`, `license_plate`).
- Python 3, OpenCV (for overlay viewer only)
- JSON for all artefacts (one summary + one per-frame JSON per run, plus an aggregate analysis JSON)

**Execution constraint (parallelism):** Inference runs serialize on the single Hailo-10H chip. **Only one subagent at a time may drive `tiling_bench.py`** (i.e. Tasks 0, 2, 3, 5 must be sequential w.r.t. each other). Tasks that don't touch the chip — writing/reviewing the analyzer (Task 4), writing/reviewing the overlay viewer (Task 6), and the findings write-up (Task 7) — can run in parallel subagents alongside the inference work.

---

## Reference Facts (verified, do not re-derive)

| Item | Value |
|------|-------|
| Test video | `/home/giladn/Videos/Drone/Training/DJI_20260430103421_0010_D.MP4` |
| Stored stream resolution | 3384 (w) × 6016 (h) with `rotate=90` side data |
| Decoded ("display") resolution | **6016 × 3384** (landscape, ~5.5K) — `decodebin`/`avdec_hevc` applies the rotation matrix |
| Frames | 1035 @ 30000/1001 fps ≈ 34.5 s |
| Codec | HEVC (`avdec_h265` fallback after VAAPI disable, same trick as `tiling_bench.py` already uses) |
| Default HEF (GT + sweep) | `/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef` — the upstream tiling app's default model (`TILING_MODEL_NAME = "hailo_yolov8n_4_classes_vga"`, see `defines.py:258`). |
| Model input | **640 wide × 480 tall** (4:3) — matches the user's "each tile is 640×480 pixels" exactly |
| Network | `hailo_yolov8n_480_640`, 2-context multi-context, on-chip `yolov8_nms_postprocess` (HAILO NMS BY CLASS, 4 classes, 100 boxes/class) |
| Post-process .so | `libyolo_hailortpp_postprocess.so` with function `filter` — auto-detected from HEF metadata by `tiling_bench.py`; no override needed |
| Labels JSON | `/usr/local/hailo/resources/json/hailo_4_classes.json` — auto-detected by `hef_utils.py:get_hef_labels_json` from the HEF name |
| Detection threshold | 0.5 (from labels JSON `detection_threshold`); max 200 boxes total. |
| Classes (HEF NMS output) | indices 0–3 → label strings `person`, `vehicle`, `face`, `license_plate` (note: labels JSON has 5 entries because index 0 is `unlabeled`, then the four real classes shift by 1 in display — the post-process .so handles the offset). |
| Headline classes | **`person`** and **`vehicle`** (these are the two the user called out as "person and car"; "car" is folded into "vehicle" in this model). `face` and `license_plate` are still scored — they may be too small to ever cross threshold and are useful as a sanity-check for very-small-object recall. |
| Output root | `tiling_benchmark/pxt_runs/` — JSONs collected at the root, one pair per run label |

> **If the user later asks for a visdrone- or yolov8m-specific run:** the existing `tiling_benchmark/run_bench.py` covers visdrone SSD MobileNet; Task 8 (below) covers a small-scale yolov8m / visdrone comparison on a subset of the matrix. Don't break either path; `run_pxt_bench.py` is additive.

---

## File Structure

| File | Status | Responsibility |
|------|--------|----------------|
| `tiling_benchmark/run_pxt_bench.py` | **new** | Driver: experiment list, dispatches `tiling_bench.py` per config, prints comparison table, kicks off `analyze_pxt.py`. |
| `tiling_benchmark/analyze_pxt.py` | **new** | Loads GT + N candidate `*.frames.json` files; per-frame greedy IoU match per class; bins GT objects by source-pixel height; emits per-bin recall, per-class AP, and a CSV-friendly summary. |
| `tiling_benchmark/overlay_dets.py` | **new** | Stand-alone OpenCV viewer that scrubs the source video and overlays bboxes loaded from one or more `frames.json` files (no encoding). |
| `tiling_benchmark/tiling_bench.py` | **modify** | Add `--source-width` / `--source-height` pass-through so the driver can explicitly set the videoscale target instead of inheriting the default 1280×800. (Currently the only way to change it is `--width` / `--height`, which the parent parser already accepts — verify in Task 1 that those flags reach `video_width`/`video_height`. If yes, NO modify is needed.) |
| `tiling_benchmark/run_bench.py` | **untouched** | Existing visdrone SSD MobileNet sweep stays as is. |
| `docs/superpowers/plans/2026-05-20-tiling-benchmark-pxt-accuracy.md` | (this file) | The plan. |

---

## Experiment Matrix

The matrix sweeps **source resolution × tile grid**. Larger source × more tiles ⇒ higher pixels-per-target ⇒ should detect smaller objects. The driver runs every cell; the analyzer then plots recall against the *actual* per-object pixel height in the source frame (which is what the user actually cares about).

The model input is **640 × 480 (4:3)**, and the cropper does a centered-letterbox scale of each tile to that input. So a 4:3-aspect tile maps 1:1 (every input pixel is used), while a 16:9-aspect tile loses ~25% of the input area to top/bottom letterbox. The matrix is deliberately weighted toward 4:3-aspect tile grids (i.e. `tiles_y / tiles_x ≈ 0.75` for a 16:9 source) but keeps a few 16:9-aspect / square grids for contrast.

| Label | tiles_x × tiles_y | source W×H | overlap (x/y) | extras | tile px (source) | tile aspect | notes |
|-------|-------------------|------------|---------------|--------|------------------|-------------|-------|
| `GT-12x9-25` | 12 × 9 | 6016 × 3384 | 0.25 / 0.25 | — | 650 × 484 | 4:3 ✓ | matches model native; **the GT** |
| `1x1-native` | 1 × 1 | 6016 × 3384 | 0 / 0 | — | 6016 × 3384 | 16:9 | baseline; 9.4× downscale, letterboxed |
| `1x1-640x480` | 1 × 1 | 640 × 480 | 0 / 0 | — | 640 × 480 | 4:3 | source pre-downscaled to model native — the "no tiling" floor |
| `2x2-native` | 2 × 2 | 6016 × 3384 | 0.25 / 0.25 | — | 3437 × 1934 | 16:9 | square grid → 16:9 tiles |
| `3x2-native` | 3 × 2 | 6016 × 3384 | 0.25 / 0.25 | — | 2407 × 1934 | 5:4 | mid-aspect |
| `3x3-native` | 3 × 3 | 6016 × 3384 | 0.25 / 0.25 | — | 2407 × 1354 | 16:9 | square grid → 16:9 tiles |
| `4x3-native` | 4 × 3 | 6016 × 3384 | 0.25 / 0.25 | — | 1851 × 1354 | 4:3 ✓ | first proper 4:3 grid |
| `6x4-native` | 6 × 4 | 6016 × 3384 | 0.25 / 0.25 | — | 1262 × 1015 | ~5:4 | |
| `8x6-native` | 8 × 6 | 6016 × 3384 | 0.25 / 0.25 | — | 962 × 677 | 4:3 ✓ | |
| `4x3-native+full` | 4 × 3 | 6016 × 3384 | 0.25 / 0.25 | `--include-full-frame` | 1851×1354 + 6016×3384 | 4:3 + 16:9 | grid + whole-frame rescue for large objects |

**Why one GT (not two):** the 12×9 / 25 % overlap config gives tiles of 650 × 484 source pixels — within 2 % of the model's native 640 × 480 input on both axes. Going to 50 % overlap costs ~2.5× more inferences without buying much, because the existing 25 % overlap already provides ~163 px of horizontal slack between adjacent tile centers (no object thinner than that can fall between two tiles). One GT, less inference cost.

**Tile-size math:** `tile_px_w = source_w / (tiles_x · (1 - overlap_x) + overlap_x)`. The `hailotilecropper_dynamic` plugin computes the same way (confirmed by `tiling_benchmark/tiling_record.py:236`). Pixel counts above are precomputed for sanity-checking; if a row's tile size deviates from the table at runtime, re-check the overlap convention before panicking.

**Total inference cost (worst case):** 1035 frames × (108 + 1 + 1 + 4 + 6 + 9 + 12 + 24 + 48 + 13) ≈ 226 tiles/frame × 1035 ≈ 234 K inferences across all configs. The yolov8n 4-classes model on H10 is fast (small backbone, 2-context); estimate ~20–40 min wall-clock total (the dynamic cropper batches efficiently — `tiling_bench` already disables sync).

---

## Self-Review

Performed after first draft; corrections applied inline:

1. **Spec coverage:** ✓ The user asked for (a) GT at best resolution with multi-tile, (b) sweep at different tiling, (c) saved detections per experiment, (d) overlay capability later. All four are addressed by Tasks 3–9.
2. **Placeholder scan:** ✓ Searched for TBD/TODO/"fill in"/"appropriate" — none remain. Code blocks are filled.
3. **Type consistency:** ✓ `frames.json` schema (the existing `{label, config, video, frames: [{frame, second, pts_ns, detections: [{bbox, confidence, label, class_id?}]}]}`) is consumed unchanged by `analyze_pxt.py` and `overlay_dets.py`. `analyze_pxt.py` keys per-class results by the `label` string (e.g. `"person"`, `"car"`), matching how `analyze_bench.py` already does it (`det_class_key`).

---

## Task ordering & parallelism

Hailo-10H runs inferences serially. Tasks that drive the chip — **Task 0 (smoke), Task 2 (GT), Task 3 (sweep), Task 5 (analysis driver re-run), Task 8 (other networks)** — MUST run one-at-a-time, in that order. Tasks that don't touch the chip — **Task 1 (driver code), Task 4 (analyzer code), Task 6 (overlay viewer code), Task 7 (report)** — can run in parallel subagents alongside the chip-bound tasks (e.g. dispatch Task 4 & Task 6 as concurrent subagents while Task 2 is grinding through inferences).

Suggested dispatch order if running via `superpowers:subagent-driven-development`:

1. Task 0 (chip; sequential) — must finish before Task 2/3 start.
2. Task 1 (no chip) + Task 4 (no chip) + Task 6 (no chip) — concurrent subagents.
3. Task 2 (chip; sequential).
4. Task 3 (chip; sequential).
5. Task 5 (chip — but the bench JSONs are already on disk; this step only re-runs the analyzer driver, no inferences. Sequential just to keep the log clean.)
6. Task 7 (no chip) — can be dispatched concurrently with Task 5.
7. Task 8 (chip; sequential) — only when the user explicitly requests the "other networks" comparison.

---

## Task 0: Pre-flight — verify model + video roundtrip

> **Chip-bound. Must complete before Task 2.**

**Why first:** before we burn ~30 min of inferences, make sure `tiling_bench.py` will accept `hailo_yolov8n_4_classes_vga` end-to-end on this test video and that the source pipeline really hands us a 6016×3384 frame after rotation.

**Files:**
- Read-only: `/home/giladn/Videos/Drone/Training/DJI_20260430103421_0010_D.MP4`
- Read-only: `/home/giladn/tappas_apps/repos/hailo-drone-follow/tiling_benchmark/tiling_bench.py`

- [ ] **Step 1: Probe decoded resolution with a 2-second smoke run**

Run a 1×1 yolov8m bench against the test video, capped at native source dims, with VERY short duration. Use the existing script — no code changes yet.

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
source setup_env.sh
mkdir -p tiling_benchmark/pxt_runs
python tiling_benchmark/tiling_bench.py \
    --input file:///home/giladn/Videos/Drone/Training/DJI_20260430103421_0010_D.MP4 \
    --tiles-x 1 --tiles-y 1 --overlap-x 0 --overlap-y 0 \
    --width 6016 --height 3384 \
    --hef-path /usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef \
    --labels-json /usr/local/hailo/resources/json/hailo_4_classes.json \
    --bench-output tiling_benchmark/pxt_runs/smoke_1x1.json \
    --bench-label smoke_1x1 \
    --disable-sync 2>&1 | tail -40
```

Expected:
- "TILING CONFIGURATION" banner printed with `Input Resolution: 6016x3384` and `Model: hailo_yolov8n_4_classes_vga.hef (YOLO, 640x480)`
- Process runs to EOS in ~30 s
- `pxt_runs/smoke_1x1.json` exists; `pxt_runs/smoke_1x1.frames.json` exists with ~1035 frame entries
- Some detections produced (the GT will be far richer, but 1×1 should still find the biggest people/vehicles)

- [ ] **Step 2: Confirm bbox label strings**

```bash
python3 -c "
import json
d = json.load(open('tiling_benchmark/pxt_runs/smoke_1x1.frames.json'))
labels = {}
for fr in d['frames']:
    for det in fr['detections']:
        labels[det.get('label', '')] = labels.get(det.get('label', ''), 0) + 1
print(labels)
"
```

Expected output: a dict whose keys include `person` and `vehicle` (and likely `face` / `license_plate`), counts > 0. If the labels come back empty strings, the analyzer's class-key fallback to `cid=N` still works but the analysis output will be less readable — note it in the run log and move on.

- [ ] **Step 3: Decide whether `--width`/`--height` are sufficient**

If Step 1 produced `Input Resolution: 6016x3384` in the banner, then no modifications are needed to `tiling_bench.py`: the parent parser's `--width` / `--height` already drive the videoscale target. Skip the `--source-width`/`--source-height` addition mentioned in the File Structure table.

If instead the banner shows the default 1280×800 (i.e. `--width 6016 --height 3384` did not override), then `tiling_benchmark/tiling_bench.py:541` calls `parser.set_defaults(width=1280, height=800)` AFTER the parent parser is built, but `argparse.set_defaults` should still let CLI flags win — verify the actual values via `--help` and re-run with the canonical flag names. **Do not patch the parser unless this verification fails.**

- [ ] **Step 4: Commit the smoke output (or not)**

The smoke JSONs live outside the repo (`/home/giladn/Videos/...`) so there is nothing to git-add. Skip.

---

## Task 1: Add the pxt driver `tiling_benchmark/run_pxt_bench.py`

> **No chip. Safe to run in parallel with any other no-chip task.**

**Files:**
- Create: `/home/giladn/tappas_apps/repos/hailo-drone-follow/tiling_benchmark/run_pxt_bench.py`

- [ ] **Step 1: Create the driver**

Write the file end-to-end. It hardcodes the experiment list (matrix above) and dispatches `tiling_bench.py` per config. Output dir is parameterised but defaults to `tiling_benchmark/pxt_runs/`.

```python
"""Driver for the pixels-per-target tiling sweep.

Runs a hardcoded experiment matrix of (source resolution) × (tile grid)
through tiling_benchmark/tiling_bench.py, then invokes tiling_benchmark/analyze_pxt.py with the
highest-recall config as pseudo-ground-truth.

Usage:
  python tiling_benchmark/run_pxt_bench.py
  python tiling_benchmark/run_pxt_bench.py --skip-existing
  python tiling_benchmark/run_pxt_bench.py --only GT-12x9-25 1x1-native
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
BENCH_SCRIPT = HERE / "tiling_bench.py"
ANALYZE_SCRIPT = HERE / "analyze_pxt.py"

DEFAULT_VIDEO = "/home/giladn/Videos/Drone/Training/DJI_20260430103421_0010_D.MP4"
DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "pxt_runs"

# The tiling app's default HEF (hailo_yolov8n_4_classes_vga, 640x480 input,
# 4 classes: person/vehicle/face/license_plate). Pass HEF_PATH=None to let
# tiling_bench.py resolve it via the parent app's defaults — that path also
# auto-attaches hailo_4_classes.json. Setting HEF_PATH explicitly here keeps
# the run reproducible if the user's local defaults ever drift.
HEF_PATH = "/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef"
LABELS_JSON = "/usr/local/hailo/resources/json/hailo_4_classes.json"
# DO NOT pass --post-process-so / --post-function. The HEF has on-chip
# yolov8_nms_postprocess metadata; tiling_bench's auto-detect picks
# libyolo_hailortpp_postprocess.so + 'filter' from the HEF, which is correct.

# Each entry produces:
#   <out_dir>/pxt_<label>.json         — headline summary
#   <out_dir>/pxt_<label>.frames.json  — per-frame normalized detections
CONFIGS = [
    # Pseudo-ground-truth: small tiles (~650x484 source-px, near-perfect match
    # to the model's native 640x480 input). 25% overlap is enough — see plan
    # rationale.
    {"label": "GT-12x9-25", "tiles_x": 12, "tiles_y": 9, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": 6016, "source_h": 3384, "is_gt": True},

    # No-tiling baselines.
    {"label": "1x1-native",  "tiles_x": 1, "tiles_y": 1, "overlap_x": 0.0, "overlap_y": 0.0,
     "source_w": 6016, "source_h": 3384},
    {"label": "1x1-640x480", "tiles_x": 1, "tiles_y": 1, "overlap_x": 0.0, "overlap_y": 0.0,
     "source_w": 640, "source_h": 480},

    # Sweep at native source resolution, increasing tile count.
    {"label": "2x2-native",  "tiles_x": 2, "tiles_y": 2, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": 6016, "source_h": 3384},
    {"label": "3x2-native",  "tiles_x": 3, "tiles_y": 2, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": 6016, "source_h": 3384},
    {"label": "3x3-native",  "tiles_x": 3, "tiles_y": 3, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": 6016, "source_h": 3384},
    {"label": "4x3-native",  "tiles_x": 4, "tiles_y": 3, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": 6016, "source_h": 3384},
    {"label": "6x4-native",  "tiles_x": 6, "tiles_y": 4, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": 6016, "source_h": 3384},
    {"label": "8x6-native",  "tiles_x": 8, "tiles_y": 6, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": 6016, "source_h": 3384},

    # Hybrid: 4x3 (4:3-aspect tiles) + whole-frame rescue for large objects.
    {"label": "4x3-native+full", "tiles_x": 4, "tiles_y": 3, "overlap_x": 0.25, "overlap_y": 0.25,
     "source_w": 6016, "source_h": 3384, "include_full_frame": True},
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

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if not Path(args.video).is_file():
        print(f"ERROR: video not found: {args.video}", file=sys.stderr)
        sys.exit(1)

    selected = CONFIGS
    if args.only:
        keep = set(args.only)
        selected = [c for c in CONFIGS if c["label"] in keep]
        missing = keep - {c["label"] for c in CONFIGS}
        if missing:
            print(f"ERROR: unknown labels in --only: {sorted(missing)}", file=sys.stderr)
            sys.exit(1)

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
                   "--out", str(out_dir / "pxt_analysis.json")]
    for p in pred_frames:
        analyze_cmd += ["--pred", str(p)]
    print("\n=== Running analyze_pxt.py ===")
    print(" ".join(analyze_cmd), flush=True)
    subprocess.run(analyze_cmd)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Sanity-check the script parses and lists configs**

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
source setup_env.sh
python tiling_benchmark/run_pxt_bench.py --help
python tiling_benchmark/run_pxt_bench.py --only NOPE 2>&1 | head -5
```

Expected:
- `--help` prints the four flags
- `--only NOPE` exits non-zero with `ERROR: unknown labels in --only: ['NOPE']`

- [ ] **Step 3: Commit**

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
git add tiling_benchmark/run_pxt_bench.py
git commit -m "bench: add pixels-per-target tiling sweep driver

run_pxt_bench.py defines a (source resolution × tile grid) experiment
matrix and dispatches tiling_benchmark/tiling_bench.py per config. Output JSONs land
under tiling_benchmark/pxt_runs/. Drives analyze_pxt.py
post-run with the GT-12x9-25 config flagged as pseudo-ground-truth."
```

---

## Task 2: Run the pseudo-ground-truth config first (longest job, want to fail fast)

> **Chip-bound. Sequential.** Do not start while Task 0 / Task 3 / Task 5 / Task 8 are running. No-chip tasks (1, 4, 6, 7) can run in parallel subagents.

**Files:**
- Read-only: `tiling_benchmark/run_pxt_bench.py`
- Output: `tiling_benchmark/pxt_runs/pxt_GT-12x9-25.json` + `.frames.json`

- [ ] **Step 1: Estimate runtime by extrapolation**

108 tiles/frame × 1035 frames = 111 780 inferences. yolov8m on H10 sustains ~80–120 fps single-tile (no overlap); with the cropper batching, expect 20–40 min wall-clock. **Do NOT block on this — run it in background with a log file.**

- [ ] **Step 2: Kick off the GT run**

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
source setup_env.sh
mkdir -p tiling_benchmark/pxt_runs
nohup python tiling_benchmark/run_pxt_bench.py --only GT-12x9-25 --skip-analyze \
    > tiling_benchmark/pxt_runs/run_gt.log 2>&1 &
echo "GT PID: $!"
```

- [ ] **Step 3: Tail the log until you see EOS / BENCH RESULT**

```bash
tail -f tiling_benchmark/pxt_runs/run_gt.log
```

Expected (eventually):
- A `TILING CONFIGURATION` banner with `Input Resolution: 6016x3384`, `Model: yolov8m.hef (YOLO, 640x640)`
- A `BENCH RESULT label='GT-12x9-25' frames=1035 frames_with_det_pct=…` line on EOS
- `BENCH WALL_CLOCK_S=…` line right after

If `frames=1035` is not the count you see, decoder/EOS issue — debug before running the rest of the matrix.

- [ ] **Step 4: Spot-check GT detection density**

```bash
python3 -c "
import json
d = json.load(open('tiling_benchmark/pxt_runs/pxt_GT-12x9-25.frames.json'))
from collections import Counter
labs = Counter()
sizes = []
for fr in d['frames']:
    for det in fr['detections']:
        labs[det.get('label', '?')] += 1
        # bbox is [x, y, w, h] normalized 0-1
        sizes.append(det['bbox'][3])  # height
print('labels:', labs.most_common(10))
print('count:', len(sizes))
if sizes:
    sizes.sort()
    print('height percentiles (norm):', sizes[len(sizes)//20], sizes[len(sizes)//4],
          sizes[len(sizes)//2], sizes[3*len(sizes)//4], sizes[19*len(sizes)//20])
"
```

Expected: `person` and `car` (or similar COCO labels) dominate; per-frame avg det count ≫ 1; height percentiles include some small (< 0.02) entries — that's the whole point.

- [ ] **Step 5: No commit** — output files are outside the repo.

---

## Task 3: Run the comparison sweep (everything except GT)

> **Chip-bound. Sequential.** Starts after Task 2 finishes.

**Files:**
- Read-only: `tiling_benchmark/run_pxt_bench.py`
- Outputs: ten more `pxt_*.json` + `.frames.json` files

- [ ] **Step 1: Kick off**

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
source setup_env.sh
nohup python tiling_benchmark/run_pxt_bench.py --skip-existing --skip-analyze \
    > tiling_benchmark/pxt_runs/run_sweep.log 2>&1 &
echo "sweep PID: $!"
```

(`--skip-existing` re-uses the already-finished GT-12x9-25.)

- [ ] **Step 2: Monitor**

```bash
tail -f tiling_benchmark/pxt_runs/run_sweep.log
```

Expect ~10–15 minutes total for the small grids (1×1, 2×2, 3×2, 3×3) and another ~10 minutes for 4×3 / 6×4 / 8×6.

- [ ] **Step 3: Confirm all outputs exist**

```bash
ls -1 tiling_benchmark/pxt_runs/pxt_*.frames.json | wc -l
```

Expected: **11** files (one per config in the matrix).

If any are missing, identify which from the log (look for a config whose `BENCH RESULT` line never printed) and re-run with `--only LABEL`.

- [ ] **Step 4: No commit.**

---

## Task 4: Add the pixels-per-target analyzer `tiling_benchmark/analyze_pxt.py`

> **No chip. Safe to dispatch in parallel with Task 2 / Task 3.**

**Files:**
- Create: `/home/giladn/tappas_apps/repos/hailo-drone-follow/tiling_benchmark/analyze_pxt.py`

- [ ] **Step 1: Create the analyzer**

```python
"""Pixels-per-target analyzer for the tiling sweep.

Like analyze_bench.py (greedy IoU per-frame, all-points AP) but adds a
*size-binned recall* table: bin each GT object by its height in source
pixels, then per candidate config, count how many of those GT objects were
recovered (IoU >= --iou with a same-class prediction).

The output is the practical answer the user is asking for: "for a 30-px
person, do I need a 3x2 grid or a 6x4 grid?". Headline classes are
'person' and 'vehicle' (the two classes the hailo_yolov8n_4_classes_vga
HEF emits that the user cares about — 'car' is folded into 'vehicle').

Usage:
  python tiling_benchmark/analyze_pxt.py \\
      --gt tiling_benchmark/pxt_runs/pxt_GT-12x9-25.frames.json \\
      --pred tiling_benchmark/pxt_runs/pxt_1x1-native.frames.json \\
      --pred tiling_benchmark/pxt_runs/pxt_3x2-native.frames.json \\
      --video-w 6016 --video-h 3384 \\
      --out tiling_benchmark/pxt_runs/pxt_analysis.json
"""

import argparse
import json
from pathlib import Path

# Reuse the iou + label-key helpers from analyze_bench.py — same package dir,
# no __init__.py, so add the parent to sys.path.
import sys
HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
from analyze_bench import iou_xywh, det_class_key, index_by_frame, load_frames  # noqa: E402

# Object-height bins in *source pixels*. Each (lo, hi] in px height. The bins
# straddle the small/medium/large boundaries that matter for tiling decisions.
DEFAULT_BINS_PX = [(0, 16), (16, 32), (32, 64), (64, 128), (128, 256), (256, 4096)]
# Classes the user explicitly cares about. Everything else is still scored,
# just not headlined. (hailo_yolov8n_4_classes_vga emits person / vehicle /
# face / license_plate — face & license_plate are too small for this dataset
# but kept in the JSON for completeness.)
HEADLINE_CLASSES = ("person", "vehicle")


def match_frame_size_aware(
    gt_dets: list[dict],
    pred_dets: list[dict],
    iou_thresh: float,
    video_h_px: int,
):
    """Per-frame greedy matching identical to analyze_bench.match_frame, BUT
    we also return, for each GT, (matched_bool, height_px) so the caller can
    bin GTs by size before aggregating recall.
    """
    pred_sorted = sorted(
        enumerate(pred_dets),
        key=lambda kv: -float(kv[1].get("confidence", 0.0)),
    )
    used_gt: set[int] = set()
    # also remember which pred is a TP — irrelevant for size-binned recall but
    # useful for keeping the AP path identical.
    for _, p in pred_sorted:
        p_class = det_class_key(p)
        best_iou = 0.0
        best_gi = -1
        for gi, g in enumerate(gt_dets):
            if gi in used_gt:
                continue
            if det_class_key(g) != p_class:
                continue
            i = iou_xywh(p["bbox"], g["bbox"])
            if i > best_iou:
                best_iou = i
                best_gi = gi
        if best_gi >= 0 and best_iou >= iou_thresh:
            used_gt.add(best_gi)

    gt_results = []
    for gi, g in enumerate(gt_dets):
        # bbox is [x, y, w, h] normalized 0-1
        h_norm = float(g["bbox"][3])
        h_px = h_norm * video_h_px
        gt_results.append((det_class_key(g), h_px, gi in used_gt))
    return gt_results


def bin_for_height(h_px: float, bins) -> int:
    for i, (lo, hi) in enumerate(bins):
        if lo < h_px <= hi:
            return i
    return len(bins) - 1


def analyze_one(gt_doc: dict, pred_doc: dict, iou_thresh: float,
                video_h_px: int, bins) -> dict:
    gt_idx = index_by_frame(gt_doc)
    pred_idx = index_by_frame(pred_doc)
    all_frames = sorted(set(gt_idx.keys()) | set(pred_idx.keys()))

    # bin_recall[cls][bin_idx] = (matched, total)
    bin_recall: dict[str, list[list[int]]] = {}
    overall_totals: dict[str, list[int]] = {}  # (matched, total) — all sizes
    for fr in all_frames:
        gts = gt_idx.get(fr, [])
        preds = pred_idx.get(fr, [])
        # Bucket by class so matching is per-class (mirrors analyze_bench).
        gts_by_class: dict[str, list[dict]] = {}
        preds_by_class: dict[str, list[dict]] = {}
        for g in gts:
            gts_by_class.setdefault(det_class_key(g), []).append(g)
        for p in preds:
            preds_by_class.setdefault(det_class_key(p), []).append(p)
        seen_classes = set(gts_by_class.keys()) | set(preds_by_class.keys())
        for cls in seen_classes:
            cls_gts = gts_by_class.get(cls, [])
            cls_preds = preds_by_class.get(cls, [])
            results = match_frame_size_aware(cls_gts, cls_preds, iou_thresh, video_h_px)
            if cls not in bin_recall:
                bin_recall[cls] = [[0, 0] for _ in bins]
                overall_totals[cls] = [0, 0]
            for cls_key, h_px, matched in results:
                # cls_key == cls by construction here; left for clarity.
                bi = bin_for_height(h_px, bins)
                bin_recall[cls][bi][1] += 1
                overall_totals[cls][1] += 1
                if matched:
                    bin_recall[cls][bi][0] += 1
                    overall_totals[cls][0] += 1

    return {
        "bins_px": [list(b) for b in bins],
        "per_class_bin": {
            cls: [
                {
                    "bin_lo": bins[i][0],
                    "bin_hi": bins[i][1],
                    "matched": bin_recall[cls][i][0],
                    "n_gt": bin_recall[cls][i][1],
                    "recall": (bin_recall[cls][i][0] / bin_recall[cls][i][1])
                              if bin_recall[cls][i][1] else None,
                }
                for i in range(len(bins))
            ]
            for cls in bin_recall
        },
        "per_class_overall": {
            cls: {
                "matched": overall_totals[cls][0],
                "n_gt": overall_totals[cls][1],
                "recall": (overall_totals[cls][0] / overall_totals[cls][1])
                          if overall_totals[cls][1] else None,
            }
            for cls in overall_totals
        },
    }


def print_size_recall_table(results: list[tuple[str, dict]], bins) -> None:
    """One table per headline class. Rows = bins; columns = configs."""
    all_classes = set()
    for _, r in results:
        all_classes.update(r["per_class_bin"].keys())
    classes_sorted = sorted(all_classes, key=lambda c:
                            (0, HEADLINE_CLASSES.index(c)) if c in HEADLINE_CLASSES
                            else (1, c))
    for cls in classes_sorted:
        # n_gt is the same per-bin across configs (same GT input).
        n_gt_per_bin = None
        for _, r in results:
            pc = r["per_class_bin"].get(cls)
            if pc:
                n_gt_per_bin = [b["n_gt"] for b in pc]
                break
        if not n_gt_per_bin:
            continue
        print()
        print("=" * (32 + 11 * len(results)))
        print(f"Size-binned recall — class: {cls}")
        header = f"{'bin_px':<14} {'n_gt':>8}"
        for lbl, _ in results:
            header += f" {lbl[:9]:>10}"
        print(header)
        print("-" * (32 + 11 * len(results)))
        for i, (lo, hi) in enumerate(bins):
            row = f"({lo:>4},{hi:>4}]    {n_gt_per_bin[i]:>8}"
            for _, r in results:
                pc = r["per_class_bin"].get(cls)
                if not pc or pc[i]["recall"] is None:
                    row += f" {'-':>10}"
                else:
                    row += f" {pc[i]['recall']*100:>9.1f}%"
            print(row)
        # overall row
        total_gt = sum(n_gt_per_bin)
        row = f"{'TOTAL':<14} {total_gt:>8}"
        for _, r in results:
            ov = r["per_class_overall"].get(cls)
            if not ov or ov["recall"] is None:
                row += f" {'-':>10}"
            else:
                row += f" {ov['recall']*100:>9.1f}%"
        print(row)
        print("=" * (32 + 11 * len(results)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gt", type=Path, required=True)
    ap.add_argument("--pred", type=Path, action="append", required=True)
    ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--video-w", type=int, required=True,
                    help="GT source width in pixels (for binning).")
    ap.add_argument("--video-h", type=int, required=True,
                    help="GT source height in pixels (for binning).")
    ap.add_argument("--out", type=Path, default=None,
                    help="Optional path to dump the full analysis as JSON.")
    args = ap.parse_args(argv)

    gt_doc = load_frames(args.gt)
    print(f"GT: {args.gt} ({len(gt_doc['frames'])} frames, "
          f"{sum(len(f['detections']) for f in gt_doc['frames'])} dets)")

    bins = DEFAULT_BINS_PX
    results: list[tuple[str, dict]] = []
    full_dump = {"gt": str(args.gt),
                 "video_w": args.video_w, "video_h": args.video_h,
                 "iou": args.iou,
                 "bins_px": [list(b) for b in bins],
                 "configs": {}}
    for p in args.pred:
        pred_doc = load_frames(p)
        label = pred_doc.get("label") or p.stem
        n_dets = sum(len(f["detections"]) for f in pred_doc["frames"])
        print(f"Pred: {p} (label={label!r}, {n_dets} dets)")
        r = analyze_one(gt_doc, pred_doc, args.iou, args.video_h, bins)
        results.append((label, r))
        full_dump["configs"][label] = r

    print_size_recall_table(results, bins)

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("w") as f:
            json.dump(full_dump, f, indent=2)
        print(f"\nFull analysis dumped to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 2: Smoke-test on a single pair**

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
source setup_env.sh
python tiling_benchmark/analyze_pxt.py \
    --gt tiling_benchmark/pxt_runs/pxt_GT-12x9-25.frames.json \
    --pred tiling_benchmark/pxt_runs/pxt_1x1-native.frames.json \
    --video-w 6016 --video-h 3384 \
    --iou 0.5
```

Expected:
- Two `Size-binned recall — class: person` / `car` tables
- The `1x1-native` column shows ~0% recall in the small bins (0–16 px, 16–32 px) and rising recall for larger sizes — exactly the curve we're trying to characterise.

- [ ] **Step 3: Commit**

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
git add tiling_benchmark/analyze_pxt.py
git commit -m "bench: add size-binned recall analyzer for pxt sweep

analyze_pxt.py reuses analyze_bench.iou + class-key helpers and adds a
per-bin recall table — bin GT objects by height in source pixels, then
per candidate config count how many GTs were recovered. Headlines the
'person' and 'car' classes; everything else is still scored."
```

---

## Task 5: Run the full analysis across all configs

> **No new inferences.** The driver short-circuits to `analyze_pxt.py` because every bench JSON already exists from Tasks 2–3. Safe to run in parallel with Task 7.

**Files:**
- Read-only: 11 `pxt_*.frames.json`
- Output: `tiling_benchmark/pxt_runs/pxt_analysis.json`

- [ ] **Step 1: Drive from the runner**

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
source setup_env.sh
python tiling_benchmark/run_pxt_bench.py --skip-existing \
    2>&1 | tee tiling_benchmark/pxt_runs/full.log
```

(All bench JSONs already exist from Tasks 2–3, so this will skip straight to the analysis step.)

Expected:
- Comparison table printed
- `=== Running analyze_pxt.py ===` block followed by the per-class size tables
- `Full analysis dumped to .../pxt_analysis.json`

- [ ] **Step 2: Eyeball the headline answer**

For each headline class, find the *smallest* bin where recall ≥ 80% (an editorial cut-off — pick whatever the user prefers). Note the tile size in source pixels for that config. That gives the answer to the user's question:

> "To reliably detect a [person|car] of height H px in the source video, you need a tile layout that gives at least T model-input pixels per object — i.e. a tile of ≤ S source-pixels wide."

- [ ] **Step 3: No commit** — analysis output is data, not code.

---

## Task 6: Add the overlay viewer `tiling_benchmark/overlay_dets.py`

> **No chip. Safe to dispatch in parallel with Task 2 / Task 3.**

**Files:**
- Create: `/home/giladn/tappas_apps/repos/hailo-drone-follow/tiling_benchmark/overlay_dets.py`

- [ ] **Step 1: Create the viewer**

OpenCV-based — opens the source video, reads the frames.json(s), and renders the bboxes per frame. Different colour per source file so multiple configs can be compared visually. No video file is written.

```python
"""On-the-fly overlay viewer for pxt sweep frames.json files.

Reads the source video frame-by-frame; for each frame, looks up the
detections from one or more frames.json files (matched by frame index) and
draws them with a per-file colour. Press SPACE to pause, Q to quit, [/] to
step backward/forward when paused, +/- to change playback speed.

Usage:
  python tiling_benchmark/overlay_dets.py \\
      --video /home/giladn/Videos/Drone/Training/DJI_20260430103421_0010_D.MP4 \\
      --frames tiling_benchmark/pxt_runs/pxt_GT-12x9-25.frames.json:GT \\
      --frames tiling_benchmark/pxt_runs/pxt_3x2-native.frames.json:3x2

The `:label` suffix is optional; defaults to the JSON's own label field or
the filename stem if absent.
"""

import argparse
import json
from pathlib import Path

import cv2

# Distinct BGR colours — keep small so the legend stays readable.
PALETTE = [
    (0, 255, 0),    # green
    (0, 0, 255),    # red
    (255, 0, 0),    # blue
    (0, 255, 255),  # yellow
    (255, 0, 255),  # magenta
    (255, 255, 0),  # cyan
    (255, 128, 0),  # orange
    (128, 0, 255),  # purple
]


def load_frames_indexed(path: Path) -> tuple[str, dict[int, list[dict]]]:
    with path.open() as f:
        doc = json.load(f)
    idx = {int(fr["frame"]): fr["detections"] for fr in doc["frames"]}
    return doc.get("label") or path.stem, idx


def parse_frames_arg(arg: str) -> tuple[Path, str | None]:
    if ":" in arg:
        p, lbl = arg.rsplit(":", 1)
        return Path(p), lbl
    return Path(arg), None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--video", type=Path, required=True)
    ap.add_argument("--frames", action="append", required=True,
                    help="frames.json path, optionally suffixed with ':LABEL'. "
                         "Repeat for multiple overlays.")
    ap.add_argument("--start-frame", type=int, default=0)
    ap.add_argument("--conf-min", type=float, default=0.25,
                    help="Don't draw boxes below this confidence (default 0.25).")
    args = ap.parse_args(argv)

    overlays = []
    for raw in args.frames:
        p, lbl_override = parse_frames_arg(raw)
        if not p.is_file():
            print(f"ERROR: frames file not found: {p}")
            return 1
        lbl, idx = load_frames_indexed(p)
        if lbl_override:
            lbl = lbl_override
        overlays.append((lbl, idx))

    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        print(f"ERROR: cannot open video {args.video}")
        return 1

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    base_delay_ms = max(1, int(1000.0 / fps))
    delay_ms = base_delay_ms
    paused = False
    frame_no = args.start_frame
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_no)

    print(f"Video: {total} frames @ {fps:.2f} fps. "
          f"Loaded {len(overlays)} overlay(s). "
          f"Controls: SPACE=pause, Q=quit, [/]=step (paused), +/-=speed.")
    print("Legend:")
    for i, (lbl, _) in enumerate(overlays):
        c = PALETTE[i % len(PALETTE)]
        print(f"  {lbl}: BGR{c}")

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok:
                print("End of video.")
                break
        else:
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_no)
            ok, frame = cap.read()
            if not ok:
                break

        h, w = frame.shape[:2]
        for i, (lbl, idx) in enumerate(overlays):
            colour = PALETTE[i % len(PALETTE)]
            for det in idx.get(frame_no, []):
                if float(det.get("confidence", 0.0)) < args.conf_min:
                    continue
                bx, by, bw, bh = det["bbox"]
                x1 = int(bx * w); y1 = int(by * h)
                x2 = int((bx + bw) * w); y2 = int((by + bh) * h)
                cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                tag = f"{det.get('label', '?')} {det.get('confidence', 0):.2f}"
                cv2.putText(frame, tag, (x1, max(15, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)

        # Legend / HUD overlay
        hud = f"frame {frame_no}/{total - 1}  speed={base_delay_ms / delay_ms:.2f}x  "
        hud += "PAUSED" if paused else "PLAY"
        cv2.putText(frame, hud, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2, cv2.LINE_AA)
        for i, (lbl, _) in enumerate(overlays):
            cv2.putText(frame, lbl, (10, 50 + 22 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        PALETTE[i % len(PALETTE)], 2, cv2.LINE_AA)

        # Fit-to-screen-ish: 6016x3384 won't fit; downscale for display only.
        if w > 1920:
            scale = 1920 / w
            disp = cv2.resize(frame, (int(w * scale), int(h * scale)))
        else:
            disp = frame
        cv2.imshow("pxt overlay", disp)

        key = cv2.waitKey(0 if paused else delay_ms) & 0xFF
        if key == ord('q') or key == 27:  # Q or ESC
            break
        elif key == ord(' '):
            paused = not paused
        elif paused and key == ord(']'):
            frame_no = min(total - 1, frame_no + 1)
        elif paused and key == ord('['):
            frame_no = max(0, frame_no - 1)
        elif key in (ord('+'), ord('=')):
            delay_ms = max(1, delay_ms // 2)
        elif key == ord('-'):
            delay_ms = min(2000, delay_ms * 2)

        if not paused:
            frame_no += 1

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 2: Smoke-test (requires a display)**

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
source setup_env.sh
DISPLAY=:0 python tiling_benchmark/overlay_dets.py \
    --video /home/giladn/Videos/Drone/Training/DJI_20260430103421_0010_D.MP4 \
    --frames tiling_benchmark/pxt_runs/pxt_GT-12x9-25.frames.json:GT \
    --frames tiling_benchmark/pxt_runs/pxt_1x1-native.frames.json:1x1
```

Expected:
- An OpenCV window opens; you see the drone footage with green boxes (GT) and red boxes (1x1) overlaid.
- The 1x1 boxes will be a strict subset of the GT boxes (the whole point of this exercise).

If no display is available (headless SSH), skip and rely on the JSON analysis from Task 5.

- [ ] **Step 3: Commit**

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
git add tiling_benchmark/overlay_dets.py
git commit -m "bench: add OpenCV overlay viewer for pxt frames.json

overlay_dets.py opens the source video and renders bboxes from one or more
frames.json files with distinct per-file colours. SPACE=pause, Q=quit,
[/] step frame-by-frame, +/- adjust speed. No annotated video is written."
```

---

## Task 7: Synthesize the final answer for the user

> **No chip.** Pure writing — can be dispatched as a subagent concurrent with Task 5.

**Files:**
- Read-only: `tiling_benchmark/pxt_runs/pxt_analysis.json`
- Output: stdout report (no new file)

- [ ] **Step 1: Read the per-class bin tables and identify the "knee"**

For each of `person` and `car`:
1. Read the size-binned recall printed by `analyze_pxt.py`.
2. For each config, find the *smallest* bin where recall ≥ 80%.
3. That bin's lower bound (in source px) is the **effective pixels-per-target floor** for that config.
4. Map config → tile size in source px (from the matrix above).
5. The pair `(tile_size_src_px, smallest_detectable_height_src_px)` is the headline answer.

- [ ] **Step 2: Write a short markdown report**

Append a section to `docs/superpowers/plans/2026-05-20-tiling-benchmark-pxt-accuracy.md` (this file) titled `## Findings` with:

- a small markdown table of (config, smallest 80%-recall bin) per class
- a one-line recommendation, e.g. "to reliably catch a person at 32 px height in source, use ≥ 4×3 tiling at native resolution; cars at the same source height need only 3×2".

This step is human-judgement-only — no automation. If the recall curve is noisy (e.g. one config beats a denser one in one bin), call that out as a caveat instead of papering over it.

- [ ] **Step 3: Commit the report**

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
git add docs/superpowers/plans/2026-05-20-tiling-benchmark-pxt-accuracy.md
git commit -m "docs: record findings of the pxt sweep

Adds the headline pixels-per-target threshold per class to the plan doc."
```

---

## Task 8: Other-networks comparison (deferred — small subset only)

> **Chip-bound. Sequential.** Only run after Tasks 0–7 are complete and the user explicitly requests it. **Does NOT run the full sweep matrix** — the network swap is informative on a handful of cells, not all ten. Other networks have different class spaces, so cross-model AP via `analyze_pxt.py` is intentionally NOT scored across models (each model is compared only against the GT *of the same model* — see Step 2 below).

**Why deferred:** the user said they want "a test with other networks, but they will not run on the entire tiling sweep". This task is here so future-me knows the intent; do not execute until asked.

**Files (when executed):**
- Create: `tiling_benchmark/run_pxt_other_networks.py` — variant driver that takes a HEF path + label set + a *subset* of the matrix.
- Modify: `tiling_benchmark/analyze_pxt.py` — no change needed; it's class-name-keyed and works for any model.

**Candidate networks (already on disk):**

| HEF | Input | Classes | Post-process |
|-----|-------|---------|--------------|
| `/usr/local/hailo/resources/models/hailo10h/yolov8m.hef` | 640×640 | COCO 80 | auto (yolo NMS on-chip → `libyolo_hailortpp_postprocess.so` + `filter`) |
| `/home/giladn/Desktop/Visdrone/h10/ssd_mobilenet_v1_visdrone.hef` | 300×300 | VisDrone 10 (incl. `pedestrian`, `people`, `car`, `van`, `truck`, …) | manual: `libmobilenet_ssd_postprocess.so` + `mobilenet_ssd_visdrone`, labels `/usr/local/hailo/resources/json/visdrone.json` |
| `/usr/local/hailo/resources/models/hailo10h/yolov6n.hef` | 640×640 | COCO 80 | auto (yolo NMS on-chip) |

**Subset matrix per network** (5 cells, not 10):

| Label | tiles_x × tiles_y | overlap | source W×H | rationale |
|-------|-------------------|---------|------------|-----------|
| `<net>-GT-12x9-25` | 12 × 9 | 0.25 / 0.25 | 6016 × 3384 | per-network GT — required for the per-network analyzer pass |
| `<net>-1x1-native` | 1 × 1 | 0 / 0 | 6016 × 3384 | floor |
| `<net>-3x2-native` | 3 × 2 | 0.25 / 0.25 | 6016 × 3384 | mid grid |
| `<net>-4x3-native` | 4 × 3 | 0.25 / 0.25 | 6016 × 3384 | 4:3-aspect grid (matches both yolov8 and ssd letterbox) |
| `<net>-6x4-native` | 6 × 4 | 0.25 / 0.25 | 6016 × 3384 | denser 4:3-aspect grid |

3 networks × 5 configs = 15 runs. Wall-clock estimate: ~30 min for yolov8n-4-class (already done), ~60–90 min for yolov8m, ~20 min for SSD MobileNet (small model, but 300×300 input → 4× the tile count needed for the same pxt; this row is mostly here to characterise the model, not the tiling).

- [ ] **Step 1: Write `tiling_benchmark/run_pxt_other_networks.py`**

Copy `tiling_benchmark/run_pxt_bench.py` to a new file, replace `CONFIGS` with the 5-row subset above, and add three more constants for the network. Take the network choice from a CLI flag so the same driver covers all three:

```python
NETS = {
    "yolov8m": {
        "hef": "/usr/local/hailo/resources/models/hailo10h/yolov8m.hef",
        "labels_json": None,   # COCO labels live in HEF metadata
        "post_so": None,       # auto-detect
        "post_fn": None,       # auto-detect
    },
    "visdrone_ssd": {
        "hef": "/home/giladn/Desktop/Visdrone/h10/ssd_mobilenet_v1_visdrone.hef",
        "labels_json": "/usr/local/hailo/resources/json/visdrone.json",
        "post_so": "/usr/local/hailo/resources/so/libmobilenet_ssd_postprocess.so",
        "post_fn": "mobilenet_ssd_visdrone",
    },
    "yolov6n": {
        "hef": "/usr/local/hailo/resources/models/hailo10h/yolov6n.hef",
        "labels_json": None,
        "post_so": None,
        "post_fn": None,
    },
}
```

The runner takes `--net {yolov8m,visdrone_ssd,yolov6n}` and prefixes all output labels with the net name so files don't collide with the main pxt run.

- [ ] **Step 2: Run a per-network analysis only against that network's own GT**

Each network's classes are different — `vehicle` (4-class), `car` + `truck` + `van` (visdrone), `car` (yolov8m). Cross-model AP is meaningless. So for each network:

1. Run that network's `<net>-GT-12x9-25` config.
2. Run the four other configs for that network.
3. Invoke `analyze_pxt.py --gt <net>-GT-12x9-25.frames.json --pred <net>-1x1...` — produces a per-network size-vs-recall table.
4. Headline class per network: `--headline-classes person vehicle` (for 4-class); `--headline-classes person car` (yolov8m); `--headline-classes pedestrian car` (visdrone). This requires a small CLI add to `analyze_pxt.py` — see Step 3.

- [ ] **Step 3: Extend `analyze_pxt.py` to accept `--headline-classes`**

Replace the module-level `HEADLINE_CLASSES = ("person", "vehicle")` with a CLI flag:

```python
ap.add_argument("--headline-classes", nargs="+", default=["person", "vehicle"],
                help="Class names to print first in the per-class table; others "
                     "are still scored, just listed after.")
```

Propagate via the existing argparse → main pathway and through to `print_size_recall_table` (which currently reads the constant). No other code changes needed.

- [ ] **Step 4: Cross-network summary table (manual)**

Once each network's per-size-bin recall is in JSON, the user can eyeball "at what source-pixel height does yolov8n-4class match / beat yolov8m / visdrone-ssd?". Build a one-table summary in the report from Task 7. No automated cross-net join — the class names don't align, and forcing a join would lie to the reader.

- [ ] **Step 5: Commit when done**

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
git add tiling_benchmark/run_pxt_other_networks.py tiling_benchmark/analyze_pxt.py
git commit -m "bench: add other-networks pxt comparison (subset of matrix)

run_pxt_other_networks.py drives yolov8m / visdrone-ssd / yolov6n over a
5-cell subset of the main pxt sweep matrix. analyze_pxt.py grows a
--headline-classes flag so non-4class label spaces can be highlighted."
```

---

## Out-of-scope (explicitly NOT in this plan)

- **Re-training the model**: this whole exercise treats the network as a fixed black box.
- **Annotated video export**: the `overlay_dets.py` viewer is on-the-fly only, per the user's "no need to save the annotated video" constraint. If a saved video is later needed, add a `--write-out PATH` to that script — small change.
- **Full pxt sweep on other networks**: explicitly scoped down to a 5-cell subset in Task 8.
- **Cross-network mAP**: class spaces don't align (4-class `vehicle` ≠ COCO `car` ≠ visdrone `car`+`van`+`truck`). Each network is scored only against its own GT.
- **GPU/CPU baseline**: comparing Hailo-8/10H output to a CUDA YOLOv8 run is not part of this. The "GT" here is "highest-recall Hailo run", not "ground truth from a different network".
- **Manual labelling**: no human GT. If the recall numbers turn out to look suspicious (e.g. 100% across all bins), that's the GT having the same blind spots as the candidates, and the only fix is a real labelled set — flag and stop.
