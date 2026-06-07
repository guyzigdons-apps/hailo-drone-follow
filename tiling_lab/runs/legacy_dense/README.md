# `legacy_dense/` — dense + static-grid tiling artifacts (provenance)

These are the raw run artifacts behind the committed tiling reports. They were
migrated here on **2026-06-07** from `tiling_benchmark/pxt_runs*/` during the
`dynamic_tiling/` → `tiling_lab/` restructure. None of this is imported by any
package — it is reference data for the frozen `tiling_benchmark` runners and the
exported insight reports.

All clip references below are FOV-prepared center-crops of the source training
clips under `~/Videos/Drone/Training/` (prepared via the
`prepare_video.fov_to_crop_dims` pipeline; the live copy now lives at
`tiling_lab/video/prepare_video.py`).

## Inventory

| Artifact | What it is |
|---|---|
| `pxt_GT-12x9-25-multi.frames.json` / `.json` | **Dense GT detection source** — heavy 12×9-tile (+1×1, +3×2 extra grids, +1 extra ROI) dense-detection pass. Source clip `DJI_20260430103421_0010_D_rotated.MP4` (100 fps, ~35 s). This is the detection base that the locked **clip 0025** GT chain (`../gt_verify_0025_fov{50,60,70}/`) was built and reviewed from. `.frames.json` = per-frame normalized detections; `.json` = the per-second summary + run config. |
| `pxt_{1x1,2x2,3x2,3x3,4x3,6x4,8x6}-native.frames.json` / `.json` | **Static-grid baselines** — the equal-budget static tiling ablation passes (one grid per file) that the committed reports compare the dynamic scheduler against. `-native` = decode at native tile resolution. |
| `pxt_{2x2,3x2,3x3,4x3,6x4,8x6}-native+vga3x.frames.json` / `.json` | Same static grids re-run with a 3× VGA upscale variant (resolution-vs-tiles probe). |
| `pxt_4x3-native+full.frames.json` / `.json` | 4×3 grid plus a full-frame pass (mixed-scale probe). |
| `pxt_analysis.json` | Aggregated comparison table across the static-grid runs above. |
| `pxt_runs_clean/` | Cleaned GT dense pass — `pxt_GT-12x9-25-multi.clean.frames.json` (post-NMS / dedup cleanup of the dense base) plus `audit_zoom/` review crops and `run.log`. |
| `pxt_runs_yolov8m/` | yolov8m comparison runs (clips `0025`, `0028`, `0031`) — heavier-model cross-check of the yolov8n-4-class detector used everywhere else. |

## Deletion record (2026-06-07 restructure, Task 0)

The following untracked artifacts were **deleted** to reclaim ~46 GB. All are
regenerable — nothing here is authoritative GT or a committed report:

| Deleted | Size | How to regenerate |
|---|---|---|
| `pxt_runs/.cache/*.bin` decode caches | 45.2 GB | Regenerated automatically on the next `tiling_benchmark/run_pxt_bench.py` run (the bench re-decodes + re-caches on demand). |
| `pxt_runs/scaled_sources/` | 221 MB | Regenerate with `ffmpeg` (center-crop + lanczos scale to 4K per FOV) from the source clip `~/Videos/Drone/Training/DJI_20260430103421_0010_D_rotated.MP4`. |
| `pxt_runs/zoom_probe*.png`, `zoom_probe*.csv`, `zoom_compare*.png` | — | Canonical copies are tracked in `tiling_benchmark/PERF_REPORT.md` + `tiling_benchmark/perf_assets/` (`zoom_probe.png`, `zoom_probe_1080p.png`, `zoom_compare_720p.png`, `zoom_compare_1080p.png`). |

## Exported insights (read these, not the raw frames dumps)

- `tiling_benchmark/PERF_REPORT.md` — full static-grid resolution/zoom performance writeup (+ `tiling_benchmark/perf_assets/` figures).
- `../BASELINE.md` (`tiling_lab/runs/baseline_0025/BASELINE.md`) — dynamic-scheduler vs static-grid baseline on clip 0025.
- `../PHASE_A.md` — Phase A scheduler evaluation.
- `../REID_ABLATION.md` — ReID-policy ablation results.
- `../MOT_BASELINE.md` — MOT-metric baseline.
