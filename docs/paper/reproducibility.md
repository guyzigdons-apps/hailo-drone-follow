# Reproducibility — chip-free ablation replay

Every number in `technical-report.md` §4 is produced by replaying a warmed,
source-pixel-keyed detection cache — **no Hailo chip is needed to reproduce the
tables** once the caches exist. Warming the caches needs the chip once.

## 0. Environment

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
source setup_env.sh   # activates hailo-apps/venv_hailo_apps + paths
```

The bench/warmer use the project venv python directly:
`hailo-apps/venv_hailo_apps/bin/python`.

## 1. (Chip, once) Warm the caches

Static grid set per (clip, FOV) — one fresh subprocess per grid (the warmer
handles this internally since the multi-grid teardown-deadlock fix):

```bash
GST_PLUGIN_PATH=gst-hailo-cache/build/src HAILO_CHIP=1 \
  hailo-apps/venv_hailo_apps/bin/python scripts/warm_gst_cache.py \
    --video /path/DJI_..._0026_..._fov50.mp4 \
    --hef   /usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef \
    --out-cache .tile_cache/DJI_..._0026_..._fov50__<hefsha16>.sqlite3 \
    --grids "1x1:0.0;2x2:0.25;3x2:0.25;3x3:0.25;4x3:0.25;6x4:0.25;8x6:0.25;12x9:0.25" \
    --source-width 3840 --source-height 2160
```

Dynamic configs warm their run-specific ROI tiles via the live
`CachingBackend(GstCropperBackend)` path (the bench dynamic runner drives the
scheduler + ByteTracker and the caching backend records each frame's tiles).
The tracker is deterministic, so a later chip-free replay requests exactly the
crops that were warmed.

Caches are gitignored (regenerable). `n_misses = 0` on replay is the gate that
proves a cache is fully warmed.

## 2. (No chip) Replay the ablation table

```bash
hailo-apps/venv_hailo_apps/bin/python -m hailo_tiling.cli.bench \
  --cache .tile_cache/DJI_..._0026_..._fov50__<hefsha16>.sqlite3 \
  --video /path/DJI_..._0026_..._fov50.mp4 \
  --configs default \
  --out-dir dynamic_tiling/runs/ablation_0026_fov50
# or the console script: hailo-tiling-bench --cache ... --video ... --out-dir ...
```

This writes one `<config>.frames.json` per row and an `ablation_table.md` with
mean tiles/frame, detection counts, and recall/precision vs the dense `12x9`
reference (IoU ≥ 0.5).

### Per-tile-buffer caches

The GStreamer `hailocachewriter` keys `frame_idx` per tile-buffer (monotonic),
not per source frame. The CLI auto-detects this and replays static rows via the
crop-occurrence-ordered path (the k-th occurrence of each crop = source frame
k). Dynamic rows, which need true source-frame indexing, are warmed/replayed via
the Python `CachingBackend`/`ReplayBackend` store (keyed by the real frame index)
in a separate cache.

> Known coverage caveat (Night-1 review finding): in the current 0026 fov50
> cache a few crops (incl. the 1x1 full-frame crop) have one fewer occurrence
> than the rest, so the crop-ordered replay truncates those configs to the
> common frame prefix. This shifts recall by ≈0.1% for affected rows and does
> not change the trend. See `docs/superpowers/reviews/2026-05-31-night1-review.md`.

## 3. Render into the report

```bash
hailo-apps/venv_hailo_apps/bin/python scripts/render_ablation_into_report.py
hailo-apps/venv_hailo_apps/bin/python scripts/render_ablation_into_report.py --check
```

The renderer splices the committed `ablation_table.md` files verbatim into
`technical-report.md` §4 between the `<!-- BEGIN:ablation -->` / `<!-- END:ablation -->`
markers — it never fabricates numbers.
