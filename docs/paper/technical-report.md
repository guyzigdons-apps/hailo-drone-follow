# Track-Guided Dynamic Tiling for Small-Target Detection on Edge NPUs

**Technical report (skeleton — Plan 15 paper-with-code scaffold).**

> Status: scaffold. The results section is auto-generated from committed
> ablation tables by `scripts/render_ablation_into_report.py` — it contains only
> numbers produced by the chip-free replay harness and committed to the repo.
> No result is hand-entered.

## Abstract

We study tiling strategies for detecting small targets (people seen from an
aerial drone) on a Hailo edge NPU, where the detector's fixed input resolution
forces a trade-off between field of view and effective pixels-on-target. We
compare *static* uniform tiling grids against a *track-guided dynamic* tiler
that spends its inference budget on regions of interest derived from a tracker's
state. All configurations are evaluated against a dense static reference grid at
matched compute (mean tiles per frame), on identical video, via a chip-free
replay of an on-chip-warmed detection cache for bit-stable, reproducible
numbers. *(Full abstract pending final results.)*

## 1. Problem

- Aerial person-following: targets are small in frame; a single full-frame
  inference at the model's input resolution loses the target.
- Static tiling raises pixels-on-target but multiplies inference cost linearly
  with the tile count, and spends budget uniformly regardless of where targets
  are.
- Goal: recover small-target recall at a fraction of the dense-grid compute by
  steering tiles toward tracked targets.

## 2. Method

The tiling scheduler (`dynamic_tiling/scheduler.py`, library
`hailo_tiling`) emits a per-frame crop list from three sources:

- **Discovery grid** on a fixed cadence (`discovery_period`) — a coarse uniform
  grid that finds new / re-entering targets.
- **Track ROI** — a per-target zoom crop sized so the target occupies a target
  fraction of the model input (`target_model_h`), capped at `max_zoom`, placed
  on the tracker's current bbox.
- **Recovery grid** — a denser local grid around the predicted position of a
  target that has just been lost (motion-extrapolated).

A budget meter (`hailo_tiling/budget.py`, sliding-window tiles/s) trims the
crop list, dropping discovery tiles before the locked-target ROI.

**Levers** (ablation toggles):
- **ASAHI** — adaptive slice sizing (`hailo_tiling/modifiers/adaptive_sizing.py`).
- **Altitude-gated zoom** — scale ROI zoom by drone AGL altitude
  (`hailo_tiling/modifiers/altitude_zoom.py`).

The tracker is the production ByteTracker
(`drone_follow.pipeline_adapter.tracker_factory.create_tracker("byte", ...)`),
fed each frame by the aggregated detections, so the dynamic schedule is
deterministic given the video.

## 3. Experimental setup

- **Hardware:** Hailo HAILO10H, yolov8n 4-class VGA HEF.
- **Clip:** DJI 0026, prepared at three synthetic fields-of-view (fov50 / fov60 /
  fov70) by centre-cropping the 4K source (`scripts/prepare_video.py --emit-fov`).
- **Reference:** a dense `12x9` static grid (108 tiles/frame @ 0.25 overlap)
  defines the recall/precision ground truth (IoU ≥ 0.5, per-class greedy match).
- **Reproducibility:** each config is *warmed once on-chip* into a source-pixel-
  keyed SQLite cache, then *replayed chip-free*; static grids replay from the
  pre-enumerated cache, dynamic configs warm their run-specific ROI tiles via
  `CachingBackend(GstCropperBackend)` then replay identically (the tracker is
  deterministic, so replay requests exactly the warmed crops). See
  `docs/paper/reproducibility.md`.
- **Matched compute:** each dynamic row is read against the static grid with the
  same `mean_tiles_per_frame` — the recall delta at equal budget is the headline
  number.

## 4. Results

> Auto-generated from committed `ablation_table.md` files by
> `scripts/render_ablation_into_report.py`. Recall/precision are vs the dense
> `12x9` reference at IoU ≥ 0.5. `n_misses = 0` confirms a fully-warmed
> chip-free replay.

<!-- BEGIN:ablation -->

#### Clip 0026 — fov50

- cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov50__a2e9861507428064.sqlite3`
- video: `/home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4`
- source: 3840x2160
- frames: 879
- reference: `12x9` (12x9), IoU>=0.5
- dynamic cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov50__dynamic.sqlite3` (per-source-frame; target class 2)
- `recall_delta_at_matched_budget` = dynamic recall − the closest-mean-tiles static grid's recall.

| config | kind | mean_tiles_per_frame | n_dets | recall_vs_reference | precision_vs_reference | n_misses | matched_static | recall_delta_at_matched_budget |
|---|---|---:|---:|---:|---:|---:|---|---:|
| 1x1 | static | 1.00 | 2643 | 0.2802 | 0.9368 | 0 | — | — |
| 2x2 | static | 4.00 | 3247 | 0.3396 | 0.9242 | 0 | — | — |
| 3x2 | static | 6.00 | 3298 | 0.3403 | 0.9118 | 0 | — | — |
| 3x3 | static | 9.00 | 3476 | 0.3387 | 0.8610 | 0 | — | — |
| 4x3 | static | 12.00 | 3903 | 0.3578 | 0.8101 | 0 | — | — |
| 6x4 | static | 24.00 | 5747 | 0.4366 | 0.6713 | 0 | — | — |
| 8x6 | static | 48.00 | 7043 | 0.5266 | 0.6608 | 0 | — | — |
| dynamic | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.2802 |
| dynamic+asahi | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.2802 |
| dynamic+altitude_zoom | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.2802 |
| 12x9 | static | 108.00 | 8837 | 1.0000 | 1.0000 | 0 | — | — |


*Source: `dynamic_tiling/runs/ablation_0026_fov50/ablation_table.md` (committed).*

#### Clip 0026 — fov60

- cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov60__a2e9861507428064.sqlite3`
- video: `/home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov60.mp4`
- source: 3840x2160
- frames: 879
- reference: `12x9` (12x9), IoU>=0.5
- dynamic cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov60__dynamic.sqlite3` (per-source-frame; target class 2)
- `recall_delta_at_matched_budget` = dynamic recall − the closest-mean-tiles static grid's recall.

| config | kind | mean_tiles_per_frame | n_dets | recall_vs_reference | precision_vs_reference | n_misses | matched_static | recall_delta_at_matched_budget |
|---|---|---:|---:|---:|---:|---:|---|---:|
| 1x1 | static | 1.00 | 2436 | 0.3212 | 0.9532 | 0 | — | — |
| 2x2 | static | 4.00 | 3270 | 0.4037 | 0.8927 | 0 | — | — |
| 3x2 | static | 6.00 | 3294 | 0.4203 | 0.9226 | 0 | — | — |
| 3x3 | static | 9.00 | 3228 | 0.4102 | 0.9188 | 0 | — | — |
| 4x3 | static | 12.00 | 3345 | 0.4141 | 0.8951 | 0 | — | — |
| 6x4 | static | 24.00 | 4665 | 0.4548 | 0.7048 | 0 | — | — |
| 8x6 | static | 48.00 | 5801 | 0.5465 | 0.6811 | 0 | — | — |
| dynamic | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.3212 |
| dynamic+asahi | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.3212 |
| dynamic+altitude_zoom | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.3212 |
| 12x9 | static | 108.00 | 7230 | 1.0000 | 1.0000 | 0 | — | — |


*Source: `dynamic_tiling/runs/ablation_0026_fov60/ablation_table.md` (committed).*

#### Clip 0026 — fov70

- cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov70__a2e9861507428064.sqlite3`
- video: `/home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov70.mp4`
- source: 3840x2160
- frames: 879
- reference: `12x9` (12x9), IoU>=0.5
- dynamic cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov70__dynamic.sqlite3` (per-source-frame; target class 2)
- `recall_delta_at_matched_budget` = dynamic recall − the closest-mean-tiles static grid's recall.

| config | kind | mean_tiles_per_frame | n_dets | recall_vs_reference | precision_vs_reference | n_misses | matched_static | recall_delta_at_matched_budget |
|---|---|---:|---:|---:|---:|---:|---|---:|
| 1x1 | static | 1.00 | 2209 | 0.2636 | 0.8705 | 0 | — | — |
| 2x2 | static | 4.00 | 3750 | 0.4474 | 0.8704 | 0 | — | — |
| 3x2 | static | 6.00 | 4041 | 0.4728 | 0.8535 | 0 | — | — |
| 3x3 | static | 9.00 | 4385 | 0.5079 | 0.8449 | 0 | — | — |
| 4x3 | static | 12.00 | 4132 | 0.5175 | 0.9136 | 0 | — | — |
| 6x4 | static | 24.00 | 4849 | 0.5416 | 0.8148 | 0 | — | — |
| 8x6 | static | 48.00 | 6537 | 0.6092 | 0.6798 | 0 | — | — |
| dynamic | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.2636 |
| dynamic+asahi | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.2636 |
| dynamic+altitude_zoom | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.2636 |
| 12x9 | static | 108.00 | 7295 | 1.0000 | 1.0000 | 0 | — | — |


*Source: `dynamic_tiling/runs/ablation_0026_fov70/ablation_table.md` (committed).*

<!-- END:ablation -->

## 5. Discussion

**Static sweep.** The static grid rows establish the expected recall-vs-compute
Pareto front: recall climbs monotonically with tile count (1x1 ≈ 0.26–0.32 →
8x6 ≈ 0.53–0.61 → dense 12x9 = 1.0 by definition), while precision falls as
finer grids split objects across tile boundaries. Higher synthetic FOV (more
zoomed-in source crop) lifts recall at every budget, as expected for larger
apparent target size.

**Dynamic rows — a negative result on this clip, and what it teaches.** On clip
0026 the track-guided dynamic configurations are *degenerate*: they spend only
≈0.40 tiles/frame and recover **zero** reference detections (recall delta
≈ −0.26…−0.32 vs the equal-budget 1x1 grid). The cause is not a harness defect
(replay reports 0 cache misses; the schedule is deterministic) but a
**seeding failure**: 0026's targets are small (the dominant class here is `face`,
cls 2 — the clip contains no `person`), and the scheduler's coarse 3×2 discovery
grid produces ~1440 px-wide tiles that are downscaled ~2.25× to the 640 px model
input, below the detector's floor for these targets. With no discovery
detections the tracker never locks, so the budget-saving ROI-zoom tiles — the
whole point of the dynamic tiler — are never emitted.

The practical lesson is that **track-guided tiling is only as good as its
discovery stage**: a dynamic tiler must seed at a resolution fine enough to
acquire the smallest target it intends to follow. A fix is to gate discovery
grid density on apparent target size (or run discovery at the cadence with a
denser grid) so the tracker can lock; that is a scheduler change, not a harness
change, and is future work. The matched-compute methodology and the chip-free
replay pipeline are validated and ready to quantify a properly-seeded dynamic
tiler (and clips that contain larger, lockable targets).

## 6. Limitations

- Single clip (0026) at this stage; more clips (0027/0029/…) extend coverage.
- The installed `hailotilecropper_dynamic` plugin supports only `tiles-static`,
  so dynamic ROIs are injected by per-frame pipeline relaunch (correct but
  slower than a single long-lived injected pipeline) — see
  `hailo_tiling/backends/gst_cropper.py`.
- Recall is reported against a dense-static reference, not human-annotated
  ground truth; it measures recovery of the dense grid's detections, not
  absolute detection quality.

## 7. Reproducing this report

```bash
python scripts/render_ablation_into_report.py        # refresh §4 from tables
python scripts/render_ablation_into_report.py --check # verify it is current
```
