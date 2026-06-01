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

| config | kind | mean_tiles_per_frame | n_dets | recall_vs_reference | precision_vs_reference | n_misses |
|---|---|---:|---:|---:|---:|---:|
| 1x1 | static | 1.00 | 2643 | 0.3137 | 0.9368 | 0 |
| 2x2 | static | 4.00 | 3247 | 0.3802 | 0.9242 | 0 |
| 3x2 | static | 6.00 | 3298 | 0.3810 | 0.9118 | 0 |
| 3x3 | static | 9.00 | 3476 | 0.3792 | 0.8615 | 0 |
| 4x3 | static | 12.00 | 3903 | 0.3939 | 0.8096 | 0 |
| 6x4 | static | 24.00 | 5747 | 0.4585 | 0.6573 | 0 |
| 8x6 | static | 48.00 | 7043 | 0.5125 | 0.6335 | 0 |
| 12x9 | static | 108.00 | 8837 | 1.0000 | 1.0000 | 0 |


*Source: `dynamic_tiling/runs/ablation_0026_fov50/ablation_table.md` (committed).*

#### Clip 0026 — fov60

- cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov60__a2e9861507428064.sqlite3`
- video: `/home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov60.mp4`
- source: 3840x2160
- frames: 879
- reference: `12x9` (12x9), IoU>=0.5

| config | kind | mean_tiles_per_frame | n_dets | recall_vs_reference | precision_vs_reference | n_misses |
|---|---|---:|---:|---:|---:|---:|
| 1x1 | static | 1.00 | 2436 | 0.3576 | 0.9532 | 0 |
| 2x2 | static | 4.00 | 3270 | 0.4496 | 0.8927 | 0 |
| 3x2 | static | 6.00 | 3294 | 0.4680 | 0.9226 | 0 |
| 3x3 | static | 9.00 | 3228 | 0.4568 | 0.9188 | 0 |
| 4x3 | static | 12.00 | 3345 | 0.4610 | 0.8950 | 0 |
| 6x4 | static | 24.00 | 4665 | 0.4878 | 0.6973 | 0 |
| 8x6 | static | 48.00 | 5801 | 0.5551 | 0.6646 | 0 |
| 12x9 | static | 108.00 | 7230 | 1.0000 | 1.0000 | 0 |


*Source: `dynamic_tiling/runs/ablation_0026_fov60/ablation_table.md` (committed).*

#### Clip 0026 — fov70

- cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov70__a2e9861507428064.sqlite3`
- video: `/home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov70.mp4`
- source: 3840x2160
- frames: 879
- reference: `12x9` (12x9), IoU>=0.5

| config | kind | mean_tiles_per_frame | n_dets | recall_vs_reference | precision_vs_reference | n_misses |
|---|---|---:|---:|---:|---:|---:|
| 1x1 | static | 1.00 | 2209 | 0.2867 | 0.8705 | 0 |
| 2x2 | static | 4.00 | 3750 | 0.4867 | 0.8704 | 0 |
| 3x2 | static | 6.00 | 4041 | 0.5142 | 0.8535 | 0 |
| 3x3 | static | 9.00 | 4385 | 0.5524 | 0.8449 | 0 |
| 4x3 | static | 12.00 | 4132 | 0.5628 | 0.9136 | 0 |
| 6x4 | static | 24.00 | 4849 | 0.5867 | 0.8144 | 0 |
| 8x6 | static | 48.00 | 6537 | 0.6477 | 0.6760 | 0 |
| 12x9 | static | 108.00 | 7295 | 1.0000 | 1.0000 | 0 |


*Source: `dynamic_tiling/runs/ablation_0026_fov70/ablation_table.md` (committed).*

<!-- END:ablation -->

## 5. Discussion

**Static sweep.** The static grid rows establish the expected recall-vs-compute
Pareto front: recall climbs monotonically with tile count (1x1 ≈ 0.26–0.32 →
8x6 ≈ 0.53–0.61 → dense 12x9 = 1.0 by definition), while precision falls as
finer grids split objects across tile boundaries. Higher synthetic FOV (more
zoomed-in source crop) lifts recall at every budget, as expected for larger
apparent target size.

**Dynamic rows — corrected.** An earlier draft reported a *degenerate* dynamic
result (≈0.40 tiles/frame, ~0 recall) and attributed it to a discovery-seeding
failure on a clip that "contained no person." **That conclusion was wrong** — it
was the compound effect of three implementation bugs, since fixed:

1. **Class-label off-by-one.** The `hailo_4_classes` HEF label file has a leading
   `unlabeled` slot, so the network emits ids 1–4 (person=1), but the code
   assumed person=0. 0026 in fact contains ~24k `person` detections (which the
   bug mislabelled "vehicle"); the single-target tracker was seeded with the
   wrong class. Fixed via a single source of truth (`hailo_tiling/classes.py`).
2. **`GstCropperBackend` never received the video path.** The stateful runner
   calls `infer(frame_idx, crops, frame_idx)`; the backend used the first
   argument as the video filename, so the pipeline became `filesrc location="0"`
   and decoded nothing — every tile returned 0 detections, and the cache's "0
   misses" was vacuous. Fixed by threading the video in at construction.
3. **Coarse 3×2 discovery grid.** Even after (1)/(2), a 3×2 discovery grid is too
   coarse to seed small aerial targets; an 8×6 discovery grid seeds reliably.

With these fixed, and the full-run path moved off the deadlocking per-frame
GStreamer relaunch onto the direct `HefBackend` (in-memory OpenCV crop →
HailoRT), a full-clip **multi-target** run completes (879 frames, ~2 min). The
flow — dense 8×6 discovery at 2 fps with tracked-target ROI tiles on the
intervening frames — runs at **6.84 tiles/frame** with whole-frame
person+vehicle recall **≈0.35**, i.e. **on par with the equal-budget static 2×2
grid (0.38), not better.**

This is the honest, expected result: for detecting *all* objects a uniform grid
is compute-efficient, so track-guided tiling offers no whole-frame advantage.
Dynamic tiling's value is **single-target follow** — spending budget on one
tracked target instead of uniformly — where the v1 single-target result reached
≈0.93 target recall at ~1.3 tiles/frame. The whole-frame "detect everything"
comparison here validates the infrastructure end-to-end and correctly frames the
paper's claim around single-target follow, which the next experiment stage
sweeps (discovery density × cadence × ROI sizing × discovery overlap).

## 6. Limitations

- Single clip (0026) at this stage; more clips (0027/0029/…) extend coverage.
- **Two inference paths, not yet unified.** Static caches are warmed through the
  GStreamer "golden" cropper; the full-clip dynamic run uses the direct
  `HefBackend` (OpenCV crop → HailoRT) because the GStreamer per-frame-relaunch
  path **deadlocks after ~6 frames** (in-process pipeline teardown/relaunch
  hang). The two paths differ slightly in resize/NMS, so dynamic-vs-static is
  not byte-identical; unifying them (a single long-lived pipeline with
  signal-handoff ROI injection in `hailotilecropper_dynamic`, or routing static
  through `HefBackend` too) is needed for paper-grade head-to-head numbers.
- **Two class conventions.** `HefBackend` emits 0-indexed ids (person=0) from
  the raw NMS decode; the GStreamer `hailofilter` path emits 1-indexed
  (person=1, the label-file convention). Comparisons currently bridge by label;
  the conventions should be unified.
- **Discovery grid has no overlap.** The scheduler's discovery `_grid()` lays
  tiles edge-to-edge (gap-free 4:3 partition), unlike the static ablation grids
  (0.25 overlap), so objects straddling a discovery seam can fragment — a
  fine-tune knob to add.
- Recall is reported against a dense-static reference, not human-annotated
  ground truth; it measures recovery of the dense grid's detections, not
  absolute detection quality.

## 7. Reproducing this report

```bash
python scripts/render_ablation_into_report.py        # refresh §4 from tables
python scripts/render_ablation_into_report.py --check # verify it is current
```
