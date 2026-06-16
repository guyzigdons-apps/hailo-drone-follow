# Dynamic-Tiling Realtime Showcase — Design

**Date:** 2026-06-15
**Branch:** `tiling-benchmark`
**Status:** Approved (design choices confirmed 2026-06-15)

## Goal

Prove the dynamic-tiling pipeline (video ingest → tile cropping → tracking →
inference) sustains **4K @ 60 fps** on the laptop, and produce a visualizer-style
showcase of single-object tracking (one **vehicle**, SOT) over a DJI drone clip.
Add **detection persistence** and **striped dense tiling** so the dense
(~2 fps) detection pass no longer spikes per-frame inference load.

The deliverable is **not** a baked-overlay video. The realtime pipeline does no
drawing and saves no video — drawing happens later in postprocess. The showcase
visual is rendered offline by the existing visualizer from captured per-frame
data.

## Context / target hardware

- Laptop has **HAILO10H**; default HEF
  `/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef`
  (classes: `unlabeled, person, vehicle, face, license_plate`).
- Source clips: `/home/giladn/Videos/Drone/Training/Car/DJI_*.MP4` — HEVC,
  3840×2160, 59.94 fps, **no rotation metadata**. Build/validate clip:
  `DJI_20260614161040_0013_D.MP4` (2200 frames / 36 s).
- On the **RPi target** the same controller runs from a **live camera** (no
  decode). The laptop test *includes* HEVC decode, so it is the conservative
  case — if the laptop holds 60 fps with decode, the RPi (decode-free) is
  comfortably within budget for the crop+track+infer portion.

## Non-goals (YAGNI)

- No FOV cropping/scaling on ingest (`--emit-fov` path is **not** used).
- No resizing of source clips.
- No overlay drawing inside the realtime pipeline; no saved overlay video.
- No multi-target scheduling — single-object tracking only (TileScheduler v1).
- No new visualizer features; reuse `overlay_viewer.py` as-is.

## Architecture

Three components, each independently testable:

### 1. Video ingest/alignment (existing flow, reused)

Run `python -m tiling_lab.video.prepare_video <clip> --verify` (no `--emit-fov`).
It detects rotation (`tags.rotate` + Display-Matrix side-data), and:
- rotation present → re-encode baking rotation into pixels, clear metadata;
- rotation absent (our clips) → passes through, used directly.

No resize. This is the canonical ingest path; no code changes required, only a
documented invocation + a smoke test asserting the 0013 clip ingests landscape
with rotation=0.

### 2. Striped dense scheduling + persistence (new logic)

Two new, focused units. Both live under `hailo_tiling/dynamic/` next to the
scheduler so the lab and any future prod consumer can use them.

#### 2a. `StripedDenseScheduler` (`hailo_tiling/dynamic/striped.py`)

Wraps a `TileScheduler` (v1). Replaces the single-frame discovery burst with a
round-robin stripe emission:

- The dense grid is `dense_grid = (DX, DY)` (default **8×6 = 48 tiles**),
  covering the whole frame, multi-scale (`"m"`), built via the existing
  `TileScheduler._grid(...)`.
- The 48 dense tiles are partitioned into **K stripes** (default
  `stripes = cycle_frames`, see below) of roughly equal size. Stripe `i`
  contains tiles `[i::K]` of the flattened grid (interleaved, so each stripe
  samples the whole frame, not one band).
- Each frame emits **exactly one stripe** of dense tiles, chosen by
  `frame_idx % K`, plus whatever the wrapped v1 scheduler returns for the
  **ROI/recovery** of the locked target (the SOT tile fires every frame).
- A full dense refresh therefore completes every `K` frames. With
  `cadence_fps = 2.0` and `fps = 60`, `K = round(fps / cadence_fps) = 30`
  frames (~0.5 s). 48 tiles / 30 frames ⇒ ~1.6 dense tiles/frame, **constant**.
- Budget is still applied (BudgetMeter), ROI-first, so a tight budget drops
  dense-stripe tiles before the SOT tile.

Key property: per-frame tile count is flat (≈ ROI + ceil(48/K)), eliminating the
periodic 6-tile (or worse) discovery spike that the current cadence design
produces.

Each emitted dense tile carries its **stripe id** and **grid-cell index** so the
persistence cache (2b) can attribute returned detections to a region.

#### 2b. `DetectionPersistence` (`hailo_tiling/dynamic/persistence.py`)

Holds the most-recent detections per dense grid cell so the visualizer draws
stable boxes between refreshes:

- `update_dense(stripe_id, cells, dets)` — replace cached detections for the
  cells covered by this frame's stripe with the dets that fell inside them.
- `published()` — return the union of all currently-cached dense detections.
- Cells refresh as their stripe revisits (every `K` frames). A detection
  therefore persists up to one full cycle (~0.5 s) — the "2 fps update"
  behavior, without flicker.
- The **SOT target** detection is *not* persisted here; it is emitted live every
  frame from the ROI tile (smooth target box) and merged with the published
  dense set when writing the frame record.

### 3. Showcase runner (new): `tiling_lab/live/run_showcase.py`

A sibling of `run_live.py`, but:

- **No overlay, no encode, no mkv.** Pipeline tail is a `fakesink` after the
  aggregator; a buffer probe on the aggregator src does all the work.
- Pipeline: `SOURCE_PIPELINE → queue → hailotilecropper_dynamic →
  (bypass → agg.sink_0) / (infer → agg.sink_1) → agg → queue → fakesink
  sync=false`.
- Per frame, the probe: reads detections, filters to `--target-class`
  (default `vehicle`), steps the `DynamicTilingController` (now backed by
  `StripedDenseScheduler` + `DetectionPersistence`), pushes the next
  `tiles-static` string onto the cropper, and appends a frame record.
- **Outputs** under `--out`:
  - `frames.json` — visualizer schema:
    `{"label": <run-label>, "frames": [{"frame", "detections":[{label,
    confidence, bbox:[x,y,w,h]}], "tiles":[...]}]}`. `detections` = live SOT
    det + persisted dense union. `tiles` = the tiles actually run this frame.
  - `metrics.json` — `{frames, wall_s, achieved_fps, mean_tiles_per_frame,
    max_tiles_per_frame, p95_tiles_per_frame, mean_infer_s, source_fps,
    sustains_60fps: bool}`.
- **FPS measurement:** pipeline runs unsynced (`sync=false`) so it processes as
  fast as the hardware allows. `achieved_fps = frames / wall_s` measured from
  first-buffer to EOS. `sustains_60fps = achieved_fps >= 60.0`. Per-frame tile
  count stats prove the spike is gone (`max ≈ mean`).

### Controller wiring

`DynamicTilingController` (`tiling_lab/live/controller.py`) gains an opt-in path:

- New kwargs: `dense_grid=(8,6)`, `cadence_fps=2.0`, `striped=True`,
  `persist=True`. When `striped`, it builds a `StripedDenseScheduler` instead of
  bare `TileScheduler`; when `persist`, it owns a `DetectionPersistence`.
- `update(persons_or_dets)` returns `(tiles_static_str, frame_record_dets)` so
  the runner can write both the cropper string and the persisted+live detection
  union. (Back-compat: existing `run_live.py` callers keep the string-only
  behavior via a thin property or a `update_tiles_only` shim — see plan.)

## Data flow (per frame)

```
decode → cropper(tiles_static[f-1]) → [bypass, infer per tile] → aggregator
   → probe(f):
        dets = aggregator detections (normalized)
        target_dets = [d for d in dets if d.label == target_class]
        lock.step(target_dets)                      # SOT auto-lock largest
        stripe_tiles = striped_sched.decide(lock, f, meter)   # ROI + 1 stripe
        persistence.update_dense(stripe_id(f), cells, dets)   # refresh region
        record.detections = live_SOT_det + persistence.published()
        cropper.tiles_static = serialize(stripe_tiles)        # for frame f+1
        metrics.note(len(stripe_tiles), infer_time)
```

## Testing strategy

- **Unit (pure, no GStreamer/Hailo):**
  - `StripedDenseScheduler`: K stripes partition the full grid; union of all
    stripes over one cycle == full dense grid; per-frame tile count flat
    (max−min ≤ 1 across a cycle, excluding ROI); ROI tile present every
    TRACKING frame; budget trims stripe before ROI.
  - `DetectionPersistence`: a det in cell C persists for K frames then is
    replaced/cleared when stripe owning C revisits; `published()` returns union;
    SOT det excluded.
  - `DynamicTilingController` striped path: returns flat per-frame tile counts;
    `frame_record_dets` includes live target + persisted union.
- **Integration smoke (GStreamer, gated on device):**
  - `prepare_video` ingest of 0013 → landscape, rotation=0 (ffprobe + 1-frame
    gst decode, via existing `--verify`).
  - `run_showcase.py --frames 300` on 0013 produces `frames.json` +
    `metrics.json`; assert schema valid and `max_tiles_per_frame -
    mean_tiles_per_frame` small (spike gone).
- **Acceptance (manual, full clip):**
  - `run_showcase.py` on full 0013 → `achieved_fps >= 60`, report metrics.
  - `overlay_viewer.py` renders `frames.json` over the clip → visual check:
    SOT vehicle tracked smoothly, context boxes stable (no flicker), tiles shown
    visualizer-style.

## Files

- Reuse: `tiling_lab/video/prepare_video.py`, `tiling_lab/viewer/overlay_viewer.py`,
  `hailo_tiling/dynamic/scheduler.py`, `hailo_tiling/budget.py`,
  `tiling_lab/harness/target_lock.py`, `tiling_lab/live/tiles_format.py`.
- Create: `hailo_tiling/dynamic/striped.py`, `hailo_tiling/dynamic/persistence.py`,
  `tiling_lab/live/run_showcase.py`, plus tests under `tiling_lab/tests/` and
  `hailo_tiling/tests/`.
- Modify: `tiling_lab/live/controller.py` (opt-in striped+persist path).

## Open risks

- HEVC 4K60 software decode on the laptop could be the bottleneck rather than
  crop/track/infer. Mitigation: `metrics.json` reports achieved_fps; if decode-
  bound, note it explicitly (RPi runs decode-free from camera, so the
  crop+track+infer budget is what matters for the RPi target). Optionally
  measure a decode-only baseline pipeline if the full pipeline misses 60.
- Tile count per frame must stay within the cropper's practical limit; 8×6
  striped keeps it to ~2–3/frame, well within range.
```