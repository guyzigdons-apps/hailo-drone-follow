# Dynamic Tiling — Design Spec

**Date:** 2026-05-27
**Status:** Draft for review
**Builds on:** `tiling_benchmark/` (tools, GT, and the findings in `tiling_benchmark/PERF_REPORT.md`)

## 1. Problem & Goal

The static tiling benchmark sweeps fixed grids (e.g. 3×3, 8×6) that run **every frame**.
A dense grid is expensive: realtime budget is **300 tile-inferences/sec**, so an `N`-tile
grid at 30 fps has `rt_factor = 300 / (N×30) = 10/N` — a 3×3 grid (9 tiles, +1 rescue =
10) sits right at `rt_factor = 1.0`, and anything denser is not realtime.

Most of that budget is wasted: in a drone-follow scenario we usually care about **one
tracked target**, and the dense grid re-scans the whole frame every frame just to keep
finding it. The goal is to **reallocate the same inference budget** — spend it on and
around the tracked target — to get **higher accuracy on that target at equal cost**.

**Goal (one sentence):** A dynamic tile scheduler that, given a fixed
tile-inference/sec budget, tracks a single locked target and concentrates inference on
it to beat the static grids' per-target recall and localisation at equal budget.

## 2. Scope (v1)

- **Single-target lock only.** One tracked target receives the budget; the discovery
  grid only surfaces candidates. Multi-target is explicitly **v2**.
- **Reuse the pipeline's ByteTracker.** Do not write a new tracker. Use
  `drone_follow.pipeline_adapter.tracker_factory.create_tracker("byte", ...)` so the
  harness behaves like production and the scheduler can later drop into the real pipeline
  with minimal glue.
- **Offline harness** over recorded video — but with **real re-inference** on every
  scheduled crop (not a pure simulation). The accuracy gain comes from actually running
  the model on a tighter crop, so the harness must really call the chip.
- **Budget:** total = **300 tile-inferences/sec**, redistributable across frames.
  Comparison working point: `rt_factor = 1.0`, i.e. equal average inf/s to the static
  baselines.
- **Levers in v1:** decimated discovery grid (1), track-guided ROI tile (2), adaptive
  zoom capped at x2 (3), motion-predicted placement + search/recovery (4+5).
- **Deferred to v2:** multiscale fusion (6), uncertainty-driven allocation beyond a
  simple rule (7), multi-target.

### Non-goals
- No live/online integration into the drone-follow app (offline harness only).
- No new model or second network — single YOLOv8n VGA HEF, as in the benchmark.
- No GStreamer pipeline changes — inference is driven directly via `HefHandle`.

## 3. Findings we build on (from PERF_REPORT.md)

- **x2 is the zoom sweet spot.** x3–x5 do not improve recall and often hurt; vehicles
  are flat at 100% from x2. → adaptive zoom is **capped at x2**.
- **Detection floors:** person needs ≥ ~12 src-px (rescuable by zoom), vehicle ≥ ~8
  src-px; person needs ~28–32 model-px height at scale ≈ 1.0 for ≥80% recall.
  → the ROI tile is sized so the target lands in the high-recall model-px band.
- **Fragment-vs-whole NMS flaw:** a whole object split across tiles is beaten by its own
  downscaled whole-frame detection (`IoU = area_small/area_big`). A crop centred on the
  *whole* target avoids this by construction. → motion-predicted placement keeps the
  target inside one tile, not straddling a boundary.
- **Centre-rescue value:** the `center_vga_3x` rescue tile fixed large-object fragment
  collapse (8×6 vehicle 256+: 30.9% → 96.5%) at +1 inference/frame. → the dynamic ROI
  tile is the moving equivalent of that rescue tile.

## 4. Architecture

```
recorded video ─► frame f ─► TileScheduler.decide(lock_state, f, budget_meter)
                               │  returns list[CropRect] in source px
                               ▼
                        for crop in crops:
                            dets += inference.infer(crop)   ← HefHandle (real chip)
                               │
                               ▼
                    aggregator.nms(dets)  → frame detections in source px
                               │  → np.ndarray[x1,y1,x2,y2,score] (normalized)
                               ▼
        ByteTrackerAdapter.update(dets)  ← create_tracker("byte", ...)
                               │  Sequence[TrackedObject] (Kalman-filtered tlwh)
                               ▼
              TargetLock.observe(tracks)  → locked track_id + status + bbox
                               ├──────────► fed back into next frame's scheduler
                               ▼
                    score.record(lock, gt[f])       ← GT frames.json
                               ▼
        results table + plot + dynamic *.frames.json (overlay_viewer-compatible)
```

The harness is offline (deterministic replay of a recorded clip) but every scheduled crop
is a **real** on-chip inference, so per-target recall/IoU reflect the true zoom benefit.

## 5. Modules & Interfaces

New top-level directory `dynamic_tiling/`, separate from `tiling_benchmark/` but importing
its inference path and emitting viewer-compatible output.

### 5.1 `dynamic_tiling/types.py`
Shared dataclasses:
- `CropRect(x: int, y: int, w: int, h: int, scale: float, mode: str)` — source-pixel rect
  fed to the cropper; `scale = 640 / w` (>1 = digital zoom), `mode ∈ {"s","m"}`.
- `Detection(cls: int, conf: float, x: float, y: float, w: float, h: float)` — bbox in
  **source px** after mapping back from a crop.
- `LockState(track_id: int | None, bbox_norm: tuple, status: str, frames_since_seen: int,
  last_velocity: tuple)` — `status ∈ {"TRACKING","SEARCHING","LOST"}`; derived from the
  ByteTracker output, **not** a separate filter.

### 5.2 `dynamic_tiling/target_lock.py`
`TargetLock` — a thin layer over the pipeline's ByteTracker; **no new tracker is written**.
- Constructs the tracker via
  `drone_follow.pipeline_adapter.tracker_factory.create_tracker("byte", track_thresh=0.4,
  track_buffer=90, match_thresh=0.5, frame_rate=30)`.
- `lock_from_gt(gt_target, tracks)` — pick the `TrackedObject` whose `filtered_tlwh`
  best-IoUs the GT target on the lock frame; remember its `track_id`.
- `observe(tracks: Sequence[TrackedObject]) -> LockState` — find the locked `track_id` in
  this frame's activated tracks. If present → status TRACKING, bbox = `filtered_tlwh`,
  reset `frames_since_seen`, update `last_velocity` from the bbox-centre delta. If absent
  → increment `frames_since_seen`, status SEARCHING (within `track_buffer`) then LOST
  (past it); the placement bbox is the last-known bbox extrapolated by `last_velocity`.
- Rationale: ByteTracker's `update()` returns only activated tracks and already keeps lost
  tracks internally for `track_buffer` frames for re-association, so the lock layer only
  needs to follow a `track_id` and derive status — no second Kalman filter.

### 5.3 `dynamic_tiling/budget.py`
`BudgetMeter` — sliding-window tile-inference accounting.
- `__init__(budget_inf_per_s: float, fps: float, window_s: float = 1.0)`.
- `charge(n_tiles: int, frame_idx: int)` — record spend for a frame.
- `available(frame_idx: int) -> float` — how many tiles may be spent now to keep the
  windowed average ≤ budget. Allows short bursts (recovery) as long as the running
  average holds.

### 5.4 `dynamic_tiling/scheduler.py`
`TileScheduler.decide(lock: LockState, frame_idx: int, meter: BudgetMeter) -> list[CropRect]`.
Implements the v1 levers:
- **(1) Decimated discovery grid:** emit the full discovery grid only when
  `frame_idx % discovery_period == 0` (e.g. period = fps/2 → 2 fps); otherwise skip it.
- **(2) Track ROI tile:** when TRACKING/SEARCHING, emit one crop centred on the locked
  target's bbox (`LockState.bbox_norm`).
- **(3) Adaptive zoom (≤x2):** size the ROI so the target's height maps into the
  high-recall model-px band; clamp the resulting `scale` to ≤ 2.0 and the crop to frame
  bounds.
- **(4) Motion-predicted placement:** while TRACKING use the ByteTracker Kalman-filtered
  bbox (`filtered_tlwh`); during a gap centre on the last-known bbox extrapolated by
  `LockState.last_velocity`, so a moving target stays inside the tile.
- **(5) Search/recovery:** when SEARCHING/LOST, emit a coarse local grid around
  last-known + predicted position for up to `recovery_frames`, sized from `cov`; collapse
  back to the single ROI once reacquired. Recovery spend is bounded by `meter.available`.

### 5.5 `dynamic_tiling/inference.py`
- `InferenceBackend` protocol: `infer(crop_img_rgb) -> list[Detection]` (crop-local).
- `HefBackend` — wraps `tiling_benchmark/probe_phantom_hef.py::HefHandle`
  (`HefHandle.open(...)`, `.infer(...)`, `decode_nms_output`). Owns one VDevice.
- `ReplayBackend` — returns pre-recorded detections for a crop key; lets scheduler/tracker
  tests run without the chip and makes integration tests deterministic.

### 5.6 `dynamic_tiling/aggregator.py`
- `map_to_source(crop_dets, crop: CropRect) -> list[Detection]` — crop-local → source px.
- `nms(dets, iou_thr) -> list[Detection]` — cross-tile dedup of overlapping detections.

### 5.7 `dynamic_tiling/replay.py`
`run(video, gt, scheduler, tracker, backend, budget) -> RunResult`:
- Initialise the lock from GT on the first frame the target is present
  (`TargetLock.lock_from_gt`).
- Per frame: `decide → infer crops → aggregate → tracker.update → lock.observe → score →
  budget.charge`.
- Emit a `*.frames.json` in the same schema `overlay_viewer` already reads, so dynamic
  runs are visualizable alongside static ones.

### 5.8 `dynamic_tiling/score.py`
- Per-frame **target recall** (matched to GT target at IoU ≥ 0.5) and **mean IoU**.
- `compare(dynamic_result, static_frames_json) -> table` at equal average inf/s.

### 5.9 `dynamic_tiling/run_dynamic.py`
CLI driver: `--video --gt --budget 300 --fps 30 --discovery-fps 2 --zoom-cap 2.0`
plus lever on/off toggles; writes the results table, a comparison plot, and the dynamic
frames.json.

## 6. Data flow / budget accounting

`rt_factor = budget_inf_per_s / (avg_tiles_per_frame × fps)`. v1 holds
`budget_inf_per_s = 300`, `fps = 30`, and reports the achieved `avg_tiles_per_frame`
plus the equal-budget static baseline it is compared against (the 3×3+vga3x and 8×6 runs
from the benchmark). Discovery-grid frames are expensive but rare (2 fps); ROI frames are
cheap (1–2 tiles), so the average stays near the budget while per-target accuracy rises.

## 7. Error handling

- **Target absent / GT gap:** the lock enters SEARCHING then LOST (driven by ByteTracker
  `track_buffer`); scheduler falls back to discovery grid only. No crash on empty
  detections (feed an empty `(0,5)` array to the tracker).
- **Crop clamped to frame bounds:** if the predicted ROI exits the frame, clamp and reduce
  `scale` so the crop stays valid; never request a crop with zero/negative size.
- **VDevice single-owner:** only one `HefBackend` holds the chip at a time (documented
  HAILO_OUT_OF_PHYSICAL_DEVICES constraint); the harness runs inference sequentially.
- **Budget overrun during recovery:** `BudgetMeter.available` throttles the next frames'
  discovery grid so the windowed average is restored.

## 8. Testing strategy (TDD)

- `target_lock.py` — feed the real ByteTracker (deterministic, no chip) synthetic
  detection arrays: constant-velocity track holds one `track_id`; occlusion gap (N missed
  frames) → SEARCHING → reacquire keeps the lock; loss past `track_buffer` → LOST.
- `budget.py` — burst-then-throttle: a recovery spike followed by throttled frames keeps
  the windowed average ≤ budget.
- `aggregator.py` — crop→source mapping correctness; duplicate detections across
  overlapping crops collapse to one.
- `scheduler.py` — with a fake `LockState`: discovery grid appears only on the cadence;
  ROI is centred on the predicted bbox; zoom clamps at x2; recovery grid expands on loss
  and collapses on reacquire. Uses `ReplayBackend`.
- `replay.py` / `score.py` — integration smoke on a short clip via `ReplayBackend`
  (deterministic), then a real-chip run via `HefBackend` for metric sanity.

## 9. Deliverable / success criteria

A `run_dynamic.py` invocation on a benchmark world that produces:
- A table: dynamic vs static (3×3+vga3x, 8×6) — average inf/s, target recall, mean IoU.
- A plot of per-frame target IoU over the clip for dynamic vs static.
- A dynamic `*.frames.json` viewable in `overlay_viewer`.

**Success = at equal average inf/s, the dynamic scheduler matches or beats the static
grids on tracked-target recall and mean IoU**, demonstrating the budget-reallocation thesis.

## 10. Open questions (resolve during planning, not blocking)

- Which benchmark world(s) to use as the v1 validation clip (likely a single-person
  approach/cross world from `sim/worlds/`).
- Exact discovery-grid layout reused for the decimated pass (likely the 3×2 medium grid
  from the GT recipe, not the full 12×9).
