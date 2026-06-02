# Tile-Scheduler Experiment Design

> Spec for the experiment sweep that finds the best-performing dynamic
> tile-scheduling algorithm for drone single-target **follow + recovery** on a
> fixed-throughput Hailo NPU, and tests whether a budget-aware "knapsack" tile
> selector beats the tuned simple scheduler.

**Date:** 2026-06-02
**Branch:** `tiling-benchmark`
**Status:** design — pending user review before plan decomposition.

---

## 1. Goal

Find the **simple best-performing** dynamic tile-scheduling algorithm for
**single-target follow** at a fixed inference budget — measured not just by
detection recall but by **tracking quality and recovery** — establish its
performance-vs-budget frontier against the static-grid baseline, then test a
**budget-aware knapsack tile selector** as a challenger to see if principled
selection beats the tuned heuristic.

The deliverable is a paper-with-code result: the tuned simple algorithm, the
knapsack comparison, and clean tracking/recovery-vs-budget curves on real drone
footage, averaged over **every object in each clip** (data augmentation).

## 2. Background & framing

A Hailo NPU has **fixed throughput**: at a target FPS you get a fixed number *N* of
tile inferences per frame. Tiling becomes a **budget-allocation** problem — how to
split *N* between *discovery* (find/keep targets), *track-ROI* (resolve the target at
high pixel density), and *recovery* (re-find after loss). Static slicing (SAHI/ASAHI)
has no budget and spends uniformly. The corrected Night-2 result showed that at equal
compute, dynamic ≈ static for whole-frame "detect everything" (a uniform grid is
near-optimal there) but dynamic **dominates for single-target follow**. This
experiment quantifies that win — emphasising the *tracking + recovery* behaviour where
budget reallocation matters most — and tries to push it further.

## 3. Scope

**In scope (this round):**
- Single-target follow + recovery (target = a **person**).
- **All objects as trials:** every clean GT object trajectory becomes one
  single-target follow trial; the others act as realistic distractors. Metrics are
  averaged across all trials. (Augmentation: one clip → K trials.)
- Person + vehicle as the only detected/tracked classes.
- Offline replay over recorded clips with **heavy-MOT pseudo-GT** — no telemetry.
- Phase 0 bench standardization + GT-track builder, Phase A simple-scheduler tuning,
  Phase B knapsack.

**Out of scope (deferred, must not be repainted-into-a-corner):**
- **Multi-target follow** (following >1 target at once) — design abstractions treat
  single-target as the K=1 case so multi-target is a later extension, not a rewrite.
- **AltitudeZoom modifier** — implemented + unit-tested, but the benchmark clips carry
  no aligned AGL telemetry. Excluded; stays in code as future-work.
- **ReID re-acquisition in the algorithm under test** — deferred. (ReID *may* be used
  offline to strengthen GT-track association; see §7 — that is GT generation, not the
  algorithm being measured.)
- **CMC (camera-motion compensation)** — future work.

## 4. Standing project rules

1. **Classes: person + vehicle only.** Face and license-plate are removed
   project-wide. GT prep strips them; every detection filter keeps only
   `{person, vehicle}`; they never enter GT, metrics, the sweep, or any report.
   (`hailo_tiling/classes.py`: `TRACKED_CLASSES = (PERSON, VEHICLE)`,
   `TARGET_CLASS = PERSON`.)
2. **One class convention.** Standardize on **person = 1, vehicle = 2** end to end.
   `HefBackend`'s 0-indexed decode is bridged with a `+1` offset so every artifact
   uses the same ids.
3. **One inference path.** All runs (static baseline + dynamic) go through
   **`HefBackend`** (OpenCV crop → HailoRT). The GStreamer cropper path deadlocks on
   per-frame relaunch and is not used for full-clip runs in this round.
4. **Selection is GT-bbox-seeded, ByteTracker-id-locked** (never GT-id). See §6.

## 5. Definitions, trials, and metrics

### 5.1 Budget
Mean **tiles per frame** (the per-frame inference count *N*) — the primary
independent variable. Always reported as a swept curve, never a single point.

### 5.2 Trials
For each clip × fov, and for **each clean GT object trajectory** *G* (§7), run one
single-target follow trial seeded on *G*. The headline numbers are **means over all
trials** per fov. A trial's frame range is the frames where *G* is present in GT.

### 5.3 Metric suite (all reported vs budget, averaged over trials, per fov)
Detection recall alone is insufficient — the contribution is *tracking + recovery*.

1. **Follow coverage (primary):** fraction of *G*-present frames where the followed
   bbox correctly localizes *G* (IoU ≥ 0.5 vs *G*'s GT bbox at that frame).
2. **Localization accuracy:** mean IoU over covered frames (and success-AUC over IoU
   thresholds 0.3–0.7 as a secondary robustness number).
3. **Drift / mis-lock rate:** fraction of *G*-present frames where the followed bbox
   instead covers a **different** GT object (IoU ≥ 0.5 with another trajectory) — i.e.
   the lock jumped to a distractor. Requires all GT trajectories at scoring time.
4. **Recovery:** a *loss* is a maximal run of frames where the followed bbox fails to
   cover *G* while *G* is present. Report:
   - **loss-event count** per trial,
   - **mean time-to-recover** (frames from loss onset until correct re-coverage),
   - **recovery success rate** (fraction of losses that re-cover the *correct* object,
     vs ending on a wrong object = mis-recovery, vs never recovering before *G* exits).

### 5.4 Clips
`DJI_20260528155239_0026` at emulated **fov50 / fov60 / fov70** (existing). With
all-objects trials this yields K×3 trials per clip; a second GT'd clip can be added if
trial count proves thin (flagged, not blocking).

## 6. Per-trial target selection (precise)

For a trial seeded on GT trajectory *G*:

1. **Seed by GT bbox, not GT id.** While the lock is unset, the runner passes *G*'s GT
   bbox **every frame**. `TargetLock.lock_from_gt` (`target_lock.py:70`) /
   `MultiTargetLock` step 3 (`:192`) IoU-match that bbox against the **live ByteTracker
   tracks** and pin the matching **ByteTracker** `track_id` (composite `(cls,
   track_id)` for multi) as the stable lock identity (IoU threshold 0.3). This
   naturally handles detection delay at low budget — seeding waits until the target is
   actually tracked.
2. **Follow by ByteTracker id thereafter,** with the existing silent IoU
   re-acquisition (`:99`) when ByteTracker drops/renumbers the track. The public
   `track_id` (stable identity) never changes.
3. **Score by bbox, not id.** Each frame, compare the followed bbox to *G*'s GT bbox by
   IoU (§5.3). GT-id ≠ ByteTracker-id is therefore a non-issue — ids are never compared.

**Forward-compat (multi-target later):** `MultiTargetLock` already keys targets by
`(cls, track_id)` and GT-seeds a `selected_key`. Single-target is the K=1 case;
scoring averages per-target so K>1 drops in without a rewrite.

## 7. Phase 0 — Bench standardization + GT-track builder (prereq)

Make the harness produce comparable numbers and good GT before any tuning counts.

### 7.1 Standardization
- Route static-baseline and dynamic runs through **`HefBackend`** only.
- Apply **person=1 / vehicle=2** everywhere (add the `+1` offset to `HefBackend`
  decode; update `run()`/`run_multi()` defaults and the replay label tuple).
- Confirm GT prep and all detection filters keep only `{person, vehicle}`.
- Lock the §5 metric suite + protocol into the scoring code.

### 7.2 Heavy-MOT GT-track builder (new)
We need **good tracking GT**, including through occlusions/crossings, because tracking
and recovery are now metrics. The builder:
- Runs a **heavy offline multi-object tracker over the dense 12×9 detections**:
  **BoT-SORT** — chosen because the drone camera moves, and BoT-SORT's built-in
  **camera-motion compensation (CMC)** directly addresses the dominant
  moving-camera association failure, while its native **ReID appearance** support lets
  us feed the existing strong ReID net to fix id-switches at crossings. Add
  **short-gap interpolation** to bridge occlusions. **Must be independent of the
  runtime ByteTracker** (the component under test — using it would be circular).
  BoT-SORT satisfies this (different tracker, with CMC + appearance).
- **Fallback** if BoT-SORT integration proves too costly: **OC-SORT + ReID-assisted
  association + gap interpolation** (motion-only, lightweight). The plan's first GT
  task is a BoT-SORT integration spike; fall back only if it stalls.
- ReID here is **offline GT generation, not the algorithm under test** — permitted and
  not circular.
- Emits one trajectory `{frame_idx: (x,y,w,h)}` per object, with a **quality filter**
  (min length, no unresolved overlap/crossing ambiguity); only clean trajectories
  become trials.
- **Validation:** spot-check the GT tracks in the overlay visualizer; pseudo-GT
  quality is a documented limitation.
- Replaces the largest-only `build_target_trajectory` as the trial source (largest
  becomes one trial among K).

### 7.3 Exit criterion
A static-grid sweep and a dynamic run on the same clip produce the §5 metrics on the
same scale, same class ids, same trials, reproducible from a single CLI; GT tracks pass
visual spot-check.

## 8. Phase A — The simple algorithm, tuned

**Definition.** "Simple algorithm" = the **current dynamic scheduler unchanged**:
discovery grid on a fixed cadence + one track-ROI for the locked target + recovery grid
on loss, with the fixed **ROI-first** budget-trim drop order. No value function, no new
modes.

**Parameter sweep** (maximise the §5 metric suite at fixed budget):
- `discovery_period` (cadence), `discovery_grid` (density),
- **discovery overlap fraction** — *new knob to add* (`scheduler._grid` lays tiles
  edge-to-edge today; static grids use 0.25, so boundary objects fragment),
- `roi_margin_frac`, `max_zoom`, `target_model_h`,
- `recovery_grid`, `recovery_span`.

**Output:** best simple config + **metric-vs-budget curves** per fov, plotted against
the **static-grid frontier**. These curves are what Phase B must beat.

**Optional stretch within Phase A** (decided *after* the plain sweep; both keep the
algorithm "simple" — still fixed-priority):
- **Predictive ROI** — place the ROI at the velocity-predicted position (reduces loss
  events; directly targets the recovery metrics).
- **Skipped-tile carry-forward** — reuse last detections for unscanned tiles with
  staleness decay (`aggregator/memory.py` is half-built).

## 9. Phase B — Knapsack challenger

**Idea.** Replace the fixed ROI-first priority with **value-based selection**: each
candidate tile (ROI(s), discovery cells, recovery cells) gets a *value*; pick the
subset maximizing total value under the budget.

```
value(tile) ≈ P(tile contains a target) × resolution_gain(tile) × freshness(tile)
```

- `P(target present)` — from carry-forward memory / predicted motion / discovery prior.
- `resolution_gain` — how much zooming this tile lifts the detector toward its
  high-recall object size (tiny target → high gain; empty sky → ~0).
- `freshness` — value of re-scanning grows with time-since-last-scan (subsumes cadence).

**Why "knapsack" and not "top-K":** with unit tile costs it degenerates to greedy
top-K. It earns the name when **costs are non-uniform** — batched NPU cost, IoU-**merged**
ROIs (one tile, two targets' value), multi-resolution tiles (wider = costlier, more
coverage). The non-uniform cost model must ship with it.

**Comparison.** Tune the value function, then compare to Phase A's best **at equal mean
tiles/frame**, per fov, on the full §5 metric suite. Single question answered: *does
principled selection beat the tuned heuristic, and by how much — especially on tracking
and recovery?*

## 10. Deliverables

- Phase 0: one reproducible CLI producing comparable static + dynamic §5 metrics; the
  heavy-MOT GT-track builder + spot-checked GT trajectories.
- Phase A: tuned simple-scheduler config + metric-vs-budget curves (per fov) vs the
  static frontier; the new discovery-overlap knob.
- Phase B: knapsack selector + non-uniform cost model; head-to-head curves vs Phase A.
- A results section in `docs/paper/technical-report.md` updating §5/§6.

## 11. Risks & open questions

- **Pseudo-GT.** "GT" is dense-detector output associated by an offline tracker — not
  human labels. Tracking/recovery numbers are relative to pseudo-GT; documented, and
  spot-checked in the visualizer.
- **GT-track association errors at crossings** can create unfair trials — the quality
  filter (and optional ReID-assist) mitigate; rejected trajectories are dropped, not
  scored.
- **Knapsack reduces to greedy** if costs stay unit and the value function is crude —
  the non-uniform cost model (§9) is what makes it meaningful.
- **`HefBackend` vs GStreamer-golden divergence** — HefBackend throughout keeps runs
  self-consistent but not production-identical. Documented as a limitation.
- **Compute.** K trials × 3 fov × B budgets × configs is many runs; the cache/replay
  layer keeps it tractable, but the sweep grid must be sized deliberately.
- **`resolution_gain` estimation** needs a detector recall-vs-apparent-size curve;
  deriving it cheaply offline is a small sub-task.
