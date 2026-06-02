# Tile-Scheduler Experiment Design

> Spec for the experiment sweep that finds the best-performing dynamic
> tile-scheduling algorithm for drone single-target follow on a fixed-throughput
> Hailo NPU, and tests whether a budget-aware "knapsack" tile selector beats the
> tuned simple scheduler.

**Date:** 2026-06-02
**Branch:** `tiling-benchmark`
**Status:** design — pending user review before plan decomposition.

---

## 1. Goal

Find the **simple best-performing** dynamic tile-scheduling algorithm for
**single-target follow** at a fixed inference budget, establish its recall-vs-budget
frontier against the static-grid baseline, then test a **budget-aware knapsack tile
selector** as a challenger to see if principled selection beats the tuned heuristic.

The deliverable is a paper-with-code result: the tuned simple algorithm, the
knapsack comparison, and clean recall-vs-budget curves on real drone footage.

## 2. Background & framing

A Hailo NPU has **fixed throughput**: at a target FPS you get a fixed number *N* of
tile inferences per frame. Tiling therefore becomes a **budget-allocation** problem
— how to split *N* between *discovery* (find/keep targets), *track-ROI* (resolve the
target at high pixel density), and *recovery* (re-find after loss). Static slicing
(SAHI/ASAHI) has no budget and spends uniformly. The corrected Night-2 result showed
that at equal compute, dynamic ≈ static for whole-frame "detect everything" (a
uniform grid is near-optimal there) but dynamic **dominates for single-target
follow**. This experiment quantifies that win and tries to push it further.

## 3. Scope

**In scope (this round):**
- Single-target follow (target = a **person**).
- Person + vehicle as the only detected/tracked classes.
- Offline replay over recorded clips with ground truth — **no telemetry inputs**.
- Phase 0 bench standardization, Phase A simple-scheduler tuning, Phase B knapsack.

**Out of scope (deferred, must not be repainted-into-a-corner):**
- **Multi-target follow** — design abstractions must treat single-target as the K=1
  case so multi-target is a later extension, not a rewrite.
- **AltitudeZoom modifier** — implemented + unit-tested, but the benchmark clips carry
  no aligned AGL telemetry to drive it. Excluded from the matrix; stays in code as
  future-work (re-enable when a clip has DJI-SRT / PX4-ulg altitude aligned).
- **ReID re-acquisition** — a stronger ReID network exists but is not relevant to a
  detector-only budget sweep. Deferred.
- **CMC (camera-motion compensation)** — future work.

## 4. Standing project rules

1. **Classes: person + vehicle only.** Face and license-plate are removed
   project-wide. GT prep strips them; every detection filter keeps only
   `{person, vehicle}`; they never enter GT, metrics, the sweep, or any report.
   (`hailo_tiling/classes.py`: `TRACKED_CLASSES = (PERSON, VEHICLE)`,
   `TARGET_CLASS = PERSON`.)
2. **One class convention.** Standardize on **person = 1, vehicle = 2** end to end
   (the label-file convention). `HefBackend`'s 0-indexed decode is bridged with a
   `+1` offset so every artifact uses the same ids.
3. **One inference path.** All runs (static baseline + dynamic) go through
   **`HefBackend`** (OpenCV crop → HailoRT). The GStreamer cropper path deadlocks on
   per-frame relaunch and is not used for full-clip runs in this round.
4. **Single target = largest person at first appearance, identity-locked.** See §6.

## 5. Definitions

- **Budget** — mean **tiles per frame** (the per-frame inference count *N*). The
  primary independent variable. Reported as a swept curve, never a single point.
- **Primary metric** — **single-target follow recall**: fraction of clip frames in
  which the locked target (the GT trajectory bbox) is detected by the aggregated
  output (IoU ≥ 0.5 vs the GT target bbox).
- **Secondary metric** — target-bbox localization IoU (mean over detected frames).
- **Comparison rule** — algorithms are always compared at **equal mean tiles/frame**.
  Plot recall vs budget; the area/curve is the result.
- **Clips** — `DJI_20260528155239_0026` at emulated **fov50 / fov60 / fov70**
  (existing GT). Report per-fov so we see behaviour as the target shrinks.

## 6. Single-target selection (precise)

1. **GT trajectory build** (`dynamic_tiling/gt_track.py::build_target_trajectory`,
   `anchor="largest"`, `label="person"`): on the first GT frame containing a person,
   pick the **largest-area** person box; greedily associate it forward (IoU +
   centre-distance) into one trajectory `{frame_idx: (x,y,w,h)}` for the whole clip.
2. **Runtime lock** (`TargetLock.lock_from_gt`): on the first frame with detections,
   match the GT box by IoU to a ByteTracker track and pin a **stable lock identity**
   that never reassigns. Follow that identity; on loss, re-acquire by IoU vs
   last-known, else fall to the recovery grid.
3. **Rationale:** mirrors the app's AUTO mode (follow the largest/nearest person);
   freezing identity via GT makes recall reproducible.

**Forward-compat (multi-target later):** `MultiTargetLock` already keys targets by
`(cls, track_id)` and GT-seeds a `selected_key`. Single-target is the K=1 case of the
same machinery; scoring is written to average per-target recall so K>1 drops in
without a rewrite.

## 7. Phase 0 — Bench standardization (prereq, no science)

Make the harness produce comparable numbers before any tuning counts.

- Route static-baseline and dynamic runs through **`HefBackend`** only.
- Apply the **person=1 / vehicle=2** convention everywhere (add the `+1` offset to
  `HefBackend` decode; update `run()`/`run_multi()` defaults `person_cls`/
  `target_classes`, and the replay label tuple, accordingly).
- Confirm GT prep and all detection filters keep only `{person, vehicle}`.
- Lock the metric + protocol of §5 into the scoring code so every run reports
  recall-vs-budget identically.
- **Exit criterion:** a static-grid sweep and a dynamic run on the same clip produce
  recall numbers on the same scale, same class ids, same metric — reproducible from a
  single CLI.

## 8. Phase A — The simple algorithm, tuned

**Definition.** "Simple algorithm" = the **current dynamic scheduler unchanged**:
discovery grid on a fixed cadence + one track-ROI for the locked target + recovery
grid on loss, with the fixed **ROI-first** budget-trim drop order. No value function,
no new modes.

**Parameter sweep** (find best single-target follow recall at fixed budget):
- `discovery_period` (cadence), `discovery_grid` (density),
- **discovery overlap fraction** — *new knob to add* (today `scheduler._grid` lays
  tiles edge-to-edge; static grids use 0.25, so boundary objects fragment),
- `roi_margin_frac`, `max_zoom`, `target_model_h`,
- `recovery_grid`, `recovery_span`.

**Output:** the best simple config + a **recall-vs-budget curve** per fov, plotted
against the **static-grid frontier** (existing baseline). This curve is what Phase B
must beat.

**Optional stretch within Phase A** (decided *after* seeing the plain sweep; both keep
the algorithm "simple" — still fixed-priority, just better ROI/memory):
- **Predictive ROI** — place the ROI at the target's velocity-predicted position
  rather than last-known, to cut recovery events.
- **Skipped-tile carry-forward** — reuse last detections for unscanned tiles with a
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
top-K. It earns the name when **costs are non-uniform** — which is our case:
batched NPU cost, IoU-**merged** ROIs (one tile, two targets' value), multi-resolution
tiles (wider = costlier, more coverage). The non-uniform cost model must ship with it.

**Comparison.** Tune the value function, then compare to Phase A's best **at equal
mean tiles/frame**, per fov. Single question answered: *does principled selection beat
the tuned heuristic, and by how much?*

## 10. Deliverables

- Phase 0: one reproducible CLI producing comparable static + dynamic recall numbers.
- Phase A: tuned simple-scheduler config + recall-vs-budget curves (per fov) vs the
  static frontier; the new discovery-overlap knob.
- Phase B: knapsack selector + cost model; head-to-head curves vs Phase A.
- A results section in `docs/paper/technical-report.md` updating §5/§6.

## 11. Risks & open questions

- **Knapsack reduces to greedy** if costs stay unit and the value function is crude —
  the non-uniform cost model (§9) is what makes it meaningful. If the cost model
  proves impractical offline, Phase B's claim weakens to "value-ranked greedy."
- **Only one base clip (0026).** Generalization is thin; if time allows, add a second
  clip with GT. Flagged, not blocking.
- **`HefBackend` vs GStreamer-golden divergence** — using HefBackend throughout keeps
  runs self-consistent, but production uses the GStreamer cropper; numbers are
  internally comparable, not production-identical. Documented as a limitation.
- **Resolution_gain estimation** needs a detector recall-vs-apparent-size curve;
  deriving it cheaply offline is itself a small sub-task.
