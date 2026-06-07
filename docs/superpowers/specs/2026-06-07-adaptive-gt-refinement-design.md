# Adaptive GT Refinement — Design

**Date:** 2026-06-07 · **Approved direction (Gilad):** pilot-first on current clips; loop converges fully BEFORE human review.

## Goal

GT generation should not be a single uniform sweep. After the round-1 dense
pass, a **planner** reviews the evidence (target positions, sizes, scale-layer
geometry, detection quality) and buys exactly the additional inference that
closes the observed gaps — iterating dense → plan → targeted sweep → re-track
until the tracks stop changing — and only THEN produces the one review queue a
human sees. Two payoffs:

1. **Best GT in the fewest human iterations** (Gilad's review time is the
   scarce resource; chip inference in the GT regime is effectively free —
   brute force, heavy trackers, and LLM reasoning are all allowed).
2. **Oracle lessons for the online scheduler**: every gap the planner has to
   patch offline is a candidate weakness of the runtime dynamic tiling. The
   refinement log is a design input for fine-tuning and for the real-time
   scheduler (e.g., recurring seam-straddle patches ⇒ runtime needs
   boundary-aware ROI sizing).

## The loop (per clip-fov)

```
R1: uniform dense sweep (existing GT-12x9-25-multi) → BoT-SORT → tracks v1
LOOP (≤3 rounds, evidence-driven):
  PLAN  — analyze tracks vN + dense det pool:
    (a) seam-straddle: object bbox vs tile geometry at its "owning" scale
        layer; object spanning a tile boundary with no whole-object tile →
        add targeted rect (or local overlap bump)
    (b) truncation signature: per-track top/bottom edge oscillation (the 0013
        leg-cut pattern) → add whole-object-scale tile over the track corridor
    (c) dropout/low-conf corridors: missing or low-conf segments → zoom tiles
        along the trajectory (generalizes run_gt_zoom_recover's gap recovery)
    (d) tiny targets: median height near the detector floor (<~10 px @VGA) →
        high-zoom tiles along the full trajectory (0027-class)
  SWEEP — infer ONLY the planned additional tiles (CachedHefBackend; union
        with R1 cache makes repeats free) → merge into the dense det pool
  RE-TRACK — BoT-SORT over the enriched pool (fresh ids each round is fine —
        pre-review)
  CONVERGED? — stop when track-level deltas are immaterial: no new tracks ≥
        min_len, per-track coverage gain < 2%, height-stability stable
THEN: auto stages (dedup/auto-merge/review queue + PNGs) → ONE human review
      → corrections chain → lock (unchanged from today)
```

Every PLAN decision is logged: {trigger evidence, tiles added, det/track delta
it produced}. The per-clip refine report feeds `docs/findings-ledger.md` and a
new **oracle-lessons** section mapping each recurring patch type to a runtime
scheduler implication.

## Pilot (this round — manual/LLM-assisted, no new pipeline code)

Clips chosen for known round-1 weaknesses:
- **0013** (truncation case): R1 produced every-~3rd-frame leg-cut fragments
  the tracker latched onto (fixed post-hoc by despike-auto; refinement should
  instead fix the DETECTIONS — does a whole-object-scale tile over the person
  corridor remove the oscillation at the source?)
- **0027 fov50** (tiny-target case): even the 12x9 dense pass misses the two
  persons in ~20% of frames — the GT itself underestimates them. Planner adds
  high-zoom corridor tiles; measure recovered GT frames.

Pilot mechanics: analysis subagents implement PLAN by hand (scripts in /tmp +
existing CLIs); SWEEP via the existing backends/caches; RE-TRACK via the
existing gt_mot path; chip runs manager-owned. Metrics per round: GT frames
recovered, height-stability (ratio-band table), review-case count, planned
tiles count. Success = converged GT measurably better than R1 with ≤2 refine
rounds and a SMALLER review queue.

## After the pilot

Codify what actually worked as `tiling_lab/gt/run_gt_refine` (planner =
deterministic heuristics distilled from the pilot; LLM-assisted planning stays
optional for odd clips). Separate plan, gated on pilot results. The
multiscale-dense-config request (vga-3x grid instead of single center rect)
folds into the planner's vocabulary rather than a static config change.

## Out of scope

- Changing the online scheduler now (lessons accumulate first).
- Automating the planner before pilot evidence.
- Re-running already-locked GT (0012, 0025 family) — refinement applies to new
  clips and not-yet-locked ones.
