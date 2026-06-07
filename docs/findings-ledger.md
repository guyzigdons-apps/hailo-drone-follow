# Findings ledger — dynamic tiling / GT / ReID strategies

> Single source of truth for **what we tried, what happened, and what's now policy**.
> Update on every experiment round. Status values: **ADOPTED** (default changed),
> **CONDITIONAL** (use when stated condition holds), **REJECTED** (evidence against),
> **NEUTRAL** (no measurable effect), **OPEN** (under test).

## Tracking & re-acquisition

| Strategy | Result | Evidence | Status |
|---|---|---|---|
| ByteTracker dedup fix (`1−IoU` distance, was raw IoU) | Removed the structural ~90-frame (track_buffer) re-lock floor; THE recovery lever | `e01fc4f`; BASELINE.md validation | **ADOPTED** ⚠ prod-shared — sim sanity before flight still pending |
| Reacq distance gate, radius grows with frames-lost (`frozen/0.001`) | Walker 0.054→0.989 (with dedup fix + overlap); 11/12 trials ttr ≤ 3 frames | Block-1 gate, BASELINE.md | **ADOPTED** (default 0.001) |
| Velocity-extrapolated reacq anchor (`--reacq-motion velocity`) | Neutral once growth>0; HARMFUL alone at fov60 | Block-1 validation grid | **REJECTED** (honest negative; frozen is default) |
| Reacq radius growth ×10 (`0.01`) on association-loss clips | 0026: cov_fw 0.754 @ **1.04 tiles/f** — near-free recovery of fast movers | round2 probe P5 | **OPEN → candidate new default** (re-verify no regression on 0025/0027 first) |
| Discovery overlap (0.25 → 0.15 at winner) | Shifts WHEN losses happen; matters only via the reacq gate; 0.15 enough at 6x4 | BASELINE.md §overlap; Phase-A sweep | **ADOPTED** (0.15 at winner config) |
| Discovery cadence 2 fps on fast-mover clips | 0026: cov_fw 0.786 but 2× tiles (2.08 t/f); combo w/ growth ×10 = 0.797 | round2 probes P4/P6 | **CONDITIONAL** (pay 2× only for fast targets) |

## Tiling / budget

| Strategy | Result | Evidence | Status |
|---|---|---|---|
| Phase-A winner grid: **6x4 @ 1 fps, ov 0.15** | 0.989 coverage @ 1.59 tiles/f on 0025 (old default 8x6@2fps: same cov @ 2.4) | PHASE_A.md, 63-config sweep | **ADOPTED** (single-target follow) |
| Budget frontier | Saturates at 600 (1.41 t/f); collapses at 300 | PHASE_A.md | **ADOPTED** (600 = default budget) |
| 4x3 discovery grid | Coverage cliff (0.955) | Phase-A sweep | **REJECTED** |
| Zoom/model_h changes at winner config (0025) | No-ops | Phase-A sweep | **NEUTRAL** |
| Dynamic tiling for whole-frame "detect everything" | Does NOT beat equal-budget static (uniform grid is efficient); dynamic wins at SINGLE-TARGET follow | 2026-06-01 eval; MOT_BASELINE.md | **ADOPTED as scope** (dynamic = target-follow tool) |
| Zoom (4×) + target_model_h 25 for tiny targets (0027, ~6–8 px @VGA) | No gain (0.513 vs 0.535); zoom adds drift (0.297) | round2 probes P1/P3 | **REJECTED** (detection-ceiling-bound — needs detector-level fix, not scheduler knobs) |

## ReID

| Strategy | Result | Evidence | Status |
|---|---|---|---|
| ReID on open drone scenes (0025/0026 class) | Zero quality delta — motion gates act first; 0026 losses detection-bound | REID_ABLATION.md | **NEUTRAL** here (default: none / ambiguity-as-insurance) |
| Gated embed policies (ambiguity/motion) vs generous | ~3% dets embedded vs 81% — ~28× cheaper at equal quality | REID_ABLATION.md | **ADOPTED** (if ReID on, gate it) |
| Person-crop-only ReID (never full frame, never vehicles) | Constraint held airtight through all callers | R1–R4 review | **ADOPTED** (enforced in reid_embedder) |
| ReID on close clips (0012/0013 with person pairs) | — | pending GT locks | **OPEN** (D2) |

## Metrics / evaluation

| Strategy | Result | Evidence | Status |
|---|---|---|---|
| Unweighted per-track mean aggregates | DISTORTS: 0026 recovery printed 0.682, event-weighted truth 0.88; 16-frame fragments weigh like 334-frame tracks | round2 failure analysis | **REJECTED** for headline numbers |
| Frame-weighted coverage + event-weighted recovery (`coverage_fw`, `recovery_success_ew`) | Honest aggregates; legacy fields kept for comparability | `e8c0bdd` | **ADOPTED** (quote fw/ew in reports) |
| Score only frames actually played (`--max-frames`) | Avoids recall-denominator distortion | run_dynamic | **ADOPTED** |

## GT generation

| Strategy | Result | Evidence | Status |
|---|---|---|---|
| Dense 12x9 + extras → BoT-SORT → review queue → corrections chain → lock | Produced 0025/0026/0027 + round-2 GT; replayable provenance | gt-generation-guide | **ADOPTED** |
| "Frame-sync glitch" hypothesis for 0013 bbox jumps | DISPROVEN: uniform PTS both files, 0 dup/drop, decoders aligned, transcode timing exact | 0013 forensics (PNGs /tmp/glitch_0013) | **REJECTED** |
| vga3-rect NMS-flip hypothesis for the jumps | Dense dets were CORRECT at jump frames — rect not the culprit for THIS artifact. (BoT-SORT did match short leg-cut dets, so multiscale dense tiles remain a requested improvement) | 0013 forensics | **REJECTED** for the jump; multiscale dense config still **OPEN** (Gilad request) |
| Despike with a single global anchor | ARTIFACT SOURCE: 0013 mixes head-cut (majority, long runs) + leg-cut (every ~3rd frame); anchor=bottom lifted leg-cut boxes over the head (cy +0.035, reproduced to 4 decimals). PLUS center-vs-topleft convention bug in the anchor math | 0013 forensics; fix in flight | **REJECTED** → per-frame **auto anchor** (hold the stabler edge) + convention fix |
| Despike threshold 0.9 + h-only smooth (close clips) | 0.7 too lax (left 150 frames); 0.9+smooth → jitter ↓8–16× | `de85700`; 0013 v2 | **ADOPTED** (anchor choice superseded by auto) |
| GT tiles rendered in overlay viewer | 116-tile dense layout visible for debug; render-only regen safe on locked dirs | `b271aff` | **ADOPTED** |
| `max_gap` = frame-index distance, NOT missing-frame count | "12 missing frames → max-gap 12" silently no-ops | 0013 interp | **ADOPTED** (gotcha documented) |
| Mandatory smoothing/deglitch in finalize | requested 2026-06-07 ("enforce tracking and smoothing") | — | **OPEN** (implementation pending forensics) |
| Clips don't all need 3 FOVs — close clips use native FOV | 0012/0013 single-variant GT in minutes | round 2 | **ADOPTED** |
| Frame-inspect a clip before interpreting odd GT (0029 "all-vehicle" was the scene, not a bug) | Saved a false bug-hunt | round 2 | **ADOPTED** (method) |

## Infrastructure / process

| Strategy | Result | Evidence | Status |
|---|---|---|---|
| SQLite tile-inference cache on every chip run | ≥82% of ~479k weekend inferences cache-served (~145 min saved); counters in every run line | `1e6db0a`; REID_ABLATION.md | **ADOPTED** (never run uncached) |
| Class conventions via `class_offset=1` at EVERY backend construction | Two weekend bugs from missing it; now ast-enforced | `591f557`/`eae88c1`; test_cli_class_offset | **ADOPTED** (test-enforced) |
| Architecture dependency rules as a test | Caught 2 real violations on first run + the probe_phantom_hef sys.path landmine class | tests/test_architecture.py | **ADOPTED** |
| Manager runs long chip jobs; subagents write code only | Two executor agents died backgrounding+pausing | weekend ops | **ADOPTED** (PM agent anti-pattern F) |
| `[b]racket` pgrep/pkill patterns; pipefail on gating pytest; venv in every command | Each bit us at least once (incl. pkill self-match again 2026-06-07) | weekend + round 2 | **ADOPTED** (PM agent G/H) |

## Open questions

- 0027-class tiny targets: detector-level fix (bigger input / conf floor in tracked tile / model upgrade) — H3 of round-2 analysis, untested.
- `reacq_radius_growth 0.01` as global default — needs no-regression check on 0025.
- MOT re-score at Phase-A winner config; 0019 multi-car scorecard (gated on GT lock).
- Person-FP floor on 0026-class scenes (vehicles mislabeled "person" by VGA model).
- vga-3x multiscale grid dense config (gated on forensic verdict).
