# Weekend autonomous run — recovery fixes, ReID budget ablations, Phase A sweep, MOT eval

**Date:** 2026-06-04 · **Branch:** `tiling-benchmark` · **Approved by:** Gilad
**Mode:** multi-day autonomous run (entire weekend), `autonomous-project-manager` pattern.

## Context

The 0025 verified-GT baseline (`dynamic_tiling/runs/baseline_0025/BASELINE.md`) proved:
hold quality is solved (IoU 0.86–0.90, zero drift); **coverage is decided entirely by
re-acquisition after loss**. Root cause: `TargetLock.step()` re-locks only when an
activated track IoU-overlaps the **frozen** last-known bbox — walking targets leave the
anchor and the loss becomes permanent, even with an activated ByteTracker track sitting
on the target for 100+ frames.

All work runs on the **offline harness** (`dynamic_tiling/`, direct `HefBackend` +
SQLite tile cache), NOT the drone-native GStreamer pipeline. Native-pipeline migration
is explicitly out of scope (interactive C++ work; known per-frame relaunch deadlock).

Research input: `docs/research/2026-06-04-reid-inference-reduction-survey.md`
(ReID-inference reduction taxonomy; ranked recommendations folded into Block R).

## Goals (priority order; manager drops from the tail only if forced)

1. **Block 1 — Motion-model re-acquisition fixes** in `TargetLock`.
2. **Block R — ReID-based recovery** + inference-budget reduction algorithms + ablations.
3. **Block 2 — Phase A single-target tiling sweep** (full 2-pass coordinate descent + budget frontier).
4. **Block 3 — MOT metrics harness** + first multi-target scorecard.
5. **Stretch** — overlap×grid re-sweep under fixed recovery; MOT at multiple budgets vs dense static.

MOT metric coding (CPU-only) runs in parallel with chip-bound blocks throughout.

## Datasets / GT

- **0025** locked GT, 3 fovs: `dynamic_tiling/runs/gt_verify_0025_fov{50,60,70}/gt_tracks.verified.json`
  (ids: 1 car-right, 2 car-left, 3 walker, 4 bg-person; fov70 adds 5 car-bottom).
- **0026 fov50** verified GT (26 tracks: 11 person + 15 vehicle):
  `dynamic_tiling/runs/gt_verify_0026_fov50/gt_tracks.verified.json` — richer scene for
  ReID ablations + MOT.
- Videos: `/home/giladn/Videos/Drone/Training/DJI_*_prepared__fov*.mp4`.
- Known failure cases (regression set for recovery fixes): walker trials fov50-ov0,
  fov60-ov25, fov70-ov25.

## Block 1 — Re-acquisition motion fixes (`dynamic_tiling/target_lock.py`)

Two independent knobs, **both off by default** (back-compat; sweeps/ablations flip them):

- `reacq_motion="frozen"|"velocity"` — with `"velocity"`, advance the search anchor by
  `state.last_velocity` each lost frame (velocity is already retained; the scheduler
  already extrapolates ROI placement with it).
- `reacq_radius_growth=k` (float, 0 = today's strict-IoU behaviour) — accept an activated
  track whose **centre distance** to the anchor is within `r0 + k·frames_lost`
  (`r0` derived from anchor size). Implemented alongside, not instead of, the IoU path:
  IoU match keeps priority; distance gate is the fallback.

TDD: synthetic trajectories (walker leaves anchor; stationary target; two crossing
targets must NOT swap locks). **Gate:** the three known real failures flip to
recovered (walker coverage > 0.8 in each) with no regression on the other six
baseline cells; full test suite green.

## Block R — ReID recovery + inference-budget ablations

### Hard constraints (user)
- ReID runs on **person bbox crops only**. Never full frame, never vehicles. The
  embedder API takes a person `Det` + frame and asserts the class; full-frame use is
  impossible by construction.
- First establish the **generous upper bound**, then reduce inferences with gated
  algorithms; ablate each algorithm AND a no-ReID arm.

### Components (new modules in `dynamic_tiling/`)
- `reid_embedder.py` — `ReidEmbedder.embed(frame, person_det) -> np.ndarray` via the
  ReID HEF (same model family the prod app uses — see `drone_follow/pipeline_adapter/`
  `reid_manager.py` and `reid_analysis/` for crop/normalize conventions). First task
  verifies HEF presence for this machine (H10: v5.3.0 `hailo10h` HEFs per
  install notes) and records the exact model in the results meta.
- **Embedding cache**: `embeddings` table in the existing SQLite cache schema
  (`frame_idx, crop_x, crop_y, crop_w, crop_h, model, vec BLOB`) + a
  `CachedReidEmbedder`. All ablation re-runs are chip-free after first pass.
- `reid_gallery.py` — offline port of prod gallery logic: cosine matching,
  `reid_threshold` / `drift_threshold` / `duplicate_threshold` / `refresh_every` knobs,
  FIFO replacement; plus an **EMA appearance anchor** option (StrongSORT-style) as a
  gallery-policy sub-ablation.
- `TargetLock` integration: a `reid_policy` hook — while TRACKING, sample gallery per
  policy; while SEARCHING/LOST, embed candidate person dets per policy and re-lock when
  cosine similarity crosses `reid_threshold`. New counters surfaced in metrics/results
  JSON: `reid_inferences`, `reid_inferences_per_frame`,
  `fraction_of_person_dets_embedded`.

### Ablation arms (policies)
| arm | policy |
|-----|--------|
| P0  | no ReID (Block 1 motion fixes only) |
| P1  | **generous**: embed locked target every frame while tracking; embed every person det every lost frame (quality upper bound) |
| P2  | prod-style: `update_interval` gallery sampling + drift/duplicate gates; match continuously during loss |
| P3  | **ambiguity-gated** (survey #1, arXiv 2409.06617 risk test): skip ReID iff exactly one confirmed track overlaps the det (clean continuation); embed only contested/unmatched dets; never embed low-confidence (2nd-stage) boxes |
| P4  | motion-gated + cadence decay: embed only candidates inside the Block-1 (growing) motion gate; decaying embed rate during long loss |
| P5  | (stretch) colour-histogram pre-filter (SDG-Track-style) shortlists recovery candidates before ReID |

Sub-ablation on the best arm: FIFO gallery vs EMA anchor vs FIFO+EMA.

### Scoring
Each arm × {0025 fov50/60/70, 0026 fov50}, budget 3000, best Block-1 config, fixed
seed/order. Report: coverage, mean IoU, drift rate, loss events, time-to-recover,
recovery success, **ReID inferences/frame**, **fraction of person dets embedded**.
Headline figure: quality-vs-embedding-fraction curve (P1 = upper bound, P0 = floor).
Deliverable: `dynamic_tiling/runs/REID_ABLATION.md` (committed) + per-run JSONs (untracked).

## Block 2 — Phase A single-target tiling sweep

Coordinate descent (NOT full cross), 2 passes over axes, all 3 fovs of 0025, recovery
config = best of Block 1+R, `--cache` always:

- discovery grid: {4x3, 6x4, 8x6, 12x9}
- discovery fps: {1, 2, 4}
- discovery overlap: {0, 0.15, 0.25}
- max zoom: {1.5, 2, 3}
- target_model_h: {30, 40, 60}

Then a **budget frontier** at the best config: budget ∈ {300, 600, 1000, 1500, 3000} →
coverage vs tiles/frame curve (the paper's headline for single-target follow).
Deliverable: `dynamic_tiling/runs/PHASE_A.md` (committed; per-run JSONs untracked).

## Block 3 — MOT metrics + first multi-target scorecard

- `dynamic_tiling/mot_metrics.py`, in-repo minimal, no new deps: per-frame greedy IoU
  matching (deterministic) → **IDF1, MOTA (FP/FN/IDsw), ID-switch count**, plus
  recovery-oriented extras (Frag, MT/ML). HOTA deferred.
  Unit tests: perfect tracking, identity swap, fragmentation, FP flood, empty preds.
- `run_multi` integration: persist per-track predictions so `score_mot(gt_tracks, preds)`
  runs offline.
- First scorecard: `run_dynamic --multi-target` on 0026-fov50 vs verified GT, equal-budget
  dense-static comparison row. Deliverable: `dynamic_tiling/runs/MOT_BASELINE.md`.

## Operations

- **Execution:** `autonomous-project-manager` agent + durable state file
  (`docs/superpowers/overnight-manager-state.md` pattern — new file
  `docs/superpowers/weekend-manager-state.md`). Subagent-driven development: each task
  implemented by a subagent, two-stage review (spec compliance + code quality), final
  review per block.
- **Session limits / quota:** per `handling-anthropic-session-limits` skill — on
  session-limit termination, salvage partial work, write state, **set a cron** to
  relaunch the manager after the reset time, and exit cleanly. Resume from state file.
- **Chip:** exclusive resource — one chip job at a time; everything goes through the
  SQLite caches (`dynamic_tiling/runs/cache/`); CPU-only work (MOT metrics, analysis)
  runs in parallel.
- **Git:** commit reviewed code + tests + docs per task. Result JSONs / frame dumps /
  caches stay **untracked** (final summary MDs are docs and are committed). Branch
  `tiling-benchmark` only. **Never push. Never touch** `reid_manager.py` (prod),
  submodule pointers, or pre-existing dirty files.
- **Telegram:** message on block completion (milestone summary) and on hard blocks
  needing a human; otherwise silent.
- **Test discipline:** full `dynamic_tiling` suite green before every commit; TDD for
  all new modules.

## Acceptance criteria (morning-after review)

1. Block 1 gate met (3 known failures recovered, no regressions, tests green).
2. REID_ABLATION.md exists with the quality-vs-embedding-fraction curve incl. P0 and P1.
3. PHASE_A.md exists with per-axis tables + budget frontier at best config.
4. MOT_BASELINE.md exists with IDF1/MOTA/IDsw for multi-target dynamic vs dense static.
5. State file tells the full story (what ran, what's pending, where every artifact is).
6. All commits reviewable, suite green at HEAD, nothing pushed.
