# Tiling Lab Restructure + Dynamic-Tiling Pipeline Integration — Design

**Date:** 2026-06-07 · **Branch:** `tiling-benchmark` · **Approved direction:** dynamic-cropper integration + full `tiling_lab` restructure (Gilad, 2026-06-07)

## Goal

Separate the dynamic-tiling benchmark + GT-generation code from the production
drone-follow code, while the benchmark keeps importing drone-follow for tests
(the offline harness exercising the *real* prod ByteTracker is how the
`e01fc4f` flight-relevant bug was found — that dependency direction is a
feature). Promote the proven scheduler into the installed `hailo_tiling`
library so the prod pipeline can consume it, and define the integration path
(dynamic cropper element).

## Current state (measured 2026-06-07)

| Unit | LOC (code) | Installed | Problem |
|---|---|---|---|
| `drone_follow/` | 7.6k | yes | none — zero imports of research code |
| `hailo_tiling/` | 5.0k | yes | one import into legacy (`bench/grid.py` → `tiling_benchmark.tiling_record._grid_to_static_tiles`) |
| `dynamic_tiling/` | 5.4k | **no** (`_vendor_paths.py` sys.path hack) | 44-file flat dir mixing algorithm, harness, ReID, and 23 GT-tool files; shim debris (`budget.py`, `types.py` re-exports) |
| `tiling_benchmark/` | 7.8k | no | mostly-dead legacy; live code still reaches in for `overlay_viewer.py`, `prepare_video.fov_to_crop_dims`, `tiling_record._grid_to_static_tiles` |

Cross-unit imports (complete list):
- `dynamic_tiling.target_lock` → `drone_follow.pipeline_adapter.{tracker_factory, byte_tracker}` — **keep** (prod tracker under test).
- `dynamic_tiling` → `hailo_tiling` (18 files) — correct direction.
- `dynamic_tiling.run_gt_crossfov_fill` → `tiling_benchmark.prepare_video.fov_to_crop_dims` — to be absorbed.
- `hailo_tiling.bench.grid` → `tiling_benchmark.tiling_record._grid_to_static_tiles` — to be absorbed.
- `scripts/warm_dynamic_cache.py` → `dynamic_tiling.gt_track`; `scripts/warm_gst_cache.py` → `tiling_benchmark.tiling_record` — update paths.

## Target layout

```
hailo_tiling/                  # installed library — only home for prod-bound algorithm code
  dynamic/
    __init__.py
    scheduler.py               # ← dynamic_tiling/scheduler.py (TileScheduler, MultiTargetTileScheduler)
  geometry.py                  # ← fov_to_crop_dims (tiling_benchmark/prepare_video.py)
                               #   + grid_to_static_tiles (tiling_benchmark/tiling_record.py, de-underscored)

tiling_lab/                    # NEW research workspace — installed, never imported by prod
  __init__.py
  harness/    replay.py, target_lock.py, inference.py, trials.py, score.py,
              metrics.py, mot_metrics.py, compare_baselines.py, aggregator.py
  reid/       reid_embedder.py, reid_gallery.py, reid_policy.py
  gt/         gt_track.py, gt_clean.py, gt_dedup.py, gt_edit.py, gt_mot.py,
              gt_review.py, gt_render_review.py, gt_review_gui.py,
              run_gt_*.py (all 15 correction/verification CLIs)
  cli/        run_dynamic.py, run_trials.py, run_sweep.py,
              run_reid_ablation.py, run_mot_eval.py
  viewer/     overlay_viewer.py (← tiling_benchmark), adapt_frames_for_viewer.py (← scripts/)
  video/      prepare_video.py (← tiling_benchmark; imports fov_to_crop_dims from hailo_tiling.geometry)
  runs/       # artifacts dir: tracked final MDs (BASELINE/PHASE_A/REID_ABLATION/MOT_BASELINE)
              # git-mv'd here; untracked artifacts (caches, frames dumps, GT trees) physically moved alongside
  tests/      ← dynamic_tiling/tests (34 files)

tiling_benchmark/              # FROZEN in place + DEPRECATED.md banner. Not physically moved
                               # (46 GB untracked pxt_runs/ lives here; paths referenced by GT docs).
                               # After this restructure nothing imports it. Delete later at leisure.
```

`dynamic_tiling/` ceases to exist (git mv into the homes above). Deleted
outright: `budget.py` + `types.py` re-export shims (imports rewritten to
`hailo_tiling.budget` / `hailo_tiling.types`), `_vendor_paths.py` (packaging
makes it obsolete).

## Dependency rules (enforced by test)

- `hailo_tiling` imports nothing internal (no drone_follow, no tiling_lab, no tiling_benchmark).
- `drone_follow` may import `hailo_tiling` only (the integration seam). Never tiling_lab/tiling_benchmark.
- `tiling_lab` may import `drone_follow` (tracker under test) and `hailo_tiling`.
- Nothing imports `tiling_lab` or `tiling_benchmark` (except scripts/, which may use tiling_lab).

Enforcement: `tests/test_architecture.py` — walks each package's `.py` files,
parses `import`/`from` statements (ast), asserts the rule table above. Runs in
the normal suite; fails the build on any future violation.

## Packaging

`pyproject.toml`:
- `include = ["drone_follow*", "reid_analysis*", "hailo_tiling*", "tiling_lab*"]`,
  exclude `tiling_lab.tests*` alongside the existing excludes.
- No new console scripts; runners stay `python -m tiling_lab.cli.run_trials` etc.
- `pytest.ini` testpaths: `hailo_tiling/tests`, `tiling_lab/tests`, `tests`.

## Migration mechanics

1. `git mv` per the layout table (history-preserving), one commit per
   destination subpackage; full suite green between commits (floor: current
   435 + 227 passing).
2. Import rewrite sweep (`dynamic_tiling.` → new module paths) in code, tests,
   and scripts/. The two `scripts/warm_*` files update with it.
3. Docs path sweep: `docs/gt-generation-guide.md`, runbooks, README snippets —
   `dynamic_tiling/` → `tiling_lab/` (incl. `runs/` paths). Historical docs
   (weekend state file, old specs/plans, committed reports' provenance lines)
   are NOT rewritten — they describe the past.
4. Artifacts: `git mv` the tracked MDs under `dynamic_tiling/runs/` →
   `tiling_lab/runs/`; plain `mv` the untracked siblings (caches, GT verify
   trees, frames dumps) preserving relative structure. GT lock integrity:
   `GT_STATUS.json` hashes file *content*, not paths — verify one locked tree
   re-validates after the move before moving the rest.
5. Cleanup commits: delete shims + `_vendor_paths.py` + stale root
   `HANDOFF.md` (superseded by `docs/superpowers/weekend-manager-state.md`);
   add `.gitignore` entries for `.venv_gt/`, `tiling_lab/runs/**` artifact
   patterns (with `!*.md` so final reports stay trackable); add
   `tiling_benchmark/DEPRECATED.md`.
6. Triage the dirty `dynamic_tiling/run_gt_zoom_recover.py` (modified,
   uncommitted) before its mv: review diff → commit or revert. The
   pre-existing dirty `drone_follow/pipeline_adapter/reid_manager.py` is NOT
   touched (standing rule).
7. Follow-up folded in: class-convention straggler sweep — grep all
   `HefBackend(` construction sites; every one passes `class_offset=1` or has
   a comment justifying raw ids; add the missing regression test.

## Phase 2 — pipeline integration (dynamic cropper element)

Decision: integrate via the vendored `hailotilecropper_dynamic` GStreamer
element. The scheduler lives in `hailo_tiling.dynamic` (installed → importable
by `drone_follow`). Architecture:

- `drone_follow.pipeline_adapter` hosts a `DynamicTilingController`: consumes
  per-frame detections/track state from the detection callback, runs
  `MultiTargetTileScheduler.plan()`, and pushes the planned tile set to the
  cropper element as a normalized `tiles-static` string
  (`"x,y,w,h;..."`), replacing the static grid configured today.
- ReID stays out of scope for Phase 2 (weekend ablation: no quality delta on
  current data; revisit on harder clips).

**Gate — spike S1 (half-day, before any Phase-2 plan is written):** verify the
installed element re-reads `tiles-static` per cropping period at runtime
(set property from an `identity` handoff / pad probe; confirm crop set changes
mid-stream, no caps renegotiation stalls).
- S1 pass → Phase 2 proceeds as above.
- S1 fail → extend the C++ (per `.claude/memory/hailotilecropper_dynamic.md`:
  property declarations + upstream `prepare_tiles()` call, est. ~50 lines,
  rebuild via `hailo-compile-postprocess install`), then re-run S1.

Phase 2 gets its own spec + plan after S1; it does NOT block the restructure.
Precondition for flying any of it: Gilad's sim/flight sanity pass on the
`e01fc4f` tracker change (still pending).

## Testing

- Full suite green at every commit (435 + 227 floor; counts re-baselined in
  the plan at execution time).
- New `tests/test_architecture.py` import-rule test.
- Smoke after migration: one warm-cache `python -m tiling_lab.cli.run_trials`
  run on 0025 fov50 reproduces the committed BASELINE numbers (cache makes
  this chip-cheap); `python -m tiling_lab.viewer.overlay_viewer --help` works.

## Out of scope

- Deleting `tiling_benchmark/` (frozen, not removed).
- Any behavior change in `drone_follow` prod code (Phase 2 is design-only here).
- ReID productization; harder-clip ReID round; 0027 GT lock (awaits human review).

## Risks

- **Path churn in muscle memory / runbooks** — mitigated by the docs sweep +
  `tiling_benchmark/DEPRECATED.md` pointing to new homes.
- **GT lock trees**: chmod-444 files complicate `mv` only if copied (not
  moved) — use `mv` within the same filesystem; verify re-validation first.
- **Hidden importers** of `dynamic_tiling.*` outside the repo (none known;
  the architecture test catches in-repo regressions).
