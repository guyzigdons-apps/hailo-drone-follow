# Weekend autonomous run — manager state

> **DURABLE MEMORY.** The managing agent updates this file after EVERY completed task,
> block transition, or incident. A fresh session must be able to resume from this file
> alone. Convert every relative time to absolute.

## Mission

Execute the four block plans of
`docs/superpowers/specs/2026-06-04-weekend-recovery-reid-sweep-mot-design.md`, in order,
via subagent-driven development (implementer subagent per task + two-stage review),
over the weekend of 2026-06-05 → 2026-06-08. Approved by Gilad 2026-06-04.

| order | plan | status |
|-------|------|--------|
| 1 | `docs/superpowers/plans/2026-06-04-block1-reacq-motion-fixes.md` | **DONE — GATE PASSED** (commit 8e3b4f1; best config `--reacq-motion frozen --reacq-radius-growth 0.001` — use THIS, not the velocity/0.002 placeholder, in R6/B2 executions) |
| 2 | `docs/superpowers/plans/2026-06-04-blockR-reid-recovery-ablations.md` | NOT STARTED |
| 3 | `docs/superpowers/plans/2026-06-04-block2-phaseA-sweep.md` | NOT STARTED |
| 4 | `docs/superpowers/plans/2026-06-04-block3-mot-metrics.md` (Tasks 1–3 CPU-only: interleave with chip blocks) | NOT STARTED |

Stretch (only after 1–4 done): overlap×grid re-sweep under fixed recovery;
MOT scorecard at budgets {600, 1000, 3000}; extend ReID ablation to more 0026 fovs if
GT becomes available.

## Hard rules

- Branch `tiling-benchmark` ONLY. **NEVER push. NEVER `git checkout` away dirty files.**
- NEVER touch: `drone_follow/pipeline_adapter/reid_manager.py` (pre-existing dirty,
  prod), `hailo-apps` / `sim/PX4-Autopilot` submodule pointers, `.venv_gt/`.
- Commits: reviewed code + tests + docs only. Result JSONs / frames dumps / cache DBs
  stay UNTRACKED. Full `dynamic_tiling` suite green before every commit
  (`source hailo-apps/venv_hailo_apps/bin/activate && python -m pytest dynamic_tiling/tests/ -q`;
  suite was 119 passing at kickoff).
- Chip (Hailo PCIe) is EXCLUSIVE: one chip-using process at a time. All trial runs go
  through `--cache dynamic_tiling/runs/cache/<clip>__yolov8n4c_vga.sqlite3`.
- ReID: person crops only, never full frame, never vehicles (enforced in
  `reid_embedder.py`; do not weaken).
- Telegram (`reply` tool): message Gilad on each BLOCK completion (short metric
  summary) and on hard blocks needing a human. Nothing else.

## Quota / session-limit protocol (user-mandated)

Per `handling-anthropic-session-limits` skill: if a subagent or this session hits
"session limit · resets HH:MM", then (1) salvage partial work into commits-safe state,
(2) update this file (set `## Incident` with the reset time as an ABSOLUTE timestamp),
(3) arrange a relaunch after the reset — preferred: have the still-alive supervisor
schedule it (CronCreate / `claude -p` cron job that opens a new session with the prompt:
"Resume the weekend run: read docs/superpowers/weekend-manager-state.md and continue"),
(4) exit cleanly. On any fresh start: read this file FIRST, then the spec, then the
in-flight plan's checkboxes.

## Key context (verified at kickoff, 2026-06-04 evening)

- Baseline + root cause: `dynamic_tiling/runs/baseline_0025/BASELINE.md` — recovery is
  the bottleneck; frozen-anchor gate proven. Known failing cells for Block 1 gate:
  walker fov50-ov0, fov60-ov25, fov70-ov25.
- GT: `dynamic_tiling/runs/gt_verify_0025_fov{50,60,70}/gt_tracks.verified.json` (locked),
  `dynamic_tiling/runs/gt_verify_0026_fov50/gt_tracks.verified.json` (verified, 26 tracks).
- Videos: `/home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov{50,60,70}.mp4`,
  `DJI_20260528155239_0026_D_prepared__fov50.mp4`.
- Caches (warm for 0025): `dynamic_tiling/runs/cache/0025_fov{50,60,70}__yolov8n4c_vga.sqlite3`.
- ReID HEFs: `/usr/local/hailo/resources/models/hailo10h/repvgg_a0_person_reid_512.hef`
  (use this), `osnet_x1_0.hef`. Extractor: `reid_analysis/reid_embedding_extractor.py`.
- Research survey: `docs/research/2026-06-04-reid-inference-reduction-survey.md`.
- HEAD at kickoff: branch `tiling-benchmark`, last commits `4beaabe` (spec) ←
  `684cee5` (baseline docs) ← `9e413e4` (trial harness feats).

## Status log

- 2026-06-04 (kickoff): plans written and committed; manager not yet started.
- 2026-06-04 (late): autonomous-project-manager subagent launched but its runtime
  lacked Agent/CronCreate/telegram tools; it verified the full setup (119 tests
  green, caches/videos/GT present, git state clean vs rules) and stopped safely
  WITHOUT spending chip/quota. **The MAIN session took over as manager** — it has
  the full toolset and dispatches implementer/reviewer subagents directly.
  Orchestration: one background implementer subagent per plan task → two-stage
  review (spec compliance, code quality) → commit → next task. This file remains
  the durable memory between turns.
- 2026-06-04 (late): Block 1 started — Task 1 implementer dispatched.
- 2026-06-04 (late): **B1 Task 1 DONE** — commit `33cbc69` (velocity anchor), 121
  tests green. Review: PASS all points; one optional clarity nit (explicit
  `_bt_track_id is not None` in the advance guard) folded into Task 2's brief.
- 2026-06-04 (late): B1 Task 2 implementer dispatched (distance gate + the nit).
- 2026-06-04 (late): **B1 Task 2 DONE** — commit `c79d8a2` (distance gate), 124 tests.
  Test loss-loops bumped 40→100 frames: ByteTracker's `remove_duplicate_stracks`
  (byte_tracker.py:306, suspected inverted `pdist < 0.15`) kills NEW tracks while the
  lost original lingers in the buffer (track_buffer=90) — a structural ~90-frame floor
  on new-id recovery that explains the baseline's bimodal recovery times. OUT OF SCOPE
  for B1 (vendored tracker); **candidate stretch ticket**: fix dedup or sweep
  track_buffer. Task 4 must check whether time-to-recover clusters at ~track_buffer.
- 2026-06-04 (late): B1 Task 2 review PASS. One LOW accepted-as-is (radius uses
  frames_since_seen pre-increment → one growth-step conservative; negligible).
  **PART C CONFIRMED the dedup bug**: byte_tracker.py:305-306 feeds RAW IoU from
  iou_batch into `pdist < 0.15` (upstream ByteTrack used 1−IoU distance) → new
  non-overlapping tracks get pruned as "duplicates" while the lost track is buffered.
  PROMOTED to **B1 Task 2b** (in Block-1 scope: it is a re-acquisition killer).
  ⚠ byte_tracker.py is PROD-SHARED with the flying app — fix must be flagged to
  Gilad in the Block-1 telegram summary.
- 2026-06-04 (late): B1 Task 2b implementer dispatched (dedup threshold fix + tests).
- 2026-06-04 (late): **B1 Task 2b DONE** — commit `e01fc4f` (1-line: `1.0 - iou_batch`),
  pre-fix failures proved inversion both ways; 30-frame-loss re-lock now works.
  ⚠ PROD-SHARED tracker change — needs Gilad's sim/flight sanity before next flight.
  **B3 Task 1 DONE** — commit `607884d` (mot_metrics.py), 132 tests green total.
  Pre-existing UNRELATED failures noted in tests/test_controller.py
  (TestDistanceForward x2) — fail with and without our changes; not ours, not fixed.
- 2026-06-05 (early): **B1 Task 3 DONE** — commit `6396b2d` (CLI plumb), 133 tests.
  B3 Task 1 review PASS (2 LOW folded into B3 Task 2's brief). B1 Task 4 validation
  grid RUNNING on chip. B3 Task 2 implementer RUNNING (multi_traj + --dump-mot).
- 2026-06-05 (early): **B3 Tasks 2+3 DONE** — `91252db` (multi_traj + --dump-mot +
  mot_metrics review fixes), `bcbbd10` (run_mot_eval CLI). 143 tests green.
  Block 3 CPU-complete; Task 4 scorecard queued behind Block 2 chip time.
  B-R Task 1 (embedding cache table, CPU) dispatched while B1.4 grid runs on chip.
- 2026-06-05 (early): **B1 Task 2b STRICT REVIEW: PASS — "safe to fly"** verdict.
  Prod-risk summary for Gilad: pre-fix dedup imposed a hard ~3 s (track_buffer/fps)
  re-acquisition floor by killing every new track spawned away from a buffered lost
  track; fix restores upstream 1−IoU semantics; dedup now fires only at IoU>0.85
  (same physical bbox) where pruning is correct; crossing people (IoU 0.5–0.7)
  unaffected; age-based winner favors the locked target. Recommend a sim/flight
  sanity pass before next real flight. Reviewer env note: their shell lacked cv2
  (didn't source the venv) — implementer counts (133) are authoritative.

- 2026-06-05: **Block R Tasks 1–4 DONE** — `28335d8` (embeddings cache table),
  `a0a35be` (ReidEmbedder; real extractor class is HailoReIDExtractor), `78f93bf`
  (ReidGallery; prod-parity deviations documented in its report), `91f1725`
  (policies + ReidAssist + adopt_overlapping). 153 dynamic + 227 hailo_tiling tests.
  **Consolidated R1–R4 review: person-crop-only constraint AIRTIGHT (verified
  through every caller); one HIGH** — reid_embedder close() vs HailoReIDExtractor
  release() (AttributeError on chip cleanup) → being fixed in R5; LOW: POLICIES
  holds classes, CLI must instantiate (R5 brief covers it).
- 2026-06-05: **B-R Task 5 DONE** — `956e9b7` (trials CLI wiring + P5 histogram +
  close/release HIGH fix), 159 tests. Chip smoke DEFERRED (grid busy) — run when
  chip free: run_trials w/ --reid-policy generous --reacq-motion velocity
  --reacq-radius-growth 0.002 --reid-cache == --cache on 0025 fov50.
- 2026-06-05: **B-R Task 6 driver DONE** — `13006ac` (run_reid_ablation driver,
  clips×arms, per-trial reid-stat DELTA fix, render_report/--render-only),
  171 tests. Execution = follow-up when chip free. Note: gallery sub-ablation
  "ema" vs "both" not behaviorally distinct (FIFO always retained) — acceptable.
- 2026-06-05: B2 Task 1 (run_sweep driver, CODE ONLY) implementer RUNNING.
  B1.4 validation grid still RUNNING on chip (long pole; fov70 stage observed).
  **CHIP QUEUE once grid done:** (1) B1 gate verdict + telegram, (2) R5 smoke,
  (3) R6 ablation execution, (4) B2 sweep execution, (5) B3.4 MOT scorecard.

- 2026-06-05: **BLOCK 1 GATE PASSED** (validator commit `8e3b4f1`, BASELINE.md
  validation section). Best config: `--reacq-motion frozen --reacq-radius-growth
  0.001`. Walker 0.989/0.967/0.987 on the 3 failing cells, recovery 1.0,
  11/12 trials ttr ≤ 3 frames (max 47; 90-frame floor GONE — dedup fix e01fc4f is
  the main lever; velocity anchor neutral once growth>0, harmful alone at fov60 —
  honest negative for the paper). No regressions; every other cell improved.
- 2026-06-05: **B2 Task 1 driver DONE** — `200400f` (run_sweep coordinate descent +
  frontier), 181 tests. ALL PROGRAM CODE COMPLETE — only chip executions + reports
  remain. Telegram MCP not connected in this session → milestone sent via desktop
  push instead (note: future sessions, launch with --channels telegram for true
  two-way).
- 2026-06-05: R5 chip smoke STARTED (generous policy, fov50, Block-1 best config,
  real repvgg ReID HEF, shared cache DB). Next in queue: R6 ablation execution
  (REMEMBER: pass --reacq-motion frozen --reacq-radius-growth 0.001, NOT the
  velocity/0.002 defaults), then B2 sweep, then B3.4 scorecard.

- 2026-06-05: **R5 chip smoke PASS** (after hotfix `cdacb76`: store.open now
  re-applies schema idempotently — pre-existing cache DBs lacked the embeddings
  table; caught only on real warm-cache run). Generous ReID on fov50: walker
  0.989 / bg 0.996 coverage, recovery 1.0, ttr ~1.5 frames, 2058 real chip embeds
  (embedding cache now warming). One operator error en route: a chained command
  ran without sourcing the venv (ModuleNotFoundError pyulog) — ALWAYS source
  hailo-apps/venv_hailo_apps/bin/activate in every chip command.
- 2026-06-05: **R6 ablation execution STARTED** (4 clips × 6 arms + gallery
  sub-ablation, frozen/0.001, sequential chip; hours-long; 0026 cache cold).
  After it: B2 sweep execution, then B3.4 MOT scorecard.
- 2026-06-04 20:08: R6 main pass RUNNING as detached PID 952267 (executor subagent
  exited early but its backgrounded process survived; log
  dynamic_tiling/runs/reid_ablation/run.log, "MAIN_EXIT=<code>" line marks end).
  MANAGER OWNS THE FOLLOW-THROUGH now: when main pass ends → run
  `python -m dynamic_tiling.run_reid_ablation --reacq-motion frozen
  --reacq-radius-growth 0.001 --gallery-sub-ablation` (venv!), then --render-only
  → write + commit dynamic_tiling/runs/REID_ABLATION.md (MD only). Watcher armed.

- 2026-06-04/05 evening: **R6 attempt #1 crashed at 19/24 cells** —
  HAILO_OUT_OF_PHYSICAL_DEVICES(74): detection VDevice (no group_id, created in
  tiling_benchmark/probe_phantom_hef.py HefHandle.open) + ReID VDevice (SHARED
  group) could not coexist on a COLD cache (0026). Warm-cache 0025 cells masked it.
  **FIXED `f23f71e`** (make_shared_vdevice_params in hailo_tiling/backends/hef.py +
  optional vdevice_params on HefHandle.open; "SHARED" literal duplicated by design,
  source: hailo_apps defines). Validated on the exact crash cell; 411 tests green.
- 2026-06-05: **KEY SCIENCE from attempt #1**: 0025 is SATURATED by Block-1 fixes
  (all arms incl. P0 tie at ~0.988 cov, recovery 1.0) → 0025 only measures COST
  (selective arms ~3% dets embedded vs generous 81% = ~28x cheaper, equal quality).
  0026:fov50 is the discriminating clip (P0 cov 0.647, rec 0.784, 11 tracks);
  early signal: generous on 0026 also ≈0.647 → remaining losses may be
  detection-bound, not association-bound. Report must not overclaim ReID.
- 2026-06-05: **R6 attempt #2 RUNNING** (all caches warm; sub-ablation + render +
  commit of REID_ABLATION.md included; interpretation requirements given).
  Then: B2 sweep execution → B3.4 MOT scorecard.

- 2026-06-06: **R6 COMPLETE + committed `ac6d8c4`** (REID_ABLATION.md w/ honest
  interpretation: all arms tie — Block-1 gates act before ReID can; 0026 residual
  losses detection-bound; default none-on-this-dataset / ambiguity-as-insurance;
  gallery sub-ablation deferred to harder clips). Ablation graphs:
  dynamic_tiling/runs/reid_ablation/ablation_graphs.png (opened for Gilad).
  Cache savings (retroactive lower bound): 57 runs consumed 478,743 tile infs,
  only 83,818 ever hit the chip → ≥82% served from cache ≈ ~145 min chip saved
  vs ~31 min spent. Precise counters being implemented (subagent in flight).
- 2026-06-06: Viewer opened for Gilad: FINAL fov50 walker run (frozen/0.001 +
  ambiguity ReID, tiles+dets+LOCK tags, coverage 0.993) vs OLD failing baseline
  vs GT. Final-config dump: dynamic_tiling/runs/baseline_0025/frames_fov50_final/.
  WATCHER LESSON: pgrep/pkill patterns must use the [b]racket trick — two watchers
  self-matched their own cmdline and hung/killed wrong targets.

## NIGHT PLAN (2026-06-06 → 07, user-approved "continue the plan; if idle, more
videos → GT → tests")

1. B2 sweep RUNNING (PID 1491522, log dynamic_tiling/runs/phase_a/sweep.log,
   2-pass coordinate descent, frozen/0.001, reid none). Watcher armed.
2. After sweep: --budget-frontier at winner → PHASE_A.md → commit (manager does
   render+commit directly; execution subagents NOT used for long chip runs —
   they keep dying by backgrounding+pausing).
3. B3.4 MOT scorecard: run_dynamic --multi-target on 0026-fov50 at the sweep
   winner config + static-cache comparison via run_mot_eval --from-static-cache
   helper (NOTE: helper not yet implemented — Block 3 Task 4 step 2 includes it;
   dispatch implementer subagent for the helper CODE, manager runs the chip part).
4. Cache-counter feature lands (subagent in flight) → include precise savings
   line in PHASE_A.md.
5. STRETCH (if time): 0027 GT — fov50 has verified-but-unlocked GT already
   (gt_verify_0027_fov50/gt_tracks.verified.json); run the dense→tracks→dedup→
   auto-merge pipeline for fov60/70 (NO human-review-required locking overnight;
   produce review queues + render review PNGs for Gilad's morning), then run
   trials + reid-ablation rows on 0027:fov50 (its losses may discriminate ReID).
6. Morning deliverables: PHASE_A.md, MOT_BASELINE.md, updated graphs, 0027
   artifacts + review queue, final state-file summary, Telegram/push summary.

- 2026-06-06 night: **SWEEP DONE** — winner `6x4 @ 1fps, ov 0.15`: coverage 0.989
  @ **1.59 tiles/frame** (old default 8x6@2fps: same coverage at 2.4 t/f; 12x9@1fps
  4.1 t/f; 4x3 is the cliff at 0.955). zoom/model_h are no-ops at the winner.
  63 config-JSONs in runs/phase_a/. **Budget frontier RUNNING** at winner
  (watcher b2fxllwt6). Cache counters landed `1e6db0a` (426 tests; warm-run
  validation pending chip-free window). MOT static-cache helper landed `26812e1`
  (434 tests). ⚠ EXECUTION TRAP for B3.4: run_mot_eval --from-static-cache CLI
  uses ppv=0 but CachedHefBackend writes ppv=1 — pass/patch ppv before the
  scorecard or the replay is silently empty.
- NEXT after frontier: PHASE_A.md (incl. measured cache line) render+commit by
  manager → B3.4 scorecard (dynamic @ winner cfg w/ --dump-mot on 0026 + static
  replay w/ correct ppv → run_mot_eval → MOT_BASELINE.md) → 0027 stretch
  (GT auto-stages fov60/70 + review queue PNGs; trials+reid rows on 0027:fov50).

- 2026-06-06/07 night: PHASE_A.md committed `a22c8c6` (winner 6x4@1fps/ov0.15:
  0.989 cov @ 1.59 t/f; frontier saturates at budget 600 → 1.41 t/f; collapse at
  300). MOT scorecard saga: static replay GOOD (IDF1 0.341, FN 203, 8.87 t/f,
  ppv=1!). Dynamic dump was garbage TWICE: (1) `591f557` run_dynamic CLI was on
  stale 0-indexed classes (tracker for 'unlabeled', gt_cls=0 → selection never
  fired); (2) `eae88c1` backend ALSO lacked class_offset=1 → tracked vehicles as
  persons (debugger's monkeypatched test couldn't see it). Both fixed + regression
  tests (`d790406` stub fix). Detector caveat for the report: VGA model mislabels
  many 0026 vehicles as 'person' — hits both systems equally; say it in
  MOT_BASELINE.md. Final dynamic rerun ON CHIP (watcher biala2cez).
  0027 GT pipeline agent running (CPU). OPS LESSONS: `pytest | tail` masks exit
  codes (use pipefail or split); watchers/pkill need [b]racket patterns; chained
  commands EACH need the venv sourced.

## FINAL SUMMARY (2026-06-07 early morning — weekend run COMPLETE)

All four blocks + stretch delivered. ~30 commits on `tiling-benchmark` (nothing
pushed), 435+227 tests green at HEAD.

**Headline results**
- Recovery SOLVED: walker coverage 0.054→0.989 (fov50); the levers were the
  ByteTracker dedup fix `e01fc4f` (⚠ PROD-SHARED — sim sanity before flying)
  + distance-growth gate (`frozen/0.001`); velocity anchor = honest negative.
- Phase A winner: **6x4 grid @1fps, ov 0.15 → 0.989 coverage @ 1.59 tiles/frame**;
  frontier saturates at budget 600 (1.41 t/f), collapses at 300. PHASE_A.md.
- ReID: zero quality delta on this dataset (Block-1 gates act first; 0026 losses
  detection-bound); gated policies = ~3% embeds vs 81% generous. Default: none /
  ambiguity-as-insurance. REID_ABLATION.md + ablation_graphs.png.
- MOT: dense static slightly ahead (IDF1 0.341 vs 0.249) but dynamic within
  striking distance at 65% of the tiles; shared detector FP flood (vehicles→
  "person") dominates MOTA. MOT_BASELINE.md.
- Cache: measured — warm full-clip run = 10,504 lookups, 100% hits, 0 chip s,
  ~231 s saved/run; weekend ≥82% of ~479k tile inferences cache-served.
- 0027 GT advanced: fov60 (13 trk/5 cases) + fov70 (12 trk/3 cases) queues +
  PNGs ready; runbook README: runs/gt_verify_0027_README_morning.md.

**Morning review list for Gilad**
1. Sim/flight sanity for `e01fc4f` (prod tracker change) before next flight.
2. Skim PHASE_A.md / REID_ABLATION.md / MOT_BASELINE.md conclusions.
3. Click through the 0027 review queues (commands in the morning README);
   then corrections + lock per docs/gt-generation-guide.md.
4. Follow-ups filed in the reports: unify remaining class-convention stragglers,
   --ppv flag for static replay, person-FP floor / model upgrade for 0026-class
   scenes, re-score MOT at the Phase-A winner config, harder-clip ReID round.

## Incident

(none — no quota events all weekend)
