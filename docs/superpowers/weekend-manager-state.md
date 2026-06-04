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
| 1 | `docs/superpowers/plans/2026-06-04-block1-reacq-motion-fixes.md` | NOT STARTED |
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

## Incident

(none)
