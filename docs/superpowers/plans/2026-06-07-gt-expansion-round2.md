# GT Expansion Round 2 — more clips, ReID/MOT-targeted tests

> **For agentic workers:** manager-driven program (chip jobs run by the manager
> directly, per PM lesson F; subagents do CPU stages + code). Checkboxes track
> progress. State updates appended to the bottom of THIS file.

**Goal (Gilad, 2026-06-07):** create GT for more clips to test and finetune the
dynamic-tiling code. Not all clips get 3 FOVs — close clips use the default
(native) FOV. Then run targeted tests.

**Clip roles:**
| clip | prep | GT scope | purpose |
|---|---|---|---|
| 0012 (31 s, portrait main cam) | rotate → `_rotated.MP4` (0010 precedent) | default FOV only | **ReID testing** (close) |
| 0013 (45 s, portrait main cam) | rotate | default FOV only | **ReID testing** (close) |
| 0019 (18 s, landscape 4K) | none | default FOV, **person+vehicle classes** | **multi-target MOT** (multiple cars) |
| 0026 | prepared fovs exist; fov50 GT verified | stretch: dense+auto stages fov60/70 | "harder" yaw clip — harden tracking |
| 0027 | GT exists | fov50 verified → usable NOW; fov60/70 await Gilad's review clicks | tests |
| 0029 (21 s, portrait main cam) | prepared fovs exist | fov50/60/70 | full 3-FOV cells |

**Hard rules (carried over):** branch `tiling-benchmark`; never push; chip
exclusive — one chip process at a time, manager-owned, background + watcher
(`[b]racket` pgrep patterns); venv sourced in EVERY command; result
JSONs/frames/caches stay untracked (only final report MDs + reviewed code
commit); GT artifacts stay untracked until human-approved; ReID = person crops
only; never touch reid_manager.py / submodules / .venv_gt.

**Pipeline per clip-fov** (docs/gt-generation-guide.md, new tiling_lab paths):
dense 12x9 pass (chip; tiling_benchmark/run_pxt_bench.py — NOTE: decode caches
regrow under tiling_benchmark/pxt_runs/.cache, ~5-7 GB per clip; DELETE each
clip's .bin after its dense pass completes to cap disk) → `tiling_lab.gt`
tracks/dedup/auto-merge stages → review queue + rendered PNGs for Gilad →
HUMAN verdicts → corrections → lock. Overnight-able up to the review queue.

---

## Phase A — prep (CPU)
- [ ] A1. ffmpeg rotate-transcode 0012, 0013 → `*_rotated.MP4` (same recipe as 0010).
- [ ] A2. Verify durations/fps; probe one frame each visually (optional).

## Phase B — dense passes (chip, strictly serial; manager-owned)
Queue order (ReID + MOT clips first per Gilad's emphasis):
- [ ] B1. 0012 default (rotated)
- [ ] B2. 0013 default (rotated)
- [ ] B3. 0019 (person+vehicle — confirm gt tools handle class 2 tracks; report if person-only assumptions exist)
- [ ] B4. 0029 fov50
- [ ] B5. 0029 fov60
- [ ] B6. 0029 fov70
- [ ] B7. (stretch) 0026 fov60, fov70
After each pass: delete that clip's decode .bin; kick the matching Phase C agent.

## Phase C — GT auto stages per clip (CPU subagents, parallel-safe, no commits)
- [ ] C1-C6. per clip-fov: gt_tracks → dedup → auto-merge → review_queue.json + review PNGs under `tiling_lab/runs/gt_verify_<clip>_<fov>/`
- [ ] C7. Morning README runbook: `tiling_lab/runs/gt_verify_round2_README.md` (click-through commands per clip, like the 0027 one)

## Phase D — tests (chip, after GT; verified-but-unlocked acceptable, flag it)
- [ ] D1. 0027 fov50 NOW (GT already verified): trials at Block-1 best (frozen/0.001) + Phase-A winner (6x4@1fps ov0.15), cached.
- [ ] D2. ReID round on 0012/0013 once verified: run_reid_ablation arms (none/generous/ambiguity at least) — the weekend's "harder-clip ReID round" follow-up; close clips should finally discriminate.
- [ ] D3. MOT scorecard on 0019 once verified: run_dynamic --multi-target (classes 1,2) + static dense replay → run_mot_eval; compare vs 0026 numbers.
- [ ] D4. 0026 fov50 trials at Phase-A winner (harder-yaw single-target cell).
- [ ] D5. 0029 cells after Gilad locks.
- [ ] D6. Results round-up MD (`tiling_lab/runs/ROUND2.md`): tables + finetune recommendations (which knob defaults move). Commit MD only.

## Human gates (Gilad)
1. 0027 fov60/70 review queues — ALREADY waiting (runbook: tiling_lab/runs/gt_verify_0027_README_morning.md).
2. New review queues from Phase C as they land.
3. Lock decisions per docs/gt-generation-guide.md.

---

## State log
- 2026-06-07: plan written; Phase A started.
