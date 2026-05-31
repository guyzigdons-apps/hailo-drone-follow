# Overnight Manager State — 2026-05-28

I am the **manager agent** for overnight work on the `hailo_tiling` library. I do not write code myself — I dispatch subagents and monitor their progress. The user is asleep and has authorised autonomous execution.

## Active work

| Track | Owner | Status | Notes |
|-------|-------|--------|-------|
| Plan 2 — write doc | Plan-subagent #1 (in flight) | dispatched | Telemetry + new modifiers + backends ABC (spec phases 3/4/5) |
| Plan 3 video prep — FOV crops | general-purpose subagent #1 (background) | dispatched | 7 clips × 3 FOV variants; ffmpeg-only; slow + CRF 18 |
| Plan 2 — implementation | (queued; depends on doc) | blocked | Dispatch implementer-1 once doc is committed |

## Conventions for this overnight session

- Tasks #31/#32/#33 in TaskList track progress.
- Branch: `tiling-benchmark` (HEAD = `cea704d` at session start).
- Subagent-driven-development workflow: implementer → spec reviewer → quality reviewer per task. Commit per task.
- Never dispatch two implementation subagents in parallel — file-conflict risk.
- Video work is safe to run in parallel with code work (touches `/home/giladn/Videos/Drone/Training/`).
- Cron fires hourly at minute :17 with prompt "Overnight status review."

## Status-review checklist (for each hourly cron fire)

1. `git log --oneline -20` — what has landed since last review?
2. `TaskList` — which entries advanced?
3. If Plan 2 doc committed and no implementer is in flight → dispatch implementer for Plan 2 Task 1.
4. If implementer reported DONE/DONE_WITH_CONCERNS → dispatch spec reviewer.
5. If spec reviewer approved → dispatch quality reviewer.
6. If quality reviewer approved → mark task complete, dispatch next implementer.
7. Check `/home/giladn/Videos/Drone/Training/*__fov*.mp4` — count output files, verify sizes look reasonable.
8. Update this file with any state change.

## Notes / blockers (append entries below)

- 2026-05-28: initial state written; cron + first dispatches set up.
- 2026-05-28 23:36: video agent first attempt aborted on 100GB disk floor I set in prompt; relaxed to 10GB and re-dispatched. Disk verified safe: 25GB free, expected outputs ~3-8GB total.
- 2026-05-28 23:36: video agent forked ffmpeg batch to background and returned. 4/21 outputs landed (clip 0029 all three + clip 0026 fov70 in progress). ~3 min/variant observed → ~50-85 min remaining for the 17 outstanding encodes. Manifest at `/home/giladn/Videos/Drone/Training/fov_variants_manifest.json` growing incrementally.
- 2026-05-28 23:55: Plan 2 doc agent returned full plan content (read-only agent couldn't write). Extracted via `/tmp/extract_plan2.py` (transcript decode + html-unescape), saved 3069-line plan to `docs/superpowers/plans/2026-05-28-telemetry-modifiers-backends.md`, INDEX flipped to `in flight`. Commit `833cf7a`. Plan covers 15 tasks: telemetry providers (1-5), scheduler-thread widening (6), AltitudeZoom + ASAHI modifiers (7-8), backends ABC + HefBackend lift (9-10), aggregator NMS/BoundaryStrip/Memory/composer (11-14), public-API + INDEX (15).
- 2026-05-28 23:56: dispatched Plan 2 Task 1 implementer (TelemetrySnapshot dataclass + NULL_SNAPSHOT sentinel) in background.
- 2026-05-29 ~00:00: Plan 2 Task 1 complete — commit `8261069`, 82/82 tests. Spec review PASS; quality review APPROVED (3 nits, no Important findings; nits intentionally not back-edited to match Plan 1 stylistic mix).
- 2026-05-29 ~00:01: dispatched Plan 2 Task 2 implementer (TelemetryProvider ABC contract test — no new impl, just contract-pinning tests).
- 2026-05-29 ~01:17 (cron fire): 13 of 15 Plan 2 tasks complete. Commits in order: `c46fa75` (T2), `5ae50bc` (T3), `0223459` (T4), `2ced7d9` (T5), `e935281` (T5 nit cleanup), `00f38bd` (T6 — scheduler widening, parity preserved), `398c65b` (T7 AltitudeZoom), `b24c1f0` (T8 ASAHI), `04aeb6c` (T9 backend ABC), `91bafd7` (T10 HefBackend lift), `d536a9f` (T10 wrapper fix), `91e94db` (T11 NMS), `30509e4` (T12 BoundaryStripFilter), `5ef41e0` (T13 DetectionMemory). Test count: 134 passed + 1 skipped. Quality reviewer for T13 currently in flight; T14 (Aggregator composer) and T15 (public API + INDEX flip) remain.
- 2026-05-29 ~01:17: video — 11 of 21 FOV outputs done, ffmpeg mid-encode on clip 0025 fov60. Disk 22GB free (started at 25GB, ~3GB consumed by 11 outputs). Manifest growing as expected; no aborts.
- 2026-05-29 ~02:30: **Plan 2 fully landed.** 15 implementation tasks + 3 nit-cleanup/fix commits = 18 commits since session start. Final commit `2e9909f` flips INDEX to `done`. Suite: 139 passed + 1 skipped. Plan 2 spec phases 3/4/5 complete: TelemetryProvider ABC + Static/Recorded/Mavsdk; AltitudeZoomModifier + AdaptiveSliceSizingModifier (ASAHI); InferenceBackend ABC + HefBackend lift (with single-crop shim wrapper for legacy callers); aggregator skeleton (NMS, BoundaryStripFilter, DetectionMemory ABC + NoOp, top-level Aggregator). Byte-parity with Plan 1 preserved throughout. Task #33 marked complete in TaskList.
- 2026-05-29 ~02:30: video — 13 of 21 FOV outputs done, ffmpeg encoding clip 0033 fov70. Disk 22GB free. Roughly half-way through; user's earlier estimate (~3.5h remaining at clip 2 completion) suggests video work finishes around 04:00–04:30. No action needed; let it run.
- 2026-05-29 ~03:30: **Plan 3 fully landed.** 7 implementation tasks (commits `8168c39`, `48bf1ea`, `41d9ca6`, `f986a45`, `5603fe5`, `8e94af1`, `a5c6801`) + INDEX flip (`ab67415`). 164 passed + 1 skipped. Plan 3 codifies `prepare_video.py --emit-fov` with FOV-70/60/50 crop math, ffmpeg argv builder, atomic JSON manifest writer (sha256 + os.replace), runtime wiring, and an `import_overnight_manifest(verify_sha=True)` validator that successfully validates the live overnight-agent artifacts (16 records sha-verified). README updated with reproducibility recipe. Task #34 complete.
- 2026-05-29 ~03:30: video — 17 of 21 FOV outputs done, ffmpeg on clip 0027 fov60. Disk 20GB free (started 25, ~5GB consumed). ~4 outputs to go.
- 2026-05-29 ~03:30: dispatching Plan 4 (cache schema + Python cache layer) doc-writer.
- 2026-05-29 ~04:17 (cron fire): Plan 4 doc-writer still in flight (Plan agent). Last commit `ab67415` (Plan 3 INDEX flip). Video: 18 of 21 FOV outputs landed (manifest at 17 records — final manifest write follows the ffmpeg completion of clip 0027 fov50, currently encoding). Disk 20GB, healthy. No new code work dispatched this cron tick because no implementer is queued and Plan 4 doc must land first.
- 2026-05-29 ~06:30: **Plan 4 fully landed.** 10 implementation tasks + INDEX flip. Commits `b9e7940` (doc), `ea4df3a` (schema), `0083871` (hashing), `96254d5` (store open+meta), `f41c71d` (store CRUD), `3bc3f91` (WAL concurrency), `8e324d0` (CachingBackend; 2 spec-data fixes), `82ff4b5` (ReplayBackend + CacheMissError), `37d2303` (warm-cache CLI), `66bd3fa` (pyproject + public API), `d57358a` (INDEX flip). 209 passed + 1 skipped. `hailo-tiling-warm-cache --help` works after editable install. INDEX now shows Plans 1, 2, 3, 4 all done.
- 2026-05-29 ~06:30: video — 21 of 21 FOV outputs landed (final ffmpeg on clip 0034 fov50 writing the file now). Disk 19GB free. Manifest will get the last record once ffmpeg finishes.
- 2026-05-29 ~06:30: **Stopping autonomous work here.** Plans 5-9 cover GStreamer plugins (C++ in submodule), GstCropperBackend, harness, telemetry import, drone-follow migration, paper artifacts. Plan 5 is the natural next step but touches hailo-apps-core submodule + GStreamer + C++ — substantially riskier than pure-Python Plans 1-4. Leaving for the user's morning review. Branch: `tiling-benchmark` clean at `d57358a`. Total overnight: ~50 commits, ~209 tests passing, 4 plans landed.
- 2026-05-29 ~07:17 (cron fire): final verification. Branch clean at `192494f`. **Video work complete:** 21 / 21 FOV outputs on disk, 21 records in manifest, no ffmpeg running. Disk 19 GB. Test suite 209/1 skipped. Plans 1-4 done in INDEX. Pre-session uncommitted changes (reid_manager.py, hailo-apps submodule pointer, sim/PX4-Autopilot pointer, .claude scheduled_tasks lock) untouched — they were there at session start.
- 2026-05-29 ~08:17 (cron fire): no drift. Branch at `192494f`. Video 21/21 confirmed (manifest + on-disk both 21). No ffmpeg running. Marked TaskList #32 (video work) complete. Manager idle — no remaining queued work. Plans 5-9 deliberately deferred for user-aware session.
- 2026-05-29 ~09:17 (cron fire): still idle, no drift. HEAD `192494f`, 21 FOV outputs, no ffmpeg.
- 2026-05-29 ~10:17 (cron fire): still idle. HEAD `192494f`, 21 FOV outputs, no ffmpeg.
- 2026-05-29 ~11:17 (cron fire): still idle. HEAD `192494f`, 21 FOV outputs, no ffmpeg. All overnight TaskList roots (#31-#35) completed.
- 2026-05-29 ~12:17 (cron fire): still idle.

## Weekend run — kicked off 2026-05-29 afternoon

User instructions for the weekend (no checkpoints until Monday):
- New GStreamer plugins land in **this repo** (`gst-hailo-cache/` next to `hailo_tiling/`), not in the `hailo-apps` submodule. Submodule patches allowed if strictly needed, with subagent approval — never push the submodule.
- **One Hailo chip → chip-using subagents are strictly serial.** Pure-Python / MockBackend / doc-writer / review work runs freely in parallel with each other and with the one chip-using job.
- Full autonomy, no waiting.
- Pre-flight bit-exact E2E validator is the GATE before Plan 5 implementers run. After every Plan 5 implementer commit that touches cache machinery, rerun the bit-exact pytest. If it diverges from the baseline JSON, **BLOCK**.

Weekend dispatch order (DAG):

```
[#36 pre-flight bit-exact E2E (CHIP)]  ─┐
[#37 Plan 5 doc (no chip)] ─┐           ├──> [#38 Plan 5 impl (CHIP, serial)]
[#39 Plan 7 doc (no chip)] ─┴──────────────> [#40 Plan 7 impl (no chip, parallel with #38)]
                                            └─> [#41 Plan 6 stretch — only if Plan 5 fully landed]
```

Hard guardrails (cron tick #2 onwards):
- Never start a dependent task unless upstream is fully reviewed AND committed.
- Test suite must stay ≥ 209 passing.
- No uncommitted changes outside `tiling-benchmark` branch.
- Submodule HEAD must not move unexpectedly.
- 3 consecutive review failures on the same task = hard stop, append diagnosis.

Cron: hourly at :17, prompt now points to weekend-mode status review (job `5b92f123`). Old session-only job `ef941e44` cancelled.

## Weekend progress log

- 2026-05-29 ~12:30: dispatched 3 parallel subagents: pre-flight bit-exact E2E (chip), Plan 5 doc, Plan 7 doc.
- 2026-05-29 ~12:35: Plan 5 doc committed at `83ac5d3` — 15 tasks, 3 [CHIP] strictly serial (T7, T11, T14), Task 12 chooses wrapper-vs-submodule for `bypass-on-cache-hit` based on a small experiment.
- 2026-05-29 ~12:40: Plan 7 doc committed at `92ae2f2` — 10 tasks all [no-chip], adapter pattern (no new provider), ffmpeg+ASS visualizer.
- 2026-05-29 ~12:45: pre-flight validator committed at `ae44880` + `53c0d62` — 3 modes (HefBackend / Caching / Replay) bit-exact verified on DJI_0029 fov50 clip. Floor-quantise + CacheMissError checks pass. Chip not invoked on Mode B pass 2. **Test baseline corrected: actual is 184 passed + 1 skipped at `192494f`, NOT the 209 I'd been carrying. Updated guardrail floor to 184.**
- 2026-05-29 ~12:55: spec review for pre-flight validator → PASS. Quality review → REWORK (2 IMPORTANT findings: latent generator-exhaustion in `_CountedHefBackend.infer`, dead `frames[0] if frames else None` defensive branch). Fix-implementer dispatched.
- 2026-05-29 ~13:00: Plan 5 Task 1 (gst scaffold) landed at `be5b7d0` — DONE_WITH_CONCERNS. Sudo on the system-install step requires interactive password (no NOPASSWD configured for `giladn`); the plugin builds + loads cleanly via `GST_PLUGIN_PATH=gst-hailo-cache/build/src` which is sufficient for all our test pipelines. System install can wait for a user-attended session. All 184 + 3 tests still pass.
- 2026-05-29 ~13:00: Plan 7 Task 1 (pyulog + scaffold) landed at `3b8c977`. Console scripts registered. 184 + 3 tests still pass. Spec + quality review dispatched.
- 2026-05-29 ~13:15: Cache E2E fix at `1e98779` quality re-review APPROVED. Task #36 closed.
- 2026-05-29 ~13:25: Plan 5 Task 1 (`be5b7d0`) — spec PASS, quality APPROVED with 1 NIT (meson_version floor, rolled into T2).
- 2026-05-29 ~13:30: Plan 7 Task 1 (`3b8c977`) — spec PASS, quality APPROVED with 1 NIT (unused `import sys`, deferred to Task 9 hardening).
- 2026-05-29 ~13:35: Plan 7 Task 2 (`e9acb05`) — ULG parser + tiny.ulg (166 KB SITL log). 3 deviations from plan, all expansions (pyulog API drift, modern PX4 topic schema). 6 tests. Spec PASS, quality APPROVED with 3 NITs (deferred to Task 9).
- 2026-05-29 ~14:00: Plan 5 Task 2 (`b3d834f`) — C++ tile_cache_db + 5 gtest cases + cross-language round-trip pytest. System libgtest used (no wrap). Spec PASS, quality APPROVED with 2 missing-include NITs (rolled into T3) + 1 const-correctness NIT deferred.
- 2026-05-29 ~14:15: Plan 7 Task 3 (`6722021`) — DJI SRT parser + 8.7 KB tiny.srt (anonymised lat/lon ×0.001). 8 tests (6 plan + 2 bonus helper tests). Compound-bracket post-process + ISO `T` fallback for 3.10 compat. Spec PASS, quality APPROVED no findings.
- 2026-05-29 ~14:30: Plan 5 Task 3 (`54b9827`) — cache_keys (canonicalize_crop + frame_id_from_buffer) + 11 C++ tests + 100-input Python parity. Rolled-in T2 NIT (missing `<cstdio>`, `<cctype>`). Spec PASS, quality APPROVED with 1 NIT (negative `q` doc, deferred). Suite at 206 + 3.
- 2026-05-29 ~14:45: Plan 7 Task 4 (`f931db2`) — `align_to_video` with 3 strategies + ffprobe. SRT detector via `_geo._agl_source`. Spec PASS, quality REWORK — silent tz strip is IMPORTANT. Fix at `dfcdc82` (log.warning + .Srt suffix + nan/inf reject + ffprobe error-path test). Re-review APPROVED. Suite at 207 + 3.
- 2026-05-29 ~15:17 (cron tick): no new commits since dfcdc82. 3 implementers in flight in parallel: Plan 5 Task 4 (writer skeleton, no-chip, coord on plugin.cpp with T8), Plan 5 Task 8 (reader skeleton, no-chip, coord on plugin.cpp with T4), Plan 7 Task 5 (import CLI). All disjoint or safely coordinated. Suite floor recalibrated to **184** (was misremembered as 209 in cron prompt; not editing cron, just keeping mental model accurate). Plans 5/7 both progressing healthily. Plan 5 Task 7 (first [CHIP] task) is still 4 tasks out — chip remains idle. **No hard stops.**

## **2026-05-29 ~16:30 — CROSS-AGENT INDEX RACE (logged for Monday review)**

Three parallel implementers running concurrently encountered a `.git/index` race:
- Plan 5 Task 6 (full_frame mode, agent `ae25761c`): staged its gst-hailo-cache writer + schema changes.
- Plan 5 Task 10 (microbench, agent `ad8527fa`): staged its bench files.
- Plan 7 Task 9 (hardening, agent `a55068da`): staged its hailo_tiling/ Python changes.

Plan 7 Task 9's report flagged: "my Plan 7 Task 9 files were already `git add`-ed in my session when [Plan 5 Task 6] ran `git commit`; since `.git/index` is shared between simultaneous sessions, my staged files got swept into their commit."

**Resulting state:**
- HEAD = `bb632b8` with commit message describing **Plan 5 Task 6** work but commit content (per `git show --stat`) is entirely **Plan 7 Task 9** Python files.
- The actual Plan 5 Task 6 + Plan 5 Task 10 changes are still in the working tree, STAGED but uncommitted.

**Decision:** Do NOT touch git until Plan 5 Task 10 (`ad8527fa`) lands. Concurrent index ops would compound the problem. Plan 5 Task 6's implementer (`ae25761c`) has not reported back — its work is "orphaned" in the index but recoverable.

**Cleanup plan once T10 lands** (manager will execute, not delegate):
1. `git restore --staged .` to clear the index.
2. `git commit --amend -m "<correct Plan 7 Task 9 message>"` — rewrites `bb632b8` to reflect actual content. (No content change, just message.)
3. Manually sort the unstaged files into Plan 5 Task 6 vs Plan 5 Task 10 buckets by file path (T10 = `bench_lookup_latency.cpp` + `.gitignore`; T6 = everything else).
4. Two clean commits, one per task.
5. Dispatch the two pending reviews against the correct SHAs.

**Test suite still healthy:** 235 + 3 passed at `bb632b8`. No regression. The race produced a labeling problem, not a code problem.

**Future preventive note for skill:** when dispatching multiple no-chip implementers in parallel, they share `.git/index`. The subagent-driven-development skill should warn that implementers MUST `git add` and `git commit` as a single atomic sequence in one Bash invocation, OR each implementer should `git stash`/`git stash pop` around its commits to isolate the index. Adding to `.claude/memory/` candidates list.

## **2026-05-31 (Sunday morning) — STOP DISPATCH, anthropic session limit reached**

Both still-in-flight subagents — Plan 5 Task 7 [CHIP] (`a6da4cf4919234243`) and Plan 5 Task 12 (`a62d17abd17d44b0c`) — returned identical termination messages: *"You've hit your session limit · resets 11:40am (Asia/Jerusalem)"*. They did NOT complete cleanly.

**State at termination:**

- HEAD = `cbf29d5` (Plan 7 closeout). Plan 7 is fully done and reviewed.
- **No new commits** from either P5 T7 or P5 T12.
- **Uncommitted working tree contains the P5 T12 (bypass wrapper) work-in-progress:**
  - `M gst-hailo-cache/README.md` (probably documents the new element)
  - `M gst-hailo-cache/src/meson.build` (adds bypass source)
  - `M gst-hailo-cache/src/plugin.cpp` (registers bypass element)
  - `M gst-hailo-cache/tests/meson.build` (registers bypass test)
  - `?? gst-hailo-cache/src/gst_hailocachebypass.{cpp,hpp}` (new files)
  - `?? gst-hailo-cache/tests/test_hailocachebypass.cpp` (new test file)
- **P5 T7 [CHIP] left no committed work and no observable working tree changes.** The chip-side run-with-real-pipeline may have produced SQLite files in `/tmp` or `tiling_benchmark/runs/`, but those aren't tracked in git. Effectively: T7 is in unknown state.

**Manager decision: STOP DISPATCH.**

Per the user's guardrails (full autonomy through the weekend), I would normally dispatch more work. But:
1. The session-limit message implies the user's Anthropic account has burned its weekend budget.
2. My own next tool calls may fail with the same error — pushing dispatches that immediately fail is wasteful.
3. The uncommitted P5 T12 work needs a HUMAN review-and-commit before more work happens on top.
4. P5 T7 needs to be re-dispatched once chip-time is available again.

**What's left in Plan 5:**
- T7 [CHIP] — live populate (needs full re-dispatch, status unknown).
- T11 [CHIP] — bit-exact gate via reader+full_frame.
- T12 — bypass wrapper (work-in-progress in tree, needs review+commit).
- T14 [CHIP] — final E2E smoke.
- T15 — close-out (INDEX flip, MEMORY note, spec retro).

**What's done in Plan 5: Tasks 1-6, 8-10, 13** (10 of 15, ~67%).

**What's done in Plan 7: ALL 10 tasks.** Plan 7 is fully complete and reviewed.

**Summary of overnight + weekend completion:**
- Plan 1: pre-existing.
- Plans 2-4: complete (overnight).
- Plan 5: 10/15 (Saturday + Sunday morning).
- Plan 6: not started (deliberately deferred).
- Plan 7: 10/10 (Saturday + Sunday morning).
- Plans 8-10: deliberately deferred.

**Test suite at HEAD `cbf29d5`: 235 + 3 skipped.** No regressions.

**Action items for user (Sunday morning review):**
1. Inspect uncommitted bypass wrapper code in working tree. If it builds + tests pass, commit it as Plan 5 Task 12. If not, debug.
2. Decide whether to re-dispatch Plan 5 Task 7 [CHIP] once rate limit resets.
3. Three Plan 5 tasks remain after T12: T7, T11, T14 (all CHIP) + T15 (close-out).
4. Optionally clean up the empty marker commit `84c0c03` and amend `bb632b8`'s misleading message.

## 2026-05-31 — retrospective + agent formalisation

Captured this weekend's manager pattern + session-limit recovery flow as reusable artifacts:

- `~/.claude/agents/autonomous-project-manager.md` — agent definition for future `Agent({subagent_type: "autonomous-project-manager", ...})` dispatches. Covers mission, dispatch flow, hard stops, session-limit recovery, and named the five anti-patterns from this weekend (cross-agent index race, wrong-venv pytest, test-count drift, empty marker commits, stray .venv).
- `.claude/skills/handling-anthropic-session-limits/SKILL.md` — focused skill on detecting the literal `session limit · resets HH:MM(am|pm) (TZ)` notification, salvage decision tree, cron-based resumption, and the concrete weekend example.
- `.claude/memory/autonomous_weekend_runs.md` — standing conventions (cron :17, state-file path, chip serialization, Plans 6/8/9 out-of-scope, venv path, parallel-agent git rules).

No code or plan state changed by this retrospective entry. Plan 5 Tasks 7/11/12/14/15 still pending user review on wake-up per the entry above.

---

# 2026-05-31 NIGHT — Plan 6 (Cache Warming + Ablation Harness) autonomous run

Plan: `docs/superpowers/plans/2026-05-31-cache-warming-and-ablation-harness.md`
Branch: `tiling-benchmark`. Never main, never push, submodule pointers untouched.

## Baseline (manager-verified at run start)
- HEAD: `5c89bd8` (Plan 6 plan commit). NOTE: repo advanced past the prior weekend-stop
  entry — Plan 5 was closed out (`eaf2e96`) and the working tree is clean of P5 WIP.
- C++ `meson test -C gst-hailo-cache/build`: **5/5**.
- pytest (venv): **238 passed + 6 skipped**.
- **Test floor: 238** (hard-stop below this).
- Chip: HAILO10H present `pci/0000:3d:00.0` FW 5.3.0. chip_in_flight: null
- HEF present, clip 0026 fov50 present (342 MB).
- Pre-existing uncommitted (NOT this run's — never touch): `reid_manager.py` (M),
  `hailo-apps` (M submodule), `sim/PX4-Autopilot` (m), untracked `.claude/scheduled_tasks.lock`,
  `HANDOFF.md`, `dynamic_tiling/runs/dynamic_run_multi_p2.frames.json`.

## ENVIRONMENT CONSTRAINT (critical for resume)
This session's available tools: **Bash, Read, Write, Edit ONLY**.
- NO `Agent`/subagent-dispatch tool — cannot spawn implementer/reviewer subagents.
- NO `Cron*`/`Task*` tools — cannot schedule :17 ticks or a session-limit resume cron.
- Adaptation: the manager executes each task DIRECTLY with rigorous in-line verification
  (write code -> run the task's tests -> run full suites -> self spec+quality review ->
  single-invocation commit), and maintains this file as durable memory.
- If a session limit hits: salvage per skill, record resume queue here + in HANDOFF.md.
  Resume is MANUAL by the user (no cron will fire).

## Conventions
- Python: `/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo-apps/venv_hailo_apps/bin/python -m pytest -q`
- C++: `meson test -C gst-hailo-cache/build`
- Commit messages end with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- Commit = single Bash invocation, explicit file paths, no `git add .`/`-A`.

## Status-review checklist (for resume)
1. `meson test -C gst-hailo-cache/build` (>=5), venv `pytest -q` (>=238). Record counts.
2. `git log --oneline -12`, `git status --short`. No changes on main; no submodule pointer moves.
3. Check chip_in_flight; if A3 warm running, poll `.tile_cache/warm_*.log`.
4. Resume from first unchecked task in the queue below.

## Task queue + status (Plan 6)
Order (one chip): A1 -> A2(+smoke,release) -> B1 -> B2 -> B3 -> A3(bg warm) -> B4 no-chip -> [after A3] B4 chip smoke + real table.
- [x] A1 [no-chip] idempotent cache writes — DONE `a5f2938`. meson 5/5, pytest 239+6.
- [x] A2 [no-chip+4f smoke] scripts/warm_gst_cache.py — DONE `c9e61f0`. Chip smoke verified LIVE (32 rows/4 frames). Chip released. pytest 243+7.
- [x] B1 [no-chip] bench config matrix — DONE `d8c1949`. pytest 248+7.
- [x] B2 [no-chip] per-config crop gen + replay run — DONE `60533f9`. Added tile_norm_to_source_px (0-deviation parity vs cropper rule). Static replays 0 misses. pytest 253+7.
- [ ] B3 [no-chip] hailo-tiling-bench CLI + ablation table
- [~] A3 [CHIP] warm 0026 fov50 (+60/70) background — IN PROGRESS (see tick log)
      grid set: 1x1:0.0;2x2:0.25;3x2:0.25;3x3:0.25;4x3:0.25;6x4:0.25;8x6:0.25;12x9:0.25
- [x] B4 [no-chip + CHIP] GstCropperBackend — DONE. No-chip `9a5f6ea`; chip smoke PASSED (CachingBackend(GstCropperBackend), 4 frames, dets+cache OK).

chip_in_flight: null

## Known carry-forwards (do NOT chase tonight)
letterbox back-mapping bug (use stretch); full_frame post-aggregator payload unwired
(use tile_cache per-tile + de-tile in Python); cross-engine equality value-exact not text.

## Tick log
### 2026-05-31 night — run start
Baseline verified: meson 5/5, pytest 238+6 at `5c89bd8`. State initialized. Starting A1.

### A1 landed — `a5f2938`
INSERT OR IGNORE in C++ (put_many + put_frame_results) and Python (put_many).
Necessary plan-mandated behavior change: old tests asserted dup-PK -> throw + rollback;
updated to idempotent first-writer-wins + added double-insert no-op tests both langs.
Verified: meson 5/5, pytest 239 passed + 6 skipped. Floor still 238 (now 239). Next: A2.

### A2 landed — `c9e61f0`
scripts/warm_gst_cache.py reuses cache_gst_replay_gate pipeline + _grid_to_static_tiles.
Grid spec "NxM:overlap;...". Chip smoke (HAILO_CHIP=1) VERIFIED LIVE: 4 frames 0026 fov50
3x2 -> 32 rows, dets present, meta video_w=3840. Chip released. Needed sys.path fix so
subprocess resolves tiling_benchmark. pytest 243+7, meson 5/5. chip_in_flight: null. Next: B1.

### B1 landed — `d8c1949`
hailo_tiling/bench/{__init__,config}.py: frozen BenchConfig + default_matrix (static set +
dynamic + asahi + altitude_zoom + 1 reference 12x9). 5 tests. pytest 248+7.

### B2 landed — `60533f9`
hailo_tiling/bench/runner.py run_config(): static -> grid -> tile_norm_to_source_px ->
ReplayBackend -> Aggregator; raises on static miss, counts dynamic misses. Added
tile_norm_to_source_px to hashing.py with 0-deviation parity test vs the cropper rule across
all canonical grids. 5 tests. pytest 253+7. Next: B3 (CLI + table + metrics).

### B3 landed — `b699660`
hailo_tiling/cli/bench.py (hailo-tiling-bench entry point) + bench/metrics.py (IoU matcher,
recall/precision vs reference) + bench/grid.py (shim reusing tiling_record._grid_to_static_tiles,
repo-root locate so console script imports work). Emits <cfg>.frames.json + ablation_table.md.
--help verified from /tmp. 3 tests. pytest 256+7, meson 5/5.

ALL no-chip code tasks (A1,A2,B1,B2,B3) DONE. Good-morning priorities 1+3 met (code).
Next: A3 chip warm (timing probe first), then validate fov50 0-miss, then real ablation table.

### A3 chip warm — starting
chip_in_flight: A3-warm (manager-run, foreground bash with timing probe first).
Doing a small timing probe (full grid set, few frames) to estimate full-clip cost before
committing the chip for the long run.

### A3 — WARMER BUG FOUND + WORKAROUND (important)
The full-grid-set warm in ONE process STALLS (deadlocks, State=S, CPU frozen) partway
through grid 8 (12x9). Root cause: running 8 sequential Gst.parse_launch + set_state(NULL)
cycles in one long-lived Python process eventually wedges GStreamer/HailoRT (state leak
across the repeated cropper+hailonet pipeline teardown/relaunch in-process).
- 12x9 grid ALONE (fresh process, 2 frames): completes clean, 251 rows.
- 6x4 grid ALONE (fresh process, 30 frames): 3.6s, 750 rows.
=> Per-grid in a FRESH PROCESS is fast and reliable. WORKAROUND: warm each grid as a
   separate warmer invocation (subprocess), all appending to ONE cache file (idempotent
   via A1). Manager orchestrates this from the shell — no warmer code change tonight.
CARRY-FORWARD for user: make warm_gst_cache.py spawn a subprocess per grid (or fully
   tear down Gst between grids) so a single invocation with many grids doesn't wedge.
   Filed as a Monday cleanup item.

### A3 — fov50 full-clip warm launching (per-grid subprocess loop, background)
Launched per-grid warm (scripts/_warm_one_fov.sh) for fov50 full clip (878 frames) into
.tile_cache/DJI_20260528155239_0026_D_prepared__fov50__a2e9861507428064.sqlite3 (gitignored).
PID 436183, log .tile_cache/warm_fov50.log. 1x1 grid done (878 rows, ~46s). Per-grid-fresh
process approach confirmed fast + reliable. Committed helper + .tile_cache gitignore.
Also committed B4 no-chip part (`<see git log>`). Waiting on fov50 warm to validate 0-miss.

Helper + gitignore commit landed; B4 no-chip part landed before chip warm started.

### A3 — fov50 warm progressing + validation chain queued
fov50 per-grid warm advancing reliably: 1x1 (878), 2x2 (3515), 3x2 (5273), 3x3 (7910),
4x3 done, 6x4/8x6/12x9 in progress. ~50s-3min per grid (12x9 slowest). Cache rows ~45k.
Launched /tmp/validate_fov50.sh (PID 451067): waits for warm PID 436183, then runs
hailo-tiling-bench static configs (1x1..8x6) into dynamic_tiling/runs/ablation_0026_fov50,
which VALIDATES 0 cache misses (the hard-stop gate) and produces the static-baseline
ablation table. Output: /tmp/validate_fov50.out.

RESUME NOTE (if interrupted): if warm finished, check the cache exists + n_rows, then run
the bench CLI manually (see validate_fov50.sh). If a static config shows n_misses>0 ->
HARD STOP (crop-key consistency broken). If 0 misses -> commit the table under
dynamic_tiling/runs/ablation_0026_fov50/ (note: dynamic_tiling/runs is partially gitignored?
check). chip_in_flight: A3-warm (fov50). fov60/fov70 NOT yet warmed.

### A3 — fov50 WARM COMPLETE (17:05:08)
All 8 grids warmed into the fov50 cache. Per-grid rows:
1x1=878, 2x2=3515, 3x2=5273, 3x3=7910, 4x3=10536(approx), 6x4=21095, 8x6=42191, 12x9=94931.
TOTAL = 186340 rows, 878 frames. video_w=3840. Chip now FREE.
Running validation ablation table (static configs 1x1..8x6 + 12x9 ref) -> checking 0 misses.

### A3/B3 — 0-MISS GATE PASSED + ablation table committed (`899b1fc`)
FOUND: GST writer keys frame_idx PER TILE-BUFFER (monotonic), not per source frame; the
per-grid-subprocess warm overlaid 8 independent monotonic sequences. The original per-frame
replay missed — BUT every source-pixel crop key is present (0 of 108 12x9 crops missing;
each crop appears once per source frame). Crop-key consistency INTACT; only frame-indexing
mismatched (documented carry-forward).
FIX: run_static_config_crop_ordered reconstructs source frames by zipping per-crop
occurrence streams. CLI auto-detects per-tile-buffer caches and uses it for static rows
(dynamic rows skipped on such caches — need live frame indexing / B4).
RESULT: committed static-baseline ablation table for 0026 fov50 (186340 rows, 879 frames),
ALL static configs n_misses=0, recall 1x1=0.28 -> 12x9=1.00 (ref). pytest 261+8, meson 5/5.
=> Good-morning priorities 1,2,3 ALL MET. chip_in_flight: null (warm done). Chip FREE.

Remaining (stretch): B4 chip smoke (4 frames, quick), warm fov60/fov70 + their tables.

### B4 chip smoke PASSED + fov60/fov70 stretch launched
B4 chip smoke (HAILO_CHIP=1, tests/integration/test_gst_cropper_chip.py) PASSED — live
GstCropperBackend wrapped in CachingBackend returns 4 crop-ordered det-lists + populates
cache. B4 fully done (all 6 plan tasks complete).
Launched fov60 warm+table chain (PID 477841, /tmp/fov60.out). Will chain fov70 after.
chip_in_flight: A3-warm-fov60.

### fov60 DONE + committed; fov70 launched
fov60 warm complete (186340 rows, 879 frames). Ablation table committed: all static configs
n_misses=0, recall 1x1=0.32 -> 12x9=1.00. Launched fov70 chain (PID 511512, /tmp/fov70.out).
chip_in_flight: A3-warm-fov70 (last FOV). After fov70: all 3 FOVs warmed + tables = full
A3 acceptance met.

### fov70 DONE + committed — ALL 3 FOVs WARMED. chip_in_flight: null
fov70 warm complete (879 frames, all 8 grids). Table committed (`e4ce677`): all static
configs n_misses=0, recall 1x1=0.26 -> 12x9=1.00. ALL plan tasks + all 3 FOV tables done.

## Wake-up summary (2026-05-31 ~17:35)

### Plan done this run
- Plan 6 (Cache Warming + Ablation Harness): **ALL 6 tasks landed + reviewed (self spec+quality),
  all 3 FOV ablation tables committed.** Branch tiling-benchmark, head `e4ce677`.
  - A1 `a5f2938` idempotent cache writes (INSERT OR IGNORE) C++ + Python.
  - A2 `c9e61f0` warm_gst_cache.py (chip smoke verified live).
  - B1 `d8c1949` bench config matrix.
  - B2 `60533f9` bench runner + tile_norm_to_source_px (0-deviation parity).
  - B3 `b699660` hailo-tiling-bench CLI + table + IoU metrics.
  - B4 `9a5f6ea` GstCropperBackend (no-chip) + chip smoke PASSED.
  - A3: warmed 0026 fov50/60/70 (each ~186k rows, 879 frames, full 8-grid set);
    crop-ordered replay fix `899b1fc`; tables `899b1fc`(fov50)/`1af34a5`(fov60)/`e4ce677`(fov70).

### Ablation results (static-baseline, recall vs 12x9 GT, IoU>=0.5, all n_misses=0)
- fov50: 1x1=0.28 .. 6x4=0.44 .. 8x6=0.53 -> 12x9=1.00
- fov60: 1x1=0.32 .. 6x4=0.45 .. 8x6=0.55 -> 12x9=1.00
- fov70: 1x1=0.26 .. 6x4=0.54 .. 8x6=0.61 -> 12x9=1.00
Tables: dynamic_tiling/runs/ablation_0026_fov{50,60,70}/ablation_table.md

### Cleanup needed before next dispatch (Monday)
- warm_gst_cache.py wedges if given MANY grids in one invocation (in-process GStreamer/HailoRT
  state leak across 8 pipeline teardown/relaunch cycles). WORKAROUND used: per-grid subprocess
  via scripts/_warm_one_fov.sh. FIX: make the warmer spawn a subprocess per grid (or fully
  reset Gst between grids). Low risk, ~30 min.
- GST hailocachewriter keys frame_idx PER TILE-BUFFER (monotonic), not per source frame. The
  bench auto-detects this and uses crop-ordered replay for static rows. The proper long-term
  fix is to have the writer stamp the source frame index (or the bench read it from buffer
  PTS). Documented; not blocking the static tables.

### Decisions for you
- Dynamic / lever ablation rows (dynamic, +asahi, +altitude_zoom) are NOT in the tables:
  they need live ROI-tile warming via CachingBackend(GstCropperBackend) (B4 path) because ROI
  tiles can't be pre-warmed, AND need live frame indexing. This was the documented stretch goal.
  Next session: run the dynamic rows through the live GstCropperBackend path to extend the tables.
- The .tile_cache/ caches (~13MB x3) are gitignored (regenerable via _warm_one_fov.sh). Keep or
  delete as you like.

### Test suite
- 261 passed + 8 skipped (pytest, venv); meson 5/5. Floor was 238. No regressions.
- No changes on main; no submodule pointer moves; pre-existing uncommitted files untouched.


---

# 2026-05-31 NIGHT 2 — Dynamic Ablation Rows + Scaling + Paper Scaffold

Plan: `docs/superpowers/plans/2026-05-31-dynamic-ablation-and-scaling.md`
Branch: `tiling-benchmark`. Never main, never push, submodule pointers untouched.

## ENVIRONMENT CONSTRAINT (critical for resume)
This session's available tools: **Bash, Read, Write, Edit ONLY** (verified via ToolSearch:
no Agent/subagent, no Cron*, no Task*). Same as Night 1.
- Adaptation: manager self-executes each task with rigorous inline verification
  (write -> task tests -> full suites -> self spec+quality review -> single-invocation commit).
- No subagent two-stage review possible; self-review is documented per task.
- Session-limit handling is MANUAL resume (no cron will fire). Salvage + record queue here +
  in HANDOFF.md.

## Baseline (manager-verified at run start)
- HEAD: `74480e6` (Night-2 plan commit, one past Plan-6-complete `b15623d`).
- pytest (venv): **261 passed + 8 skipped**.  C++ meson: **5/5**.
- **Test floor: 261** (hard-stop below this).
- Chip: HAILO10H `pci/0000:3d:00.0` FW 5.3.0 present. chip_in_flight: null
- Warmed (Night 1): `.tile_cache/DJI_..._0026_..._fov{50,60,70}__a2e9861507428064.sqlite3`
  (each ~186k rows, 879 frames, full 8-grid static set, 0 misses). Gitignored.
- Pre-existing uncommitted (NOT this run's — never touch): reid_manager.py (M),
  hailo-apps (M submodule), sim/PX4-Autopilot (m), .claude/scheduled_tasks.lock, HANDOFF.md,
  dynamic_tiling/runs/dynamic_run_multi_p2.frames.json.

## Conventions
- Python: `/home/giladn/.../hailo-apps/venv_hailo_apps/bin/python -m pytest -q`
- C++: `meson test -C gst-hailo-cache/build`
- Commit trailer: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- Commit = single Bash invocation, explicit file paths, no `git add .`/`-A`.

## KEY FINDING re B1 (recorded before starting)
KB note `hailotilecropper_dynamic.md`: the INSTALLED `hailotilecropper_dynamic` plugin supports
ONLY `tiles-static`. The dynamic `HailoTileROI`-via-`identity signal-handoffs` injection path
the plan's B1 references was NEVER landed in the C++. THEREFORE B1's literal signal-handoff
injector is unbuildable against the real plugin -> take the plan's documented **per-frame-relaunch
fallback**: GstCropperBackend.infer() ALREADY accepts a per-frame `crops` list and builds a
per-frame `tiles-static` string + selects the frame via frame_idx. So per-frame dynamic tiling is
already mechanically supported via tiles-static; B1 work = make the no-chip test assert per-frame
tiles-static variation (the fallback), B2 drives the scheduler to produce per-frame crops.

## Task queue + order (one chip)
No-chip first: A1, B1(no-chip fallback), B2, C1, C2. Then chip serialised: B1 smoke -> A3 -> B3
-> A2 -> C3 smoke.
- [ ] A1  [no-chip] warmer subprocess-per-grid
- [ ] C1  [no-chip] independent review of Night-1 12 commits a5f2938..b15623d
- [ ] C2  [no-chip] paper scaffold + renderer
- [ ] B1  [no-chip fallback + CHIP smoke] GstCropperBackend per-frame tiles
- [ ] B2  [no-chip] dynamic runner (scheduler + ByteTracker per-frame tiles)
- [ ] A3  [CHIP] warm 0026 dynamic ROI tiles
- [ ] B3  [no-chip after A3] dynamic-vs-static ablation tables
- [ ] A2  [CHIP] warm 0027/0029 static
- [ ] C3  [no-chip + CHIP smoke] full_frame detection payload

chip_in_flight: null

## Status-review checklist (for manual resume)
1. meson test -C gst-hailo-cache/build (>=5); venv pytest -q (>=261). Record counts.
2. git log --oneline -12; git status --short. No main changes; no submodule pointer moves.
3. chip_in_flight; if a warm is running poll its .tile_cache/*.log.
4. Resume from first unchecked task above.

## Tick log
### run start
Baseline verified: meson 5/5, pytest 261+8 at `74480e6`. State initialized. Starting A1.

### A1 landed — `442d86e`
warm() now spawns one fresh subprocess per grid (internal --_single-grid-child child runs
_warm_one_grid_in_process); parent stamps meta once. subprocess_runner seam injected for the
no-chip test test_warmer_spawns_subprocess_per_grid (asserts one child per grid, in order).
_warm_one_fov.sh kept (redundant but harmless). pytest 262+8, meson 5/5. Next: B1 (no-chip
fallback) + B2.

### B2 landed — `94d04e4`
hailo_tiling/bench/runner.py run_dynamic_config(): stateful TileScheduler + production
ByteTracker (TargetLock) per-frame loop; backend-agnostic (CachingBackend warm OR ReplayBackend
replay); counts per-crop misses without raising; GT-seeded lock. 2 tests (per-frame tile
variation + ROI-on-lock; replay miss-count). Existing run_config placeholder untouched.
Lever flags not yet reshaping crops (noted follow-up). pytest 264+8, meson 5/5. Next: B1
(per-frame-relaunch fallback — signal-handoff plugin path unimplemented per KB note).

### B1 landed — `108f0c0`
GstCropperBackend per-frame ROI injection via per-frame-relaunch fallback (signal-handoff plugin
path unimplemented per KB note). No-chip test test_gst_cropper_injects_per_frame_rois (2 vs 4 ROI
frames -> distinct pipelines, exact counts/rects). Chip smoke
test_gst_cropper_per_frame_roi_injection_ordered added (runs in chip phase). Module docstring
documents the fallback. pytest 265+9, meson 5/5.

HEADLINE no-chip path DONE: A1, B1, B2 landed. Remaining headline: A3 (chip warm) -> B3 (tables).
Plan: do Track C no-chip (C1 review, C2 paper scaffold) next, THEN the chip phase
(B1 smoke -> A3 -> B3 -> A2 -> C3 smoke).

### C1 landed — `6fc6e34`
Independent review doc docs/superpowers/reviews/2026-05-31-night1-review.md. No blocking issues.
1 IMPORTANT non-blocking: warmed fov50 cache has 8 crops at 878 occ vs 204 at 879 (incl 1x1
full-frame crop) -> crop-ordered min-n_frames truncates short-crop configs to 878 while ref=879
-> ~0.1% recall understate; verified NOT misalignment (trailing frame, contiguous). Carry into
B3: score all configs over common frame prefix. 2 nits logged. No fix commits needed.

### C2 landed — `e0b2fdb`
docs/paper/technical-report.md + reproducibility.md + scripts/render_ablation_into_report.py
(marker-based, committed-numbers-only, idempotent, --check mode). Rendered the 3 real FOV tables.
6 no-chip tests. pytest 271+9, meson 5/5.

## CHIP PHASE BEGINS — chip serialised
All no-chip headline + Track-C work landed (A1 442d86e, B2 94d04e4, B1 108f0c0, C1 6fc6e34,
C2 e0b2fdb). Chip order: B1 smoke -> A3 (dynamic warm, priority) -> B3 (tables) -> A2 -> C3 smoke.
Note for B3: score configs over common frame prefix (C1 finding).
chip_in_flight: B1-smoke

### B1 chip smoke PASSED
HAILO_CHIP=1 tests/integration/test_gst_cropper_chip.py -> 2 passed: existing CachingBackend
4-tile smoke + new per-frame-relaunch ordered test (2-ROI then 4-ROI frame). Per-frame relaunch
injection confirmed live on HAILO10H. chip_in_flight: null (smoke done). Next: A3.

### A3 — TARGET-CLASS FINDING (important, recorded before warming)
Clip 0026 has NO person (cls 0) detections in the 12x9 reference — class histogram fov50:
{cls1 vehicle:3468, cls2 face:4425, cls3 lp:944}. The dynamic scheduler's TargetLock defaults to
person_cls=0, so person-tracking would never lock -> degenerate (discovery-grid-only) dynamic run.
DECISION: drive the dynamic single-target tracker on the most-present class. Stitched trajectory
frames/878 (fov50): face=862, license_plate=827, vehicle=435. -> Track cls 2 (FACE) as the
single target (most continuous). The tiler is class-agnostic; this faithfully exercises ROI tiling.
run_dynamic_config(person_cls=2), GT trajectory from build_target_trajectory(label='face') over
the committed 12x9.frames.json (schema-adapted). Dynamic tiles warmed into a SEPARATE
per-source-frame cache via CachingBackend(GstCropperBackend); static caches untouched.
chip_in_flight: A3-warm

### A3 — DEGENERATE DYNAMIC FINDING (honest result, recorded)
Probe (40 frames, fov50, track face cls2): mean_tiles=0.45, n_dets=0, 18 rows, 0 misses, ~1s.
The dynamic config (discovery_period=15, 3x2 discovery grid) emits ONLY the discovery grid every
15 frames; those 6 coarse tiles (~1440x1080 each, ~2.25x downscale to 640 model input) find ZERO
faces -> tracker never locks -> NO ROI tiles ever emitted. Faces in 0026 are below the coarse
discovery grid's detection floor (dense static 8x6/12x9 succeed because their tiles are small
enough). This is NOT a hard-stop bug (injection works; 0 misses; tracker deterministic) but a
data/config reality: the dynamic tiler can't engage ROI tracking on this clip as configured.
DECISION: warm the dynamic configs over the FULL clip anyway (cheap, ~seconds) and produce the
dynamic rows in the B3 table as an HONEST result (dynamic ~0.5 tiles/frame, near-zero recall
because it never locks), with this explanation in the table notes + report discussion. The
headline 'dynamic-vs-static tables for 0026' is still delivered; the finding (coarse discovery
grid under-seeds small targets) is itself a paper-worthy observation.

### A3 DONE — `6fc6cdd` (driver committed; caches gitignored)
Warmed dynamic configs (dynamic, +asahi, +altitude_zoom) full clip (878 frames) for fov50/60/70
into separate per-source-frame caches .tile_cache/..._{fov}__dynamic.sqlite3 (354 rows each, ~11s
each). ACCEPTANCE MET: chip-free ReplayBackend replay of each dynamic config = 0 MISSES (all
configs, fov50 verified; deterministic tracker reproduces warmed crops). Dynamic is degenerate
(0.40 tiles/frame, 0 dets — never locks, see finding above) but the warm/replay machinery is
sound. All 3 dynamic configs identical (levers don't reshape crops when no ROI exists).
chip_in_flight: null. Next: B3 (tables with dynamic rows + matched-compute column).

### B3 DONE — table commit + report discussion `347c8ff`
hailo-tiling-bench --dynamic-cache/--ref/--target-class merges dynamic rows (ReplayBackend +
run_dynamic_config, 0 misses) into the table with a matched_compute column (metrics.
matched_compute_delta, 4 tests). Regenerated all 3 FOV tables: dynamic rows present, 0.40
tiles/frame, recall 0, delta -0.26..-0.32 vs 1x1 (honest degenerate result — never locks).
Report discussion rewritten with the finding. pytest 275+9, meson 5/5.

## HEADLINE PATH COMPLETE: A1->B1->B2->A3->B3 all landed + reviewed (self).
Good-morning priorities 1 (A1), 2 (dynamic-vs-static 0026 all FOV), 3 (C1), 4 (C2) ALL MET.
Remaining filler: A2 (warm 0027/0029 static), C3 (full_frame payload).
chip_in_flight: null. Next: C3 no-chip part (C++), then A2 chip warm, then C3 chip smoke.

### C3 (no-chip C++) DONE — `02e3ec7`
full_frame writer now serializes real dets_json (read_tile_dets_json_ on post-aggregator ROI) +
tiles_json (new read_tiles_json_ over HailoTileROI sub-objects, cache_keys kTileField* consts).
No-chip gtest FullFramePayload (seeds ROI det + tile, asserts non-empty payloads; writer test
gains HAVE_GSTHAILOMETA+tappas+appsrc deps). INDEX Phase-14 follow-up -> RESOLVED. meson 5/5,
pytest 275+9. C3 chip smoke (full_frame over real frames) remains for chip phase.

### Chip filler: A2 (warm 0027/0029 static) + C3 chip smoke
chip_in_flight: A2

### A2 fov50 warm — 0027 IN PROGRESS
Launched 0027 fov50 static warm (full 8-grid set, subprocess-per-grid via fixed warmer A1).
PID 632009, log .tile_cache/warm_0027_fov50.log. Progress: 1x1..4x3 done, 61k rows, on 6x4/8x6/
12x9. 0027 denser than 0026 (1x1=1913 rows). After 0027 fov50: validate 0-miss static replay +
emit table, then warm 0027 fov60/70 + 0029 all FOV. chip_in_flight: A2 (0027 fov50).

### A2 fov50 0027 — still warming dense grids (8x6/12x9)
6x4 done (107k rows). 8x6 (48 tiles/frame) + 12x9 (108) over ~880 frames are the slow tail.
PID 632009 still running. Monitors bt4hltjbs/bj6ze2b2q watch for exit.
RESUME: when 632009 exits, check .tile_cache/warm_0027_fov50.log for DONE; then run
hailo-tiling-bench static-only over the 0027 fov50 cache (per-tile-buffer -> crop-ordered) into
dynamic_tiling/runs/ablation_0027_fov50 to validate 0 misses + emit table; commit table; then
warm 0027 fov60/70 + 0029 all FOV (same loop). C3 chip smoke (full_frame over real frames) also
pending. If session-limit/interrupt: A2 + C3 chip smoke are the only remaining items; headline +
C1 + C2 + C3-no-chip all DONE.

### A2 0027 fov50 — 8x6 done (199k rows), 12x9 (final grid) in flight
8x6 added 91,871 rows. 12x9 (108 tiles, slowest) started. 0027 is denser/longer than 0026.
A2 full 6-clip-FOV warm is SLOW (each dense FOV ~12-15 min). REVISED A2 SCOPE given time:
finish 0027 fov50 + validate + table, do C3 chip smoke (higher value), then warm 0029 fov50 if
time. Will NOT attempt all 6 caches tonight (lowest-priority filler; headline + C1/C2/C3 done).

### A2 0027 fov50 DONE — table `<committed>`
405,760 rows / 1914 frames, ALL static configs 0 misses. recall 1x1=0.11..6x4=0.54..12x9=1.0.
Subprocess-per-grid (A1) worked perfectly — no deadlock. Cache gitignored.
Now warming 0029 fov50 (smaller clip).
chip_in_flight: A2 (0029 fov50)

### A2 DONE — 0027 + 0029 fov50 tables (acceptance met)
0027 fov50: 405,760 rows/1914 frames, 0 misses, recall 1x1=0.11..12x9=1.0.
0029 fov50: 134,824 rows/636 frames, 0 misses, recall 1x1=0.00..8x6=0.78..12x9=1.0 (small targets).
Both committed. A2 acceptance (0027/0029 fov50 static tables) MET. Did NOT warm fov60/70 of
0027/0029 (time — dense 12x9 grids ~10-15min each; lowest-priority filler).
chip_in_flight: null. Next: C3 chip smoke (full_frame post-aggregator non-empty payload).

### C3 CHIP SMOKE PASSED — full_frame payload verified live
Ran cropper->hailonet->hailofilter->hailotileaggregator->hailocachewriter mode=full_frame over
5 frames of 0027 fov50 (HAILO_CHIP=1). frame_results: 6 rows, 5/6 non-empty dets_json (the
empty one = a frame with no dets, correct), 6/6 non-empty tiles_json. dets carry source-frame
cls1/cls2 coords; tiles_json carries the 3x2 layout [{x,y,w,h,mode}]. Notably tiles_json survives
post-aggregator (better than expected). C3 FULLY COMPLETE (no-chip + chip smoke). chip_in_flight:
null.

## Wake-up summary (2026-06-01, Night 2)

### Run mode
Self-execution (no Agent/Cron/Task tools this session — Bash/Read/Write/Edit only, same as
Night 1). Each task: write -> task tests -> full suites -> self spec+quality review -> single
commit. The autonomous-project-manager two-stage subagent review was NOT possible; self-review
is documented per task above.

### Headline result — DELIVERED (good-morning priorities 1-4 all met)
Dynamic-vs-static tiling tables for clip 0026 (fov50/60/70) with a matched-compute column now
exist (`dynamic_tiling/runs/ablation_0026_fov{50,60,70}/ablation_table.md`). The full path
A1->B1->B2->A3->B3 landed:
- A1 `442d86e` warmer subprocess-per-grid (fixes the multi-grid teardown deadlock; verified on
  the real A2 warms — 8 grids, no wedge).
- B2 `94d04e4` stateful dynamic runner (TileScheduler + production ByteTracker per frame).
- B1 `108f0c0` GstCropperBackend per-frame ROI injection via per-frame-relaunch FALLBACK (the
  signal-handoff plugin path is NOT in the installed C++ — KB note). Chip smoke PASSED.
- A3 `6fc6cdd` warmed dynamic ROI tiles on-chip into per-source-frame caches; chip-free replay
  of every dynamic config = 0 MISSES (deterministic tracker reproduces warmed crops).
- B3 `a3bf094`+`347c8ff` regenerated tables w/ dynamic + +asahi + +altitude_zoom rows + a
  matched_compute recall-delta column vs the equal-tiles static grid.

**The dynamic result is an HONEST NEGATIVE on 0026:** the configured 3x2 discovery grid
(~1440px tiles, 2.25x downscale) cannot detect 0026's small faces (clip has NO person/cls0;
dominant class is face), so the tracker never locks and no ROI tiles are emitted -> dynamic runs
at ~0.40 tiles/frame with ~0 recall (delta -0.26..-0.32 vs the 1x1 grid). The warm/replay/
matched-compute MACHINERY is fully sound (0 misses); the finding (track-guided tiling is only as
good as its discovery stage) is itself the paper-worthy observation, written up in the report
discussion.

### Also landed
- C1 `6fc6e34` independent review of Night-1's 12 commits — no blocking issues; 1 IMPORTANT
  non-blocking finding (the 878-vs-879 per-crop occurrence asymmetry; verified NOT a misalignment
  bug). Review doc: `docs/superpowers/reviews/2026-05-31-night1-review.md`.
- C2 `e0b2fdb` paper scaffold: `docs/paper/technical-report.md` + `reproducibility.md` +
  `scripts/render_ablation_into_report.py` (marker-based, committed-numbers-only, idempotent,
  --check mode). Report's results section auto-rendered from the real committed tables.
- C3 `02e3ec7` full_frame writer now records REAL dets_json + tiles_json (Phase 14). No-chip
  gtest + CHIP SMOKE PASSED (5/6 non-empty dets, 6/6 non-empty tiles over real 0027 frames).
  INDEX Phase-14 follow-up flipped to RESOLVED.
- A2 `89eb56e`+`0dfa9cf` warmed clips 0027 + 0029 fov50 (full static grid set, 0 misses) +
  tables. 0027: recall 1x1=0.11..12x9=1.0 (1914 frames). 0029: 1x1=0.00..8x6=0.78..12x9=1.0
  (636 frames, small targets — a clean tiling-benefit illustration).

### Cleanup / decisions for you
- **Dynamic on 0026 is degenerate by design-of-clip, not a bug.** To get a POSITIVE dynamic
  result, either (a) run the dynamic configs on a clip with larger lockable targets, or (b) make
  the scheduler's discovery grid denser / size-gated so it can seed on small targets (a scheduler
  change, future work). The methodology + pipeline are validated and ready.
- **C1 finding (precision refinement, not a bug):** the per-tile-buffer caches have a few crops
  with 878 vs 879 occurrences (incl the 1x1 full-frame crop), so crop-ordered replay truncates a
  short-crop config to the common prefix (~0.1% recall understate). Consider scoring all configs
  over the common frame prefix in a future bench pass, or re-warm uniformly.
- **A2 left fov60/70 of 0027/0029 unwarmed** (only fov50 per acceptance — the dense 12x9 grids
  take ~10-15 min each; lowest-priority filler). Re-run `scripts/warm_gst_cache.py` per the
  reproducibility doc to extend coverage.
- `.tile_cache/*.sqlite3` (static + new `*__dynamic.sqlite3`) are gitignored (regenerable).
- Pre-existing uncommitted files (reid_manager.py, hailo-apps + sim/PX4 submodule pointers,
  .claude lock, HANDOFF.md, dynamic_run_multi_p2.frames.json) were NOT touched. No changes on
  main; no submodule pointer moves; nothing pushed.

### Test suite
- pytest **275 passed + 9 skipped** (venv); meson **5/5**. Floor was 261. No regressions
  (grew +14 from new tests). All commits on `tiling-benchmark`, head `<state commit follows>`.
