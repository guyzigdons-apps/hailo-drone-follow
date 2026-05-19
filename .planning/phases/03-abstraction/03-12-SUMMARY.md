failed: F1 + F2 closed by 03-11; new F3 surfaced — `/follow/<id>` rejects clicks with stale ByteTracker IDs

---
phase: 03-abstraction
plan: 12
status: failed
gate: operator-witnessed SITL (re-run after 03-11 gap closure)
world: walk_across_then_approach
operator: guyz
date: 2026-05-19
---

## Resume signal

**failed** — F1 (web UI velocity stale) and F2 (INFO log spam) from the 03-10 gate are CONFIRMED CLOSED by the 03-11 gap closure (operator verified the status bar updates live during follow and no `ctrl: bh=...` lines appear in the default console). However, the witnessed re-run surfaced a third issue (F3) on a different code path: clicking a visible person in the web UI overlay sometimes sends a stale ByteTracker ID that no longer exists, and `follow_server` rejects the click with `Detection ID 1 not found. Available: {45}` instead of recovering.

Phase 3 does NOT close. Run `/gsd-plan-phase 3 --gaps` to generate a fix plan (03-13); operator re-runs SITL as a 03-14 re-gate.

## Findings

### F1 — CLOSED (verified by 03-11)

The web UI status bar now updates live during follow with mode ∈ {AUTO, LOCKED, LOST, IDLE} and non-zero forward / yaw values that change at ≥5 Hz. The 03-11 commits (`47f796c` + `abba4cf`) thread `ui_state` through `run_robot_loop` and publish `(forward, down, yawspeed, mode)` at each tick branch + the shutdown finally. The `MavsdkDroneAdapter` remained untouched (axes-only Capabilities contract preserved). Closed.

### F2 — CLOSED (verified by 03-11)

No `ctrl: bh=... factor=... filtered=...` INFO log lines appeared in the default console during the SITL run. The 03-11 commit `47f796c` demoted the line at `hailo_drone_detection_manager.py:533` from `LOGGER.info` to `LOGGER.debug`. Data preserved at DEBUG level for diagnostic use. Closed.

### F3 — `/follow/<id>` rejects clicks with stale ByteTracker IDs (NEW — pre-existing, surfaced by gate re-run)

**Severity:** medium (operator click-to-lock UX broken; AUTO mode still works as fallback)
**Origin:** pre-Phase-3 (predates the milestone; the strict ID check has been in `follow_server.py` since the v1.1 rename `5850558` and earlier — `git log --all --oneline -- robot_follow/servers/follow_server.py` shows only the rename commit). NOT a regression from 03-11.
**File evidence:**
- `robot_follow/servers/follow_server.py:96-112` — `do_POST` for `/follow/<id>` parses the integer ID, looks it up in `self.shared_state.get_available_ids()`, and returns 404 if absent. Logged at line 111: `LOGGER.info("Detection ID %d not found. Available: %s", detection_id, available_ids)`.
- `robot_follow/ui/src/App.jsx:285-292` — `handleFollow(id)` posts to `/follow/${id}` with no retry / position fallback.
- `robot_follow/ui/src/App.jsx:686` — onClick handler closure captures `det.id` at React render time; if the SSE detection stream advances between render and click, the closure can hold a stale ID.
- `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py:131` — ByteTracker assigns sequential `track_id` per activation; when a track is lost (occlusion / low detector confidence / off-screen edge) and re-acquired for the same person, the new track_id is higher than the previous one.

**Symptom (operator-observed during 03-12 SITL run):** during the `walk_across_then_approach` flow, the operator clicked on the visible actor bbox to lock-on. The console emitted `INFO | servers.follow_server | Detection ID 1 not found. Available: {45}` and the lock did not take. The actor was still successfully tracked under track_id=45 (AUTO mode would have followed correctly), but the explicit click-to-lock failed because the click handler sent a stale ID (likely from an earlier track that had been lost and re-acquired during the run).

**Why F1/F2 fixes did not cover this:** the 03-11 gap closure was scoped to the orchestrator/adapter UI publish boundary and the log demotion. F3 is on a separate code path (HTTP follow_server + UI click handler) that 03-11 did not touch.

**Suggested fix shapes (any one closes F3; pick during plan-phase):**

1. **Server-side leniency by uniqueness** (smallest change — 1 file): in `follow_server.py:96-112`, when the requested `detection_id` is unknown but `len(available_ids) == 1`, lock onto that single available ID instead of returning 404. Operator intent is unambiguous. Add a unit test in `test_follow_server.py` pinning this behavior.

2. **Server-side leniency by position** (more robust — UI + server): UI sends click coordinates `{x, y, id}` to follow_server; server falls back to "closest visible person to click point" when the ID lookup fails. Requires SSE schema enrichment (already has bbox geometry) and a UI payload change.

3. **UI-side ID refresh** (UI-only): before firing the POST, `handleFollow` re-reads `/api/detections` and remaps the clicked bbox to the current ID at that bbox position. Trade-off: extra HTTP round-trip per click.

The plan-phase step should pick one (recommend #1 — smallest, testable, deterministic) and call it out in CONTEXT.

**Acceptance for the gap plan (03-13):**
- Click on a visible person bbox in the web UI always either (a) locks onto the visible person or (b) returns a deterministic message that the operator can act on (e.g., "stale ID; retry").
- New unit test in `test_follow_server.py` pinning the chosen leniency behavior (e.g., single-available-ID lock).
- No regression in the existing `test_follow_server.py` suite.
- `MavsdkDroneAdapter` untouched (axes-only contract preserved — this is server-layer behavior).

## Acceptance criteria scorecard

| # | Criterion | Result | Note |
|---|-----------|--------|------|
| 1 | Drone arms + takes off cleanly with `--takeoff-landing` | ✅ | unchanged from 03-10 |
| 2 | Drone follows actor through full walk pattern | ✅ | AUTO mode (no click) follows correctly |
| 3 | Altitude held within ±0.5 m of target | ✅ | unchanged from 03-10 |
| 4 | Smooth motion (no oscillation/jerk) | ✅ | unchanged from 03-10 |
| 5 | Clean Ctrl+C shutdown + land | ✅ | unchanged from 03-10 |
| 6 | No new FATAL / ERROR vs pre-Phase-3 | ✅ | no new fatal/error |
| 7 | **F1**: status bar shows mode + non-zero velocity during follow at ≥5 Hz | ✅ | **CLOSED by 03-11** |
| 8 | **F1**: forward swings negative on actor approach (pre-smoothing values flowing) | ✅ | **CLOSED by 03-11** |
| 9 | **F2**: no `ctrl: bh=...` INFO log spam in default console | ✅ | **CLOSED by 03-11** |
| 10 | No new regression in follow quality vs 03-10 | ✅ | follow quality indistinguishable |
| 11 | Full pytest suite green | ✅ | 290 passed, 7 skipped (post-merge) |
| 12 | **F3**: click-to-lock locks onto the visible person | ❌ | **NEW gap — server rejects stale ID** |

## What was tested

- Operator launched `sim/start_sim.sh --bridge --world walk_across_then_approach` (Terminal 1) and `drone-follow --robot drone --input udp://0.0.0.0:5600 --takeoff-landing --webui` (Terminal 2)
- Watched the web UI status bar during the full walk pattern — confirmed live mode + velocity updates
- Watched the default console — confirmed no `ctrl: bh=...` INFO spam
- Attempted click-to-lock on the visible actor in the web UI — observed F3 failure mode
- AUTO mode (no click) continued to follow correctly throughout

## Next step

1. `/gsd-plan-phase 3 --gaps` reads this SUMMARY's Findings section and creates `03-13-PLAN.md` (server-side leniency, recommended) + `03-14-PLAN.md` (operator SITL re-gate, autonomous: false).
2. Operator runs `/gsd-execute-phase 3 --gaps-only` to land 03-13.
3. Operator re-runs SITL gate (03-14) to confirm F3 closed AND F1/F2 stay closed AND no new regression.
4. On approved 03-14: Phase 3 closes; ABS-11 marked Complete; rover phases (04, 05) unblock.

## Verifier note

The `gsd-verifier` should keep `human_needed` raised against ABS-11 until 03-14-SUMMARY.md records `approved` (or `approved-with-deferral`). The 03-10 and 03-12 SUMMARYs both record `failed` — three SITL attempts on this gate is unusual but defensible given F1 was a Phase-3 regression and F3 is a pre-existing UX bug surfaced under operator scrutiny. Future verifier runs should not auto-close this gate based on test-suite green alone.
