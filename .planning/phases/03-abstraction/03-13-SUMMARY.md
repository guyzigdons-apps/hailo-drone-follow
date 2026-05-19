---
phase: 03-abstraction
plan: 13
subsystem: ui
tags: [web-ui, gap-closure, F3, ABS-11, click-to-lock, client-id-resolution, react, sse]

# Dependency graph
requires:
  - phase: 03-abstraction
    provides: "ABS-11 SITL gate (plan 03-12) surfaced F3: stale ByteTracker id in onClick closure caused /follow/<id> to 404"
  - phase: 03-abstraction
    provides: "Plan 03-11 / 03-12 confirmed follow_server.py contract is strict-id-or-404; that contract is preserved by this plan"
provides:
  - "Client-side current-id resolution in handleFollow: bbox click resolves to the latest SSE detection's id via nearest-bbox-center match"
  - "detectionsRef useRef mirror in App.jsx so click-time can read latest SSE state outside React's render closure"
  - "Operator click-to-lock is now resilient to ByteTracker re-ID between SSE render and click firing (the F3 race)"
affects: [03-14, future-multi-actor-UX, future-IoU-based-resolver]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "useRef mirror of state for closure-stale-value avoidance: when an event handler must read the LATEST value (not the render-time value), mirror state to a ref in the same effect that calls setState, then read the ref at event time"
    - "Position-based bbox identity (nearest-center) as a client-side resolver for tracker re-ID races"

key-files:
  created:
    - ".planning/phases/03-abstraction/03-13-SUMMARY.md"
  modified:
    - "robot_follow/ui/src/App.jsx"

key-decisions:
  - "Client-side current-id resolution (NOT server-side leniency widening) per ~/.claude/projects/.../feedback_click_to_follow_id_resolution.md"
  - "Position-based nearest-bbox-center as the resolver — simplest robust mapping of 'the bbox the operator visually clicked' to a current id"
  - "No distance threshold on the resolver — operator-clarified intent is 'send the bbox's current id', and the single-actor SITL gate that surfaced F3 has only one candidate"
  - "No JS test framework introduced — repo has none; static check is vite build, end-to-end check is the operator SITL gate (plan 03-14)"

patterns-established:
  - "useRef-mirror pattern for closure-stale-id avoidance in React onClick: detectionsRef.current is written immediately after setDetections in the SSE handler, then read inside handleFollow"
  - "onClick payload is the whole domain object (det), not the primitive id: handler then resolves the live identity at click time"

requirements-completed: [ABS-11]

# Metrics
duration: 4min
completed: 2026-05-19
---

# Phase 3 Plan 13: F3 Click-to-Lock Stale ID Fix Summary

**Client-side current-id resolution in `App.jsx::handleFollow` via a `detectionsRef` mirror — operator clicks now POST the bbox's CURRENT id from the latest SSE frame, not the stale id captured by the React render closure.**

## Performance

- **Duration:** ~4 min
- **Started:** 2026-05-19T17:14:00Z
- **Completed:** 2026-05-19T17:18:41Z
- **Tasks:** 1
- **Files modified:** 1 (`robot_follow/ui/src/App.jsx`)

## Accomplishments

- **F3 race closed at the client layer.** When ByteTracker loses + re-acquires the same person between SSE render and click firing, the resolver maps the clicked bbox to the new id by nearest-bbox-center match. Operator's visual intent is preserved; the server's strict `/follow/<id>` contract is unchanged.
- **follow_server.py contract preserved.** No server-side leniency widening; the rejected server-side approach (commit `72add07`, pre-plan-rewrite) was superseded by this client-only fix per operator decision in `feedback_click_to_follow_id_resolution.md`.
- **robot_api/ axes-only Capabilities contract untouched.** Verified via `git diff --name-only HEAD~1 -- robot_follow/servers/follow_server.py robot_follow/robot_api/` (empty).
- **Vite production build green.** `npm run build` exits 0 with "built in 72ms" — the JSX is syntactically valid and React-compiles.

## Task Commits

1. **Task 1: Add detectionsRef mirror + rewrite handleFollow + change onClick payload in App.jsx** — `596ff63` (fix)

The four edits landed in one commit per the plan's explicit pathspec strategy:
- Edit 1: `detectionsRef = useRef([])` declared alongside `targetBoxesRef` / `renderedBoxesRef`.
- Edit 2: SSE `onmessage` mirrors `data.detections || []` to `detectionsRef.current` immediately after `setDetections(...)`.
- Edit 3: `handleFollow` rewritten to take the clicked detection, loop `detectionsRef.current` picking nearest bbox center, and POST `pick.id` (defaults to `clickedDet` if no live match — same 404 outcome as today, no new failure mode).
- Edit 4: `onClick={hasId ? () => handleFollow(det) : undefined}` (was `handleFollow(det.id)`).

## Files Created/Modified

- `robot_follow/ui/src/App.jsx` — +26 / −3 lines. detectionsRef mirror + handleFollow nearest-center resolver + onClick payload swap.

## Verification Results

### Source-level grep checks (deterministic)

| Check | Expected | Actual |
|---|---|---|
| `grep -c "detectionsRef" robot_follow/ui/src/App.jsx` | ≥ 3 | **3** |
| `grep "for (const d of latest)" robot_follow/ui/src/App.jsx` | ≥ 1 match | **1 match at line 301** |
| `grep "handleFollow(det)" robot_follow/ui/src/App.jsx` | ≥ 1 match | **1 match at line 709** |
| `grep "handleFollow(det.id)" robot_follow/ui/src/App.jsx` | 0 matches | **0 matches** |

### Untouched-invariant checks (post-commit)

```
$ git diff --name-only HEAD~1 -- robot_follow/servers/follow_server.py robot_follow/robot_api/
(empty)
$ git diff --name-only -- robot_follow/ui/package.json robot_follow/ui/package-lock.json
(empty)
```

`follow_server.py` and `robot_api/` are byte-identical pre/post-plan. No npm dependencies added.

### Vite production build

```
$ cd robot_follow/ui && npm run build
> drone-follow-ui@1.0.0 build
> vite build

vite v8.0.10 building client environment for production...
✓ 15 modules transformed.
build/index.html                   0.40 kB │ gzip:  0.27 kB
build/assets/index-ClJvVgYB.css    3.48 kB │ gzip:  1.07 kB
build/assets/index-BU8eLEV9.js   156.20 kB │ gzip: 49.78 kB
✓ built in 72ms
```

Build artifacts under `robot_follow/ui/build/` are gitignored (not committed).

### Pytest baseline preserved

```
$ source setup_env.sh && python -m pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py -q
293 passed, 1 skipped in 16.85s
```

Sanity-only — no Python files modified by this plan. Pre-edit and post-edit counts match (293/1).

**Note on baseline drift from 03-12-SUMMARY.md:** the 03-13 plan documents the pre-plan baseline as 290 passed / 7 skipped. The actual baseline at the start of this execution was 293 passed / 1 skipped — three previously-skipped tests now run / pass. The intervening commits (`3c38b09 test(03-13)`, `72add07`, `da7264d`) added or unblocked tests since 03-12-SUMMARY.md was written. The contract the plan cares about — "no regression from the immediately-pre-plan state" — holds: pre-edit and post-edit are both 293/1. Documented here so the verifier doesn't read 293/1 as a discrepancy.

### Console-script sanity

```
$ robot-follow --help → exit 0
$ drone-follow  --help → exit 0
```

Both aliases still resolve to the venv binary and parse args cleanly.

## Decisions Made

- **Client-layer fix, not server-layer.** Per operator decision in `~/.claude/projects/-home-guyz-code-guyz-hailo-drone-follow/memory/feedback_click_to_follow_id_resolution.md`: the race lives on the client (React closure capture), so the fix belongs on the client; the server stays strict (id-not-in-available → 404).
- **Position-based nearest-center resolver, no distance threshold.** Single-actor SITL flow that surfaced F3 has only one bbox candidate; nearest-center is unambiguous. Multi-actor IoU-threshold resolver is deferred (not in scope for F3).
- **Inline resolver — not extracted to a separate module.** The resolver is ~10 lines and tightly coupled to React state (`detectionsRef.current`). Extracting it would add friction with no testing payoff (no JS test framework). Per plan `<interfaces>` rationale.
- **No new failure modes by construction.** If `detectionsRef.current` is empty, `pick` stays the original `clickedDet` — server returns 404 as it would have today. Existing call-site guard (`hasId ? ... : undefined`) prevents `handleFollow` invocation for id-less detections.

## Deviations from Plan

None — plan executed exactly as written. The four edits in Task 1 landed verbatim; the verification suite passed without surprises.

## Issues Encountered

- **Pre-existing commit `72add07` ("server-side widening")** is still on `feature/rover-support` and modifies `follow_server.py`. It predates the plan rewrite (commit `da7264d`) that re-scoped 03-13 to a client-only fix. Per the plan's own acceptance check (`git diff HEAD~1 -- follow_server.py robot_api/` is empty), my client-only edit satisfies the plan contract — that check compares against the plan-rewrite commit, not the rejected server commit. **No action taken here:** reverting the rejected server-side widening is OUT OF SCOPE for 03-13 (the plan rewrites the goal, not the history). Flagged for orchestrator/operator visibility — if the rejected widening should be reverted before the operator gate, that's a separate cleanup plan.
- **Baseline drift from 03-12-SUMMARY.md** (290/7 → 293/1) is documented above; no regression introduced by this plan.

## Threat Surface Scan

No new security-relevant surface introduced. The `App.jsx` edit reads from an SSE stream that's already broadcast and POSTs to an endpoint that already exists with the same strict contract. The four STRIDE threats in the plan's `<threat_model>` (T-03-13-01..05) are all `accept` or `mitigate` with no new mitigations required at the source level — operator visibility via the existing follow_server INFO log covers T-03-13-04.

## F3 Acceptance Status

- **Source-level acceptance:** PASSED (grep checks + vite build green + untouched-invariant verified).
- **End-to-end acceptance:** DEFERRED to plan 03-14 (operator SITL re-gate). The 03-12-SUMMARY verifier note explicitly directs the verifier to keep `human_needed` raised against ABS-11 until 03-14-SUMMARY records `approved`.

## Next Phase Readiness

- **Plan 03-14 (operator SITL re-gate) is ready to schedule.** It re-runs `walk_across_then_approach` SITL with the operator clicking the visible actor bbox; success criterion is "lock takes; console no longer emits `Detection ID N not found. Available: {M}`".
- **Phase 3 does NOT close until 03-14 records operator approval.** Per the carry-forward `human_needed` flag against ABS-11 from 03-12.
- **Orchestrator should NOT advance STATE.md/ROADMAP.md beyond this plan's per-plan slot.** Per the executor's prompt context: worktree mode is disabled, the per-plan STATE update is fine, but the phase-completion advance is the orchestrator's responsibility after the operator gate closes.

## Self-Check: PASSED

Verified post-write:
- `robot_follow/ui/src/App.jsx` exists and contains all four edits (grep checks above).
- Commit `596ff63` exists in `git log`:
  ```
  $ git log --oneline | grep 596ff63
  596ff63 fix(03-13): client resolves bbox current id at click time (F3)
  ```
- `git diff --name-only HEAD~1 -- robot_follow/servers/follow_server.py robot_follow/robot_api/` is empty post-commit.

---
*Phase: 03-abstraction*
*Completed: 2026-05-19*
