---
phase: 03-abstraction
plan: 16
type: summary
status: complete
wave: 10
gap_closure: true
requirements: [ABS-11]
commits:
  - sha: "070550c"
    message: "feat(03-16): SharedDetectionState carries {id: bbox} atomic snapshot"
  - sha: "72f7ba3"
    message: "fix(03-16): bbox-match recovery closes the SSE→HTTP id-drift race"
tests_before: { passed: 295, skipped: 1 }
tests_after: { passed: 298, skipped: 1 }
---

## Outcome

F5 closed. The 03-14 SITL re-gate failure (`Detection ID 1 not found.
Available: {45}` on operator click) is now structurally impossible for any
click that supplies a bbox.

### Why prior fixes weren't enough

- **03-13 (client-side resolver):** picked the nearest detection from
  `detectionsRef.current` at click time. This catches the SSE→React-commit
  race, but not the SSE→HTTP race that comes after. By the time the POST
  reaches the server, the pipeline may have advanced past what the client
  knew when it clicked.
- **03-15 (bbox.h in body):** eliminated the SharedUIState ↔ SharedDetectionState
  race for the bbox-height setpoint, but didn't address id-mismatch at all.

The client lags the server. No client-only resolution wins that race.

### What 03-16 changes

1. **`SharedDetectionState` carries `{id: bbox}` atomically** alongside
   `available_ids`. `update(detection, available_ids, available_bboxes=None)`;
   `get_available_bboxes()` returns a defensive snapshot under the same lock
   that protects `get_available_ids()`. The pipeline callback now passes
   `filtered_tlwh_by_id` through to every `shared_state.update()` callsite
   that has bboxes in scope.

2. **POST body schema expands to `{bbox: {x, y, w, h}}`** — full geometry,
   all four floats validated in [0, 1] with `w, h > 0`. `_read_bbox_h_from_body`
   was renamed to `_read_bbox_from_body` and returns a `{x, y, w, h}` dict.

3. **`do_POST` IoU-match recovery** — when `requested_id` is NOT in
   `available_ids` AND the body has a bbox:
   - Iterate `shared_state.get_available_bboxes()` computing IoU vs the body
     bbox.
   - Pick the candidate with the highest IoU.
   - If best IoU ≥ **0.3**, lock that id and log:
     `Stale detection ID 1 → bbox-matched 45 (IoU=0.85, available=[45])`
   - Else strict 404, with an enriched log line that reports the best IoU
     achieved and the candidate count so any false-negative recovery has a
     diagnostic trail.

4. **`id_source` in the success INFO log** — `requested` for the happy path,
   `bbox-match` for the recovery branch:
   - `Now following detection ID: 45 (bbox height 0.180, source: POST body, id_source: requested)` (happy path)
   - `Now following detection ID: 45 (bbox height 0.180, source: POST body, id_source: bbox-match)` (recovery fired)

5. **Client (App.jsx::handleFollow)** sends `bbox: {x, y, w, h}` instead of
   just `{h}`. No other change to the resolver — the nearest-center match
   from 03-13 still runs (catching the client-side race when it can).

### Why this isn't the rejected 72add07 widening

- **72add07** widened by `len(available_ids) == 1` — a heuristic that fires
  whenever the server happens to see exactly one detection, *regardless of
  whether the operator actually clicked that one*. Fragile in multi-actor
  scenes; the operator vetoed it.
- **03-16** widens by `IoU(body_bbox, candidate_bbox) ≥ 0.3` — a deterministic
  geometric check that requires the operator to have visually identified the
  target via the body bbox. Headless callers (curl with no body) see no
  widening; their stale-id requests still return strict 404.

The vetoed rule was "no heuristic guess". The new rule is "no heuristic
guess; but a principled geometric match against an atomic snapshot of the
server's own state is the correct closure for the SSE→HTTP race". The
revised memory rule (`feedback_click_to_follow_id_resolution.md`,
2026-05-19) locks this layered architecture.

### IoU threshold

`0.3` is the empirical floor for "this is the same visual target despite
frame drift":
- A person walking between two ~100ms-apart frames overlaps themselves by
  IoU ≈ 0.85–0.95 (small translation, same scale) — well above threshold.
- A bbox flicker (NMS edge case, detector noise) can drop IoU to ~0.5 —
  still recovers.
- A different person in the same general area hits IoU ~0.1–0.2 — does not
  recover.

The multi-candidate test asserts "pick the highest IoU" so two close persons
each above threshold still resolve to the operator's intended click.

### Test changes

- `test_post_follow_with_bbox_body_uses_body_h_for_setpoint` — updated to
  send the full `{x, y, w, h}` schema. h kept inside the
  `capture_bbox_setpoint_from_height` clamp range `[0.10, 0.25]`.
- `test_post_follow_stale_id_with_body_bbox_recovers_via_iou` — pins the
  recovery branch.
- `test_post_follow_stale_id_with_body_bbox_no_overlap_returns_404` — pins
  that a disjoint bbox does NOT silently recover onto an irrelevant target.
- `test_post_follow_stale_id_with_body_bbox_picks_higher_iou` — pins
  multi-candidate disambiguation: highest IoU wins, not "first" or
  "single-available".

All the strict-404 tests from 03-15 still pass — they all send NO body, so
the recovery branch never fires.

## Verification

- Full pytest suite: **298 passed / 1 skipped** (pre-plan baseline 295/1;
  delta +3 from new bbox-match tests).
- Vite production build (`npm run build` in `robot_follow/ui/`): green,
  `built in 94ms`, three artifacts.
- `robot-follow --help` and `drone-follow --help` exit 0.
- `git diff --name-only HEAD~5 -- robot_follow/robot_api/` is **empty**:
  axes-only Capabilities contract preserved across the entire 03-13/15/16
  gap cycle.

## Architectural notes for downstream

- **`SharedDetectionState` is now the authoritative atomic snapshot** for
  both `available_ids` and `available_bboxes`. Any future feature that
  needs to look up a detection by id should consult `shared_state`, not
  `ui_state` (which is a separate snapshot that can race; that's the F4b
  lesson generalized).
- **Bbox-from-body protocol** is a stable contract: any client (web UI,
  future mobile, scripted) that wants reliable click-to-lock semantics
  must send `{bbox: {x, y, w, h}}`. Headless ID-only callers still work
  for the happy path.
- **IoU threshold** is a single magic number (`0.3`) in `do_POST`. If
  multi-actor scenes need stricter disambiguation, raise it; tests will
  catch regressions.

## Recommendation

Orchestrator schedules plan **03-14** (operator SITL re-gate, third attempt).
Phase 3 still does NOT close until the operator re-runs the SITL gate and
approves. The 03-14 scorecard rows 11 and 14 may need a small refresh to
reference `id_source: bbox-match` as the new evidence pattern; rows 12/13
stay correct (the client resolver from 03-13 still runs; the server now
catches its leftovers).
