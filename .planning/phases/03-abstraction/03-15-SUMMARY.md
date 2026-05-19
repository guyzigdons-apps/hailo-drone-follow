---
phase: 03-abstraction
plan: 15
type: summary
status: complete
wave: 10
gap_closure: true
requirements: [ABS-11]
commits:
  - sha: "5bd69b9"
    message: "revert(03-15): undo 72add07 server-side stale-id widening (wrong layer)"
  - sha: "4d03f69"
    message: "fix(03-15): /follow body carries bbox.h to bypass ui_state race (F4b)"
tests_before: { passed: 293, skipped: 1 }
tests_after: { passed: 295, skipped: 1 }
---

## Outcome

Two F-tier defects closed in two atomic commits.

### F4a — Server widening reverted

Commit `5bd69b9` removes the `if len(available_ids) == 1` branch and the
`Stale detection ID …; locking onto single available …` INFO log line from
`follow_server.do_POST`. The server returns to its pre-72add07 strict
contract: detection_id not in `available_ids` → 404 with the existing JSON
payload + INFO log + early return. No "guess intent" branch on the server.

The previously-shipped F3 widening lived in commit `72add07`; the operator's
review rejected it in favor of the client-side current-id resolver landed
in 03-13 (commit `596ff63`). This commit cleans up the rejected code that
was still on the branch.

Test changes:
- Deleted: `test_post_follow_stale_id_locks_onto_single_available`
- Added: `test_post_follow_stale_id_with_single_available_returns_404` — pins
  the strict-404 behavior so a future re-introduction of the widening
  triggers a failing test instead of a silent contract change.
- Preserved: the two existing F3 tests (`_with_zero_available_returns_404`,
  `_with_multiple_available_returns_404`).

### F4b — `bbox height n/a` fixed via body-carried bbox.h

Commit `4d03f69` adds an optional JSON body to `POST /follow/<id>` carrying
the bbox height of the detection the client resolved at click time:

```
Content-Type: application/json
{ "bbox": { "h": 0.180 } }
```

Server-side:
- New `_read_bbox_h_from_body` helper on `FollowServerHandler`. Defensive
  ladder: Content-Length missing/non-int/zero/>4096 → None; body decode
  error → None; JSON not a dict → None; no nested `bbox.h` → None; `h` not
  a finite float in (0, 1] → None. Every reject path degrades to the
  existing ui_state fallback; never raises out of the handler thread.
- `do_POST` success branch now tries the body first; on body present + valid
  + `controller_config` available, calls
  `capture_bbox_setpoint_from_height(self.controller_config, h, source="CLICK")`
  directly. Otherwise falls back to `_capture_bbox_for_id(detection_id)`.
- INFO log line now reports the bbox source:
  - `Now following detection ID: 45 (bbox height 0.180, source: POST body)`
  - `Now following detection ID: 45 (bbox height 0.200, source: ui_state)`
  - `Now following detection ID: 45 (bbox height n/a, source: n/a)`
  Any future "n/a" report now identifies whether the body was missing or
  the ui_state lookup failed.

Client-side:
`handleFollow` in `App.jsx` now sends
`Content-Type: application/json` + `body: JSON.stringify({ bbox: { h: pick.bbox.h } })`
using the resolved nearest-center detection from 03-13.

Test changes:
- `test_post_follow_with_bbox_body_uses_body_h_for_setpoint` — pins the
  body-aware path. Constructs a `ControllerConfig`, attaches it to the
  handler class, POSTs with a JSON body, asserts `target_bbox_height` in
  both the response JSON and the config object. Note: the body value must
  fall inside `capture_bbox_setpoint_from_height`'s clamp range
  `[0.10, 0.25]` (preserved from pre-plan behavior) — the test uses `0.18`.
- `test_post_follow_without_body_falls_back_to_ui_state_lookup` — pins the
  body-absent path. With no body, no ui_state, no controller_config, the
  request still returns 200 + `target_bbox_height: None`. Backward-compat
  for curl-style callers.

## Verification

- Full pytest suite: **295 passed / 1 skipped** (pre-plan baseline: 293/1;
  delta +2 from the two new tests).
- Vite production build (`npm run build` in `robot_follow/ui/`): green,
  built in 90ms, three artifacts (`index.html`, CSS bundle, JS bundle).
- `robot-follow --help` and `drone-follow --help` exit 0.
- `git diff --name-only HEAD~2 -- robot_follow/robot_api/` is **empty**:
  axes-only Capabilities contract preserved across both commits.

## Architectural notes for downstream

- The bbox-from-body protocol is narrow on purpose. Only `bbox.h` is parsed
  — `x`, `y`, `w` are ignored. Wider geometry can be added later if a future
  client needs to pin x/y for off-centre tracking; the server's tolerance
  ladder already rejects unknown fields by ignoring them.
- The clamp in `capture_bbox_setpoint_from_height` (`[0.10, 0.25]`) is the
  authoritative range for `target_bbox_height`; the server applies it
  uniformly whether the value comes from the body or from ui_state.
- The new INFO log `source: …` token is a recovery breadcrumb. If field
  operators ever see `source: n/a` again, it's specifically the case where
  ui_state and controller_config are both wired but the lookup raced — that
  narrows the bug surface vs the pre-plan "n/a" line which conflated multiple
  causes.

## Recommendation

Orchestrator schedules plan **03-14** (operator SITL re-gate). Phase 3 still
does NOT close until the operator re-runs the SITL gate and approves.

The 03-14 plan's text was written before 03-13 was rewritten and before
03-15 landed; the verifier should expect:
- F3 (stale click id) is closed at the client layer (03-13), not the server
  layer as originally documented in 03-14's <objective>.
- F4b (bbox n/a) is closed at the protocol layer (03-15) — the operator
  should look for `source: POST body` in the INFO log during the re-gate.
