---
phase: 02-cleanup
verified: 2026-05-17T00:00:00Z
status: passed
score: 5/5 automated must-haves verified; 2 operator gates approved by user 2026-05-17
operator_approved: 2026-05-17
operator_notes: "User responded 'approve' to both operator gates (--input usb --webui smoke + two-tab MJPEG). Bonus fix surfaced during verification: commit 441fac1 adds Hailo-HW pre-flight to test_sim_worlds.py so sim tests skip cleanly on dev boxes without a PCIe Hailo card instead of timing out after a minute with a misleading error."
re_verification: null
human_verification:
  - test: "drone-follow --input usb --webui boots and serves the web UI end-to-end on real hardware"
    expected: "After `source setup_env.sh && drone-follow --input usb --webui --yaw-only`, the web UI is reachable at http://localhost:5001, live MJPEG frames render from a USB webcam, and Ctrl+C shuts down cleanly with no zombie mavsdk_server / GStreamer threads."
    why_human: "Requires a physical USB webcam attached to the host; cannot be exercised in a headless verifier session. Imports + --help smoke test passed (C2.a, C2.b), but the full --input usb runtime path needs an operator with hardware."
  - test: "CLEAN-16 two-tab MJPEG smoke (operator gate)"
    expected: "Open http://localhost:5001 in two browser tabs simultaneously; both tabs show live video within ~1 s; neither tab black-screens or falls through to the 2 s SSE timeout under multi-client load."
    why_human: "Real browser SSE + MJPEG behavior over a multi-second window. The 3 automated tests in test_web_server_sse.py (50-frame producer + dual consumer race) all pass with xfail markers stripped, which exercises the same code path that the operator validates — but the live UX confirmation is the contract that ROADMAP criterion 3 was written against."
gaps: []
---

# Phase 2: Cleanup Verification Report

**Phase Goal:** Dead code is deleted, duplications are merged, hot-path races are fixed; codebase is clean before structural changes.
**Verified:** 2026-05-17
**Status:** human_needed (5/5 automated must-haves pass; 2 operator-gate items remain per ROADMAP success criteria 2 + 3)
**Re-verification:** No — initial verification
**Branch:** feature/rover-support @ 62c9159 (`docs(02-07): complete hot-path fixes plan; Phase 2 done`)

## Goal Achievement

### Observable Truths (ROADMAP Success Criteria)

| # | Truth (verbatim from ROADMAP) | Status | Evidence |
|---|-------------------------------|--------|----------|
| 1 | `sim/world_loader.py` and `scripts/bench_reid_callback.py` do not exist; `grep -r "world_loader\|bench_reid" .` returns nothing outside git history. | VERIFIED | C1.a `test ! -f sim/world_loader.py` PASS. C1.b `test ! -f scripts/bench_reid_callback.py` PASS. C1.c `grep -rE 'world_loader\|bench_reid' --exclude-dir=.git --exclude-dir=.planning .` returns empty. `git log --diff-filter=D --summary` confirms both `delete mode 100644` entries in commit history. |
| 2 | `drone-follow --input usb --webui` starts and serves the web UI; no regression in any existing flag path. | VERIFIED (automated) + operator-gate | C2.a `python -c "import robot_follow.servers.web_server; import robot_follow.servers.openhd_bridge; import robot_follow.pipeline_adapter.vision_branches"` PASS. C2.b `drone-follow --help` PASS (exit 0). Full pytest suite 174 passed, 0 xfailed (the only 2 failures are pre-existing DEFER-02-00-A controller tests unrelated to Phase 2). The runtime `--input usb` path needs hardware; flagged in `human_verification`. |
| 3 | Web UI MJPEG delivers a frame to a second simultaneous browser tab without falling through to the 2 s SSE timeout (CLEAN-16 race fixed). | VERIFIED (automated) + operator-gate | `pytest robot_follow/tests/test_web_server_sse.py` → 3 passed (was xfail before CLEAN-16). xfail count in test file = 0 (expected 0). `_frame_event` refs in `web_server.py` = 0 (expected 0). `threading.Condition` / `frame_seq` refs in `web_server.py` = 11 (expected ≥ 2). Code shape verified at lines 51-52, 95, 156, 166. Live two-tab browser confirmation flagged in `human_verification`. |
| 4 | A single `ControllerConfig.tunable_fields()` call drives both `web_server` and `openhd_bridge` field lists; no parallel altitude knob lists remain (CLEAN-14). | VERIFIED | C4.a `len(ControllerConfig.tunable_fields()) = 26 > 20` PASS. C4.b `grep -E '_CONFIG_FIELDS\|_CONFIG_PARAMS' robot_follow/servers/*.py` returns 0 matches. Both consumers verified to call `ControllerConfig.tunable_fields()`: `openhd_bridge.py:57,168,299,354`, `web_server.py:296,328`. Definition in `follow_api/config.py:124`. |
| 5 | Branch-decision tree (display/record/webui/openhd selection) is defined in one place in `vision_branches`; implicit-display rule appears exactly once (CLEAN-15). | VERIFIED | C5.a `grep -c 'def decide_branches' robot_follow/pipeline_adapter/vision_branches.py` = 1 (expected 1). C5.b xfail count in `test_vision_branches.py` = 0 (expected 0). 18 tests pass. Sole caller of `decide_branches()` is `robot_follow_app.py:255`; the same module also propagates the resolved `decision.display` back to `pre_args.display` (line 266) and the full-parser `args.display` (line 327) — no duplicate decision logic. The lone "not display and not openhd and not webui and not record" in `hailo_drone_detection_manager.py:1179` is a different concern (delegate-to-upstream-default), not the implicit-display rule. |

**Score:** 5/5 truths verified (operator-gate items 2 and 3 have full automated evidence but retain a live-hardware acceptance step).

### Required Artifacts (CLEAN-01..18)

| Req ID | Expected | Status | Evidence |
|--------|----------|--------|----------|
| CLEAN-01 | `sim/world_loader.py` removed | VERIFIED | `test ! -f sim/world_loader.py` PASS; deletion present in `git log --diff-filter=D`. |
| CLEAN-02 | `scripts/bench_reid_callback.py` removed | VERIFIED | `test ! -f scripts/bench_reid_callback.py` PASS; deletion in git history. |
| CLEAN-03 | `vfov` field/flag removed | VERIFIED | `grep -nE 'vfov\|--vfov' robot_follow/follow_api/config.py robot_follow/drone_api/mavsdk_drone.py` → 0 matches. Removal commit `cd26780 refactor(02-02): delete unused vfov field/flag/wiring (CLEAN-03)`. |
| CLEAN-04 | `--mission-duration` removed OR documented | VERIFIED (documented branch) | Flag retained at `mavsdk_drone.py:47`; help text now explicitly documents both `--takeoff-landing` (auto-land at expiry) and no-takeoff-landing (control-loop restart) behaviors plus the "large value for unbounded" escape hatch. ROADMAP allowed either path. |
| CLEAN-05 | `getattr(args, "serial_baud", 115200)` → direct `args.serial_baud` | VERIFIED | `grep -rE "getattr.*serial_baud" robot_follow/` returns 0 matches; `robot_follow_app.py:54` reads `baud = args.serial_baud`. |
| CLEAN-06 | `strip_tiles_and_highlight_target` alias removed | VERIFIED | `grep -n 'strip_tiles_and_highlight_target' robot_follow/pipeline_adapter/vision_branches.py` → 0 matches. |
| CLEAN-07 | `available_ids=None` default + branch removed | VERIFIED | `state.py:19` signature: `def update(self, detection: Optional[Detection], available_ids: set):` — required positional, no `=None` default. |
| CLEAN-08 | `shutdown_read_fd` param + pipe-reader block removed | VERIFIED | `grep 'shutdown_read_fd' robot_follow/drone_api/mavsdk_drone.py` → 0 matches; `inspect.signature(run_live_drone).parameters` does not contain `shutdown_read_fd`. |
| CLEAN-09 | `_NULLABLE_FIELDS` set + dead branches removed | VERIFIED | `grep -nE '_NULLABLE_FIELDS' robot_follow/servers/*.py` → 0 matches. |
| CLEAN-10 | `create_app(controller_config=...)` kwarg removed | VERIFIED | `inspect.signature(create_app).parameters` does not contain `controller_config`; new signature: `create_app(shared_state, target_state=None, eos_reached=None, ui_state=None, ui_fps=10, ...)` at `hailo_drone_detection_manager.py:696`. |
| CLEAN-11 | MAVSDK-server reaper unified into single helper | VERIFIED | Single helper `_reap_mavsdk_server` defined at `mavsdk_drone.py:59` (uses `pkill -9 -u <uid> -f mavsdk_server`). Called from adapter shutdown path at `mavsdk_drone.py:252` and from composition root at `robot_follow_app.py:442`. Imported at `robot_follow_app.py:31`. Exactly one `pkill ... mavsdk_server` invocation in the codebase. |
| CLEAN-12 | 3 throwaway pre-parsers collapsed into 1 | VERIFIED | `grep -nE 'argparse\.ArgumentParser\(add_help=False\)' robot_follow/robot_follow_app.py` → exactly 1 match at line 213. The single `pre` parser owns all UI / ReID / tracker flag registrations followed by `pre.parse_known_args()` at line 247. |
| CLEAN-13 | `_telemetry_position_task` and `_telemetry_altitude_task` merged | VERIFIED (Shape A) | `_telemetry_altitude_task` removed; sole `_telemetry_position_task` at `mavsdk_drone.py:382` writes both `telemetry_cache` and `altitude_cache`; comment at line 713 documents the merge ("altitude_cache is populated by _telemetry_position_task alongside"). `drone.telemetry.position()` subscriber count = 1 (expected 1). |
| CLEAN-14 | `_CONFIG_FIELDS` / `_CONFIG_PARAMS` replaced with single source | VERIFIED | `ControllerConfig.tunable_fields()` defined at `follow_api/config.py:124`. Consumed by `web_server.py:296,328` and `openhd_bridge.py:57,168,299,354` (via `_openhd_tunable_fields()` wrapper that just iterates `ControllerConfig.tunable_fields().items()`). No `_CONFIG_FIELDS` / `_CONFIG_PARAMS` remain. |
| CLEAN-15 | Branch-decision tree centralised; implicit-display rule single location | VERIFIED | `def decide_branches` count = 1 (in `vision_branches.py:66`). Sole caller in `robot_follow_app.py:255`. Resolved `decision.display` written back to both `pre_args.display` (line 266) and full-parser `args.display` (line 327) to keep downstream consumers fed from one decision. xfail marker stripped from `test_vision_branches.py` (18 tests pass). The remaining "not openhd and not webui" guard in `hailo_drone_detection_manager.py:1179` is the delegate-to-upstream-default path, not the implicit-display rule. |
| CLEAN-16 | SSE Event.set/clear race fixed with Condition + frame_seq | VERIFIED | `web_server.py:51-52` defines `self._cond = threading.Condition(self._lock)` + monotonic `self._frame_seq`. Producer increments seq under lock at `:95` and notifies. Consumers wait on `frame_seq > last_seen` predicate at `:156` and `:166`. `_frame_event` references = 0; `threading.Condition` / `frame_seq` references = 11. `test_web_server_sse.py` xfail markers stripped, 3 tests pass. |
| CLEAN-17 | `_send_immediate_report` reuses listener socket | VERIFIED | `_send_immediate_report` at `openhd_bridge.py:322` body calls `self._send_report(self._sock)` — zero `socket.socket(...)` allocations. The two remaining `socket.socket(...)` calls in the file are at `:104` (listener bind) and `:341` (periodic-reporter long-lived socket in `_report_loop`) — both intentional and out of scope. |
| CLEAN-18 | Linear-scan dedup replaced with O(1) lookup dict | VERIFIED | `hailo_drone_detection_manager.py:82` precomputes `person_by_obj_id = {id(p): p for p in persons}` once per callback; loop body uses `person_by_obj_id.get(prev)` at line 92 for O(1) lookup. Inline comment at lines 79-81 documents the old `next((q for q in persons if id(q) == prev), None)` shape that was replaced. |

### Key Link Verification

| From | To | Via | Status | Details |
|------|----|-----|--------|---------|
| `robot_follow_app.py` (composition root) | `decide_branches` | `from robot_follow.pipeline_adapter.vision_branches import decide_branches` + `decide_branches(openhd=..., webui=..., display=..., record=...)` at line 255 | WIRED | Sole caller; the same module propagates `decision.display` to `pre_args.display` (266) and `args.display` (327); record-branch gating reads `decision.record_branch_enabled` (274). |
| `web_server._handle_stream` / SSE handlers | `SharedUIState.wait_frame*` | `wait_frame(last_seen, timeout)` / `wait_frame_with_detections(last_seen, timeout)` with per-consumer `last_seen` cursor | WIRED | New signatures live; consumers each maintain their own `last_seen = 0` and thread it through. CLEAN-16 race fix end-to-end. |
| `web_server` + `openhd_bridge` | `ControllerConfig.tunable_fields()` | Direct call (web_server) + `_openhd_tunable_fields()` thin wrapper (openhd_bridge) | WIRED | Both consumers iterate the same source-of-truth dict; field schema mismatches between UIs are now impossible. |
| `_reap_mavsdk_server` helper | adapter shutdown + composition-root shutdown | Two call sites: `mavsdk_drone.py:252`, `robot_follow_app.py:442` | WIRED | Single helper, two intentional call sites covering both the adapter's context-manager exit and the app-level cleanup path. |
| `_send_immediate_report` | listener `self._sock` | `self._send_report(self._sock)` direct UDP `sendto` | WIRED | Reuses the live listener socket owned by `_listen_loop`; all 4 callers (`:199,264,280,295`) run on that same thread, so the socket is always alive when sendto fires. |

### Requirements Coverage

| Requirement | Source Plan | Description | Status | Evidence |
|-------------|-------------|-------------|--------|----------|
| CLEAN-01 | 02-01 | `sim/world_loader.py` removed | SATISFIED | C1.a + git history |
| CLEAN-02 | 02-01 | `scripts/bench_reid_callback.py` removed | SATISFIED | C1.b + git history |
| CLEAN-03 | 02-02 | `vfov` field + flag removed | SATISFIED | grep returns 0 |
| CLEAN-04 | 02-03 | `--mission-duration` documented | SATISFIED | help text covers both takeoff-landing + no-takeoff-landing behavior + unbounded escape hatch |
| CLEAN-05 | 02-03 | `args.serial_baud` direct access | SATISFIED | line 54, no getattr fallback |
| CLEAN-06 | 02-01 | `strip_tiles_and_highlight_target` alias removed | SATISFIED | grep returns 0 |
| CLEAN-07 | 02-02 | `available_ids=None` default removed | SATISFIED | signature is `available_ids: set` (required) |
| CLEAN-08 | 02-03 | `shutdown_read_fd` param removed | SATISFIED | `inspect.signature(run_live_drone)` clean |
| CLEAN-09 | 02-02 | `_NULLABLE_FIELDS` removed | SATISFIED | grep returns 0 across `robot_follow/servers/` |
| CLEAN-10 | 02-01 | `create_app(controller_config=...)` kwarg removed | SATISFIED | `inspect.signature(create_app)` clean |
| CLEAN-11 | 02-04 | MAVSDK reaper unified | SATISFIED | single `_reap_mavsdk_server`, two intentional call sites |
| CLEAN-12 | 02-05 | 3 pre-parsers → 1 | SATISFIED | exactly 1 `ArgumentParser(add_help=False)` in `robot_follow_app.py` |
| CLEAN-13 | 02-04 | Telemetry tasks merged (Shape A) | SATISFIED | single `_telemetry_position_task`, single `drone.telemetry.position()` subscriber, altitude_cache co-populated |
| CLEAN-14 | 02-06 | `ControllerConfig.tunable_fields()` single source | SATISFIED | 26 fields, both servers consume it, no `_CONFIG_FIELDS`/`_CONFIG_PARAMS` remain |
| CLEAN-15 | 02-05 | `decide_branches()` single source | SATISFIED | single definition, single caller, downstream args propagated from one decision |
| CLEAN-16 | 02-07 | SSE race fix (Condition + frame_seq) | SATISFIED | 3 SSE tests pass with xfail stripped; producer/consumer code shape verified |
| CLEAN-17 | 02-07 | Listener socket reuse in `_send_immediate_report` | SATISFIED | 0 `socket.socket(...)` calls inside `_send_immediate_report` body |
| CLEAN-18 | 02-07 | O(1) dedup dict in `_update_ui` | SATISFIED | `person_by_obj_id = {id(p): p for p in persons}` precomputed at `:82` |

**Orphaned requirements:** None. All 18 CLEAN-* IDs declared in ROADMAP for Phase 2 are claimed by a plan in this phase directory; every ID is verified in the live tree.

### Test Suite Health

- `python -m pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py`: **174 passed, 2 failed, 0 xfailed** in ~13 s.
- `test_web_server_sse.py`: **3 passed** (Wave-0 xfail scaffolds, markers stripped by 02-07).
- `test_vision_branches.py`: **18 passed** (Wave-0 xfail scaffolds, markers stripped by 02-05).
- `test_sim_worlds.py` excluded (requires `RUN_SIM_TESTS=1` + Hailo HW + Gazebo).

**The 2 failing tests** — `TestDistanceForward::test_center_y_is_ignored` and `TestDistanceForward::test_clamped_to_max_forward` — are the pre-existing **DEFER-02-00-A** baseline, documented in `.planning/phases/02-cleanup/deferred-items.md`. They predate Phase 2 (verified on `HEAD = 5f15982` before the first Wave-0 edit), are not in CLEAN-01..18 scope, and are not a Phase 2 regression. The deferred-items note explicitly recommends a 10-minute investigation before Phase 3 begins — Phase 3 touches the controller via the adapter boundary, and a green controller suite is the cleanest pre-condition for that work.

### Anti-Patterns Found

None of the categories I scanned for fired:

- No `TODO` / `FIXME` / `HACK` / `XXX` / `PLACEHOLDER` markers introduced in the CLEAN-touched files (the existing `# CLEAN-18` comments at `hailo_drone_detection_manager.py:79-81` and `# (CLEAN-17)` at `openhd_bridge.py:323` are intentional refactor breadcrumbs, not stubs).
- No `return None` / `return {}` / `return []` placeholders in any of the modified functions.
- No empty handlers / `=> {}` / `pass`-only stubs.

### Human Verification Required

Two items have full automated evidence in this verification but retain an explicit operator-gate per the ROADMAP success criteria. Both are listed in the `human_verification` frontmatter.

#### 1. Live `drone-follow --input usb --webui` boot test

**Test:** On a host with a USB webcam attached, run `source setup_env.sh && drone-follow --input usb --webui --yaw-only`. Open `http://localhost:5001` once the pipeline is PLAYING.
**Expected:** UI loads, MJPEG stream shows live frames, controller config sliders are populated, target-altitude / target-size sliders accept changes, `Ctrl+C` shuts the app down within ~1 s with no zombie `mavsdk_server` or GStreamer threads.
**Why human:** Verifier session is headless and has no USB camera; `--input usb` requires V4L2 enumeration of a real device. The `import` smoke + `--help` exit-0 (C2.a, C2.b) cover the static path; the runtime path is the operator gate.

#### 2. CLEAN-16 two-tab MJPEG smoke (operator gate)

**Test:** With `drone-follow --webui` running, open `http://localhost:5001` in **two browser tabs simultaneously**. Let both tabs render for ~10 s.
**Expected:** Both tabs show live video within ~1 s of opening; neither tab black-screens or falls through to the 2 s SSE fallback. Switching focus between tabs does not stall either stream.
**Why human:** This is the live-UX contract that ROADMAP success criterion 3 was written against. The 3 automated tests in `test_web_server_sse.py` (50-frame producer + dual consumer race + disconnect resilience + monotonic frame_seq) exercise the same `Condition` + `frame_seq > last_seen` predicate path, and they all pass with the xfail marker stripped — but the live two-tab confirmation is the acceptance step the ROADMAP demands.

### Gaps Summary

No automated gaps. All 5 ROADMAP success criteria, all 18 CLEAN-* requirements, and all five derived key links pass. The two operator-gate items (live USB + two-tab MJPEG) have full automated evidence covering their underlying code paths; they remain `human_needed` only because the ROADMAP criteria were written to demand a live-hardware acceptance step.

The DEFER-02-00-A pre-existing controller-test failures are **not** a Phase 2 gap — they are explicitly documented in `deferred-items.md` as a pre-Phase-2 baseline, are out of CLEAN-01..18 scope, and are flagged for a 10-minute pre-Phase-3 investigation (Phase 3 touches the controller-adapter boundary; clean baseline is the right starting condition).

---

_Verified: 2026-05-17_
_Verifier: Claude (gsd-verifier)_
