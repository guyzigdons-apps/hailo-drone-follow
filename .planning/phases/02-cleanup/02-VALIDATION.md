---
phase: 2
slug: cleanup
status: draft
nyquist_compliant: false
wave_0_complete: false
created: 2026-05-14
---

# Phase 2 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution. Derived from `02-RESEARCH.md` § Validation Architecture.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | pytest 9.0.2 (system: `/home/guyz/.local/bin/pytest`) — but **always invoke as `python -m pytest`** (Phase 1 verification footgun: bare `pytest` resolves to system Python and doesn't see the venv's `robot_follow` package) |
| **Config file** | none — discovered via `robot_follow/tests/conftest.py` (adds repo root to `sys.path`) |
| **Test directory** | `robot_follow/tests/` (14 test files, 159 tests collected pre-Phase-2; Wave 0 adds 1–2 new files) |
| **Quick run command** | `python -m pytest robot_follow/tests/test_install_smoke.py robot_follow/tests/test_shared_state.py robot_follow/tests/test_config_persistence.py robot_follow/tests/test_velocity_command_shape.py -x` (~30 tests, < 2 s) |
| **Full suite command** | `python -m pytest robot_follow/tests -x --ignore=robot_follow/tests/test_sim_worlds.py` (~159 tests, < 30 s; sim test needs Hailo HW + Gazebo) |
| **Smoke gate (Phase 1 contract, still in force)** | `python -m pytest robot_follow/tests/test_install_smoke.py -v` (10 tests, < 2 s) |
| **Dev-loop integration smoke** | `drone-follow --input usb --webui --yaw-only` (ROADMAP criterion 2) |
| **CLEAN-16 acceptance** | Two-tab browser test against `http://localhost:5001` (manual; both tabs show live video, neither falls through SSE 2 s timeout) |

---

## Sampling Rate

- **After every task commit** (pre-commit): quick suite — `< 2 s`.
- **After every plan wave** (pre-push): full suite (excluding `test_sim_worlds.py` on non-Hailo hosts) — `< 30 s`.
- **Before `/gsd:verify-work` / phase close** (manual):
  1. Full suite green.
  2. `drone-follow --input usb --webui --yaw-only` boots, serves the web UI on :5001, video frames appear, `Ctrl+C` shuts down cleanly.
  3. **Two-tab CLEAN-16 smoke**: open `http://localhost:5001` in two browser tabs simultaneously; both show live video within 2 s; no black screen in either tab.
  4. (Optional, on Hailo host) `python -m pytest robot_follow/tests/test_sim_worlds.py` — end-to-end pipeline still works.
- **Max feedback latency**: 2 s (quick suite).

---

## Per-Task Verification Map

Sourced verbatim from `02-RESEARCH.md` § Phase Requirements → Test Map. Each CLEAN-* item has a concrete automated or shell-grep verification; CLEAN-15 and CLEAN-16 need new test files (Wave 0 deliverables).

| Req ID | Behavior | Test Type | Automated Command | Status |
|--------|----------|-----------|-------------------|--------|
| CLEAN-01 | `sim/world_loader.py` is gone | smoke | `test ! -f sim/world_loader.py && ! grep -r "world_loader" --exclude-dir=.git . \| grep -v world_loader.py` | ⬜ |
| CLEAN-02 | `scripts/bench_reid_callback.py` is gone | smoke | `test ! -f scripts/bench_reid_callback.py && ! grep -r "bench_reid" --exclude-dir=.git .` | ⬜ |
| CLEAN-03 | `vfov` field + flag absent | unit | `python -m pytest robot_follow/tests/test_config_persistence.py -v` (asserts no `vfov` attr on `ControllerConfig`) | ⬜ |
| CLEAN-04 | `--mission-duration` documented (NOT removed — research found it's load-bearing) | smoke | `drone-follow --help 2>&1 \| grep -i "mission-duration.*auto-land\|mission-duration.*timeout"` returns ≥ 1 line | ⬜ |
| CLEAN-05 | `_resolve_serial_connection` uses `args.serial_baud` directly | unit | `grep -n 'getattr(args, "serial_baud"' robot_follow/robot_follow_app.py` returns 0 lines | ⬜ |
| CLEAN-06 | `strip_tiles_and_highlight_target` alias absent | unit | `grep -n 'strip_tiles_and_highlight_target' robot_follow/pipeline_adapter/vision_branches.py` returns 0 lines | ⬜ |
| CLEAN-07 | `state.update` requires `available_ids` | unit | `python -m pytest robot_follow/tests/test_shared_state.py -v` (all 11 sites updated **in same commit** as signature change) | ⬜ |
| CLEAN-08 | `run_live_drone` has no `shutdown_read_fd` | unit | `python -c "import inspect; from robot_follow.drone_api.mavsdk_drone import run_live_drone; assert 'shutdown_read_fd' not in inspect.signature(run_live_drone).parameters"` | ⬜ |
| CLEAN-09 | `_NULLABLE_FIELDS` is absent | unit | `grep -n '_NULLABLE_FIELDS' robot_follow/servers/*.py` returns 0 lines | ⬜ |
| CLEAN-10 | `create_app` has no `controller_config` | unit | `python -c "import inspect; from robot_follow.pipeline_adapter.hailo_drone_detection_manager import create_app; assert 'controller_config' not in inspect.signature(create_app).parameters"` | ⬜ |
| CLEAN-11 | Single `_reap_mavsdk_server` helper | unit | `grep -cE 'pkill.*mavsdk_server' robot_follow/**/*.py` returns 1 (inside the helper only) | ⬜ |
| CLEAN-12 | Single pre-parser | unit | `grep -c 'ArgumentParser(add_help=False)' robot_follow/robot_follow_app.py` returns ≤ 1 | ⬜ |
| CLEAN-13 | Single position-stream subscription | unit | `grep -c 'drone.telemetry.position()' robot_follow/drone_api/mavsdk_drone.py` returns 1 line | ⬜ |
| CLEAN-14 | `ControllerConfig.tunable_fields` exists; both servers use it | unit + grep | `python -c "from robot_follow.follow_api.config import ControllerConfig; assert hasattr(ControllerConfig, 'tunable_fields') and len(ControllerConfig.tunable_fields()) > 20"` + `grep -c '_CONFIG_FIELDS\b\|_CONFIG_PARAMS\b' robot_follow/servers/*.py` returns 0 | ⬜ |
| CLEAN-15 | One `decide_branches` site (single source of truth) | new unit test | `python -m pytest robot_follow/tests/test_vision_branches.py::test_decide_branches -v` (Wave 0 lands as xfail; closes in Wave 2) | ⬜ |
| CLEAN-16 | Two-consumer SSE delivers without dropping | new unit + manual | `python -m pytest robot_follow/tests/test_web_server_sse.py -v` (Wave 0 lands as xfail; closes in Wave 3) + two-tab browser test | ⬜ |
| CLEAN-17 | `_send_immediate_report` does not create a new socket per call | grep | `grep -c 'socket.socket' robot_follow/servers/openhd_bridge.py` returns ≤ 2; `_send_immediate_report` body has 0 `socket.socket(` calls | ⬜ |
| CLEAN-18 | `_update_ui` has no `next((q for q in persons` linear scan | grep | `grep -c 'next((q for q in persons' robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` returns 0 | ⬜ |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

Wave 0 lands new test scaffolds **with `@pytest.mark.xfail`** so the test files exist as gates from Wave 1 onward; the xfail markers are stripped in the wave that lands the corresponding fix (Wave 2 for CLEAN-15, Wave 3 for CLEAN-16).

- [ ] **`robot_follow/tests/test_web_server_sse.py`** (NEW) — covers CLEAN-16 with two-consumer + monotonic-seq invariants. Test cases:
  - Two consumers each receive every frame within `2 s` of `_on_frame()` (no SSE timeout).
  - A consumer that disconnects mid-stream doesn't block the other.
  - `frame_seq` is monotonically increasing across consumers.
  - Initially marked `@pytest.mark.xfail(reason="CLEAN-16 race not yet fixed")`; Wave 3 strips the marker.
- [ ] **`robot_follow/tests/test_vision_branches.py`** (NEW or extension of existing) — covers CLEAN-15's `decide_branches()` over the 16-combo flag matrix (`--display × --record × --webui × --openhd`). Asserts the implicit-display rule appears exactly once (i.e., is computed only inside `decide_branches`). Initially marked `xfail`; Wave 2 strips the marker.
- [ ] **No conftest changes needed** — existing `conftest.py` path setup covers new files.
- [ ] **No test framework install needed** — pytest 9.0.2 already on PATH.

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| `drone-follow --input usb --webui --yaw-only` boots and serves the web UI | ROADMAP success criterion 2 | Needs USB webcam + browser; not pytestable | Run command; open `http://localhost:5001`; confirm video stream within 10 s; `Ctrl+C` shuts down cleanly with no orphan `mavsdk_server` |
| **Two-tab CLEAN-16 acceptance** | ROADMAP success criterion 3 | Browser concurrency behavior | While `--webui` is running, open `http://localhost:5001` in two browser tabs simultaneously. Both must show live video within 2 s; neither falls through to the SSE 2 s timeout (would manifest as black screen + reconnect) |
| `--mission-duration` semantics in `--help` | CLEAN-04 | Visual inspection of help text | `drone-follow --help \| less` — confirm the `--mission-duration` line explains auto-land/timeout behavior |
| Boot-service deployment (next field deployment) | None (carry-over from Phase 1) | Deployed hardware not present | Smoke step on the next time the air unit is rebooted: `systemctl status drone-follow-boot` shows the unit happy |

---

## Validation Sign-Off

- [ ] All CLEAN-01..18 requirements have an automated or grep verify (or a documented manual step for the 2 user-facing items)
- [ ] Sampling continuity: quick suite covers every modified file group (servers / pipeline_adapter / follow_api / drone_api)
- [ ] Wave 0 deliverables landed (two new test files with `xfail` markers)
- [ ] No watch-mode flags (pytest invoked with `-x` for fail-fast)
- [ ] Feedback latency `< 2 s` (quick suite); `< 30 s` (full suite)
- [ ] `python -m pytest` invocation discipline documented (Phase 1 footgun)
- [ ] CLEAN-16 two-tab manual acceptance documented and runnable in 1 minute
- [ ] `nyquist_compliant: true` set in frontmatter (after planner sign-off)

**Approval:** pending
