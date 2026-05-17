---
phase: 3
slug: abstraction
status: draft
nyquist_compliant: false
wave_0_complete: false
created: 2026-05-17
---

# Phase 3 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution. Derived from `03-RESEARCH.md` § Validation Architecture. Phase 3 is the critical gate — full automated coverage of ABS-01..11 plus one operator-witnessed SITL run before phase close.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | pytest (venv at `./hailo-apps/venv_hailo_apps`) |
| **Config file** | none — pytest uses default discovery |
| **Quick run** | `python -m pytest robot_follow/tests/test_controller.py robot_follow/tests/test_mavsdk_drone_adapter.py robot_follow/tests/test_robot_command_snapshot.py -x -q` (~3 s) |
| **Full suite** | `python -m pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py -x -q` (~13 s; matches Phase 2 baseline) |
| **Phase 1 baseline carried forward** | Always `python -m pytest`, never bare `pytest`. Sim tests skip cleanly without Hailo HW. |
| **Operator SITL gate** | `bash sim/start_sim.sh --bridge --world walk_across_then_approach` + `drone-follow --robot drone --input udp://0.0.0.0:5600 --takeoff-landing --webui` — manual; `human_needed` until operator approves |

---

## Sampling Rate

- **Per task commit** (pre-commit): quick suite — `~3 s`.
- **Per wave merge** (pre-push): full suite — `~13 s`.
- **Before `/gsd:verify-work` / phase close**:
  1. Full suite green.
  2. **Snapshot test green** — `test_robot_command_snapshot.py` shows ~100 cases passing; drone behavior byte-equivalent pre/post refactor.
  3. **Operator SITL run** — walks drone through `walk_across_then_approach` world; visually confirms target-follow behavior is unchanged. Marked `human_needed` in verifier until approval.
- **Max feedback latency**: 3 s quick · 13 s full.

---

## Per-Task Verification Map

Sourced verbatim from `03-RESEARCH.md` § Phase Requirements → Test Map. Every ABS-* requirement has an automated command. ABS-11 (full SITL) is the single manual-only step.

| Req ID | Behavior | Test Type | Automated Command | Status |
|--------|----------|-----------|-------------------|--------|
| ABS-01 | `robot_api/robot.py` Robot protocol; types in `follow_api/types.py` | unit | `python -c "from robot_follow.follow_api.types import Axis, Capabilities, RobotCommand, SafetyContext; from robot_follow.robot_api.robot import Robot"` + `test_robot_protocol_shape.py` | ⬜ |
| ABS-02 | `RobotCommand` field shape; `VelocityCommand` removed | unit | `python -m pytest robot_follow/tests/test_robot_command_shape.py -v` (renamed from `test_velocity_command_shape.py`) | ⬜ |
| ABS-03 | `robot_api/adapters/mavsdk_drone.py` exists; `drone_api/mavsdk_drone.py` does NOT | unit | `python -c "import robot_follow.robot_api.adapters.mavsdk_drone"` succeeds AND `python -c "import robot_follow.drone_api" 2>&1` raises ModuleNotFoundError; `test_layout_smoke.py` | ⬜ |
| ABS-04 | Altitude P-loop gates on `Axis.ALTITUDE in caps.axes` | unit | `python -m pytest robot_follow/tests/test_mavsdk_drone_adapter.py::TestApplyAltitudeP -v` (pure-function extract) | ⬜ |
| ABS-05 | Bottom-edge frame safety: controller emits 0; adapter applies retreat | unit | `python -m pytest robot_follow/tests/test_mavsdk_drone_adapter.py::TestApplyRetreatFromTilt -v` + `test_controller.py::TestFrameEdgeBoundary` | ⬜ |
| ABS-06 | Search-mode yaw-spin lives in adapter; controller returns None on lost | unit | `python -m pytest robot_follow/tests/test_mavsdk_drone_adapter.py::TestComputeSearchYawspeed -v` | ⬜ |
| ABS-07 | `ControllerConfig` altitude fields are Optional; validate skips when ALTITUDE absent | unit | `python -m pytest robot_follow/tests/test_config_persistence.py::TestAltitudeOptional -v` | ⬜ |
| ABS-08 | `run_robot()` exists; `run_drone()` does not | unit | `python -c "from robot_follow.robot_follow_app import run_robot"` succeeds; `hasattr(module, 'run_drone')` is False | ⬜ |
| ABS-09 | `--robot drone --help` shows drone flags; `--robot rover --help` does not | smoke | `test_cli_help_dispatch.py` runs `robot-follow --robot {drone,rover} --help` via `subprocess.run` and asserts flag presence/absence | ⬜ |
| ABS-10 | `setup_env.sh` sources ROS when `/opt/ros/humble/setup.bash` exists; venv-first ordering | smoke | `test_setup_env_sh.py` sources `setup_env.sh` in a sub-shell, asserts `$ROS_DISTRO=humble` IFF ROS installed; file-based check works without ROS | ⬜ |
| ABS-11 | Full SITL drone follow-the-person passes with `--robot drone` | **manual** | Operator runs sim + drone-follow; observes target-follow for the full walk pattern. `human_needed` per Phase 1 Verification A pattern | ⬜ |
| **Phase-3 snapshot gate** | Drone command sequence byte-equivalent pre/post refactor | unit | `python -m pytest robot_follow/tests/test_robot_command_snapshot.py -v` — ~100 cases pass | ⬜ |
| **ROS env-leak verification** (DESIGN-NOTES risk #8) | Drone-follow behavior unchanged with vs without ROS sourcing | smoke | On a ROS-equipped operator box: `env > /tmp/before; source /opt/ros/humble/setup.bash; env > /tmp/after; diff /tmp/before /tmp/after | wc -l` AND run drone smoke before+after sourcing; outputs match | ⬜ (operator-side; documented in 03-RESEARCH § ROS env-leak verification test) |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

Wave 0 lands the test infrastructure BEFORE the structural refactor so each subsequent commit can be gated against it. Per the planner's commit-shape proposal, this is essentially commit 1.

- [ ] **`robot_follow/tests/test_robot_command_snapshot.py`** (NEW) — ~100 cases covering target-centered, target-offset L/R, bbox-too-small, bbox-too-large, bbox-at-top-margin, bbox-at-bottom-margin, bbox-in-fade-zone, emergency-safety, detection=None+various last_detection. Each case: input Detection, expected baseline `VelocityCommand` (captured pre-rewrite), expected post-rewrite `(RobotCommand + adapter outputs)` equivalence.
- [ ] **`robot_follow/tests/cases/__init__.py` + `cases/drone_command_baseline.py`** (NEW) — fixture file with `BaselineCase` dataclass entries (typed; Python `.py` not `.json` for type-checked baseline values).
- [ ] **`robot_follow/tests/test_mavsdk_drone_adapter.py`** (NEW) — unit tests for the R5 pure-function extracts:
  - `TestApplyAltitudeP` — altitude-hold correction math (gated on `Axis.ALTITUDE in caps.axes`)
  - `TestApplyRetreatFromTilt` — bottom-margin fade + retreat overlay; early-returns on `safety_ctx.target_lost`
  - `TestApplySmoothing` — EMA smoothing; migrated from `test_velocity_api_and_smoother.py` (46 tests)
  - `TestComputeSearchYawspeed` — search direction from last bbox side
  - `TestMavsdkDroneAdapterIntegration` — instantiate adapter with mock MAVSDK system; orchestrate the pure functions in `send_command`
- [ ] **`robot_follow/tests/test_robot_protocol_shape.py`** (NEW) — asserts `Robot` has 6 methods (`connect`, `start_session`, `send_command`, `send_zero`, `on_target_lost`, `shutdown`) + `caps` attribute; asserts `MavsdkDroneAdapter` implements all of them (ABS-01).
- [ ] **`robot_follow/tests/test_layout_smoke.py`** (NEW) — asserts `robot_api.adapters.mavsdk_drone` imports cleanly + `drone_api` module no longer exists (ABS-03).
- [ ] **`robot_follow/tests/test_cli_help_dispatch.py`** (NEW) — runs `subprocess.run([..., "--robot", "drone", "--help"])` and `["--robot", "rover", "--help"]`; asserts `--takeoff-landing` appears in drone-help and is absent from rover-help. ABS-09 lock.
- [ ] **`robot_follow/tests/test_setup_env_sh.py`** (NEW) — sources `setup_env.sh` in a sub-shell, asserts ROS env vars present iff `/opt/ros/humble/setup.bash` exists. File-existence path testable without ROS on this dev box.
- [ ] **Rename `test_velocity_command_shape.py` → `test_robot_command_shape.py`** (in-place rewrite, ABS-02).
- [ ] **Migrate `test_velocity_api_and_smoother.py`** (46 tests) → `test_mavsdk_drone_adapter.py::TestApplySmoothing` (functional equivalence; the smoothing logic relocates from `VelocityCommandAPI.send` to `_apply_smoothing` pure function).
- [ ] **Migrate 49 `compute_velocity_command(` call sites + 54 `VelocityCommand(...)` constructions** in `test_controller.py` to the new signature (`compute(det, caps, config)` returning `RobotCommand`). Per research: 103 mechanical edits total. **Atomic commit** like CLEAN-07's pattern.

No new framework install required. No new conftest changes.

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| Full SITL drone follow-the-person passes with `--robot drone` | ABS-11 | Requires Hailo HW + Gazebo + GPU + actor sim — operator-witnessed | `bash sim/start_sim.sh --bridge --world walk_across_then_approach` (Terminal 1); `source setup_env.sh && drone-follow --robot drone --input udp://0.0.0.0:5600 --takeoff-landing --webui` (Terminal 2); confirm drone follows actor for the full walk pattern without losing the target |
| ROS env-leak verification on a ROS-equipped dev box | DESIGN-NOTES risk #8 | Requires `/opt/ros/humble` installed | On a ROS-equipped box: capture `env` before/after sourcing ROS; diff; run drone-follow smoke before+after; confirm behavior identical |
| Two-tab CLEAN-16 carry-over | Phase 2 carry-over | Browser concurrency | Should still pass after Phase 3 (no SSE changes); operator confirms with `drone-follow --robot drone --input usb --webui` + two tabs |

---

## Validation Sign-Off

- [ ] All ABS-01..11 requirements have an automated `<verify><automated>` command OR a documented manual step (ABS-11)
- [ ] Sampling continuity: quick suite (~3 s) covers every modified module group (`follow_api/`, `robot_api/`, `robot_follow_app.py`)
- [ ] Wave 0 deliverables landed (~8 new test files + 1 rename + 2 migrations)
- [ ] No watch-mode flags
- [ ] Feedback latency `< 3 s` quick · `< 13 s` full
- [ ] `python -m pytest` invocation discipline maintained
- [ ] Snapshot test green pre-Phase-3-close; archive plan documented (Phase-3-only artifact per R5)
- [ ] Operator SITL run signed off (or explicitly deferred with mitigation, per Phase 1 / Phase 2 pattern)
- [ ] `nyquist_compliant: true` set in frontmatter after planner reviews and signs off

**Approval:** pending
