---
phase: 03-abstraction
plan: 04
subsystem: robot-api-scaffold
tags: [protocol, runtime-checkable, robot-api, scaffold, package-structure, orchestrator-skeleton]

# Dependency graph
requires:
  - phase: 03-abstraction
    provides: "Wave-2 types in follow_api/types.py (Axis, Capabilities, RobotCommand, SafetyContext, Detection) — 03-03 landed all 4 actuator-boundary types"
  - phase: 03-abstraction
    provides: "Wave-0 xfail scaffolds (03-02) — test_robot_protocol_shape.py awaiting the Robot protocol landing"
provides:
  - "robot_follow.robot_api package — actuator-boundary layer marker"
  - "robot_follow.robot_api.robot.Robot Protocol (6 methods + caps attribute), @runtime_checkable"
  - "robot_follow.robot_api.adapters empty subpackage — git mv target for 03-05"
  - "robot_follow.robot_api.orchestrator.run_robot_loop signature (body NotImplementedError until 03-08)"
affects: [03-05 mavsdk_drone git mv, 03-06 MavsdkDroneAdapter implementation, 03-07 controller migration to RobotCommand, 03-08 run_robot_loop body, 03-09 composition root]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Layer rule: robot_api may import from follow_api; follow_api may NEVER import from robot_api (R1 lock 2026-05-17)"
    - "@runtime_checkable Protocol — enables isinstance() checks for adapter-implements-Robot test in 03-06 without breaking duck-typing"
    - "TYPE_CHECKING imports + string forward-refs in orchestrator — keeps runtime import graph minimal until body lands"
    - "NotImplementedError skeleton — locks the signature contract so 03-05/03-06 can import orchestrator without circular-import pain; loud failure if accidentally called"
    - "Q5 lock documented in send_command docstring: cmd.yaw_rate is in caps.yaw_unit; adapter does NO unit conversion"
    - "Q6 lock documented in send_command docstring: adapter MUST early-return if safety_ctx.target_lost is True"

key-files:
  created:
    - "robot_follow/robot_api/__init__.py (14 lines) — package marker + Robot re-export"
    - "robot_follow/robot_api/robot.py (108 lines) — Robot Protocol class with 6 methods + caps annotation"
    - "robot_follow/robot_api/adapters/__init__.py (5 lines) — empty adapters subpackage marker"
    - "robot_follow/robot_api/orchestrator.py (48 lines) — run_robot_loop signature skeleton"
  modified: []

key-decisions:
  - "pyproject.toml NOT edited: existing `[tool.setuptools.packages.find] include = [\"robot_follow*\", ...]` glob already covers robot_follow.robot_api.* (verified via direct import). Per plan Task 2 step 2 'NO EDIT NEEDED if the glob is already robot_follow*'."
  - "@runtime_checkable decorator added (was implied but not in original CONTEXT shape). Required so 03-02's test_mavsdk_drone_adapter_implements_robot can use isinstance() in 03-06 without TypeError."
  - "TYPE_CHECKING imports for ControllerConfig/SharedDetectionState/Robot — runtime import graph stays minimal; orchestrator doesn't need to materialize these types until the body lands in 03-08."
  - "Per-task commits (2 commits, not 1 combined): plan verification text allowed combining if Task 2 no-op'd pyproject.toml, but per-task discipline kept the diff atomically bisectable. Task 1 = pure scaffold; Task 2 = orchestrator signature lock."

patterns-established:
  - "Pattern: skeleton-first orchestrator — land the signature before the body so dependents (composition root, CLI dispatch) can import and type-check against the contract"
  - "Pattern: @runtime_checkable on actuator-boundary Protocol — supports both static type-checking AND runtime structural conformance tests"
  - "Pattern: TYPE_CHECKING + string forward-refs for cross-layer signature types — keeps runtime imports minimal while preserving signature clarity"

requirements-completed: [ABS-01]

# Metrics
duration: 7min
completed: 2026-05-19
---

# Phase 03 Plan 04: Scaffold robot_api/ package with Robot protocol + orchestrator skeleton

**Actuator-boundary scaffold landed: `robot_follow.robot_api` package now exposes the `@runtime_checkable` `Robot` Protocol (6 methods + `caps` annotation), an empty `adapters/` subpackage ready for 03-05's `git mv`, and a `run_robot_loop` orchestrator signature with `NotImplementedError` body (full state machine lands in 03-08). Protocol-shape xfail test from 03-02 flips xpass as predicted; no pyproject.toml edit needed (existing `robot_follow*` glob covers the new package).**

## Performance

- **Duration:** ~7 min (412 s wall-clock from plan start)
- **Started:** 2026-05-19T10:36:29Z
- **Completed:** 2026-05-19
- **Tasks completed:** 2 of 2
- **Files created:** 4 (Python); 0 modified
- **Commits:** 2 per-task + 1 SUMMARY/state commit

## Goals Achieved

- [x] `robot_follow/robot_api/__init__.py` created; re-exports `Robot`
- [x] `robot_follow/robot_api/robot.py` defines `Robot` Protocol (6 methods: `connect`, `start_session`, `send_command`, `send_zero`, `on_target_lost`, `shutdown`) + `caps: Capabilities` annotation
- [x] `@runtime_checkable` decorator applied for 03-06's `isinstance` test
- [x] All actuator-boundary types imported from `follow_api/types.py` per R1
- [x] `robot_follow/robot_api/adapters/__init__.py` created (empty target for 03-05's `git mv`)
- [x] `robot_follow/robot_api/orchestrator.py` defines `run_robot_loop(robot, shared_state, config, shutdown)` — body raises `NotImplementedError` until 03-08
- [x] Q5 lock (yaw_unit pass-through) documented in `send_command` docstring
- [x] Q6 lock (target_lost early-return) documented in `send_command` docstring
- [x] pyproject.toml package discovery verified (no edit needed)
- [x] `test_robot_protocol_shape.py::test_robot_protocol_has_six_methods_plus_caps` flipped xfail → xpass
- [x] `test_robot_protocol_shape.py::test_mavsdk_drone_adapter_implements_robot` stays skipped (try/except guard until 03-06)
- [x] Full suite: 175 PASSED + 0 unexpected failures (baseline maintained)
- [x] `robot-follow --help` exits 0
- [x] Drone path unchanged (no imports of robot_api yet)

## Decisions Made

### 1. `@runtime_checkable` decorator added (not in original CONTEXT shape)

**Why:** The 03-02 test scaffold `test_mavsdk_drone_adapter_implements_robot` (lands in 03-06) needs `isinstance(adapter, Robot)` to work without `TypeError`. `@runtime_checkable` is the minimal enabler — it does NOT change method dispatch and does NOT break structural duck-typing for Protocols. Adapters still implement structurally; the decorator just unlocks the `isinstance` check.

**Trade-off:** `@runtime_checkable` Protocols have a small runtime cost for `isinstance` checks (linear scan of methods). Not in any hot path — only in tests and at adapter-instantiation time. Acceptable.

### 2. pyproject.toml NOT edited

**Why:** Per Task 2 step 2 in the plan ("NO EDIT NEEDED if the glob is already `robot_follow*`"). Confirmed via direct import after creating the package files:

```
$ python -c "import robot_follow.robot_api; print(robot_follow.robot_api.__file__)"
/home/guyz/code/guyz/hailo-drone-follow/robot_follow/robot_api/__init__.py
```

`setuptools.packages.find` with `include = ["robot_follow*", ...]` discovers all subpackages under `robot_follow/` recursively. Skipped the redundant `pip install -e .` cycle since the in-tree import via `setup_env.sh` already proves discovery works.

### 3. Per-task commits (2 commits) over combined single commit

**Why:** The plan's verification text *allowed* combining into one commit if Task 2 no-op'd `pyproject.toml` (it did). But two atomic commits keep the diff bisectable: Task 1 (`c38854e`) = pure scaffold (3 files); Task 2 (`a15bdba`) = orchestrator signature lock (1 file). If a future bisect lands on either commit, the failure mode is unambiguous.

### 4. `TYPE_CHECKING` imports in orchestrator

**Why:** Body is deferred to 03-08. Materializing the runtime imports for `ControllerConfig` + `SharedDetectionState` + `Robot` now would introduce coupling before the orchestrator actually needs them. String forward-refs (`"Robot"`, `"ControllerConfig"`) keep the signature compile-clean.

## Deviations from Plan

**None — plan executed exactly as written.**

The plan's verification command in Task 1 contained a minor stylistic divergence (it expected `caps` to appear in `dir(Robot)`, but Python's `Protocol` puts class-level annotations in `__annotations__` / `get_type_hints` not `dir()` unless they have a default value). The actual pytest assertion in `test_robot_protocol_shape.py` correctly uses `get_type_hints` and passes. No code change needed — the protocol is structurally correct per the test (which is the authoritative gate).

## Files Changed Inventory

### Created (4)

```
robot_follow/robot_api/__init__.py        # 14 lines — package marker, re-exports Robot
robot_follow/robot_api/robot.py           # 108 lines — Robot Protocol class
robot_follow/robot_api/adapters/__init__.py # 5 lines — empty subpackage marker
robot_follow/robot_api/orchestrator.py    # 48 lines — run_robot_loop signature
```

### Modified (0)

`pyproject.toml` left untouched. Existing `robot_follow*` glob already covers `robot_follow.robot_api.*`.

## Test Results

**Baseline (HEAD f2c39ac, 03-03 complete):** 175 passed, 9 skipped, 82 xfailed, 31 xpassed
**After 03-04 (HEAD a15bdba):** 175 passed, 8 skipped, 82 xfailed, 32 xpassed

**Delta:**
- 175 passed → 175 passed (no regressions ✓)
- 9 skipped → 8 skipped (one fewer — `test_robot_protocol_has_six_methods_plus_caps` moved from skipped via try/except → xpassed)
- 82 xfailed → 82 xfailed (no change ✓)
- 31 xpassed → 32 xpassed (one more — protocol-shape test flipped as predicted ✓)

**Protocol-shape verification:**

```
robot_follow/tests/test_robot_protocol_shape.py::test_robot_protocol_has_six_methods_plus_caps XPASS
robot_follow/tests/test_robot_protocol_shape.py::test_mavsdk_drone_adapter_implements_robot   SKIPPED
```

The adapter-implements-Robot test stays SKIPPED (try/except guard catches `ImportError` from the not-yet-existing `robot_follow.robot_api.adapters.mavsdk_drone`); it will flip in 03-06 when the adapter lands.

## Sanity Checks

- `from robot_follow.robot_api import Robot` — succeeds
- `from robot_follow.robot_api.robot import Robot` — succeeds
- `Robot is (robot_follow.robot_api.Robot)` — True (re-export identity)
- `'caps' in typing.get_type_hints(Robot)` — True (caps annotation present)
- 6 methods present in `dir(Robot)`: `connect`, `start_session`, `send_command`, `send_zero`, `on_target_lost`, `shutdown`
- `inspect.signature(run_robot_loop).parameters == ['robot', 'shared_state', 'config', 'shutdown']` — True
- `robot-follow --help` exits 0
- `git log --oneline -3` shows: a15bdba, c38854e, f2c39ac (Task 2, Task 1, prior phase HEAD)

## Commits

| Task | Commit  | Files                                                                                            | Message                                                                  |
| ---- | ------- | ------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------ |
| 1    | c38854e | robot_api/__init__.py, robot_api/robot.py, robot_api/adapters/__init__.py                        | feat(03-04): scaffold robot_api/ package with Robot protocol             |
| 2    | a15bdba | robot_api/orchestrator.py                                                                        | feat(03-04): add run_robot_loop signature + verify package discovery     |

## Next Steps (For 03-05)

03-05 will `git mv robot_follow/drone_api/mavsdk_drone.py robot_follow/robot_api/adapters/mavsdk_drone.py` (preserving git history) and update all imports. The empty `adapters/__init__.py` is the move target.

03-06 will then implement `MavsdkDroneAdapter` against the Robot Protocol (adding `caps`, wrapping `connect`/`start_session`/`send_command`/`send_zero`/`on_target_lost`/`shutdown`), at which point `test_mavsdk_drone_adapter_implements_robot` flips xpass.

03-08 fills the `run_robot_loop` body (current `NotImplementedError` raise becomes the full 10 Hz tick state machine per CONTEXT § Orchestrator state machine).

## Known Stubs

- `robot_follow/robot_api/orchestrator.py::run_robot_loop` raises `NotImplementedError`. **Intentional** — body deferred to 03-08-PLAN (see plan Task 2 rationale: full state machine requires `MavsdkDroneAdapter` from 03-06 + `compute()` signature from 03-07). The raise message explicitly names 03-08-PLAN so any accidental caller knows where the body lands. Nothing in the codebase currently imports this function.

## Self-Check: PASSED

- `robot_follow/robot_api/__init__.py` — FOUND
- `robot_follow/robot_api/robot.py` — FOUND
- `robot_follow/robot_api/adapters/__init__.py` — FOUND
- `robot_follow/robot_api/orchestrator.py` — FOUND
- Commit `c38854e` — FOUND in git log
- Commit `a15bdba` — FOUND in git log
- All claims in this SUMMARY verified against working tree + git log.
