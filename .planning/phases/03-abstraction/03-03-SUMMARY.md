---
phase: 03-abstraction
plan: 03
subsystem: domain-types
tags: [dataclass, enum, frozen-dataclass, follow-api, robot-abstraction, axes-only, capabilities, safety-context]

# Dependency graph
requires:
  - phase: 03-abstraction
    provides: "Wave-0 xfail scaffolds (03-01, 03-02) — test_robot_command_shape.py, test_robot_protocol_shape.py, test_robot_command_snapshot.py awaiting the type landings"
provides:
  - "Axis enum (FORWARD, YAW, ALTITUDE) in follow_api/types.py"
  - "Capabilities frozen dataclass (axes: frozenset[Axis], yaw_unit) — mechanical only"
  - "RobotCommand dataclass (forward_m_s, yaw_rate, down_m_s; all default 0.0)"
  - "SafetyContext frozen dataclass (bbox_bottom_normalized, bbox_size_normalized, target_lost, last_target_x) + from_detection / lost classmethods"
  - "follow_api package re-exports of all 4 new types"
affects: [03-04 Robot protocol, 03-05 controller migration, 03-06 mavsdk_drone adapter, 03-07 VelocityCommand deletion, 04-rover-adapter, 05-rover-sim]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Pure-leaf core: actuator-boundary types live in follow_api/types.py; robot_api imports from here, never the other way (R1 lock 2026-05-17)"
    - "Axes-only Capabilities — no behavioral policy flags; robot-specific behaviors live INSIDE adapters"
    - "Q5 lock: RobotCommand.yaw_rate is in caps.yaw_unit; adapter does NO unit conversion"
    - "Q6 lock: SafetyContext.target_lost=True is an early-return signal for adapters; lost-case bbox values are unspecified"
    - "SafetyContext as a decoupling shim between Detection and adapter (insulates adapter from v1.2 Detection-shape evolution)"

key-files:
  created: []
  modified:
    - "robot_follow/follow_api/types.py — 28 → 149 lines (+121, +4 types + module docstring expansion)"
    - "robot_follow/follow_api/__init__.py — 20 → 30 lines (+10, re-exports + __all__ updated)"

key-decisions:
  - "Frozen dataclasses for Capabilities + SafetyContext (value objects: hashable, immutable, comparable). Non-frozen RobotCommand because adapters may want to clone-modify mid-pipeline (smoothing path returns a new instance)."
  - "Lost-case sentinel values fixed at (bbox_bottom_normalized=0.5, bbox_size_normalized=0.25) per RESEARCH § SafetyContext derivation — 'center of frame, neutral bbox'. Chosen so a misbehaving adapter that ignores target_lost would NOT trigger spurious retreat-from-tilt (values are safely inside any edge zone)."
  - "Detection placed BEFORE SafetyContext in file order so SafetyContext.from_detection's type hint resolves at runtime without a forward-ref string."
  - "Single commit for both Tasks 1+2 per plan verification guidance ('Both tasks land as ONE commit per per-plan discipline — research commit-shape § Wave 1 commit 2'). Standard per-task-commit pattern overridden by explicit plan direction."

patterns-established:
  - "Pattern: actuator-boundary types in pure-leaf core — keeps follow_api third-party-free so it's reusable across drone/rover/v1.2 robots"
  - "Pattern: legacy + new type co-existence during migration (VelocityCommand stays alive until 03-07 finishes the 103-test-edit migration)"
  - "Pattern: xfail scaffold + type landing → xpass without flipping markers (strict=False allows coincidental passes during the migration; markers strip in 03-07)"

requirements-completed: [ABS-01, ABS-02]

# Metrics
duration: 4min
completed: 2026-05-19
---

# Phase 03 Plan 03: Add Axis + Capabilities + RobotCommand + SafetyContext Summary

**4 new actuator-boundary domain types added to follow_api/types.py (Axis enum, Capabilities, RobotCommand, SafetyContext + classmethods) — pure-leaf core stays third-party-free; ABS-02 type-shape gate flips xpass.**

## Performance

- **Duration:** 4 min (268 s)
- **Started:** 2026-05-19T09:50:19Z
- **Completed:** 2026-05-19T09:54:47Z
- **Tasks:** 2 (combined into one commit per plan direction)
- **Files modified:** 2

## Accomplishments

- `Axis` enum landed with FORWARD, YAW, ALTITUDE — drone-shaped scope per CONTEXT.md domain note (pan-tilt / holonomic-lateral / submarine-roll-pitch / robot-arm explicitly excluded; rename target documented as `Robot → BodyVelocityRobot` if v1.2 surfaces a non-velocity actuator).
- `Capabilities` frozen dataclass — axes-only design (decided 2026-05-14, ratified R1 2026-05-17). Zero behavioral policy flags; carries only the mechanical actuator surface (`axes`, `yaw_unit`).
- `RobotCommand` dataclass with 3 float fields, all defaulting to 0.0. Replaces `VelocityCommand` once 03-07 lands the controller migration. `yaw_rate` is in `caps.yaw_unit` — no adapter-side conversion (Q5 lock).
- `SafetyContext` frozen dataclass + `from_detection` / `lost` classmethods. Decouples adapter from `Detection` shape (insulates against v1.2 depth / multi-camera / multi-target evolution). `target_lost=True` is the early-return signal for adapters (Q6 lock).
- `follow_api` package `__init__.py` re-exports the 4 new types alphabetically alongside existing `Detection`, `VelocityCommand`, etc. Both import forms (`from robot_follow.follow_api.types import X` and `from robot_follow.follow_api import X`) resolve.
- `VelocityCommand` + `Detection` unchanged at the field level. `VelocityCommand` docstring extended with a DEPRECATED note pointing at 03-07.
- `test_robot_command_shape.py::test_robot_command_shape` flips from xfail → **xpass** (skip→xpass), confirming the new RobotCommand shape gate.

## Task Commits

Both tasks landed in a single combined commit per plan verification guidance (research commit-shape § Wave 1 commit 2):

1. **Tasks 1+2: Add 4 new types + package re-exports** — `ea64a90` (feat)
   - Task 1: `robot_follow/follow_api/types.py` rewrite (28 → 149 lines; +121 lines)
   - Task 2: `robot_follow/follow_api/__init__.py` re-export update (20 → 30 lines; +10 lines)

**Plan metadata (this SUMMARY + STATE/ROADMAP):** to be committed as the final docs commit.

## Files Created/Modified

- `robot_follow/follow_api/types.py` (modified) — added Axis, Capabilities, RobotCommand, SafetyContext (with from_detection / lost classmethods); preserved VelocityCommand + Detection byte-identical at the field level (docstring updates only on VelocityCommand). File order: docstring → imports → Axis → Capabilities → RobotCommand → VelocityCommand (legacy) → Detection → SafetyContext.
- `robot_follow/follow_api/__init__.py` (modified) — `from .types import` extended to all 6 types; `__all__` updated alphabetically.

## Test Results

**Baseline (HEAD = 66c1106):** 175 passed, 10 skipped, 82 xfailed, 30 xpassed, 0 failed.
**After this plan (HEAD = ea64a90):** 175 passed, 9 skipped, 82 xfailed, **31 xpassed**, 0 failed.

**Delta:**
- `+1 xpassed` — `test_robot_command_shape.py::test_robot_command_shape` flipped from `skip` (was hitting `pytest.skip("RobotCommand not yet defined")` at the top of the try-block) to `xpass` (markers strip in 03-07).
- `-1 skipped` — same test, no longer skipping.
- `0 unexpected failures.`
- Pass count unchanged (175) — this plan added zero new passing tests; the xfail scaffolds from 03-02 carry the regression detection.

`test_robot_command_shape.py::test_velocity_command_shape_legacy` continues to xpass (legacy VelocityCommand still around — flips to xfail or strip in 03-07).

## Decisions Made

1. **Single combined commit for Tasks 1 + 2.** Plan verification explicitly directs this: "Both tasks land as ONE commit per per-plan discipline (research commit-shape § Wave 1 commit 2)". Standard per-task-commit pattern overridden by plan direction. Both edits are tightly coupled — the `__init__.py` re-export depends on the new types existing, and any intermediate commit would leave `from robot_follow.follow_api import Axis` failing.
2. **Frozen Capabilities + SafetyContext; non-frozen RobotCommand.** Value-object semantics (hashable, immutable, comparable) wanted for Capabilities (one per robot, never mutated) and SafetyContext (per-tick read-only snapshot). RobotCommand non-frozen because Wave-4 smoothing returns a clone-modified instance, and unit tests construct mutable ones for staging — matching CONTEXT § Robot protocol shape.
3. **Lost-case sentinel = `(0.5, 0.25)` not `(0.0, 0.0)`.** Per RESEARCH § SafetyContext derivation lines 911-928: values must be "safely inside any edge zone" so a buggy adapter ignoring `target_lost` would NOT trigger spurious retreat-from-tilt. `(0.5, 0.25)` = center of frame, neutral bbox — fail-safe. `(0.0, 0.0)` would parse as "person at top edge with zero-size bbox", which an edge-aware adapter might respond to.
4. **Detection placed BEFORE SafetyContext in file order.** Avoids the forward-ref string in `SafetyContext.from_detection(det: Detection)`. Module-load order matters because `SafetyContext` evaluates its annotations at class-definition time.
5. **`from .types import` listed alphabetically + `__all__` re-sorted.** Makes future additions a single-line append, not a midway insertion. Eight-name `__all__` block reads cleanly.

## Deviations from Plan

**None — plan executed exactly as written.**

Per plan verification §, both tasks were combined into a single commit (`feat(03-03): add Axis + Capabilities + RobotCommand + SafetyContext to follow_api/types.py`). This is not a deviation; it is the plan's documented commit shape ("Both tasks land as ONE commit per per-plan discipline").

## Issues Encountered

None. The xfail scaffold from 03-02 (`test_robot_command_shape.py::test_robot_command_shape`) was using `pytest.skip(...)` as a fallback when `RobotCommand` was not yet importable — once the type landed, the skip-guard short-circuited and the assertions executed against the real `RobotCommand`, reporting as `xpass` (assertions pass + xfail marker still present + `strict=False`).

## Self-Check: PASSED

- `robot_follow/follow_api/types.py` — FOUND (149 lines)
- `robot_follow/follow_api/__init__.py` — FOUND (30 lines)
- Commit `ea64a90` — FOUND in git log
- `python -c "from robot_follow.follow_api.types import Axis, Capabilities, RobotCommand, SafetyContext"` exit 0 — VERIFIED
- `python -c "from robot_follow.follow_api import Axis, Capabilities, RobotCommand, SafetyContext"` exit 0 — VERIFIED
- `test_robot_command_shape.py::test_robot_command_shape` reports `xpass` — VERIFIED
- Full suite: 175 passed, 9 skipped, 82 xfailed, 31 xpassed, 0 failed — VERIFIED

## Next Phase Readiness

**Ready for Wave 3 (03-04, 03-05).** Specifically:
- **03-04 — Robot protocol:** can now `from robot_follow.follow_api.types import Axis, Capabilities, RobotCommand, SafetyContext` and define the `Robot` Protocol against the real types. `test_robot_protocol_shape.py` scaffolds (PROTOCOL + ADAPTER) can read these types — the xfail flips when the Protocol itself lands.
- **03-05 — controller-side migration:** can now build a `RobotCommand` instead of a `VelocityCommand` in the controller. Q5 unit-of-yaw is locked (controller emits `yaw_rate` in `caps.yaw_unit`).
- **03-06 — MavsdkDroneAdapter:** can now type its `send_command(cmd: RobotCommand, safety: SafetyContext, caps: Capabilities)` signature.
- **03-07 — VelocityCommand deletion + xfail strip:** both legacy `test_velocity_command_shape_legacy` and the new `test_robot_command_shape` markers come off in the same commit as the controller signature change.

No blockers introduced. The pre-existing `DEFER-02-00-A` (2 controller failures in `test_controller.py::TestDistanceForward`) is unchanged by this plan — recommended action remains a Phase-2.5 patch or a Plan 03-05 follow-up before the controller migration lands.

---
*Phase: 03-abstraction*
*Completed: 2026-05-19*
