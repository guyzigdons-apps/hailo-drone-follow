---
phase: 05-rover-sim
plan: 02
subsystem: sim
tags: [phase-5, rover-sim, sdf, worlds, actor, RSIM-03, wave-1]
requires:
  - sim/rover/rover.sdf (from Plan 05-01, landed on this branch as 9431e04 / b934d94)
  - sim/worlds/Walking actor/meshes/walk.dae (existing drone-sim asset, reused via GZ_SIM_RESOURCE_PATH)
provides:
  - sim/rover/worlds/walk_across_then_approach.sdf (primary E2E target — Phase 6 RINT-04)
  - sim/rover/worlds/random_walk.sdf
  - sim/rover/worlds/circle_around.sdf
affects:
  - sim/rover/ (new worlds/ subdirectory)
tech-stack:
  added: []
  patterns:
    - Garden-style SDF (sdf 1.9) with `gz::sim::systems::*` plugin namespace (no `ignition::`)
    - `<include><uri>model://rover</uri></include>` resolves at sim launch via Plan 05-05's `GZ_SIM_RESOURCE_PATH` (NOT at SDF parse time)
key-files:
  created:
    - sim/rover/worlds/walk_across_then_approach.sdf
    - sim/rover/worlds/random_walk.sdf
    - sim/rover/worlds/circle_around.sdf
  modified: []
key-decisions:
  - "Actor renamed from 'actor' to 'person' in random_walk + circle_around rover variants so all three rover worlds share a single `<actor name=\"person\">` convention (walk_across_then_approach was already 'person' in the source drone world). The verifier greps for this exact string."
  - "`gz sdf -k` emits an `Unable to find uri[model://rover]` info message at parse time and still returns exit 0 — Garden / Harmonic do not resolve `<include>` URIs without `GZ_SIM_RESOURCE_PATH` set, which is Plan 05-05's responsibility. Treated as skip-on-Garden-mismatch per phase context."
requirements-completed: [RSIM-03]
duration: 12 min
completed: 2026-05-20
---

# Phase 05 Plan 02: Rover-adapted actor worlds Summary

Three rover-adapted Gazebo actor worlds under `sim/rover/worlds/`, each
spawning the rover model from Plan 05-01 via `<include><uri>model://rover</uri></include>`
and an animated `<actor name="person">` walking a rover-scaled trajectory.

## What landed

| Path | Lines | World name | Waypoints | Scaling rule |
|------|-------|------------|-----------|--------------|
| `sim/rover/worlds/walk_across_then_approach.sdf` | 210 | `walk_across_then_approach` | 13 | `X * 0.4`, Y/Z/yaw/time unchanged |
| `sim/rover/worlds/random_walk.sdf` | 364 | `random_walk` | 61 (60 waypoints + 1 trailing) | `X * 0.4`, Y/Z/yaw/time unchanged |
| `sim/rover/worlds/circle_around.sdf` | 146 | `circle_around` | 17 | `X * 0.5` AND `Y * 0.5` (radius 5 m → 2.5 m); Z/yaw/time unchanged |

`walk_across_then_approach.sdf` is the primary Phase 6 RINT-04 E2E test
target — Plan 05-05's `start_rover_sim.sh` defaults to it per RESEARCH Q2 lock.

## Per-file grep gate evidence

### `sim/rover/worlds/walk_across_then_approach.sdf`

```
ignition::=0           ✓
world_name=1           ✓
include=1              ✓
actor "person"=1       ✓
gz-sim-imu-system=0    ✓ (dropped)
gz-sim-air-pressure-system=0  ✓ (dropped)
gz-sim-sensors-system=1       ✓ (preserved — needed for rover camera)
walking actor mesh=2          ✓ (skin + animation refs)
waypoints=13                  ✓ (≥2)
lines=210                     ✓ (≥80)
gz sdf -k exit=0              ✓ (info msg about model://rover include resolution
                                expected — Plan 05-05 sets GZ_SIM_RESOURCE_PATH)
```

### `sim/rover/worlds/random_walk.sdf`

```
ignition::=0           ✓
world_name=1           ✓
include=1              ✓
actor "person"=1       ✓ (renamed from "actor" in the drone source)
gz-sim-imu-system=0    ✓ (dropped)
gz-sim-air-pressure-system=0  ✓ (dropped)
gz-sim-sensors-system=1       ✓ (preserved)
walking actor mesh=2          ✓
waypoints=61                  ✓ (≥2)
lines=364                     ✓ (≥80)
gz sdf -k exit=0              ✓ (info msg about model://rover include — expected)
```

### `sim/rover/worlds/circle_around.sdf`

```
ignition::=0           ✓
world_name=1           ✓
include=1              ✓
actor "person"=1       ✓ (renamed from "actor" in the drone source)
gz-sim-imu-system=0    ✓ (dropped)
gz-sim-air-pressure-system=0  ✓ (dropped)
gz-sim-sensors-system=1       ✓ (preserved)
walking actor mesh=2          ✓
waypoints=17                  ✓ (≥2)
lines=146                     ✓ (≥80)
gz sdf -k exit=0              ✓ (info msg about model://rover include — expected)
```

## Architectural locks evidence

```
$ git diff --name-only HEAD~2 HEAD -- sim/worlds/ robot_follow/ sim/bridge/ install.sh sim/start_sim.sh
(empty)
```

Drone worlds (`sim/worlds/`), the drone-follow Python tree (`robot_follow/`),
the shared camera bridge (`sim/bridge/`), the install script, and the drone
sim launcher are byte-identical to `HEAD~2`. No Phase 3 / Phase 4 / Phase 0
files touched.

## Commits (pathspec)

| # | Commit | Message |
|---|--------|---------|
| 1 | `ee8285d` | `feat(05-02): add rover-adapted walk_across_then_approach world (RSIM-03)` |
| 2 | `76583cb` | `feat(05-02): add rover-adapted random_walk + circle_around worlds (RSIM-03)` |

Each commit touches only `sim/rover/worlds/*.sdf` paths.

## Deviations from Plan

### Auto-fixed issues

**1. [Rule 1 — Naming consistency] `<actor name>` renamed from `actor` to `person` in random_walk + circle_around**

- **Found during:** Task 2
- **Issue:** The plan's `must_haves.truths` and its grep gate require
  `<actor name="person">` in every rover world. The drone source worlds for
  `random_walk.sdf` and `circle_around.sdf` use `<actor name="actor">`,
  whereas `walk_across_then_approach.sdf` already uses `<actor name="person">`.
  A verbatim carry-over of the drone source would fail the grep gate.
- **Fix:** Renamed the actor element to `person` in both rover variants so
  all three rover worlds share the same actor name. The drone source worlds
  are unchanged (architectural lock preserved).
- **Files modified:** `sim/rover/worlds/random_walk.sdf`, `sim/rover/worlds/circle_around.sdf`
- **Verification:** `grep -c '<actor name="person">'` returns `1` in each
  file; `grep -c '<actor name="actor">'` returns `0`.
- **Commit:** `76583cb`

Other than the actor-name normalization, no deviations.

**Total deviations:** 1 auto-fixed (1 × Rule 1 naming/grep-gate alignment).
**Impact:** Neutral. The drone worlds keep their original `<actor name="actor">`
(they are untouched). The rover worlds use a single, consistent actor name
that the verifier and future grep-based introspection can rely on.

## Authentication Gates

None — this plan only touches SDF files.

## Self-Check: PASSED

- Created files exist on disk:
  - `sim/rover/worlds/walk_across_then_approach.sdf` ✓
  - `sim/rover/worlds/random_walk.sdf` ✓
  - `sim/rover/worlds/circle_around.sdf` ✓
- Commits exist:
  - `ee8285d` (Task 1) ✓
  - `76583cb` (Task 2) ✓
- All structural grep gates per success criteria pass for each file.
- `git diff --name-only HEAD~2 HEAD -- sim/worlds/ robot_follow/ sim/bridge/ install.sh sim/start_sim.sh` is empty.

## Notes for downstream plans

- Plan 05-05's `start_rover_sim.sh` defaults to `walk_across_then_approach`
  per RESEARCH Q2 lock; that name is now hard-encoded in `<world name="…">`
  and the launcher must not rename it without updating Phase 6 RINT-04.
- The dev box runs Gazebo Harmonic (`gz-sim8`), not Garden (`gz-sim7`).
  Full runtime verification — actor walks, rover renders on ground plane,
  camera frames reach UDP 5600 — is deferred to Phase 6 RINT-04 under an
  operator gate. Phase 5's bar is "files exist, parse as SDF, structurally
  match the rover-adaptation rules", which all three files clear.
- Each world's `<include><uri>model://rover</uri></include>` is unresolved
  by `gz sdf -k` (info message, exit 0). At sim launch, Plan 05-05's
  `start_rover_sim.sh` will export
  `GZ_SIM_RESOURCE_PATH=$SCRIPT_DIR(sim/rover):sim/worlds:…`, after which
  `model://rover` resolves to `sim/rover/rover.sdf` from Plan 05-01.

Ready for Plan 05-03 (README documenting the rover sim entry point).
