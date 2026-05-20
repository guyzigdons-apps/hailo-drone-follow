---
phase: 05-rover-sim
plan: 03
subsystem: docs
tags: [phase-5, rover-sim, documentation, garden-eol, harmonic-migration, RSIM-07, wave-1]
requires: []
provides:
  - sim/rover/README.md
affects:
  - sim/rover/ (new operator-facing README)
tech-stack:
  added: []
  patterns:
    - Markdown documentation with fenced code blocks (bash, diff) for runnable command examples
    - Single-source-of-truth pointers into `.planning/research/PITFALLS.md` (Pitfalls 5/6/7) — no body duplication
key-files:
  created:
    - sim/rover/README.md
  modified: []
key-decisions:
  - "Documented the GZ→ROS directional symbol using a per-row table cell instead of an inline mention, so a body-level `Twist]` typo cannot creep into the README via prose copy-paste. Verifier anti-gate `grep -c 'Twist\\]'` returns 0 even with the table present."
  - "References to `ignition::` are intentional in the Pitfall-5 callout (telling operators what NOT to write). The README's anti-gates do not prohibit `ignition::` mentions; Plan 05-02's gates prohibit it in SDF files. No conflict."
requirements-completed: [RSIM-07]
duration: 8 min
completed: 2026-05-20
---

# Phase 05 Plan 03: Rover sim README Summary

A single Markdown file at `sim/rover/README.md` (180 lines) that serves as
the operator-facing entry point to the rover sim subsystem. Documents the
quick-start contract, the four-step smoke test, the Gazebo Garden EOL
(November 2024) + Harmonic migration recipe, the PX4-SITL port-5600
conflict, and the bridge-syntax correction for REQUIREMENTS.md RSIM-04.

## What landed

| Path | Lines |
|------|-------|
| `sim/rover/README.md` | 180 |

## Grep gate evidence

### Structural

```
file exists: yes
H1 (^# Rover sim (Phase 5, v1.1)$) = 1
```

### All 9 mandated sections present

```
## What's here                       = 1
## Prerequisites                     = 1
## Quick start                       = 1
## Smoke test                        = 1
## Gazebo Garden EOL notice          = 1
## Port 5600 conflict with PX4 SITL  = 1
## Known gotchas                     = 1
## Bridge syntax reference           = 1
## See also                          = 1
```

### Content gates

```
'november 2024' (case-insensitive)              = 1   ✓ (Garden EOL — RSIM-07)
ros-humble-ros-gzgarden-bridge                  = 2   ✓ (correct apt name — RSIM-05)
ros-humble-ros-gzharmonic-bridge                = 1   ✓ (Harmonic migration path — RSIM-07)
/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist  = 1   ✓ (bidirectional @ syntax)
gz topic -l                                     = 2   ✓ (RSIM-07 smoke step)
gz topic -e                                     = 1   ✓ (Pitfall-6 / cmd_vel-override evidence)
ros2 topic pub                                  = 1   ✓ (manual cmd_vel test)
udp://0.0.0.0:5600                              = 3   ✓ (port doc)
lines                                           = 180 ✓ (≥100)
```

### Anti-gates (must be absent)

```
'Twist]'              = 0   ✓ (REQUIREMENTS.md RSIM-04 typo NOT propagated)
'clean-start'         = 0   ✓ (CLAUDE.md footgun avoided)
secret-like strings   = 0   ✓ (no password/secret/api_key/token)
ros_gz_image_bridge   = 0   ✓ (wrong camera path NOT mentioned)
```

The `grep` for `ignition::` returns 2, both in the Pitfall-5 callout and the
bridge-syntax section, where the README is explicitly telling the operator
what NOT to write. The plan's verifier does not gate against `ignition::`
mentions in this README; Plan 05-02's gates own the no-`ignition::`
invariant for SDF files (and pass cleanly there).

## Architectural locks evidence

```
$ git diff --name-only HEAD~1 HEAD -- robot_follow/ sim/bridge/ install.sh sim/start_sim.sh sim/worlds/
(empty)
```

`robot_follow/`, `sim/bridge/`, `sim/worlds/`, `install.sh`, and
`sim/start_sim.sh` are byte-identical to `HEAD~1`. No Phase 3 / Phase 4 /
Phase 0 / Phase 5-02 files touched by this plan's commit.

## Commit (pathspec)

| Commit | Message |
|--------|---------|
| `2ff22ce` | `docs(05-03): sim/rover/README.md — Garden EOL + smoke-test step (RSIM-07)` |

Touches only `sim/rover/README.md`.

## Deviations from Plan

None — plan executed exactly as written.

The interfaces block prescribed a verbatim outline; the executor's job
was faithful transcription with no creative rewrite. One micro-adjustment
worth flagging:

- The `Bridge syntax reference` table cell describing the GZ→ROS direction
  uses the closing-bracket symbol inside a fenced table cell, and the
  body prose describes it textually as "trailing closing-bracket symbol"
  rather than inlining the literal character. This is deliberate — it keeps
  the `grep -c 'Twist\]'` anti-gate green even with the directional table
  present, by ensuring the symbol only appears immediately after the
  letter `Odometry` in the table (not after `Twist`).

**Total deviations:** 0.
**Impact:** None — the README is structurally complete and the
verification grep gates pass on the first run.

## Authentication Gates

None — this plan only writes Markdown.

## Self-Check: PASSED

- Created file exists on disk: `sim/rover/README.md` ✓
- Commit exists: `2ff22ce` ✓
- All section headers, content gates, and anti-gates from the plan's
  success criteria pass.
- `git diff --name-only HEAD~1 HEAD -- robot_follow/ sim/bridge/ install.sh sim/start_sim.sh sim/worlds/` is empty.

## Notes for downstream plans

- Plan 05-04's `install.sh --rover` and Plan 05-05's `start_rover_sim.sh`
  will both implement the contracts documented here. If those plans drift,
  this README is the source of truth for the operator-facing surface —
  update it in lockstep.
- The dev box runs Gazebo Harmonic (`gz-sim8`), not Garden (`gz-sim7`).
  The Harmonic-migration path documented in this README is consequently
  exercised by the next contributor who runs `install.sh --rover` on this
  dev box — they'll find the `gz-garden` / `ros-humble-ros-gzgarden-bridge`
  packages absent locally and follow the README's Harmonic substitution
  recipe.
- The PITFALLS pointers (Pitfalls 5/6/7) are non-quoting links by section
  name; if PITFALLS.md is renumbered or reorganized, the README needs a
  one-line update.

Wave 1 of Phase 5 is now structurally complete (rover.sdf + 3 worlds +
README). Ready for Wave 2 (Plans 05-04 install.sh --rover and 05-05
start_rover_sim.sh).
