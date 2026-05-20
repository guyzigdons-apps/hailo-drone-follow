---
phase: 05-rover-sim
plan: 05
subsystem: rover-sim
tags: [phase-5, rover-sim, launcher, bash, setsid, cleanup, RSIM-04, RSIM-06]
requires: [05-01, 05-02]
provides: [sim/rover/start_rover_sim.sh — one-command rover sim launcher]
affects: [sim/rover/start_rover_sim.sh]
tech-stack:
  added: []
  patterns: [setsid process-group cleanup, trap-on-EXIT-INT-TERM, pgrep-based stale reap, bash parameter expansion for path computation]
key-files:
  created:
    - sim/rover/start_rover_sim.sh
  modified: []
key-decisions:
  - Rule 3 deviation — use bash parameter expansion (${var%/*}) instead of external dirname so the stripped-PATH exit-3 verify gate can fire the gz preflight without aborting at exit 127 on dirname-not-found
  - Refactor kill-0-then-kill-9 pattern from sim/start_sim.sh prior art into an explicit if-then block to satisfy shellcheck SC2015 cleanly (info-level on default severity), avoiding a per-line disable comment
  - export PROJECT_ROOT to silence SC2034 (computed-but-unused) cleanly — keeps the variable available for downstream debug invocations without a disable comment
requirements-completed: [RSIM-04, RSIM-06]
metrics:
  duration: 22 min
  completed: 2026-05-20
  tasks: 1
  files_created: 1
---

# Phase 05 Plan 05: start_rover_sim.sh Launcher Summary

`sim/rover/start_rover_sim.sh` is now the one-command rover-sim launcher. It boots gz sim Garden with the rover world, attaches `ros_gz_bridge parameter_bridge` for `/cmd_vel`, and forwards `/camera` H.264 RTP to UDP `127.0.0.1:5600` via the unchanged `sim/bridge/video_bridge.py`. Process-group cleanup via `setsid` + `kill -- -$pid` + `trap cleanup EXIT INT TERM` so children don't survive Ctrl+C and block the next launch.

## Commits

| Hash | Description |
|------|-------------|
| `2e55a6f` | feat(05-05): sim/rover/start_rover_sim.sh — one-command rover sim launcher (RSIM-04, RSIM-06) |

## Final Line Count

`sim/rover/start_rover_sim.sh`: 193 lines (≥80 required).

Executable bit: set (`-rwxr-xr-x`).

## Grep Gate Evidence

```
=== Anti-gates (each must be 0) ===
ignition::                                   → 0 ✓
Twist] (REQUIREMENTS.md RSIM-04 typo)        → 0 ✓
Fortress no-suffix gz-bridge form            → 0 ✓ (not applicable here, but verified clean)

=== Required-content gates ===
/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist (bidirectional)  → 1 ✓
setsid (one per background launcher: gz sim, ros_gz_bridge,
        video_bridge.py)                                          → 3 ✓
trap cleanup EXIT INT TERM                                       → 1 ✓
kill -- "-$pid" (group kill)                                     → 1 ✓
GZ_SIM_RESOURCE_PATH export                                      → 1 ✓
video_bridge.py references                                       → 4 ✓
--topic /camera                                                  → 1 ✓
--host 127.0.0.1 (defense-in-depth: don't rely on bridge default)→ 1 ✓
--port 5600                                                      → 1 ✓
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python                    → 1 ✓
World-name allowlist regex [a-z0-9_]                             → 1 ✓
sleep 3 (Pitfall F race-window mitigation)                       → 1 ✓
exit 2 (bad arg / unknown world)                                 → 3 ✓
exit 3 (missing gz CLI)                                          → 1 ✓
exit 4 (missing /opt/ros/humble/setup.bash)                      → 1 ✓
```

## Invocation Test Outcomes

| Test | Expected | Actual | Result |
|------|----------|--------|--------|
| `start_rover_sim.sh --help` | exit 0 | 0 | PASS |
| `start_rover_sim.sh --bogus` | exit 2 (unknown flag) | 2 | PASS |
| `start_rover_sim.sh --world ../../etc/passwd` | exit 2 (allowlist reject) | 2 | PASS |
| `start_rover_sim.sh --world nonexistent_world` | exit 2 (file not found) | 2 | PASS |
| `env -i PATH=/tmp HOME="$HOME" /usr/bin/bash sim/rover/start_rover_sim.sh` | exit 3 (gz missing) | 3 | PASS |

## Shellcheck

`shellcheck sim/rover/start_rover_sim.sh` → exit 0. Clean. No `# shellcheck disable=` comments beyond the single existing convention from `sim/start_sim.sh` (SC1091 for `source /opt/ros/humble/setup.bash`).

## Architectural-Lock Evidence

```
$ git diff -- sim/bridge/video_bridge.py
(empty)

$ git diff -- robot_follow/
(empty)

$ git diff -- sim/rover/rover.sdf sim/rover/worlds/ sim/rover/README.md sim/start_sim.sh sim/worlds/
(empty)

$ git diff HEAD~1 HEAD --name-only
sim/rover/start_rover_sim.sh
```

Only `sim/rover/start_rover_sim.sh` is in this commit. RSIM-06 byte-identity lock on `sim/bridge/video_bridge.py` is intact.

## Exit 4 — Not Exercised on This Dev Box

Exit 4 (missing `/opt/ros/humble/setup.bash`) cannot be deterministically tested here because the stripped-PATH route hits exit 3 (`gz` missing) FIRST in the preflight order. On a contributor's box that has `gz` installed but no ROS Humble, exit 4 fires. Phase 6 RINT-04 covers this under operator gate.

## Gazebo Harmonic vs Garden — Live-Sim Verification Deferred

This dev box has Gazebo Harmonic (gz-sim8) installed, not Garden (gz-sim7). If invoked with all preflights satisfied, `gz sim` would launch Harmonic, but the rover.sdf (Plan 05-01) and the ros-gzgarden-bridge package (Plan 05-04) target Garden. Live-sim "rover spawns + parameter_bridge attaches + drone-follow consumes RTP" is **deferred to Phase 6 RINT-04** under operator gate on a Garden VM. The launcher's STRUCTURE is verified here (grep + shellcheck + invocation exit codes); live-sim end-to-end is the Phase 6 deliverable.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 — Blocking] Path computation switched from `dirname` to bash parameter expansion**

- **Found during:** Task 1 verify (stripped-PATH exit-3 test)
- **Issue:** The plan's verbatim launcher uses `SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"`. The plan's verify gate then invokes `env -i PATH=/tmp HOME="$HOME" bash sim/rover/start_rover_sim.sh` and expects exit 3 (missing gz). However, `dirname` is `/usr/bin/dirname` — under `PATH=/tmp` it cannot be found, `set -e` aborts the script with exit 127 before reaching the gz preflight.
- **Fix:** Replaced `dirname` calls with bash parameter expansion (`${var%/*}`), which is a builtin and doesn't depend on PATH. Added explicit relative-path-to-absolute resolution (`if [[ "$_SCRIPT_PATH" != /* ]]; then _SCRIPT_PATH="$PWD/$_SCRIPT_PATH"; fi`) so SCRIPT_DIR is always absolute (matches `dirname` behavior). Documented the rationale in a leading comment.
- **Files modified:** sim/rover/start_rover_sim.sh (path-computation block)
- **Verification:** `env -i PATH=/tmp HOME="$HOME" /usr/bin/bash sim/rover/start_rover_sim.sh` now exits 3 with the friendly "'gz' CLI missing" message.
- **Commit:** 2e55a6f

**2. [Rule 1 — Bug] kill-0-then-kill-9 pattern refactored to satisfy shellcheck SC2015**

- **Found during:** Task 1 verify (shellcheck run)
- **Issue:** The plan's verbatim launcher copies the `kill -0 "$pid" 2>/dev/null && kill -9 "$pid" 2>/dev/null || true` pattern from `sim/start_sim.sh` prior art. Shellcheck flags this as SC2015 info-level: `A && B || C is not if-then-else; C may run when A is true.` In this case it's harmless (the `|| true` is just for `set -e` suppression), but the plan's verify gate runs default-severity shellcheck which returns non-zero on info-level findings.
- **Fix:** Refactored to an explicit if-then block: `if kill -0 "$pid" 2>/dev/null; then kill -9 "$pid" 2>/dev/null || true; fi`. Same semantics; shellcheck clean. (The prior-art version in `sim/start_sim.sh` is OUT OF SCOPE per the plan's must-not-do list, so it stays as-is; this plan's launcher uses the cleaner form.)
- **Files modified:** sim/rover/start_rover_sim.sh (preflight_reap_stale function)
- **Verification:** `shellcheck sim/rover/start_rover_sim.sh` exits 0.
- **Commit:** 2e55a6f

**3. [Rule 1 — Bug] export PROJECT_ROOT to silence SC2034**

- **Found during:** Task 1 verify (shellcheck run)
- **Issue:** The plan's verbatim launcher computes `PROJECT_ROOT="$(dirname "$SIM_DIR")"` but doesn't use it. Shellcheck SC2034: `PROJECT_ROOT appears unused. Verify use (or export if used externally).`
- **Fix:** Added `export PROJECT_ROOT` immediately after the path computations. PROJECT_ROOT is available to child processes (intentional — useful for debug invocations) and shellcheck is silenced cleanly without a disable comment.
- **Files modified:** sim/rover/start_rover_sim.sh (added one `export PROJECT_ROOT` line)
- **Verification:** `shellcheck sim/rover/start_rover_sim.sh` exits 0.
- **Commit:** 2e55a6f

**Total deviations:** 3 auto-fixed (1× Rule 3 — blocking under stripped-PATH verify gate, 2× Rule 1 — shellcheck-clean polish on the verbatim transcription). All resolved inside the single Task 1 commit per the pathspec-only rule.

**Impact:** Plan deliverable shape unchanged. Functionality identical to the planner's verbatim launcher (same exit codes, same setsid/trap behavior, same bridge invocation). The path-computation switch makes the verify gate work; the shellcheck polish keeps the code clean against the strictest severity.

## Threat Surface Scan

No new threat surface introduced beyond the existing `<threat_model>` (T-05-05-01..08 + T-05-05-SC). All mitigations in place:

- T-05-05-01 (path traversal via `--world`): allowlist regex `^[a-z0-9_]+$` + file existence check → exit 2 ✓
- T-05-05-02 (bridge syntax typo): correct bidirectional `@` form present; `Twist]` anti-grep returns 0 ✓
- T-05-05-03 (`ignition::` snuck in): anti-grep returns 0 ✓
- T-05-05-04 (UDP on 0.0.0.0): launcher passes `--host 127.0.0.1` EXPLICITLY ✓
- T-05-05-05 (stale gz sim blocks new launch): `preflight_reap_stale()` greps repo-scoped patterns and reaps ✓
- T-05-05-06 (children survive Ctrl+C): `setsid` ×3 + `kill -- -$pid` + `trap cleanup EXIT INT TERM` ✓
- T-05-05-07 (parameter_bridge race): `sleep 3` between gz sim launch and parameter_bridge launch ✓
- T-05-05-08 (out-of-scope edit): pathspec commit, only sim/rover/start_rover_sim.sh staged ✓

## Self-Check: PASSED

- [x] `sim/rover/start_rover_sim.sh` exists, ≥80 lines (193), executable bit set
- [x] All 24 success criteria from PLAN.md verified
- [x] Shellcheck clean (RC=0)
- [x] All 5 invocation tests pass (--help, --bogus, path-traversal, unknown world, stripped-PATH)
- [x] `sim/bridge/video_bridge.py` byte-identical (RSIM-06 architectural lock)
- [x] No out-of-scope files touched
- [x] Single pathspec commit landed (`2e55a6f`)

With Plans 05-01..05-05 landed (rover.sdf, rover worlds, README, install.sh --rover, start_rover_sim.sh), the Phase 5 deliverable surface (RSIM-01..07) is structurally complete. Plan 05-06 (OPTIONAL Python parse-only smoke test) is the next item if scheduled; otherwise the phase is ready for verify-work + RINT-04 operator gate on a Garden VM.
