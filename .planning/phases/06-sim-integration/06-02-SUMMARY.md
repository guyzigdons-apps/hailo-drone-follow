---
phase: 06-sim-integration
plan: "02"
subsystem: docs-and-tests
tags: [phase-6, sim-integration, docs, port-isolation, RINT-05, wave-1]
dependency_graph:
  requires: []
  provides: [RINT-05-closure]
  affects: [sim/rover/README.md, robot_follow/tests/test_rover_sim_smoke.py]
tech_stack:
  added: []
  patterns: [TDD RED/GREEN, parse-only smoke test]
key_files:
  modified:
    - sim/rover/README.md
    - robot_follow/tests/test_rover_sim_smoke.py
decisions:
  - "Appended '### Port usage comparison' table INSIDE the existing 'Port 5600 conflict with PX4 SITL' section rather than creating a new top-level section — preserves the existing section's byte-identical content while closing the RINT-05 gap."
  - "TDD RED/GREEN split across two commits: test commit (0f14c0f) + implementation commit (6b35826). Allows git bisect to identify exactly when RINT-05 closure landed."
metrics:
  duration_min: 8
  completed_date: "2026-05-20"
  tasks_completed: 1
  files_modified: 2
---

# Phase 06 Plan 02: Port isolation documentation + regression-guard test (RINT-05) Summary

**One-liner:** Port-comparison table appended to rover README naming MAVLink UDP 14540 (PX4 SITL only) vs ROS DDS (rover sim, no MAVLink), closing RINT-05 with a regression-guard smoke test.

## What Was Built

### New table block in `sim/rover/README.md`

Appended immediately after the existing v1.2 remap note, before `## Known gotchas`:

```markdown
### Port usage comparison

| Stack    | Video (UDP) | Actuator wire             |
|----------|-------------|---------------------------|
| PX4 SITL (drone) | 5600 | MAVLink on UDP 14540 |
| rover sim        | 5600 | ROS DDS (no MAVLink) |

The video port (UDP 5600) is the only collision. The actuator wires
are isolated: PX4 SITL never speaks ROS, and the rover sim never
speaks MAVLink. This means you can't run BOTH sims on the same host
at once (video collision), but you can run drone-follow against
EITHER sim on the same host without MAVLink-vs-ROS interference —
they're on entirely different wires.
```

### Existing section header preserved (grep evidence)

```
## Port 5600 conflict with PX4 SITL

Both PX4 SITL (`sim/start_sim.sh --bridge`) and rover sim
(`sim/rover/start_rover_sim.sh`) bind `udp://0.0.0.0:5600` for the
camera feed.  **They cannot run simultaneously on the same machine.**
```

Header and both existing paragraphs are byte-identical to HEAD~2 (APPEND only).

### New test in `robot_follow/tests/test_rover_sim_smoke.py`

```python
def test_readme_documents_port_isolation_table() -> None:
    """RINT-05: sim/rover/README.md documents the port-isolation contrast
    between PX4 SITL and rover sim.

    Required ground (per RINT-05 wording):
    - Video port (5600 UDP) collides between both stacks
    - PX4 SITL also uses MAVLink on UDP 14540
    - Rover sim uses ROS DDS — NO MAVLink

    The 'Port usage comparison' table inside the existing 'Port 5600
    conflict with PX4 SITL' section closes the gap. Plan 06-02 owns
    this addition.
    """
    assert ROVER_README.is_file(), f"{ROVER_README} missing"
    text = ROVER_README.read_text()
    # Existing section header preserved (lock — must NOT have been rewritten)
    assert "Port 5600 conflict with PX4 SITL" in text, (
        f"existing 'Port 5600 conflict with PX4 SITL' section was lost — "
        f"06-02 must APPEND, not REPLACE"
    )
    # New table header
    assert "Port usage comparison" in text, (
        "RINT-05: missing 'Port usage comparison' table header in "
        "sim/rover/README.md (06-02 task contract)"
    )
    # The 7 substrings the table must contain
    required = [
        "PX4 SITL",
        "5600",
        "MAVLink",
        "14540",
        "rover sim",
        "ROS DDS",
        "no MAVLink",
    ]
    for s in required:
        assert s in text, (
            f"RINT-05: required string {s!r} missing from "
            f"sim/rover/README.md port-isolation table"
        )
```

## Test Results

- `robot_follow/tests/test_rover_sim_smoke.py`: **11 passed** (was 10 — +1 PASS from `test_readme_documents_port_isolation_table`)
- Full suite (excl. `test_sim_worlds.py`): **337 passed, 5 failed** — the 5 failures are pre-existing `test_install_smoke.py` failures requiring the package to be installed as a console script (unrelated to this plan; present on the branch HEAD before this plan).
- `test_rover_sim_smoke.py::test_readme_documents_port_isolation_table`: PASSED

## TDD Gate Compliance

- RED commit: `0f14c0f` — `test(06-02): add failing test_readme_documents_port_isolation_table (RINT-05 RED)`
- GREEN commit: `6b35826` — `docs(06-02): add port-usage comparison table to rover README (RINT-05)`
- REFACTOR: not needed (minimal doc + test; no cleanup required)

## Architectural Lock Evidence

```
sim/rover/ (excl README): empty diff OK
sim/bridge/video_bridge.py: empty diff OK
robot_follow/ (excluding the smoke test): empty diff OK
code locks (robot_api/adapters/, follow_api/): empty diff OK
```

`git diff --name-only HEAD~1 HEAD -- 'sim/rover/*' ':(exclude)sim/rover/README.md'` returned empty.
`git diff --name-only HEAD~1 HEAD -- sim/bridge/video_bridge.py robot_follow/robot_api/adapters/ robot_follow/follow_api/` returned empty.

## Deviations from Plan

None — plan executed exactly as written.

## RINT-05 Status

RINT-05 is now **CLOSED** for the v1.1 sim-only milestone. The existing README text already documented the UDP 5600 video collision; the new table adds the previously missing explicit contrast of the actuator wires (MAVLink UDP 14540 on PX4 SITL vs ROS DDS on rover sim). The deferred `--rover-video-port` flag remains a v1.2 concern per the existing README's "future fix" note.

## Known Stubs

None — this plan is documentation and test only; no data flow or UI rendering involved.

## Threat Flags

None — no new network endpoints, auth paths, file access patterns, or schema changes introduced. Both files are read-only at runtime.

## Self-Check: PASSED

- sim/rover/README.md: FOUND
- robot_follow/tests/test_rover_sim_smoke.py: FOUND
- .planning/phases/06-sim-integration/06-02-SUMMARY.md: FOUND
- Commit 0f14c0f (RED): FOUND
- Commit 6b35826 (GREEN): FOUND
