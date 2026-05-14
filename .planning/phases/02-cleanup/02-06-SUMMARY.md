---
phase: 02-cleanup
plan: 06
subsystem: infra
tags: [controller-config, schema, openhd-bridge, web-server, mavlink, refactor]

# Dependency graph
requires:
  - phase: 02-cleanup
    provides: ControllerConfig dataclass with stabilised field set (after CLEAN-03 vfov removal in 02-02)
provides:
  - "ControllerConfig.tunable_fields() classmethod — single source of truth for runtime-mutable controller fields"
  - "TunableField NamedTuple (py_type, mavlink_id_or_None) schema type for downstream consumers"
  - "Migrated web_server and openhd_bridge to consume the unified schema; deleted both legacy dicts (_CONFIG_FIELDS, _CONFIG_PARAMS)"
affects: [phase-03-abstraction, openhd-mavlink-schema-extension-v1.2]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Single-source-of-truth schema lookup via classmethod on the owning dataclass"
    - "Web-UI-only fields encoded as mavlink_id=None in a shared schema (caller filters)"

key-files:
  created: []
  modified:
    - robot_follow/follow_api/config.py
    - robot_follow/servers/web_server.py
    - robot_follow/servers/openhd_bridge.py
    - robot_follow/tests/test_config_persistence.py

key-decisions:
  - "TunableField is a NamedTuple of (py_type, mavlink_id|None); minimal schema, easy to extend later with display_name/bounds without breaking call sites."
  - "Web-only fields (top_margin_safety, bottom_margin_safety) get mavlink_id=None — they are intentionally NOT exposed to OpenHD. Adding new OpenHD MAVLink IDs requires a C++ patch on the OpenHD side (see OpenHD/HAILO_INTEGRATION.md) and is deferred to v1.2."
  - "openhd_bridge consumes a private _openhd_tunable_fields() helper that filters out mavlink_id=None entries; web_server iterates the full schema unfiltered."
  - "Test-file porting: 4 tests in test_config_persistence.py that asserted membership in the old _CONFIG_FIELDS / _CONFIG_PARAMS dicts were updated in the same commit as the dict deletion (Task 2), so the suite stays green at every intermediate commit."

patterns-established:
  - "Schema as classmethod: when two consumer modules need the same field metadata, hoist the schema onto the owning dataclass as a classmethod rather than duplicating dicts in each consumer."
  - "Capability filtering at the consumer: rather than maintaining two separate schemas (one with subset, one with full), expose a single schema and let each consumer filter by capability (here, openhd_bridge filters mavlink_id != None)."

requirements-completed: [CLEAN-14]

# Metrics
duration: 5 min
completed: 2026-05-14
---

# Phase 02 Plan 06: ControllerConfig.tunable_fields() source of truth Summary

**Replaced two drifted runtime-mutable field dicts (web_server._CONFIG_FIELDS, 26 entries; openhd_bridge._CONFIG_PARAMS, 24 entries) with a single `ControllerConfig.tunable_fields()` classmethod returning a `dict[str, TunableField(py_type, mavlink_id|None)]` schema.**

## Performance

- **Duration:** 5 min
- **Started:** 2026-05-14T17:15:38Z
- **Completed:** 2026-05-14T17:20:36Z
- **Tasks:** 2
- **Files modified:** 4 (1 schema source, 2 consumers, 1 test)

## Accomplishments

- Hoisted runtime-mutable field schema onto `ControllerConfig` as a single classmethod — closes ROADMAP success criterion 4 ("no parallel altitude knob lists remain").
- Killed both legacy dicts (`_CONFIG_FIELDS` and `_CONFIG_PARAMS`) — grep across `robot_follow/servers/` returns zero hits.
- Locked the schema with `test_tunable_fields_source_of_truth` (asserts `>= 24` entries, every value is a `TunableField`, every key is a real `ControllerConfig` attribute, every `mavlink_id` is None or `<= 16` chars).
- Ported the 4 pre-existing tests that asserted membership in the deleted dicts in the same commit as the deletion (Task 2) — suite stays green at every intermediate commit.

## Pre-migration dict audit (Step A)

Captured before any deletion, for the historical record:

```
web fields count: 26
openhd params count: 24
shared count: 24
web-only: ['bottom_margin_safety', 'top_margin_safety']
openhd-only: []
```

The two-key web-only delta (`top_margin_safety`, `bottom_margin_safety`) are the only fields without MAVLink exposure. The unified schema therefore has 26 entries: 24 dual-exposed + 2 web-only with `mavlink_id=None`.

Note: planning RESEARCH § CLEAN-14 had estimated a ~3-key web-only delta. Actual count is 2.

## Task Commits

1. **Task 1: Add `ControllerConfig.tunable_fields()` classmethod** — `5511f11` (feat)
   - Added `TunableField` NamedTuple to `config.py`
   - Added `tunable_fields()` classmethod with all 26 entries (24 with MAVLink IDs copied verbatim from `_CONFIG_PARAMS`, 2 web-only with `mavlink_id=None`)
   - Added `test_tunable_fields_source_of_truth` to lock the schema invariants

2. **Task 2: Migrate consumers, delete legacy dicts** — `fed0db7` (refactor)
   - Deleted `_CONFIG_FIELDS` from `web_server.py`; `_handle_get_config` and `_handle_post_config` now iterate `ControllerConfig.tunable_fields()` directly, reading `schema.py_type` for coercion
   - Deleted `_CONFIG_PARAMS` from `openhd_bridge.py`; added private `_openhd_tunable_fields()` helper that filters to `mavlink_id != None`; `_listen_loop`, `_apply_config_param`, and `_send_report` all consume the filtered view
   - Updated 4 tests in `test_config_persistence.py` (kp_alt_hold + forward_velocity_deadband × bridge/web) to assert against `tunable_fields()` instead of the deleted dicts

**Plan metadata:** _pending docs commit below_

## Files Created/Modified

- `robot_follow/follow_api/config.py` — added `TunableField` NamedTuple and `ControllerConfig.tunable_fields()` classmethod (+69 lines)
- `robot_follow/servers/web_server.py` — deleted `_CONFIG_FIELDS` dict (28 lines); replaced two consumer sites with `ControllerConfig.tunable_fields()` iteration; added import (-42 / +14)
- `robot_follow/servers/openhd_bridge.py` — deleted `_CONFIG_PARAMS` dict (26 lines); added `_openhd_tunable_fields()` filter helper; updated three consumer sites (-26 / +34)
- `robot_follow/tests/test_config_persistence.py` — added `test_tunable_fields_source_of_truth`; updated 4 pre-existing tests to use `tunable_fields()` (+42 / -22, +1 net test)

## Decisions Made

- **TunableField schema is minimal `(py_type, mavlink_id|None)`.** Planner discussed possibly adding `display_name` and `bounds` fields, but NamedTuple's append-only field semantics mean any new field requires touching every constructor — so we keep the shape minimal for Phase 2 and add fields later only when there's an actual consumer. Captured in the inline docstring on `TunableField`.
- **Web-only fields stay web-only in Phase 2.** Extending OpenHD's MAVLink param table requires a C++ patch on the OpenHD side (the parameter ID strings are mapped in `OpenHD/HAILO_INTEGRATION.md`'s param table). That's outside the Python repo's blast radius and deferred to v1.2 — captured in the docstrings on both `TunableField` and `tunable_fields()`.
- **`_openhd_tunable_fields()` helper is private to `openhd_bridge.py`.** Could have been a method on `ControllerConfig` (e.g. `openhd_tunable_fields()`), but that would leak OpenHD's filtering policy into the controller config module. Keeping it private to the bridge keeps the abstraction one-sided: `ControllerConfig` exposes everything; each consumer decides what subset it consumes.

## Deviations from Plan

None — plan executed exactly as written. The PLAN's "~3-key" estimate in RESEARCH was off by one (actual delta is 2 keys, not 3), but that's an estimate discrepancy in planning, not an execution deviation.

## Issues Encountered

**Parallel-wave staging-area noise — handled correctly.** When Task 2 came to commit, `git status` showed 5 files modified (the 3 plan files plus 2 from a parallel agent's Wave 3 work on `mavsdk_drone.py` / `robot_follow_app.py`). Used `git add <explicit pathspec>` followed by `git commit -m "..." -- <pathspec>` to isolate Task 2's files. This matches the parallel-wave-hygiene lesson from the prompt's `additional_context` and prevents the kind of attribution drift that landed in Wave 1B (recorded in 02-01-SUMMARY.md). No accidental cross-plan staging occurred.

## User Setup Required

None — no external service configuration required.

## Next Phase Readiness

- ROADMAP success criterion 4 closed: "A single `ControllerConfig.tunable_fields()` call drives both `web_server` and `openhd_bridge` field lists; no parallel altitude knob lists remain."
- CLEAN-14 closed. Wave 3 remaining: CLEAN-15, CLEAN-16 (and any final Wave 3 plans).
- Phase 3 (Robot abstraction) can now grow new tunable fields by adding a single entry to `tunable_fields()` — no longer two parallel edits.
- v1.2 follow-up flagged: if/when we want `top_margin_safety` / `bottom_margin_safety` on OpenHD MAVLink, the Python side just flips `mavlink_id=None` → `"DF_TOP_MAR"` / `"DF_BOT_MAR"`; the corresponding OpenHD C++ patch is the gating work.

---
*Phase: 02-cleanup*
*Completed: 2026-05-14*

## Self-Check: PASSED

- `[ -f robot_follow/follow_api/config.py ]` — FOUND
- `[ -f robot_follow/servers/web_server.py ]` — FOUND
- `[ -f robot_follow/servers/openhd_bridge.py ]` — FOUND
- `[ -f robot_follow/tests/test_config_persistence.py ]` — FOUND
- Task 1 commit `5511f11` — FOUND in `git log --oneline`
- Task 2 commit `fed0db7` — FOUND in `git log --oneline`
- `grep -rE '_CONFIG_FIELDS\b|_CONFIG_PARAMS\b' robot_follow/servers/` returns 0 matches
- `ControllerConfig.tunable_fields()` returns 26 entries, all `TunableField` — verified
- Full pytest suite (excluding sim): 153 passed, 2 pre-existing baseline failures (DEFER-02-00-A), 21 xfailed — no regressions
