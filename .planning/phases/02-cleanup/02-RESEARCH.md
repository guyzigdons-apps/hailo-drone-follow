# Phase 2: Cleanup - Research

**Researched:** 2026-05-14
**Domain:** Codebase hygiene — dead-code deletion, duplication merges, hot-path concurrency fix
**Confidence:** HIGH (every claim is a grep + line-number on the post-rename tree)

## Summary

Phase 2 is 18 surgical edits on a known surface. No new libraries, no new patterns — every CLEAN item already names the file, line, and dead construct. All 18 line references in `REQUIREMENTS.md` were captured pre-rename; **every one of them still resolves cleanly post-rename** because the rename was a pure path swap (`drone_follow/` → `robot_follow/`). Specifically: mavsdk_drone.py:47, drone_follow_app.py:54 (now `robot_follow_app.py:54`), vision_branches.py:397, state.py:23, mavsdk_drone.py:642, web_server.py:84, openhd_bridge.py:332, hailo_drone_detection_manager.py:88 — all verified.

Risk is concentrated in three places: (a) CLEAN-16 is a concurrency primitive change in the SSE/MJPEG hot path; (b) CLEAN-07 is a test-breaking signature change (`test_shared_state.py:47` calls `state.update(None)` with one argument); (c) CLEAN-14 + CLEAN-15 each touch three files and need ordered serialisation. Everything else is independent and parallelisable.

**Primary recommendation:** Three waves. Wave 1 = all 10 dead-code deletes (CLEAN-01..10) parallel within edit-isolated groups + CLEAN-07 fixup in the same commit as its test update. Wave 2 = duplication merges (CLEAN-11..15) serialised by file. Wave 3 = hot-path fixes (CLEAN-16..18), with CLEAN-16 getting a dedicated two-tab integration test before merge.

## User Constraints

No CONTEXT.md exists for Phase 2 (no `/gsd:discuss-phase` was run). The constraints are derived from ROADMAP success criteria and STATE.md "Accumulated Context":

### Phase 2 ROADMAP Success Criteria (locked)

1. `sim/world_loader.py` and `scripts/bench_reid_callback.py` do not exist; `grep -r "world_loader\|bench_reid" .` returns nothing outside git history.
2. `drone-follow --input usb --webui` starts and serves the web UI; no regression in any existing flag path.
3. Web UI MJPEG stream delivers a frame to a second simultaneous browser tab without either tab falling through to the 2 s SSE timeout (CLEAN-16 race fixed).
4. A single `ControllerConfig.tunable_fields()` call drives both `web_server` and `openhd_bridge` field lists; no parallel altitude knob lists remain (CLEAN-14).
5. Branch-decision tree (display/record/webui/openhd selection) is defined in one place in `vision_branches`; implicit-display rule appears exactly once (CLEAN-15).

### Out-of-scope (NOT to touch in Phase 2)

- Anything tagged `ABS-*`, `ROVER-*`, `RSIM-*`, `RINT-*` (Phases 3-6). In particular:
  - **Don't** introduce a `Robot` protocol while merging `_CONFIG_FIELDS` / `_CONFIG_PARAMS` (CLEAN-14). Just hoist a `tunable_fields()` classmethod onto `ControllerConfig`. Phase 3 will move altitude fields to `Optional[float]` — Phase 2 doesn't pre-empt that.
  - **Don't** rename `run_drone()` → `run_robot()` (that's ABS-08).
  - **Don't** factor `MavsdkDroneAdapter` (that's ABS-03).
- Anything not in CLEAN-01..18 (no opportunistic refactors).
- Don't add new test files beyond what each CLEAN item minimally requires; reuse the existing 14 test files.

## Phase Requirements

| ID | Description | Research Support |
|----|-------------|-----------------|
| CLEAN-01 | Delete `sim/world_loader.py` | Per-item: zero callers confirmed |
| CLEAN-02 | Delete `scripts/bench_reid_callback.py` | Per-item: imports nonexistent `reid_worker` |
| CLEAN-03 | Delete `vfov` field + `--vfov` flag | Per-item: zero readers in entire tree |
| CLEAN-04 | Remove or document `--mission-duration` | Per-item: load-bearing, **document** |
| CLEAN-05 | Replace `getattr(args, "serial_baud", 115200)` with `args.serial_baud` | Per-item: arg always registered |
| CLEAN-06 | Remove `strip_tiles_and_highlight_target` alias | Per-item: zero external callers |
| CLEAN-07 | Remove `available_ids=None` default from `state.py:update()` | Per-item: every caller passes a set (one test exception, must update) |
| CLEAN-08 | Remove `shutdown_read_fd` param + pipe-reader block from `mavsdk_drone.py:642` | Per-item: single caller doesn't pass it |
| CLEAN-09 | Remove `_NULLABLE_FIELDS = set()` and dead branches | Per-item: empty set gates dead branches |
| CLEAN-10 | Remove `controller_config=` kwarg from `create_app()` | Per-item: attached after via `app.user_data.controller_config = ...` |
| CLEAN-11 | Unified `_reap_mavsdk_server()` helper | Per-item: 2 call sites at `mavsdk_drone.py:225` and `robot_follow_app.py:448` |
| CLEAN-12 | Collapse 3 pre-parsers → 1 | Per-item: 3 independent parsers, no order dependency |
| CLEAN-13 | Merge `_telemetry_position_task` + `_telemetry_altitude_task` | Per-item: both subscribe to `drone.telemetry.position()` |
| CLEAN-14 | `ControllerConfig.tunable_fields()` source-of-truth | Per-item: two parallel lists, schema needs `(mavlink_id, type)` |
| CLEAN-15 | Single branch-decision tree + single implicit-display rule site | Per-item: rule appears at 3 sites; consolidate in `vision_branches.assemble_output_stage` (already exists, just needs to own the rule) |
| CLEAN-16 | Fix SSE race — `Event.set() / clear()` → `Condition` + monotonic `frame_seq` | Per-item + deep dive: classic Python primitive misuse |
| CLEAN-17 | Reuse socket in `_send_immediate_report` | Per-item: 4 call sites; new socket per call |
| CLEAN-18 | One-shot `{id(p): p}` lookup dict | Per-item: O(n) `next(...)` inside O(k) outer = O(n*k) |

---

## Per-Item Analysis

### CLEAN-01: Delete `sim/world_loader.py`

- **File:** `sim/world_loader.py` (139 lines)
- **Callers:** **NONE** outside the file itself
  - `grep -rnE "world_loader|from sim.world_loader|import world_loader" --include='*.py' --include='*.sh' --include='*.md' --include='*.json' --include='*.toml' --exclude-dir={.git,.planning,hailo-apps,__pycache__,node_modules} .`
  - Returns ONLY `sim/world_loader.py:103:            prefix=".world_loader_",` (an internal `tempfile` prefix string, not an import)
- **Proposed change:** `git rm sim/world_loader.py`
- **Risk:** None. Orphan since `start_sim.sh` switched to `PX4_GZ_WORLD` env var.
- **Test surface:** None affected.

### CLEAN-02: Delete `scripts/bench_reid_callback.py`

- **File:** `scripts/bench_reid_callback.py`
- **Status:** Already touched by Phase 1 rename — imports rewritten to `from robot_follow.pipeline_adapter.reid_manager import ReIDManager` and `from robot_follow.pipeline_adapter.reid_worker import ReIDWorker`. Confirmed at lines 38-39.
- **Why dead:** `reid_worker` module does not exist (`find . -name "reid_worker*"` returns nothing in `robot_follow/pipeline_adapter/`). The script has been broken since pre-milestone — the `ReIDWorker` async path was removed but the benchmark wasn't deleted.
- **External refs:** None in `README.md`, `CLAUDE.md`, `TROUBLESHOOTING.md`, `docs/`. No runbook references. `grep -rnE "bench_reid"` returns nothing outside the file itself.
- **Proposed change:** `git rm scripts/bench_reid_callback.py`
- **Risk:** None.
- **Test surface:** None.

### CLEAN-03: Delete `vfov` field + `--vfov` flag

- **Sites:**
  - `robot_follow/follow_api/config.py:21` — `vfov: float = 41.0` dataclass field
  - `robot_follow/follow_api/config.py:172` — `group.add_argument("--vfov", ...)` CLI flag
  - `robot_follow/follow_api/config.py:278` — `vfov=_arg("vfov", default=defaults.vfov)` constructor wiring
- **Readers:** **NONE**
  - `grep -rnE "\.vfov|config\.vfov|cfg\.vfov"` returns ONLY the 3 sites above (the definitions themselves). No `.vfov` access anywhere in `controller.py` or any other consumer. Compare to `hfov` which is read at `controller.py:147` and `controller.py:158`.
- **External presence (informational, NOT touched):**
  - `df_config.example.json:3`, `sim/configs/simulation.json:7`, `sim/configs/simulation_follow.json:11` all have `"vfov": 41.0` keys. After CLEAN-03 these become unknown keys; `ControllerConfig.from_args` and `load_from_file` likely ignore unknown keys (verify in plan), but if they hard-fail, plan must also strip the JSON entries.
- **Proposed change:** Delete the 3 source lines. Verify config-loading tolerates the legacy `"vfov"` key in user JSON (graceful drop) or strip it from the example/sim configs.
- **Risk:** LOW. Possible hidden coupling: if `from_args` / `load_from_file` enforce a strict schema. **Verify during planning** — read `ControllerConfig.from_args` and `load_from_file` to confirm extra-key tolerance.
- **Test surface:** `test_config_persistence.py` — verify no `vfov` assertion. Add a delete-then-roundtrip test if not present.

### CLEAN-04: `--mission-duration` — DOCUMENT, do not remove

- **Site:** `robot_follow/drone_api/mavsdk_drone.py:47`
- **Usage:** Load-bearing. Read at:
  - Line 767 — `asyncio.create_task(asyncio.sleep(args.mission_duration))` in the `--takeoff-landing` branch (auto-land timeout, default 300 s)
  - Line 799 — same pattern in the no-takeoff-landing branch (per-iteration timeout)
- **Documentation:** Mentioned in `docs/control-architecture.md:250` as "mission-duration expiry → `_land_safely()`" but **never in README, CLAUDE.md, or the `--help` blurb itself** (help text is empty: `group.add_argument("--mission-duration", type=float, default=300.0)` with no `help=...`).
- **Hazard:** Default 300 s silent auto-land after 5 min — operator surprise.
- **Recommendation:** **Document, not remove**. Concrete edits:
  1. Add `help=` string at `mavsdk_drone.py:47`: e.g. `"Maximum mission duration in seconds before auto-land/auto-reconnect (default: 300.0 = 5 min). With --takeoff-landing: triggers a land. Without: triggers a control-loop restart."`
  2. Add a one-line note in CLAUDE.md "Key CLI Flags" section so operators know about the timeout.
- **Risk:** LOW. Pure documentation + help-string change.
- **Test surface:** Spot-check `--help` output mentions mission-duration with a help string (could be a smoke assertion in `test_install_smoke.py` but not required).

### CLEAN-05: Replace `getattr(args, "serial_baud", 115200)` with `args.serial_baud`

- **Site:** `robot_follow/robot_follow_app.py:54` (inside `_resolve_serial_connection`)
- **Why dead:** `--serial-baud` is unconditionally registered at `mavsdk_drone.py:41` with `default=57600`. The parser is always built before `_resolve_serial_connection` runs (see `robot_follow_app.py:264` `parser = _build_parser()` → `robot_follow_app.py:324` `args = app.options_menu`). The fallback `115200` is unreachable.
- **Lie:** If it somehow did fire, the fallback `115200` contradicts the registered default `57600` — a fallback that lies is worse than no fallback.
- **Proposed change:** Replace line 54 with `baud = args.serial_baud`. (The outer `if getattr(args, "serial", None) is not None:` guard stays.)
- **Risk:** None.
- **Test surface:** Existing `test_install_smoke.py` covers `--help` parsing path. No new test required.

### CLEAN-06: Remove `strip_tiles_and_highlight_target` alias

- **Site:** `robot_follow/pipeline_adapter/vision_branches.py:397` — `strip_tiles_and_highlight_target = highlight_target`
- **Callers of the alias:** **NONE**
  - `grep -rn "strip_tiles_and_highlight_target"` returns ONLY the definition line (line 397)
  - Internal callers all use `highlight_target` directly: `vision_branches.py:419`, `hailo_drone_detection_manager.py:841` (in a comment), `hailo_drone_detection_manager.py:1016` (in a comment)
- **Proposed change:** Delete line 397.
- **Risk:** None.
- **Test surface:** None.

### CLEAN-07: Remove `available_ids=None` default from `state.update()`

- **Site:** `robot_follow/follow_api/state.py:19`
  ```python
  def update(self, detection: Optional[Detection], available_ids: set = None):
      with self._lock:
          self._detection = detection
          self._frame_count += 1
          if available_ids is not None:
              self._available_ids = available_ids
  ```
- **Callers (production):** All pass `available_ids=` explicitly:
  - `hailo_drone_detection_manager.py:283` — `available_ids=set()`
  - `hailo_drone_detection_manager.py:377, 437, 464, 472, 494, 501, 534` — all pass `available_ids=available_ids` or `set()`
- **Callers (tests):** Two patterns:
  - `test_follow_server.py:48, 64, 116, 154` — pass `available_ids={...}` ✓ OK
  - `test_shared_state.py:32, 40, 46, 47, 54, 55, 56, 63, 66, 82, 104` — call `state.update(d)` or `state.update(None)` with **ONE argument** ✗ BREAKS
- **Proposed change:** Two-part edit, **single commit**:
  1. `state.py:19` → `def update(self, detection: Optional[Detection], available_ids: set):`
  2. Update every `test_shared_state.py` call site to pass `available_ids=set()` (the test only cares about `detection` + `frame_count`).
- **Risk:** MEDIUM (test surface). Both edits in same commit so `pytest` stays green on each commit.
- **Test surface:** `test_shared_state.py` 11 call sites. After edit, `python -m pytest robot_follow/tests/test_shared_state.py -v` must show 100% pass.

### CLEAN-08: Remove `shutdown_read_fd` param + pipe-reader block

- **Site:** `robot_follow/drone_api/mavsdk_drone.py:642-665`
  ```python
  async def run_live_drone(args, shared_state, shutdown, shutdown_read_fd=None,
                           config=None, ui_state=None, target_state=None):
      ...
      if shutdown_read_fd is not None:
          loop = asyncio.get_running_loop()
          def _on_shutdown_pipe(): ...
          loop.add_reader(shutdown_read_fd, _on_shutdown_pipe)
  ```
- **Single caller:** `robot_follow/robot_follow_app.py:396`:
  ```python
  loop.run_until_complete(
      run_live_drone(args, shared_state, shutdown,
                    config=controller_config, ui_state=ui_state,
                    target_state=target_state))
  ```
  Does **not** pass `shutdown_read_fd=`. The whole block (lines 652-665) is unreachable.
- **Proposed change:** Remove the `shutdown_read_fd=None` parameter and the `if shutdown_read_fd is not None:` block (lines 652-665). Also remove the unused `import os` if it's no longer needed (verify — `os` is likely used elsewhere; in mavsdk_drone it's used for env vars and `os.getuid()` so keep).
- **Risk:** LOW.
- **Test surface:** None directly. Smoke: `drone-follow --help` parses without error.

### CLEAN-09: Remove `_NULLABLE_FIELDS = set()` and dead branches

- **Sites:**
  - `robot_follow/servers/web_server.py:325` — local `_NULLABLE_FIELDS = set()`
  - `robot_follow/servers/web_server.py:333` — `if key in _NULLABLE_FIELDS and (value is None or value == 0):` (dead branch)
  - `robot_follow/servers/openhd_bridge.py:68` — module-level `_NULLABLE_FIELDS = set()`
  - `robot_follow/servers/openhd_bridge.py:310` — `if param_name in _NULLABLE_FIELDS and value == 0:` (dead branch)
  - `robot_follow/servers/openhd_bridge.py:359` — `if param_name in _NULLABLE_FIELDS and py_value is None:` (dead branch)
- **Why dead:** Empty set → branch never taken.
- **Proposed change:** Delete the 5 lines on web_server.py and the 3 dead `if` branches on openhd_bridge.py (collapse `if x: do_a else: do_b` → `do_b`).
- **Risk:** LOW. Touches the same files as CLEAN-14 (`_CONFIG_FIELDS` consolidation). **Group with CLEAN-14 in same wave** to avoid merge conflicts.
- **Test surface:** No dedicated test for `_NULLABLE_FIELDS`. Smoke test: `drone-follow --webui` plus `POST /api/config` with one numeric field round-trips correctly.

### CLEAN-10: Remove `create_app(controller_config=...)` kwarg

- **Site:** `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py:695, 731, 739, 1285`
  - Line 695: `def create_app(... controller_config=None, ...)`
  - Line 731: `DroneFollowUserData.__init__(... controller_config=None, ...)`
  - Line 739: `self.controller_config = controller_config`
  - Line 1285: `controller_config=controller_config` (passes from outer scope to inner class)
- **Single caller of `create_app`:** `robot_follow/robot_follow_app.py:317-323` — does NOT pass `controller_config=`
- **What happens instead:** `robot_follow_app.py:340` — `app.user_data.controller_config = controller_config` (attached after construction)
- **Proposed change:**
  1. Remove `controller_config=None` from `create_app()` signature (line 695)
  2. Remove `controller_config=None` from `DroneFollowUserData.__init__` (line 731) — but **keep** `self.controller_config = None` initialised in `__init__` so the attribute exists (`hailo_drone_detection_manager.py:278` does `config = user_data.controller_config`).
  3. Remove the `controller_config=controller_config` kwarg at line 1285
- **Risk:** LOW. The attribute is still set externally at `robot_follow_app.py:340` (unchanged).
- **Test surface:** Smoke: `drone-follow --help` parses; `python -c "from robot_follow.pipeline_adapter import create_app"` imports without error.

### CLEAN-11: Unified `_reap_mavsdk_server()` helper

- **Call sites (2):**
  - `robot_follow/drone_api/mavsdk_drone.py:225` — inside `DetachedMavsdkServer.__enter__` (reap stale server before starting fresh one)
  - `robot_follow/robot_follow_app.py:448` — inside `finally` block when drone thread fails to join (reap survivor process)
- **Both invocations:**
  ```python
  subprocess.run(["pkill", "-9", "-u", str(os.getuid()), "-f", "mavsdk_server"],
                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3)
  ```
  (Wrapped in `try/except (OSError, subprocess.TimeoutExpired)`)
- **Proposed helper (location: `robot_follow/drone_api/mavsdk_drone.py`):**
  ```python
  def _reap_mavsdk_server() -> None:
      """Kill any stragglers from prior runs (scoped to current uid)."""
      try:
          subprocess.run(
              ["pkill", "-9", "-u", str(os.getuid()), "-f", "mavsdk_server"],
              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3,
          )
      except (OSError, subprocess.TimeoutExpired):
          pass
  ```
  Export via `robot_follow/drone_api/__init__.py` so `robot_follow_app.py` can `from robot_follow.drone_api import _reap_mavsdk_server` (or expose as public `reap_mavsdk_server` — minor naming choice for the planner).
- **Risk:** LOW. Same external semantics. The `time.sleep(0.3)` after the pkill at line 230 stays inline at the caller (it's specific to the `__enter__` flow before starting a new server; the `finally`-block caller doesn't need it).
- **Test surface:** None directly. Smoke: connection still works.

### CLEAN-12: Collapse 3 pre-parsers → 1

- **Site:** `robot_follow/robot_follow_app.py:212-220, 267-290, 312-314`
- **Current parsers (independent — no order dependency):**
  - `ui_pre` (lines 212-220): `--webui`, `--webui-port`, `--webui-fps`, `--openhd`, `--display`, `--record`, `--log-perf`
  - `reid_pre` (lines 267-290): `--reid-model`, `--no-reid`, `--update-interval`, `--reid-threshold`, `--reid-timeout`, `--reid-drift-threshold`, `--reid-duplicate-threshold`, `--reid-refresh-every`, `--reid-min-gallery-for-drift-check`, `--reid-bootstrap-consistency`, `--reid-overlap-skip-iou`, `--reid-dump-embeddings`
  - `tracker_pre` (lines 312-314): `--tracker`
- **Why independent:**
  - `ui_pre_args` used immediately at lines 222-262 (mutual-exclusion check, implicit-display rule, record-branch gating, web_server import gate). These checks must run BEFORE `_build_parser()` builds the heavy parser (because import-gating drives whether to build the UI).
  - `reid_pre_args` used at lines 292-306 to construct `ReIDManager` BEFORE `create_app`.
  - `tracker_pre_args` used at line 322 in `create_app(... tracker_name=tracker_pre_args.tracker, ...)`.
  - **All three feed `create_app` in one call (line 317-323).** Their `parse_known_args()` calls are independent — `argparse.ArgumentParser(add_help=False).parse_known_args()` is idempotent on `sys.argv` and doesn't consume.
- **Proposed single-pass shape:**
  ```python
  pre = argparse.ArgumentParser(add_help=False)
  # UI / output branches
  pre.add_argument("--webui", action="store_true")
  pre.add_argument("--webui-port", type=int, default=5001)
  ...
  # ReID
  pre.add_argument("--reid-model", type=str, default=_DEFAULT_REID_HEF)
  pre.add_argument("--no-reid", action="store_true")
  ...
  # Tracker
  pre.add_argument("--tracker", default=DEFAULT_TRACKER, choices=TRACKER_CHOICES)
  pre_args, _ = pre.parse_known_args()
  # Then use pre_args.webui, pre_args.reid_model, pre_args.tracker, etc.
  ```
  All downstream references `ui_pre_args.X` / `reid_pre_args.X` / `tracker_pre_args.X` → `pre_args.X`.
- **Risk:** LOW. Mechanical rename. Reduces 3 `parse_known_args()` calls to 1.
- **Caveat for planner:** The `from ... import TRACKER_CHOICES, DEFAULT_TRACKER` at line 311 (just above `tracker_pre`) is currently late so it doesn't run if `--no-reid` is set early — but actually it imports the module unconditionally. Move the import to top of the function or near the consolidated pre-parser. No behavioural difference.
- **Test surface:** `drone-follow --help` parses; all three flag groups still recognised. Existing `test_install_smoke.py::test_help_outputs_byte_identical` is the gate.

### CLEAN-13: Merge `_telemetry_position_task` + `_telemetry_altitude_task`

- **Sites:** `robot_follow/drone_api/mavsdk_drone.py:345-353` (altitude) and `:369-381` (position)
- **Current shape:**
  ```python
  async def _telemetry_altitude_task(drone, altitude_cache: dict, shutdown: asyncio.Event) -> None:
      async for position in drone.telemetry.position():
          if shutdown.is_set(): return
          altitude_cache["m"] = position.relative_altitude_m

  async def _telemetry_position_task(drone, telemetry_cache: dict, shutdown: asyncio.Event) -> None:
      async for pos in drone.telemetry.position():
          if shutdown.is_set(): return
          telemetry_cache["lat"] = pos.latitude_deg
          telemetry_cache["lon"] = pos.longitude_deg
          telemetry_cache["abs_alt"] = pos.absolute_altitude_m
          telemetry_cache["rel_alt"] = pos.relative_altitude_m
  ```
- **Both subscribe to the same stream** (`drone.telemetry.position()`). Two subscribers double the MAVLink bandwidth cost; the rate-limit is on the stream side (MAVSDK requests `position` at a fixed rate regardless of subscriber count — but the code does open two async iterators on the same stream method, which MAVSDK handles via internal fan-out). The merge eliminates one of them.
- **Consumers:**
  - `altitude_cache["m"]` read at `mavsdk_drone.py:487, 528` (live_control_loop altitude-hold P-loop)
  - `telemetry_cache["rel_alt"]` read at `mavsdk_drone.py:392` (telemetry log task)
  - `telemetry_cache["lat"|"lon"|"abs_alt"]` read at `mavsdk_drone.py:401-405` (telemetry log task)
- **Note:** Both `altitude_cache["m"]` AND `telemetry_cache["rel_alt"]` carry `position.relative_altitude_m` — same data, two homes.
- **Two viable shapes:**
  - **A (minimal):** Keep both dict outputs, single task:
    ```python
    async def _telemetry_position_task(drone, telemetry_cache: dict, altitude_cache: dict,
                                       shutdown: asyncio.Event) -> None:
        async for pos in drone.telemetry.position():
            if shutdown.is_set(): return
            telemetry_cache["lat"] = pos.latitude_deg
            telemetry_cache["lon"] = pos.longitude_deg
            telemetry_cache["abs_alt"] = pos.absolute_altitude_m
            telemetry_cache["rel_alt"] = pos.relative_altitude_m
            altitude_cache["m"] = pos.relative_altitude_m
    ```
    Spawn-site at lines 709 + 718 → single spawn. `_start_altitude_telemetry()` (defined at lines 730-744) becomes either a no-op or simply ensures the cache exists.
  - **B (cache consolidation):** Drop the separate `altitude_cache` dict; have `live_control_loop` read `telemetry_cache.get("rel_alt")` instead of `altitude_cache.get("m")`. Cleaner long-term but touches more code (`live_control_loop` signature, line 413, 487, 528). Bigger blast radius — defer or include only if the planner is comfortable.
- **Recommendation: Shape A** (single task writes both dicts). Drops half the duplicate work and preserves all consumer APIs.
- **Risk:** MEDIUM. The two tasks have slightly different lifecycle in current code (`_telemetry_position_task` is spawned unconditionally at line 709; `_telemetry_altitude_task` is spawned later inside `_start_altitude_telemetry` at line 730 — only when `live_control_loop` actually starts). Merging means altitude_cache populates earlier. **Verify this doesn't cause spurious altitude-hold triggers** — but altitude-hold only runs inside `live_control_loop`, which gates on its own conditions, so populating the cache early is harmless.
- **Test surface:** Existing telemetry tests (none specific to this task). Smoke under SITL: `drone-follow --input udp://0.0.0.0:5600 --takeoff-landing` reaches OFFBOARD, altitude P-loop holds altitude.

### CLEAN-14: `ControllerConfig.tunable_fields()` source of truth

- **Sites:**
  - `robot_follow/servers/web_server.py:259-286` — `_CONFIG_FIELDS = {name: type}` (27 entries)
  - `robot_follow/servers/openhd_bridge.py:40-65` — `_CONFIG_PARAMS = {name: (mavlink_id, type)}` (24 entries)
- **Diff between the two:**
  - `web_server._CONFIG_FIELDS` has 27 keys; `openhd_bridge._CONFIG_PARAMS` has 24 keys
  - Missing from `_CONFIG_PARAMS` (3): `dead_zone_deg` (actually both have it), `top_margin_safety`, `bottom_margin_safety` — verify in plan
  - Each `_CONFIG_PARAMS` value carries a `("DF_*", type)` tuple (MAVLink param ID needed for the wire), each `_CONFIG_FIELDS` value is just a type
- **Proposed schema for `tunable_fields()`:**
  ```python
  @classmethod
  def tunable_fields(cls) -> dict[str, "TunableField"]:
      """Source of truth for which dataclass fields are runtime-mutable.

      Returns {attr_name: TunableField(py_type, mavlink_id_or_None)}.
      web_server reads .py_type; openhd_bridge reads both.
      """
  ```
  With `TunableField` a small frozen dataclass or NamedTuple:
  ```python
  class TunableField(NamedTuple):
      py_type: type
      mavlink_id: Optional[str]  # None if not exposed to OpenHD
  ```
- **Migration:**
  - `web_server.py:293`: `for k in cls.tunable_fields():`
  - `web_server.py:328-336`: `expected = ControllerConfig.tunable_fields()[key].py_type`
  - `openhd_bridge.py:176, 307, 357`: iterate `ControllerConfig.tunable_fields().items()`, unpack `(py_type, mavlink_id)`
- **Risk:** MEDIUM. Subtle list-of-fields differences between the two consumers need reconciliation. **The planner MUST audit the 3-key diff** (run a diff of the two sets before consolidation) and confirm with the user / current behaviour which is correct. The merged list may differ from either current list.
- **Decision needed:** Should `top_margin_safety` / `bottom_margin_safety` be exposed to OpenHD? Likely YES (they're safety knobs the operator may want to tune in flight). Plan must decide.
- **Test surface:**
  - Existing `test_config_persistence.py` may cover `to_dict`/`from_dict` round-trip; verify all `tunable_fields()` keys are included.
  - New assertion (small): `len(ControllerConfig.tunable_fields()) == N` where N is locked.
  - Smoke: web UI POSTs a tunable field and reads it back; QOpenHD sets a tunable field and the value applies.

### CLEAN-15: Single branch-decision tree + single implicit-display rule

- **Implicit-display rule appears at 3 sites:**
  1. `robot_follow/robot_follow_app.py:226-227` — `if not ui_pre_args.openhd and not ui_pre_args.webui: ui_pre_args.display = True` (pre-parser)
  2. `robot_follow/robot_follow_app.py:330-331` — `if not getattr(args, "openhd", False) and not getattr(args, "webui", False): args.display = True` (after `create_app` populates `args`)
  3. `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py:1166-1167` — `if not openhd and not webui: display = True` (inside `_build_pipeline`)
- **Branch-build call** (`assemble_output_stage`) at `robot_follow/pipeline_adapter/vision_branches.py:267-307` — already centralised; it decides webui/openhd/local branches based on its boolean args. **Good — this is the consolidation target.**
- **Other branch-decision logic:**
  - `robot_follow_app.py:240-244` — `record_branch_enabled = (ui_pre_args.record or ui_pre_args.webui or ui_pre_args.openhd)` (record branch gating)
  - `robot_follow_app.py:222-224` — `--openhd` + `--webui` mutual exclusion check
  - `robot_follow_app.py:325-326` — SAME mutual-exclusion check duplicated after full parse
- **Proposed consolidation:** Add a helper to `vision_branches.py` that takes the raw flags and returns a structured `BranchDecision`:
  ```python
  @dataclass(frozen=True)
  class BranchDecision:
      display: bool
      record_branch_enabled: bool   # whether to build the branch (may be valve-gated at runtime)
      webui: bool
      openhd: bool

  def decide_branches(*, openhd: bool, webui: bool, display: bool, record: bool) -> BranchDecision:
      """Apply the implicit-display rule and record-branch gating once, in one place."""
      if openhd and webui:
          raise ValueError("--openhd and --webui are mutually exclusive "
                           "(only one network encoder may run at a time)")
      if not openhd and not webui:
          display = True
      record_branch_enabled = record or webui or openhd
      return BranchDecision(display=display, record_branch_enabled=record_branch_enabled,
                            webui=webui, openhd=openhd)
  ```
- **Call from `robot_follow_app.py`** (after pre-parse): one call, store the result, use throughout. **Call from `hailo_drone_detection_manager._build_pipeline`** (line ~1160): replace the inline `if not openhd and not webui: display = True` with `decide_branches(...)` — but **carefully**: this method reads `self.options_menu.openhd / .webui / .display` which is the post-full-parse args, and the result must match what the pre-parser decided. The simplest contract is: the pre-parser is authoritative and writes back to `args`, so by the time `_build_pipeline` runs, `args.display` is already correct. **Removing line 1167's redundant rule is the easier path.**
- **Risk:** MEDIUM. The 3 sites must produce **bit-identical** decisions for every flag combination. Test matrix: { `--display`, `--openhd`, `--webui`, `--record`, none } × { combinations of 0, 1, 2 flags } = 16 combinations. Most are no-ops, but `--openhd --webui` raising and `none → display=True` must be preserved.
- **Caveat:** The first mutual-exclusion check at line 222-224 fires **before** `_build_parser()`, so the error message is from the pre-parser. After consolidation, the check still has to run at pre-parse time (so we don't waste resources building anything if invalid). The duplicate check at 325-326 can be removed.
- **Test surface:**
  - Add a unit test for `decide_branches`: 16 input combos × expected output.
  - Smoke: existing `--openhd --webui` rejection ($? != 0); `drone-follow --input usb` defaults `display=True`; `drone-follow --input usb --webui` sets `display=False`.

### CLEAN-16: SSE race fix — `Event.set()/clear()` → `Condition` + monotonic `frame_seq`

See **Deep Dive** section below for full before/after and test plan.

- **Site:** `robot_follow/servers/web_server.py:84-85` (the offending `set() / clear()` pair)
- **Risk:** MEDIUM-HIGH. Concurrency primitive change in the hot path. Two consumers (MJPEG + SSE) per browser tab, multiplied by the number of open tabs.
- **Test surface:** New integration test — two browser tabs simultaneously fetch `/api/video` and `/api/detections/stream`; both must receive frames within < 2 s of pipeline start.

### CLEAN-17: Reuse socket in `_send_immediate_report`

- **Site:** `robot_follow/servers/openhd_bridge.py:332-340`
  ```python
  def _send_immediate_report(self):
      """Send a one-shot report on a transient socket (callable from any thread)."""
      sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      try:
          self._send_report(sock)
      except OSError:
          pass
      finally:
          sock.close()
  ```
- **Call sites (4):** Lines 207, 272, 288, 303 — all from inside `_apply_*` handlers, invoked on the **listener thread** (in `_listen_loop`).
- **Existing `self._sock` (listener socket):** Bound to `127.0.0.1:listen_port` (line 114), used for `recvfrom` in `_listen_loop` (line 145). UDP sockets are bidirectional, and `sendto` is thread-safe on the same socket. The destination is `("127.0.0.1", self._report_port)` (line 428) — different port, so no self-loopback.
- **Existing `_report_loop`** at lines 342-352 keeps its own dedicated `report_sock` (separate from `self._sock`).
- **Two viable proposals:**
  - **A (recommended): reuse `self._sock`.** Both `_send_immediate_report` and `_report_loop` use `self._sock`. UDP `sendto` is atomic on Linux for messages < MTU; Python's `socket.socket` is thread-safe for `sendto` calls. Removes both the immediate-socket-per-call AND the periodic `report_sock` allocation.
  - **B (conservative): dedicated `self._report_sock`.** Created in `start()`, used by both immediate and periodic. Avoids any concern that immediate reports might delay listener `recvfrom` (they won't — `sendto` is non-blocking for UDP — but provides isolation).
- **Recommendation: A** unless the planner has a reason to keep listener/sender isolated. A is simpler and the `recvfrom` blocking is timeout-bounded (1 s) so doesn't matter for `sendto` interleaving.
- **Risk:** LOW.
- **Test surface:** Smoke — toggle follow_id from QOpenHD (or simulate via `python -c "import socket; s = socket.socket(...); s.sendto(...)"`) and observe the bridge's `_apply_follow_id` runs and the report reaches `:5511`. Existing flow under `--openhd` testing on SITL covers this.

### CLEAN-18: One-shot `{id(p): p}` lookup

- **Site:** `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py:88` (inside `_update_ui`)
- **Current code:**
  ```python
  for p in persons:
      tid = person_to_id.get(id(p))
      if tid is None: continue
      prev = keep_obj_id_by_tid.get(tid)
      if prev is None:
          keep_obj_id_by_tid[tid] = id(p)
      else:
          prev_p = next((q for q in persons if id(q) == prev), None)  # O(n)
          if prev_p is not None and p.get_confidence() > prev_p.get_confidence():
              keep_obj_id_by_tid[tid] = id(p)
  ```
- **Cost:** Worst case O(n²) when many persons share track ids (e.g. multi-scale tile duplicates). Typical n is small (~20 detections per frame) so absolute cost is bounded, but this runs on every frame on the GStreamer thread.
- **Proposed change:**
  ```python
  person_by_obj_id = {id(p): p for p in persons}  # O(n), built once
  for p in persons:
      tid = person_to_id.get(id(p))
      if tid is None: continue
      prev = keep_obj_id_by_tid.get(tid)
      if prev is None:
          keep_obj_id_by_tid[tid] = id(p)
      else:
          prev_p = person_by_obj_id.get(prev)  # O(1)
          if prev_p is not None and p.get_confidence() > prev_p.get_confidence():
              keep_obj_id_by_tid[tid] = id(p)
  ```
- **Risk:** LOW. Mechanical refactor. Identical semantics.
- **Test surface:** None directly. Verify under load (2+ persons + multi-scale) the UI still de-duplicates correctly (`--webui` + 2-person sim world).

---

## File-Conflict Matrix

Critical for wave grouping. Each row is a file; columns show which CLEAN items modify it.

| File | CLEAN items touching it | Notes |
|------|------------------------|-------|
| `sim/world_loader.py` | **01** | Delete file |
| `scripts/bench_reid_callback.py` | **02** | Delete file |
| `robot_follow/follow_api/config.py` | **03, 14** | 03 deletes fields; 14 adds `tunable_fields()` classmethod. **Same file, serialise within wave.** |
| `robot_follow/drone_api/mavsdk_drone.py` | **04, 08, 11, 13** | 04 adds help string at :47; 08 removes pipe-reader at :642-665; 11 adds `_reap_mavsdk_server` helper; 13 merges two telemetry tasks. **Serialise — 4 edits on one file.** |
| `robot_follow/robot_follow_app.py` | **05, 11, 12, 15** | 05 fixes `getattr` fallback; 11 calls the new helper; 12 collapses pre-parsers; 15 calls `decide_branches`. **Serialise — 4 edits on one file.** |
| `robot_follow/pipeline_adapter/vision_branches.py` | **06, 15** | 06 deletes alias at :397; 15 adds `decide_branches` helper. Independent edits in same file but easy to combine. |
| `robot_follow/follow_api/state.py` | **07** | Update signature; remove `None` branch |
| `robot_follow/tests/test_shared_state.py` | **07** (test fixup) | Update 11 call sites to pass `available_ids=set()`. Co-commit with state.py change. |
| `robot_follow/servers/web_server.py` | **09, 14, 16** | 09 removes `_NULLABLE_FIELDS`; 14 replaces `_CONFIG_FIELDS` with `ControllerConfig.tunable_fields()`; 16 fixes `SharedUIState` SSE race. **3 items on one file — serialise.** Note: 09 and 14 touch the same block (lines 259-347); 16 touches lines 39-142 (different block). |
| `robot_follow/servers/openhd_bridge.py` | **09, 14, 17** | 09 removes `_NULLABLE_FIELDS`; 14 replaces `_CONFIG_PARAMS`; 17 fixes socket reuse. **3 items on one file — serialise.** All three touch different lines / blocks. |
| `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` | **10, 15, 18** | 10 removes `controller_config=` kwarg; 15 removes redundant implicit-display at :1166-1167; 18 fixes O(n) dedup at :88. Three independent edits; can combine. |

### Inter-wave file conflicts (serialisation needs):

- **`mavsdk_drone.py`**: 04 (doc), 08 (pipe-reader), 11 (helper), 13 (telemetry merge). All in same Wave/group; pick a stable order (e.g. 04 → 08 → 11 → 13). One file, one author, ideally one commit per item.
- **`robot_follow_app.py`**: 05, 11, 12, 15. Same one-author rule.
- **`web_server.py`**: 09 ∪ 14 in Wave 2 (`_CONFIG_FIELDS` + `_NULLABLE_FIELDS` are colocated lines 325-336); 16 in Wave 3 (different block).
- **`openhd_bridge.py`**: 09 + 14 in Wave 2; 17 in Wave 3.

---

## Wave-Grouping Recommendation

### Wave 0 — Test scaffolding (≤ 1 task)

Update `test_shared_state.py` is paired with CLEAN-07 in same commit, so no separate Wave 0 is needed unless the planner wants to land the **two-tab integration test for CLEAN-16** as a scaffold first (failing) and then close it green in Wave 3. Recommendation: write the failing test in Wave 0 to give Wave 3 a target.

| Task | Files | Notes |
|------|-------|-------|
| W0-T1 | `robot_follow/tests/test_web_server_sse.py` (new) | Two-consumer Condition + monotonic-seq test (mocked `SharedUIState`); add `xfail` marker; remove marker in Wave 3 |

### Wave 1 — Dead code (10 items, 1 commit per item OR grouped by file)

Edit-isolated where possible. Files touched by multiple items must serialise.

| Group | CLEAN items | Files |
|-------|-------------|-------|
| W1-G1 | 01 | `sim/world_loader.py` (delete) |
| W1-G2 | 02 | `scripts/bench_reid_callback.py` (delete) |
| W1-G3 | 03 | `robot_follow/follow_api/config.py` (single edit; CLEAN-14 will follow in Wave 2 — order: 03 before 14) |
| W1-G4 | 06 | `robot_follow/pipeline_adapter/vision_branches.py` (one-line delete; CLEAN-15 follows in Wave 2) |
| W1-G5 | 07 + test fixup | `robot_follow/follow_api/state.py` + `robot_follow/tests/test_shared_state.py` |
| W1-G6 | 04 | `robot_follow/drone_api/mavsdk_drone.py:47` (help string only) + `CLAUDE.md` |
| W1-G7 | 05 | `robot_follow/robot_follow_app.py:54` (one-line fix) |
| W1-G8 | 08 | `robot_follow/drone_api/mavsdk_drone.py:642-665` (independent of 04 — different lines) |
| W1-G9 | 09 | `robot_follow/servers/web_server.py` + `robot_follow/servers/openhd_bridge.py` (5 lines + 3 dead branches) |
| W1-G10 | 10 | `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` (single signature change) |

**Wave 1 parallelism:** G1, G2, G5, G7, G9, G10 are file-independent and could be done in parallel. G3 must finish before Wave 2 G14. G4 must finish before Wave 2 G15. G6, G8 both touch `mavsdk_drone.py` but at distinct line ranges (47 vs 642-665) — safe to parallelise if author can manage the conflict.

### Wave 2 — Duplication merges (5 items, file-serialised)

| Group | CLEAN items | Files | Notes |
|-------|-------------|-------|-------|
| W2-G11 | 11 | `robot_follow/drone_api/mavsdk_drone.py` + `robot_follow/robot_follow_app.py` + `robot_follow/drone_api/__init__.py` | Extract helper; update 2 callers |
| W2-G12 | 12 | `robot_follow/robot_follow_app.py` | Collapse 3 → 1 pre-parser |
| W2-G13 | 13 | `robot_follow/drone_api/mavsdk_drone.py` | Merge 2 telemetry tasks |
| W2-G14 | 14 | `robot_follow/follow_api/config.py` + `robot_follow/servers/web_server.py` + `robot_follow/servers/openhd_bridge.py` | `tunable_fields()` classmethod + replace 2 dict literals |
| W2-G15 | 15 | `robot_follow/pipeline_adapter/vision_branches.py` + `robot_follow/robot_follow_app.py` + `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` | `decide_branches()` helper + collapse 3 implicit-display sites |

**Wave 2 serialisation:** G11 and G13 both touch `mavsdk_drone.py`; G11 and G12 and G15 all touch `robot_follow_app.py`. **Run G11 → G12 → G13** for mavsdk_drone+app pair; G14 standalone (3 files, no overlap with the trio); G15 last (touches `robot_follow_app.py` again, plus 2 pipeline_adapter files).

Suggested execution order: **G11 → G13 → G12 → G14 → G15**.

### Wave 3 — Hot-path (3 items, mostly parallelisable)

| Group | CLEAN items | Files | Notes |
|-------|-------------|-------|-------|
| W3-G16 | 16 | `robot_follow/servers/web_server.py` (SharedUIState only — lines 39-142, separate block from G14) | Plus close the xfail in `test_web_server_sse.py` |
| W3-G17 | 17 | `robot_follow/servers/openhd_bridge.py` (socket reuse — different block from G14) | |
| W3-G18 | 18 | `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py:88` | |

**Wave 3 parallelism:** G16, G17, G18 are file-independent — fully parallel.

### Wave summary table

| Wave | Items | Parallel groups | Serial chains |
|------|-------|-----------------|---------------|
| 0 | xfail test scaffold | 1 task | — |
| 1 | CLEAN-01..10 | G1, G2, G5, G7, G9, G10 (independent) | G6+G8 (same file, distinct ranges); G3 before W2 G14; G4 before W2 G15 |
| 2 | CLEAN-11..15 | G14 alone | G11 → G13 → G12 → G15 (shared file chain) |
| 3 | CLEAN-16..18 | All three | — |

---

## CLEAN-16 Deep Dive

CLEAN-16 is the headline of Phase 2 and the only item with non-trivial concurrency reasoning.

### Current code (the bug)

`robot_follow/servers/web_server.py:69-85`

```python
def update_frame(self, jpeg_bytes: bytes):
    """Called from appsink callback with pre-encoded JPEG bytes."""
    with self._lock:
        self._frame_jpeg = jpeg_bytes
        self._frame_snapshot = { ... }
    self._frame_event.set()      # line 84 — wake all waiters
    self._frame_event.clear()    # line 85 — IMMEDIATELY clear flag
```

`wait_frame()` / `wait_frame_with_detections()` block on `self._frame_event.wait(timeout=2.0)`.

### Why it races

Python `threading.Event.set()` + immediate `clear()` is the textbook anti-pattern. Reference: [Python docs note on `Event`](https://docs.python.org/3/library/threading.html#threading.Event) — "An event manages an internal flag." The waiter sees the **edge**, not the **transition** — and the edge has zero duration if `clear()` runs before another consumer enters `wait()`.

Per-consumer timeline (two MJPEG clients):

```
Frame N arrives:
  Producer:         lock --- set() --- clear()     (atomic-ish: maybe 1 µs gap)
                            ^         ^
                            wakes     flag down
                            both
                            waiters
  Consumer A:       wait(2s) ........ returns at set()                 ✓ gets frame
  Consumer B:       (not yet in wait — about to call it)               ✗ misses set+clear
                                                              wait(2s) starts AFTER clear()
                                                              → falls through to 2s timeout
                                                              → blank tab for up to 2s
```

In the steady state with a single consumer, the bug is hidden — the consumer is in `wait()` when `set()` fires every time. With two consumers and Python's GIL switching, B's `wait()` and A's frame-processing race against the producer's `set/clear`.

### Proposed fix: `Condition` + monotonic `frame_seq`

This is the standard pattern for "producer publishes a snapshot, multiple consumers each track their own progress."

```python
import threading
from typing import Optional

class SharedUIState:
    def __init__(self):
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._frame_seq: int = 0          # monotonic counter
        self._frame_jpeg: Optional[bytes] = None
        self._frame_snapshot: Optional[dict] = None
        # ... rest unchanged

    def update_frame(self, jpeg_bytes: bytes):
        """Producer. Each frame increments _frame_seq atomically."""
        with self._cond:
            self._frame_jpeg = jpeg_bytes
            self._frame_snapshot = { ... }
            self._frame_seq += 1
            self._cond.notify_all()       # safe — runs under the lock

    def wait_frame(self, last_seen: int, timeout: float = 1.0):
        """Consumer. Returns (jpeg, seq) when frame_seq > last_seen, or (None, last_seen) on timeout."""
        with self._cond:
            ok = self._cond.wait_for(lambda: self._frame_seq > last_seen, timeout=timeout)
            if not ok:
                return None, last_seen
            return self._frame_jpeg, self._frame_seq

    def wait_frame_with_detections(self, last_seen: int, timeout: float = 1.0):
        with self._cond:
            ok = self._cond.wait_for(lambda: self._frame_seq > last_seen, timeout=timeout)
            if not ok:
                return None, None, last_seen
            return self._frame_jpeg, self._frame_snapshot, self._frame_seq
```

Each consumer loop tracks its own `last_seen`:

```python
def _handle_mjpeg(self):
    self.send_response(200)
    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
    ...
    last_seen = 0
    try:
        while True:
            jpeg, last_seen = self.ui_state.wait_frame(last_seen, timeout=2.0)
            if jpeg is None:
                continue
            # write multipart frame ...
    except (BrokenPipeError, ConnectionResetError):
        pass

def _handle_detections_sse(self):
    ...
    last_seen = 0
    try:
        while True:
            _jpeg, snapshot, last_seen = self.ui_state.wait_frame_with_detections(last_seen, timeout=2.0)
            if snapshot is None:
                continue
            self.wfile.write(f"data: {json.dumps(snapshot)}\n\n".encode())
            self.wfile.flush()
    except (BrokenPipeError, ConnectionResetError):
        pass
```

### Why this is correct under each scenario

1. **Single consumer.** `cond.wait_for(seq > last_seen)` blocks until next frame; producer increments `seq` under lock + `notify_all()`; consumer wakes, returns the JPEG + new `seq`. `last_seen` advances monotonically. **No edges lost.**
2. **Two consumers (the failing case today).** Both consumers track independent `last_seen`. Producer increments and `notify_all()`. Both consumers wake, both call `wait_for` predicate (`self._frame_seq > last_seen`), both find it `True`, both proceed. **No race window** — the lock serialises `wait_for` re-check; consumers who weren't yet in `wait()` will see the higher `_frame_seq` on their next call and return immediately.
3. **Consumer disconnects mid-stream.** `self.wfile.write` raises `BrokenPipeError`; outer `try` catches it; the loop exits and the `wait_for` thread returns. No leaked condition variable (it's owned by `SharedUIState`, shared across all consumers — no per-consumer state to clean up).
4. **Slow consumer (e.g. high-latency JSON write).** While the slow consumer is mid-write, the producer keeps running and bumping `_frame_seq`. When the slow consumer comes back to `wait_for`, the predicate is already `True` (`_frame_seq > last_seen`), so `wait_for` returns immediately without blocking. Slow consumer **catches up** by skipping intermediate frames (it always gets the latest snapshot, never a stale one) — which is the exact desired behaviour for live MJPEG.
5. **Pipeline produces faster than consumers consume.** Same as (4). Each consumer gets only the latest frame on each call. No queue backs up, no memory leak.
6. **No frames yet (`_frame_seq = 0`, `last_seen = 0`).** `wait_for(0 > 0)` is False → block until first frame. ✓ correct.
7. **Pipeline stops (no more frames).** Consumer waits for `timeout=2.0`; `wait_for` returns `False`; we return `None, last_seen`. Consumer loop `continue`s and waits again. **2-second poll cycle.** Acceptable — same as today.

### Concurrency gotchas

- **`notify_all()` MUST be inside the `with self._cond:` block.** Python's docs are explicit: notifications outside the lock are undefined behaviour (race against `wait_for` re-checking the predicate).
- **`Condition` is reentrant-safe but `notify_all` is bounded.** With N consumers, `notify_all` wakes all of them; each then re-acquires the lock and re-checks the predicate. Cost is O(N) per frame. At 10 FPS and N=2..5 consumers, this is negligible (~µs).
- **`wait_for` is correct under spurious wake-ups.** The predicate re-check loop is internal to `wait_for`; we don't need to write the `while not predicate: wait()` loop manually.
- **Lock contention with `update_detections`.** Producer calls `update_detections` (line 64), then later `update_frame` (line 84). Both acquire `self._lock` (and now `self._cond`, same underlying lock). If a consumer is mid-`wait_for`, it releases the lock, so producer can grab it. **No deadlock.**
- **`update_velocity`, `update_perf`, `update_detections`** all take `self._lock`. After this change they need to take `self._cond` (same lock object — `Condition(self._lock)` shares the underlying lock). Confirm by reading the migrated code that **every `with self._lock:` becomes `with self._cond:` OR we keep `self._lock` for the non-condition writers and use `self._cond` only where notification is needed.** The latter (keep `self._lock` for plain-CRUD writes; use `self._cond` for `update_frame` + `wait_*`) is safe because `Condition(self._lock)` uses the same primitive — both names point at the same RLock. *But* you cannot `notify_all` without acquiring `self._cond` (which is identical to `self._lock`). Concrete shape: keep all existing `with self._lock:` blocks unchanged; only `update_frame` and `wait_*` migrate to `with self._cond:` (which acquires the same lock). Notifications work because `notify_all` checks "lock is held" — by anyone — and Python's `Condition` uses the same RLock.

### Two-tab integration test (CLEAN-16 acceptance gate)

```python
# robot_follow/tests/test_web_server_sse.py (new)
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from robot_follow.servers.web_server import SharedUIState


def test_two_consumers_both_receive_frames_within_timeout():
    """The CLEAN-16 acceptance test. Two consumers track frames independently.

    Pre-fix: under heavy producer + two consumers, one consumer falls through
    the 2 s timeout because set()+clear() in update_frame() lose the edge.
    Post-fix (Condition + frame_seq): both consumers get every frame.
    """
    ui = SharedUIState()

    results = {"a": [], "b": []}
    stop = threading.Event()

    def consumer(name):
        last_seen = 0
        while not stop.is_set():
            jpeg, last_seen = ui.wait_frame(last_seen, timeout=0.5)
            if jpeg is not None:
                results[name].append(last_seen)

    pool = ThreadPoolExecutor(max_workers=2)
    pool.submit(consumer, "a")
    pool.submit(consumer, "b")

    # Producer: 50 frames at ~100 Hz (fast enough to expose the race)
    time.sleep(0.05)
    for i in range(50):
        ui.update_frame(f"frame-{i}".encode())
        time.sleep(0.01)

    time.sleep(0.5)  # let consumers drain
    stop.set()
    pool.shutdown(wait=True, cancel_futures=False)

    # Each consumer should have received most frames (allowing some loss
    # for the early window before consumers started waiting).
    assert len(results["a"]) >= 45, f"consumer A only got {len(results['a'])} frames"
    assert len(results["b"]) >= 45, f"consumer B only got {len(results['b'])} frames"
    # Both consumers must have made it past frame 5 (early window) AND past frame 45.
    assert max(results["a"]) >= 45
    assert max(results["b"]) >= 45
```

**Smoke test (manual / browser):** open `http://localhost:5001/` in two tabs simultaneously. Both must show live video without a black-screen window. The 2-tab criterion is ROADMAP success criterion #3.

---

## Validation Architecture

### Test Framework

| Property | Value |
|----------|-------|
| Framework | pytest 9.0.2 |
| Config file | none — discovered via conftest.py at `robot_follow/tests/conftest.py` (adds repo root to `sys.path`) |
| Test directory | `robot_follow/tests/` (14 test files, 159 tests collected) |
| Quick run command | `python -m pytest robot_follow/tests -x --ignore=robot_follow/tests/test_sim_worlds.py` |
| Full suite command | `python -m pytest robot_follow/tests -v` (test_sim_worlds requires Hailo HW + sim; skip on dev box) |
| Smoke gate (Phase 1 contract) | `python -m pytest robot_follow/tests/test_install_smoke.py -v` (10 tests, < 2 s) |
| Dev loop integration | `drone-follow --input usb --webui --yaw-only` (ROADMAP criterion 2) |

**Invocation gotcha (from Phase 1 verification):** Always use `python -m pytest`, never bare `pytest`. Bare `pytest` resolves to `~/.local/bin/pytest` with system-python shebang, which doesn't see the venv's `robot_follow` package.

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| CLEAN-01 | `sim/world_loader.py` is gone | smoke | `test ! -f sim/world_loader.py && grep -r "world_loader" --exclude-dir=.git . | wc -l` returns 0 | manual / one-line shell |
| CLEAN-02 | `scripts/bench_reid_callback.py` is gone | smoke | `test ! -f scripts/bench_reid_callback.py && grep -r "bench_reid" --exclude-dir=.git . | wc -l` returns 0 | manual / one-line shell |
| CLEAN-03 | `vfov` field + flag absent | unit | `pytest robot_follow/tests/test_config_persistence.py -v` (extend to assert no `vfov` attribute on `ControllerConfig`) | ✅ exists, extend |
| CLEAN-04 | `--help` mentions `--mission-duration` semantics | smoke | `drone-follow --help 2>&1 | grep -i "mission-duration.*auto-land\|mission-duration.*timeout"` returns 1+ lines | manual |
| CLEAN-05 | `_resolve_serial_connection` uses `args.serial_baud` directly | unit | grep gate: `grep -n 'getattr(args, "serial_baud"' robot_follow/robot_follow_app.py` returns 0 lines | manual |
| CLEAN-06 | alias `strip_tiles_and_highlight_target` absent | unit | `grep -n 'strip_tiles_and_highlight_target' robot_follow/pipeline_adapter/vision_branches.py` returns 0 lines | manual |
| CLEAN-07 | `state.update` requires `available_ids` | unit | `pytest robot_follow/tests/test_shared_state.py -v` (all 11 sites updated) | ✅ exists, update |
| CLEAN-08 | `run_live_drone` signature has no `shutdown_read_fd` | unit | `python -c "import inspect; from robot_follow.drone_api.mavsdk_drone import run_live_drone; assert 'shutdown_read_fd' not in inspect.signature(run_live_drone).parameters"` | manual |
| CLEAN-09 | `_NULLABLE_FIELDS` is absent | unit | `grep -n '_NULLABLE_FIELDS' robot_follow/servers/*.py` returns 0 lines | manual |
| CLEAN-10 | `create_app` signature has no `controller_config` | unit | `python -c "import inspect; from robot_follow.pipeline_adapter.hailo_drone_detection_manager import create_app; assert 'controller_config' not in inspect.signature(create_app).parameters"` | manual |
| CLEAN-11 | Single `_reap_mavsdk_server` helper | unit | `grep -c 'pkill.*mavsdk_server' robot_follow/**/*.py` returns 1 line (inside the helper only) | manual |
| CLEAN-12 | Single pre-parser | unit | `grep -c 'ArgumentParser(add_help=False)' robot_follow/robot_follow_app.py` returns ≤ 1 | manual |
| CLEAN-13 | Single position-stream subscription | unit | `grep -c 'drone.telemetry.position()' robot_follow/drone_api/mavsdk_drone.py` returns 1 line | manual |
| CLEAN-14 | `ControllerConfig.tunable_fields` exists; web_server + openhd_bridge use it | unit | `python -c "from robot_follow.follow_api.config import ControllerConfig; assert hasattr(ControllerConfig, 'tunable_fields') and len(ControllerConfig.tunable_fields()) > 20"` PLUS `grep -c '_CONFIG_FIELDS\\b\|_CONFIG_PARAMS\\b' robot_follow/servers/*.py` returns 0 (the dicts are gone) | manual + new test |
| CLEAN-15 | One `decide_branches` site | unit | new unit test `test_decide_branches` over 16-combo matrix | new file needed |
| CLEAN-16 | Two consumers get frames | unit + smoke | `pytest robot_follow/tests/test_web_server_sse.py -v` (new) + manual two-tab browser test | new file needed |
| CLEAN-17 | `_send_immediate_report` does not create a new socket per call | unit | `grep -c 'socket.socket' robot_follow/servers/openhd_bridge.py` returns ≤ 2 (listener + ONE shared report socket OR shared with listener); `_send_immediate_report` body grep returns 0 occurrences of `socket.socket(` | manual |
| CLEAN-18 | `_update_ui` has no `next((q for q in persons` linear scan | unit | `grep -c 'next((q for q in persons' robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` returns 0 | manual |

### Sampling Rate

- **Per task commit:** `python -m pytest robot_follow/tests/test_install_smoke.py robot_follow/tests/test_shared_state.py robot_follow/tests/test_config_persistence.py robot_follow/tests/test_velocity_command_shape.py -x` (subset that covers the touched modules, ≈ 30 tests, < 2 s)
- **Per wave merge:** `python -m pytest robot_follow/tests -x --ignore=robot_follow/tests/test_sim_worlds.py` (159 tests minus the Hailo-HW gated ones, < 30 s)
- **Phase gate (manual, before `/gsd:verify-work`):**
  1. Full suite green: `python -m pytest robot_follow/tests -v` (skip test_sim_worlds without Hailo HW)
  2. Integration smoke: `drone-follow --input usb --webui --yaw-only` runs without error
  3. Two-tab browser smoke for CLEAN-16: open `http://localhost:5001` in two tabs simultaneously, both must show live video without a black-screen window within 2 s
  4. (Optional, on Hailo host) Run the full `test_sim_worlds` suite to confirm pipeline still works end-to-end

### Wave 0 Gaps

- [ ] `robot_follow/tests/test_web_server_sse.py` (NEW) — covers CLEAN-16 with two-consumer + monotonic-seq invariants; landed with `@pytest.mark.xfail(reason="CLEAN-16 race not yet fixed")` in Wave 0, marker removed in Wave 3 G16
- [ ] `robot_follow/tests/test_vision_branches.py` (NEW) — covers CLEAN-15's `decide_branches()` over the 16-combo flag matrix. Landed in Wave 0 with xfail; closes in Wave 2 G15. *Optional* — could also be appended to an existing test file.
- [ ] Conftest update — none needed; existing path setup already covers new files

---

## Open Questions for Planner

1. **CLEAN-14 — three-key diff between `_CONFIG_FIELDS` and `_CONFIG_PARAMS`.** `web_server._CONFIG_FIELDS` has 27 entries; `openhd_bridge._CONFIG_PARAMS` has 24. The deltas (most likely `top_margin_safety`, `bottom_margin_safety`, and potentially others) need a user decision: do they get MAVLink IDs (exposed to QOpenHD) or are they web-UI-only? Recommendation: expose all to OpenHD with new `DF_TOP_MAR` / `DF_BOT_MAR` MAVLink IDs (need short names ≤ 16 chars). Planner should diff the two dicts byte-by-byte and propose the merged schema. **OpenHD side may also need a C++ patch** — check `OpenHD/HAILO_INTEGRATION.md` before adding new MAVLink IDs (out of scope if it requires OpenHD changes; in scope if Python-side schema is enough).

2. **CLEAN-04 — does anyone actually use `--mission-duration` at flight-time?** No documentation in README/CLAUDE.md, only in `docs/control-architecture.md` (low-traffic doc). Operators may not know this exists. **Recommendation: document in CLAUDE.md "Key CLI Flags" section** AND add the help string. If the operator confirms they never set it, an alternative path is to remove `--mission-duration` and just hardcode a sentinel-large value (or `None` → use `asyncio.wait` with only `shutdown.wait()`). Plan should pick one.

3. **CLEAN-13 — Shape A vs Shape B (merge tasks vs consolidate caches).** Shape A keeps both `altitude_cache` and `telemetry_cache` dicts and writes both from one task. Shape B drops `altitude_cache` and has `live_control_loop` read `telemetry_cache.get("rel_alt")` instead. Shape B is cleaner long-term but bigger blast radius (changes `live_control_loop`'s signature). Recommendation: Shape A in Phase 2; Shape B is a "consolidate caches" item that fits better in Phase 3 (which already moves altitude fields to `Optional`).

4. **CLEAN-15 — `decide_branches` location: `vision_branches` or new module?** Recommendation: `vision_branches.py` (already owns `assemble_output_stage`). A new `branch_policy.py` module is over-engineered for a 10-line dataclass + helper.

5. **CLEAN-17 — Option A (reuse listener `self._sock`) vs Option B (dedicated `self._report_sock`).** Recommendation: A. Option B is one extra socket FD for marginal isolation; A is simpler. Planner can defer this decision to implementation time.

6. **CLEAN-03 — does the JSON-config loader tolerate unknown keys?** If `df_config.example.json` and `sim/configs/*.json` keep their `"vfov": 41.0` entry, will `ControllerConfig.from_args` / `load_from_file` fail or silently drop the key? Planner must verify in `config.py` before deleting the field. If strict, also strip the JSON entries.

---

## Out-of-Scope Confirmations (DO NOT TOUCH)

- **Phase 3 abstraction work.** No `Robot` protocol, no `Capabilities`, no `RobotCommand`, no `--robot` flag, no `MavsdkDroneAdapter` in Phase 2. Resist the temptation to introduce these while merging `_CONFIG_FIELDS` / `_CONFIG_PARAMS` (CLEAN-14) or while touching `mavsdk_drone.py` (CLEAN-04, 08, 11, 13). Phase 3 will move these — Phase 2 leaves the structure as-is.
- **Renaming `run_drone()` → `run_robot()`.** That's ABS-08.
- **Moving `drone_api/mavsdk_drone.py` → `robot_api/adapters/mavsdk_drone.py`.** That's ABS-03.
- **Making altitude fields `Optional[float]`.** That's ABS-07.
- **The 0.3 s `time.sleep` in `DetachedMavsdkServer.__enter__` (line 230 currently).** Not on the dead-code list; load-bearing for the reap-then-respawn race. Leave it.
- **The `time.sleep(1.0)` in `_start_recording_delayed` at `robot_follow_app.py:421`.** Not on the dead-code list. Leave it.
- **Phase 1 retroactive concerns.** Deferred verifications B (boot-service on RPi) and C (full pytest on Hailo host) from Phase 1 verification are NOT Phase 2 items. They become regression smoke at the next field-deployment sync.
- **New tests beyond what each CLEAN item minimally requires.** Add `test_web_server_sse.py` (CLEAN-16 acceptance) and optionally extend `test_config_persistence.py` (CLEAN-03 / CLEAN-14). That's it. No "while we're here" test additions.

---

## Sources

### Primary (HIGH confidence — direct repo inspection)

- `robot_follow/follow_api/config.py` (lines 1-60, 168-180, 270-285) — `vfov`, `hfov`, dataclass shape
- `robot_follow/follow_api/state.py` (lines 1-50) — `SharedDetectionState.update` signature + callers
- `robot_follow/drone_api/mavsdk_drone.py` (lines 41-50, 215-240, 345-410, 642-665, 755-810) — CLI flags, pkill helper, telemetry tasks, run_live_drone
- `robot_follow/robot_follow_app.py` (lines 40-70, 200-460) — pre-parsers, branch logic, single caller of `run_live_drone`
- `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` (lines 55-100, 690-745, 1155-1175, 1280-1290) — `_update_ui` dedup, `create_app` signature, branch-build logic
- `robot_follow/pipeline_adapter/vision_branches.py` (lines 265-310, 390-405) — `assemble_output_stage`, `strip_tiles_and_highlight_target` alias
- `robot_follow/servers/web_server.py` (lines 1-145, 200-350) — `SharedUIState`, MJPEG/SSE handlers, `_CONFIG_FIELDS`, `_NULLABLE_FIELDS`
- `robot_follow/servers/openhd_bridge.py` (lines 1-200, 305-365, 420-430) — `_CONFIG_PARAMS`, `_send_immediate_report`, listener/reporter sockets
- `robot_follow/tests/` directory listing + `test_shared_state.py` + `test_install_smoke.py` (current shape and assertion patterns)
- `.planning/STATE.md` — Phase 1 decisions (post-rename baseline, parser.prog pin)
- `.planning/phases/01-rename/01-VERIFICATION.md` — Phase 1 verified state
- `.planning/REQUIREMENTS.md` — CLEAN-01..18 spec
- `.planning/ROADMAP.md` — Phase 2 success criteria
- `.planning/presentations/v1_1_design_review.md` (lines 470-490) — CLEAN-16 design-review framing

### Secondary (HIGH confidence — supporting docs)

- `CLAUDE.md` — Documented CLI flags; basis for CLEAN-04 documentation gap analysis
- `docs/control-architecture.md` (line 250) — Only existing mention of `--mission-duration`

### Tertiary

- Python `threading.Condition` / `Event` semantics (verified against Python 3.11 stdlib behaviour; consistent with Python 3.10+ used by this repo's `requires-python = ">=3.10"`)

## Metadata

**Confidence breakdown:**
- Per-item analysis: HIGH — every line reference verified on the current tree
- File-conflict matrix: HIGH — all touch points are deterministic from the per-item analysis
- Wave grouping: MEDIUM — depends on planner's preference for commit granularity (1-commit-per-CLEAN vs grouped commits)
- CLEAN-16 deep dive: HIGH — concurrency reasoning is standard Python `threading` idiom; standard fix for a standard antipattern
- Validation architecture: HIGH — test framework discovered (pytest 9.0.2), 159 tests collected on dev box, the integration smoke `drone-follow --input usb --webui` is documented in CLAUDE.md and verified by Phase 1

**Research date:** 2026-05-14
**Valid until:** 2026-06-13 (30 days — codebase is stable, no upstream churn expected before Phase 2 lands)
