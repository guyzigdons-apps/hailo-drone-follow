# Phase 1: Rename - Research

**Researched:** 2026-05-14
**Domain:** Mechanical Python package rename (`drone_follow` → `robot_follow`) + console-script alias preservation
**Confidence:** HIGH (inventory verified against `git grep` on `feature/rover-support`)

## Summary

Phase 1 is a mechanical, fully-scoped rename. Inventory confirms CONTEXT.md's claims exactly:
**48 `from drone_follow` / `import drone_follow` statements** across 19 Python files (matches CONTEXT.md's "~48"). The non-Python surface is short and well-bounded: 1 `pyproject.toml`, 4 shell scripts with **path strings** that need rewriting (`install.sh`, `scripts/install_air.sh` × 2 chown lines, plus comments in many others), 0 `pytest.ini`, 0 `.pre-commit-config.yaml`, 0 `.github/workflows/`. Documentation surface is 23 `.md` files with `drone-follow`/`drone_follow` references; the bulk (`.planning/`) is whitelisted.

The boot service (`drone-follow-boot.service`, `drone-follow-boot.sh`, `~/Desktop/drone-follow.conf`) is **path-stable** (RENAME-04) — these files are NOT renamed; only the underlying console script alias keeps them working. Egg-info is gitignored — no manual cleanup needed; `pip install -e .` regenerates it. The console-script shim at `hailo-apps/venv_hailo_apps/bin/drone-follow` literally embeds `from drone_follow.drone_follow_app import main` and **must be regenerated** by uninstall + reinstall.

**Primary recommendation:** Execute as a single atomic commit:
1. `git mv drone_follow robot_follow` and `git mv robot_follow/drone_follow_app.py robot_follow/robot_follow_app.py`
2. `sed -i 's/\bdrone_follow\b/robot_follow/g; s/\bdrone_follow_app\b/robot_follow_app/g'` on all 19 .py files
3. Update `pyproject.toml` (name, packages.include, [project.scripts] — add both `robot-follow` and `drone-follow` mapping to `robot_follow.robot_follow_app:main`)
4. Update `install.sh` (insert `pip uninstall drone-follow -y` before `pip install -e .`; rewrite UI path)
5. Rewrite docs (CLI examples use `robot-follow`; add one-line alias note in README + CLAUDE.md)
6. Clean `__pycache__` and stale `*.pyc` (gitignored but local) — `find . -name __pycache__ -prune -exec rm -rf {} +` to guarantee no stale bytecode masks the rename
7. Re-run `pip install -e .` so the console-script shim regenerates with the new import path
8. Grep-gate, lint, `python -c 'import robot_follow'`, both `--help` outputs identical
9. Pre-push: full pytest (Hailo-capable host)

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

**Commit shape & ordering**
- **Single atomic commit** landing dir rename + 48 import rewrites + pyproject + shell scripts + docs. Every commit on the branch stays buildable; `git bisect` survives.
- **`git mv drone_follow robot_follow`** — git records the move, `git log --follow` survives.
- **`drone_follow_app.py` is also renamed** → `robot_follow_app.py`. Console scripts target `robot_follow.robot_follow_app:main`. The package and its main module share the family name; obscure `python -m drone_follow.drone_follow_app` invocations are an acceptable break (the console scripts are the supported surface).
- **Verification gate (two-tier):**
  - Pre-commit (fast, no Hailo HW needed): lint passes, `python -c 'import robot_follow'` works, `drone-follow --help` and `robot-follow --help` both run and produce identical output.
  - Pre-push (Hailo-capable host): full `pytest` suite under `robot_follow/tests/`.

**pyproject name + alias surface**
- **`[project] name = "robot-follow"`** — single PyPI-style distribution. Matches the success criterion verbatim: `pip show robot-follow` works, `pip show drone-follow` returns nothing.
- **Both console scripts** declared in `[project.scripts]`, identical target:
  ```toml
  robot-follow = "robot_follow.robot_follow_app:main"
  drone-follow = "robot_follow.robot_follow_app:main"
  ```
  Boot service keeps invoking `drone-follow`; identical `--help` output is guaranteed by construction.
- **`drone-follow` is a permanent alias** — no deprecation banner on stderr. It's the primary entry on every deployed unit and in the systemd boot unit; treating it as "going away" would be a lie.
- **`install.sh` does `pip uninstall drone-follow -y` before `pip install -e .`** so already-deployed units don't keep stale `pip show drone-follow` metadata from the old distribution. Idempotent (no-op on fresh installs). Runs as the invoking user, not root.
- **`reid_analysis` sibling package** stays included in `pyproject.toml` packages — out of scope to relocate.

**UI directory location** — Keep nested. `drone_follow/ui/` becomes `robot_follow/ui/` via `git mv`. No Vite/npm config churn.

**Recordings directory** — Keep nested. `drone_follow/recordings/` becomes `robot_follow/recordings/`.

**Docs & examples normalization**
- Rewrite every `drone-follow --...` example to `robot-follow --...` across `README.md`, `CLAUDE.md`, `TROUBLESHOOTING.md`, `docs/*.md`, `PARAMETERS.md`, `RESOLUTION_CONTROL.md`, `SETUP_GUIDE.md`, `TEST_PLAN.md`, and the `system/` and `scripts/` shell-script comments.
- Mention the `drone-follow` alias once near the top of `README.md` and once in `CLAUDE.md`.
- `.claude/memory/*.md` — update text references (commands, examples, file paths inside the package). Don't rename the memory files themselves.

**Top-level helper file names** — Internal contents updated; filenames preserved. `run_drone.sh`, `df_params.json`, `scripts/start_air.sh`, `scripts/install_air.sh`, etc. keep their names. `drone-follow-boot.service` and `~/Desktop/drone-follow.conf` preserved (RENAME-04).

**Verification stragglers** — Pre-commit `git grep` gate: `git grep -nE 'drone_follow|from drone_follow|import drone_follow'` must return nothing except whitelist (alias mention in README + CLAUDE.md + planning artefacts).

### Claude's Discretion
- Exact wording of the README/CLAUDE.md alias note.
- Whether to add a one-line `[deprecated]` placeholder in `pyproject.toml` keywords/metadata — likely no, but planner can decide.
- Test-path config (`pyproject.toml` `[tool.pytest.ini_options]` if present, or `pytest.ini`) — Claude updates whatever's wired.
- Whether `install.sh` reports the uninstall step to stdout or stays silent (cosmetic).
- Ordering of file edits within the single commit (mechanical, doesn't affect outcome).
- Pre-commit hook behavior in this repo — Claude reads `.pre-commit-config.yaml` (if any) and adapts. *(Research confirms: NO `.pre-commit-config.yaml` exists; nothing to adapt to.)*

### Deferred Ideas (OUT OF SCOPE)
- **Hoist `ui/` and `recordings/` to repo-root** — defer until a second consumer.
- **Rename `drone_follow_app.py` callers' `python -m` muscle memory in user notes** — out of scope; console scripts are the supported surface.
- **Rename `run_drone.sh`, `df_params.json`** → covered in Phase 3 alongside `run_drone()` → `run_robot()`.
- **Rename `.claude/skills/drone-follow-dev/` skill** → out of scope (requires coordinating with user's local Claude config and trigger phrases).
- **Bump pyproject `version` to v1.1-dev** — likely yes, but a packaging-discipline decision for the planner.
- **Sticky `pip cache purge`-style cleanup** for users hitting cached old-name wheels — only if testing reveals an actual problem.
</user_constraints>

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| **RENAME-01** | Package dir `drone_follow/` renamed to `robot_follow/`; all internal imports updated to `from robot_follow.*` | Inventory: exactly 48 `from drone_follow` / `import drone_follow` lines across 19 .py files (table below). `git mv` preserves history. |
| **RENAME-02** | `pyproject.toml` package name is `robot-follow`; console scripts: `robot-follow` (primary) and `drone-follow` (alias) | Current `pyproject.toml` is 28 lines (full diff below). `[tool.setuptools.packages.find]` already supports `robot_follow*` shape change. |
| **RENAME-03** | `setup_env.sh`, `install.sh`, `scripts/start_air.sh` updated; existing functionality preserved | `setup_env.sh` has zero `drone_follow` refs — no changes needed. `install.sh` needs 1 path rewrite (line 120) + new pip-uninstall line. `scripts/start_air.sh` has only comments + 2 invocation lines that work unchanged via the alias. |
| **RENAME-04** | `drone-follow-boot.service` systemd unit + `~/Desktop/drone-follow.conf` config name preserved | Confirmed: 0 changes to `scripts/boot/drone-follow-boot.{service,sh}` or `scripts/boot/install.sh`. They invoke `start_air.sh`, which invokes `drone-follow` (alias). |
| **RENAME-05** | `README.md`, `CLAUDE.md`, `TROUBLESHOOTING.md`, `docs/*.md`, `.claude/memory/*.md` updated | Full file list + per-file ref count in Docs Change Set below. 23 .md files outside `.planning/` (whitelist). |
</phase_requirements>

## Inventory

### Python imports (exact: 48 `from/import drone_follow` lines across 19 files)

| File | `from/import drone_follow` lines | Total `drone_follow` mentions |
|------|----------------------------------|-------------------------------|
| `drone_follow/drone_api/mavsdk_drone.py` | 3 | 4 (one is logger name `"drone_follow.telemetry"`) |
| `drone_follow/drone_follow_app.py` | 11 | 14 (3 are doc/log strings) |
| `drone_follow/pipeline_adapter/hailo_drone_detection_manager.py` | 3 | 3 |
| `drone_follow/pipeline_defaults.py` | 0 | 1 (docstring `:class:` ref) |
| `drone_follow/servers/follow_server.py` | 2 | 2 |
| `drone_follow/servers/openhd_bridge.py` | 2 | 2 |
| `drone_follow/servers/web_server.py` | 2 | 2 |
| `drone_follow/tests/conftest.py` | 0 | 2 (comments) |
| `drone_follow/tests/test_config_persistence.py` | 5 | 5 |
| `drone_follow/tests/test_controller.py` | 4 | 4 |
| `drone_follow/tests/test_filtered_bbox.py` | 1 | 1 |
| `drone_follow/tests/test_follow_server.py` | 2 | 2 |
| `drone_follow/tests/test_follow_target_state.py` | 1 | 1 |
| `drone_follow/tests/test_install_smoke.py` | 0 | 8 (string literals + comments — ALL need rewrite; this file IS the smoke test for the rename) |
| `drone_follow/tests/test_reid_drift_protection.py` | 1 | 1 |
| `drone_follow/tests/test_reid_gallery_coherence_gate.py` | 1 | 1 |
| `drone_follow/tests/test_shared_state.py` | 1 | 1 |
| `drone_follow/tests/test_sim_worlds.py` | 1 | 4 (one is `RECORDINGS_DIR = REPO_ROOT / "drone_follow" / "recordings"` — STRING LITERAL, must rewrite) |
| `drone_follow/tests/test_tracker_protocol.py` | 3 | 3 |
| `drone_follow/tests/test_velocity_api_and_smoother.py` | 2 | 2 |
| `drone_follow/tests/test_velocity_command_shape.py` | 1 | 1 |
| `reid_analysis/reid_analysis_app.py` | 0 | 1 (comment) |
| `scripts/bench_reid_callback.py` | 2 | 2 |
| **TOTAL** | **48** | **66** |

**Note:** `scripts/bench_reid_callback.py` is on the CLEAN-02 dead-code list (Phase 2) — its 2 imports reference `reid_worker` which doesn't exist. Planner choice: rewrite to `robot_follow.*` (mechanical) or accept the unused-import will still be wrong (it's already wrong). Safest: rewrite, document Phase 2 will delete the whole file.

### Non-Python files referencing `drone_follow` (the package path)

| File | Lines | Type |
|------|-------|------|
| `pyproject.toml` | 23, 27 | `packages.include = ["drone_follow*", ...]`, `drone-follow = "drone_follow.drone_follow_app:main"` |
| `install.sh` | 120 | `pushd "$SCRIPT_DIR/drone_follow/ui"` |
| `scripts/install_air.sh` | 322, 323 | `chown_back "${APP_ROOT}/drone_follow/ui/node_modules"`, `.../ui/build` |
| `configs/overlay_style.yaml` | 4 | Comment: `\`drone_follow/pipeline_adapter/vision_branches.py:local_branch\`` |
| `df_params.json` | 409 | JSON `description` field references `drone_follow/recordings/*.mp4` path |
| `.gitignore` | 14, 15 | `drone_follow/ui/node_modules/`, `drone_follow/ui/build/` |

**Total non-py path edits: 8 lines across 6 files.** All are mechanical sed candidates.

### Files referencing `drone-follow` (the CLI/console-script name)

These work unchanged via the alias — **no semantic change required**. Only doc/comment text may be normalized to `robot-follow` per RENAME-05.

| File | `drone-follow` refs |
|------|---------------------|
| `pyproject.toml` | 1 (script declaration — gets a sibling entry) |
| `install.sh` | 2 (banner text only) |
| `scripts/start_air.sh` | ~10 (comments + 2 invocation lines: `drone-follow "${MODE_ARGS[@]}"` works via alias) |
| `scripts/install_air.sh` | ~15 (comments + 1 existence check on `bin/drone-follow` which works via alias) |
| `scripts/install_ground_station.sh` | 3 (comments) |
| `scripts/uninstall_air.sh` | ~10 (comments + boot-unit refs that STAY) |
| `scripts/uninstall_ground_station.sh` | 2 (comments) |
| `scripts/boot/drone-follow-boot.{service,sh}` | preserved verbatim (RENAME-04) |
| `scripts/boot/install.sh`, `uninstall.sh` | preserved verbatim (RENAME-04) |
| `run_drone.sh` | 1 (invocation line: `drone-follow --input rpi ...` — works via alias; **filename stays** per CONTEXT) |
| `sim/setup_sim.sh` | 3 (echoed help text) |
| `sim/start_sim.sh` | 2 (echoed help text) |
| `sim/bridge/video_bridge.py` | 1 (docstring) |
| `sim/mavlink_relay.py` | 2 (docstring + `--help` text) |
| `system/drone-network-mode.sh` | `DRONE_SERVICE="drone-follow.service"` — this is a **systemd user service name** (not the boot one); see Open Questions |
| `system/install.sh` | `systemctl --user disable drone-follow.service` — same |
| `system/README.md` | 1 mention |
| `mafat/run_bench.py` | 1 (comment) |
| `mafat/tiling_record.py` | 1 (comment) |
| `df_params.json` | 3 (description text) |

### Console-script shim (will be regenerated by pip)

`hailo-apps/venv_hailo_apps/bin/drone-follow`:
```python
#!/home/guyz/code/guyz/hailo-drone-follow/hailo-apps/venv_hailo_apps/bin/python3
import sys
from drone_follow.drone_follow_app import main
if __name__ == '__main__':
    sys.argv[0] = sys.argv[0].removesuffix('.exe')
    sys.exit(main())
```

After `pip uninstall drone-follow -y && pip install -e .`, this file is removed and recreated with `from robot_follow.robot_follow_app import main`, and a sibling `bin/robot-follow` is created. **Both files exist post-install.**

### egg-info (gitignored, regenerated automatically)

`drone_follow.egg-info/` — `top_level.txt` says `drone_follow`, `entry_points.txt` says `drone-follow = drone_follow.drone_follow_app:main`. Gitignored via `*.egg-info/` (`.gitignore:3`). Regenerated cleanly by `pip install -e .` after `pip uninstall drone-follow -y`. The egg-info directory **name** changes to `robot_follow.egg-info/` on next install.

### Stale `__pycache__` directories (gitignored)

Found 6 `__pycache__` dirs inside `drone_follow/` containing `*.cpython-310.pyc` files. These are bytecode caches that survive `git mv` and can mask the rename (see Pitfalls). **Must be deleted** before the verification grep gate.

## pyproject.toml Change Set

**Current (verbatim):**
```toml
[project]
name = "drone-follow"
version = "0.1.0"
description = "Autonomous drone-follow using Hailo AI tiling detection"
requires-python = ">=3.10"
dependencies = [
    "mavsdk",
    "matplotlib",
    "numpy",
    "scipy>=1.11",
]

# hailo-apps is installed by the submodule's installer (./hailo-apps/install.sh)
# into ./hailo-apps/venv_hailo_apps. It ships system bits (postprocess .so files,
# /usr/local/hailo/resources) that pip alone can't lay down, so it is NOT a
# pyproject dependency. ./install.sh runs the parent installer for you.

[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[tool.setuptools.packages.find]
include = ["drone_follow*", "reid_analysis*"]
exclude = ["configs", "configs.*"]

[project.scripts]
drone-follow = "drone_follow.drone_follow_app:main"
```

**Target:**
```toml
[project]
name = "robot-follow"
version = "0.1.0"          # planner discretion: bump to "1.1.0.dev0" — deferred
description = "Autonomous robot-follow using Hailo AI tiling detection (drone + rover)"
requires-python = ">=3.10"
dependencies = [
    "mavsdk",
    "matplotlib",
    "numpy",
    "scipy>=1.11",
]

# (comment block unchanged)

[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[tool.setuptools.packages.find]
include = ["robot_follow*", "reid_analysis*"]
exclude = ["configs", "configs.*"]

[project.scripts]
robot-follow = "robot_follow.robot_follow_app:main"
drone-follow = "robot_follow.robot_follow_app:main"
```

**Diff summary:**
- L2 `name`: `drone-follow` → `robot-follow`
- L4 `description`: rewrite (planner discretion exact wording)
- L23 `include`: `drone_follow*` → `robot_follow*`
- L27 entry: rewrite; add L28 alias

**No new sections needed.** No `[tool.pytest.ini_options]` currently exists. No `[tool.setuptools.package-data]` ever needed UI assets — they're served at runtime from `build/`, not packaged.

## install.sh & setup_env.sh Change Set

### `setup_env.sh` (37 lines)
- **Zero `drone_follow` references** — operates purely on relative paths and `hailo-apps/`.
- **No changes required.**

### `install.sh` (137 lines)
- **L120:** `pushd "$SCRIPT_DIR/drone_follow/ui" >/dev/null` → `pushd "$SCRIPT_DIR/robot_follow/ui" >/dev/null`
- **NEW step (insert between L86 (`python -c "import hailo_apps"...`) and L88 (`==> [3/5] pip install -e $SCRIPT_DIR`)):**

  ```bash
  # Idempotent: remove any prior installation of the old distribution name so
  # pip metadata + the old `drone-follow` console-script shim don't linger.
  # No-op on fresh installs.
  echo "==> Removing any prior 'drone-follow' distribution (legacy name)"
  pip uninstall drone-follow -y >/dev/null 2>&1 || true
  ```

  Runs as the invoking user (we're inside `source "$VENV/bin/activate"` from L80). `|| true` so the no-op fresh-install case doesn't trip `set -euo pipefail`. Discretion: keep stdout chatter or `>/dev/null 2>&1` it.

- **Optional doc text:** L8 ("the drone-follow Python package editable into that venv") and L134 ("drone-follow install done.") may be rewritten to "robot-follow"; both lines are comment/banner only.

## Shell Scripts Change Set

| File | Lines to edit | Reason | Severity |
|------|---------------|--------|----------|
| `scripts/start_air.sh` | 0 functional changes; 2-10 comment/echo lines (discretion) | All `drone-follow` invocations resolve via alias. L91, L97 stay. | TEXT-ONLY |
| `scripts/install_air.sh` | L322, L323 (paths) + comments (discretion) | `${APP_ROOT}/drone_follow/ui/node_modules` → `robot_follow/...`; L312-L313 + L496 check `bin/drone-follow` existence — keep verbatim (alias works) | 2 hard edits |
| `scripts/install_ground_station.sh` | 0 functional changes; 3 comments (discretion) | Pure comments | TEXT-ONLY |
| `scripts/start_ground.sh` | **0** | No `drone-follow` references | NONE |
| `scripts/uninstall_air.sh` | 0 functional changes; ~10 comments (discretion) | L63-72 reference `drone-follow-boot.service` etc — PRESERVED (RENAME-04). Comment text only. | TEXT-ONLY |
| `scripts/uninstall_ground_station.sh` | 0 functional changes; 2 comments (discretion) | Pure comments | TEXT-ONLY |
| `scripts/boot/drone-follow-boot.sh` | **0** (RENAME-04) | Filename + content preserved | NONE |
| `scripts/boot/drone-follow-boot.service` | **0** (RENAME-04) | Filename + content preserved | NONE |
| `scripts/boot/install.sh` | **0** (RENAME-04) | Installs the unchanged boot service | NONE |
| `scripts/boot/uninstall.sh` | **0** (RENAME-04) | Removes the unchanged boot service | NONE |
| `scripts/bench_reid_callback.py` | 2 imports (Phase 2 deletes whole file) | See Open Questions | 2 edits OR skip |
| `run_drone.sh` | 0 functional; CONTEXT confirms filename + `drone-follow` invocation stay | Alias-resolved | NONE |
| `sim/setup_sim.sh` | 3 echo lines (discretion) | Pure comments / help text | TEXT-ONLY |
| `sim/start_sim.sh` | 2 echo lines (discretion) | Pure comments / help text | TEXT-ONLY |
| `sim/bridge/video_bridge.py` | 1 docstring (discretion) | Pure comment | TEXT-ONLY |
| `sim/mavlink_relay.py` | 2 docstrings + help text (discretion) | Pure comment | TEXT-ONLY |
| `system/drone-network-mode.sh` | `DRONE_SERVICE="drone-follow.service"` | Systemd USER service name (not boot) | See Open Questions |
| `system/install.sh` | `systemctl --user disable drone-follow.service` | Same | See Open Questions |
| `system/README.md` | 1 mention (discretion) | Pure prose | TEXT-ONLY |

**Hard functional edits (count): 4 lines total — install.sh L120, install_air.sh L322 + L323, plus the new `pip uninstall` block.**

## Docs Change Set

### Tier 1: User-facing project docs (REWRITE all `drone-follow` CLI examples → `robot-follow`; add alias note)

| File | `drone[-_]follow` count | Notes |
|------|------------------------|-------|
| `README.md` | 31 | Add alias one-liner near top. L73 `pytest drone_follow/tests/` → `robot_follow/tests/`. L393 dir-tree diagram. |
| `CLAUDE.md` | 40 | Add alias one-liner. Multiple `drone_follow/*` path refs in architecture section (L23-27, L92). L48 `drone_follow/recordings/...`. |
| `TROUBLESHOOTING.md` | 32 | L317 `pytest community/apps/hailo_drone_follow/drone_follow/tests/` → `robot_follow/tests/`. Many CLI invocation examples. |
| `PARAMETERS.md` | 16 | L204 `from drone_follow.follow_api.config import ControllerConfig` (literal Python invocation example). |
| `SETUP_GUIDE.md` | 67 | Largest doc surface. L31 architecture pointer; L526 `~/hailo-drone-follow/drone_follow/recordings/...`. |
| `RESOLUTION_CONTROL.md` | 14 | L81, L206 `drone_follow/pipeline_adapter/...` path refs. |
| `TEST_PLAN.md` | 1 | Trivial. |

### Tier 2: Implementation docs (REWRITE path refs; CLI examples to `robot-follow`)

| File | Count | Notes |
|------|-------|-------|
| `docs/tracking-reid-algorithm.md` | 4 | Path refs `drone_follow/pipeline_adapter/...`. |
| `docs/calibration-flight-guide.md` | 3 | Mixed. |
| `docs/control-architecture.md` | 1 | Trivial. |
| `docs/design-review.md` | 24 | Has `drone_follow_app.py` refs that ALSO become `robot_follow_app.py`. |
| `docs/superpowers/plans/2026-03-29-x86-ground-station-setup.md` | 1 | Historical plan doc; planner discretion. |
| `docs/superpowers/plans/2026-04-16-repo-review-bugfixes.md` | 33 | Historical; planner discretion (text-only). |
| `docs/superpowers/plans/2026-04-29-post-merge-stabilization.md` | 179 | Historical; planner discretion — likely **leave as historical record**. |

**Recommendation:** Rewrite Tier 1 + `docs/tracking-reid-algorithm.md`, `docs/calibration-flight-guide.md`, `docs/control-architecture.md`, `docs/design-review.md`. **Leave `docs/superpowers/plans/*` as historical** — they're dated plan records and rewriting them lies about what was planned at the time.

### Tier 3: `.claude/` (memory + skills)

| File | Count | Notes |
|------|-------|-------|
| `.claude/memory/MEMORY.md` | 1 | "Drone-Follow Memory Index" — keep as title (sub-project name); the path ref to `../skills/drone-follow-dev/` stays (skill dir explicitly not renamed). |
| `.claude/memory/openhd_pairing.md` | 2 | `<repo>/community/apps/hailo_drone_follow/...` path refs — these are historical path examples, planner discretion. |
| `.claude/memory/tracking_callback_risks.md` | 4 | Path refs `drone_follow/pipeline_adapter/*` (L12-14) — REWRITE to `robot_follow/pipeline_adapter/*`. L56 has a historical path string used in the diff example — keep verbatim. |
| `.claude/memory/webui_build.md` | 6 | `drone_follow/ui/...` path refs throughout — REWRITE to `robot_follow/ui/...`. |
| `.claude/skills/drone-follow-dev/SKILL.md` | 13 | CONTEXT says skill dir + trigger phrases NOT renamed. But `cd drone_follow/ui && ...` (L76) is a path string — REWRITE that one line. Other refs are the dir name itself (trigger phrase context) — keep. |
| `.claude/skills/safe-pull-and-rollback/SKILL.md` | 2 | Both are historical/project-name mentions, no path strings — keep verbatim. |

### Tier 4: Planning artefacts (WHITELIST — references to both old & new are EXPECTED)

These files document the rename itself. Their `drone_follow` references are baseline/historical context, not source-of-truth code.

| File | Count | Reason whitelisted |
|------|-------|-------------------|
| `.planning/MILESTONES.md` | 3 | v1.0 historical record |
| `.planning/PROJECT.md` | 8 | Describes the rename in progress |
| `.planning/REQUIREMENTS.md` | 12 | RENAME-XX specs reference both names |
| `.planning/ROADMAP.md` | 13 | Phase 1 spec |
| `.planning/phases/01-rename/01-CONTEXT.md` | 32 | The phase context itself |
| `.planning/presentations/v1_1_overview.md` | 26 | Design deck describing rename |
| `.planning/research/SUMMARY.md` | 7 | Cross-phase research |
| `.planning/research/ARCHITECTURE.md` | 10 | Cross-phase research |
| `.planning/research/PITFALLS.md` | 8 | Cross-phase research (references `drone_follow_app.py:402`) |
| `.planning/research/FEATURES.md` | 4 | Cross-phase research |
| `.planning/research/STACK.md` | 4 | Cross-phase research |

**Grep-gate whitelist regex:**
```bash
git grep -nE 'drone_follow|from drone_follow|import drone_follow' \
  | grep -vE '^\.planning/' \
  | grep -vE '^README\.md.*alias' \
  | grep -vE '^CLAUDE\.md.*alias'
```
Must be empty post-commit.

## Tests Change Set

### Test infrastructure
- **Discovery:** `drone_follow/tests/` with `drone_follow/tests/conftest.py` adding repo root to `sys.path`.
- **Framework:** pytest 9.0.2 (`/home/guyz/.local/bin/pytest` — system pytest, not in venv). The smoke test `test_install_smoke.py` documents this exactly.
- **No `pytest.ini`, no `pyproject.toml [tool.pytest.ini_options]`** — pure default discovery.
- **Test files: 14 in `drone_follow/tests/`** (+ `conftest.py` + `_reid_gate.py` helper).

### Test files needing import rewrites (all 14 files — handled by the same sed)

| Test file | `from drone_follow` imports | String literals |
|-----------|----------------------------|-----------------|
| `conftest.py` | 0 | "drone_follow package" comment (L1, L6) |
| `test_config_persistence.py` | 5 | 0 |
| `test_controller.py` | 4 | 0 |
| `test_filtered_bbox.py` | 1 | 0 |
| `test_follow_server.py` | 2 | 0 |
| `test_follow_target_state.py` | 1 | 0 |
| `test_install_smoke.py` | 0 imports BUT **8 references in test code itself** — this is the renamed-package smoke test. ALL must rewrite (test names, `import_module("drone_follow")` literals, `shutil.which("drone-follow")` call) **AND** add a sibling `test_robot_follow_help_exits_zero` that ALSO asserts both `robot-follow` and `drone-follow` console scripts work | 8 |
| `test_reid_drift_protection.py` | 1 | 0 |
| `test_reid_gallery_coherence_gate.py` | 1 | 0 |
| `test_shared_state.py` | 1 | 0 |
| `test_sim_worlds.py` | 1 import + `RECORDINGS_DIR = REPO_ROOT / "drone_follow" / "recordings"` STRING LITERAL on L28 | 1 critical literal + 3 comments |
| `test_tracker_protocol.py` | 3 | 0 |
| `test_velocity_api_and_smoother.py` | 2 | 0 |
| `test_velocity_command_shape.py` | 1 | 0 |

### `test_install_smoke.py` — the key test (rewrite + extend)

Current asserts:
1. `import_module("drone_follow")` works
2. `import_module("drone_follow.follow_api.{types,config,controller,state}")` all work
3. `shutil.which("drone-follow")` non-null AND `drone-follow --help` exits 0

**After rename it must assert:**
1. `import_module("robot_follow")` works
2. `import_module("robot_follow.follow_api.{types,config,controller,state}")` all work
3. `shutil.which("robot-follow")` non-null AND `robot-follow --help` exits 0
4. **NEW:** `shutil.which("drone-follow")` non-null AND `drone-follow --help` exits 0 — alias verification
5. **NEW (optional):** assert both `--help` outputs are byte-identical (success criterion #2 from ROADMAP). Cheap and decisive.

**This file IS the verification for RENAME-01 + RENAME-02.**

## Pitfalls

### Pitfall 1: Stale `__pycache__` masks the rename
**What goes wrong:** `git mv drone_follow robot_follow` does NOT remove the `drone_follow/__pycache__/*.pyc` files (gitignored, untracked). Python's import system can use the orphan `.pyc` to "resolve" `import drone_follow` even after the source moved.
**Why it happens:** `.pyc` files don't need their parent `.py` to exist; Python falls back to the bytecode.
**How to avoid:** Before re-running `pip install -e .`, run:
```bash
find . -name __pycache__ -type d -prune -exec rm -rf {} +
find . -name '*.pyc' -delete
```
**Warning signs:** `python -c 'import drone_follow'` succeeds (returns the OLD module) after the rename + reinstall. Or `pytest` collects but fails with `ModuleNotFoundError: drone_follow.foo` despite all sources looking right.

### Pitfall 2: Console-script shim caches old import path
**What goes wrong:** `pip install -e .` does NOT regenerate `bin/drone-follow` if pip thinks the distribution name `drone-follow` is the same one being reinstalled. The shim retains `from drone_follow.drone_follow_app import main` and bombs at runtime.
**Why it happens:** With a clean rename (old name `drone-follow` → new name `robot-follow`), pip sees a NEW distribution. The old metadata + shim are leftovers from a different distribution.
**How to avoid:** Run `pip uninstall drone-follow -y` BEFORE `pip install -e .`. This is the `install.sh` insertion. After both, both `bin/drone-follow` and `bin/robot-follow` exist with `from robot_follow.robot_follow_app import main`.
**Warning signs:** `cat $(which drone-follow)` shows `from drone_follow.drone_follow_app import main` post-install.

### Pitfall 3: Editable install metadata drift across machines
**What goes wrong:** On a deployed RPi that has been running `drone-follow` for months, pip's metadata still has the old distribution. Just doing `git pull` + `pip install -e .` produces TWO distributions (`drone-follow` AND `robot-follow`) — `pip list` shows both, `pip show` shows different things, and which shim wins on PATH becomes order-dependent.
**Why it happens:** pip's RECORD file is per-distribution; reinstalling under a different distribution name does NOT cascade-uninstall the old one.
**How to avoid:** The `install.sh` `pip uninstall drone-follow -y` line is **mandatory**, not cosmetic. Every deployed unit needs `./install.sh` re-run after the rename commit lands.
**Warning signs:** `pip list | grep -i follow` shows both `drone-follow` and `robot-follow`. Or `pip show drone-follow` returns metadata (success criterion 1 fails).

### Pitfall 4: IDE/LSP/import-resolver caches
**What goes wrong:** VSCode/Pylance/jedi caches resolved imports. After the rename, the IDE may report import errors that are stale and disappear on restart.
**Why it happens:** Per-IDE; not a CLI issue.
**How to avoid:** Document in commit message that contributors should reload their IDE workspace. Not a verification-gate concern.

### Pitfall 5: `drone_follow_app.py` `python -m` invocations
**What goes wrong:** Anyone (a script, a CI job, a memory in someone's head) doing `python -m drone_follow.drone_follow_app` breaks.
**Why it happens:** CONTEXT accepts this break. Console scripts are the supported surface.
**How to avoid:** Inventory check — `git grep 'python -m drone_follow'` returns nothing in this repo. **Verified clean** (0 hits). Document it in commit message anyway.
**Warning signs:** None expected.

### Pitfall 6: Boot-service start_air.sh path-traversal assumes parent dir layout
**What goes wrong:** `scripts/boot/drone-follow-boot.sh` L14 computes `APP_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"`. This is **path-shape, not path-name** dependent. Renaming the package dir does NOT change this — `scripts/boot/` stays at `scripts/boot/`.
**Verification:** Re-read the script (already done — it's two `..` jumps from `scripts/boot/`, totally independent of package dir name).

### Pitfall 7: `system/drone-network-mode.sh` references `drone-follow.service` (NOT the boot service)
**What goes wrong:** Line 7: `DRONE_SERVICE="drone-follow.service"` and line 39: `systemctl --user start "$DRONE_SERVICE"`. This is a **systemd USER service** named `drone-follow.service` — distinct from `drone-follow-boot.service`. The CONTEXT preserves the boot service explicitly (RENAME-04) but doesn't say a word about this user-level service.
**Why it might happen:** The `system/install.sh` actively DISABLES this user service (L32: `systemctl --user disable drone-follow.service`). It may be vestigial/disabled by default. Unclear if it's still in use.
**How to investigate:** Planner should ask user: "Is `drone-follow.service` (user-level, not the boot one) still in active use? If yes, is its name preserved like the boot service?"
**Recommendation:** Treat as a deferred-cleanup question, NOT a blocker for Phase 1. Default behavior: leave verbatim (preserves RENAME-04 spirit — user-deployed unit file).

### Pitfall 8: `git log --follow` doesn't follow the import-rewrite, only the file move
**What goes wrong:** `git log --follow robot_follow/follow_api/config.py` will show the rename + history. But searching commit history for the old `drone_follow.follow_api.config` import path on the renamed file will only surface the rename commit.
**Why it happens:** Inherent to git. CONTEXT accepts this.
**How to avoid:** None needed; documented for reviewers.

### Pitfall 9: pip cache may have a `drone-follow-0.1.0-*.whl` cached
**What goes wrong:** If a future user does `pip install drone-follow` (without `-e`), pip's cache could return the old wheel.
**Why it happens:** pip caches built wheels by name.
**How to avoid:** N/A — `drone-follow` is no longer a publishable distribution name (it's a console-script alias). Nobody is `pip install drone-follow`-ing from PyPI; we're editable-only. CONTEXT's "deferred — only if testing reveals an actual problem" is correct.

### Pitfall 10: Submodule cwd quirk in `setup_env.sh`
**What goes wrong:** `setup_env.sh` does `cd "$APPS_DIR"` and sources `hailo-apps/setup_env.sh` which uses `$(pwd)` for PYTHONPATH. After the rename, `pwd` inside `hailo-apps/` exports a PYTHONPATH that does NOT include the repo root (which is `..`).
**Why it might happen:** Pre-existing behavior — but `hailo-apps/setup_env.sh` adds `$(pwd)` (= `hailo-apps/`) to PYTHONPATH. The drone-follow package is found via the editable install in the venv's `site-packages`, not via PYTHONPATH. So nothing breaks.
**Verification:** Already in production today; rename doesn't disturb it.

## Verification Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest 9.0.2 (`/home/guyz/.local/bin/pytest`) — system pytest, not in venv |
| Config file | none (default discovery via `drone_follow/tests/conftest.py`) |
| Quick run command | `pytest robot_follow/tests/test_install_smoke.py -x` (single fast smoke) |
| Full suite command | `pytest robot_follow/tests/ -x` |
| Pre-existing tier-1 smoke | `test_install_smoke.py` — already exists, needs rewrite (see below) |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| **RENAME-01** | Package importable under new name; no `drone_follow.*` import path resolves | unit | `python -c 'import robot_follow; import robot_follow.follow_api.config'` (must succeed) | ✅ asserts in `test_install_smoke.py` (after rewrite) |
| **RENAME-01** | No source file imports the old name | grep | `! git grep -nE 'from drone_follow\|import drone_follow' -- '*.py'` | ✅ new test or shell command |
| **RENAME-01** | Old import path raises | unit | `python -c 'import drone_follow' && exit 1 || exit 0` (must FAIL — i.e. exit 0 from `\|\|`) | ❌ Wave 0: add to `test_install_smoke.py` |
| **RENAME-02** | `pip show robot-follow` succeeds; `pip show drone-follow` fails | smoke | `pip show robot-follow >/dev/null && ! pip show drone-follow >/dev/null 2>&1` | ❌ Wave 0: add to `test_install_smoke.py` |
| **RENAME-02** | Both console scripts on PATH | smoke | `command -v robot-follow && command -v drone-follow` | ❌ Wave 0: extend `test_install_smoke.py` |
| **RENAME-02** | `robot-follow --help` and `drone-follow --help` identical | smoke | `diff <(robot-follow --help) <(drone-follow --help)` | ❌ Wave 0: add to `test_install_smoke.py` |
| **RENAME-03** | `setup_env.sh` clean source | smoke | `bash -c 'source setup_env.sh && python -c "import robot_follow"'` | manual; not pytestable easily |
| **RENAME-03** | `install.sh` idempotent re-run completes | smoke | `./install.sh --skip-apps --skip-hefs --skip-ui` (single dev box only) | manual; tier-2 only |
| **RENAME-03** | `scripts/start_air.sh` doesn't error on path resolution (cannot run end-to-end without OpenHD + camera) | manual-only | `bash -n scripts/start_air.sh` (syntax check) + path-exists check on `${APP_ROOT}/robot_follow/ui/build` | shell-only |
| **RENAME-04** | Boot service files preserved verbatim | grep | `test -f scripts/boot/drone-follow-boot.service && grep -q 'drone-follow-boot' scripts/boot/drone-follow-boot.service` | shell-only |
| **RENAME-04** | `~/Desktop/drone-follow.conf` name unchanged in scripts/boot/install.sh | grep | `grep -q 'drone-follow.conf' scripts/boot/install.sh` | shell-only |
| **RENAME-05** | `git grep` gate (no rogue `drone_follow` outside whitelist) | grep | `git grep -nE 'drone_follow' \| grep -vE '^\\.planning/\|^README\\.md\|^CLAUDE\\.md'` returns 0 lines (or only known-good whitelist) | shell-only — pre-commit |

### Sampling Rate

**Per task commit (pre-commit, no Hailo HW required, < 30 s):**
```bash
# 1. No stale bytecode
find . -name __pycache__ -type d -prune -exec rm -rf {} +
find . -name '*.pyc' -delete

# 2. Import works under new name
python -c 'import robot_follow; import robot_follow.follow_api.config; import robot_follow.follow_api.controller; import robot_follow.follow_api.state; import robot_follow.follow_api.types'

# 3. Old import fails (negative assert)
! python -c 'import drone_follow' 2>/dev/null

# 4. Both console scripts work and produce identical --help
command -v robot-follow
command -v drone-follow
diff <(robot-follow --help 2>&1) <(drone-follow --help 2>&1)

# 5. pip metadata
pip show robot-follow >/dev/null
! pip show drone-follow >/dev/null 2>&1

# 6. Grep gate (whitelist regex)
test -z "$(git grep -nE 'drone_follow|from drone_follow|import drone_follow' \
         | grep -vE '^\.planning/|^README\.md|^CLAUDE\.md.*alias')"

# 7. Smoke tests (only the install/import ones, no Hailo)
pytest robot_follow/tests/test_install_smoke.py -x
```

**Per wave merge (pre-push, Hailo-capable host, full coverage):**
```bash
# All of the above, plus:
pytest robot_follow/tests/ -x --ignore=robot_follow/tests/test_sim_worlds.py
# (test_sim_worlds.py needs PX4 SITL + Gazebo + bridge running — separate tier)
```

**Phase gate (pre-`/gsd:verify-work`):**
- Pre-commit suite green
- `pytest robot_follow/tests/` (full, including sim if running)
- Manual: `scripts/start_air.sh --mode stream` syntax-checks (`bash -n`)
- Manual on a dev box: `./install.sh --skip-apps --skip-hefs --skip-ui` re-runs cleanly

### Wave 0 Gaps

- [ ] **Rewrite `robot_follow/tests/test_install_smoke.py`** to:
  - assert `import robot_follow` works (replaces current `import drone_follow` test)
  - assert all `robot_follow.follow_api.*` submodules import
  - **NEW**: assert `import drone_follow` raises `ModuleNotFoundError`
  - **NEW**: assert both `which robot-follow` and `which drone-follow` resolve
  - **NEW**: assert `robot-follow --help` exits 0 AND `drone-follow --help` exits 0
  - **NEW**: assert byte-identical `--help` output (single `subprocess.run` each, compare `stdout + stderr`)
  - **NEW**: assert `pip show robot-follow` succeeds AND `pip show drone-follow` fails (via `subprocess.run`)
- [ ] **No new conftest/fixtures needed** — existing `conftest.py` adds repo root to `sys.path`; rename does not break this (the `..` traversal lands at repo root regardless of package name).
- [ ] **No new test framework install needed** — pytest 9.0.2 already on PATH (`/home/guyz/.local/bin/pytest`).

## Open Questions for the Planner

1. **`system/drone-network-mode.sh` references `drone-follow.service` (a user-level systemd unit)** — distinct from `drone-follow-boot.service` (boot, RENAME-04-preserved). The user service is actively `systemctl --user disable`d by `system/install.sh`. Is it vestigial or still in use somewhere?
   - **Recommendation:** Leave verbatim (matches RENAME-04 spirit — user-deployed unit-file names preserved). Flag for user confirmation on commit.

2. **`scripts/bench_reid_callback.py`** — already broken (imports non-existent `reid_worker`); on CLEAN-02 dead-code list for Phase 2.
   - **Option A:** Mechanical sed rewrites its 2 imports anyway. Stays broken but stays consistent. Phase 2 deletes the file.
   - **Option B:** Skip it in the sed pass; file remains importing `drone_follow.*` which post-rename triggers the grep-gate. Add to whitelist for Phase 1.
   - **Recommendation:** Option A. Cleaner grep-gate, file dies next phase anyway.

3. **`drone_follow/ui/package.json` `"name": "drone-follow-ui"`** — package-lock.json also has this. CONTEXT mentions verifying — research confirms it exists.
   - **Option A:** Rename to `"robot-follow-ui"`, regenerate package-lock.json (`npm install`).
   - **Option B:** Leave verbatim. The name is only consumed by npm internally; it's not user-visible. Vite/build output is identical.
   - **Recommendation:** Option B. Avoids regenerating package-lock.json (which would create a noisy diff). Add as a deferred cleanup if desired.

4. **`drone_follow/ui/index.html` `<title>Drone Follow UI</title>`** — user-visible browser tab title.
   - **Recommendation:** Rewrite to "Robot Follow UI" (single character-level edit, no build churn). Defer if planner prefers.

5. **`docs/superpowers/plans/*.md`** — 179 + 33 + 1 references across three historical plan documents.
   - **Recommendation:** Leave verbatim. They are dated implementation plans; rewriting them would lie about what was planned at the time. Add to grep-gate whitelist.

6. **`pyproject.toml` version bump** — CONTEXT lists "bump to v1.1-dev" as "likely yes, but a packaging-discipline decision for the planner."
   - **Recommendation:** Bump to `"1.1.0.dev0"` per [PEP 440](https://peps.python.org/pep-0440/) dev-release convention. Single-line edit. Documents that v1.1 is in progress.

7. **Whitespace/line-break in `pyproject.toml` `[project.scripts]`** — current has one entry; target has two. Trivial. Planner discretion on `# alias for boot service + muscle memory` comment.

8. **`mafat/run_bench.py` and `mafat/tiling_record.py`** — each have one `drone-follow` mention in comments.
   - **Recommendation:** Rewrite (text-only, mechanical). Or leave (mafat/ may be a parallel tool tree unrelated to follow loop). Confirm with user if planner uncertain.

9. **Test `test_sim_worlds.py` L28 string literal** `RECORDINGS_DIR = REPO_ROOT / "drone_follow" / "recordings"` — the test reads recording artifacts to validate sim runs. Must rewrite to `"robot_follow"`.
   - **Recommendation:** This is mandatory; not optional. Catch-block: `--record-output` default in `robot_follow_app.py` L101 must ALSO match this path. Verify both rewrites are made.

## Sources

### Primary (HIGH confidence — direct file inspection on `feature/rover-support` HEAD)
- All `git grep` and `find` results in this document — verified 2026-05-14
- `pyproject.toml`, `install.sh`, `setup_env.sh`, `run_drone.sh`, `scripts/start_air.sh`, `scripts/install_air.sh`, `scripts/boot/*` — read directly
- `drone_follow/__init__.py`, `drone_follow/drone_follow_app.py`, `drone_follow/tests/conftest.py`, `drone_follow/tests/test_install_smoke.py` — read directly
- `hailo-apps/venv_hailo_apps/bin/drone-follow` — console-script shim contents verified
- `.gitignore` — confirms egg-info + pycache patterns

### Secondary (MEDIUM confidence — web research, cross-verified with multiple sources)
- pip + console_scripts editable-install behavior — [setuptools editable docs](https://setuptools.pypa.io/en/latest/userguide/development_mode.html), [pip Issue #5997](https://github.com/pypa/pip/issues/5997), [pythontutorials.net cleanup guide](https://www.pythontutorials.net/blog/how-to-cleanly-uninstall-my-python-packages-with-pip3-or-any-other-way/)
- `__pycache__` stale-bytecode behavior — [PEP 3147](https://peps.python.org/pep-3147/), [Towards Data Science __pycache__ guide](https://towardsdatascience.com/pycache-python-991424aabad8/), [pip Issue #11835](https://github.com/pypa/pip/issues/11835)

### Tertiary (LOW confidence — none flagged)
- N/A — all findings verified against the actual codebase.

## Metadata

**Confidence breakdown:**
- Inventory completeness: HIGH — every grep cross-checked, counts match CONTEXT.md's "~48"
- pyproject change set: HIGH — current file read verbatim, target derived from CONTEXT
- Shell/script change set: HIGH — every script read or grep-confirmed
- Doc inventory: HIGH — `git grep -c` per file
- Tests change set: HIGH — every test file enumerated, smoke test contents inspected
- Pitfalls: HIGH for technical (pycache, console-script regen, egg-info), MEDIUM for IDE/LSP (project-specific)
- Verification architecture: HIGH — commands constructed from inspection, no Hailo HW assumed for tier-1

**Research date:** 2026-05-14
**Valid until:** 2026-06-14 (30 days — mechanical refactor, codebase frozen on `feature/rover-support` for this phase)
