# Phase 1: Rename - Context

**Gathered:** 2026-05-13
**Status:** Ready for planning

<domain>
## Phase Boundary

Mechanical rename `drone_follow/` → `robot_follow/`. Updates the package directory, internal imports (~48 sites), `pyproject.toml`, shell scripts, and docs/memory text references. `drone-follow` console-script alias is preserved so the boot service unit, `~/Desktop/drone-follow.conf`, and existing field deployments keep working unchanged.

Sub-package names (`drone_api/`, `follow_api/`, `pipeline_adapter/`, `servers/`, etc.) are **out of scope here** — they renamed/reshuffled in Phase 3 (`drone_api/mavsdk_drone.py` → `robot_api/adapters/mavsdk_drone.py`) and Phase 2 (cleanup). This phase only touches the top-level package and the rename's mechanical surface.

**Branch scope:** Lives only on `feature/rover-support`. `main` continues to ship as `drone_follow` until v1.1 merges.

</domain>

<decisions>
## Implementation Decisions

### Commit shape & ordering
- **Single atomic commit** landing dir rename + 48 import rewrites + pyproject + shell scripts + docs. Every commit on the branch stays buildable; `git bisect` survives.
- **`git mv drone_follow robot_follow`** — git records the move, `git log --follow` survives.
- **`drone_follow_app.py` is also renamed** → `robot_follow_app.py`. Console scripts target `robot_follow.robot_follow_app:main`. The package and its main module share the family name; obscure `python -m drone_follow.drone_follow_app` invocations are an acceptable break (the console scripts are the supported surface).
- **Verification gate (two-tier):**
  - Pre-commit (fast, no Hailo HW needed): lint passes, `python -c 'import robot_follow'` works, `drone-follow --help` and `robot-follow --help` both run and produce identical output.
  - Pre-push (Hailo-capable host): full `pytest` suite under `robot_follow/tests/`.

### pyproject name + alias surface
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

### UI directory location
- **Keep nested** — `drone_follow/ui/` becomes `robot_follow/ui/`. The `git mv` already does this; no Vite/npm config churn.
- Defer hoisting to repo-root `ui/` until a second consumer exists (e.g., a rover-specific UI variant); premature now.

### Recordings directory
- **Keep nested** — `drone_follow/recordings/` becomes `robot_follow/recordings/`. `--record-output` default path follows the package rename mechanically.

### Docs & examples normalization
- **Rewrite every `drone-follow --...` example to `robot-follow --...`** across `README.md`, `CLAUDE.md`, `TROUBLESHOOTING.md`, `docs/*.md`, `PARAMETERS.md`, `RESOLUTION_CONTROL.md`, `SETUP_GUIDE.md`, `TEST_PLAN.md`, and the `system/` and `scripts/` shell-script comments.
- **Mention the `drone-follow` alias once** near the top of `README.md` and once in `CLAUDE.md` — "drone-follow is a kept alias of robot-follow for backwards compat with the boot service / deployed units."
- **`.claude/memory/*.md`** — update text references (commands, examples, file paths inside the package). Don't rename the memory files themselves; their behavior content (gotchas, key handling) stays. The `drone-follow-dev` skill's trigger phrases and dir name are explicitly out of scope here (would require coordinating with the user's local Claude config; not a Phase 1 concern).

### Top-level helper file names
- **Internal contents updated; filenames preserved** — `run_drone.sh`, `df_params.json`, `scripts/start_air.sh`, `scripts/install_air.sh`, etc. keep their names. Their contents are updated for the new import path / package name.
- `drone-follow-boot.service` and `~/Desktop/drone-follow.conf` are explicitly preserved (RENAME-04).
- Renaming `run_drone.sh` → `run_robot.sh` and `df_params.json` → `rf_params.json` belongs in Phase 3 alongside the `run_drone()` → `run_robot()` function rename, **not here**.

### Verification stragglers
- **`git grep` gate before commit:** `git grep -nE 'drone_follow|from drone_follow|import drone_follow'` must return nothing except a small whitelist:
  - The one-sentence alias mention in `README.md` + `CLAUDE.md`
  - `.planning/MILESTONES.md` historical/baseline references
  - This `CONTEXT.md` itself + roadmap/requirements (planning artefacts, not source)
- Code references to the import path `drone_follow.*` must be zero.

### Claude's Discretion
- Exact wording of the README/CLAUDE.md alias note.
- Whether to add a one-line `[deprecated]` placeholder in `pyproject.toml` keywords/metadata — likely no, but planner can decide.
- Test-path config (`pyproject.toml` `[tool.pytest.ini_options]` if present, or `pytest.ini`) — Claude updates whatever's wired.
- Whether `install.sh` reports the uninstall step to stdout or stays silent (cosmetic).
- Ordering of file edits within the single commit (mechanical, doesn't affect outcome).
- Pre-commit hook behavior in this repo — Claude reads `.pre-commit-config.yaml` (if any) and adapts.

</decisions>

<specifics>
## Specific Ideas

- The Phase 1 success criterion verbatim is the contract: `pip show robot_follow` shows the renamed package; `pip show drone_follow` returns nothing. This is achieved by **single distribution named `robot-follow`** + **`install.sh` runs `pip uninstall drone-follow -y` first**.
- Boot service compatibility is the hardest constraint. `drone-follow-boot.sh` reads `~/Desktop/drone-follow.conf` (`ENABLED`, `MODE`) and calls `scripts/start_air.sh`. After the rename: same file names, same script path, same conf key names — only the venv binary `drone-follow` resolves to the renamed package via the console-script alias.
- Branch-scope decision (`feature/rover-support` only; main stays drone_follow) is logged in user memory (`v11_rename_branch_scope.md`) — relevant context for the eventual merge-back to main, but not a Phase 1 implementation concern.

</specifics>

<code_context>
## Existing Code Insights

### Reusable Assets
- `git mv` — preserves history; native git operation handles the dir rename cleanly.
- `[project.scripts]` in pyproject already declares one console script; the schema trivially supports two pointing at the same target.
- `install.sh` exists and is the canonical onboarding/upgrade path on every deployment — natural place to wedge the `pip uninstall drone-follow -y` migration step.
- `setup_env.sh` already activates `./hailo-apps/venv_hailo_apps` and exports `PYTHONPATH`/`HAILO_APPS_PATH` — only needs path-string updates, no structural change.

### Established Patterns
- ~48 internal imports of the form `from drone_follow.X import Y` — uniform shape, ripe for a single `sed -i 's/from drone_follow\b/from robot_follow/g; s/import drone_follow\b/import robot_follow/g'` pass over `.py` files plus a verification grep.
- Tests live at `drone_follow/tests/` — pytest discovery is package-relative, so the `git mv` carries them along with no pytest config change required (assuming no hard-coded paths inside tests; verify during planning).
- `vision_branches.py`, `mavsdk_drone.py`, `web_server.py`, `openhd_bridge.py`, `hailo_drone_detection_manager.py` and other modules all sit inside `drone_follow/` — none are touched by Phase 1 beyond their import lines and parent dir name.
- `recordings/` is gitignored; no committed artifacts move.

### Integration Points
- `pyproject.toml` (`name`, `[tool.setuptools] packages.include`, `[project.scripts]`) — the contract surface for pip.
- `scripts/start_air.sh` — invokes `drone-follow` by name from PATH; works unchanged via the alias.
- `scripts/boot/drone-follow-boot.sh` — same. Boot service unit-file path on disk (`/etc/systemd/system/drone-follow-boot.service`) is unchanged, also per RENAME-04.
- `setup_env.sh` and `install.sh` — both reference the package import path; both updated in the single rename commit.
- `drone_follow/ui/package.json` and `package-lock.json` — Vite app metadata. Path-relative, so the `git mv` is sufficient; no `name` field change required unless it currently says `drone-follow` (verify in planning).
- `.claude/memory/*.md` and `.claude/skills/drone-follow-dev/SKILL.md` — text-only references. Skill dir/trigger phrases NOT renamed in Phase 1.

### Out-of-scope code (does not move/change in Phase 1)
- `drone_follow/drone_api/mavsdk_drone.py` stays at `robot_follow/drone_api/mavsdk_drone.py`. It moves to `robot_follow/robot_api/adapters/mavsdk_drone.py` in Phase 3 (ABS-03).
- All identified dead code (`sim/world_loader.py`, `scripts/bench_reid_callback.py`, `--vfov`, etc.) is **not** removed here — Phase 2 (Cleanup) owns that.
- `run_drone()` function name in the composition root stays — renamed to `run_robot()` in Phase 3 (ABS-08).

</code_context>

<deferred>
## Deferred Ideas

- **Hoist `ui/` and `recordings/` to repo-root** — defer until a second UI consumer or output-path policy decision arises. Not a v1.1 phase candidate yet.
- **Rename `drone_follow_app.py` callers' `python -m` muscle memory in user notes** — out of scope; console scripts are the supported surface.
- **Rename `run_drone.sh`, `df_params.json`** → covered in Phase 3 alongside `run_drone()` → `run_robot()`.
- **Rename `.claude/skills/drone-follow-dev/` skill** → out of scope (requires coordinating with user's local Claude config and trigger phrases).
- **Bump pyproject `version` to v1.1-dev** — likely yes, but a packaging-discipline decision for the planner; not a gray area worth the user's time.
- **Sticky `pip cache purge`-style cleanup** for users hitting cached old-name wheels — only if testing reveals an actual problem.

</deferred>

---

*Phase: 01-rename*
*Context gathered: 2026-05-13*
