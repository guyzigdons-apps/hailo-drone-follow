---
phase: 01-rename
plan: 02
subsystem: infra
tags: [rename, refactor, pyproject, console-scripts, alias, atomic-commit, argparse]

# Dependency graph
requires:
  - phase: 01-rename
    provides: "Plan 01-01 Wave 0 — forward-compatible install smoke test (8 skip-guarded + 2 always-on)"
provides:
  - "Package renamed: drone_follow/ -> robot_follow/ via git mv (history preserved via git log --follow)"
  - "Main module renamed: drone_follow_app.py -> robot_follow_app.py"
  - "pyproject.toml: name=robot-follow, version=1.1.0.dev0, both console scripts (robot-follow primary + drone-follow alias) map to robot_follow.robot_follow_app:main"
  - "install.sh: idempotent 'pip uninstall drone-follow -y' before 'pip install -e .' so deployed units get clean metadata"
  - "Tier 1/2 docs (README, CLAUDE, TROUBLESHOOTING, PARAMETERS, SETUP_GUIDE, RESOLUTION_CONTROL, TEST_PLAN, docs/*.md) rewritten with robot-follow CLI examples and robot_follow/ path references"
  - "Alias note added near top of README.md and CLAUDE.md documenting drone-follow as permanent alias"
  - ".claude/memory/{tracking_callback_risks,webui_build}.md + drone-follow-dev/SKILL.md L76 updated"
  - "Smoke test stripped of forward-compat skip guards: 10 tests assert unconditionally (10 PASSED, 0 SKIPPED)"
  - "parser.prog='robot-follow' pinned so byte-identical --help is achievable across console-script aliases (auto-fix)"
affects: [01-03-rename-followups, 02-cleanup, 03-abstraction, 04-rover-adapter, 05-rover-sim]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Single atomic commit for milestone-shaping rename — every commit on the branch stays buildable; git bisect survives"
    - "git mv preserves history across rename (git log --follow surfaces pre-rename commits on renamed files)"
    - "Console-script alias pattern in pyproject.toml [project.scripts]: two entries pointing at the same target = byte-identical entry points after pip install"
    - "Idempotent 'pip uninstall <legacy> -y' before 'pip install -e .' to clean stale distribution metadata on deployed units"
    - "argparse parser.prog pinning so console-script aliases produce identical --help output regardless of invocation name"

key-files:
  created:
    - .planning/phases/01-rename/01-02-SUMMARY.md
  modified:
    - robot_follow/  # entire tree (was drone_follow/)
    - robot_follow/robot_follow_app.py  # was drone_follow_app.py; +parser.prog pin
    - pyproject.toml
    - install.sh
    - scripts/install_air.sh
    - scripts/bench_reid_callback.py
    - mafat/run_bench.py
    - mafat/tiling_record.py
    - configs/overlay_style.yaml
    - df_params.json
    - .gitignore
    - README.md
    - CLAUDE.md
    - TROUBLESHOOTING.md
    - PARAMETERS.md
    - SETUP_GUIDE.md
    - RESOLUTION_CONTROL.md
    - TEST_PLAN.md
    - docs/tracking-reid-algorithm.md
    - docs/calibration-flight-guide.md
    - docs/control-architecture.md
    - docs/design-review.md
    - reid_analysis/reid_analysis_app.py
    - .claude/memory/tracking_callback_risks.md
    - .claude/memory/webui_build.md
    - .claude/skills/drone-follow-dev/SKILL.md

key-decisions:
  - "Pin parser.prog='robot-follow' in _build_parser() so both console-script aliases produce byte-identical --help output (required by Phase 1 success criterion 2; argparse's natural sys.argv[0] derivation breaks this)"
  - "Rewrote webui_build.md path examples from the obsolete in-repo layout (community/apps/hailo_drone_follow/...) to the current standalone repo layout (<repo-root>/robot_follow/ui/) — closes a stale doc reference while we're touching it"
  - "Stripped Wave-0 skip-guards from robot_follow/tests/test_install_smoke.py in the same atomic commit as the rename (per plan Step 5) — keeps smoke-test contract aligned with the new tree state in a single commit (git bisect-safe)"

patterns-established:
  - "Atomic-commit discipline for milestone shapers: even when execute-plan.md's default is per-task commits, structural renames commit ONCE with the full verification gate run before the commit"
  - "Console-script alias preservation: pyproject.toml [project.scripts] supports declaring N names pointing at the same target; combined with parser.prog pinning, both names are indistinguishable at runtime"

requirements-completed: [RENAME-01, RENAME-02, RENAME-03, RENAME-04, RENAME-05]

# Metrics
duration: 73 min
completed: 2026-05-14
---

# Phase 01 Plan 02: Atomic drone_follow -> robot_follow Rename Summary

**Single atomic commit (`5850558`) renames the package, rewrites 48 internal imports across 19 files, updates pyproject (name=robot-follow, version=1.1.0.dev0, both console scripts), idempotent-uninstalls the legacy distribution in install.sh, rewrites Tier 1/2 docs + .claude memory, strips forward-compat skip guards from the smoke test, and pins argparse parser.prog so the `robot-follow` and `drone-follow` console-script aliases produce byte-identical --help — 10/10 smoke tests pass, grep gate clean, boot service preserved verbatim.**

## Performance

- **Duration:** 73 min
- **Started:** 2026-05-14T14:09:43Z
- **Completed:** 2026-05-14T15:23:19Z
- **Tasks:** 3 (atomic; one commit)
- **Files modified:** 77 (per `git show --shortstat 5850558`)
- **Lines:** +232 / -241

## Accomplishments

- `git mv drone_follow robot_follow` + `git mv robot_follow/drone_follow_app.py robot_follow/robot_follow_app.py` — history preserved across rename.
- 48 internal imports rewritten across 19 .py files via 8-clause sed pass (no remaining `from drone_follow` / `import drone_follow` in `*.py` outside the smoke test's intentional negative-assertion).
- `pyproject.toml`: `name="robot-follow"`, `version="1.1.0.dev0"`, `packages.find.include=["robot_follow*", "reid_analysis*"]`, `[project.scripts]` with both `robot-follow` (primary) and `drone-follow` (alias) mapping to `robot_follow.robot_follow_app:main`.
- `install.sh`: idempotent `pip uninstall drone-follow -y` inserted before `pip install -e .` (no-op on fresh installs, clears stale metadata on deployed units). UI pushd path -> `robot_follow/ui`.
- `scripts/install_air.sh`: chown paths -> `${APP_ROOT}/robot_follow/ui/{node_modules,build}`.
- Tier 1 docs rewritten: README.md (+ alias note), CLAUDE.md (+ alias note), TROUBLESHOOTING.md, PARAMETERS.md, SETUP_GUIDE.md, RESOLUTION_CONTROL.md, TEST_PLAN.md.
- Tier 2 docs rewritten: docs/tracking-reid-algorithm.md, docs/calibration-flight-guide.md, docs/control-architecture.md, docs/design-review.md.
- `.claude/memory/{tracking_callback_risks,webui_build}.md` path refs updated; `.claude/skills/drone-follow-dev/SKILL.md` L76 updated.
- `robot_follow/tests/test_install_smoke.py` skip-guards stripped — file is now strict-mode (10 tests assert unconditionally; 10 PASSED, 0 SKIPPED, 0 FAILED post-commit).
- Boot service files (`scripts/boot/drone-follow-boot.{service,sh}`, `scripts/boot/install.sh`, `scripts/boot/uninstall.sh`) and `system/*` user-level systemd files preserved verbatim per RENAME-04 + planner Q1 — `git diff scripts/boot/ system/` is empty.

## Task Commits

This plan was committed as ONE atomic commit (per CONTEXT-locked decision; overrides execute-plan.md's per-task default). Tasks 1, 2, 3 staged sequentially without intermediate commits; full verification gate run before the single commit:

1. **Atomic rename (Tasks 1+2+3 combined)** — `5850558` (refactor)

**Plan metadata commit:** to follow this summary (state updates).

## Files Created/Modified

(See key-files.modified frontmatter for the full inventory. Highlights below.)

**Renamed (77 git renames + main module rename):**
- `drone_follow/` -> `robot_follow/` (entire tree; 54 files via git mv)
- `drone_follow/drone_follow_app.py` -> `robot_follow/robot_follow_app.py`

**Configuration / metadata:**
- `pyproject.toml` — `name=robot-follow`, `version=1.1.0.dev0`, both console scripts.
- `install.sh` — pip-uninstall step + UI pushd path + banner text.
- `scripts/install_air.sh` — chown paths.
- `.gitignore` — `robot_follow/ui/{node_modules,build}/`.
- `configs/overlay_style.yaml` — comment path ref.
- `df_params.json` — 4 description strings (recordings path + CLI invocations).

**Source code (path/log/comment refs):**
- `robot_follow/__init__.py` — module docstring updated; entry-point file ref.
- `robot_follow/robot_follow_app.py` — module docstring; recordings default path; comment refs; **`parser.prog="robot-follow"` pin (auto-fix)**.
- `robot_follow/drone_api/mavsdk_drone.py` — `logging.getLogger("robot_follow.telemetry")`.
- `robot_follow/tests/test_install_smoke.py` — skip-guards stripped, docstring rewritten for post-rename canonical state, 10 tests assert unconditionally.
- `robot_follow/tests/test_sim_worlds.py` — RECORDINGS_DIR literal + 3 comment refs.
- `robot_follow/tests/conftest.py` — 2 docstring/comment refs.
- `reid_analysis/reid_analysis_app.py` — 1 comment ref.
- `mafat/{run_bench,tiling_record}.py` — 1 comment ref each.

**Docs + .claude:**
- README.md (+ alias note at top), CLAUDE.md (+ alias note after Project Overview).
- TROUBLESHOOTING.md, PARAMETERS.md, SETUP_GUIDE.md, RESOLUTION_CONTROL.md, TEST_PLAN.md.
- docs/{tracking-reid-algorithm,calibration-flight-guide,control-architecture,design-review}.md.
- .claude/memory/{tracking_callback_risks,webui_build}.md.
- .claude/skills/drone-follow-dev/SKILL.md (L76 path ref only; skill dir + trigger phrases preserved per CONTEXT).

## Decisions Made

- **One atomic commit, not per-task.** CONTEXT-locked. Tasks 1+2+3 staged sequentially without intermediate commits; the full pre-commit verification gate (8 checks) ran once before the single commit landed. This guarantees `git bisect` survives every intermediate state and deployed units re-running `./install.sh` see a single atomic transition rather than a half-renamed tree.
- **Pin `parser.prog="robot-follow"` in `_build_parser()`.** Without this, argparse derives the program name from `sys.argv[0]`, so `robot-follow --help` and `drone-follow --help` differ by their `usage:` line. The plan's success criterion 2 ("byte-identical output") was technically unachievable as written; pinning prog makes it achievable. The pin is in the planning composition root, kept narrow.
- **Rewrote `webui_build.md` to current repo layout.** The file's path examples referenced `community/apps/hailo_drone_follow/...` — an obsolete location from when drone-follow lived as a subdirectory of hailo-apps-infra. While we were updating its `drone_follow/ui/` paths, we modernised them to `<repo-root>/robot_follow/ui/` to remove a stale doc reference. Doesn't touch source code; reduces grep-gate whitelist surface.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 2 - Missing critical functionality] Pin `parser.prog='robot-follow'` so console-script aliases produce byte-identical --help**

- **Found during:** Task 3 Step 5 (`diff <(robot-follow --help) <(drone-follow --help)`)
- **Issue:** Plan success criterion 2 ("byte-identical --help output") was unachievable as written. argparse derives `prog` from `sys.argv[0]`, so the two invocations produced output that differed in the `usage:` line (`usage: robot-follow [-h]` vs `usage: drone-follow [-h]` on L1 and L351). Without an explicit `parser.prog`, no amount of pyproject [project.scripts] aliasing fixes this — the divergence happens after pip's shim hands off to argparse.
- **Fix:** Added `parser.prog = "robot-follow"` in `_build_parser()` immediately after `get_pipeline_parser()` returns, with a comment explaining why.
- **Files modified:** robot_follow/robot_follow_app.py (after the rename, line ~171).
- **Verification:** `diff <(robot-follow --help 2>&1) <(drone-follow --help 2>&1)` now empty; `pytest robot_follow/tests/test_install_smoke.py::test_help_outputs_byte_identical` PASSES.
- **Committed in:** 5850558 (atomic rename commit).

**2. [Rule 1 - Bug] Remove stray repo-root `drone_follow.egg-info/` before pip uninstall**

- **Found during:** Task 3 Step 2 (pre-flight `pip uninstall drone-follow -y`)
- **Issue:** A stale `drone_follow.egg-info/` directory at the repo root (left over from a non-venv install — `Modified` date 2025-05-11) caused `pip uninstall drone-follow -y` to print "Not uninstalling drone-follow at /home/guyz/code/guyz/hailo-drone-follow, outside environment" instead of actually uninstalling the venv-installed editable. Pip's heuristic looks at egg-info location and refuses to operate on a location that doesn't match the active environment.
- **Fix:** `rm -rf drone_follow.egg-info` (the file is gitignored anyway — `.gitignore:3 *.egg-info/`), then re-ran `pip uninstall drone-follow -y` which then operated on the venv editable as expected.
- **Files modified:** none (the egg-info is gitignored; the rm is a runtime cleanup).
- **Verification:** `pip show drone-follow` exits non-zero post-uninstall; site-packages no longer contains `drone_follow-0.1.0.dist-info` or `__editable__.drone_follow-*.pth`.
- **Committed in:** N/A (gitignored; no file change committed).

**3. [Rule 3 - Blocking] Escape backticks in install.sh banner echo**

- **Found during:** Task 2 Step 1 (writing the rewrite for the final `echo` banner in install.sh)
- **Issue:** Initial draft used `` echo "    robot-follow --help    # or `drone-follow --help` (alias)" `` — the backticks inside the double-quoted echo would trigger command substitution at runtime (`drone-follow` would be invoked from inside `echo`).
- **Fix:** Replaced backticks with single-quote-style: `echo "    robot-follow --help    # 'drone-follow --help' (alias) also works"`.
- **Files modified:** install.sh (banner echo).
- **Verification:** `bash -n install.sh` passes; no command substitution risk.
- **Committed in:** 5850558 (atomic rename commit).

---

**Total deviations:** 3 auto-fixed (1 missing critical, 1 bug, 1 blocking).
**Impact on plan:** All three were necessary to make the plan's success criteria achievable. None are scope creep — they're load-bearing for the rename being correct, idempotent, and producing the byte-identical --help that ROADMAP requires.

## Issues Encountered

- **Smoke-test docstring tripped the grep gate on first run.** The first version of the rewritten `test_install_smoke.py` docstring contained the phrase `` `from drone_follow` / `import drone_follow` `` inside backticks for documentation purposes. The strict Task 1 Step 11 verify command (`git grep -nE 'from drone_follow|import drone_follow' -- '*.py'`) matched these backtick-quoted strings as if they were real import statements. Rephrased the docstring to "The legacy `drone_follow` import path raises ModuleNotFoundError at runtime" (no `from drone_follow` / `import drone_follow` phrase appears outside the test body's actual `pytest.raises(ModuleNotFoundError)` block).
- **Grep-gate whitelist needed two additions beyond the plan's draft regex.** The plan's whitelist didn't account for `^.claude/skills/safe-pull-and-rollback/` (historical branch-name references kept verbatim per planner Q5 spirit), `^SETUP_GUIDE.md` (3 historical in-repo-path references referring to the old hailo-apps-infra co-located layout), or `^robot_follow/tests/test_install_smoke.py` (the file that DEFINES the negative-assertion contract, so it must mention `drone_follow` literally in test names + docstrings). Also added `hailo_drone_follow` as a literal substring exclusion because `git grep -E 'drone_follow'` matches inside `hailo_drone_follow` (the legacy in-repo dir name, distinct from the package name). Documented in the temp gate script at `/tmp/phase1_grep_gate.sh`; Plan 01-03 will fold these into a permanent script if desired.

## Whitelist hits accepted (per grep gate)

Documenting accepted whitelist hits per file, so Plan 01-03's permanent gate has the correct shape:

| Whitelist entry | Hit count | Reason |
| --- | --- | --- |
| `^.planning/` | many | Planning artefacts (CONTEXT, RESEARCH, etc.) |
| `^README.md` | ~18 | Alias note + boot-service refs + `hailo-drone-follow` repo name |
| `^CLAUDE.md` | ~21 | Alias note + memory-index title "Drone-Follow Memory Index" + skill dir name |
| `^TROUBLESHOOTING.md` | ~34 | Boot-service refs + `drone-follow-boot.service` mentions + legacy paths |
| `^docs/superpowers/plans/` | 213 | Historical plan documents (per CONTEXT Q5) |
| `^scripts/boot/` | many | RENAME-04: preserved verbatim |
| `^.claude/skills/drone-follow-dev/` | 13 | Skill dir name preserved (out of scope) |
| `^.claude/skills/safe-pull-and-rollback/` | 5 | Historical branch-name references (per plan Step 5) |
| `^.claude/memory/openhd_pairing.md` | 2 | Historical path examples |
| `^.claude/memory/MEMORY.md` | 1 | Index title "Drone-Follow Memory Index" |
| `^system/` | 2 | User-level systemd unit name (per planner Q1) |
| `^MILESTONES.md` | none observed in this rewrite | (whitelisted defensively) |
| `^SETUP_GUIDE.md` | 3 | Historical in-repo-path references (hailo-apps-infra co-located layout) |
| `^robot_follow/tests/test_install_smoke.py` | 8 | Test names + docstrings asserting old-import-raises contract |
| `hailo_drone_follow` substring | several | Legacy in-repo dir name; distinct from package name (substring match issue) |

## 7-step quick-suite results (post-commit, all green)

1. **Flush stale bytecode** — done; no `__pycache__/` or `*.pyc` outside submodules.
2. **New import path works** — `python -c 'import robot_follow; import robot_follow.follow_api.*'` exits 0.
3. **Old import path raises** — `python -c 'import drone_follow'` exits non-zero with `ModuleNotFoundError`.
4. **Both console scripts + byte-identical --help** — `command -v` resolves both; `diff <(robot-follow --help) <(drone-follow --help)` empty.
5. **pip metadata exclusive** — `pip show robot-follow` exits 0; `pip show drone-follow` exits non-zero.
6. **Grep gate** — `/tmp/phase1_grep_gate.sh` returns "GREP GATE PASSED".
7. **Smoke test** — `pytest robot_follow/tests/test_install_smoke.py -v` → **10 passed, 0 skipped, 0 failed** in 2.06s.

Additional Plan-checklist items (Task 3 verify):
8. **Bash syntax check** — `bash -n install.sh && bash -n scripts/install_air.sh && bash -n scripts/start_air.sh` clean.
9. **Boot service preservation** — `git diff scripts/boot/ system/` returns nothing (0 chars).

## User Setup Required

None - no external service configuration required.

The pip metadata transition (`drone-follow` distribution -> `robot-follow` distribution) is handled by the idempotent `pip uninstall drone-follow -y` step in `install.sh`. Deployed units that re-run `./install.sh` will transition cleanly.

For developers with stale local installs: the runtime fix used during this plan (`rm -rf drone_follow.egg-info` at the repo root, then `pip uninstall drone-follow -y && pip install -e .`) is documented in `/tmp/plan_01_02_start.txt` for reference if anyone hits the "outside environment" pip warning. Plan 01-03 may surface this in TROUBLESHOOTING.md if reports come in.

## Next Phase Readiness

**Ready for 01-03 (rename followups).** Plan 01-03 (if planned that way) should cover:
- Permanent grep-gate script committed to the repo (currently lives at `/tmp/phase1_grep_gate.sh`).
- Optional: deferred items from RESEARCH Q3/Q4 (`drone_follow/ui/package.json` "name" field — currently still says `"drone-follow-ui"` since it's npm-internal-only; `drone_follow/ui/index.html` `<title>Drone Follow UI</title>` — user-visible browser tab title).
- Manual dev-box re-run of `./install.sh --skip-apps --skip-hefs --skip-ui` (success criterion verification per VALIDATION.md manual tier).

**Ready for Phase 2 (cleanup).** With the package name now reflecting the abstraction goal, Phase 2 can delete `scripts/bench_reid_callback.py` (CLEAN-02 dead code) and other identified dead surfaces without naming-confusion friction.

**Ready for Phase 3 (abstraction).** `robot_follow/drone_api/mavsdk_drone.py` will move to `robot_follow/robot_api/adapters/mavsdk_drone.py` in ABS-03; `run_drone()` function -> `run_robot()` in ABS-08; `run_drone.sh`, `df_params.json` filenames renamed in Phase 3. None of these are blocked.

### Self-Check

- File `robot_follow/` exists; `drone_follow/` does not. ✓
- File `robot_follow/robot_follow_app.py` exists; `drone_follow_app.py` does not. ✓
- Commit `5850558` exists on `feature/rover-support` (`git log --oneline -3` shows it as HEAD). ✓
- `git log --follow robot_follow/follow_api/config.py` shows pre-rename history (oldest commits prefix the rename). ✓
- pyproject.toml: name=robot-follow, version=1.1.0.dev0, both console scripts present. ✓
- install.sh: `pip uninstall drone-follow -y` line present at line 95. ✓
- scripts/install_air.sh: chown paths reference `robot_follow/ui/{node_modules,build}`. ✓
- `git diff scripts/boot/ system/`: empty (RENAME-04 + planner Q1). ✓
- pytest robot_follow/tests/test_install_smoke.py: 10 passed, 0 skipped, 0 failed. ✓
- Grep gate (whitelist-filtered): zero unexpected hits. ✓
- Both console scripts on PATH; --help byte-identical. ✓
- pip show robot-follow: OK; pip show drone-follow: WARNING (not found). ✓

## Self-Check: PASSED

---
*Phase: 01-rename*
*Completed: 2026-05-14*
