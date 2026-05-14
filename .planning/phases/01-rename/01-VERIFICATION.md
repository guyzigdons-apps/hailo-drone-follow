---
phase: 01-rename
verified: 2026-05-14T16:30:00Z
status: passed
score: 4/4 must-haves verified
requirements_verified: [RENAME-01, RENAME-02, RENAME-03, RENAME-04, RENAME-05]
deferred:
  - id: VerificationB
    item: "Boot-service ExecStart path on a deployed RPi (RENAME-04)"
    reason: "No air-unit RPi reachable from dev box"
    mitigation: "git diff 5850558 -- scripts/boot/ is empty (boot service mechanically untouched by rename commit); ExecStart=/usr/local/bin/drone-follow-boot.sh is unchanged"
    follow_up: "Smoke step on next field deployment / RPi sync"
  - id: VerificationC
    item: "Full pytest suite on a Hailo-capable host"
    reason: "No Hailo-8 PCIe on x86_64 dev box"
    mitigation: "Plan 01-02 Wave-1 smoke gate ran pytest robot_follow/tests/test_install_smoke.py -v with 10/10 PASS covering all RENAME-01..05 testable surface"
    follow_up: "Run pytest robot_follow/tests/ -x --ignore=robot_follow/tests/test_sim_worlds.py on next Hailo-equipped sync"
---

# Phase 1: Rename Verification Report

**Phase Goal:** The package is `robot_follow`; `drone-follow` and `robot-follow` both work; no field deployment breaks.
**Verified:** 2026-05-14T16:30:00Z
**Status:** passed
**Re-verification:** No — initial verification
**Branch:** `feature/rover-support` (commits `0869454` → `5850558` → `bee6db3`)

## Goal Achievement

### Observable Truths (Must-Haves)

| # | Truth | Status | Evidence |
|---|-------|--------|----------|
| 1 | `pip show robot_follow` shows the renamed package; `pip show drone_follow` returns nothing. | ✓ VERIFIED | `pip show robot-follow` → exit 0, `Name: robot-follow / Version: 1.1.0.dev0 / Editable project location: /home/guyz/code/guyz/hailo-drone-follow`. `pip show drone-follow` → exit 1, `WARNING: Package(s) not found: drone-follow`. |
| 2 | `drone-follow --help` and `robot-follow --help` produce identical output (same `main()` entry point). | ✓ VERIFIED | Both shims resolve to `venv_hailo_apps/bin/`. Both shims contain `from robot_follow.robot_follow_app import main`. `diff <(robot-follow --help 2>&1) <(drone-follow --help 2>&1)` → exit 0, empty diff. `parser.prog = "robot-follow"` pin at `robot_follow/robot_follow_app.py:175` makes this byte-identical (necessary auto-fix in 01-02, otherwise argparse derives prog from `sys.argv[0]` and diverges). |
| 3 | `scripts/start_air.sh` on a fresh checkout runs without path errors; boot service unit file is unchanged on disk. | ✓ VERIFIED | `bash -n scripts/start_air.sh` → exit 0. `git diff 5850558 -- scripts/boot/` returns 0 bytes (boot directory byte-identical to atomic rename commit). `scripts/boot/drone-follow-boot.service` exists with `ExecStart=/usr/local/bin/drone-follow-boot.sh`; `scripts/boot/drone-follow-boot.sh` retains `LOG_TAG="drone-follow-boot"` and reads `~/Desktop/drone-follow.conf`. |
| 4 | All documentation examples using `drone-follow` still work via the alias; no internal import of `drone_follow` remains in the source tree. | ✓ VERIFIED | Grep gate (whitelist-filtered: `.planning/`, `README.md`, `CLAUDE.md`, `TROUBLESHOOTING.md`, `docs/superpowers/plans/`, `scripts/boot/`, `.claude/skills/drone-follow-dev/`, `system/`, `MILESTONES.md`, `.claude/memory/openhd_pairing.md`, `.claude/memory/MEMORY.md`, `.claude/skills/safe-pull-and-rollback/`, `SETUP_GUIDE.md`, `robot_follow/tests/test_install_smoke.py`, substring `hailo_drone_follow`) returns 0 hits. `grep -rn "from drone_follow\|import drone_follow" --include="*.py"` returns 0 hits anywhere. README alias note at L9, CLAUDE.md alias section after Project Overview, both `--help` outputs byte-identical (Truth 2). |

**Score:** 4/4 truths verified.

### Required Artifacts

| Artifact | Expected | Exists | Substantive | Wired | Status |
|----------|----------|--------|-------------|-------|--------|
| `robot_follow/` | Renamed package directory | ✓ | ✓ (full tree present) | ✓ (`import robot_follow` succeeds) | ✓ VERIFIED |
| `robot_follow/robot_follow_app.py` | Main entry point with `def main` | ✓ | ✓ (contains `def main`, `parser.prog="robot-follow"` pin at L175) | ✓ (both console scripts import `robot_follow.robot_follow_app:main`) | ✓ VERIFIED |
| `drone_follow/` (must be absent) | Old directory removed | ✓ absent | n/a | n/a | ✓ VERIFIED |
| `pyproject.toml` | `name="robot-follow"`, `version="1.1.0.dev0"`, both console scripts | ✓ | ✓ (`name = "robot-follow"`, `version = "1.1.0.dev0"`, `[project.scripts]` declares `robot-follow` and `drone-follow` both → `robot_follow.robot_follow_app:main`) | ✓ (pip metadata reflects spec; shims regenerated) | ✓ VERIFIED |
| `install.sh` | Idempotent `pip uninstall drone-follow -y` before reinstall | ✓ | ✓ (line 99: `pip uninstall drone-follow -y >/dev/null 2>&1 \|\| true`; context comment at L12) | ✓ (`bash -n` clean; operator-witnessed re-run in 01-03 Verification A) | ✓ VERIFIED |
| `robot_follow/tests/test_install_smoke.py` | 10/10 PASS in strict mode | ✓ | ✓ (10 tests, skip-guards stripped in 01-02) | ✓ (`python -m pytest robot_follow/tests/test_install_smoke.py -v` → 10 passed in 1.98s — see note about pytest invocation below) | ✓ VERIFIED |
| `scripts/boot/drone-follow-boot.service` | Unit file unchanged (RENAME-04) | ✓ | ✓ (`ExecStart=/usr/local/bin/drone-follow-boot.sh`) | ✓ (boot helper still references `drone-follow.conf`) | ✓ VERIFIED |

### Key Link Verification

| From | To | Via | Status | Details |
|------|-----|-----|--------|---------|
| `bin/robot-follow` console-script shim | `robot_follow.robot_follow_app:main` | `[project.scripts]` in pyproject.toml + `pip install -e .` | ✓ WIRED | Shim head contains `from robot_follow.robot_follow_app import main` |
| `bin/drone-follow` console-script alias | `robot_follow.robot_follow_app:main` | Same `[project.scripts]` entry under aliased name | ✓ WIRED | Shim head also contains `from robot_follow.robot_follow_app import main`; alias regenerated cleanly (no stale `from drone_follow.drone_follow_app` import). |
| All `.py` files under `robot_follow/` | `robot_follow.*` import paths | sed-rewritten import statements | ✓ WIRED | 48 internal imports rewritten across 19 files in 01-02; `git grep 'from drone_follow' -- '*.py'` = 0; `python -c 'import robot_follow.follow_api.{config,controller,state,types}'` succeeds. |
| `install.sh` | `robot_follow/ui` build location | `pushd $SCRIPT_DIR/robot_follow/ui` | ✓ WIRED | `robot_follow/ui/` exists; install.sh paths updated in 01-02. |
| `scripts/install_air.sh` | `${APP_ROOT}/robot_follow/ui/{node_modules,build}` | chown paths | ✓ WIRED | `bash -n scripts/install_air.sh` exits 0; paths reference renamed directory. |
| Boot service `ExecStart` | `/usr/local/bin/drone-follow-boot.sh` | systemd unit file | ✓ PRESERVED (per RENAME-04 verbatim contract) | File byte-identical to pre-rename (`git diff 5850558 -- scripts/boot/` = 0 bytes). |

### Requirements Coverage

| Requirement | Source Plan(s) | Description | Status | Evidence |
|-------------|----------------|-------------|--------|----------|
| RENAME-01 | 01-01, 01-02 | Package directory `drone_follow/` renamed to `robot_follow/`; all internal imports updated to `from robot_follow.*` | ✓ SATISFIED | `drone_follow/` absent; `robot_follow/` present; `git grep 'from drone_follow\|import drone_follow' -- '*.py'` returns 0; `import robot_follow.*` succeeds, `import drone_follow` raises `ModuleNotFoundError`. |
| RENAME-02 | 01-01, 01-02 | `pyproject.toml` package name is `robot_follow`; console scripts `robot-follow` (primary) and `drone-follow` (alias) | ✓ SATISFIED | `name = "robot-follow"`, both `[project.scripts]` entries map to `robot_follow.robot_follow_app:main`; both shims on PATH with byte-identical `--help`. |
| RENAME-03 | 01-02, 01-03 | `setup_env.sh`, `install.sh`, `scripts/start_air.sh` updated; existing functionality preserved | ✓ SATISFIED | `bash -n` clean on all three; `install.sh:99` has idempotent `pip uninstall drone-follow -y`; UI paths reference `robot_follow/ui`; Verification A in 01-03 operator-confirmed `./install.sh --skip-apps --skip-hefs --skip-ui` re-runs cleanly with banner `==> Removing any prior 'drone-follow' distribution (legacy name)` present. |
| RENAME-04 | 01-02, 01-03 | `drone-follow-boot.service` systemd unit + `~/Desktop/drone-follow.conf` preserved (so existing field deployments keep working); only underlying binary path changes | ✓ SATISFIED (deployment-side observation DEFERRED with mitigation) | `git diff 5850558 -- scripts/boot/` = 0 bytes; `ExecStart=/usr/local/bin/drone-follow-boot.sh` unchanged; `drone-follow-boot.sh` still references `~/Desktop/drone-follow.conf` and `LOG_TAG="drone-follow-boot"`. The underlying binary path change is mediated by the `drone-follow` console-script alias — same name on disk, new import target. **Deferred: live RPi systemctl status check** (no air unit reachable); mitigated by byte-identical-on-disk evidence. |
| RENAME-05 | 01-02 | README, CLAUDE.md, TROUBLESHOOTING.md, docs/*.md, .claude/memory/*.md updated (drone-follow examples still valid via alias) | ✓ SATISFIED | README L9 alias note added; CLAUDE.md alias section added after Project Overview; Tier-1 docs (TROUBLESHOOTING, PARAMETERS, SETUP_GUIDE, RESOLUTION_CONTROL, TEST_PLAN) rewritten; Tier-2 docs (tracking-reid-algorithm, calibration-flight-guide, control-architecture, design-review) rewritten; `.claude/memory/{tracking_callback_risks,webui_build}.md` updated; `.claude/skills/drone-follow-dev/SKILL.md` L76 path ref updated. README examples switched to `robot-follow` invocations; `drone-follow` invocations remain valid through alias (Truth 2). |

**All 5 RENAME requirements: SATISFIED.** Zero orphaned requirements (REQUIREMENTS.md maps RENAME-01..05 to Phase 1; PLAN frontmatter across 01-01..03 collectively declares all 5).

### Anti-Patterns Found

None of severity 🛑 Blocker. One ℹ️ Info-level observation surfaced during verification:

| File | Pattern | Severity | Impact |
|------|---------|----------|--------|
| `~/.local/bin/pytest` (host environment, not repo file) | System pytest at `/usr/bin/python3` resolves `sys.executable` to system python; smoke test's `subprocess.run([sys.executable, '-m', 'pip', 'show', 'robot-follow'])` then queries the system pip (not venv pip) and reports "not found", failing `test_pip_show_robot_follow_succeeds`. | ℹ️ Info (test-runner-environment artifact, not a code defect) | When operator runs the smoke test, MUST invoke via `python -m pytest robot_follow/tests/test_install_smoke.py -v` (venv python), not bare `pytest` (which resolves to `~/.local/bin/pytest` with system-python shebang). With the correct invocation, 10/10 PASS in 1.98s. This is exactly the failure mode the 01-01 SUMMARY flagged ("`sys.executable -m pip show` over bare `pip`" decision); the test is correct, but the test-runner invocation must use the venv interpreter. |

### Deviations from Plan (Documented)

All 3 from 01-02 auto-fix log were necessary load-bearing fixes, not scope creep:

1. **`parser.prog="robot-follow"` pin** (auto-fix, robot_follow/robot_follow_app.py:175) — Without this, success criterion 2 ("byte-identical `--help`") was technically unachievable; argparse derives prog from `sys.argv[0]`, so the two console-script invocations diverged on the `usage:` line. Pin makes both invocations indistinguishable. Confirmed at L175.
2. **Stray repo-root `drone_follow.egg-info/` removed at runtime** (gitignored; no source-tree impact) — Stale egg-info from a pre-venv install made `pip uninstall drone-follow -y` no-op with "outside environment" warning. One-time cleanup; idempotency of install.sh confirmed by operator in 01-03 Verification A.
3. **Backticks escaped in install.sh banner echo** — Command-substitution hazard fixed pre-commit; `bash -n install.sh` clean.

### Deferred Items (with Mitigations)

**Verification B — Boot-service on deployed RPi (RENAME-04 deployment side):**
- Deferred for: no air-unit RPi reachable from dev box at verification time.
- Mitigation: `git diff 5850558 -- scripts/boot/` returns 0 bytes — boot service unit file mechanically unchanged by the rename commit. `ExecStart=/usr/local/bin/drone-follow-boot.sh` is byte-identical; `drone-follow-boot.sh` still reads `~/Desktop/drone-follow.conf`. The `drone-follow` console-script alias (verified-wired, Truth 2) means the underlying invocation chain still terminates at the same Python entry point. Existing field deployments re-running `./install.sh` will see the legacy-uninstall banner and clean transition.
- Follow-up: Next field deployment / RPi sync, run `systemctl status drone-follow-boot && grep ExecStart /etc/systemd/system/drone-follow-boot.service && cat ~/Desktop/drone-follow.conf`. Expected: unit loaded, ExecStart unchanged, config present.

**Verification C — Full pytest suite on a Hailo-capable host:**
- Deferred for: no Hailo-8 PCIe on x86_64 dev box; `hailortcli fw-control identify` not available.
- Mitigation: 01-02 Wave-1 verification gate ran `pytest robot_follow/tests/test_install_smoke.py` with 10/10 PASS (the canonical Phase 1 smoke contract, covering import-path negative-assertion, console-script alias byte-identity, and pip-metadata exclusivity — all RENAME-01..05 surface that is testable without Hailo HW). The non-smoke pytest paths exercise tracking / pipeline / sim modules whose names/paths were mechanically updated and have no Phase-1-specific contract.
- Follow-up: Next Hailo-host sync, run `pytest robot_follow/tests/ -x --ignore=robot_follow/tests/test_sim_worlds.py`. Expected: all PASSED.

### Human Verification Required

None required for Phase 1 sign-off. All goal-essential checks are either VERIFIED programmatically here or DEFERRED with explicit mitigations (see above). The deferred items are field-deployment regression smoke steps, not Phase 1 blockers.

### Gaps Summary

**No gaps.** All 4 ROADMAP success criteria verified; all 5 RENAME requirements satisfied; the atomic rename commit `5850558` preserves history (git mv), preserves boot service verbatim (`scripts/boot/` diff = 0), preserves the `drone-follow` invocation contract (alias regenerated with correct `from robot_follow.robot_follow_app import main`), and the smoke test gate passes 10/10 under the correct (venv) interpreter. The two deferred verifications (B: deployed RPi; C: full Hailo-host pytest) are scope-out by hardware/deployment unavailability and have on-disk mitigations that cover the same risk surface.

**Verdict:** Phase 1 goal achieved. Ready for Phase 2 (Cleanup).

---

_Verified: 2026-05-14T16:30:00Z_
_Verifier: Claude (gsd-verifier, opus-4-7)_
