---
phase: 1
slug: rename
status: draft
nyquist_compliant: false
wave_0_complete: false
created: 2026-05-14
---

# Phase 1 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution. Derived from `01-RESEARCH.md` § Verification Architecture.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | pytest 9.0.2 (system: `/home/guyz/.local/bin/pytest`, not in venv) |
| **Config file** | none — default discovery via `drone_follow/tests/conftest.py` (path-relative; survives rename) |
| **Quick run command** | `pytest robot_follow/tests/test_install_smoke.py -x` |
| **Full suite command** | `pytest robot_follow/tests/ -x --ignore=robot_follow/tests/test_sim_worlds.py` |
| **Estimated runtime** | ~15 s quick · ~60 s full (excluding sim) |

`test_install_smoke.py` already exists and currently tests the old name; Wave 0 rewrites it as the canonical Phase 1 gate.

---

## Sampling Rate

- **After every task commit** (pre-commit, no Hailo HW): the 7-step quick suite below — `< 30 s`.
- **After every plan wave** (pre-push, Hailo-capable host): full pytest suite excluding `test_sim_worlds.py`.
- **Before `/gsd:verify-work`**: full suite green + manual `scripts/start_air.sh` syntax-check + `./install.sh --skip-apps --skip-hefs --skip-ui` clean re-run on a dev box.
- **Max feedback latency:** 30 seconds (quick suite).

### Quick suite (pre-commit)

```bash
# 1. Flush stale bytecode (rename's #1 footgun)
find . -name __pycache__ -type d -prune -exec rm -rf {} +
find . -name '*.pyc' -delete

# 2. New import path works (positive assert)
python -c 'import robot_follow; import robot_follow.follow_api.config; \
           import robot_follow.follow_api.controller; \
           import robot_follow.follow_api.state; \
           import robot_follow.follow_api.types'

# 3. Old import path raises (negative assert)
! python -c 'import drone_follow' 2>/dev/null

# 4. Both console scripts on PATH, byte-identical --help
command -v robot-follow && command -v drone-follow
diff <(robot-follow --help 2>&1) <(drone-follow --help 2>&1)

# 5. pip metadata exclusive
pip show robot-follow >/dev/null
! pip show drone-follow >/dev/null 2>&1

# 6. Grep gate (whitelist-limited)
test -z "$(git grep -nE 'drone_follow|from drone_follow|import drone_follow' \
         | grep -vE '^\.planning/|^README\.md|^CLAUDE\.md.*alias')"

# 7. Smoke test
pytest robot_follow/tests/test_install_smoke.py -x
```

---

## Per-Task Verification Map

| Task ID | Plan | Wave | Requirement | Test Type | Automated Command | File Exists | Status |
|---------|------|------|-------------|-----------|-------------------|-------------|--------|
| 1-01-W0 | 01 | 0 | RENAME-01..05 | wave-0 | rewrite `test_install_smoke.py` (see Wave 0 below) | ❌ rewrite needed | ⬜ pending |
| 1-01-01 | 01 | 1 | RENAME-01 | unit + grep | `python -c 'import robot_follow.*'` + `! git grep -nE 'from drone_follow\|import drone_follow' -- '*.py'` | ✅ after W0 | ⬜ pending |
| 1-01-02 | 01 | 1 | RENAME-01 | negative unit | `! python -c 'import drone_follow' 2>/dev/null` | ✅ after W0 | ⬜ pending |
| 1-01-03 | 01 | 1 | RENAME-02 | smoke | `pip show robot-follow >/dev/null && ! pip show drone-follow >/dev/null 2>&1` | ✅ after W0 | ⬜ pending |
| 1-01-04 | 01 | 1 | RENAME-02 | smoke | `command -v robot-follow && command -v drone-follow` | ✅ after W0 | ⬜ pending |
| 1-01-05 | 01 | 1 | RENAME-02 | smoke | `diff <(robot-follow --help) <(drone-follow --help)` | ✅ after W0 | ⬜ pending |
| 1-01-06 | 01 | 1 | RENAME-03 | smoke | `bash -c 'source setup_env.sh && python -c "import robot_follow"'` | shell-only | ⬜ pending |
| 1-01-07 | 01 | 1 | RENAME-03 | syntax | `bash -n scripts/start_air.sh && bash -n scripts/install_air.sh` + path check on `${APP_ROOT}/robot_follow/ui/build` | shell-only | ⬜ pending |
| 1-01-08 | 01 | 1 | RENAME-04 | grep | `test -f scripts/boot/drone-follow-boot.service && grep -q 'drone-follow-boot' scripts/boot/drone-follow-boot.service` | shell-only | ⬜ pending |
| 1-01-09 | 01 | 1 | RENAME-04 | grep | `grep -q 'drone-follow.conf' scripts/boot/install.sh` | shell-only | ⬜ pending |
| 1-01-10 | 01 | 1 | RENAME-05 | grep gate | `test -z "$(git grep -nE 'drone_follow' \| grep -vE '<whitelist>')"` | shell-only — pre-commit hook | ⬜ pending |
| 1-01-11 | 01 | 2 | RENAME-03 | manual | `./install.sh --skip-apps --skip-hefs --skip-ui` re-runs cleanly on a dev box | manual | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

Plan numbering (`01-01-*`) provisional — the planner may split into multiple plans/waves; this table is intent, not implementation.

---

## Wave 0 Requirements

Wave 0 lands the test-infrastructure changes **before** the rename commit so they're available as the gate during execution. The rename commit itself is Wave 1.

- [ ] **Rewrite `robot_follow/tests/test_install_smoke.py`** to:
  - assert `import robot_follow` works (replaces current `import drone_follow` test)
  - assert all `robot_follow.follow_api.*` submodules import (`config`, `controller`, `state`, `types`)
  - **NEW**: assert `import drone_follow` raises `ModuleNotFoundError`
  - **NEW**: assert both `shutil.which('robot-follow')` and `shutil.which('drone-follow')` resolve to a file
  - **NEW**: assert `robot-follow --help` exits 0 AND `drone-follow --help` exits 0
  - **NEW**: assert byte-identical `--help` output (`subprocess.run` each, compare `stdout + stderr`)
  - **NEW**: assert `pip show robot-follow` succeeds AND `pip show drone-follow` fails (via `subprocess.run`)
- [ ] **No new conftest/fixtures needed** — existing `conftest.py` adds repo root to `sys.path`; rename does not break this (the `..` traversal lands at repo root regardless of package name).
- [ ] **No new test framework install needed** — pytest 9.0.2 already on PATH.

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| `setup_env.sh` clean source | RENAME-03 | Sources venv + sets env — shell-only, not pytestable in-process | `bash -c 'source setup_env.sh && python -c "import robot_follow"'` returns 0 |
| `scripts/start_air.sh` runs without path errors | RENAME-03 | Needs OpenHD + camera; only path-resolution is verifiable headlessly | `bash -n scripts/start_air.sh` + `test -d robot_follow/ui/build` (post-install) |
| `./install.sh` idempotent re-run | RENAME-03 | Touches `/usr/local/hailo/resources/` + the submodule installer; cannot run in CI | One-time dev-box re-run with `--skip-apps --skip-hefs --skip-ui` |
| Boot service file unchanged on disk | RENAME-04 | The deployed `/etc/systemd/system/drone-follow-boot.service` is per-host state | `systemctl status drone-follow-boot` shows the unit; verify `ExecStart` path on a target host post-deploy |

---

## Validation Sign-Off

- [ ] All RENAME-01..05 requirements have an automated `<automated>` verify or a documented manual step
- [ ] Sampling continuity: no 3 consecutive tasks without automated verify (quick suite covers steps 2–6 inline)
- [ ] Wave 0 (rewrite `test_install_smoke.py`) covers all MISSING references in the per-task map
- [ ] No watch-mode flags (pytest invoked with `-x` for fail-fast)
- [ ] Feedback latency `< 30s` (quick suite)
- [ ] Grep-gate whitelist documented and enforceable as a one-liner
- [ ] `nyquist_compliant: true` set in frontmatter (after planner reviews & signs off)

**Approval:** pending
