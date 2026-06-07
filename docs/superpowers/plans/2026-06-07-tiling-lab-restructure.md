# Tiling Lab Restructure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Separate benchmark + GT-generation code into an installed `tiling_lab` package, promote the scheduler into `hailo_tiling`, purge ~46 GB of regenerable artifacts, and enforce the dependency rules with a test.

**Architecture:** Per `docs/superpowers/specs/2026-06-07-tiling-lab-restructure-design.md`. Dependency rules: `hailo_tiling` imports nothing internal; `drone_follow` → `hailo_tiling` only; `tiling_lab` → `drone_follow` + `hailo_tiling`; nothing → `tiling_lab`/`tiling_benchmark`.

**Tech Stack:** Python 3.10, pytest, setuptools (pyproject), git mv.

**Execution rules (every task):**
- venv binary, always: `/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo-apps/venv_hailo_apps/bin/python -m pytest -q` (never bare `pytest`; never pipe pytest through `tail` without `set -o pipefail`).
- Commit = ONE Bash invocation: `git add <explicit paths> && git commit -m "..."`. Never `git add .`/`-A`.
- Branch `tiling-benchmark` only. NEVER push. Never touch `drone_follow/pipeline_adapter/reid_manager.py` (pre-existing dirty), submodule pointers (`hailo-apps`, `sim/PX4-Autopilot`), `.venv_gt/`.
- Suite must be green (vs the baseline recorded in Task 0) before every commit.

---

### Task 0: Record test baseline + purge regenerable artifacts

**Files:**
- Delete: `tiling_benchmark/pxt_runs/.cache/` (45.2 GB), `tiling_benchmark/pxt_runs/scaled_sources/` (221 MB), `tiling_benchmark/pxt_runs/zoom_*` (PNG/CSV duplicates), `tiling_benchmark/hailort.log`, `tiling_benchmark/hailort.1.log`, `tiling_benchmark/__pycache__/`
- All deletions are UNTRACKED artifacts — no git changes, no commit in this task.

- [ ] **Step 1: Record baseline test count**

Run: `/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/ hailo_tiling/tests/ -q 2>&1 | tail -2`
Recorded 2026-06-07 by the manager: **436 passed, 1 skipped** — this is the floor for every later task.

- [ ] **Step 2: Verify the keep-list exists before deleting**

```bash
ls tiling_benchmark/pxt_runs/pxt_GT-12x9-25-multi.frames.json \
   tiling_benchmark/pxt_runs_clean/pxt_GT-12x9-25-multi.clean.frames.json \
   tiling_benchmark/PERF_REPORT.md tiling_benchmark/perf_assets/zoom_probe.png
```
Expected: all four paths listed. ABORT the purge if any is missing.

- [ ] **Step 3: Purge (untracked artifacts only)**

```bash
rm -rf tiling_benchmark/pxt_runs/.cache \
       tiling_benchmark/pxt_runs/scaled_sources \
       tiling_benchmark/pxt_runs/zoom_probe*.png tiling_benchmark/pxt_runs/zoom_probe*.csv \
       tiling_benchmark/pxt_runs/zoom_compare*.png \
       tiling_benchmark/hailort.log tiling_benchmark/hailort.1.log \
       tiling_benchmark/__pycache__
du -sh tiling_benchmark   # expected: ~80 MB (was 46 GB)
git status --short tiling_benchmark/   # expected: NO output (nothing tracked touched)
```

---

### Task 1: `hailo_tiling/geometry.py` — absorb the two legacy helpers

**Files:**
- Create: `hailo_tiling/geometry.py`
- Modify: `hailo_tiling/bench/grid.py` (drop `tiling_benchmark` import + its sys.path shim), `scripts/warm_gst_cache.py`, `hailo_tiling/tests/test_bench_grid.py` (and any other test importing `_grid_to_static_tiles` from legacy — find with `grep -rl "tiling_record" hailo_tiling/ scripts/`)
- Test: `hailo_tiling/tests/test_geometry.py`

- [ ] **Step 1: Write the failing test**

```python
"""Tests for hailo_tiling.geometry — helpers absorbed from tiling_benchmark."""
import pytest

from hailo_tiling.geometry import grid_to_static_tiles, fov_to_crop_dims


def test_grid_to_static_tiles_matches_legacy():
    from tiling_benchmark.tiling_record import _grid_to_static_tiles as legacy
    assert grid_to_static_tiles(3, 2, 0.25, 0.25) == legacy(3, 2, 0.25, 0.25)


def test_grid_to_static_tiles_no_overlap_covers_unit_square():
    tiles = grid_to_static_tiles(2, 2, 0.0, 0.0)
    assert len(tiles) == 4
    xs = sorted({t[0] for t in tiles})
    assert xs == [0.0, 0.5]


def test_fov_to_crop_dims_matches_legacy():
    from tiling_benchmark.prepare_video import fov_to_crop_dims as legacy
    assert fov_to_crop_dims(50, 3840, 2160) == legacy(50, 3840, 2160)
```

(Adjust the legacy-function signatures to the actual ones in `tiling_benchmark/tiling_record.py` and `tiling_benchmark/prepare_video.py` — read them first; the parity tests are the spec.)

- [ ] **Step 2: Run to verify failure** — `ModuleNotFoundError`/`ImportError` expected.
- [ ] **Step 3: Implement** — copy both function bodies verbatim into `hailo_tiling/geometry.py` (public name `grid_to_static_tiles`, docstring noting provenance commit `7d9a8d9`). Do NOT edit the frozen `tiling_benchmark` originals.
- [ ] **Step 4: Rewire consumers** — `hailo_tiling/bench/grid.py` (also delete its repo-root sys.path shim), `scripts/warm_gst_cache.py`, affected hailo_tiling tests: import from `hailo_tiling.geometry`.
- [ ] **Step 5: Full suite green** (baseline + 3 new).
- [ ] **Step 6: Commit** — `git add hailo_tiling/geometry.py hailo_tiling/bench/grid.py scripts/warm_gst_cache.py hailo_tiling/tests/test_geometry.py <other touched tests> && git commit -m "refactor(tiling): absorb grid/fov geometry helpers into hailo_tiling.geometry"`

---

### Task 2: Promote scheduler → `hailo_tiling/dynamic/`

**Files:**
- Create: `hailo_tiling/dynamic/__init__.py`
- Move: `dynamic_tiling/scheduler.py` → `hailo_tiling/dynamic/scheduler.py` (git mv)
- Recreate: `dynamic_tiling/scheduler.py` as a temporary re-export shim (dies in Task 3)
- Modify: `hailo_tiling/types.py` IF needed (see Step 2)

- [ ] **Step 1: git mv**

```bash
mkdir -p hailo_tiling/dynamic && git mv dynamic_tiling/scheduler.py hailo_tiling/dynamic/scheduler.py
```

- [ ] **Step 2: Fix its single import.** The file's only import is `from .types import CropRect, LockState, TargetState, ScheduledTile, MODEL_W, MODEL_H, MODEL_ASPECT`. Check which of those names exist in `hailo_tiling/types.py` vs only in the `dynamic_tiling/types.py` shim. Any name defined ONLY in the shim (likely `LockState`/`TargetState`) MOVES into `hailo_tiling/types.py`. Then the import becomes `from ..types import ...`.

- [ ] **Step 3: Write `hailo_tiling/dynamic/__init__.py`:**

```python
"""Dynamic tile scheduling — discovery grid / ROI follow / recovery."""
from .scheduler import TileScheduler, MultiTargetTileScheduler

__all__ = ["TileScheduler", "MultiTargetTileScheduler"]
```

- [ ] **Step 4: Temporary back-compat shim** at `dynamic_tiling/scheduler.py` so the not-yet-moved harness keeps working:

```python
"""Temporary shim — scheduler promoted to hailo_tiling.dynamic (removed in Task 3)."""
from hailo_tiling.dynamic.scheduler import *  # noqa: F401,F403
from hailo_tiling.dynamic.scheduler import TileScheduler, MultiTargetTileScheduler  # noqa: F401
```

- [ ] **Step 5: Full suite green** (scheduler tests still in dynamic_tiling/tests pass through the shim).
- [ ] **Step 6: Commit** — `git add hailo_tiling/dynamic/ dynamic_tiling/scheduler.py hailo_tiling/types.py && git commit -m "refactor(tiling): promote dynamic TileScheduler into hailo_tiling.dynamic"`

---

### Task 3: The big move — `dynamic_tiling/` → `tiling_lab/`

One atomic commit: skeleton + git mv + import rewrite + packaging. No shims survive.

**Files:**
- Create: `tiling_lab/__init__.py`, `tiling_lab/{harness,reid,gt,cli,viewer,video}/__init__.py` (each a one-line docstring package init)
- git mv (exact mapping):

| From `dynamic_tiling/` | To |
|---|---|
| `replay.py target_lock.py inference.py trials.py score.py metrics.py mot_metrics.py compare_baselines.py aggregator.py` | `tiling_lab/harness/` |
| `reid_embedder.py reid_gallery.py reid_policy.py` | `tiling_lab/reid/` |
| `gt_track.py gt_clean.py gt_dedup.py gt_edit.py gt_mot.py gt_review.py gt_render_review.py gt_review_gui.py` + all 15 `run_gt_*.py` | `tiling_lab/gt/` |
| `run_dynamic.py run_trials.py run_sweep.py run_reid_ablation.py run_mot_eval.py` | `tiling_lab/cli/` |
| `tests/` (all 34 files + conftest) | `tiling_lab/tests/` |
| — `tiling_benchmark/overlay_viewer.py` (git mv) + `scripts/adapt_frames_for_viewer.py` (git mv) | `tiling_lab/viewer/` |
| — `tiling_benchmark/prepare_video.py` (git mv) | `tiling_lab/video/` |

- Delete (git rm): `dynamic_tiling/budget.py`, `dynamic_tiling/types.py`, `dynamic_tiling/scheduler.py` (Task-2 shim), `dynamic_tiling/_vendor_paths.py`, `dynamic_tiling/__init__.py` — directory gone except `dynamic_tiling/runs/` (handled in Task 4).
- Modify: `pyproject.toml`, `pytest.ini`, `scripts/warm_dynamic_cache.py`

- [ ] **Step 1: skeleton + git mv per the table** (single Bash invocation per destination group is fine; verify with `git status --short | head -60` that everything is `R`-status).

- [ ] **Step 2: Import rewrite sweep.** Over `tiling_lab/` and `scripts/`:

| Old | New |
|---|---|
| `from .types import` / `from dynamic_tiling.types import` | `from hailo_tiling.types import` |
| `from .budget import` / `from dynamic_tiling.budget import` | `from hailo_tiling.budget import` |
| `from .scheduler import` / `from dynamic_tiling.scheduler import` | `from hailo_tiling.dynamic.scheduler import` |
| `from .inference\|replay\|target_lock\|trials\|score\|metrics\|mot_metrics\|compare_baselines\|aggregator import` | relative `from .X import` (within `harness/`) or `from tiling_lab.harness.X import` (from gt/cli/tests) |
| `from .reid_* import` | relative within `reid/`; `from tiling_lab.reid.X import` elsewhere |
| `from .gt_* import` / `from .run_gt_* import` | relative within `gt/`; `from tiling_lab.gt.X import` elsewhere |
| `from dynamic_tiling.X import` (tests) | the new home per this table |
| `import dynamic_tiling._vendor_paths` / `from . import _vendor_paths` | DELETE the line |
| `from tiling_benchmark.prepare_video import fov_to_crop_dims` | `from hailo_tiling.geometry import fov_to_crop_dims` |
| `from .prepare_video import` (inside moved prepare_video consumers, if any) | `from tiling_lab.video.prepare_video import` |

`prepare_video.py` itself keeps its local `fov_to_crop_dims` OR re-exports from `hailo_tiling.geometry` — make it import from `hailo_tiling.geometry` and delete its local copy (single source of truth).
After the sweep: `grep -rn "dynamic_tiling\|_vendor_paths" tiling_lab/ scripts/ hailo_tiling/ --include='*.py'` must return ZERO code hits (docstring usage-example strings get fixed here too — they are part of the file).

- [ ] **Step 3: Packaging.** `pyproject.toml`: `include = ["drone_follow*", "reid_analysis*", "hailo_tiling*", "tiling_lab*"]`, add `"tiling_lab.tests*"` to exclude. `pytest.ini`: testpaths → `hailo_tiling/tests tiling_lab/tests tests`.

- [ ] **Step 4: Re-install editable + full suite**

```bash
/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo-apps/venv_hailo_apps/bin/pip install -e . --no-deps -q
set -o pipefail; /home/giladn/tappas_apps/repos/hailo-drone-follow/hailo-apps/venv_hailo_apps/bin/python -m pytest -q 2>&1 | tail -2
```
Expected: baseline counts (same totals, new paths).

- [ ] **Step 5: Smoke the CLIs** (import-level only, no chip):

```bash
V=/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo-apps/venv_hailo_apps/bin/python
for m in tiling_lab.cli.run_dynamic tiling_lab.cli.run_trials tiling_lab.cli.run_sweep \
         tiling_lab.cli.run_reid_ablation tiling_lab.cli.run_mot_eval \
         tiling_lab.gt.run_gt_verify tiling_lab.viewer.overlay_viewer; do
  $V -m $m --help >/dev/null || echo "FAIL: $m"; done
```
Expected: no FAIL lines.

- [ ] **Step 6: Commit** — `git add -u && git add tiling_lab/ pyproject.toml pytest.ini && git commit -m "refactor(lab): dynamic_tiling -> tiling_lab package (harness/reid/gt/cli/viewer/video)"`
  (Exception to the no-`-u` rule is NOT taken: `-u` here only stages the git-mv'd/deleted tracked files this task touched; verify with `git status --short` that reid_manager.py / submodules are NOT staged before committing. If they appear, unstage them explicitly.)

---

### Task 4: Artifacts migration + GT-lock re-validation

**Files:**
- git mv tracked: `dynamic_tiling/runs/baseline_0025/BASELINE.md`, `dynamic_tiling/runs/REID_ABLATION.md`, `dynamic_tiling/runs/PHASE_A.md`, `dynamic_tiling/runs/MOT_BASELINE.md` (+ any other tracked file under `dynamic_tiling/runs/` — list with `git ls-files dynamic_tiling/runs/`) → same relative paths under `tiling_lab/runs/`
- Plain mv untracked: everything else under `dynamic_tiling/runs/` → `tiling_lab/runs/` (caches, gt_verify trees, frames dumps)
- Plain mv keep-list: `tiling_benchmark/pxt_runs/*.frames.json`, `tiling_benchmark/pxt_runs/*.json`, `tiling_benchmark/pxt_runs_clean/`, `tiling_benchmark/pxt_runs_yolov8m/` → `tiling_lab/runs/legacy_dense/`
- Create: `tiling_lab/runs/legacy_dense/README.md` (provenance: what these are, what was deleted in Task 0 and how to regenerate it)

- [ ] **Step 1: GT-lock safety probe FIRST.** Pick one locked tree (`dynamic_tiling/runs/gt_verify_0025_fov50/`), `mv` it to `tiling_lab/runs/`, re-run its verification (`GT_STATUS.json` hashes content, not paths — confirm by re-running the lock-check command from `docs/gt-generation-guide.md`). Only proceed when it re-validates; otherwise `mv` it back and STOP — report.
- [ ] **Step 2: Move the rest** (git mv for tracked, mv for untracked; preserve relative layout). `rmdir dynamic_tiling` at the end — the directory must vanish.
- [ ] **Step 3: Write `legacy_dense/README.md`** — inventory table (dense GT source 12x9, static-grid baselines, yolov8m runs), the Task-0 deletion record (decode caches `.cache/*.bin` 45.2 GB regenerate automatically on next `run_pxt_bench`; `scaled_sources` via ffmpeg from `~/Videos/Drone/Training/DJI_20260430103421_0010_D_rotated.MP4`), and pointers to the exported insights (`tiling_benchmark/PERF_REPORT.md`, `tiling_lab/runs/*.md`).
- [ ] **Step 4: Suite green** (some tests reference `dynamic_tiling/runs/` fixture paths — fix any that fail; fixtures live in-tests so expect few).
- [ ] **Step 5: Commit** tracked moves + README — explicit paths.

---

### Task 5: Freeze + hygiene

**Files:**
- Create: `tiling_benchmark/DEPRECATED.md`
- Modify: `.gitignore`
- Delete: `HANDOFF.md` (repo root, untracked & stale — superseded by `docs/superpowers/weekend-manager-state.md`)

- [ ] **Step 1: `tiling_benchmark/DEPRECATED.md`** — "Frozen 2026-06-07. Nothing imports this package. Live homes: viewer → `tiling_lab/viewer/`, prepare_video → `tiling_lab/video/`, geometry helpers → `hailo_tiling/geometry.py`, dense artifacts → `tiling_lab/runs/legacy_dense/`. Kept for the legacy pxt/zoom/upscale runners' reference; delete the whole dir when no longer needed."
- [ ] **Step 2: `.gitignore` additions:** `.venv_gt/`, `tiling_lab/runs/**` + `!tiling_lab/runs/**/*.md`, `tiling_benchmark/pxt_runs*/`, `*.frames.json` already-untracked patterns — verify with `git status --short | wc -l` that root-level noise drops; CRITICAL: `git ls-files | grep -c "tiling_lab/runs.*md"` unchanged (tracked MDs not ignored).
- [ ] **Step 3: `rm HANDOFF.md`**
- [ ] **Step 4: Commit** DEPRECATED.md + .gitignore.

---

### Task 6: Architecture-rule test

**Files:**
- Create: `tests/test_architecture.py`

- [ ] **Step 1: Write the test** (this is the enforcement contract):

```python
"""Dependency rules between top-level packages (spec 2026-06-07).

hailo_tiling  -> nothing internal
drone_follow  -> hailo_tiling only
tiling_lab    -> drone_follow + hailo_tiling
nothing       -> tiling_lab / tiling_benchmark
"""
import ast
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
INTERNAL = {"drone_follow", "hailo_tiling", "tiling_lab", "tiling_benchmark", "dynamic_tiling"}
ALLOWED = {
    "hailo_tiling": set(),
    "drone_follow": {"hailo_tiling"},
    "tiling_lab": {"drone_follow", "hailo_tiling"},
}


def _imports(pkg: str):
    for py in (REPO / pkg).rglob("*.py"):
        tree = ast.parse(py.read_text(), filename=str(py))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for a in node.names:
                    yield py, a.name.split(".")[0]
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                yield py, node.module.split(".")[0]


@pytest.mark.parametrize("pkg", sorted(ALLOWED))
def test_package_respects_dependency_rules(pkg):
    bad = [(str(f.relative_to(REPO)), m) for f, m in _imports(pkg)
           if m in INTERNAL and m != pkg and m not in ALLOWED[pkg]]
    assert not bad, f"{pkg} has forbidden internal imports: {bad}"


def test_dynamic_tiling_is_gone():
    assert not (REPO / "dynamic_tiling").exists()
```

- [ ] **Step 2: Run it** — expected PASS (tests dirs are inside the packages and scanned too; if a test file legitimately violates a rule — e.g. `tiling_lab/tests` importing drone_follow is ALLOWED already — fix any genuine offender found, don't loosen the rule).
- [ ] **Step 3: Commit.**

---

### Task 7: class_offset straggler sweep

**Files:**
- Modify: every `HefBackend(`/`CachedHefBackend(` construction site missing `class_offset` (find: `grep -rn "HefBackend(" --include='*.py' tiling_lab/ hailo_tiling/ scripts/ | grep -v class_offset | grep -v "def \|class "`)
- Test: extend `tiling_lab/tests/test_run_dynamic_classes.py` (the weekend regression test) or sibling

- [ ] **Step 1: Inventory the sites.** For each: either add `class_offset=1` (unified person=1/vehicle=2 convention) or add a `# raw ids: <reason>` comment if intentionally raw.
- [ ] **Step 2: Regression test** asserting every construction site in `tiling_lab/cli/` passes the offset (ast-walk the cli modules for `Call` nodes named `*HefBackend` and assert the kwarg present).
- [ ] **Step 3: Suite green, commit.**

---

### Task 8: Docs path sweep

**Files:**
- Modify: `docs/gt-generation-guide.md`, `dynamic_tiling`-path references in `CLAUDE.md`? (NO — CLAUDE.md doesn't reference dynamic_tiling; verify), `tiling_lab/runs/gt_verify_0027_README_morning.md` (runbook commands), README snippets — find all with `grep -rln "dynamic_tiling" docs/ *.md tiling_lab/runs/*.md --include='*.md'`
- Do NOT rewrite history docs: `docs/superpowers/weekend-manager-state.md`, old specs/plans, committed reports' provenance lines stay as-is.

- [ ] **Step 1: Sweep** `python -m dynamic_tiling.X` → `python -m tiling_lab.<sub>.X`, `dynamic_tiling/runs/` → `tiling_lab/runs/`, `tiling_benchmark/overlay_viewer.py` → `python -m tiling_lab.viewer.overlay_viewer` in LIVE docs (the guide + morning runbook + any README).
- [ ] **Step 2: Update `.claude/memory/` project-local memories** that point at moved paths (`MEMORY.md` index entries stay; fix file contents that give commands).
- [ ] **Step 3: Commit docs.**

---

### Task 9: End-to-end validation

- [ ] **Step 1: Full suite** at HEAD — baseline counts + new tests.
- [ ] **Step 2: Warm-cache baseline repro** (chip-cheap, exclusive chip rule applies):

```bash
source hailo-apps/venv_hailo_apps/bin/activate && \
python -m tiling_lab.cli.run_trials \
  --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov50.mp4 \
  --gt tiling_lab/runs/gt_verify_0025_fov50/gt_tracks.verified.json \
  --cache tiling_lab/runs/cache/0025_fov50__yolov8n4c_vga.sqlite3 \
  --reacq-motion frozen --reacq-radius-growth 0.001 \
  --out /tmp/restructure_smoke.json
```
Expected: walker coverage ≈0.989, `cache: ... hit_rate=100.0%` (paths/flags per the moved CLI's --help; adjust to actual arg names).

- [ ] **Step 3: Viewer smoke:** `python -m tiling_lab.viewer.overlay_viewer --help` exits 0.
- [ ] **Step 4: Final report** — summarize commits, before/after sizes (46 GB → ~0.4 GB), test counts, into the plan-completion note.
