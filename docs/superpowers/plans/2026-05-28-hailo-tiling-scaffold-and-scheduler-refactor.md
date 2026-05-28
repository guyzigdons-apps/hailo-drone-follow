# `hailo_tiling` Scaffold + Scheduler Emitter+Modifier Refactor — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Promote `dynamic_tiling/` to a reusable `hailo_tiling/` library with a composable Emitter+Modifier scheduler architecture, while keeping behaviour byte-equivalent to the current single-target `TileScheduler`.

**Architecture:** A new `hailo_tiling/` package lives alongside (does **not** replace) `dynamic_tiling/` during this plan. The single-target `dynamic_tiling.TileScheduler` is reimplemented as `hailo_tiling.scheduler.TileScheduler`, which composes three `TileEmitter` classes (`DiscoveryGridEmitter`, `TrackROIEmitter`, `RecoveryGridEmitter`) and a `BudgetTrimModifier`. A golden-file integration test parameterised over scheduling scenarios asserts that the new composed scheduler returns the *exact* `list[CropRect]` the legacy `dynamic_tiling.TileScheduler` returns. The `MultiTargetTileScheduler` is **not** touched in this plan — multi-target work is v2.

**Tech Stack:** Python 3.10+, pytest. No new runtime dependencies. The existing project venv at `./hailo-apps/venv_hailo_apps` is reused (created by `setup_env.sh`). All hailo_tiling code is pure Python with no Hailo/HailoRT/GStreamer imports — those land in later plans.

**Spec reference:** `docs/superpowers/specs/2026-05-28-tiling-library-design.md` Sections 3.1, 3.2 (Phases 1 and 2 of the spec's phase list).

---

## File Structure

**Files this plan creates:**

```
hailo_tiling/
  __init__.py                     # re-exports public API
  types.py                        # CropRect, Det, LockState, TargetState, ScheduledTile, MODEL_W/H/ASPECT
  budget.py                       # BudgetMeter (verbatim port)
  scheduler.py                    # TileScheduler + TileEmitter Protocol + TileModifier Protocol
  emitters/
    __init__.py                   # re-exports
    discovery_grid.py             # DiscoveryGridEmitter
    track_roi.py                  # TrackROIEmitter
    recovery.py                   # RecoveryGridEmitter
  modifiers/
    __init__.py                   # re-exports
    budget_trim.py                # BudgetTrimModifier
  tests/
    __init__.py
    conftest.py                   # pytest fixtures: lock states, src dims, budget meters
    test_types.py                 # types parity with dynamic_tiling.types
    test_budget.py                # budget parity
    test_emitters.py              # unit tests per emitter
    test_modifiers.py             # unit tests per modifier
    test_scheduler_legacy_parity.py  # golden-file integration test vs dynamic_tiling.TileScheduler

pyproject.toml                    # PEP 621 + setuptools; package = hailo_tiling
LICENSE                           # Apache 2.0
docs/superpowers/plans/INDEX.md   # follow-up plans index (this is plan 1 of 9)
```

**Files this plan modifies:**

- `dynamic_tiling/types.py` — becomes a thin re-export shim from `hailo_tiling.types` so existing callers don't break.
- `dynamic_tiling/budget.py` — same shim treatment.

**Files this plan does NOT touch:**

- `dynamic_tiling/scheduler.py` — stays as the legacy implementation; serves as the ground truth in the golden-file test.
- `dynamic_tiling/aggregator.py`, `dynamic_tiling/inference.py`, `dynamic_tiling/run_dynamic.py`, `dynamic_tiling/replay.py`, `dynamic_tiling/score.py`, `dynamic_tiling/gt_track.py`, `dynamic_tiling/compare_baselines.py`, `dynamic_tiling/target_lock.py` — out of scope for this plan; addressed in Plan 2 (telemetry+backends) and Plan 6 (harness).
- `MultiTargetTileScheduler` (multi-target v2 work) — out of scope.

---

## Pre-flight: virtual environment

All commands assume the project venv is active. If you've never set it up, run `source setup_env.sh` once at the start of the session. After that, use `.venv-style` direct binaries — `/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo-apps/venv_hailo_apps/bin/python` and `.../bin/pytest` — because shell state doesn't persist between tool calls.

For brevity in this plan, paths are written as `python` and `pytest`; the executor should resolve them to the venv binaries.

---

## Task 1: Package scaffold + license + pyproject

**Files:**
- Create: `hailo_tiling/__init__.py`
- Create: `pyproject.toml`
- Create: `LICENSE`

- [ ] **Step 1: Create the package directory and empty `__init__.py`.**

```bash
mkdir -p hailo_tiling
```

```python
# hailo_tiling/__init__.py
"""hailo_tiling — reusable dynamic-tiling library for Hailo inference pipelines.

See docs/superpowers/specs/2026-05-28-tiling-library-design.md.
"""
__version__ = "0.1.0.dev0"
```

- [ ] **Step 2: Write `pyproject.toml` with PEP 621 metadata.**

```toml
# pyproject.toml
[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "hailo_tiling"
version = "0.1.0.dev0"
description = "Reusable dynamic-tiling library for Hailo inference pipelines (scheduler, telemetry, cache, backends)."
readme = "README.md"
requires-python = ">=3.10"
license = { text = "Apache-2.0" }
authors = [{ name = "Hailo drone-follow team" }]
classifiers = [
  "Development Status :: 3 - Alpha",
  "Intended Audience :: Developers",
  "License :: OSI Approved :: Apache Software License",
  "Programming Language :: Python :: 3.10",
  "Programming Language :: Python :: 3.11",
  "Topic :: Scientific/Engineering :: Image Recognition",
]
dependencies = []

[project.optional-dependencies]
mavsdk = ["mavsdk>=2.0"]
test = ["pytest>=7"]

[tool.setuptools.packages.find]
include = ["hailo_tiling*"]
exclude = ["hailo_tiling.tests*"]

[tool.pytest.ini_options]
testpaths = ["hailo_tiling/tests"]
```

- [ ] **Step 3: Drop the Apache-2.0 license.**

```bash
curl -fsSL https://www.apache.org/licenses/LICENSE-2.0.txt -o LICENSE
```

If the network is unavailable, paste the standard Apache 2.0 text manually. Verify:

```bash
head -1 LICENSE
```
Expected: `Apache License` (the standard preamble line).

- [ ] **Step 4: Write a minimal `README.md` for the package root.**

```markdown
# hailo_tiling

Reusable dynamic-tiling library for Hailo inference pipelines. Provides a
composable Emitter+Modifier scheduler architecture, telemetry abstraction,
inference backends (HailoRT and GStreamer), and an SQLite-backed cache
that lets ablation studies skip both inference and post-processing on
previously-seen crops.

See `docs/superpowers/specs/2026-05-28-tiling-library-design.md` for the
full design spec.
```

- [ ] **Step 5: Verify the package imports.**

Run:
```bash
python -c "import hailo_tiling; print(hailo_tiling.__version__)"
```
Expected: `0.1.0.dev0` (no exceptions).

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/__init__.py pyproject.toml LICENSE README.md
git commit -m "hailo_tiling: scaffold package with Apache-2.0 license"
```

---

## Task 2: Move types into `hailo_tiling`, make `dynamic_tiling.types` a shim

**Files:**
- Create: `hailo_tiling/types.py` (verbatim content of `dynamic_tiling/types.py`)
- Modify: `dynamic_tiling/types.py` (becomes a shim)
- Create: `hailo_tiling/tests/__init__.py`
- Create: `hailo_tiling/tests/conftest.py`
- Create: `hailo_tiling/tests/test_types.py`

- [ ] **Step 1: Copy types.py verbatim.**

```bash
cp dynamic_tiling/types.py hailo_tiling/types.py
```

Verify the file is byte-identical:
```bash
diff dynamic_tiling/types.py hailo_tiling/types.py
```
Expected: no output (files identical).

- [ ] **Step 2: Replace `dynamic_tiling/types.py` with a re-export shim.**

```python
# dynamic_tiling/types.py
"""Compatibility shim: types live in hailo_tiling.types.

This module re-exports the public API so legacy `from dynamic_tiling.types import …`
imports keep working during the migration. Remove this shim in Plan 8
(drone-follow migration) once all callers move to hailo_tiling.
"""
from hailo_tiling.types import (  # noqa: F401
    MODEL_W,
    MODEL_H,
    MODEL_ASPECT,
    CropRect,
    Det,
    LockState,
    TargetState,
    ScheduledTile,
)
```

- [ ] **Step 3: Create the tests package + conftest with fixtures.**

```python
# hailo_tiling/tests/__init__.py
```
(empty file)

```python
# hailo_tiling/tests/conftest.py
"""Shared fixtures for hailo_tiling tests."""
from __future__ import annotations

import pytest

from hailo_tiling.types import LockState


@pytest.fixture
def src_dims():
    """Standard 4K source dimensions used across scheduler tests."""
    return (3840, 2160)


@pytest.fixture
def tracking_lock():
    """A TRACKING lock state with a person bbox roughly centered."""
    return LockState(
        track_id=42,
        bbox_norm=(0.45, 0.40, 0.05, 0.15),
        status="TRACKING",
        frames_since_seen=0,
        last_velocity=(0.0, 0.0),
    )


@pytest.fixture
def searching_lock():
    """A SEARCHING lock state with last-known bbox and small velocity."""
    return LockState(
        track_id=7,
        bbox_norm=(0.60, 0.55, 0.04, 0.12),
        status="SEARCHING",
        frames_since_seen=5,
        last_velocity=(0.001, -0.002),
    )


@pytest.fixture
def lost_lock():
    """A LOST lock state."""
    return LockState(
        track_id=None,
        bbox_norm=(0.0, 0.0, 0.0, 0.0),
        status="LOST",
        frames_since_seen=999,
        last_velocity=(0.0, 0.0),
    )
```

- [ ] **Step 4: Write the failing parity test.**

```python
# hailo_tiling/tests/test_types.py
"""Verify hailo_tiling.types and dynamic_tiling.types are the same objects."""
from __future__ import annotations


def test_dynamic_tiling_types_reexport_hailo_tiling():
    """The shim must re-export the same classes by identity."""
    from hailo_tiling.types import CropRect as HtCropRect
    from hailo_tiling.types import Det as HtDet
    from hailo_tiling.types import LockState as HtLockState
    from hailo_tiling.types import TargetState as HtTargetState
    from hailo_tiling.types import ScheduledTile as HtScheduledTile

    from dynamic_tiling.types import CropRect as DtCropRect
    from dynamic_tiling.types import Det as DtDet
    from dynamic_tiling.types import LockState as DtLockState
    from dynamic_tiling.types import TargetState as DtTargetState
    from dynamic_tiling.types import ScheduledTile as DtScheduledTile

    assert HtCropRect is DtCropRect
    assert HtDet is DtDet
    assert HtLockState is DtLockState
    assert HtTargetState is DtTargetState
    assert HtScheduledTile is DtScheduledTile


def test_model_constants():
    from hailo_tiling.types import MODEL_W, MODEL_H, MODEL_ASPECT
    assert MODEL_W == 640
    assert MODEL_H == 480
    assert MODEL_ASPECT == 640 / 480


def test_croprect_clamp_keeps_w_h():
    from hailo_tiling.types import CropRect
    r = CropRect(x=-10, y=-5, w=100, h=75, mode="s").clamp(3840, 2160)
    assert r.x == 0 and r.y == 0
    assert r.w == 100 and r.h == 75
```

- [ ] **Step 5: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_types.py -v
```
Expected: 3 passed.

- [ ] **Step 6: Verify legacy `dynamic_tiling` tests still pass through the shim.**

Run:
```bash
pytest dynamic_tiling/tests/test_types.py -v
```
Expected: all existing tests pass (the legacy test file imports from `dynamic_tiling.types`, which now goes through the shim).

- [ ] **Step 7: Commit.**

```bash
git add hailo_tiling/types.py hailo_tiling/tests/__init__.py \
        hailo_tiling/tests/conftest.py hailo_tiling/tests/test_types.py \
        dynamic_tiling/types.py
git commit -m "hailo_tiling: move types; dynamic_tiling.types is now a re-export shim"
```

---

## Task 3: Move budget into `hailo_tiling`, make `dynamic_tiling.budget` a shim

**Files:**
- Create: `hailo_tiling/budget.py`
- Modify: `dynamic_tiling/budget.py` (becomes a shim)
- Create: `hailo_tiling/tests/test_budget.py`

- [ ] **Step 1: Copy budget.py verbatim.**

```bash
cp dynamic_tiling/budget.py hailo_tiling/budget.py
```

- [ ] **Step 2: Replace `dynamic_tiling/budget.py` with a re-export shim.**

```python
# dynamic_tiling/budget.py
"""Compatibility shim: BudgetMeter lives in hailo_tiling.budget."""
from hailo_tiling.budget import BudgetMeter  # noqa: F401
```

- [ ] **Step 3: Write the failing parity + behaviour test.**

```python
# hailo_tiling/tests/test_budget.py
"""BudgetMeter unit tests + parity with dynamic_tiling.budget."""
from __future__ import annotations

from hailo_tiling.budget import BudgetMeter


def test_shim_identity():
    from dynamic_tiling.budget import BudgetMeter as DtBudgetMeter
    assert BudgetMeter is DtBudgetMeter


def test_available_at_start_returns_per_frame_share():
    meter = BudgetMeter(budget_inf_per_s=300.0, fps=30.0, window_s=1.0)
    # At start, full window cap available; per-frame share = 300/30 = 10.0
    assert meter.available(0) == 10.0


def test_charge_and_available_after_spend():
    meter = BudgetMeter(budget_inf_per_s=300.0, fps=30.0, window_s=1.0)
    meter.charge(n_tiles=5, frame_idx=0)
    # 300 cap - 5 spent = 295 remaining over 30 frames = 9.833...
    assert meter.available(1) == 295.0 / 30.0


def test_eviction_after_window():
    meter = BudgetMeter(budget_inf_per_s=300.0, fps=30.0, window_s=1.0)
    meter.charge(n_tiles=300, frame_idx=0)
    # After window_frames + 1, the old charge is evicted: full budget back.
    assert meter.available(31) == 10.0
```

- [ ] **Step 4: Run the test.**

Run:
```bash
pytest hailo_tiling/tests/test_budget.py -v
```
Expected: 4 passed.

- [ ] **Step 5: Verify legacy budget tests still pass.**

Run:
```bash
pytest dynamic_tiling/tests/test_budget.py -v
```
Expected: all existing tests pass.

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/budget.py dynamic_tiling/budget.py hailo_tiling/tests/test_budget.py
git commit -m "hailo_tiling: move budget; dynamic_tiling.budget is now a re-export shim"
```

---

## Task 4: Define `TileEmitter` and `TileModifier` Protocols + `TileScheduler` skeleton

**Files:**
- Create: `hailo_tiling/scheduler.py`
- Create: `hailo_tiling/tests/test_scheduler_protocols.py`

- [ ] **Step 1: Write the failing protocol test.**

```python
# hailo_tiling/tests/test_scheduler_protocols.py
"""Validate the Protocol shapes and TileScheduler composition contract."""
from __future__ import annotations

from typing import Sequence

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.scheduler import (
    TileEmitter,
    TileModifier,
    TileScheduler,
)
from hailo_tiling.types import CropRect, LockState


class _DummyEmitter:
    """Test double: emits a fixed list of CropRects."""
    name = "dummy_emitter"

    def __init__(self, crops):
        self._crops = crops

    def emit(self, src_w, src_h, lock, frame_idx, meter):
        return list(self._crops)


class _DummyModifier:
    """Test double: tags each crop's mode with a fixed string."""
    name = "dummy_modifier"

    def __init__(self, tag):
        self._tag = tag

    def modify(self, tiles, src_w, src_h, lock, frame_idx, meter):
        return [CropRect(x=t.x, y=t.y, w=t.w, h=t.h, mode=self._tag) for t in tiles]


def test_protocol_runtime_compat():
    """Dummy classes structurally satisfy the Protocols."""
    e: TileEmitter = _DummyEmitter([])
    m: TileModifier = _DummyModifier("t")
    assert e.name == "dummy_emitter"
    assert m.name == "dummy_modifier"


def test_scheduler_runs_emitters_then_modifiers_in_order(src_dims, tracking_lock):
    src_w, src_h = src_dims
    meter = BudgetMeter(budget_inf_per_s=300.0, fps=30.0)
    a = CropRect(x=0, y=0, w=640, h=480, mode="s")
    b = CropRect(x=10, y=10, w=640, h=480, mode="s")
    scheduler = TileScheduler(
        emitters=[_DummyEmitter([a]), _DummyEmitter([b])],
        modifiers=[_DummyModifier("X")],
    )
    out = scheduler.decide(src_w, src_h, tracking_lock, frame_idx=0, meter=meter)
    # Emitters concatenated in order, then modifier rewrote each mode.
    assert [c.mode for c in out] == ["X", "X"]
    assert (out[0].x, out[0].y) == (0, 0)
    assert (out[1].x, out[1].y) == (10, 10)


def test_scheduler_empty_emitters_returns_empty():
    meter = BudgetMeter(budget_inf_per_s=300.0, fps=30.0)
    scheduler = TileScheduler(emitters=[], modifiers=[])
    out = scheduler.decide(3840, 2160, LockState(), 0, meter)
    assert out == []
```

- [ ] **Step 2: Run the test, see failure (import error).**

Run:
```bash
pytest hailo_tiling/tests/test_scheduler_protocols.py -v
```
Expected: FAIL — `ImportError: cannot import name 'TileEmitter' from 'hailo_tiling.scheduler'`.

- [ ] **Step 3: Implement the protocols and scheduler.**

```python
# hailo_tiling/scheduler.py
"""TileScheduler — composes TileEmitters and TileModifiers.

The scheduler runs each emitter in order, concatenates the resulting CropRects,
then runs each modifier in order, threading the working tile list through. The
last modifier should typically be `BudgetTrimModifier`, which enforces the
per-frame inference budget.

This module deliberately contains no Hailo / GStreamer / OpenCV imports.
"""
from __future__ import annotations

from typing import Protocol, Sequence, runtime_checkable

from .types import CropRect, LockState


@runtime_checkable
class TileEmitter(Protocol):
    """Produces a list of CropRects for the current frame."""

    name: str

    def emit(
        self,
        src_w: int,
        src_h: int,
        lock: LockState,
        frame_idx: int,
        meter,
    ) -> list[CropRect]: ...


@runtime_checkable
class TileModifier(Protocol):
    """Mutates the working tile list before submission."""

    name: str

    def modify(
        self,
        tiles: list[CropRect],
        src_w: int,
        src_h: int,
        lock: LockState,
        frame_idx: int,
        meter,
    ) -> list[CropRect]: ...


class TileScheduler:
    """Composable scheduler. Pure orchestration; emitters/modifiers do the work."""

    def __init__(
        self,
        emitters: Sequence[TileEmitter],
        modifiers: Sequence[TileModifier],
    ):
        self.emitters = list(emitters)
        self.modifiers = list(modifiers)

    def decide(
        self,
        src_w: int,
        src_h: int,
        lock: LockState,
        frame_idx: int,
        meter,
    ) -> list[CropRect]:
        tiles: list[CropRect] = []
        for e in self.emitters:
            tiles.extend(e.emit(src_w, src_h, lock, frame_idx, meter))
        for m in self.modifiers:
            tiles = m.modify(tiles, src_w, src_h, lock, frame_idx, meter)
        return tiles
```

- [ ] **Step 4: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_scheduler_protocols.py -v
```
Expected: 3 passed.

- [ ] **Step 5: Commit.**

```bash
git add hailo_tiling/scheduler.py hailo_tiling/tests/test_scheduler_protocols.py
git commit -m "hailo_tiling: TileEmitter/TileModifier Protocols + composing TileScheduler"
```

---

## Task 5: `DiscoveryGridEmitter` (port of `dynamic_tiling._grid` for the discovery branch)

**Files:**
- Create: `hailo_tiling/emitters/__init__.py`
- Create: `hailo_tiling/emitters/discovery_grid.py`
- Create: `hailo_tiling/tests/test_emitter_discovery_grid.py`

- [ ] **Step 1: Create the `emitters/__init__.py` re-exports file.**

```python
# hailo_tiling/emitters/__init__.py
"""Tile emitters — classes that produce CropRects given frame + track state."""
from .discovery_grid import DiscoveryGridEmitter  # noqa: F401
```

- [ ] **Step 2: Write the failing emitter test.**

```python
# hailo_tiling/tests/test_emitter_discovery_grid.py
"""DiscoveryGridEmitter — full-frame N×M grid on a cadence."""
from __future__ import annotations

import pytest

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.emitters import DiscoveryGridEmitter
from hailo_tiling.types import CropRect, LockState


def _meter():
    return BudgetMeter(budget_inf_per_s=1000.0, fps=30.0)


def test_emits_3x2_on_cadence(src_dims):
    src_w, src_h = src_dims
    e = DiscoveryGridEmitter(grid=(3, 2), period=15, mode="m")
    out = e.emit(src_w, src_h, LockState(), frame_idx=0, meter=_meter())
    assert len(out) == 6
    assert all(isinstance(c, CropRect) for c in out)
    assert all(c.mode == "m" for c in out)


def test_emits_empty_off_cadence(src_dims):
    src_w, src_h = src_dims
    e = DiscoveryGridEmitter(grid=(3, 2), period=15, mode="m")
    out = e.emit(src_w, src_h, LockState(), frame_idx=1, meter=_meter())
    assert out == []


def test_matches_legacy_grid(src_dims):
    """Exact output equality with the legacy `_grid` implementation."""
    from dynamic_tiling.scheduler import TileScheduler as LegacyScheduler
    src_w, src_h = src_dims
    legacy = LegacyScheduler(src_w, src_h, discovery_grid=(3, 2))
    expected = legacy._grid(3, 2, 0, 0, src_w, src_h, "m")

    e = DiscoveryGridEmitter(grid=(3, 2), period=15, mode="m")
    actual = e.emit(src_w, src_h, LockState(), frame_idx=0, meter=_meter())

    assert actual == expected
```

- [ ] **Step 3: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_emitter_discovery_grid.py -v
```
Expected: FAIL — module not found.

- [ ] **Step 4: Implement `DiscoveryGridEmitter`.**

```python
# hailo_tiling/emitters/discovery_grid.py
"""Full-frame N×M discovery grid emitted on a fixed-period cadence."""
from __future__ import annotations

from ..types import CropRect, LockState, MODEL_ASPECT


def _grid_full(src_w: int, src_h: int, gx: int, gy: int, mode: str) -> list[CropRect]:
    """Build the N×M grid over the full frame.

    Each tile is 4:3 (model aspect). Tile width is grown to the larger of the
    cell width and the aspect-scaled cell height so cells are fully covered
    (tiles may overlap) rather than leaving vertical/horizontal gaps.

    This is the inlined equivalent of `dynamic_tiling.TileScheduler._grid(
    gx, gy, 0, 0, src_w, src_h, mode)` and must produce byte-identical output.
    """
    out: list[CropRect] = []
    cw = src_w / gx
    ch = src_h / gy
    crop_w = max(cw, ch * MODEL_ASPECT)
    for j in range(gy):
        for i in range(gx):
            cx = (i + 0.5) * cw
            cy = (j + 0.5) * ch
            r = CropRect.from_center_width(cx, cy, int(round(crop_w)), mode=mode)
            out.append(r.clamp(src_w, src_h))
    return out


class DiscoveryGridEmitter:
    """Emits an N×M grid covering the full frame every `period` frames."""

    name = "discovery_grid"

    def __init__(self, grid: tuple[int, int] = (3, 2), period: int = 15, mode: str = "m"):
        self.gx, self.gy = grid
        self.period = period
        self.mode = mode

    def emit(self, src_w, src_h, lock, frame_idx, meter) -> list[CropRect]:
        if frame_idx % self.period != 0:
            return []
        return _grid_full(src_w, src_h, self.gx, self.gy, self.mode)
```

- [ ] **Step 5: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_emitter_discovery_grid.py -v
```
Expected: 3 passed.

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/emitters/__init__.py \
        hailo_tiling/emitters/discovery_grid.py \
        hailo_tiling/tests/test_emitter_discovery_grid.py
git commit -m "hailo_tiling: DiscoveryGridEmitter (full-frame N×M on cadence)"
```

---

## Task 6: `TrackROIEmitter` (port of `dynamic_tiling._roi`)

**Files:**
- Modify: `hailo_tiling/emitters/__init__.py` (add export)
- Create: `hailo_tiling/emitters/track_roi.py`
- Create: `hailo_tiling/tests/test_emitter_track_roi.py`

- [ ] **Step 1: Write the failing emitter test.**

```python
# hailo_tiling/tests/test_emitter_track_roi.py
"""TrackROIEmitter — predicted-bbox ROI tile while TRACKING."""
from __future__ import annotations

import pytest

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.emitters.track_roi import TrackROIEmitter
from hailo_tiling.types import LockState


def _meter():
    return BudgetMeter(budget_inf_per_s=1000.0, fps=30.0)


def test_emits_one_tile_when_tracking(src_dims, tracking_lock):
    src_w, src_h = src_dims
    e = TrackROIEmitter(max_zoom=2.0, target_model_h=40.0, roi_margin_frac=0.25)
    out = e.emit(src_w, src_h, tracking_lock, frame_idx=0, meter=_meter())
    assert len(out) == 1
    assert out[0].mode == "s"


def test_emits_empty_when_not_tracking(src_dims, searching_lock, lost_lock):
    src_w, src_h = src_dims
    e = TrackROIEmitter()
    assert e.emit(src_w, src_h, searching_lock, 0, _meter()) == []
    assert e.emit(src_w, src_h, lost_lock, 0, _meter()) == []


def test_matches_legacy_roi(src_dims, tracking_lock):
    """Exact output equality with the legacy `_roi` implementation."""
    from dynamic_tiling.scheduler import TileScheduler as LegacyScheduler
    src_w, src_h = src_dims
    legacy = LegacyScheduler(src_w, src_h, max_zoom=2.0, target_model_h=40.0,
                             roi_margin_frac=0.25)
    expected = legacy._roi(tracking_lock)

    e = TrackROIEmitter(max_zoom=2.0, target_model_h=40.0, roi_margin_frac=0.25)
    actual = e.emit(src_w, src_h, tracking_lock, 0, _meter())

    assert actual == [expected]
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_emitter_track_roi.py -v
```
Expected: FAIL — module not found.

- [ ] **Step 3: Implement `TrackROIEmitter`.**

```python
# hailo_tiling/emitters/track_roi.py
"""Track-guided ROI tile centered on the currently locked bbox."""
from __future__ import annotations

from ..types import CropRect, LockState, MODEL_W, MODEL_H, MODEL_ASPECT


class TrackROIEmitter:
    """Emits a single ROI tile while `lock.status == 'TRACKING'`.

    Crop width is sized so the locked target occupies `target_model_h` pixels
    of vertical extent after the crop is rescaled to model input. Width is
    clamped to [MODEL_W/max_zoom, MODEL_W] (don't upscale; don't zoom past
    max_zoom) but always grown to contain the *whole* target plus margin.

    This is the inlined equivalent of `dynamic_tiling.TileScheduler._roi(lock)`
    and must produce byte-identical output.
    """

    name = "track_roi"

    def __init__(
        self,
        max_zoom: float = 2.0,
        target_model_h: float = 40.0,
        roi_margin_frac: float = 0.25,
    ):
        self.max_zoom = max_zoom
        self.target_model_h = target_model_h
        self.roi_margin_frac = roi_margin_frac

    def emit(self, src_w, src_h, lock: LockState, frame_idx, meter) -> list[CropRect]:
        if lock.status != "TRACKING":
            return []
        bx, by, bw, bh = lock.bbox_norm
        cx = (bx + bw / 2) * src_w
        cy = (by + bh / 2) * src_h
        src_h_px = max(1.0, bh * src_h)
        crop_w = src_h_px * MODEL_H * MODEL_ASPECT / self.target_model_h
        lo = MODEL_W / self.max_zoom
        crop_w = max(lo, crop_w)
        crop_w = min(crop_w, float(MODEL_W))
        need_w = bw * src_w * (1 + 2 * self.roi_margin_frac)
        crop_w = max(crop_w, need_w)
        rect = CropRect.from_center_width(cx, cy, int(round(crop_w))).clamp(src_w, src_h)
        return [rect]
```

- [ ] **Step 4: Add the export to `hailo_tiling/emitters/__init__.py`.**

```python
# hailo_tiling/emitters/__init__.py
"""Tile emitters — classes that produce CropRects given frame + track state."""
from .discovery_grid import DiscoveryGridEmitter  # noqa: F401
from .track_roi import TrackROIEmitter  # noqa: F401
```

- [ ] **Step 5: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_emitter_track_roi.py -v
```
Expected: 3 passed.

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/emitters/__init__.py \
        hailo_tiling/emitters/track_roi.py \
        hailo_tiling/tests/test_emitter_track_roi.py
git commit -m "hailo_tiling: TrackROIEmitter (predicted-bbox ROI when TRACKING)"
```

---

## Task 7: `RecoveryGridEmitter` (port of the SEARCHING/LOST recovery branch)

**Files:**
- Modify: `hailo_tiling/emitters/__init__.py` (add export)
- Create: `hailo_tiling/emitters/recovery.py`
- Create: `hailo_tiling/tests/test_emitter_recovery.py`

- [ ] **Step 1: Write the failing emitter test.**

```python
# hailo_tiling/tests/test_emitter_recovery.py
"""RecoveryGridEmitter — search grid around the predicted lost-target position."""
from __future__ import annotations

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.emitters.recovery import RecoveryGridEmitter
from hailo_tiling.types import LockState


def _meter():
    return BudgetMeter(budget_inf_per_s=1000.0, fps=30.0)


def test_emits_3x3_when_searching(src_dims, searching_lock):
    src_w, src_h = src_dims
    e = RecoveryGridEmitter(grid=(3, 3), span=0.4)
    out = e.emit(src_w, src_h, searching_lock, frame_idx=0, meter=_meter())
    assert len(out) == 9
    assert all(c.mode == "s" for c in out)


def test_emits_empty_when_tracking(src_dims, tracking_lock):
    e = RecoveryGridEmitter()
    out = e.emit(src_dims[0], src_dims[1], tracking_lock, 0, _meter())
    assert out == []


def test_emits_empty_when_lost_without_track_id(src_dims, lost_lock):
    # lost_lock has track_id=None — recovery should not fire.
    e = RecoveryGridEmitter()
    out = e.emit(src_dims[0], src_dims[1], lost_lock, 0, _meter())
    assert out == []


def test_matches_legacy_recovery(src_dims, searching_lock):
    """Exact output equality with the legacy recovery branch."""
    from dynamic_tiling.scheduler import TileScheduler as LegacyScheduler
    from hailo_tiling.types import MODEL_ASPECT  # noqa: F401  (matches legacy import order)

    src_w, src_h = src_dims
    legacy = LegacyScheduler(src_w, src_h, recovery_grid=(3, 3), recovery_span=0.4)

    bx, by, bw, bh = searching_lock.bbox_norm
    ecx = bx + bw / 2 + searching_lock.last_velocity[0] * searching_lock.frames_since_seen
    ecy = by + bh / 2 + searching_lock.last_velocity[1] * searching_lock.frames_since_seen
    span = 0.4
    half = span / 2
    x0_n = max(0.0, min(1.0 - span, ecx - half))
    y0_n = max(0.0, min(1.0 - span, ecy - half))
    expected = legacy._grid(
        3, 3,
        x0_n * src_w, y0_n * src_h,
        span * src_w, span * src_h,
        "s",
    )

    e = RecoveryGridEmitter(grid=(3, 3), span=0.4)
    actual = e.emit(src_w, src_h, searching_lock, 0, _meter())

    assert actual == expected
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_emitter_recovery.py -v
```
Expected: FAIL — module not found.

- [ ] **Step 3: Implement `RecoveryGridEmitter`.**

```python
# hailo_tiling/emitters/recovery.py
"""Search grid emitted when the locked target is SEARCHING or LOST."""
from __future__ import annotations

from ..types import CropRect, LockState, MODEL_ASPECT
from .discovery_grid import _grid_full as _full_frame_grid_unused  # noqa: F401


def _grid_region(src_w: int, src_h: int, gx: int, gy: int,
                 x0: float, y0: float, w: float, h: float, mode: str) -> list[CropRect]:
    """Build a gx×gy grid over the [x0, y0, w, h] src-pixel region.

    Equivalent to `dynamic_tiling.TileScheduler._grid(gx, gy, x0, y0, w, h, mode)`.
    Must produce byte-identical output.
    """
    out: list[CropRect] = []
    cw = w / gx
    ch = h / gy
    crop_w = max(cw, ch * MODEL_ASPECT)
    for j in range(gy):
        for i in range(gx):
            cx = x0 + (i + 0.5) * cw
            cy = y0 + (j + 0.5) * ch
            r = CropRect.from_center_width(cx, cy, int(round(crop_w)), mode=mode)
            out.append(r.clamp(src_w, src_h))
    return out


class RecoveryGridEmitter:
    """Emits a search grid around the predicted lost-target position.

    Fires only when `lock.status in {'SEARCHING', 'LOST'}` AND `lock.track_id is not None`.
    The grid is centered on the last-known bbox extrapolated by velocity * frames_since_seen
    (motion-predicted placement).
    """

    name = "recovery_grid"

    def __init__(self, grid: tuple[int, int] = (3, 3), span: float = 0.4, mode: str = "s"):
        self.gx, self.gy = grid
        self.span = span
        self.mode = mode

    def emit(self, src_w, src_h, lock: LockState, frame_idx, meter) -> list[CropRect]:
        if lock.status not in ("SEARCHING", "LOST"):
            return []
        if lock.track_id is None:
            return []
        bx, by, bw, bh = lock.bbox_norm
        ecx = bx + bw / 2 + lock.last_velocity[0] * lock.frames_since_seen
        ecy = by + bh / 2 + lock.last_velocity[1] * lock.frames_since_seen
        span = self.span
        half = span / 2
        x0_n = max(0.0, min(1.0 - span, ecx - half))
        y0_n = max(0.0, min(1.0 - span, ecy - half))
        return _grid_region(
            src_w, src_h, self.gx, self.gy,
            x0_n * src_w, y0_n * src_h,
            span * src_w, span * src_h,
            self.mode,
        )
```

- [ ] **Step 4: Add the export.**

```python
# hailo_tiling/emitters/__init__.py
"""Tile emitters — classes that produce CropRects given frame + track state."""
from .discovery_grid import DiscoveryGridEmitter  # noqa: F401
from .recovery import RecoveryGridEmitter  # noqa: F401
from .track_roi import TrackROIEmitter  # noqa: F401
```

- [ ] **Step 5: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_emitter_recovery.py -v
```
Expected: 4 passed.

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/emitters/__init__.py \
        hailo_tiling/emitters/recovery.py \
        hailo_tiling/tests/test_emitter_recovery.py
git commit -m "hailo_tiling: RecoveryGridEmitter (search grid when SEARCHING/LOST)"
```

---

## Task 8: `BudgetTrimModifier` (final-stage budget enforcement)

**Files:**
- Create: `hailo_tiling/modifiers/__init__.py`
- Create: `hailo_tiling/modifiers/budget_trim.py`
- Create: `hailo_tiling/tests/test_modifier_budget_trim.py`

- [ ] **Step 1: Write the failing modifier test.**

```python
# hailo_tiling/tests/test_modifier_budget_trim.py
"""BudgetTrimModifier — trims the tile list to fit the per-frame budget."""
from __future__ import annotations

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.modifiers import BudgetTrimModifier
from hailo_tiling.types import CropRect, LockState


def _tiles(n: int) -> list[CropRect]:
    return [CropRect(x=i, y=0, w=640, h=480, mode="s") for i in range(n)]


def test_no_trim_when_under_budget(src_dims):
    src_w, src_h = src_dims
    meter = BudgetMeter(budget_inf_per_s=1000.0, fps=30.0)  # 33 tiles/frame share
    m = BudgetTrimModifier()
    tiles = _tiles(5)
    out = m.modify(tiles, src_w, src_h, LockState(), 0, meter)
    assert out == tiles


def test_trims_from_tail(src_dims):
    src_w, src_h = src_dims
    meter = BudgetMeter(budget_inf_per_s=90.0, fps=30.0)  # 3 tiles/frame share
    m = BudgetTrimModifier()
    tiles = _tiles(10)
    out = m.modify(tiles, src_w, src_h, LockState(), 0, meter)
    assert out == tiles[:3]


def test_budget_zero_returns_empty(src_dims):
    src_w, src_h = src_dims
    meter = BudgetMeter(budget_inf_per_s=0.0, fps=30.0)
    m = BudgetTrimModifier()
    out = m.modify(_tiles(5), src_w, src_h, LockState(), 0, meter)
    assert out == []


def test_negative_budget_disabled_returns_all(src_dims):
    """A meter that returns a negative available count means 'unlimited'."""

    class UnlimitedMeter:
        def available(self, frame_idx):
            return -1.0

    src_w, src_h = src_dims
    m = BudgetTrimModifier()
    tiles = _tiles(5)
    out = m.modify(tiles, src_w, src_h, LockState(), 0, UnlimitedMeter())
    assert out == tiles
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_modifier_budget_trim.py -v
```
Expected: FAIL — module not found.

- [ ] **Step 3: Implement `BudgetTrimModifier` and the modifiers re-export module.**

```python
# hailo_tiling/modifiers/__init__.py
"""Tile modifiers — classes that mutate the working tile list before submission."""
from .budget_trim import BudgetTrimModifier  # noqa: F401
```

```python
# hailo_tiling/modifiers/budget_trim.py
"""Final-stage modifier that enforces the per-frame inference budget."""
from __future__ import annotations

from ..types import CropRect, LockState


class BudgetTrimModifier:
    """Truncate the tile list to fit `meter.available(frame_idx)`.

    Mirrors the budget-enforcement tail in `dynamic_tiling.TileScheduler.decide`:
    a negative `available()` is treated as 'unlimited'; otherwise the list is
    truncated to the floor of the available per-frame share. Tiles are kept
    from the head — emitter order is the priority order, so the upstream
    composition (ROI before discovery) decides what survives a tight budget.
    """

    name = "budget_trim"

    def modify(
        self,
        tiles: list[CropRect],
        src_w: int,
        src_h: int,
        lock: LockState,
        frame_idx: int,
        meter,
    ) -> list[CropRect]:
        budget = int(meter.available(frame_idx))
        if budget < 0:
            return tiles
        if len(tiles) <= budget:
            return tiles
        return tiles[: max(0, budget)]
```

- [ ] **Step 4: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_modifier_budget_trim.py -v
```
Expected: 4 passed.

- [ ] **Step 5: Commit.**

```bash
git add hailo_tiling/modifiers/__init__.py \
        hailo_tiling/modifiers/budget_trim.py \
        hailo_tiling/tests/test_modifier_budget_trim.py
git commit -m "hailo_tiling: BudgetTrimModifier (per-frame budget enforcement)"
```

---

## Task 9: Legacy-parity golden-file integration test

The earlier per-emitter tests verified each unit against the legacy private methods. This task verifies the **composed** scheduler matches the legacy `decide()` end-to-end across the scheduling scenarios we care about.

**Files:**
- Create: `hailo_tiling/tests/test_scheduler_legacy_parity.py`

- [ ] **Step 1: Write the parity test.**

```python
# hailo_tiling/tests/test_scheduler_legacy_parity.py
"""End-to-end golden-file parity vs dynamic_tiling.TileScheduler.

For each parameterised scenario, build a composed hailo_tiling.TileScheduler
with the same parameters and assert that `decide()` returns the byte-identical
list[CropRect] as the legacy scheduler.

Once Plan 8 (drone-follow migration) removes the dynamic_tiling shim, this
test will be replaced by static golden files generated from this test's
expected outputs.
"""
from __future__ import annotations

import pytest

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.emitters import (
    DiscoveryGridEmitter,
    RecoveryGridEmitter,
    TrackROIEmitter,
)
from hailo_tiling.modifiers import BudgetTrimModifier
from hailo_tiling.scheduler import TileScheduler
from hailo_tiling.types import LockState


# (description, src_w, src_h, scheduler_kwargs, lock, frame_idx, meter_args)
SCENARIOS = [
    (
        "tracking_on_cadence_4k",
        3840, 2160,
        dict(discovery_period=15, discovery_grid=(3, 2),
             recovery_grid=(3, 3), max_zoom=2.0, target_model_h=40.0,
             roi_margin_frac=0.25, recovery_span=0.4),
        LockState(track_id=42, bbox_norm=(0.45, 0.40, 0.05, 0.15),
                  status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0)),
        0,
        dict(budget_inf_per_s=1000.0, fps=30.0),
    ),
    (
        "tracking_off_cadence_4k",
        3840, 2160,
        dict(discovery_period=15, discovery_grid=(3, 2),
             recovery_grid=(3, 3), max_zoom=2.0, target_model_h=40.0,
             roi_margin_frac=0.25, recovery_span=0.4),
        LockState(track_id=42, bbox_norm=(0.45, 0.40, 0.05, 0.15),
                  status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0)),
        7,
        dict(budget_inf_per_s=1000.0, fps=30.0),
    ),
    (
        "searching_4k",
        3840, 2160,
        dict(discovery_period=15, discovery_grid=(3, 2),
             recovery_grid=(3, 3), max_zoom=2.0, target_model_h=40.0,
             roi_margin_frac=0.25, recovery_span=0.4),
        LockState(track_id=7, bbox_norm=(0.60, 0.55, 0.04, 0.12),
                  status="SEARCHING", frames_since_seen=5,
                  last_velocity=(0.001, -0.002)),
        12,
        dict(budget_inf_per_s=1000.0, fps=30.0),
    ),
    (
        "lost_with_track_id_4k",
        3840, 2160,
        dict(discovery_period=15, discovery_grid=(3, 2),
             recovery_grid=(3, 3), max_zoom=2.0, target_model_h=40.0,
             roi_margin_frac=0.25, recovery_span=0.4),
        LockState(track_id=99, bbox_norm=(0.50, 0.50, 0.05, 0.15),
                  status="LOST", frames_since_seen=20, last_velocity=(0.0, 0.0)),
        0,
        dict(budget_inf_per_s=1000.0, fps=30.0),
    ),
    (
        "lost_no_track_id_4k_off_cadence",
        3840, 2160,
        dict(discovery_period=15, discovery_grid=(3, 2),
             recovery_grid=(3, 3), max_zoom=2.0, target_model_h=40.0,
             roi_margin_frac=0.25, recovery_span=0.4),
        LockState(track_id=None, bbox_norm=(0.0, 0.0, 0.0, 0.0),
                  status="LOST", frames_since_seen=999, last_velocity=(0.0, 0.0)),
        3,
        dict(budget_inf_per_s=1000.0, fps=30.0),
    ),
    (
        "tracking_tight_budget_4k",
        3840, 2160,
        dict(discovery_period=15, discovery_grid=(3, 2),
             recovery_grid=(3, 3), max_zoom=2.0, target_model_h=40.0,
             roi_margin_frac=0.25, recovery_span=0.4),
        LockState(track_id=42, bbox_norm=(0.45, 0.40, 0.05, 0.15),
                  status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0)),
        0,
        dict(budget_inf_per_s=90.0, fps=30.0),   # 3 tiles/frame: ROI + first 2 discovery
    ),
    (
        "tracking_2k_source",
        1920, 1080,
        dict(discovery_period=10, discovery_grid=(2, 2),
             recovery_grid=(2, 2), max_zoom=2.0, target_model_h=40.0,
             roi_margin_frac=0.25, recovery_span=0.5),
        LockState(track_id=1, bbox_norm=(0.30, 0.30, 0.10, 0.20),
                  status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0)),
        0,
        dict(budget_inf_per_s=1000.0, fps=30.0),
    ),
]


@pytest.mark.parametrize(
    "desc,src_w,src_h,sched_kwargs,lock,frame_idx,meter_args",
    SCENARIOS,
    ids=[s[0] for s in SCENARIOS],
)
def test_composed_scheduler_matches_legacy(
    desc, src_w, src_h, sched_kwargs, lock, frame_idx, meter_args,
):
    from dynamic_tiling.scheduler import TileScheduler as LegacyScheduler

    legacy = LegacyScheduler(src_w, src_h, **sched_kwargs)
    legacy_out = legacy.decide(lock, frame_idx, BudgetMeter(**meter_args))

    new = TileScheduler(
        emitters=[
            TrackROIEmitter(
                max_zoom=sched_kwargs["max_zoom"],
                target_model_h=sched_kwargs["target_model_h"],
                roi_margin_frac=sched_kwargs["roi_margin_frac"],
            ),
            DiscoveryGridEmitter(
                grid=sched_kwargs["discovery_grid"],
                period=sched_kwargs["discovery_period"],
                mode="m",
            ),
            RecoveryGridEmitter(
                grid=sched_kwargs["recovery_grid"],
                span=sched_kwargs["recovery_span"],
                mode="s",
            ),
        ],
        modifiers=[BudgetTrimModifier()],
    )
    new_out = new.decide(src_w, src_h, lock, frame_idx, BudgetMeter(**meter_args))

    assert new_out == legacy_out, (
        f"[{desc}] scheduler outputs diverged.\n"
        f"  legacy: {legacy_out}\n"
        f"  new   : {new_out}"
    )
```

- [ ] **Step 2: Run the test.**

Run:
```bash
pytest hailo_tiling/tests/test_scheduler_legacy_parity.py -v
```
Expected: 7 passed.

If any case fails, the diverging scenario is the bug. The legacy branch order is:
1. If `status in {SEARCHING, LOST}` AND `track_id is not None`: emit recovery grid only.
2. Else: emit ROI (if TRACKING) + discovery (if on cadence).

The new composition (`TrackROI`, `DiscoveryGrid`, `RecoveryGrid`) lets the three emitters self-guard their conditions and yields the same result because:
- `TrackROIEmitter` returns `[]` unless `status == TRACKING` (so when the legacy is in the recovery branch, TrackROI emits nothing).
- `RecoveryGridEmitter` returns `[]` unless `status in {SEARCHING, LOST}` and `track_id is not None` (so when the legacy is in the normal branch, Recovery emits nothing).
- `DiscoveryGridEmitter` returns `[]` off-cadence — and the legacy puts discovery in the *normal* branch only, but: when `status in {SEARCHING, LOST}` and `track_id is not None`, the legacy returns *only* the recovery grid; the new path would also try to emit discovery on cadence. **This case is a parity risk.** If the parity test reveals this, the fix is to add a `discover-only-when-not-recovering` condition to `DiscoveryGridEmitter` OR (cleaner) make `DiscoveryGridEmitter` skip when `status in {SEARCHING, LOST}` and `track_id is not None`. Add that guard and re-run.

- [ ] **Step 3: If parity failed on the SEARCHING / LOST-with-track-id scenarios, harden `DiscoveryGridEmitter`.**

(Only execute Steps 3a–3c if Step 2 reported a failure on `searching_4k` or `lost_with_track_id_4k`.)

- [ ] **Step 3a:** Add the guard to `hailo_tiling/emitters/discovery_grid.py`:

Replace the `emit` method with:

```python
    def emit(self, src_w, src_h, lock, frame_idx, meter) -> list[CropRect]:
        # Legacy parity: when recovery is active (SEARCHING/LOST with a known
        # track), the legacy scheduler emits only the recovery grid. Mirror
        # that here by suppressing discovery in the same condition.
        if lock.status in ("SEARCHING", "LOST") and lock.track_id is not None:
            return []
        if frame_idx % self.period != 0:
            return []
        return _grid_full(src_w, src_h, self.gx, self.gy, self.mode)
```

- [ ] **Step 3b:** Re-run the parity test.

Run:
```bash
pytest hailo_tiling/tests/test_scheduler_legacy_parity.py -v
```
Expected: 7 passed.

- [ ] **Step 3c:** Re-run the existing discovery-grid unit tests to confirm no regression.

Run:
```bash
pytest hailo_tiling/tests/test_emitter_discovery_grid.py -v
```
Expected: 3 passed.

- [ ] **Step 4: Run the full hailo_tiling test suite to confirm everything still passes.**

Run:
```bash
pytest hailo_tiling/tests -v
```
Expected: all tests pass (types, budget, scheduler protocols, three emitter suites, one modifier suite, and the parity suite).

- [ ] **Step 5: Commit.**

```bash
git add hailo_tiling/tests/test_scheduler_legacy_parity.py
# If Step 3a was needed, also stage the discovery_grid.py change:
git add hailo_tiling/emitters/discovery_grid.py
git commit -m "hailo_tiling: golden-file parity test vs dynamic_tiling.TileScheduler"
```

---

## Task 10: Top-level public-API re-exports

Make the common entry points importable as `from hailo_tiling import TileScheduler, DiscoveryGridEmitter, ...` so downstream consumers don't have to know the submodule layout.

**Files:**
- Modify: `hailo_tiling/__init__.py`
- Create: `hailo_tiling/tests/test_public_api.py`

- [ ] **Step 1: Write the failing API test.**

```python
# hailo_tiling/tests/test_public_api.py
"""Public re-exports — fail fast if a class moves and we forget to update __init__."""
from __future__ import annotations


def test_top_level_imports():
    import hailo_tiling as ht
    # Types
    assert ht.CropRect is not None
    assert ht.Det is not None
    assert ht.LockState is not None
    # Scheduler & protocols
    assert ht.TileScheduler is not None
    assert ht.TileEmitter is not None
    assert ht.TileModifier is not None
    # Emitters
    assert ht.DiscoveryGridEmitter is not None
    assert ht.TrackROIEmitter is not None
    assert ht.RecoveryGridEmitter is not None
    # Modifiers
    assert ht.BudgetTrimModifier is not None
    # Budget
    assert ht.BudgetMeter is not None
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_public_api.py -v
```
Expected: FAIL (AttributeError on `ht.CropRect` etc.).

- [ ] **Step 3: Expand `hailo_tiling/__init__.py`.**

```python
# hailo_tiling/__init__.py
"""hailo_tiling — reusable dynamic-tiling library for Hailo inference pipelines.

See docs/superpowers/specs/2026-05-28-tiling-library-design.md.
"""
__version__ = "0.1.0.dev0"

from .budget import BudgetMeter
from .emitters import DiscoveryGridEmitter, RecoveryGridEmitter, TrackROIEmitter
from .modifiers import BudgetTrimModifier
from .scheduler import TileEmitter, TileModifier, TileScheduler
from .types import (
    MODEL_ASPECT,
    MODEL_H,
    MODEL_W,
    CropRect,
    Det,
    LockState,
    ScheduledTile,
    TargetState,
)

__all__ = [
    "__version__",
    "BudgetMeter",
    "CropRect",
    "Det",
    "LockState",
    "TargetState",
    "ScheduledTile",
    "MODEL_W",
    "MODEL_H",
    "MODEL_ASPECT",
    "TileScheduler",
    "TileEmitter",
    "TileModifier",
    "DiscoveryGridEmitter",
    "TrackROIEmitter",
    "RecoveryGridEmitter",
    "BudgetTrimModifier",
]
```

- [ ] **Step 4: Run the test.**

Run:
```bash
pytest hailo_tiling/tests/test_public_api.py -v
```
Expected: 1 passed.

- [ ] **Step 5: Run the full suite.**

Run:
```bash
pytest hailo_tiling/tests -v
```
Expected: all tests pass (≥ 25 across the plan's test files).

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/__init__.py hailo_tiling/tests/test_public_api.py
git commit -m "hailo_tiling: top-level public-API re-exports"
```

---

## Task 11: Editable install + final smoke test

Make the library installable into the project venv so downstream plans can `from hailo_tiling import …` outside the source tree.

**Files:**
- No source changes. One smoke-test script kept on disk (under `scripts/`) so it can be re-run later.
- Create: `scripts/smoke_hailo_tiling.py`

- [ ] **Step 1: Install the package editable into the project venv.**

Run:
```bash
./hailo-apps/venv_hailo_apps/bin/pip install -e . 2>&1 \
  | ~/.claude/token-filter/token-filter.sh --cmd "pip install -e ."
```

Expected: "Successfully installed hailo_tiling-0.1.0.dev0" (or a successful re-install line).

- [ ] **Step 2: Write the smoke test script.**

```python
# scripts/smoke_hailo_tiling.py
"""End-to-end smoke test for hailo_tiling.

Runs a composed TileScheduler over a fixed scenario and prints the resulting
CropRects. Intended to be re-runnable as a CI sanity check.

Usage:
    python scripts/smoke_hailo_tiling.py
"""
from __future__ import annotations

from hailo_tiling import (
    BudgetMeter,
    BudgetTrimModifier,
    DiscoveryGridEmitter,
    LockState,
    RecoveryGridEmitter,
    TileScheduler,
    TrackROIEmitter,
)


def main() -> int:
    src_w, src_h = 3840, 2160
    scheduler = TileScheduler(
        emitters=[
            TrackROIEmitter(max_zoom=2.0, target_model_h=40.0, roi_margin_frac=0.25),
            DiscoveryGridEmitter(grid=(3, 2), period=15, mode="m"),
            RecoveryGridEmitter(grid=(3, 3), span=0.4, mode="s"),
        ],
        modifiers=[BudgetTrimModifier()],
    )
    lock = LockState(
        track_id=42,
        bbox_norm=(0.45, 0.40, 0.05, 0.15),
        status="TRACKING",
        frames_since_seen=0,
        last_velocity=(0.0, 0.0),
    )
    meter = BudgetMeter(budget_inf_per_s=300.0, fps=30.0)
    crops = scheduler.decide(src_w, src_h, lock, frame_idx=0, meter=meter)
    print(f"emitted {len(crops)} crops:")
    for c in crops:
        print(f"  {c}")
    return 0 if crops else 1


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 3: Run the smoke test.**

Run:
```bash
python scripts/smoke_hailo_tiling.py
```
Expected: prints `emitted 7 crops:` followed by 7 `CropRect(...)` lines (1 ROI + 6 from the 3×2 discovery grid, capped by the budget of 300/30 = 10 tiles/frame). Exit code 0.

- [ ] **Step 4: Run the full test suite one last time.**

Run:
```bash
pytest hailo_tiling/tests dynamic_tiling/tests -v
```
Expected: every test passes — `hailo_tiling` suite plus the existing `dynamic_tiling` suite (through the shims).

- [ ] **Step 5: Commit.**

```bash
git add scripts/smoke_hailo_tiling.py
git commit -m "hailo_tiling: smoke test + editable install confirmed"
```

---

## Task 12: Plans index document (so we don't lose the follow-ups)

**Files:**
- Create: `docs/superpowers/plans/INDEX.md`

- [ ] **Step 1: Write the index.**

```markdown
# Implementation Plans Index

Tracks the decomposed plans derived from the master spec:
`docs/superpowers/specs/2026-05-28-tiling-library-design.md`.

| #   | Plan                                                  | Spec phases | Status     |
|-----|-------------------------------------------------------|-------------|------------|
| 1   | `2026-05-28-hailo-tiling-scaffold-and-scheduler-refactor.md` | 1, 2        | in flight  |
| 2   | Telemetry + new modifiers + backends ABC              | 3, 4, 5     | not started |
| 3   | FOV emulation source-data prep                        | 6           | not started |
| 4   | Cache schema + Python cache layer                     | 7           | not started |
| 5   | GStreamer cache plugins + hailo-apps-core patches     | 8, 14       | not started |
| 6   | GstCropperBackend + ablation harness                  | 9, 10       | not started |
| 7   | Telemetry import (ULG/SRT) + visualizer               | 11, 12      | not started |
| 8   | Drone-follow migration + RPI-GS data collection       | 13, 16      | not started |
| 9   | Paper-with-code artifacts                             | 15          | not started |
| (10)| DJI optical-zoom maximum-range bonus shoot            | 17          | bonus / ops |

Update the **Status** column as plans land. When a plan finishes, set its
status to `done` and bump the next plan to `in flight`.
```

- [ ] **Step 2: Commit.**

```bash
git add docs/superpowers/plans/INDEX.md
git commit -m "plans: add INDEX tracking the 9-plan decomposition + bonus"
```

---

## Plan-wide success criteria (self-check before declaring this plan done)

- [ ] `hailo_tiling/` package imports cleanly: `python -c "import hailo_tiling; print(hailo_tiling.__version__)"` prints `0.1.0.dev0`.
- [ ] `pytest hailo_tiling/tests -v` reports **all** tests passing (≥ 25 cases across types, budget, protocols, three emitter suites, one modifier suite, parity, public-API).
- [ ] `pytest dynamic_tiling/tests -v` reports the existing legacy tests still passing (the types/budget shims keep them working).
- [ ] `scripts/smoke_hailo_tiling.py` exits 0 and prints 7 CropRects.
- [ ] `pip show hailo_tiling` reports the package installed from this repo in editable mode.
- [ ] No new dependencies were added to `pyproject.toml` beyond `pytest` in the optional `test` extras.
- [ ] `dynamic_tiling/scheduler.py` is **unchanged** — the legacy code path is intact.
- [ ] All commits are on the current branch (`tiling-benchmark`); no force-pushes; each task ended with at least one commit.

---

## Out of scope for this plan (handled later)

- `dynamic_tiling.aggregator`, `dynamic_tiling.inference`, the live HEF backend → Plan 2.
- `MultiTargetTileScheduler` — multi-target work stays in `dynamic_tiling/` until v2.
- Telemetry providers, ASAHI modifier, altitude modifier → Plan 2.
- **Richer Emitter/Modifier protocol signatures** — the spec (Section 3.2) defines `emit(ctx: FrameContext, track: TrackState, telemetry: TelemetrySnapshot)`. This plan keeps the leaner `emit(src_w, src_h, lock, frame_idx, meter)` to maintain byte-parity with the legacy scheduler. The signature widens in Plan 2 once `FrameContext`/`TrackState`/`TelemetrySnapshot` types exist. `LockState` is also renamed to `TrackState` in Plan 2.
- **CI setup** (spec Phase 1) — adding GitHub Actions / GitLab CI for `pytest hailo_tiling/tests` is a small follow-up commit, not bundled in this plan because the repo doesn't yet have a Python-only CI workflow.
- Cache layer (SQLite, CachingBackend, ReplayBackend) → Plan 4.
- GStreamer plugins → Plan 5.
- Ablation harness, visualizer, paper artifacts → later plans.
- Removing `dynamic_tiling/` and migrating callers (`run_dynamic.py`, `compare_baselines.py`) → final cleanup plan after Plans 2-8 are all green.
