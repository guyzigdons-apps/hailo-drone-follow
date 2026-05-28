# Plan 2: Telemetry + New Modifiers + Backends ABC + Aggregator Skeleton — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Land spec phases 3 (telemetry), 4 (two new modifiers), and 5 (backends ABC + aggregator skeleton) of `docs/superpowers/specs/2026-05-28-tiling-library-design.md`. After this plan, `hailo_tiling/` exposes a `TelemetryProvider` ABC + three implementations, two new modifiers (`AltitudeZoomModifier`, `AdaptiveSliceSizingModifier`), an `InferenceBackend` ABC with `HefBackend` lifted from `dynamic_tiling/inference.py`, and the four-piece aggregator scaffolding (NMS, boundary strip, memory ABC, top-level `Aggregator`). The Plan 1 scheduler + emitters + `BudgetTrimModifier` keep working byte-identically; nothing in `dynamic_tiling/` changes behaviourally.

**Architecture:**
- Telemetry boundary is the only thing that knows about MAVSDK. `MavsdkTelemetry` lazy-imports the SDK so the library installs cleanly without it. All `TelemetrySnapshot` fields are `Optional` and consumers degrade gracefully on `None`.
- Modifiers gain a richer signature in this plan: `modify(tiles, src_w, src_h, lock, frame_idx, meter, telemetry)`. `BudgetTrimModifier` is updated to accept (and ignore) the new arg via `**kwargs`-style tolerance OR a `telemetry=None` default. The `TileScheduler` is taught to thread a `TelemetrySnapshot` through every call, defaulting to `_NULL_SNAPSHOT` if none is provided so Plan 1 tests don't break.
- `InferenceBackend.infer(frame, crops) → list[list[Det]]` is the spec-correct signature (one call covers multiple crops in order). `HefBackend` reuses the lifted per-crop loop internally so the on-chip code path is unchanged. A `MockBackend` fixture in tests returns canned per-crop dets without any Hailo chip.
- The aggregator ships as a *skeleton*: NMS lifted verbatim from `dynamic_tiling/aggregator.py`, a Python `BoundaryStripFilter` whose semantics match the C++ `remove_exceeded_bboxes(border_threshold)` documented in `tiling_benchmark/PERF_REPORT.md` §8, and a `DetectionMemory` ABC with a `NoOpMemory` default. Full carry-forward memory is v2.

**Tech Stack:** Python 3.10+, pytest. New optional extras: `mavsdk` (already declared in `pyproject.toml` from Plan 1). No new hard dependencies. The venv at `./hailo-apps/venv_hailo_apps` is reused.

**Spec reference:** `docs/superpowers/specs/2026-05-28-tiling-library-design.md` §3.3, §3.4, §3.5 (data flow), §4, §5 (error handling), §11 phases 3–5.

**Branch / starting HEAD:** `tiling-benchmark` @ `cea704d` (post Plan 1).

---

## File Structure

**Files this plan creates:**

```
hailo_tiling/
  telemetry/
    __init__.py                     # re-exports
    provider.py                     # TelemetrySnapshot dataclass + TelemetryProvider ABC + _NULL_SNAPSHOT
    static.py                       # StaticTelemetry
    mavsdk.py                       # MavsdkTelemetry (lazy import; graceful degradation)
    recorded.py                     # RecordedTelemetry (JSONL replay)
  modifiers/
    altitude_zoom.py                # AltitudeZoomModifier
    adaptive_sizing.py              # AdaptiveSliceSizingModifier (ASAHI)
  backends/
    __init__.py                     # re-exports
    backend.py                      # InferenceBackend ABC
    hef.py                          # HefBackend (lifted from dynamic_tiling/inference.py)
  aggregator/
    __init__.py                     # re-exports
    nms.py                          # _iou, nms, map_to_source (lifted from dynamic_tiling/aggregator.py)
    boundary_strip.py               # BoundaryStripFilter (default threshold 0.005)
    memory.py                       # DetectionMemory ABC + NoOpMemory default
    aggregator.py                   # Aggregator.aggregate(...) composition
  tests/
    test_telemetry_snapshot.py
    test_telemetry_provider_abc.py
    test_telemetry_static.py
    test_telemetry_recorded.py
    test_telemetry_mavsdk.py        # skipped when `mavsdk` is not installed
    test_modifier_altitude_zoom.py
    test_modifier_adaptive_sizing.py
    test_backend_abc.py
    test_backend_hef_shim.py        # only exercises the shim contract; HEF init is mocked
    test_aggregator_nms.py
    test_aggregator_boundary_strip.py
    test_aggregator_memory.py
    test_aggregator_integration.py
```

**Files this plan modifies:**

- `hailo_tiling/modifiers/__init__.py` — add the two new modifier exports.
- `hailo_tiling/modifiers/budget_trim.py` — widen the `modify(...)` signature to accept (and ignore) `telemetry`.
- `hailo_tiling/scheduler.py` — thread `telemetry: TelemetrySnapshot | None = None` through `decide(...)`; default to `_NULL_SNAPSHOT` when callers don't supply one; widen the `TileEmitter`/`TileModifier` Protocols to include the new keyword arg.
- `hailo_tiling/emitters/discovery_grid.py`, `hailo_tiling/emitters/track_roi.py`, `hailo_tiling/emitters/recovery.py` — accept (and ignore) the new `telemetry` arg so the emitter Protocol stays satisfied. No behavioural change.
- `hailo_tiling/__init__.py` — re-export the new public API.
- `hailo_tiling/tests/conftest.py` — add `null_snapshot`, `low_altitude_snapshot`, `high_altitude_snapshot` fixtures and a `make_mock_backend` factory.
- `hailo_tiling/tests/test_scheduler_protocols.py` — update the dummy emitter/modifier signatures to accept `telemetry`.
- `hailo_tiling/tests/test_scheduler_legacy_parity.py`, `hailo_tiling/tests/test_emitter_discovery_grid.py`, `hailo_tiling/tests/test_emitter_track_roi.py`, `hailo_tiling/tests/test_emitter_recovery.py`, `hailo_tiling/tests/test_modifier_budget_trim.py` — pass `telemetry=_NULL_SNAPSHOT` or rely on the default; expected to still pass byte-identically.
- `dynamic_tiling/inference.py` — becomes a re-export shim that delegates `HefBackend` and `ReplayBackend` to `hailo_tiling.backends.hef` (HefBackend) and to a tiny `dynamic_tiling.inference._LegacyReplayShim` that preserves the existing single-crop `.infer(frame, crop, frame_idx)` API used by `dynamic_tiling.tests.test_inference`. (`ReplayBackend` itself is NOT moved — its single-crop, per-frame-canned API is Plan-1-style. The full multi-crop `ReplayBackend` lives later in Plan 4.)
- `docs/superpowers/plans/INDEX.md` — flip Plan 2 status from `not started` → `in flight` at start of work; → `done` at end.

**Files this plan does NOT touch:**

- `dynamic_tiling/scheduler.py` — still the legacy parity ground truth. `MultiTargetTileScheduler` stays here for v2.
- `dynamic_tiling/aggregator.py` — kept as-is for `dynamic_tiling.tests.test_aggregator` to keep passing. The new `hailo_tiling/aggregator/nms.py` is a verbatim port; both files coexist until a future cleanup plan removes the duplicate.
- `dynamic_tiling/replay.py`, `dynamic_tiling/run_dynamic.py`, `dynamic_tiling/compare_baselines.py`, `dynamic_tiling/score.py`, `dynamic_tiling/gt_track.py`, `dynamic_tiling/target_lock.py` — out of scope. They keep using their own imports.
- The discovery-grid recovery-active guard at `hailo_tiling/emitters/discovery_grid.py:43-47` — left in place (see Open Questions §1).
- `tiling_benchmark/` Python — left alone; this plan does not consume `analyze_pxt.containment_merge` or related filters.

---

## Pre-flight: virtual environment

All commands assume the project venv is active. If you've never set it up, run `source setup_env.sh` once at the start of the session. After that, use the direct binaries — `/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo-apps/venv_hailo_apps/bin/python` and `.../bin/pytest` — because shell state doesn't persist between tool calls.

For brevity in this plan, paths are written as `python` and `pytest`; the executor should resolve them to the venv binaries.

**Quick sanity check before starting:**
```bash
python -c "import hailo_tiling; print(hailo_tiling.__version__)"
pytest hailo_tiling/tests dynamic_tiling/tests -q
```
Expected: `0.1.0.dev0`, then 78+ tests passing (Plan 1 baseline).

---

## Task 1: `TelemetrySnapshot` dataclass + tests

The frozen dataclass that flows through every emitter and modifier. All fields are `Optional` per spec §3.3. A module-level `_NULL_SNAPSHOT` constant gives modifiers a sentinel they can compare against.

**Files:**
- Create: `hailo_tiling/telemetry/__init__.py`
- Create: `hailo_tiling/telemetry/provider.py` (snapshot dataclass + ABC stub, ABC body filled in Task 2)
- Create: `hailo_tiling/tests/test_telemetry_snapshot.py`

- [ ] **Step 1: Create the `telemetry` subpackage skeleton.**

```bash
mkdir -p hailo_tiling/telemetry
```

```python
# hailo_tiling/telemetry/__init__.py
"""Telemetry abstraction — the only seam that knows what MAVSDK is.

See docs/superpowers/specs/2026-05-28-tiling-library-design.md §3.3.
"""
from .provider import TelemetryProvider, TelemetrySnapshot, NULL_SNAPSHOT  # noqa: F401
```

- [ ] **Step 2: Write the failing dataclass test.**

```python
# hailo_tiling/tests/test_telemetry_snapshot.py
"""TelemetrySnapshot — frozen dataclass with optional fields and a NULL sentinel."""
from __future__ import annotations

import dataclasses

import pytest

from hailo_tiling.telemetry import TelemetrySnapshot, NULL_SNAPSHOT


def test_snapshot_is_frozen():
    s = TelemetrySnapshot(timestamp=0.0)
    with pytest.raises(dataclasses.FrozenInstanceError):
        s.timestamp = 1.0  # type: ignore[misc]


def test_snapshot_defaults_are_none_except_timestamp():
    s = TelemetrySnapshot(timestamp=12.5)
    assert s.timestamp == 12.5
    assert s.altitude_agl_m is None
    assert s.yaw_rate_rad_s is None
    assert s.velocity_world is None
    assert s.attitude_quat is None


def test_snapshot_accepts_all_fields():
    s = TelemetrySnapshot(
        altitude_agl_m=12.5,
        yaw_rate_rad_s=0.1,
        velocity_world=(1.0, -2.0, 0.5),
        attitude_quat=(1.0, 0.0, 0.0, 0.0),
        timestamp=3.14,
    )
    assert s.altitude_agl_m == 12.5
    assert s.velocity_world == (1.0, -2.0, 0.5)


def test_null_snapshot_is_a_singleton_constant():
    assert NULL_SNAPSHOT.timestamp == 0.0
    assert NULL_SNAPSHOT.altitude_agl_m is None
    # Identity is required so modifiers can short-circuit on `is NULL_SNAPSHOT`.
    from hailo_tiling.telemetry import NULL_SNAPSHOT as also_null
    assert also_null is NULL_SNAPSHOT
```

- [ ] **Step 3: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_telemetry_snapshot.py -v
```
Expected: `ModuleNotFoundError: No module named 'hailo_tiling.telemetry'` or `ImportError: cannot import name 'TelemetrySnapshot' ...`.

- [ ] **Step 4: Implement `TelemetrySnapshot` + `NULL_SNAPSHOT`.**

```python
# hailo_tiling/telemetry/provider.py
"""TelemetryProvider ABC + TelemetrySnapshot dataclass.

Fields match the spec §3.3 — every domain field is Optional so consumers
gracefully degrade when a stream is missing (offline runs, MAVSDK absent,
sensor dropout). The `timestamp` field is always populated (defaults to 0.0
for the NULL sentinel).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class TelemetrySnapshot:
    """Per-frame telemetry slice consumed by tiling modifiers and tracker components.

    See spec §3.3. Every field except `timestamp` is Optional.
    """
    altitude_agl_m: Optional[float] = None
    yaw_rate_rad_s: Optional[float] = None
    velocity_world: Optional[Tuple[float, float, float]] = None
    attitude_quat: Optional[Tuple[float, float, float, float]] = None
    timestamp: float = 0.0


# Sentinel used by the scheduler when no provider is wired.
# Modifiers MAY short-circuit on `telemetry is NULL_SNAPSHOT` but must also
# accept ordinary all-None snapshots produced by a degraded real provider.
NULL_SNAPSHOT = TelemetrySnapshot()


class TelemetryProvider(ABC):
    """Returns a TelemetrySnapshot for a given monotonic time.

    Concrete implementations: StaticTelemetry, RecordedTelemetry, MavsdkTelemetry.
    """

    @abstractmethod
    def snapshot(self, t: float) -> TelemetrySnapshot:
        """Return the snapshot at (or nearest to) monotonic time `t` seconds."""
```

- [ ] **Step 5: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_telemetry_snapshot.py -v
```
Expected: 4 passed.

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/telemetry/__init__.py \
        hailo_tiling/telemetry/provider.py \
        hailo_tiling/tests/test_telemetry_snapshot.py
git commit -m "hailo_tiling: TelemetrySnapshot dataclass + NULL_SNAPSHOT sentinel"
```

---

## Task 2: `TelemetryProvider` ABC — interface contract test

The ABC body is already in `provider.py` from Task 1. This task adds an interface-contract test that verifies subclass instantiation fails without `snapshot()`, and that the contract returns `TelemetrySnapshot`.

**Files:**
- Create: `hailo_tiling/tests/test_telemetry_provider_abc.py`

- [ ] **Step 1: Write the failing contract test.**

```python
# hailo_tiling/tests/test_telemetry_provider_abc.py
"""TelemetryProvider — ABC contract."""
from __future__ import annotations

import pytest

from hailo_tiling.telemetry import TelemetryProvider, TelemetrySnapshot


def test_cannot_instantiate_bare_abc():
    with pytest.raises(TypeError):
        TelemetryProvider()  # type: ignore[abstract]


def test_subclass_without_snapshot_method_fails():
    class _BadProvider(TelemetryProvider):
        pass  # missing snapshot()
    with pytest.raises(TypeError):
        _BadProvider()  # type: ignore[abstract]


def test_subclass_returns_snapshot():
    class _GoodProvider(TelemetryProvider):
        def snapshot(self, t):
            return TelemetrySnapshot(timestamp=t, altitude_agl_m=10.0)

    p = _GoodProvider()
    s = p.snapshot(1.5)
    assert isinstance(s, TelemetrySnapshot)
    assert s.timestamp == 1.5
    assert s.altitude_agl_m == 10.0
```

- [ ] **Step 2: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_telemetry_provider_abc.py -v
```
Expected: 3 passed. (The ABC is already implemented; this task is a contract-pinning test.)

- [ ] **Step 3: Commit.**

```bash
git add hailo_tiling/tests/test_telemetry_provider_abc.py
git commit -m "hailo_tiling: pin TelemetryProvider ABC contract with abstract-method tests"
```

---

## Task 3: `StaticTelemetry` — constant snapshot

Simplest concrete provider: stores one snapshot, returns it for every `t` (with the request's `t` substituted into `.timestamp` so callers can still tell snapshots apart by time).

**Files:**
- Create: `hailo_tiling/telemetry/static.py`
- Modify: `hailo_tiling/telemetry/__init__.py` (add export)
- Create: `hailo_tiling/tests/test_telemetry_static.py`

- [ ] **Step 1: Write the failing test.**

```python
# hailo_tiling/tests/test_telemetry_static.py
"""StaticTelemetry — returns a fixed snapshot, timestamp substituted in."""
from __future__ import annotations

from hailo_tiling.telemetry import StaticTelemetry, TelemetrySnapshot


def test_static_returns_configured_values():
    p = StaticTelemetry(altitude_agl_m=25.0, yaw_rate_rad_s=0.0)
    s = p.snapshot(0.0)
    assert s.altitude_agl_m == 25.0
    assert s.yaw_rate_rad_s == 0.0
    assert s.timestamp == 0.0


def test_static_substitutes_timestamp_on_each_call():
    p = StaticTelemetry(altitude_agl_m=10.0)
    s0 = p.snapshot(0.0)
    s5 = p.snapshot(5.5)
    assert s0.timestamp == 0.0 and s5.timestamp == 5.5
    # Domain fields unchanged across calls.
    assert s0.altitude_agl_m == s5.altitude_agl_m == 10.0


def test_static_defaults_all_none():
    p = StaticTelemetry()
    s = p.snapshot(0.0)
    assert s.altitude_agl_m is None
    assert s.yaw_rate_rad_s is None
    assert s.velocity_world is None
    assert s.attitude_quat is None


def test_static_snapshot_is_telemetry_snapshot():
    p = StaticTelemetry(altitude_agl_m=1.0)
    assert isinstance(p.snapshot(0.0), TelemetrySnapshot)
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_telemetry_static.py -v
```
Expected: `ImportError: cannot import name 'StaticTelemetry' from 'hailo_tiling.telemetry'`.

- [ ] **Step 3: Implement `StaticTelemetry`.**

```python
# hailo_tiling/telemetry/static.py
"""Constant-valued telemetry provider for tests and offline runs."""
from __future__ import annotations

from dataclasses import replace
from typing import Optional, Tuple

from .provider import TelemetryProvider, TelemetrySnapshot


class StaticTelemetry(TelemetryProvider):
    """Provider that returns the same snapshot for every t.

    The `timestamp` of the returned snapshot is the requested `t`, so callers
    that key on timestamp (e.g., for caching) still see distinct snapshots.
    """

    def __init__(
        self,
        altitude_agl_m: Optional[float] = None,
        yaw_rate_rad_s: Optional[float] = None,
        velocity_world: Optional[Tuple[float, float, float]] = None,
        attitude_quat: Optional[Tuple[float, float, float, float]] = None,
    ):
        self._template = TelemetrySnapshot(
            altitude_agl_m=altitude_agl_m,
            yaw_rate_rad_s=yaw_rate_rad_s,
            velocity_world=velocity_world,
            attitude_quat=attitude_quat,
            timestamp=0.0,
        )

    def snapshot(self, t: float) -> TelemetrySnapshot:
        return replace(self._template, timestamp=t)
```

- [ ] **Step 4: Add the export.**

Modify `hailo_tiling/telemetry/__init__.py` to add:

```python
from .static import StaticTelemetry  # noqa: F401
```

- [ ] **Step 5: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_telemetry_static.py -v
```
Expected: 4 passed.

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/telemetry/static.py \
        hailo_tiling/telemetry/__init__.py \
        hailo_tiling/tests/test_telemetry_static.py
git commit -m "hailo_tiling: StaticTelemetry — constant snapshot provider"
```

---

## Task 4: `RecordedTelemetry` — JSONL replay

Reads a JSONL file (one JSON object per line, fields matching `TelemetrySnapshot` plus `timestamp`). On `snapshot(t)`, returns the latest record with `timestamp <= t`. Used for offline ablation runs against a captured flight log.

**Files:**
- Create: `hailo_tiling/telemetry/recorded.py`
- Modify: `hailo_tiling/telemetry/__init__.py` (add export)
- Create: `hailo_tiling/tests/test_telemetry_recorded.py`

- [ ] **Step 1: Write the failing test.**

```python
# hailo_tiling/tests/test_telemetry_recorded.py
"""RecordedTelemetry — JSONL replay provider."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from hailo_tiling.telemetry import RecordedTelemetry, TelemetrySnapshot


def _write_jsonl(tmp_path: Path, rows: list[dict]) -> Path:
    p = tmp_path / "telem.jsonl"
    p.write_text("\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8")
    return p


def test_returns_nearest_le_record(tmp_path):
    rows = [
        {"timestamp": 0.0, "altitude_agl_m": 5.0},
        {"timestamp": 1.0, "altitude_agl_m": 7.5},
        {"timestamp": 2.0, "altitude_agl_m": 10.0},
    ]
    p = RecordedTelemetry.from_path(_write_jsonl(tmp_path, rows))
    assert p.snapshot(0.0).altitude_agl_m == 5.0
    assert p.snapshot(0.5).altitude_agl_m == 5.0   # falls back to last <= t
    assert p.snapshot(1.0).altitude_agl_m == 7.5
    assert p.snapshot(1.9).altitude_agl_m == 7.5
    assert p.snapshot(2.0).altitude_agl_m == 10.0
    assert p.snapshot(99.0).altitude_agl_m == 10.0  # past end -> last row


def test_returns_null_before_first_record(tmp_path):
    """Before the first record, return NULL_SNAPSHOT-style (timestamp=t, all None)."""
    rows = [{"timestamp": 5.0, "altitude_agl_m": 10.0}]
    p = RecordedTelemetry.from_path(_write_jsonl(tmp_path, rows))
    s = p.snapshot(0.0)
    assert s.altitude_agl_m is None
    assert s.timestamp == 0.0


def test_velocity_and_attitude_round_trip(tmp_path):
    rows = [{
        "timestamp": 0.0,
        "velocity_world": [1.0, -2.0, 0.5],
        "attitude_quat": [1.0, 0.0, 0.0, 0.0],
        "yaw_rate_rad_s": 0.1,
    }]
    p = RecordedTelemetry.from_path(_write_jsonl(tmp_path, rows))
    s = p.snapshot(0.0)
    assert s.velocity_world == (1.0, -2.0, 0.5)
    assert s.attitude_quat == (1.0, 0.0, 0.0, 0.0)
    assert s.yaw_rate_rad_s == 0.1


def test_skips_malformed_lines(tmp_path):
    p = tmp_path / "bad.jsonl"
    p.write_text(
        '{"timestamp": 0.0, "altitude_agl_m": 1.0}\n'
        'not a json line\n'
        '\n'
        '{"timestamp": 1.0, "altitude_agl_m": 2.0}\n',
        encoding="utf-8",
    )
    rt = RecordedTelemetry.from_path(p)
    assert rt.snapshot(0.0).altitude_agl_m == 1.0
    assert rt.snapshot(1.0).altitude_agl_m == 2.0


def test_empty_file_returns_null_snapshot(tmp_path):
    p = tmp_path / "empty.jsonl"
    p.write_text("", encoding="utf-8")
    rt = RecordedTelemetry.from_path(p)
    s = rt.snapshot(1.0)
    assert s.altitude_agl_m is None
    assert s.timestamp == 1.0


def test_requires_sorted_or_sorts_on_load(tmp_path):
    """Out-of-order rows must still produce monotonic-friendly lookups."""
    rows = [
        {"timestamp": 2.0, "altitude_agl_m": 20.0},
        {"timestamp": 0.0, "altitude_agl_m": 5.0},
        {"timestamp": 1.0, "altitude_agl_m": 10.0},
    ]
    rt = RecordedTelemetry.from_path(_write_jsonl(tmp_path, rows))
    assert rt.snapshot(0.5).altitude_agl_m == 5.0
    assert rt.snapshot(1.5).altitude_agl_m == 10.0
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_telemetry_recorded.py -v
```
Expected: `ImportError: cannot import name 'RecordedTelemetry'`.

- [ ] **Step 3: Implement `RecordedTelemetry`.**

```python
# hailo_tiling/telemetry/recorded.py
"""JSONL-replay telemetry provider for offline / paper-reproducibility runs."""
from __future__ import annotations

import bisect
import json
from dataclasses import replace
from pathlib import Path
from typing import Iterable

from .provider import TelemetryProvider, TelemetrySnapshot


def _row_to_snapshot(row: dict) -> TelemetrySnapshot:
    def _tuple_or_none(v):
        if v is None:
            return None
        return tuple(float(x) for x in v)
    return TelemetrySnapshot(
        altitude_agl_m=row.get("altitude_agl_m"),
        yaw_rate_rad_s=row.get("yaw_rate_rad_s"),
        velocity_world=_tuple_or_none(row.get("velocity_world")),
        attitude_quat=_tuple_or_none(row.get("attitude_quat")),
        timestamp=float(row.get("timestamp", 0.0)),
    )


class RecordedTelemetry(TelemetryProvider):
    """Provider backed by an in-memory list of TelemetrySnapshots sorted by timestamp."""

    def __init__(self, snapshots: Iterable[TelemetrySnapshot]):
        snaps = sorted(snapshots, key=lambda s: s.timestamp)
        self._timestamps = [s.timestamp for s in snaps]
        self._snaps = snaps

    @classmethod
    def from_path(cls, path: Path | str) -> "RecordedTelemetry":
        snaps: list[TelemetrySnapshot] = []
        path = Path(path)
        if path.exists():
            for line in path.read_text(encoding="utf-8").splitlines():
                line = line.strip()
                if not line:
                    continue
                try:
                    row = json.loads(line)
                except json.JSONDecodeError:
                    continue
                snaps.append(_row_to_snapshot(row))
        return cls(snaps)

    def snapshot(self, t: float) -> TelemetrySnapshot:
        if not self._snaps:
            return TelemetrySnapshot(timestamp=t)
        # Find rightmost snap with timestamp <= t.
        idx = bisect.bisect_right(self._timestamps, t) - 1
        if idx < 0:
            return TelemetrySnapshot(timestamp=t)
        return replace(self._snaps[idx], timestamp=t)
```

- [ ] **Step 4: Add the export.**

Modify `hailo_tiling/telemetry/__init__.py` to add:

```python
from .recorded import RecordedTelemetry  # noqa: F401
```

- [ ] **Step 5: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_telemetry_recorded.py -v
```
Expected: 6 passed.

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/telemetry/recorded.py \
        hailo_tiling/telemetry/__init__.py \
        hailo_tiling/tests/test_telemetry_recorded.py
git commit -m "hailo_tiling: RecordedTelemetry — JSONL replay with binary search lookup"
```

---

## Task 5: `MavsdkTelemetry` — lazy-imported MAVSDK adapter

Wraps a connected `mavsdk.System`. On construction, only the connection-URL is stashed. The actual `mavsdk` import + connection is lazy on first `snapshot()` call. If `mavsdk` is not installed, raises `ImportError` lazily with a clear message — the library still imports cleanly.

For the test, we **do not** require `mavsdk` to be installed. The test verifies:
1. Importing `MavsdkTelemetry` works even when `mavsdk` is absent.
2. Calling `snapshot()` without `mavsdk` installed raises a specific `ImportError`.
3. With `mavsdk` mocked (via `sys.modules` injection), `snapshot()` returns a sensible `TelemetrySnapshot` populated from the mock.

**Files:**
- Create: `hailo_tiling/telemetry/mavsdk.py`
- Modify: `hailo_tiling/telemetry/__init__.py` (add export)
- Create: `hailo_tiling/tests/test_telemetry_mavsdk.py`

- [ ] **Step 1: Write the failing test.**

```python
# hailo_tiling/tests/test_telemetry_mavsdk.py
"""MavsdkTelemetry — lazy MAVSDK import + graceful degradation when absent."""
from __future__ import annotations

import importlib
import sys
import types

import pytest

# The class is importable regardless of mavsdk presence.
from hailo_tiling.telemetry import MavsdkTelemetry, TelemetrySnapshot


def _mavsdk_installed() -> bool:
    try:
        import mavsdk  # noqa: F401
        return True
    except ImportError:
        return False


def test_import_works_without_mavsdk():
    """The class must be importable on a machine that doesn't have mavsdk."""
    assert MavsdkTelemetry is not None  # tautological but pins the import path


def test_construct_without_mavsdk_does_not_raise():
    """Construction is a no-op pending lazy connect; no ImportError yet."""
    t = MavsdkTelemetry(system_address="udp://:14540")
    assert isinstance(t, MavsdkTelemetry)


@pytest.mark.skipif(_mavsdk_installed(),
                    reason="mavsdk IS installed; this test verifies the absent-path")
def test_snapshot_raises_clear_error_when_mavsdk_absent():
    t = MavsdkTelemetry(system_address="udp://:14540")
    with pytest.raises(ImportError) as exc:
        t.snapshot(0.0)
    msg = str(exc.value).lower()
    assert "mavsdk" in msg
    # Hint at the install command in the error message.
    assert "pip install" in msg or "extras" in msg


def test_snapshot_with_mocked_mavsdk(monkeypatch):
    """With a faked `mavsdk` module, snapshot() returns a populated TelemetrySnapshot.

    This test does NOT exercise asyncio plumbing — MavsdkTelemetry exposes a
    `_pull_snapshot_sync` seam that the test patches. The real implementation
    uses asyncio internally; the seam is the unit boundary we test against.
    """
    captured = TelemetrySnapshot(
        altitude_agl_m=12.3,
        yaw_rate_rad_s=0.05,
        velocity_world=(1.0, -1.0, 0.2),
        attitude_quat=(1.0, 0.0, 0.0, 0.0),
        timestamp=0.0,
    )
    t = MavsdkTelemetry(system_address="udp://:14540")
    monkeypatch.setattr(t, "_pull_snapshot_sync", lambda: captured)

    s = t.snapshot(7.0)
    assert s.altitude_agl_m == 12.3
    assert s.timestamp == 7.0
    assert s.velocity_world == (1.0, -1.0, 0.2)


def test_snapshot_returns_null_fields_when_pull_returns_none(monkeypatch):
    """If the internal pull returns None (e.g., no link yet), degrade to NULL fields."""
    t = MavsdkTelemetry(system_address="udp://:14540")
    monkeypatch.setattr(t, "_pull_snapshot_sync", lambda: None)
    s = t.snapshot(3.0)
    assert s.altitude_agl_m is None
    assert s.timestamp == 3.0
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_telemetry_mavsdk.py -v
```
Expected: `ImportError: cannot import name 'MavsdkTelemetry'`.

- [ ] **Step 3: Implement `MavsdkTelemetry`.**

The implementation has a `_pull_snapshot_sync()` seam so the test can stub the asyncio interior. Real-world plumbing (subscribing to `position()`, `attitude_quaternion()`, `velocity_ned()` streams) lives behind that seam and is exercised in integration testing only.

```python
# hailo_tiling/telemetry/mavsdk.py
"""MAVSDK-backed telemetry provider.

The `mavsdk` package is imported lazily on first `_pull_snapshot_sync()` call so
the library installs cleanly on machines without it. To use this provider,
install with the optional extras: `pip install hailo_tiling[mavsdk]`.
"""
from __future__ import annotations

from dataclasses import replace
from typing import Optional

from .provider import TelemetryProvider, TelemetrySnapshot

_MAVSDK_INSTALL_HINT = (
    "MavsdkTelemetry requires the optional `mavsdk` package. "
    "Install it with: pip install 'hailo_tiling[mavsdk]'"
)


class MavsdkTelemetry(TelemetryProvider):
    """Pulls telemetry from a PX4 (or other MAVLink) endpoint via MAVSDK.

    Connection is lazy: the MAVSDK system is created and connected on first
    `snapshot()` call, not at construction time. Subsequent calls reuse the
    same connection. If the connection fails or mavsdk is not installed, the
    error is raised lazily on the first `snapshot()` call.
    """

    def __init__(self, system_address: str = "udp://:14540"):
        self.system_address = system_address
        self._system = None  # mavsdk.System | None
        self._last_pull: Optional[TelemetrySnapshot] = None

    # ----- Seam used by tests; real impl is asyncio-driven (Plan 8 fleshes
    # this out for live drone-follow integration). -----
    def _pull_snapshot_sync(self) -> Optional[TelemetrySnapshot]:
        """Pull one synchronous-feeling snapshot. Returns None if no data yet.

        Real implementation:
        1. On first call, lazy-import `mavsdk` (ImportError surfaces here).
        2. If `self._system is None`, run a short asyncio task that connects
           and subscribes to position/attitude/velocity streams; cache the
           latest values in `self._last_pull` via the stream callbacks.
        3. Return `self._last_pull` (may be None until the first stream tick).

        Plan 8 (drone-follow migration) replaces this skeleton with the real
        asyncio impl. Plan 2 ships only the seam + ImportError plumbing so
        downstream code (AltitudeZoomModifier integration tests) can run.
        """
        try:
            import mavsdk  # noqa: F401
        except ImportError as e:
            raise ImportError(_MAVSDK_INSTALL_HINT) from e
        # Real subscription wiring lands in Plan 8.
        return self._last_pull  # may be None on early frames

    def snapshot(self, t: float) -> TelemetrySnapshot:
        pulled = self._pull_snapshot_sync()
        if pulled is None:
            return TelemetrySnapshot(timestamp=t)
        return replace(pulled, timestamp=t)
```

- [ ] **Step 4: Add the export.**

Modify `hailo_tiling/telemetry/__init__.py` to add:

```python
from .mavsdk import MavsdkTelemetry  # noqa: F401
```

- [ ] **Step 5: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_telemetry_mavsdk.py -v
```
Expected: 5 passed (one of them — `test_snapshot_raises_clear_error_when_mavsdk_absent` — is conditionally skipped on machines where `mavsdk` IS installed; passes elsewhere).

- [ ] **Step 6: Confirm import is still clean.**

Run:
```bash
python -c "from hailo_tiling.telemetry import MavsdkTelemetry; print('ok')"
```
Expected: `ok`. (Importing must succeed even if `mavsdk` is absent.)

- [ ] **Step 7: Commit.**

```bash
git add hailo_tiling/telemetry/mavsdk.py \
        hailo_tiling/telemetry/__init__.py \
        hailo_tiling/tests/test_telemetry_mavsdk.py
git commit -m "hailo_tiling: MavsdkTelemetry — lazy MAVSDK import with _pull_snapshot_sync seam"
```

---

## Task 6: Thread telemetry through the scheduler + widen modifier signatures

Before introducing new modifiers, update the scheduler to thread `telemetry: TelemetrySnapshot` through every `emit()` / `modify()` call. This is a backwards-compatible change because:
- `TileScheduler.decide(...)` gains a `telemetry: TelemetrySnapshot | None = None` kwarg defaulting to `NULL_SNAPSHOT`.
- The Protocols (`TileEmitter`, `TileModifier`) widen by one positional-keyword arg.
- The three existing emitters and `BudgetTrimModifier` are updated to accept (and ignore) the new arg.
- The Plan 1 parity test still passes because all existing callers pass `telemetry=NULL_SNAPSHOT` (the default).

**Files:**
- Modify: `hailo_tiling/scheduler.py`
- Modify: `hailo_tiling/emitters/discovery_grid.py`
- Modify: `hailo_tiling/emitters/track_roi.py`
- Modify: `hailo_tiling/emitters/recovery.py`
- Modify: `hailo_tiling/modifiers/budget_trim.py`
- Modify: `hailo_tiling/tests/conftest.py` (add telemetry fixtures)
- Modify: `hailo_tiling/tests/test_scheduler_protocols.py` (dummy classes accept telemetry)

- [ ] **Step 1: Add telemetry fixtures to `conftest.py`.**

Append to `hailo_tiling/tests/conftest.py`:

```python
# ----------------------------------------------------------------------
# Telemetry fixtures (added in Plan 2)
# ----------------------------------------------------------------------
from hailo_tiling.telemetry import (
    NULL_SNAPSHOT,
    StaticTelemetry,
    TelemetrySnapshot,
)


@pytest.fixture
def null_snapshot():
    return NULL_SNAPSHOT


@pytest.fixture
def low_altitude_snapshot():
    """5 m AGL — close to subject, modifiers should widen the ROI."""
    return TelemetrySnapshot(altitude_agl_m=5.0, timestamp=0.0)


@pytest.fixture
def high_altitude_snapshot():
    """40 m AGL — far from subject, modifiers should narrow the ROI."""
    return TelemetrySnapshot(altitude_agl_m=40.0, timestamp=0.0)


@pytest.fixture
def static_low_telemetry():
    return StaticTelemetry(altitude_agl_m=5.0)


@pytest.fixture
def static_high_telemetry():
    return StaticTelemetry(altitude_agl_m=40.0)
```

- [ ] **Step 2: Widen `scheduler.py` Protocols and `decide()`.**

Replace the file contents:

```python
# hailo_tiling/scheduler.py
"""TileScheduler — composes TileEmitters and TileModifiers.

Plan 2 widens the protocols by one keyword argument: `telemetry: TelemetrySnapshot`.
Existing emitters/modifiers from Plan 1 accept-and-ignore the new arg; new
modifiers (AltitudeZoom, AdaptiveSliceSizing) read it.

This module deliberately contains no Hailo / GStreamer / OpenCV imports.
"""
from __future__ import annotations

from typing import Protocol, Sequence, runtime_checkable

from .telemetry import NULL_SNAPSHOT, TelemetrySnapshot
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
        telemetry: TelemetrySnapshot = NULL_SNAPSHOT,
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
        telemetry: TelemetrySnapshot = NULL_SNAPSHOT,
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
        telemetry: TelemetrySnapshot | None = None,
    ) -> list[CropRect]:
        tel = NULL_SNAPSHOT if telemetry is None else telemetry
        tiles: list[CropRect] = []
        for e in self.emitters:
            tiles.extend(e.emit(src_w, src_h, lock, frame_idx, meter, tel))
        for m in self.modifiers:
            tiles = m.modify(tiles, src_w, src_h, lock, frame_idx, meter, tel)
        return tiles
```

- [ ] **Step 3: Widen each existing emitter to accept the new arg.**

For `hailo_tiling/emitters/discovery_grid.py`, change the `emit` method to:

```python
    def emit(self, src_w, src_h, lock, frame_idx, meter, telemetry=None) -> list[CropRect]:
        # Legacy parity: when recovery is active (SEARCHING/LOST with a known
        # track), the legacy scheduler emits only the recovery grid. Mirror
        # that here by suppressing discovery in the same condition.
        if lock.status in ("SEARCHING", "LOST") and lock.track_id is not None:
            return []
        if frame_idx % self.period != 0:
            return []
        return _grid_full(src_w, src_h, self.gx, self.gy, self.mode)
```

Same `telemetry=None` widening in `track_roi.py` and `recovery.py`. Body unchanged in both — they don't read telemetry yet.

- [ ] **Step 4: Widen `BudgetTrimModifier.modify(...)`.**

Replace in `hailo_tiling/modifiers/budget_trim.py`:

```python
    def modify(
        self,
        tiles: list[CropRect],
        src_w: int,
        src_h: int,
        lock: LockState,
        frame_idx: int,
        meter,
        telemetry=None,  # accepted for Protocol compatibility; not used
    ) -> list[CropRect]:
        budget = int(meter.available(frame_idx))
        if budget < 0:
            return tiles
        if len(tiles) <= budget:
            return tiles
        return tiles[: max(0, budget)]
```

- [ ] **Step 5: Update `test_scheduler_protocols.py` dummy classes.**

The Plan 1 file currently has `def emit(self, src_w, src_h, lock, frame_idx, meter):` and `def modify(self, tiles, ...):`. Add `telemetry=None` to both:

```python
class _DummyEmitter:
    name = "dummy_emitter"

    def __init__(self, crops):
        self._crops = crops

    def emit(self, src_w, src_h, lock, frame_idx, meter, telemetry=None):
        return list(self._crops)


class _DummyModifier:
    name = "dummy_modifier"

    def __init__(self, tag):
        self._tag = tag

    def modify(self, tiles, src_w, src_h, lock, frame_idx, meter, telemetry=None):
        return [CropRect(x=t.x, y=t.y, w=t.w, h=t.h, mode=self._tag) for t in tiles]
```

(All assertions stay the same; the Plan 1 scheduler call uses positional args only.)

- [ ] **Step 6: Run the full Plan 1 test suite — must still pass byte-identically.**

Run:
```bash
pytest hailo_tiling/tests -v --ignore=hailo_tiling/tests/test_telemetry_mavsdk.py \
    --ignore=hailo_tiling/tests/test_telemetry_recorded.py \
    --ignore=hailo_tiling/tests/test_telemetry_static.py \
    --ignore=hailo_tiling/tests/test_telemetry_snapshot.py \
    --ignore=hailo_tiling/tests/test_telemetry_provider_abc.py
```
Expected: **every Plan 1 test passes unchanged.** If `test_scheduler_legacy_parity.py` regresses, the widening introduced a behavioural drift; fix before continuing.

Then run the full suite (Plan 1 + Plan 2 so far):
```bash
pytest hailo_tiling/tests dynamic_tiling/tests -v
```
Expected: all green.

- [ ] **Step 7: Commit.**

```bash
git add hailo_tiling/scheduler.py \
        hailo_tiling/emitters/discovery_grid.py \
        hailo_tiling/emitters/track_roi.py \
        hailo_tiling/emitters/recovery.py \
        hailo_tiling/modifiers/budget_trim.py \
        hailo_tiling/tests/conftest.py \
        hailo_tiling/tests/test_scheduler_protocols.py
git commit -m "hailo_tiling: thread TelemetrySnapshot through emitters/modifiers (Plan 1 parity preserved)"
```

---

## Task 7: `AltitudeZoomModifier` — telemetry-gated ROI zoom

End-to-end validation that telemetry flows correctly. Reads `telemetry.altitude_agl_m`; gates the ROI tile's effective zoom (wider tile at low altitude → less zoom; narrower tile at high altitude → more zoom). Falls back to `fallback_max_zoom` (or leaves the ROI alone) when telemetry returns `None`.

**Design choice (locked):** Only the ROI tile (`mode == "s"` produced by `TrackROIEmitter`) is touched. Discovery and recovery tiles pass through unchanged. The modifier expects to run **before** `BudgetTrimModifier` so a re-zoomed ROI tile is still counted by the budget logic correctly.

**Files:**
- Create: `hailo_tiling/modifiers/altitude_zoom.py`
- Modify: `hailo_tiling/modifiers/__init__.py` (add export)
- Create: `hailo_tiling/tests/test_modifier_altitude_zoom.py`

- [ ] **Step 1: Write the failing test.**

```python
# hailo_tiling/tests/test_modifier_altitude_zoom.py
"""AltitudeZoomModifier — validates the telemetry data-flow end-to-end."""
from __future__ import annotations

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.emitters import (
    DiscoveryGridEmitter,
    RecoveryGridEmitter,
    TrackROIEmitter,
)
from hailo_tiling.modifiers import AltitudeZoomModifier, BudgetTrimModifier
from hailo_tiling.scheduler import TileScheduler
from hailo_tiling.telemetry import NULL_SNAPSHOT, TelemetrySnapshot
from hailo_tiling.types import CropRect, LockState


def _meter():
    return BudgetMeter(budget_inf_per_s=1000.0, fps=30.0)


def test_low_altitude_widens_roi(src_dims, tracking_lock):
    """At low altitude, the modifier widens the ROI crop (less zoom)."""
    src_w, src_h = src_dims
    base_roi = TrackROIEmitter(max_zoom=2.0, target_model_h=40.0, roi_margin_frac=0.25)
    base_tiles = base_roi.emit(src_w, src_h, tracking_lock, 0, _meter())

    low = TelemetrySnapshot(altitude_agl_m=5.0)
    m = AltitudeZoomModifier(zoom_at_low_agl=1.0, zoom_at_high_agl=2.0,
                              low_agl_m=5.0, high_agl_m=40.0,
                              fallback_max_zoom=2.0)
    tiles_low = m.modify(list(base_tiles), src_w, src_h, tracking_lock, 0, _meter(), low)
    # At low altitude (zoom_at_low_agl=1.0), the ROI tile should be at
    # MODEL_W width or wider (less zoomed).
    assert tiles_low[0].w >= base_tiles[0].w  # wider or equal


def test_high_altitude_narrows_roi(src_dims, tracking_lock):
    """At high altitude, the modifier narrows the ROI crop (more zoom)."""
    src_w, src_h = src_dims
    base_roi = TrackROIEmitter(max_zoom=2.0, target_model_h=40.0, roi_margin_frac=0.25)
    base_tiles = base_roi.emit(src_w, src_h, tracking_lock, 0, _meter())

    high = TelemetrySnapshot(altitude_agl_m=40.0)
    m = AltitudeZoomModifier(zoom_at_low_agl=1.0, zoom_at_high_agl=2.0,
                              low_agl_m=5.0, high_agl_m=40.0,
                              fallback_max_zoom=2.0)
    tiles_high = m.modify(list(base_tiles), src_w, src_h, tracking_lock, 0, _meter(), high)
    # At high altitude, the ROI tile should be narrower (more zoomed in).
    assert tiles_high[0].w <= base_tiles[0].w


def test_null_telemetry_is_passthrough(src_dims, tracking_lock):
    """If telemetry.altitude_agl_m is None, leave tiles untouched."""
    src_w, src_h = src_dims
    base_roi = TrackROIEmitter()
    base_tiles = list(base_roi.emit(src_w, src_h, tracking_lock, 0, _meter()))

    m = AltitudeZoomModifier()
    out = m.modify(list(base_tiles), src_w, src_h, tracking_lock, 0, _meter(), NULL_SNAPSHOT)
    assert out == base_tiles


def test_only_roi_tiles_are_modified(src_dims, tracking_lock):
    """Discovery/recovery tiles (mode != 's') must be passed through unchanged."""
    src_w, src_h = src_dims
    roi = CropRect(x=100, y=100, w=400, h=300, mode="s")
    disc = CropRect(x=0, y=0, w=1280, h=960, mode="m")
    rec = CropRect(x=200, y=200, w=640, h=480, mode="s")  # mode 's' but no track_id semantics

    # AltitudeZoomModifier targets the *ROI* tile, which is the first 's'-mode
    # tile emitted while TRACKING. We pin the identification rule to "first
    # 's' tile by index when lock.status == 'TRACKING'".
    high = TelemetrySnapshot(altitude_agl_m=40.0)
    m = AltitudeZoomModifier(zoom_at_low_agl=1.0, zoom_at_high_agl=2.0,
                              low_agl_m=5.0, high_agl_m=40.0)
    out = m.modify([roi, disc, rec], src_w, src_h, tracking_lock, 0, _meter(), high)
    # Discovery tile unchanged.
    assert out[1] == disc
    # First 's' tile modified (or at minimum, returned). Second 's' tile unchanged.
    assert out[2] == rec


def test_end_to_end_with_scheduler(src_dims, tracking_lock,
                                    low_altitude_snapshot, high_altitude_snapshot):
    """Full scheduler-level path: ROI -> altitude-zoom -> budget-trim."""
    src_w, src_h = src_dims
    scheduler = TileScheduler(
        emitters=[
            TrackROIEmitter(max_zoom=2.0, target_model_h=40.0, roi_margin_frac=0.25),
            DiscoveryGridEmitter(grid=(3, 2), period=15, mode="m"),
            RecoveryGridEmitter(grid=(3, 3), span=0.4),
        ],
        modifiers=[
            AltitudeZoomModifier(zoom_at_low_agl=1.0, zoom_at_high_agl=2.0,
                                  low_agl_m=5.0, high_agl_m=40.0),
            BudgetTrimModifier(),
        ],
    )
    tiles_low = scheduler.decide(src_w, src_h, tracking_lock, 0, _meter(),
                                  telemetry=low_altitude_snapshot)
    tiles_high = scheduler.decide(src_w, src_h, tracking_lock, 0, _meter(),
                                   telemetry=high_altitude_snapshot)
    # ROI tile (first tile by emitter order) is wider at low altitude.
    assert tiles_low[0].w >= tiles_high[0].w
    # Discovery tile count unchanged in both.
    assert sum(1 for t in tiles_low if t.mode == "m") == 6
    assert sum(1 for t in tiles_high if t.mode == "m") == 6


def test_altitude_below_low_clamps_to_low_zoom(src_dims, tracking_lock):
    """Altitude below low_agl_m clamps to zoom_at_low_agl."""
    src_w, src_h = src_dims
    base = TrackROIEmitter().emit(src_w, src_h, tracking_lock, 0, _meter())

    very_low = TelemetrySnapshot(altitude_agl_m=0.5)
    at_low = TelemetrySnapshot(altitude_agl_m=5.0)

    m = AltitudeZoomModifier(zoom_at_low_agl=1.0, zoom_at_high_agl=2.0,
                              low_agl_m=5.0, high_agl_m=40.0)
    very_low_tiles = m.modify(list(base), src_w, src_h, tracking_lock, 0, _meter(), very_low)
    at_low_tiles = m.modify(list(base), src_w, src_h, tracking_lock, 0, _meter(), at_low)
    assert very_low_tiles[0].w == at_low_tiles[0].w
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_modifier_altitude_zoom.py -v
```
Expected: `ImportError: cannot import name 'AltitudeZoomModifier' from 'hailo_tiling.modifiers'`.

- [ ] **Step 3: Implement `AltitudeZoomModifier`.**

```python
# hailo_tiling/modifiers/altitude_zoom.py
"""AltitudeZoomModifier — gates ROI tile zoom from drone altitude telemetry.

Reads `telemetry.altitude_agl_m` and linearly interpolates an effective
`max_zoom` between `zoom_at_low_agl` (when at or below `low_agl_m`) and
`zoom_at_high_agl` (when at or above `high_agl_m`). The first ROI tile in
the working list — defined as "the first tile with mode == 's' when
lock.status == 'TRACKING'" — is rebuilt with the new zoom factor.

Falls back to `fallback_max_zoom` (or leaves the tile untouched if that is
None) when `telemetry.altitude_agl_m is None`.

Reference: arXiv:2511.19728 (altitude-aware dynamic tiling).
"""
from __future__ import annotations

from typing import Optional

from ..telemetry import NULL_SNAPSHOT, TelemetrySnapshot
from ..types import CropRect, LockState, MODEL_W


def _interp_zoom(agl_m: float, low_agl: float, high_agl: float,
                 z_low: float, z_high: float) -> float:
    """Linear interp of zoom factor between low_agl and high_agl. Clamped."""
    if high_agl <= low_agl:
        return z_low
    if agl_m <= low_agl:
        return z_low
    if agl_m >= high_agl:
        return z_high
    f = (agl_m - low_agl) / (high_agl - low_agl)
    return z_low + f * (z_high - z_low)


class AltitudeZoomModifier:
    """Gate the ROI tile's effective max-zoom by drone AGL altitude."""

    name = "altitude_zoom"

    def __init__(
        self,
        zoom_at_low_agl: float = 1.0,
        zoom_at_high_agl: float = 2.0,
        low_agl_m: float = 5.0,
        high_agl_m: float = 40.0,
        fallback_max_zoom: Optional[float] = None,
    ):
        self.zoom_at_low_agl = zoom_at_low_agl
        self.zoom_at_high_agl = zoom_at_high_agl
        self.low_agl_m = low_agl_m
        self.high_agl_m = high_agl_m
        self.fallback_max_zoom = fallback_max_zoom

    def _rescale_roi(self, roi: CropRect, lock: LockState, src_w: int, src_h: int,
                      zoom: float) -> CropRect:
        """Rebuild the ROI tile centered on the locked bbox at the new zoom."""
        bx, by, bw, bh = lock.bbox_norm
        cx = (bx + bw / 2) * src_w
        cy = (by + bh / 2) * src_h
        crop_w = int(round(MODEL_W / max(0.01, zoom)))
        # Always contain the whole target with margin (same rule as TrackROIEmitter).
        need_w = int(round(bw * src_w * 1.5))  # 1 + 2*0.25 margin
        crop_w = max(crop_w, need_w)
        return CropRect.from_center_width(cx, cy, crop_w, mode="s").clamp(src_w, src_h)

    def modify(
        self,
        tiles: list[CropRect],
        src_w: int,
        src_h: int,
        lock: LockState,
        frame_idx: int,
        meter,
        telemetry: TelemetrySnapshot = NULL_SNAPSHOT,
    ) -> list[CropRect]:
        if lock.status != "TRACKING":
            return tiles

        agl = telemetry.altitude_agl_m
        if agl is None:
            if self.fallback_max_zoom is None:
                return tiles
            zoom = self.fallback_max_zoom
        else:
            zoom = _interp_zoom(agl, self.low_agl_m, self.high_agl_m,
                                self.zoom_at_low_agl, self.zoom_at_high_agl)

        # Rewrite the first 's'-mode tile (the ROI tile). If there is none,
        # this modifier is a no-op for the frame.
        out = list(tiles)
        for i, t in enumerate(out):
            if t.mode == "s":
                out[i] = self._rescale_roi(t, lock, src_w, src_h, zoom)
                break
        return out
```

- [ ] **Step 4: Add the export.**

Update `hailo_tiling/modifiers/__init__.py` to:

```python
"""Tile modifiers — classes that mutate the working tile list before submission."""
from .altitude_zoom import AltitudeZoomModifier  # noqa: F401
from .budget_trim import BudgetTrimModifier  # noqa: F401
```

- [ ] **Step 5: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_modifier_altitude_zoom.py -v
```
Expected: 6 passed.

- [ ] **Step 6: Re-run the full hailo_tiling suite — no regressions.**

Run:
```bash
pytest hailo_tiling/tests dynamic_tiling/tests -v
```
Expected: all green.

- [ ] **Step 7: Commit.**

```bash
git add hailo_tiling/modifiers/altitude_zoom.py \
        hailo_tiling/modifiers/__init__.py \
        hailo_tiling/tests/test_modifier_altitude_zoom.py
git commit -m "hailo_tiling: AltitudeZoomModifier — gate ROI zoom on altitude_agl_m"
```

---

## Task 8: `AdaptiveSliceSizingModifier` (ASAHI) — reshape the discovery grid

Reference: `docs/research/2026-05-27-industry-tiling-drone-tracking.md` §1.2 (ASAHI). Reshapes the discovery grid based on detected/expected object scale: when the locked target is large in the frame (apparent from `lock.bbox_norm[3] * src_h`), use a coarser grid; when small, use a finer grid.

**Design (locked):**
- Operates on the discovery tiles only — identified by `mode == "m"`.
- Reads target scale from `lock.bbox_norm[3]` (height fraction) when `lock.status == "TRACKING"`. Falls back to `default_grid` if no track.
- Optional telemetry hint: if `telemetry.altitude_agl_m` is present, modifier may use it as a *secondary* scale signal (proportional to expected target size in pixels at that altitude). Plan 2 implementation uses ONLY the track bbox; altitude integration is a Plan-3 follow-up note.
- Replaces the existing discovery tiles in-place with a freshly-built grid using the new `(gx, gy)` — relies on the helper from `discovery_grid._grid_full`. Recovery and ROI tiles untouched.
- "No telemetry / no track" degraded mode: returns `tiles` unchanged.

**Files:**
- Create: `hailo_tiling/modifiers/adaptive_sizing.py`
- Modify: `hailo_tiling/modifiers/__init__.py` (add export)
- Create: `hailo_tiling/tests/test_modifier_adaptive_sizing.py`

- [ ] **Step 1: Write the failing test.**

```python
# hailo_tiling/tests/test_modifier_adaptive_sizing.py
"""AdaptiveSliceSizingModifier (ASAHI) — reshape discovery grid by object scale."""
from __future__ import annotations

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.emitters.discovery_grid import _grid_full
from hailo_tiling.modifiers import AdaptiveSliceSizingModifier
from hailo_tiling.telemetry import NULL_SNAPSHOT, TelemetrySnapshot
from hailo_tiling.types import CropRect, LockState


def _meter():
    return BudgetMeter(budget_inf_per_s=1000.0, fps=30.0)


def _disc_tiles(src_w, src_h, gx, gy):
    return _grid_full(src_w, src_h, gx, gy, "m")


def test_no_track_returns_tiles_unchanged(src_dims, lost_lock):
    src_w, src_h = src_dims
    base = _disc_tiles(src_w, src_h, 3, 2)
    m = AdaptiveSliceSizingModifier()
    out = m.modify(base, src_w, src_h, lost_lock, 0, _meter(), NULL_SNAPSHOT)
    assert out == base


def test_no_telemetry_is_degraded_mode_still_uses_track_scale(src_dims):
    """No telemetry but a TRACKING lock — modifier still operates from bbox scale.

    Spec §3.3: each field is Optional; consumers degrade gracefully.
    """
    src_w, src_h = src_dims
    small_target = LockState(
        track_id=1, bbox_norm=(0.5, 0.5, 0.02, 0.04),
        status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0),
    )
    base = _disc_tiles(src_w, src_h, 3, 2)
    m = AdaptiveSliceSizingModifier(target_h_thresholds=(0.05, 0.15),
                                     small_grid=(6, 4), medium_grid=(3, 2),
                                     large_grid=(2, 1))
    out = m.modify(base, src_w, src_h, small_target, 0, _meter(), NULL_SNAPSHOT)
    # Small target -> finer grid -> 6*4 = 24 tiles instead of 6.
    assert len(out) == 24


def test_large_target_coarser_grid(src_dims):
    src_w, src_h = src_dims
    big_target = LockState(
        track_id=1, bbox_norm=(0.4, 0.3, 0.3, 0.4),
        status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0),
    )
    base = _disc_tiles(src_w, src_h, 3, 2)
    m = AdaptiveSliceSizingModifier(target_h_thresholds=(0.05, 0.15),
                                     small_grid=(6, 4), medium_grid=(3, 2),
                                     large_grid=(2, 1))
    out = m.modify(base, src_w, src_h, big_target, 0, _meter(), NULL_SNAPSHOT)
    assert len(out) == 2  # 2*1


def test_medium_target_keeps_default_grid(src_dims):
    src_w, src_h = src_dims
    med_target = LockState(
        track_id=1, bbox_norm=(0.4, 0.3, 0.05, 0.10),
        status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0),
    )
    base = _disc_tiles(src_w, src_h, 3, 2)
    m = AdaptiveSliceSizingModifier(target_h_thresholds=(0.05, 0.15),
                                     small_grid=(6, 4), medium_grid=(3, 2),
                                     large_grid=(2, 1))
    out = m.modify(base, src_w, src_h, med_target, 0, _meter(), NULL_SNAPSHOT)
    assert len(out) == 6  # 3*2


def test_does_not_touch_non_discovery_tiles(src_dims, tracking_lock):
    """ROI ('s') and recovery ('s') tiles must be passed through unchanged."""
    src_w, src_h = src_dims
    roi = CropRect(x=100, y=100, w=400, h=300, mode="s")
    disc = _disc_tiles(src_w, src_h, 3, 2)
    tiles_in = [roi] + disc

    m = AdaptiveSliceSizingModifier(target_h_thresholds=(0.05, 0.15),
                                     small_grid=(6, 4), medium_grid=(3, 2),
                                     large_grid=(2, 1))
    # Force "large" path: bbox_norm h=0.20
    big = LockState(track_id=1, bbox_norm=(0.4, 0.3, 0.2, 0.20),
                     status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0))
    out = m.modify(tiles_in, src_w, src_h, big, 0, _meter(), NULL_SNAPSHOT)
    # First tile is the unchanged ROI tile.
    assert out[0] == roi
    # Remaining tiles are the new 2x1 = 2 discovery tiles.
    assert len(out) == 1 + 2


def test_returns_empty_discovery_when_no_input_discovery_tiles(src_dims, tracking_lock):
    """If the input has no 'm' tiles (e.g., off-cadence frame), no-op on discovery."""
    src_w, src_h = src_dims
    roi = CropRect(x=100, y=100, w=400, h=300, mode="s")
    m = AdaptiveSliceSizingModifier()
    out = m.modify([roi], src_w, src_h, tracking_lock, 0, _meter(), NULL_SNAPSHOT)
    assert out == [roi]
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_modifier_adaptive_sizing.py -v
```
Expected: `ImportError: cannot import name 'AdaptiveSliceSizingModifier'`.

- [ ] **Step 3: Implement `AdaptiveSliceSizingModifier`.**

```python
# hailo_tiling/modifiers/adaptive_sizing.py
"""AdaptiveSliceSizingModifier — ASAHI-style discovery grid resizing.

When the locked target's apparent height (bbox_norm[3]) is small, use a
finer discovery grid (more, smaller tiles) so small targets are found; when
the apparent height is large, use a coarser grid so cycles aren't wasted on
redundant tiles.

Reference: Akyon et al., Remote Sensing 15(5):1249, MDPI 2023 (ASAHI).
docs/research/2026-05-27-industry-tiling-drone-tracking.md §1.2.

Plan 2 implementation uses ONLY the bbox height as the scale signal. A
secondary altitude-based signal (telemetry.altitude_agl_m → expected pixel
height per target class) is a Plan 3+ follow-up.
"""
from __future__ import annotations

from ..emitters.discovery_grid import _grid_full
from ..telemetry import NULL_SNAPSHOT, TelemetrySnapshot
from ..types import CropRect, LockState


class AdaptiveSliceSizingModifier:
    """Reshape the discovery grid based on target apparent size."""

    name = "adaptive_slice_sizing"

    def __init__(
        self,
        target_h_thresholds: tuple[float, float] = (0.05, 0.15),
        small_grid: tuple[int, int] = (6, 4),
        medium_grid: tuple[int, int] = (3, 2),
        large_grid: tuple[int, int] = (2, 1),
        discovery_mode_tag: str = "m",
    ):
        """target_h_thresholds = (T_small, T_large):
            bbox_h < T_small  -> small_grid
            T_small <= bbox_h < T_large -> medium_grid
            bbox_h >= T_large -> large_grid
        """
        self.t_small, self.t_large = target_h_thresholds
        self.small_grid = small_grid
        self.medium_grid = medium_grid
        self.large_grid = large_grid
        self.discovery_mode_tag = discovery_mode_tag

    def _pick_grid(self, lock: LockState) -> tuple[int, int]:
        bh = lock.bbox_norm[3]
        if bh < self.t_small:
            return self.small_grid
        if bh < self.t_large:
            return self.medium_grid
        return self.large_grid

    def modify(
        self,
        tiles: list[CropRect],
        src_w: int,
        src_h: int,
        lock: LockState,
        frame_idx: int,
        meter,
        telemetry: TelemetrySnapshot = NULL_SNAPSHOT,
    ) -> list[CropRect]:
        # Degraded mode: no track signal -> pass tiles through unchanged.
        if lock.status != "TRACKING":
            return tiles

        gx, gy = self._pick_grid(lock)
        non_discovery = [t for t in tiles if t.mode != self.discovery_mode_tag]
        any_discovery = any(t.mode == self.discovery_mode_tag for t in tiles)
        if not any_discovery:
            return tiles  # no discovery tiles to reshape
        new_disc = _grid_full(src_w, src_h, gx, gy, self.discovery_mode_tag)
        # Preserve relative order: non-discovery tiles first (where they appeared
        # at the start of the list, by emitter order — ROI before discovery before
        # recovery), then the new discovery tiles.
        return non_discovery + new_disc
```

- [ ] **Step 4: Add the export.**

Update `hailo_tiling/modifiers/__init__.py`:

```python
"""Tile modifiers — classes that mutate the working tile list before submission."""
from .adaptive_sizing import AdaptiveSliceSizingModifier  # noqa: F401
from .altitude_zoom import AltitudeZoomModifier  # noqa: F401
from .budget_trim import BudgetTrimModifier  # noqa: F401
```

- [ ] **Step 5: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_modifier_adaptive_sizing.py -v
```
Expected: 6 passed.

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/modifiers/adaptive_sizing.py \
        hailo_tiling/modifiers/__init__.py \
        hailo_tiling/tests/test_modifier_adaptive_sizing.py
git commit -m "hailo_tiling: AdaptiveSliceSizingModifier (ASAHI) — reshape discovery grid by target scale"
```

---

## Task 9: `InferenceBackend` ABC + `MockBackend` fixture

Defines the spec-correct multi-crop signature `infer(frame, crops) → list[list[Det]]`. The HefBackend in Task 10 implements this by looping internally so the existing per-crop chip path is preserved.

**Files:**
- Create: `hailo_tiling/backends/__init__.py`
- Create: `hailo_tiling/backends/backend.py`
- Modify: `hailo_tiling/tests/conftest.py` (add `mock_backend` factory)
- Create: `hailo_tiling/tests/test_backend_abc.py`

- [ ] **Step 1: Write the failing ABC + MockBackend test.**

```python
# hailo_tiling/tests/test_backend_abc.py
"""InferenceBackend ABC contract + MockBackend fixture sanity."""
from __future__ import annotations

import numpy as np
import pytest

from hailo_tiling.backends import InferenceBackend, MockBackend
from hailo_tiling.types import CropRect, Det


def test_cannot_instantiate_bare_abc():
    with pytest.raises(TypeError):
        InferenceBackend()  # type: ignore[abstract]


def test_subclass_without_infer_method_fails():
    class _Bad(InferenceBackend):
        pass
    with pytest.raises(TypeError):
        _Bad()  # type: ignore[abstract]


def test_mock_backend_returns_one_list_per_crop():
    canned = {
        (0, (0, 0, 640, 480)): [Det(cls=0, score=0.9, x=0.1, y=0.1, w=0.05, h=0.1)],
        (0, (640, 0, 640, 480)): [],
    }
    be = MockBackend(canned)
    crops = [
        CropRect(x=0, y=0, w=640, h=480, mode="s"),
        CropRect(x=640, y=0, w=640, h=480, mode="s"),
    ]
    frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
    out = be.infer(frame, crops, frame_idx=0)
    assert len(out) == 2
    assert len(out[0]) == 1 and out[0][0].cls == 0
    assert out[1] == []


def test_mock_backend_unknown_crop_returns_empty_list():
    be = MockBackend({})
    crops = [CropRect(x=0, y=0, w=640, h=480, mode="s")]
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    out = be.infer(frame, crops, frame_idx=0)
    assert out == [[]]


def test_mock_backend_records_calls_for_assertions():
    be = MockBackend({})
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    crops_a = [CropRect(x=0, y=0, w=640, h=480, mode="s")]
    crops_b = [CropRect(x=10, y=10, w=640, h=480, mode="s")]
    be.infer(frame, crops_a, frame_idx=0)
    be.infer(frame, crops_b, frame_idx=1)
    assert be.call_count == 2
    assert be.calls[0]["frame_idx"] == 0
    assert be.calls[1]["crops"] == crops_b
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_backend_abc.py -v
```
Expected: `ImportError: cannot import name 'InferenceBackend' from 'hailo_tiling.backends'`.

- [ ] **Step 3: Implement the ABC and `MockBackend`.**

```python
# hailo_tiling/backends/__init__.py
"""Inference backends — the seam between scheduler policy and execution mechanism."""
from .backend import InferenceBackend, MockBackend  # noqa: F401
```

```python
# hailo_tiling/backends/backend.py
"""InferenceBackend ABC + MockBackend.

The spec (§3.4) defines `infer(frame, crops) -> list[list[Det]]`: one call
covers an ordered batch of crops and returns one detection-list per crop in
the same order. This batched signature is what GstCropperBackend (Plan 6)
needs; HefBackend (Task 10) implements it by looping over crops internally
so the on-chip code path stays per-crop.

`MockBackend` is the chip-free fixture used throughout the rest of the
hailo_tiling test suite — no Hailo hardware, no HailoRT import.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Sequence

from ..types import CropRect, Det


class InferenceBackend(ABC):
    """Run inference on an ordered batch of crops, return one list per crop."""

    @abstractmethod
    def infer(
        self,
        frame: Any,
        crops: Sequence[CropRect],
        frame_idx: int,
    ) -> list[list[Det]]:
        """Return crop-local normalized detections, one list per input crop, in order."""

    def close(self) -> None:  # pragma: no cover - default no-op
        return None


class MockBackend(InferenceBackend):
    """Test double. Looks up canned `(frame_idx, (x,y,w,h)) -> list[Det]` entries.

    Records every call into `self.calls` for assertion-by-test patterns.
    """

    def __init__(self, canned: dict[tuple[int, tuple[int, int, int, int]], list[Det]] | None = None):
        self.canned = dict(canned or {})
        self.calls: list[dict] = []

    @property
    def call_count(self) -> int:
        return len(self.calls)

    def infer(self, frame, crops, frame_idx):
        self.calls.append({"frame_idx": frame_idx, "crops": list(crops)})
        out: list[list[Det]] = []
        for c in crops:
            key = (frame_idx, (c.x, c.y, c.w, c.h))
            out.append(list(self.canned.get(key, [])))
        return out
```

- [ ] **Step 4: Add a `mock_backend` factory fixture to `conftest.py`.**

Append to `hailo_tiling/tests/conftest.py`:

```python
# ----------------------------------------------------------------------
# Backend fixtures (added in Plan 2)
# ----------------------------------------------------------------------
from hailo_tiling.backends import MockBackend


@pytest.fixture
def make_mock_backend():
    """Factory: returns a callable that builds a MockBackend with given canned dets."""
    def _make(canned=None):
        return MockBackend(canned or {})
    return _make
```

- [ ] **Step 5: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_backend_abc.py -v
```
Expected: 5 passed.

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/backends/__init__.py \
        hailo_tiling/backends/backend.py \
        hailo_tiling/tests/conftest.py \
        hailo_tiling/tests/test_backend_abc.py
git commit -m "hailo_tiling: InferenceBackend ABC + chip-free MockBackend fixture"
```

---

## Task 10: `HefBackend` lift + `dynamic_tiling.inference` shim

Lifts `dynamic_tiling/inference.py:HefBackend` into `hailo_tiling/backends/hef.py`, widens its `infer` API from per-crop to the batched `(frame, crops)` form spec'd in Task 9, and makes the old `dynamic_tiling.inference` module a shim that:
- re-exports `HefBackend` from the new location (kept under the legacy single-crop signature via a thin wrapper); and
- keeps `ReplayBackend` (the Plan-1 single-crop-canned version) in place for `dynamic_tiling.tests.test_inference`.

**Why a wrapper, not a hard move:** legacy `dynamic_tiling.run_dynamic` calls `HefBackend(...).infer(frame, crop, frame_idx)` with a single crop. We don't touch that file in this plan (out-of-scope, see Plan 8). The shim preserves the legacy single-crop interface while the new code path is the batched interface.

**The test must not require a Hailo chip.** It exercises the shim contract via monkeypatching out the HailoRT-touching constructor (`HefBackend.__init__` is dependency-injected with a fake handle).

**Files:**
- Create: `hailo_tiling/backends/hef.py`
- Modify: `hailo_tiling/backends/__init__.py` (add export)
- Modify: `dynamic_tiling/inference.py` (becomes a shim)
- Create: `hailo_tiling/tests/test_backend_hef_shim.py`

- [ ] **Step 1: Write the failing shim test.**

```python
# hailo_tiling/tests/test_backend_hef_shim.py
"""HefBackend — shim contract.

The test mocks the HailoRT-touching parts of HefBackend so it runs without a
Hailo chip, asserting the batched `infer(frame, crops, frame_idx)` shape and
the legacy single-crop shim path used by dynamic_tiling.
"""
from __future__ import annotations

import numpy as np
import pytest

from hailo_tiling.backends import HefBackend
from hailo_tiling.types import CropRect, Det


class _FakeHandle:
    """Stand-in for tiling_benchmark HefHandle."""

    def __init__(self):
        self.calls = []
        self.closed = False

    def infer(self, rgb_chw_or_hwc):
        self.calls.append(rgb_chw_or_hwc.shape)
        # Return a sentinel that the fake decoder maps to one detection.
        return "RAW"

    def close(self):
        self.closed = True


class _FakeDet:
    def __init__(self, cls=0, x=0.25, y=0.25, w=0.5, h=0.5, score=0.9):
        self.cls, self.x, self.y, self.w, self.h, self.score = cls, x, y, w, h, score


def _fake_decode(raw):
    assert raw == "RAW"
    return [_FakeDet()]


@pytest.fixture
def patched_hef(monkeypatch):
    """Patch HefBackend internals so __init__ does no HailoRT work."""
    handle = _FakeHandle()

    def _from_handle(self, handle_obj, decode):
        self._handle = handle_obj
        self._decode = decode

    monkeypatch.setattr(HefBackend, "__init__", _from_handle, raising=False)
    backend = HefBackend(handle, _fake_decode)  # type: ignore[call-arg]
    return backend, handle


def test_batched_infer_returns_list_per_crop(patched_hef):
    be, _ = patched_hef
    crops = [
        CropRect(x=0, y=0, w=640, h=480, mode="s"),
        CropRect(x=10, y=10, w=640, h=480, mode="s"),
    ]
    frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
    out = be.infer(frame, crops, frame_idx=0)
    assert len(out) == 2
    assert all(len(dets) == 1 for dets in out)


def test_legacy_dynamic_tiling_inference_reexports_hefbackend():
    """dynamic_tiling.inference must still expose HefBackend (shim path)."""
    from dynamic_tiling.inference import HefBackend as LegacyHefBackend
    assert LegacyHefBackend is HefBackend


def test_legacy_replay_backend_still_works():
    """dynamic_tiling.inference.ReplayBackend keeps its single-crop API."""
    from dynamic_tiling.inference import ReplayBackend
    canned = {(0, (0, 0, 640, 480)): [_FakeDet()]}
    be = ReplayBackend(canned)
    out = be.infer(frame=np.zeros((480, 640, 3), dtype=np.uint8),
                    crop=CropRect(x=0, y=0, w=640, h=480, mode="s"),
                    frame_idx=0)
    assert len(out) == 1
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_backend_hef_shim.py -v
```
Expected: `ImportError: cannot import name 'HefBackend' from 'hailo_tiling.backends'`.

- [ ] **Step 3: Implement the lifted `HefBackend`.**

```python
# hailo_tiling/backends/hef.py
"""HefBackend — direct HailoRT inference, lifted from dynamic_tiling/inference.py.

Spec §7.11: this backend is **dev/debug only**. Paper-reported results run
through GstCropperBackend (Plan 6). The lift preserves the original per-crop
inference behaviour while exposing the spec-correct batched signature
`infer(frame, crops, frame_idx) -> list[list[Det]]`.

The HailoRT-touching dependencies live inside `__init__` and `_infer_one` so
that this module imports cleanly on a chip-free dev laptop. Tests
dependency-inject a fake handle via `HefBackend(handle, decode)` to avoid
ever instantiating HailoRT.
"""
from __future__ import annotations

from typing import Any, Callable, Optional, Sequence

import numpy as np

from ..types import CropRect, Det, MODEL_H, MODEL_W
from .backend import InferenceBackend

# Lazy cv2 import — kept inside the per-crop fast path so module import
# remains clean on machines without OpenCV.


class HefBackend(InferenceBackend):
    """Real on-chip backend wrapping the tiling_benchmark HefHandle.

    Two construction modes:
    1. Production: `HefBackend(hef_path="...", nms_score_threshold=0.25)`
       — lazy-imports HailoRT bits and opens the HEF.
    2. Test injection: `HefBackend(handle, decode)` — pre-built handle and
       decode callable. Used by unit tests to avoid touching HailoRT.
    """

    def __init__(self, *args, **kwargs):
        if len(args) == 2 and not kwargs:
            # Test-injection path: positional (handle, decode).
            handle, decode = args
            self._handle = handle
            self._decode = decode
            return
        hef_path = kwargs.get("hef_path") or (args[0] if args else None)
        nms_score_threshold = kwargs.get("nms_score_threshold", 0.25)
        if hef_path is None:
            raise TypeError("HefBackend requires hef_path or (handle, decode)")
        # Lazy import — only when actually opening a real HEF.
        from probe_phantom_hef import HefHandle, decode_nms_output  # via _vendor_paths
        self._handle = HefHandle.open(hef_path, nms_score_threshold=nms_score_threshold)
        self._decode = decode_nms_output

    def _infer_one(self, frame: np.ndarray, crop: CropRect) -> list:
        import cv2  # local import — chip-free dev machines may lack OpenCV
        sub = frame[crop.y:crop.y + crop.h, crop.x:crop.x + crop.w]
        resized = cv2.resize(sub, (MODEL_W, MODEL_H), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        return self._decode(self._handle.infer(rgb))

    def infer(self, frame, crops: Sequence[CropRect], frame_idx: int) -> list[list[Det]]:
        return [self._infer_one(frame, c) for c in crops]

    def close(self) -> None:
        self._handle.close()
```

- [ ] **Step 4: Add the export.**

Update `hailo_tiling/backends/__init__.py`:

```python
"""Inference backends — the seam between scheduler policy and execution mechanism."""
from .backend import InferenceBackend, MockBackend  # noqa: F401
from .hef import HefBackend  # noqa: F401
```

- [ ] **Step 5: Rewrite `dynamic_tiling/inference.py` as a shim.**

```python
# dynamic_tiling/inference.py
"""Compatibility shim: backends live in hailo_tiling.backends.

`HefBackend` is the same class as `hailo_tiling.backends.hef.HefBackend`.
`ReplayBackend` is kept here as a legacy single-crop helper used by
`dynamic_tiling.tests.test_inference`. The Plan-1-style per-crop API
(infer(frame, crop, frame_idx)) is preserved exactly.

This shim disappears in Plan 8 (drone-follow migration) once all callers
move to hailo_tiling.
"""
from __future__ import annotations

from typing import Protocol

from hailo_tiling.backends.hef import HefBackend  # noqa: F401

from .types import CropRect


class InferenceBackend(Protocol):
    """Legacy single-crop protocol; preserved for dynamic_tiling callers."""

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:  # noqa: D401
        ...


class ReplayBackend:
    """Legacy deterministic backend keyed on (frame_idx, (x, y, w, h)).

    Single-crop API. The batched ReplayBackend (`hailo_tiling.backends`) lands
    in Plan 4 alongside the SQLite cache layer.
    """

    def __init__(self, canned: dict):
        self._canned = canned

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:
        return list(self._canned.get((frame_idx, (crop.x, crop.y, crop.w, crop.h)), []))
```

- [ ] **Step 6: Run the new test and the legacy test together.**

Run:
```bash
pytest hailo_tiling/tests/test_backend_hef_shim.py dynamic_tiling/tests/test_inference.py -v
```
Expected: all passed. (`test_replay_backend_returns_canned_crop_local_dets` and `test_replay_backend_empty_for_unknown_crop` should still pass through the shim.)

- [ ] **Step 7: Full suite check.**

Run:
```bash
pytest hailo_tiling/tests dynamic_tiling/tests -v
```
Expected: all green.

- [ ] **Step 8: Commit.**

```bash
git add hailo_tiling/backends/hef.py \
        hailo_tiling/backends/__init__.py \
        dynamic_tiling/inference.py \
        hailo_tiling/tests/test_backend_hef_shim.py
git commit -m "hailo_tiling: lift HefBackend; dynamic_tiling.inference becomes a re-export shim"
```

---

## Task 11: Aggregator — per-class NMS + `map_to_source`

Lift `dynamic_tiling/aggregator.py:nms` and `map_to_source` into `hailo_tiling/aggregator/nms.py`. The legacy `dynamic_tiling/aggregator.py` is **not** replaced — both files coexist (verbatim port) until a final cleanup plan removes the duplicate.

**Files:**
- Create: `hailo_tiling/aggregator/__init__.py`
- Create: `hailo_tiling/aggregator/nms.py`
- Create: `hailo_tiling/tests/test_aggregator_nms.py`

- [ ] **Step 1: Create the subpackage and the failing test.**

```bash
mkdir -p hailo_tiling/aggregator
```

```python
# hailo_tiling/aggregator/__init__.py
"""Detection aggregator — maps tile-local dets to source-frame and applies NMS + filters."""
from .nms import map_to_source, nms  # noqa: F401
```

```python
# hailo_tiling/tests/test_aggregator_nms.py
"""NMS + map_to_source — lift parity from dynamic_tiling.aggregator."""
from __future__ import annotations

from hailo_tiling.aggregator import map_to_source, nms
from hailo_tiling.types import CropRect, Det


class _CropLocalDet:
    """Mimics probe_phantom_hef.Detection (normalized in crop)."""
    def __init__(self, cls, x, y, w, h, score):
        self.cls, self.x, self.y, self.w, self.h, self.score = cls, x, y, w, h, score


def test_map_to_source_places_box_correctly():
    crop = CropRect(x=1000, y=500, w=640, h=480)
    d = _CropLocalDet(cls=0, x=0.25, y=0.25, w=0.5, h=0.5, score=0.9)
    out = map_to_source([d], crop, src_w=4000, src_h=3000)
    assert len(out) == 1
    o = out[0]
    assert abs(o.x - 1160 / 4000) < 1e-6
    assert abs(o.y - (500 + 0.25 * 480) / 3000) < 1e-6
    assert abs(o.w - (0.5 * 640) / 4000) < 1e-6
    assert abs(o.h - (0.5 * 480) / 3000) < 1e-6
    assert o.cls == 0
    assert abs(o.score - 0.9) < 1e-6


def test_nms_merges_overlapping_same_class():
    a = Det(cls=0, score=0.9, x=0.10, y=0.10, w=0.10, h=0.20)
    b = Det(cls=0, score=0.7, x=0.105, y=0.105, w=0.10, h=0.20)
    c = Det(cls=0, score=0.8, x=0.60, y=0.60, w=0.10, h=0.20)
    kept = nms([a, b, c], iou_thr=0.5)
    assert len(kept) == 2


def test_nms_keeps_different_classes():
    a = Det(cls=0, score=0.9, x=0.10, y=0.10, w=0.10, h=0.20)
    b = Det(cls=1, score=0.7, x=0.10, y=0.10, w=0.10, h=0.20)
    assert len(nms([a, b], iou_thr=0.5)) == 2


def test_parity_with_dynamic_tiling_aggregator_nms():
    """Verbatim parity check: identical inputs -> identical outputs."""
    from dynamic_tiling.aggregator import nms as legacy_nms

    dets = [
        Det(cls=0, score=0.9, x=0.10, y=0.10, w=0.10, h=0.20),
        Det(cls=0, score=0.7, x=0.105, y=0.105, w=0.10, h=0.20),
        Det(cls=0, score=0.8, x=0.60, y=0.60, w=0.10, h=0.20),
        Det(cls=1, score=0.9, x=0.10, y=0.10, w=0.10, h=0.20),
    ]
    assert nms(dets, iou_thr=0.5) == legacy_nms(dets, iou_thr=0.5)
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_aggregator_nms.py -v
```
Expected: `ImportError: cannot import name 'map_to_source' from 'hailo_tiling.aggregator'`.

- [ ] **Step 3: Implement `nms.py` (verbatim port of `dynamic_tiling/aggregator.py`).**

```python
# hailo_tiling/aggregator/nms.py
"""Per-class NMS + crop-local-to-source detection mapping.

Lifted verbatim from dynamic_tiling/aggregator.py (Plan 1 byte-parity).
"""
from __future__ import annotations

from typing import Iterable, Sequence

from ..types import CropRect, Det


def map_to_source(crop_dets: Iterable, crop: CropRect, src_w: int, src_h: int) -> list[Det]:
    """Map crop-local normalized detections into normalized full-frame Dets.

    `crop_dets` items expose .cls .x .y .w .h .score normalized within the crop.
    """
    out: list[Det] = []
    for d in crop_dets:
        src_x = crop.x + d.x * crop.w
        src_y = crop.y + d.y * crop.h
        src_bw = d.w * crop.w
        src_bh = d.h * crop.h
        out.append(Det(
            cls=int(d.cls), score=float(d.score),
            x=src_x / src_w, y=src_y / src_h,
            w=src_bw / src_w, h=src_bh / src_h,
        ))
    return out


def _iou(a: Det, b: Det) -> float:
    ax1, ay1, ax2, ay2 = a.xyxy
    bx1, by1, bx2, by2 = b.xyxy
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = a.w * a.h + b.w * b.h - inter
    return inter / ua if ua > 0 else 0.0


def nms(dets: Sequence[Det], iou_thr: float = 0.5) -> list[Det]:
    """Greedy per-class NMS over normalized full-frame detections."""
    kept: list[Det] = []
    for d in sorted(dets, key=lambda x: x.score, reverse=True):
        if all(not (k.cls == d.cls and _iou(k, d) >= iou_thr) for k in kept):
            kept.append(d)
    return kept
```

- [ ] **Step 4: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_aggregator_nms.py -v
```
Expected: 4 passed.

- [ ] **Step 5: Commit.**

```bash
git add hailo_tiling/aggregator/__init__.py \
        hailo_tiling/aggregator/nms.py \
        hailo_tiling/tests/test_aggregator_nms.py
git commit -m "hailo_tiling: aggregator — verbatim port of map_to_source + per-class NMS"
```

---

## Task 12: `BoundaryStripFilter` — Python port of the C++ `remove_exceeded_bboxes`

Spec §5 / `tiling_benchmark/PERF_REPORT.md` §8 — the C++ `remove_exceeded_bboxes(border_threshold)` strip drops detections whose bbox touches a tile's *interior* boundary (i.e., a boundary that is **not** also the source-frame edge). The threshold is the proximity (in normalized coords) below which the bbox is considered touching. Default 0.005, locked.

We implement a pure-Python equivalent that takes:
- A list of source-frame normalized `Det`s.
- The list of `CropRect`s the dets came from (so we can identify "tile interior" vs "frame edge").
- `border_threshold` (default 0.005).
- `src_w`, `src_h` (to identify frame-edge equivalence).

Only dets that originated from a tile with `mode == "m"` are subjected to the strip — single-scale (`mode == "s"`) tiles are exempt by convention. (See PERF_REPORT §8.6 "per-tile mode override".)

**Files:**
- Create: `hailo_tiling/aggregator/boundary_strip.py`
- Modify: `hailo_tiling/aggregator/__init__.py` (add export)
- Create: `hailo_tiling/tests/test_aggregator_boundary_strip.py`

- [ ] **Step 1: Write the failing test.**

```python
# hailo_tiling/tests/test_aggregator_boundary_strip.py
"""BoundaryStripFilter — Python port of C++ remove_exceeded_bboxes."""
from __future__ import annotations

from hailo_tiling.aggregator import BoundaryStripFilter
from hailo_tiling.types import CropRect, Det


def _tile_at(x, y, w, h, mode="m") -> CropRect:
    return CropRect(x=x, y=y, w=w, h=h, mode=mode)


SRC_W, SRC_H = 3840, 2160


def test_passes_dets_not_near_any_boundary():
    """A det in the center of a tile (and away from frame edges) is kept."""
    tile = _tile_at(0, 0, 1280, 960, mode="m")  # tile occupies upper-left third
    det = Det(cls=0, score=0.9, x=0.10, y=0.10, w=0.05, h=0.05)  # well inside
    flt = BoundaryStripFilter(border_threshold=0.005)
    out = flt.filter([det], [tile], SRC_W, SRC_H)
    assert out == [det]


def test_drops_det_touching_tile_interior_right_edge():
    """A det whose right edge lies within `border_threshold` of a tile's interior right
    boundary is dropped."""
    tile = _tile_at(0, 0, 1280, 960, mode="m")
    # tile right edge in normalized coords = 1280/3840 = 0.3333...
    # det right edge: x + w = 0.330 + 0.005 = 0.335 → within 0.005 of 0.333
    det = Det(cls=0, score=0.9, x=0.330, y=0.10, w=0.005, h=0.05)
    flt = BoundaryStripFilter(border_threshold=0.005)
    out = flt.filter([det], [tile], SRC_W, SRC_H)
    assert out == []


def test_keeps_det_touching_frame_edge_even_at_tile_boundary():
    """If a tile boundary IS the frame edge, dets touching it are kept."""
    # Right column tile: x = 2560, w = 1280 → right edge = 3840 (frame edge).
    tile = _tile_at(2560, 0, 1280, 960, mode="m")
    det = Det(cls=0, score=0.9, x=0.99, y=0.10, w=0.01, h=0.05)  # touches right frame edge
    flt = BoundaryStripFilter(border_threshold=0.005)
    out = flt.filter([det], [tile], SRC_W, SRC_H)
    assert out == [det]


def test_single_scale_tiles_are_exempt_from_strip():
    """Tiles with mode='s' (ROI/recovery) are exempt — dets pass through."""
    tile = _tile_at(0, 0, 1280, 960, mode="s")
    det = Det(cls=0, score=0.9, x=0.330, y=0.10, w=0.005, h=0.05)
    flt = BoundaryStripFilter(border_threshold=0.005)
    out = flt.filter([det], [tile], SRC_W, SRC_H)
    assert out == [det]


def test_default_threshold_is_0_005():
    flt = BoundaryStripFilter()
    assert flt.border_threshold == 0.005


def test_threshold_zero_disables_strip():
    tile = _tile_at(0, 0, 1280, 960, mode="m")
    det = Det(cls=0, score=0.9, x=0.330, y=0.10, w=0.005, h=0.05)
    flt = BoundaryStripFilter(border_threshold=0.0)
    out = flt.filter([det], [tile], SRC_W, SRC_H)
    assert out == [det]


def test_multiple_tiles_only_strips_against_originating_tile_mode():
    """Each det is checked against the *interior* edges of every 'm' tile in the
    list. Dets from 's' tiles still get tested against 'm'-tile interior edges
    only if that's documented behaviour — Plan 2 chooses the conservative
    rule: dets are stripped iff ANY 'm' tile's interior edge is within the
    threshold AND the det is NOT touching a frame edge."""
    tile_m = _tile_at(0, 0, 1280, 960, mode="m")
    tile_s = _tile_at(1280, 0, 1280, 960, mode="s")
    det = Det(cls=0, score=0.9, x=0.330, y=0.10, w=0.005, h=0.05)  # near tile_m right
    flt = BoundaryStripFilter(border_threshold=0.005)
    out = flt.filter([det], [tile_m, tile_s], SRC_W, SRC_H)
    assert out == []
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_aggregator_boundary_strip.py -v
```
Expected: `ImportError: cannot import name 'BoundaryStripFilter'`.

- [ ] **Step 3: Implement `BoundaryStripFilter`.**

```python
# hailo_tiling/aggregator/boundary_strip.py
"""BoundaryStripFilter — drops detections touching a tile's interior boundary.

Python port of the C++ `remove_exceeded_bboxes(border_threshold)` documented
in tiling_benchmark/PERF_REPORT.md §8. The default threshold (0.005) is the
same value used in the production drone-follow pipeline.

Semantics:
- For each 'm'-mode tile, compute its four edges in normalized source coords.
- For each edge that is NOT also the source-frame edge, mark it as "interior".
- A detection is stripped iff any of its four bbox edges lies within
  `border_threshold` of any interior edge AND that detection is not also
  touching a frame edge on that side.
- 's'-mode tiles (ROI / recovery) are exempt — they do not contribute
  interior edges to the strip set.

A `border_threshold` of 0.0 disables stripping entirely (matches the C++
`bypass-on-zero` rule documented in §8.7).
"""
from __future__ import annotations

from typing import Sequence

from ..types import CropRect, Det


def _interior_edges_norm(tile: CropRect, src_w: int, src_h: int
                         ) -> tuple[float | None, float | None, float | None, float | None]:
    """Return (left, right, top, bottom) interior edges in normalized coords.

    An edge is None if it coincides with the source-frame edge.
    """
    left = tile.x / src_w if tile.x > 0 else None
    right = (tile.x + tile.w) / src_w if (tile.x + tile.w) < src_w else None
    top = tile.y / src_h if tile.y > 0 else None
    bottom = (tile.y + tile.h) / src_h if (tile.y + tile.h) < src_h else None
    return left, right, top, bottom


class BoundaryStripFilter:
    """Filter detections whose bbox touches a tile's interior boundary."""

    name = "boundary_strip"

    def __init__(self, border_threshold: float = 0.005):
        self.border_threshold = border_threshold

    def filter(self, dets: Sequence[Det], tiles: Sequence[CropRect],
               src_w: int, src_h: int) -> list[Det]:
        if self.border_threshold <= 0.0:
            return list(dets)

        thr = self.border_threshold
        # Collect interior edges only from 'm' tiles.
        lefts: list[float] = []
        rights: list[float] = []
        tops: list[float] = []
        bottoms: list[float] = []
        for t in tiles:
            if t.mode != "m":
                continue
            l, r, top, bot = _interior_edges_norm(t, src_w, src_h)
            if l is not None: lefts.append(l)
            if r is not None: rights.append(r)
            if top is not None: tops.append(top)
            if bot is not None: bottoms.append(bot)

        out: list[Det] = []
        for d in dets:
            dx1, dy1, dx2, dy2 = d.xyxy
            # Skip if det touches a frame edge -> always keep that side.
            touches_left_frame = dx1 <= thr
            touches_right_frame = dx2 >= 1.0 - thr
            touches_top_frame = dy1 <= thr
            touches_bottom_frame = dy2 >= 1.0 - thr

            stripped = False
            if not touches_left_frame:
                # Left edge of det near any interior right edge of a tile?
                if any(abs(dx1 - r) < thr for r in rights):
                    stripped = True
                # Or near any interior left edge (det is hugging left side of a tile)?
                elif any(abs(dx1 - l) < thr for l in lefts):
                    stripped = True
            if not stripped and not touches_right_frame:
                if any(abs(dx2 - l) < thr for l in lefts):
                    stripped = True
                elif any(abs(dx2 - r) < thr for r in rights):
                    stripped = True
            if not stripped and not touches_top_frame:
                if any(abs(dy1 - b) < thr for b in bottoms):
                    stripped = True
                elif any(abs(dy1 - t) < thr for t in tops):
                    stripped = True
            if not stripped and not touches_bottom_frame:
                if any(abs(dy2 - t) < thr for t in tops):
                    stripped = True
                elif any(abs(dy2 - b) < thr for b in bottoms):
                    stripped = True

            if not stripped:
                out.append(d)
        return out
```

- [ ] **Step 4: Add the export.**

Update `hailo_tiling/aggregator/__init__.py`:

```python
"""Detection aggregator — maps tile-local dets to source-frame and applies NMS + filters."""
from .boundary_strip import BoundaryStripFilter  # noqa: F401
from .nms import map_to_source, nms  # noqa: F401
```

- [ ] **Step 5: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_aggregator_boundary_strip.py -v
```
Expected: 7 passed.

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/aggregator/boundary_strip.py \
        hailo_tiling/aggregator/__init__.py \
        hailo_tiling/tests/test_aggregator_boundary_strip.py
git commit -m "hailo_tiling: BoundaryStripFilter — Python port of remove_exceeded_bboxes"
```

---

## Task 13: `DetectionMemory` ABC + `NoOpMemory` default

Future-proofs the aggregator for the v2 detection-carry-forward feature. Plan 2 only ships the ABC + a no-op default.

**Files:**
- Create: `hailo_tiling/aggregator/memory.py`
- Modify: `hailo_tiling/aggregator/__init__.py` (add exports)
- Create: `hailo_tiling/tests/test_aggregator_memory.py`

- [ ] **Step 1: Write the failing test.**

```python
# hailo_tiling/tests/test_aggregator_memory.py
"""DetectionMemory ABC + NoOpMemory."""
from __future__ import annotations

import pytest

from hailo_tiling.aggregator import DetectionMemory, NoOpMemory
from hailo_tiling.types import Det


def test_cannot_instantiate_abc():
    with pytest.raises(TypeError):
        DetectionMemory()  # type: ignore[abstract]


def test_noop_memory_observe_then_predict_returns_empty():
    mem = NoOpMemory()
    mem.observe([Det(cls=0, score=0.9, x=0.1, y=0.1, w=0.1, h=0.1)], frame_idx=0)
    assert mem.predict(frame_idx=1) == []


def test_noop_memory_reset_is_safe():
    mem = NoOpMemory()
    mem.observe([Det(cls=0, score=0.9, x=0.1, y=0.1, w=0.1, h=0.1)], frame_idx=0)
    mem.reset()
    assert mem.predict(frame_idx=1) == []


def test_noop_memory_implements_abc():
    assert isinstance(NoOpMemory(), DetectionMemory)
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_aggregator_memory.py -v
```
Expected: `ImportError: cannot import name 'DetectionMemory'`.

- [ ] **Step 3: Implement `DetectionMemory` and `NoOpMemory`.**

```python
# hailo_tiling/aggregator/memory.py
"""DetectionMemory — interface for carry-forward of unscheduled-tile detections.

Plan 2 ships only the ABC + a NoOpMemory default. The full v2 implementation
(CarryForwardMemory) tracks (det → tile) mappings and resurfaces "missed"
detections on frames where the corresponding tile was not scheduled. See
spec §3.1 and `docs/research/2026-05-27-industry-tiling-drone-tracking.md`
§1.7 (Selective Tile Processing with Memory).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Iterable

from ..types import Det


class DetectionMemory(ABC):
    """Maintains a memory of recent detections; predicts forward when a tile is skipped."""

    @abstractmethod
    def observe(self, dets: Iterable[Det], frame_idx: int) -> None:
        """Update memory state with this frame's confirmed detections."""

    @abstractmethod
    def predict(self, frame_idx: int) -> list[Det]:
        """Return memory-carried detections to inject into this frame's results.

        For NoOpMemory this is always `[]`. CarryForwardMemory (v2) returns
        the last-known detections whose tiles were NOT scheduled this frame.
        """

    def reset(self) -> None:  # pragma: no cover - default no-op
        """Optional: clear all stored state."""


class NoOpMemory(DetectionMemory):
    """Default memory implementation — never carries detections forward."""

    name = "noop_memory"

    def observe(self, dets, frame_idx: int) -> None:
        return None

    def predict(self, frame_idx: int) -> list[Det]:
        return []

    def reset(self) -> None:
        return None
```

- [ ] **Step 4: Add the exports.**

Update `hailo_tiling/aggregator/__init__.py`:

```python
"""Detection aggregator — maps tile-local dets to source-frame and applies NMS + filters."""
from .boundary_strip import BoundaryStripFilter  # noqa: F401
from .memory import DetectionMemory, NoOpMemory  # noqa: F401
from .nms import map_to_source, nms  # noqa: F401
```

- [ ] **Step 5: Run the test, expect pass.**

Run:
```bash
pytest hailo_tiling/tests/test_aggregator_memory.py -v
```
Expected: 4 passed.

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/aggregator/memory.py \
        hailo_tiling/aggregator/__init__.py \
        hailo_tiling/tests/test_aggregator_memory.py
git commit -m "hailo_tiling: DetectionMemory ABC + NoOpMemory default (carry-forward is v2)"
```

---

## Task 14: Top-level `Aggregator.aggregate(...)` composition + integration test

Composes `map_to_source` → boundary strip → NMS → memory injection. `aggregate(frame_idx, crops, dets_per_crop, src_w, src_h, *, telemetry=None) -> list[Det]`.

**Files:**
- Create: `hailo_tiling/aggregator/aggregator.py`
- Modify: `hailo_tiling/aggregator/__init__.py` (add export)
- Create: `hailo_tiling/tests/test_aggregator_integration.py`

- [ ] **Step 1: Write the failing integration test.**

```python
# hailo_tiling/tests/test_aggregator_integration.py
"""Top-level Aggregator.aggregate — composes NMS + boundary strip + memory."""
from __future__ import annotations

from hailo_tiling.aggregator import (
    Aggregator,
    BoundaryStripFilter,
    NoOpMemory,
)
from hailo_tiling.backends import MockBackend
from hailo_tiling.types import CropRect, Det


class _CropLocalDet:
    def __init__(self, cls, x, y, w, h, score):
        self.cls, self.x, self.y, self.w, self.h, self.score = cls, x, y, w, h, score


def test_aggregate_maps_dedups_and_emits_source_normalized():
    """Two overlapping tile-local dets in the same class should NMS to one."""
    src_w, src_h = 4000, 3000
    crops = [
        CropRect(x=0, y=0, w=640, h=480, mode="s"),
        CropRect(x=640, y=0, w=640, h=480, mode="s"),
    ]
    # Both tiles produce a det at their right/left interior edge respectively,
    # which after map_to_source should overlap heavily in source coords.
    dets_per_crop = [
        [_CropLocalDet(cls=0, x=0.95, y=0.1, w=0.05, h=0.1, score=0.9)],
        [_CropLocalDet(cls=0, x=0.0,  y=0.1, w=0.05, h=0.1, score=0.7)],
    ]
    agg = Aggregator(boundary_strip=BoundaryStripFilter(border_threshold=0.0),
                     memory=NoOpMemory(), iou_thr=0.5)
    out = agg.aggregate(frame_idx=0, crops=crops, dets_per_crop=dets_per_crop,
                         src_w=src_w, src_h=src_h)
    assert len(out) == 1
    assert out[0].cls == 0
    assert out[0].score == 0.9   # higher-scoring kept


def test_aggregate_applies_boundary_strip():
    """A det touching an interior tile boundary is dropped."""
    src_w, src_h = 3840, 2160
    tile_left = CropRect(x=0, y=0, w=1280, h=960, mode="m")
    tile_right = CropRect(x=1280, y=0, w=1280, h=960, mode="m")
    # det fully inside tile_left but right edge sits exactly on the
    # interior boundary (1280/3840 = 0.3333).
    det_local = _CropLocalDet(cls=0, x=0.98, y=0.1, w=0.02, h=0.05, score=0.9)
    agg = Aggregator(boundary_strip=BoundaryStripFilter(border_threshold=0.005))
    out = agg.aggregate(frame_idx=0,
                         crops=[tile_left, tile_right],
                         dets_per_crop=[[det_local], []],
                         src_w=src_w, src_h=src_h)
    assert out == []


def test_aggregate_with_backend_e2e(make_mock_backend):
    """End-to-end: MockBackend returns canned dets, Aggregator emits source-frame Dets."""
    src_w, src_h = 4000, 3000
    crops = [CropRect(x=1000, y=500, w=640, h=480, mode="s")]
    canned = {
        (0, (1000, 500, 640, 480)): [
            Det(cls=0, score=0.9, x=0.25, y=0.25, w=0.5, h=0.5),
        ],
    }
    be = make_mock_backend(canned)
    dets_per_crop = be.infer(frame=None, crops=crops, frame_idx=0)
    agg = Aggregator()
    out = agg.aggregate(frame_idx=0, crops=crops, dets_per_crop=dets_per_crop,
                         src_w=src_w, src_h=src_h)
    assert len(out) == 1
    o = out[0]
    assert abs(o.x - 1160 / 4000) < 1e-6
    assert o.cls == 0


def test_aggregate_invokes_memory_observe_and_predict():
    """Aggregator must call memory.observe with the final det list and merge predict()."""
    class _RecordingMemory:
        def __init__(self):
            self.observed = []
            self._to_inject = [Det(cls=1, score=0.5, x=0.5, y=0.5, w=0.01, h=0.01)]
        def observe(self, dets, frame_idx):
            self.observed.append((frame_idx, list(dets)))
        def predict(self, frame_idx):
            return list(self._to_inject)
        def reset(self):
            pass

    mem = _RecordingMemory()
    src_w, src_h = 4000, 3000
    crops = [CropRect(x=0, y=0, w=640, h=480, mode="s")]
    dets = [[_CropLocalDet(cls=0, x=0.25, y=0.25, w=0.1, h=0.1, score=0.9)]]
    agg = Aggregator(memory=mem)
    out = agg.aggregate(frame_idx=0, crops=crops, dets_per_crop=dets,
                         src_w=src_w, src_h=src_h)
    # Memory predict() injects a class-1 det -> total 2 dets in the result.
    assert len(out) == 2
    assert any(d.cls == 1 for d in out)
    assert mem.observed[0][0] == 0
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_aggregator_integration.py -v
```
Expected: `ImportError: cannot import name 'Aggregator'`.

- [ ] **Step 3: Implement `Aggregator`.**

```python
# hailo_tiling/aggregator/aggregator.py
"""Top-level Aggregator that composes the per-stage filters.

Order:
1. map_to_source — convert crop-local tile-local dets to source-frame normalized.
2. boundary_strip — drop dets touching tile interior edges (if enabled).
3. nms — per-class greedy NMS.
4. memory.observe(final_dets) — update memory state.
5. memory.predict() — inject carry-forward dets (NoOpMemory returns []).

Returns a single flat list of source-frame normalized `Det`s for the tracker
to consume. See spec §4 (Data Flow).
"""
from __future__ import annotations

from typing import Iterable, Optional, Sequence

from ..telemetry import NULL_SNAPSHOT, TelemetrySnapshot
from ..types import CropRect, Det
from .boundary_strip import BoundaryStripFilter
from .memory import DetectionMemory, NoOpMemory
from .nms import map_to_source, nms


class Aggregator:
    """Composes map → boundary-strip → NMS → memory."""

    def __init__(
        self,
        boundary_strip: Optional[BoundaryStripFilter] = None,
        memory: Optional[DetectionMemory] = None,
        iou_thr: float = 0.5,
    ):
        self.boundary_strip = boundary_strip if boundary_strip is not None else BoundaryStripFilter()
        self.memory = memory if memory is not None else NoOpMemory()
        self.iou_thr = iou_thr

    def aggregate(
        self,
        frame_idx: int,
        crops: Sequence[CropRect],
        dets_per_crop: Sequence[Iterable],
        src_w: int,
        src_h: int,
        telemetry: TelemetrySnapshot = NULL_SNAPSHOT,
    ) -> list[Det]:
        if len(crops) != len(dets_per_crop):
            raise ValueError(
                f"crops and dets_per_crop length mismatch: {len(crops)} vs {len(dets_per_crop)}"
            )
        # 1. Map crop-local -> source-frame normalized.
        flat: list[Det] = []
        for crop, dets in zip(crops, dets_per_crop):
            flat.extend(map_to_source(dets, crop, src_w, src_h))
        # 2. Boundary strip.
        stripped = self.boundary_strip.filter(flat, crops, src_w, src_h)
        # 3. NMS.
        kept = nms(stripped, iou_thr=self.iou_thr)
        # 4. Memory observe.
        self.memory.observe(kept, frame_idx)
        # 5. Memory predict — inject carry-forward dets.
        injected = self.memory.predict(frame_idx)
        if injected:
            kept = nms(kept + list(injected), iou_thr=self.iou_thr)
        return kept
```

- [ ] **Step 4: Add the export.**

Update `hailo_tiling/aggregator/__init__.py`:

```python
"""Detection aggregator — maps tile-local dets to source-frame and applies NMS + filters."""
from .aggregator import Aggregator  # noqa: F401
from .boundary_strip import BoundaryStripFilter  # noqa: F401
from .memory import DetectionMemory, NoOpMemory  # noqa: F401
from .nms import map_to_source, nms  # noqa: F401
```

- [ ] **Step 5: Run the integration test.**

Run:
```bash
pytest hailo_tiling/tests/test_aggregator_integration.py -v
```
Expected: 4 passed.

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/aggregator/aggregator.py \
        hailo_tiling/aggregator/__init__.py \
        hailo_tiling/tests/test_aggregator_integration.py
git commit -m "hailo_tiling: top-level Aggregator.aggregate composes strip + NMS + memory"
```

---

## Task 15: Public-API re-exports + INDEX.md status flip

Expose the Plan 2 surface from `hailo_tiling` top-level and update the plans index.

**Files:**
- Modify: `hailo_tiling/__init__.py`
- Modify: `hailo_tiling/tests/test_public_api.py`
- Modify: `docs/superpowers/plans/INDEX.md`

- [ ] **Step 1: Extend the public-API test.**

Append to `hailo_tiling/tests/test_public_api.py`:

```python
def test_plan2_top_level_imports():
    import hailo_tiling as ht
    # Telemetry
    assert ht.TelemetryProvider is not None
    assert ht.TelemetrySnapshot is not None
    assert ht.NULL_SNAPSHOT is not None
    assert ht.StaticTelemetry is not None
    assert ht.RecordedTelemetry is not None
    assert ht.MavsdkTelemetry is not None
    # New modifiers
    assert ht.AltitudeZoomModifier is not None
    assert ht.AdaptiveSliceSizingModifier is not None
    # Backends
    assert ht.InferenceBackend is not None
    assert ht.MockBackend is not None
    assert ht.HefBackend is not None
    # Aggregator
    assert ht.Aggregator is not None
    assert ht.BoundaryStripFilter is not None
    assert ht.DetectionMemory is not None
    assert ht.NoOpMemory is not None
    assert ht.nms is not None
    assert ht.map_to_source is not None
```

- [ ] **Step 2: Run the test, see failure.**

Run:
```bash
pytest hailo_tiling/tests/test_public_api.py -v
```
Expected: FAIL with `AttributeError: module 'hailo_tiling' has no attribute 'TelemetryProvider'` (or similar).

- [ ] **Step 3: Extend `hailo_tiling/__init__.py`.**

Replace the file with:

```python
"""hailo_tiling — reusable dynamic-tiling library for Hailo inference pipelines.

See docs/superpowers/specs/2026-05-28-tiling-library-design.md.
"""
__version__ = "0.1.0.dev0"

# --- Plan 1 surface ---
from .budget import BudgetMeter
from .emitters import DiscoveryGridEmitter, RecoveryGridEmitter, TrackROIEmitter
from .modifiers import (
    AdaptiveSliceSizingModifier,
    AltitudeZoomModifier,
    BudgetTrimModifier,
)
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

# --- Plan 2 surface ---
from .telemetry import (
    MavsdkTelemetry,
    NULL_SNAPSHOT,
    RecordedTelemetry,
    StaticTelemetry,
    TelemetryProvider,
    TelemetrySnapshot,
)
from .backends import HefBackend, InferenceBackend, MockBackend
from .aggregator import (
    Aggregator,
    BoundaryStripFilter,
    DetectionMemory,
    NoOpMemory,
    map_to_source,
    nms,
)

__all__ = [
    "__version__",
    # Plan 1
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
    # Plan 2 — telemetry
    "TelemetryProvider",
    "TelemetrySnapshot",
    "NULL_SNAPSHOT",
    "StaticTelemetry",
    "RecordedTelemetry",
    "MavsdkTelemetry",
    # Plan 2 — modifiers
    "AltitudeZoomModifier",
    "AdaptiveSliceSizingModifier",
    # Plan 2 — backends
    "InferenceBackend",
    "MockBackend",
    "HefBackend",
    # Plan 2 — aggregator
    "Aggregator",
    "BoundaryStripFilter",
    "DetectionMemory",
    "NoOpMemory",
    "map_to_source",
    "nms",
]
```

- [ ] **Step 4: Re-run the public-API test.**

Run:
```bash
pytest hailo_tiling/tests/test_public_api.py -v
```
Expected: 2 passed (Plan 1 + Plan 2 tests).

- [ ] **Step 5: Run the entire test matrix.**

Run:
```bash
pytest hailo_tiling/tests dynamic_tiling/tests -v
```
Expected: every test passes. The total count should be at least Plan 1's 78 tests plus the new ones added across Tasks 1-14 (≈ 40+ new cases, target ≥ 115 total).

- [ ] **Step 6: Update INDEX.md.**

Edit `docs/superpowers/plans/INDEX.md`: change the Plan 2 status from `not started` to `done` and add a column or row pointer for this plan's filename. After the edit, row 2 should read:

```
| 2   | `2026-05-28-telemetry-modifiers-backends.md`         | 3, 4, 5     | done       |
```

- [ ] **Step 7: Commit.**

```bash
git add hailo_tiling/__init__.py \
        hailo_tiling/tests/test_public_api.py \
        docs/superpowers/plans/INDEX.md
git commit -m "hailo_tiling: Plan 2 public-API re-exports + flip INDEX status to done"
```

---

## Plan-wide success criteria (self-check before declaring this plan done)

- [ ] `python -c "import hailo_tiling; print(hailo_tiling.__version__)"` prints `0.1.0.dev0` and does not require `mavsdk` to be installed.
- [ ] `python -c "from hailo_tiling.telemetry import MavsdkTelemetry; MavsdkTelemetry(); print('ok')"` prints `ok` on a machine **without** `mavsdk` installed.
- [ ] `pytest hailo_tiling/tests -v` reports **all** tests passing. The count should be roughly Plan 1's 78 + Plan 2's ~40 = ≥ 115 cases. The one `mavsdk-installed` conditional skip is acceptable.
- [ ] `pytest dynamic_tiling/tests -v` reports the legacy suite still passing (the `dynamic_tiling/inference.py` shim is the load-bearing change here).
- [ ] `scripts/smoke_hailo_tiling.py` still exits 0 with 7 CropRects (Plan 1 smoke test, unchanged).
- [ ] No new hard dependencies in `pyproject.toml` (the `mavsdk` extra is unchanged from Plan 1).
- [ ] `dynamic_tiling/scheduler.py`, `dynamic_tiling/aggregator.py`, and the `MultiTargetTileScheduler` are unchanged.
- [ ] `hailo_tiling/emitters/discovery_grid.py` discovery-active guard at lines 43-47 is preserved (NOT moved).
- [ ] Every modifier, telemetry provider, backend, and aggregator stage has at least one explicit unit test.
- [ ] `AltitudeZoomModifier` has a degraded-mode test (NULL_SNAPSHOT → passthrough) and an end-to-end scheduler-level test.
- [ ] `AdaptiveSliceSizingModifier` has a "no telemetry" test.
- [ ] `HefBackend` test does not import or touch HailoRT (verified by `grep -n "HailoRT\|hailo_platform" hailo_tiling/tests/test_backend_hef_shim.py` returning nothing).
- [ ] `docs/superpowers/plans/INDEX.md` shows Plan 2 = done; Plan 3 = `in flight` or `not started` (whichever is appropriate at handoff).
- [ ] Every task closed with at least one commit on the current branch (`tiling-benchmark`); no force-pushes.

---

## Out of scope for this plan (handled later)

- **Discovery-grid recovery-active guard rehoming.** The guard at `hailo_tiling/emitters/discovery_grid.py:43-47` is a structural anomaly: the emitter encodes knowledge about the recovery branch. The cleaner layering routes this through telemetry / modifier composition (e.g., a "discovery-suppress-when-recovering" Modifier, or a richer `lock`/`telemetry` flag). Plan 2 keeps the guard in place for byte-parity with Plan 1; the cleaner design is on the table for Plan 3 or a dedicated refactor. See Open Questions §1.
- **Detection-memory carry-forward (v2 `CarryForwardMemory`).** Plan 2 ships only `DetectionMemory` ABC + `NoOpMemory`. A real implementation needs (det → tile-of-origin) bookkeeping, plus a policy for when a memory hit overrides a fresh empty inference. See Open Questions §2.
- **`GstCropperBackend`, `CachingBackend`, batched `ReplayBackend`.** Plan 4 (cache layer) and Plan 6 (harness) own these.
- **`tracking/` subpackage (`Tracker` ABC, `ByteTrackAdapter`).** A separate later plan; spec §2 calls it out as deferred.
- **MAVSDK live wiring inside `_pull_snapshot_sync`.** Plan 2 ships the seam + ImportError plumbing. The real asyncio implementation lands in Plan 8 (drone-follow migration).
- **`MultiTargetTileScheduler` migration.** Stays in `dynamic_tiling/` for v2.
- **Wider modifier signature using `FrameContext` / richer track state.** Plan 2 keeps the Plan-1-compatible `(src_w, src_h, lock, frame_idx, meter, telemetry)` signature. The richer `FrameContext`-based signature (spec §3.2) lands when the multi-target work moves into `hailo_tiling`.
- **Replacing `dynamic_tiling/aggregator.py` and `dynamic_tiling/inference.py` outright.** Both are now duplicates (aggregator) or shims (inference); a final cleanup plan after Plan 8 deletes the duplicates.

---

## Open Questions

1. **Discovery-grid recovery-active guard — rehome via telemetry/modifiers in Plan 3?**
   The guard at `hailo_tiling/emitters/discovery_grid.py:43-47` (added in Plan 1 Task 9 as a parity hack) couples the emitter to recovery semantics. The cleaner layering would route this through a `RecoveryActiveModifier` that runs *before* `BudgetTrimModifier` and drops `mode == "m"` tiles when `lock.status in {SEARCHING, LOST}` and `lock.track_id is not None`. Plan 3 (FOV emulation work) probably isn't the right home for this — propose handling it as a small dedicated cleanup task between Plans 3 and 4, OR rolling it into Plan 6 when `MultiTargetTileScheduler` migration forces a clean look at the recovery branch anyway.

2. **`DetectionMemory` v2 scope.** What's the minimum-viable carry-forward implementation? Two candidate designs:
   - **Tile-keyed:** each det records its originating tile; a det is "carry-forwardable" iff its tile was NOT scheduled this frame. Requires the aggregator to be told the *current* tile list, not just the *previous* one.
   - **Track-keyed (simpler):** rely on the tracker. The memory is the tracker's state; the aggregator just feeds the tracker. This collapses memory into the tracker subpackage.
   The spec (§1.7 research, §3.1 package layout) treats them as separate. Recommendation: revisit when `tracking/` lands.

3. **HEF lift strategy — shim vs hard move.** Plan 2 lands a shim (`dynamic_tiling/inference.py` re-exports `HefBackend`, keeps a legacy `ReplayBackend`). The cleaner alternative is a hard move + updating `dynamic_tiling/run_dynamic.py` and its two callers. The hard-move path touches `dynamic_tiling/run_dynamic.py` (which Plan 1 explicitly carved out as Plan-8 work) — so the shim is the right call for Plan 2. The shim disappears in Plan 8 (drone-follow migration). If the shim survives past Plan 8, that's a regression.

4. **Boundary-strip parity with C++.** The Python port in Task 12 implements the *documented* semantics from `PERF_REPORT.md` §8. We don't currently have an end-to-end Python ↔ C++ byte-parity test (it'd require running the full GStreamer aggregator on the same crop list). Recommendation: defer until Plan 6 stands up `GstCropperBackend` and we can record a reference frame to cross-check.

5. **`AdaptiveSliceSizingModifier` altitude integration.** Plan 2 uses only `lock.bbox_norm[3]` as the scale signal. The spec hints at altitude as a *secondary* signal (`docs/research/2026-05-27-...` §1.4). A small follow-up could weight the two signals. Out of scope for Plan 2; revisit after Plan 6 generates ablation data showing whether the bbox-only signal already captures the trend.

---

### Critical Files for Implementation

- /home/giladn/tappas_apps/repos/hailo-drone-follow/docs/superpowers/specs/2026-05-28-tiling-library-design.md
- /home/giladn/tappas_apps/repos/hailo-drone-follow/docs/superpowers/plans/2026-05-28-hailo-tiling-scaffold-and-scheduler-refactor.md
- /home/giladn/tappas_apps/repos/hailo-drone-follow/hailo_tiling/scheduler.py
- /home/giladn/tappas_apps/repos/hailo-drone-follow/hailo_tiling/emitters/discovery_grid.py
- /home/giladn/tappas_apps/repos/hailo-drone-follow/dynamic_tiling/inference.py
- /home/giladn/tappas_apps/repos/hailo-drone-follow/dynamic_tiling/aggregator.py
- /home/giladn/tappas_apps/repos/hailo-drone-follow/docs/research/2026-05-27-industry-tiling-drone-tracking.md
- /home/giladn/tappas_apps/repos/hailo-drone-follow/tiling_benchmark/PERF_REPORT.md
- /home/giladn/tappas_apps/repos/hailo-drone-follow/hailo_tiling/__init__.py
- /home/giladn/tappas_apps/repos/hailo-drone-follow/hailo_tiling/tests/conftest.py

---
