# Dynamic-Tiling Realtime Showcase Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prove the dynamic-tiling pipeline (decode → tile-crop → track → infer) sustains 4K@60fps on the laptop, and produce a visualizer-style single-vehicle (SOT) showcase, with striped dense tiling (no inference spike) and persisted dense detections.

**Architecture:** A new `StripedDenseScheduler` spreads the 8×6 dense grid round-robin over K=30 frames (~2fps full refresh at 60fps) so per-frame inference is flat. A `DetectionPersistence` cache holds the latest detections per grid cell so the visualizer draws stable boxes between refreshes. A new `run_showcase.py` runs the real GStreamer pipeline with **no overlay/no saved video** (fakesink tail), emits `frames.json` (visualizer schema) + `metrics.json` (achieved fps + per-frame tile stats). Source clips are ingested via the existing rotation-only `prepare_video.py` (no FOV, no resize).

**Tech Stack:** Python 3.10, GStreamer (`hailotilecropper_dynamic`, `hailotileaggregator`, `hailonet`), HailoRT (HAILO10H), pytest. Reuses `hailo_tiling.dynamic.scheduler.TileScheduler`, `hailo_tiling.budget.BudgetMeter`, `tiling_lab.harness.target_lock.TargetLock`, `tiling_lab.live.tiles_format.crops_to_tiles_static`.

**Run env:** Always `source setup_env.sh` first (activates `./hailo-apps/venv_hailo_apps`, exports PYTHONPATH). Run pytest with `./hailo-apps/venv_hailo_apps/bin/python -m pytest`.

---

## File Structure

- **Create** `hailo_tiling/dynamic/striped.py` — `StripedDenseScheduler` (round-robin dense stripes + ROI/recovery passthrough).
- **Create** `hailo_tiling/dynamic/persistence.py` — `DetectionPersistence` (per-cell latest-detection cache).
- **Create** `hailo_tiling/tests/test_striped_scheduler.py` — unit tests for the scheduler.
- **Create** `hailo_tiling/tests/test_persistence.py` — unit tests for persistence.
- **Modify** `tiling_lab/live/controller.py` — add opt-in `step_showcase()` path (striped + persist), leaving the existing `update()` string API intact.
- **Create** `tiling_lab/tests/test_showcase_controller.py` — unit test for the controller showcase path.
- **Create** `tiling_lab/live/run_showcase.py` — the no-draw realtime runner emitting `frames.json` + `metrics.json`.
- **Create** `tiling_lab/tests/test_run_showcase_smoke.py` — GStreamer integration smoke (device-gated).

Detection records use the **visualizer schema dict** end-to-end in the showcase path: `{"label": str, "confidence": float, "bbox": [x, y, w, h]}` (normalized). `TargetLock` consumes `Det` built from the target-class subset.

---

### Task 1: StripedDenseScheduler — stripe partitioning

**Files:**
- Create: `hailo_tiling/dynamic/striped.py`
- Test: `hailo_tiling/tests/test_striped_scheduler.py`

- [ ] **Step 1: Write the failing test**

```python
# hailo_tiling/tests/test_striped_scheduler.py
from hailo_tiling.dynamic.striped import StripedDenseScheduler


def test_stripes_partition_full_grid_with_no_overlap():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0)
    # 60/2 = 30 frames per full refresh.
    assert s.K == 30
    # 8*6 = 48 dense crops built once.
    assert len(s._dense) == 48
    # Union of all stripe index-sets over one cycle == every dense cell, once.
    seen = []
    for f in range(s.K):
        seen.extend(s.stripe_indices(f))
    assert sorted(seen) == list(range(48))


def test_stripes_are_interleaved_and_balanced():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0)
    sizes = [len(s.stripe_indices(f)) for f in range(s.K)]
    # 48 tiles over 30 stripes => sizes differ by at most 1.
    assert max(sizes) - min(sizes) <= 1
    # Interleaved: stripe 0 starts at cell 0, stripe 1 at cell 1, etc.
    assert s.stripe_indices(0)[0] == 0
    assert s.stripe_indices(1)[0] == 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && ./hailo-apps/venv_hailo_apps/bin/python -m pytest hailo_tiling/tests/test_striped_scheduler.py -v`
Expected: FAIL — `ModuleNotFoundError: hailo_tiling.dynamic.striped`.

- [ ] **Step 3: Write minimal implementation**

```python
# hailo_tiling/dynamic/striped.py
from __future__ import annotations

from .scheduler import TileScheduler
from ..types import CropRect, LockState

__all__ = ["StripedDenseScheduler"]


class StripedDenseScheduler:
    """Round-robin striped dense tiling.

    The dense grid (default 8x6, whole frame, multi-scale) is partitioned into
    K = round(fps / cadence_fps) interleaved stripes. Each frame emits exactly
    one stripe of dense tiles plus the wrapped v1 ROI/recovery tile(s), so the
    per-frame inference count is flat (no periodic discovery spike). A full
    dense refresh completes every K frames (~cadence_fps Hz).
    """

    def __init__(self, src_w: int, src_h: int, *,
                 dense_grid: tuple = (8, 6), fps: float = 60.0,
                 cadence_fps: float = 2.0, grid_overlap: float = 0.0,
                 **v1_kwargs):
        self.src_w = int(src_w)
        self.src_h = int(src_h)
        self.dense_grid = dense_grid
        self.K = max(1, int(round(fps / cadence_fps)))
        self._v1 = TileScheduler(self.src_w, self.src_h,
                                 grid_overlap=grid_overlap, **v1_kwargs)
        gx, gy = dense_grid
        # Row-major (j outer, i inner) => crop index == logical cell j*gx + i.
        self._dense = self._v1._grid(gx, gy, 0, 0,
                                     self.src_w, self.src_h, "m")
        self._stripes = [list(range(i, len(self._dense), self.K))
                         for i in range(self.K)]

    def stripe_indices(self, frame_idx: int) -> list[int]:
        """Logical dense-cell indices run on `frame_idx` (== crop indices)."""
        return self._stripes[frame_idx % self.K]
```

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && ./hailo-apps/venv_hailo_apps/bin/python -m pytest hailo_tiling/tests/test_striped_scheduler.py -v`
Expected: PASS (2 passed).

- [ ] **Step 5: Commit**

```bash
git add hailo_tiling/dynamic/striped.py hailo_tiling/tests/test_striped_scheduler.py
git commit -m "feat(tiling): StripedDenseScheduler stripe partitioning"
```

---

### Task 2: StripedDenseScheduler.decide — ROI + one stripe + budget + recovery

**Files:**
- Modify: `hailo_tiling/dynamic/striped.py`
- Test: `hailo_tiling/tests/test_striped_scheduler.py`

- [ ] **Step 1: Write the failing test**

```python
# append to hailo_tiling/tests/test_striped_scheduler.py
from hailo_tiling.types import LockState
from hailo_tiling.budget import BudgetMeter


class _Unlimited:
    def available(self, frame_idx):
        return 9999.0


def _tracking_lock():
    return LockState(track_id=7, bbox_norm=(0.5, 0.5, 0.04, 0.03),
                     status="TRACKING", frames_since_seen=0,
                     last_velocity=(0.0, 0.0))


def test_decide_emits_roi_first_then_one_stripe():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0)
    lock = _tracking_lock()
    crops = s.decide(lock, frame_idx=0, meter=_Unlimited())
    # ROI tile (single-scale) + stripe-0 dense tiles (multi-scale).
    n_stripe = len(s.stripe_indices(0))
    assert len(crops) == 1 + n_stripe
    assert crops[0].mode == "s"           # ROI first
    assert all(c.mode == "m" for c in crops[1:])  # dense tiles multi-scale


def test_decide_per_frame_count_is_flat_across_cycle():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0)
    lock = _tracking_lock()
    counts = [len(s.decide(lock, f, _Unlimited())) for f in range(s.K)]
    # ROI(1) + ~1.6 dense/frame => counts differ by at most 1 (no spike).
    assert max(counts) - min(counts) <= 1


def test_budget_trims_dense_stripe_before_roi():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0)
    lock = _tracking_lock()

    class _OneTile:
        def available(self, frame_idx):
            return 1.0
    crops = s.decide(lock, frame_idx=0, meter=_OneTile())
    assert len(crops) == 1
    assert crops[0].mode == "s"           # the ROI survives, dense dropped


def test_recovery_owns_frame_no_dense_stripe():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0,
                              recovery_grid=(3, 3))
    lock = LockState(track_id=7, bbox_norm=(0.5, 0.5, 0.04, 0.03),
                     status="SEARCHING", frames_since_seen=3,
                     last_velocity=(0.0, 0.0))
    crops = s.decide(lock, frame_idx=0, meter=_Unlimited())
    # 3x3 recovery grid, all single-scale, no multi-scale dense tiles.
    assert len(crops) == 9
    assert all(c.mode == "s" for c in crops)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && ./hailo-apps/venv_hailo_apps/bin/python -m pytest hailo_tiling/tests/test_striped_scheduler.py -v`
Expected: FAIL — `StripedDenseScheduler` has no attribute `decide`.

- [ ] **Step 3: Write minimal implementation**

Append to `hailo_tiling/dynamic/striped.py`:

```python
    def _target_crops(self, lock: LockState):
        """Return (crops, in_recovery). ROI when TRACKING; recovery grid when
        SEARCHING/LOST with a known track. Mirrors TileScheduler.decide minus
        the discovery grid (this class owns the dense pass)."""
        if lock.status in ("SEARCHING", "LOST") and lock.track_id is not None:
            gx, gy = self._v1.recovery_grid
            bx, by, bw, bh = lock.bbox_norm
            ecx = bx + bw / 2 + lock.last_velocity[0] * lock.frames_since_seen
            ecy = by + bh / 2 + lock.last_velocity[1] * lock.frames_since_seen
            span = self._v1.recovery_span
            half = span / 2
            x0_n = max(0.0, min(1.0 - span, ecx - half))
            y0_n = max(0.0, min(1.0 - span, ecy - half))
            crops = self._v1._grid(gx, gy, x0_n * self.src_w, y0_n * self.src_h,
                                   span * self.src_w, span * self.src_h, "s")
            return crops, True
        crops = []
        if lock.status == "TRACKING":
            crops.append(self._v1._roi(lock))
        return crops, False

    def decide(self, lock: LockState, frame_idx: int, meter) -> list[CropRect]:
        target_crops, in_recovery = self._target_crops(lock)
        if in_recovery:
            crops = target_crops          # recovery owns the frame
        else:
            stripe = [self._dense[i] for i in self.stripe_indices(frame_idx)]
            crops = target_crops + stripe  # ROI first, then dense stripe
        budget = int(meter.available(frame_idx))
        if budget >= 0 and len(crops) > budget:
            crops = crops[:max(0, budget)]
        return crops
```

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && ./hailo-apps/venv_hailo_apps/bin/python -m pytest hailo_tiling/tests/test_striped_scheduler.py -v`
Expected: PASS (6 passed).

- [ ] **Step 5: Commit**

```bash
git add hailo_tiling/dynamic/striped.py hailo_tiling/tests/test_striped_scheduler.py
git commit -m "feat(tiling): StripedDenseScheduler.decide — flat per-frame load, ROI-first, recovery"
```

---

### Task 3: DetectionPersistence — per-cell latest-detection cache

**Files:**
- Create: `hailo_tiling/dynamic/persistence.py`
- Test: `hailo_tiling/tests/test_persistence.py`

Detections are visualizer-schema dicts: `{"label", "confidence", "bbox": [x, y, w, h]}` (normalized). A detection is attributed to the dense cell containing its bbox center, in the same row-major `j*gx + i` indexing the scheduler uses.

- [ ] **Step 1: Write the failing test**

```python
# hailo_tiling/tests/test_persistence.py
from hailo_tiling.dynamic.persistence import DetectionPersistence


def _det(x, y, w=0.02, h=0.02, label="vehicle", conf=0.9):
    return {"label": label, "confidence": conf, "bbox": [x, y, w, h]}


def test_cell_of_maps_center_to_rowmajor_index():
    p = DetectionPersistence(dense_grid=(8, 6))
    # center near (0,0) -> cell 0; center near (1,1) -> last cell 8*6-1.
    assert p.cell_of(_det(0.0, 0.0)) == 0
    assert p.cell_of(_det(0.99, 0.99)) == 8 * 6 - 1
    # column 1 (i=1), row 0 (j=0) -> index 1.
    assert p.cell_of(_det(0.13, 0.0)) == 1


def test_update_replaces_only_run_cells_others_persist():
    p = DetectionPersistence(dense_grid=(8, 6))
    d_cell0 = _det(0.01, 0.01)     # cell 0
    d_cell1 = _det(0.13, 0.01)     # cell 1
    # Run cells {0,1}: both stored.
    p.update([0, 1], [d_cell0, d_cell1])
    assert len(p.published()) == 2
    # Run only cell {0} with a new det: cell 0 replaced, cell 1 persists.
    d_cell0b = _det(0.02, 0.02)
    p.update([0], [d_cell0b])
    pub = p.published()
    assert d_cell0 not in pub          # old cell-0 det gone
    assert d_cell0b in pub             # new cell-0 det present
    assert d_cell1 in pub              # cell-1 det persisted


def test_run_cell_with_no_detection_clears_that_cell():
    p = DetectionPersistence(dense_grid=(8, 6))
    p.update([0], [_det(0.01, 0.01)])
    assert len(p.published()) == 1
    # Re-run cell 0 with no detections falling in it -> cell emptied.
    p.update([0], [])
    assert p.published() == []


def test_detection_landing_outside_run_cells_is_ignored():
    p = DetectionPersistence(dense_grid=(8, 6))
    # Run cell {0} but the det belongs to cell 1 -> not stored.
    p.update([0], [_det(0.13, 0.01)])
    assert p.published() == []
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && ./hailo-apps/venv_hailo_apps/bin/python -m pytest hailo_tiling/tests/test_persistence.py -v`
Expected: FAIL — `ModuleNotFoundError: hailo_tiling.dynamic.persistence`.

- [ ] **Step 3: Write minimal implementation**

```python
# hailo_tiling/dynamic/persistence.py
from __future__ import annotations

__all__ = ["DetectionPersistence"]


class DetectionPersistence:
    """Per-cell latest-detection cache for striped dense tiling.

    Each dense grid cell keeps the most recent detections whose bbox center
    falls inside it. A cell refreshes only when its stripe is re-run (every K
    frames); between refreshes its detections persist, so the visualizer draws
    stable boxes. `published()` returns the union across all cells.

    Detections are visualizer-schema dicts:
        {"label": str, "confidence": float, "bbox": [x, y, w, h]}  (normalized)
    """

    def __init__(self, dense_grid: tuple = (8, 6)):
        self.gx, self.gy = dense_grid
        self._cells: dict[int, list[dict]] = {}

    def cell_of(self, det: dict) -> int:
        x, y, w, h = det["bbox"]
        cx = x + w / 2.0
        cy = y + h / 2.0
        i = min(self.gx - 1, max(0, int(cx * self.gx)))
        j = min(self.gy - 1, max(0, int(cy * self.gy)))
        return j * self.gx + i

    def update(self, run_cells, dets) -> None:
        """Refresh `run_cells`: clear them, then refill from `dets` that land
        in them. Cells not in `run_cells` are left untouched (persist)."""
        run = set(run_cells)
        for c in run:
            self._cells[c] = []
        for d in dets:
            c = self.cell_of(d)
            if c in run:
                self._cells[c].append(d)

    def published(self) -> list[dict]:
        out: list[dict] = []
        for ds in self._cells.values():
            out.extend(ds)
        return out
```

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && ./hailo-apps/venv_hailo_apps/bin/python -m pytest hailo_tiling/tests/test_persistence.py -v`
Expected: PASS (4 passed).

- [ ] **Step 5: Commit**

```bash
git add hailo_tiling/dynamic/persistence.py hailo_tiling/tests/test_persistence.py
git commit -m "feat(tiling): DetectionPersistence per-cell latest-detection cache"
```

---

### Task 4: Controller showcase path — step_showcase()

**Files:**
- Modify: `tiling_lab/live/controller.py`
- Test: `tiling_lab/tests/test_showcase_controller.py`

`step_showcase(target_dets, all_dets)` orchestrates one frame: steps the lock on the target-class `Det`s, asks the `StripedDenseScheduler` for crops, charges the budget, updates persistence with this frame's stripe, and returns `(tiles_static_str, record_dets)`. `record_dets` = the live SOT detection (when TRACKING) plus the persisted dense union with the SOT's own box removed (IoU > 0.5) to avoid a double box. The existing `update()` string API is unchanged.

- [ ] **Step 1: Write the failing test**

```python
# tiling_lab/tests/test_showcase_controller.py
from hailo_tiling.types import Det
from tiling_lab.live.controller import DynamicTilingController


def _det_dict(x, y, w=0.04, h=0.03, label="vehicle", conf=0.9):
    return {"label": label, "confidence": conf, "bbox": [x, y, w, h]}


def _target_det(x, y, w=0.04, h=0.03, conf=0.9):
    # cls value is irrelevant to TargetLock (it locks the largest).
    return Det(cls=2, score=conf, x=x, y=y, w=w, h=h)


def test_step_showcase_returns_tiles_string_and_records():
    ctrl = DynamicTilingController(3840, 2160, fps=60.0, budget_inf_per_s=300.0,
                                   striped=True, persist=True,
                                   dense_grid=(8, 6), cadence_fps=2.0)
    tgt = _target_det(0.5, 0.5)
    others = [_det_dict(0.5, 0.5, label="vehicle"),   # the target itself
              _det_dict(0.1, 0.1, label="person")]    # background context
    tiles, records = ctrl.step_showcase([tgt], others)
    assert isinstance(tiles, str) and tiles          # non-empty tiles-static
    assert isinstance(records, list)
    # Background person should appear; lock takes a few frames, so just assert
    # the persisted union is exposed.
    labels = {r["label"] for r in records}
    assert "person" in labels


def test_step_showcase_flat_tile_count_no_spike():
    ctrl = DynamicTilingController(3840, 2160, fps=60.0, budget_inf_per_s=300.0,
                                   striped=True, persist=True,
                                   dense_grid=(8, 6), cadence_fps=2.0)
    tgt = _target_det(0.5, 0.5)
    counts = []
    for _ in range(30):
        tiles, _records = ctrl.step_showcase([tgt], [_det_dict(0.5, 0.5)])
        counts.append(tiles.count(";") + 1 if tiles else 0)
    assert max(counts) - min(counts) <= 1            # no discovery spike
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && ./hailo-apps/venv_hailo_apps/bin/python -m pytest tiling_lab/tests/test_showcase_controller.py -v`
Expected: FAIL — `DynamicTilingController.__init__() got an unexpected keyword argument 'striped'`.

- [ ] **Step 3: Write minimal implementation**

Edit `tiling_lab/live/controller.py`. Add imports near the top:

```python
from hailo_tiling.dynamic.striped import StripedDenseScheduler
from hailo_tiling.dynamic.persistence import DetectionPersistence
```

Extend `__init__` signature and body (keep the existing `TileScheduler` path for the legacy `update()`):

```python
    def __init__(self, src_w: int, src_h: int, *, fps: float = 30.0,
                 budget_inf_per_s: float = 60.0, track_buffer: int = 90,
                 scheduler_kwargs: dict | None = None,
                 striped: bool = False, persist: bool = False,
                 dense_grid: tuple = (8, 6), cadence_fps: float = 2.0):
        self.src_w = int(src_w)
        self.src_h = int(src_h)
        self.striped = striped
        if striped:
            self._sched = StripedDenseScheduler(
                self.src_w, self.src_h, dense_grid=dense_grid, fps=float(fps),
                cadence_fps=cadence_fps, **(scheduler_kwargs or {}))
        else:
            self._sched = TileScheduler(self.src_w, self.src_h,
                                        **(scheduler_kwargs or {}))
        self._lock = TargetLock(track_buffer=track_buffer)
        self._meter = BudgetMeter(budget_inf_per_s=float(budget_inf_per_s),
                                  fps=float(fps))
        self._persist = DetectionPersistence(dense_grid) if persist else None
        self._frame = 0
        self._total_tiles = 0
```

Add the showcase step method (with a small IoU helper) below `update()`:

```python
    @staticmethod
    def _iou(a, b) -> float:
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        ix1 = max(ax, bx); iy1 = max(ay, by)
        ix2 = min(ax + aw, bx + bw); iy2 = min(ay + ah, by + bh)
        iw = max(0.0, ix2 - ix1); ih = max(0.0, iy2 - iy1)
        inter = iw * ih
        ua = aw * ah + bw * bh - inter
        return inter / ua if ua > 0 else 0.0

    def step_showcase(self, target_dets, all_dets):
        """Step one frame for the showcase runner.

        target_dets : Sequence[Det]   target-class detections (for the lock)
        all_dets    : Sequence[dict]  ALL detections, visualizer-schema dicts
        Returns (tiles_static_str, record_dets) where record_dets is the live
        SOT detection plus the persisted dense union (SOT box de-duplicated).
        """
        if not self.striped:
            raise RuntimeError("step_showcase requires striped=True")
        self._lock.step(list(target_dets), lock_if_unlocked=True)
        crops = self._sched.decide(self._lock.state, self._frame, self._meter)
        self._meter.charge(len(crops), self._frame)
        self._total_tiles += len(crops)

        records: list[dict] = []
        target_bbox = None
        st = self._lock.state
        if st.status == "TRACKING" and st.bbox_norm[2] > 0:
            target_bbox = tuple(st.bbox_norm)
            # Best confidence among target dets for the live box label.
            conf = max((d.score for d in target_dets), default=1.0)
            records.append({"label": "target", "confidence": float(conf),
                            "bbox": list(target_bbox)})

        if self._persist is not None:
            self._persist.update(self._sched.stripe_indices(self._frame),
                                 list(all_dets))
            for d in self._persist.published():
                if target_bbox is not None and \
                        self._iou(target_bbox, tuple(d["bbox"])) > 0.5:
                    continue
                records.append(d)

        tiles = crops_to_tiles_static(crops, self.src_w, self.src_h)
        self._frame += 1
        return tiles, records
```

Note: `crops_to_tiles_static` is already imported at module top.

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && ./hailo-apps/venv_hailo_apps/bin/python -m pytest tiling_lab/tests/test_showcase_controller.py tiling_lab/tests/test_live_controller.py -v`
Expected: PASS — new tests pass AND the existing `test_live_controller.py` still passes (legacy `update()` untouched).

- [ ] **Step 5: Commit**

```bash
git add tiling_lab/live/controller.py tiling_lab/tests/test_showcase_controller.py
git commit -m "feat(tiling): controller step_showcase — striped + persisted detections"
```

---

### Task 5: run_showcase.py — no-draw realtime runner with metrics

**Files:**
- Create: `tiling_lab/live/run_showcase.py`
- Test: `tiling_lab/tests/test_run_showcase_smoke.py`

This runs the real pipeline. The smoke test is device-gated (skips when no Hailo device or no GStreamer). The runner mirrors `run_live.py`'s pipeline construction but ends in `fakesink` (no overlay, no encode, no mkv) and writes `frames.json` + `metrics.json`.

- [ ] **Step 1: Write the failing test**

```python
# tiling_lab/tests/test_run_showcase_smoke.py
import json
import os
import shutil
import subprocess
import sys

import pytest

CLIP = "/home/giladn/Videos/Drone/Training/Car/DJI_20260614161040_0013_D.MP4"
HEF = "/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef"


def _have_device():
    if not (os.path.exists(CLIP) and os.path.exists(HEF)):
        return False
    if shutil.which("hailortcli") is None:
        return False
    r = subprocess.run(["hailortcli", "fw-control", "identify"],
                       capture_output=True, text=True)
    return r.returncode == 0


@pytest.mark.skipif(not _have_device(),
                    reason="requires Hailo device + 0013 clip + HEF")
def test_run_showcase_emits_frames_and_metrics(tmp_path):
    out = tmp_path / "showcase"
    cmd = [sys.executable, "-m", "tiling_lab.live.run_showcase",
           "--video", CLIP, "--out", str(out),
           "--frames", "300", "--fps", "60", "--target-class", "vehicle"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stderr

    frames = json.loads((out / "frames.json").read_text())
    assert frames["frames"], "no frames recorded"
    f0 = frames["frames"][0]
    assert set(["frame", "detections", "tiles"]).issubset(f0.keys())

    metrics = json.loads((out / "metrics.json").read_text())
    for k in ["frames", "wall_s", "achieved_fps", "mean_tiles_per_frame",
              "max_tiles_per_frame", "sustains_60fps", "source_fps"]:
        assert k in metrics
    # The spike is gone: max per-frame tiles is close to the mean.
    assert metrics["max_tiles_per_frame"] - metrics["mean_tiles_per_frame"] <= 2
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && ./hailo-apps/venv_hailo_apps/bin/python -m pytest tiling_lab/tests/test_run_showcase_smoke.py -v`
Expected: FAIL — `No module named tiling_lab.live.run_showcase` (or the subprocess returns nonzero). If it SKIPS, the device/clip is unavailable; note it and proceed — the acceptance run in Task 6 covers the device path.

- [ ] **Step 3: Write minimal implementation**

```python
# tiling_lab/live/run_showcase.py
"""Run striped dynamic tiling live, NO drawing / NO saved video.

Validates that decode + tile-crop + track + infer sustains the source fps and
captures per-frame data for the offline visualizer.

    source setup_env.sh
    python -m tiling_lab.live.run_showcase \
        --video /home/giladn/Videos/Drone/Training/Car/DJI_20260614161040_0013_D.MP4 \
        --out tiling_lab/runs/showcase_0013 --fps 60 --target-class vehicle

Pipeline: filesrc -> decodebin -> hailotilecropper_dynamic -> hailonet infer
-> hailotileaggregator -> fakesink. A probe on the aggregator src steps the
striped controller and pushes next-frame tiles onto the cropper. Outputs
frames.json (visualizer schema) and metrics.json (achieved fps + tile stats).
"""
import argparse
import json
import os
import statistics
import subprocess
import sys
import time

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib  # noqa: E402
import hailo  # noqa: E402

from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
    SOURCE_PIPELINE, INFERENCE_PIPELINE,
)

from hailo_tiling.types import Det
from tiling_lab.live.controller import DynamicTilingController

DEFAULT_HEF = "/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef"
DEFAULT_SO = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"
DEFAULT_FUNC = "filter"
DEFAULT_LABELS = "/usr/local/hailo/resources/json/hailo_4_classes.json"

# 8x6 dense seed for frame 0 (multi-scale), before any lock exists.
INITIAL_TILES = ("0.0,0.0,0.18,0.18,m;0.40,0.0,0.18,0.18,m;"
                 "0.80,0.0,0.18,0.18,m;0.40,0.40,0.18,0.18,m")


def probe_dims(video):
    out = subprocess.check_output([
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=width,height", "-of", "csv=p=0:s=x", video,
    ]).decode().strip()
    w, h = out.split("x")
    return int(w), int(h)


def build_pipeline(video, hef, post_so, func, labels, w, h, fps):
    inner = INFERENCE_PIPELINE(hef_path=hef, post_process_so=post_so,
                               post_function_name=func, batch_size=1,
                               config_json=labels, name="show_infer")
    src = SOURCE_PIPELINE(video_source=video, video_width=w, video_height=h,
                          frame_rate=fps, sync=False)
    return (
        f"{src} ! queue name=show_in_q ! "
        f"hailotilecropper_dynamic name=tc internal-offset=true "
        f"tiling-mode=single-scale tiles-static=\"{INITIAL_TILES}\" "
        f"hailotileaggregator name=agg flatten-detections=true iou-threshold=0.3 "
        f"tc. ! queue name=show_bypass_q ! agg.sink_0 "
        f"tc. ! video/x-raw,format=RGB ! {inner} ! agg.sink_1 "
        f"agg. ! queue name=show_out_q ! fakesink sync=false"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=0, help="0 = whole clip")
    ap.add_argument("--fps", type=float, default=60.0)
    ap.add_argument("--budget", type=float, default=300.0)
    ap.add_argument("--target-class", default="vehicle")
    ap.add_argument("--dense-grid", default="8x6")
    ap.add_argument("--cadence-fps", type=float, default=2.0)
    ap.add_argument("--label", default="showcase")
    ap.add_argument("--hef", default=DEFAULT_HEF)
    ap.add_argument("--post-so", default=DEFAULT_SO)
    ap.add_argument("--func", default=DEFAULT_FUNC)
    ap.add_argument("--labels", default=DEFAULT_LABELS)
    args = ap.parse_args()

    gx, gy = (int(v) for v in args.dense_grid.lower().split("x"))
    os.makedirs(args.out, exist_ok=True)
    w, h = probe_dims(args.video)
    print(f"[showcase] dims {w}x{h} fps={args.fps} target={args.target_class}",
          flush=True)

    Gst.init(None)
    pipeline = Gst.parse_launch(build_pipeline(
        args.video, args.hef, args.post_so, args.func, args.labels,
        w, h, args.fps))
    cropper = pipeline.get_by_name("tc")
    agg = pipeline.get_by_name("agg")

    ctrl = DynamicTilingController(
        src_w=w, src_h=h, fps=args.fps, budget_inf_per_s=args.budget,
        striped=True, persist=True, dense_grid=(gx, gy),
        cadence_fps=args.cadence_fps)

    frame_records = []
    tile_counts = []
    state = {"frame": 0, "t0": None, "t_last": None}
    loop = GLib.MainLoop()

    def probe(pad, info):
        if state["t0"] is None:
            state["t0"] = time.perf_counter()
        buf = info.get_buffer()
        roi = hailo.get_roi_from_buffer(buf)
        dets = roi.get_objects_typed(hailo.HAILO_DETECTION)
        all_dets = []
        target_dets = []
        for d in dets:
            b = d.get_bbox()
            rec = {"label": d.get_label(), "confidence": d.get_confidence(),
                   "bbox": [b.xmin(), b.ymin(), b.width(), b.height()]}
            all_dets.append(rec)
            if d.get_label() == args.target_class:
                target_dets.append(Det(cls=0, score=d.get_confidence(),
                                       x=b.xmin(), y=b.ymin(),
                                       w=b.width(), h=b.height()))
        tiles, records = ctrl.step_showcase(target_dets, all_dets)
        pushed = tiles or INITIAL_TILES
        cropper.set_property("tiles-static", pushed)
        n_tiles = pushed.count(";") + 1 if pushed else 0
        tile_counts.append(n_tiles)
        f = state["frame"]
        frame_records.append({"frame": f, "detections": records,
                              "tiles": pushed})
        if f % 60 == 0:
            print(f"[showcase] frame {f} status={ctrl.status} "
                  f"n_tiles={n_tiles}", flush=True)
        state["frame"] += 1
        state["t_last"] = time.perf_counter()
        if args.frames and state["frame"] >= args.frames:
            loop.quit()
        return Gst.PadProbeReturn.OK

    agg.get_static_pad("src").add_probe(Gst.PadProbeType.BUFFER, probe)

    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_msg(_bus, msg):
        if msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print(f"[showcase][BUS-ERROR] {err}: {dbg}", flush=True)
            loop.quit()
        elif msg.type == Gst.MessageType.EOS:
            print("[showcase] EOS", flush=True)
            loop.quit()
        return True

    bus.connect("message", on_msg)
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)

    n = len(frame_records)
    wall_s = (state["t_last"] - state["t0"]) if (state["t0"] and n > 1) else 0.0
    achieved = (n / wall_s) if wall_s > 0 else 0.0
    counts = tile_counts or [0]
    metrics = {
        "frames": n,
        "wall_s": round(wall_s, 3),
        "achieved_fps": round(achieved, 2),
        "source_fps": args.fps,
        "sustains_60fps": bool(achieved >= 60.0),
        "mean_tiles_per_frame": round(statistics.mean(counts), 3),
        "max_tiles_per_frame": max(counts),
        "p95_tiles_per_frame": sorted(counts)[int(0.95 * (len(counts) - 1))],
    }
    with open(os.path.join(args.out, "frames.json"), "w") as fp:
        json.dump({"label": args.label, "frames": frame_records}, fp)
    with open(os.path.join(args.out, "metrics.json"), "w") as fp:
        json.dump(metrics, fp, indent=2)
    print(f"[showcase] {n} frames, achieved_fps={achieved:.2f} "
          f"sustains_60fps={metrics['sustains_60fps']} "
          f"mean_t/f={metrics['mean_tiles_per_frame']} "
          f"max_t/f={metrics['max_tiles_per_frame']}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Run test to verify it passes (or skips cleanly)**

Run: `source setup_env.sh && ./hailo-apps/venv_hailo_apps/bin/python -m pytest tiling_lab/tests/test_run_showcase_smoke.py -v`
Expected: PASS on the device. If it SKIPS (no device), that is acceptable here; Task 6 runs it for real.

- [ ] **Step 5: Commit**

```bash
git add tiling_lab/live/run_showcase.py tiling_lab/tests/test_run_showcase_smoke.py
git commit -m "feat(tiling): run_showcase — no-draw realtime runner, frames.json + metrics.json"
```

---

### Task 6: Ingest + acceptance run + visualizer render

**Files:**
- Reuse: `tiling_lab/video/prepare_video.py`, `tiling_lab/viewer/overlay_viewer.py`
- Create: `tiling_lab/runs/showcase_0013/` (run artifacts; not committed)
- Create: `tiling_lab/runs/showcase_README.md` (results summary; committed)

This task is run-and-verify (no new unit code). It exercises the full flow on the device.

- [ ] **Step 1: Ingest the build clip (rotation-only, no FOV, no resize)**

Run:
```bash
source setup_env.sh
python -m tiling_lab.video.prepare_video \
  /home/giladn/Videos/Drone/Training/Car/DJI_20260614161040_0013_D.MP4 --verify
```
Expected: prints `rotation=0` and `video is already landscape-oriented; using directly: ...` (these clips carry no rotation metadata), with `--verify` confirming the gst one-frame decode matches 3840×2160. No resized/re-encoded output is produced (correct — no FOV, no resize).

- [ ] **Step 2: Acceptance run — 300-frame quick check**

Run:
```bash
source setup_env.sh
python -m tiling_lab.live.run_showcase \
  --video /home/giladn/Videos/Drone/Training/Car/DJI_20260614161040_0013_D.MP4 \
  --out tiling_lab/runs/showcase_0013 --frames 300 --fps 60 --target-class vehicle
cat tiling_lab/runs/showcase_0013/metrics.json
```
Expected: `metrics.json` with `max_tiles_per_frame - mean_tiles_per_frame <= 2` (spike gone). Record `achieved_fps`.

- [ ] **Step 3: Acceptance run — full clip**

Run:
```bash
source setup_env.sh
python -m tiling_lab.live.run_showcase \
  --video /home/giladn/Videos/Drone/Training/Car/DJI_20260614161040_0013_D.MP4 \
  --out tiling_lab/runs/showcase_0013 --fps 60 --target-class vehicle
cat tiling_lab/runs/showcase_0013/metrics.json
```
Expected: full ~2200 frames processed; `achieved_fps` reported. If `sustains_60fps` is false, capture the value and check whether the bottleneck is HEVC decode (see Step 5).

- [ ] **Step 4: Render the visualizer showcase**

Run (the viewer CLI; confirm exact flags with `python -m tiling_lab.viewer.overlay_viewer --help` first):
```bash
source setup_env.sh
python -m tiling_lab.viewer.overlay_viewer --help
# then render frames.json over the source clip per its documented flags, e.g.:
python -m tiling_lab.viewer.overlay_viewer \
  --video /home/giladn/Videos/Drone/Training/Car/DJI_20260614161040_0013_D.MP4 \
  --pred tiling_lab/runs/showcase_0013/frames.json \
  --out tiling_lab/runs/showcase_0013/showcase_overlay.mp4
```
Expected: an overlay video where the locked vehicle is tracked smoothly, dense context boxes are stable between ~0.5s refreshes (no flicker), and tiles render visualizer-style. Adjust flags to match `overlay_viewer.py`'s actual CLI.

- [ ] **Step 5 (conditional): Decode-only baseline if 60fps missed**

If Step 3 reports `sustains_60fps=false`, measure whether decode is the bottleneck:
```bash
source setup_env.sh
gst-launch-1.0 -v filesrc location=/home/giladn/Videos/Drone/Training/Car/DJI_20260614161040_0013_D.MP4 \
  ! decodebin ! videoconvert ! fakesink sync=false 2>&1 | tail -5
```
Note the decode-only throughput. On the RPi target the camera path is decode-free, so the crop+track+infer budget is the figure that matters there. Document the finding.

- [ ] **Step 6: Write results summary + commit**

Create `tiling_lab/runs/showcase_README.md` summarizing: clip, achieved_fps (300-frame + full), per-frame tile stats (mean/max/p95, demonstrating the flat load vs the old cadence spike), persistence behavior observed, and the visualizer output path. Then:
```bash
git add tiling_lab/runs/showcase_README.md
git commit -m "docs(tiling): showcase realtime validation results on 0013 (4K60)"
```

---

## Self-Review

**Spec coverage:**
- Ingest (rotation-only, no FOV, no resize) → Task 6 Step 1 (reuses prepare_video).
- Realtime 60fps validation → Task 5 (runner + metrics) + Task 6 Steps 2-3, 5.
- Striped dense tiling (no spike) → Tasks 1-2 + flat-count assertions in Tasks 2, 4, 5.
- Detection persistence → Task 3 + Task 4 (controller composes live SOT + persisted union).
- Single-object (vehicle) tracking → Task 4 (TargetLock on target-class Dets) + `--target-class` flag.
- Visualizer-style showcase → Task 6 Step 4 (overlay_viewer).

**Placeholder scan:** None. Every code step shows complete code; the only deferred detail is `overlay_viewer.py`'s exact CLI flags (Task 6 Step 4 instructs verifying with `--help` first, since that file was not modified by this plan).

**Type consistency:** `StripedDenseScheduler.decide(lock, frame_idx, meter) -> list[CropRect]` matches `crops_to_tiles_static`. `stripe_indices(frame_idx)` returns logical cell indices used by both the scheduler (crop selection) and persistence (`run_cells`). Detection dicts use `{"label","confidence","bbox":[x,y,w,h]}` consistently across persistence, controller, and runner. `DynamicTilingController` keeps `update()` (string) and adds `step_showcase()` (tuple) — no signature collision.
