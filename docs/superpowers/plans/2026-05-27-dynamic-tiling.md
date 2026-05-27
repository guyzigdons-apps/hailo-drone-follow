# Dynamic Tiling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an offline harness with a dynamic tile scheduler that, given a fixed tile-inference/sec budget, locks onto a single target and concentrates inference on it to beat static grids' per-target recall/IoU at equal budget.

**Architecture:** New `dynamic_tiling/` package. Per frame, `TileScheduler.decide()` returns source-px crop rects (decimated discovery grid + an adaptive-zoom ROI tile on the locked target + recovery grid on loss); each crop is **really** inferred via the existing `HefHandle`; detections are mapped to source px, NMS-merged, fed to the pipeline's ByteTracker; a `TargetLock` follows one track_id; results are scored against the GT and a viewer-compatible `frames.json` is emitted.

**Tech Stack:** Python 3, NumPy, OpenCV, matplotlib, HailoRT (via `tiling_benchmark/probe_phantom_hef.py::HefHandle`), the pipeline ByteTracker (`drone_follow/pipeline_adapter/tracker_factory.py`), pytest.

**Spec:** `docs/superpowers/specs/2026-05-27-dynamic-tiling-design.md`

---

## Conventions used throughout

- **Source px:** integer pixel coords in the full recorded frame (`src_W × src_H`, read at runtime from the video).
- **Normalized:** `[0..1]` frame fractions, `bbox = (x, y, w, h)` (top-left + size), matching the existing `frames.json` schema.
- **Model input:** the HEF expects a `(480, 640, 3)` RGB uint8 array. `MODEL_W=640`, `MODEL_H=480`, `MODEL_ASPECT=640/480`.
- **Zoom / scale:** `scale = MODEL_W / crop_w_src`. `crop_w_src = 640` → scale 1.0 (native: src-px == model-px). `crop_w_src = 320` → scale 2.0 (x2 zoom). v1 clamps scale to `[1.0, 2.0]`.
- **ByteTracker detection array:** `np.ndarray` shape `(N, 5)` float32, columns `[xmin, ymin, xmax, ymax, score]` in **SCALE=1000 units** (normalized × 1000), as built in `hailo_drone_detection_manager.py:112-120`. The adapter returns `TrackedObject.filtered_tlwh` already divided back to normalized `[0..1]`.
- **Person class id = 0** (HEF `4_classes`: person=0, vehicle=1, face=2, license_plate=3). v1 locks a person.

## Reused interfaces (do not reimplement)

From `tiling_benchmark/probe_phantom_hef.py`:
- `HefHandle.open(hef_path, nms_score_threshold=0.05) -> HefHandle` (owns one VDevice).
- `HefHandle.infer(image_uint8) -> list[np.ndarray]` — input must be exactly `(480,640,3)` uint8; returns NMS-by-class list, each row `(y_min, x_min, y_max, x_max, score)` normalized in the crop.
- `decode_nms_output(nms_out) -> list[Detection]` where `Detection(cls, x, y, w, h, score)` is normalized in the crop (`x=x_min, y=y_min, w=x_max-x_min, h=y_max-y_min`).

From `drone_follow/pipeline_adapter/`:
- `tracker_factory.create_tracker("byte", track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30) -> ByteTrackerAdapter`.
- `tracker.TrackedObject(track_id, input_index, is_activated, score, filtered_tlwh)` — `filtered_tlwh` is normalized `(x, y, w, h)`.

## File Structure

```
dynamic_tiling/
  __init__.py
  _vendor_paths.py     # add tiling_benchmark + repo root to sys.path for imports
  types.py             # CropRect, Det, LockState, MODEL_* constants
  budget.py            # BudgetMeter
  aggregator.py        # crop-local dets -> source px, cross-tile NMS
  target_lock.py       # TargetLock over the pipeline ByteTracker
  inference.py         # InferenceBackend protocol, ReplayBackend, HefBackend
  scheduler.py         # TileScheduler (levers 1-5)
  gt_track.py          # build single-target GT trajectory from GT frames.json
  score.py             # per-frame target recall/IoU + compare to a baseline run
  replay.py            # offline harness: video -> scheduler -> infer -> track -> score
  run_dynamic.py       # CLI driver
  tests/
    __init__.py
    test_budget.py
    test_aggregator.py
    test_target_lock.py
    test_scheduler.py
    test_gt_track.py
    test_score.py
    test_replay_integration.py
```

Tests run from the repo root with the venv: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/ -v` (after `source setup_env.sh`).

---

## Task 1: Package scaffold + shared types

**Files:**
- Create: `dynamic_tiling/__init__.py`
- Create: `dynamic_tiling/_vendor_paths.py`
- Create: `dynamic_tiling/types.py`
- Create: `dynamic_tiling/tests/__init__.py`
- Test: `dynamic_tiling/tests/test_types.py`

- [ ] **Step 1: Write the failing test**

```python
# dynamic_tiling/tests/test_types.py
from dynamic_tiling.types import CropRect, Det, LockState, MODEL_W, MODEL_H, MODEL_ASPECT


def test_model_constants():
    assert (MODEL_W, MODEL_H) == (640, 480)
    assert abs(MODEL_ASPECT - 640 / 480) < 1e-9


def test_croprect_scale_and_aspect():
    # crop_w 640 -> native (scale 1.0); 320 -> x2 zoom
    assert CropRect.from_center_width(cx=1000, cy=800, crop_w=640).scale == 1.0
    assert CropRect.from_center_width(cx=1000, cy=800, crop_w=320).scale == 2.0
    r = CropRect.from_center_width(cx=1000, cy=800, crop_w=640)
    assert r.h == round(640 / MODEL_ASPECT)  # 480


def test_croprect_clamp_keeps_inside_bounds():
    r = CropRect.from_center_width(cx=10, cy=10, crop_w=640).clamp(src_w=4000, src_h=3000)
    assert r.x >= 0 and r.y >= 0
    assert r.x + r.w <= 4000 and r.y + r.h <= 3000


def test_lockstate_defaults():
    s = LockState()
    assert s.track_id is None and s.status == "LOST" and s.frames_since_seen == 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_types.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dynamic_tiling'`

- [ ] **Step 3: Write the implementation**

```python
# dynamic_tiling/__init__.py
"""Dynamic, budget-aware tile scheduler (offline harness). See
docs/superpowers/specs/2026-05-27-dynamic-tiling-design.md."""
```

```python
# dynamic_tiling/_vendor_paths.py
"""Make tiling_benchmark and the repo root importable from this package.

tiling_benchmark/ holds loose scripts (HefHandle in probe_phantom_hef.py) that
are not a package; the drone_follow package lives at the repo root."""
from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
_TILING = _REPO_ROOT / "tiling_benchmark"

for p in (str(_REPO_ROOT), str(_TILING)):
    if p not in sys.path:
        sys.path.insert(0, p)
```

```python
# dynamic_tiling/__init__.py  (append the import so paths are set on first use)
from . import _vendor_paths  # noqa: F401,E402
```

```python
# dynamic_tiling/types.py
from __future__ import annotations

from dataclasses import dataclass, field

MODEL_W = 640
MODEL_H = 480
MODEL_ASPECT = MODEL_W / MODEL_H  # 4:3


@dataclass(frozen=True)
class CropRect:
    """A source-pixel crop fed to the cropper/HEF. 4:3, height derived from w."""
    x: int
    y: int
    w: int
    h: int
    mode: str = "s"  # cropper per-tile mode tag ("s" single-scale, "m" multi)

    @property
    def scale(self) -> float:
        return MODEL_W / self.w

    @classmethod
    def from_center_width(cls, cx: float, cy: float, crop_w: int,
                          mode: str = "s") -> "CropRect":
        w = int(round(crop_w))
        h = int(round(w / MODEL_ASPECT))
        return cls(x=int(round(cx - w / 2)), y=int(round(cy - h / 2)),
                   w=w, h=h, mode=mode)

    def clamp(self, src_w: int, src_h: int) -> "CropRect":
        w = min(self.w, src_w)
        h = min(self.h, src_h)
        x = max(0, min(src_w - w, self.x))
        y = max(0, min(src_h - h, self.y))
        return CropRect(x=x, y=y, w=w, h=h, mode=self.mode)


@dataclass(frozen=True)
class Det:
    """A detection in NORMALIZED source-frame coords. bbox = (x, y, w, h)."""
    cls: int
    score: float
    x: float
    y: float
    w: float
    h: float

    @property
    def xyxy(self) -> tuple:
        return (self.x, self.y, self.x + self.w, self.y + self.h)


@dataclass
class LockState:
    track_id: int | None = None
    bbox_norm: tuple = (0.0, 0.0, 0.0, 0.0)  # (x,y,w,h) normalized
    status: str = "LOST"                     # TRACKING | SEARCHING | LOST
    frames_since_seen: int = 0
    last_velocity: tuple = (0.0, 0.0)        # (dvx, dvy) of bbox centre, normalized/frame
```

```python
# dynamic_tiling/tests/__init__.py
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_types.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/__init__.py dynamic_tiling/_vendor_paths.py dynamic_tiling/types.py dynamic_tiling/tests/__init__.py dynamic_tiling/tests/test_types.py
git commit -m "dynamic_tiling: package scaffold + shared types"
```

---

## Task 2: BudgetMeter

**Files:**
- Create: `dynamic_tiling/budget.py`
- Test: `dynamic_tiling/tests/test_budget.py`

- [ ] **Step 1: Write the failing test**

```python
# dynamic_tiling/tests/test_budget.py
from dynamic_tiling.budget import BudgetMeter


def test_steady_budget_allows_average_tiles_per_frame():
    # 300 inf/s at 30 fps -> 10 tiles/frame average.
    m = BudgetMeter(budget_inf_per_s=300, fps=30, window_s=1.0)
    assert round(m.available(frame_idx=0)) == 10


def test_burst_then_throttle_keeps_window_average():
    m = BudgetMeter(budget_inf_per_s=300, fps=30, window_s=1.0)
    # Spend a big recovery burst on frame 0.
    m.charge(40, frame_idx=0)
    # Next frame the window already holds 40; remaining over 30 frames is tight.
    avail_after_burst = m.available(frame_idx=1)
    assert avail_after_burst < 10  # throttled below steady average
    # After the burst ages out of the 1 s window, steady budget returns.
    for f in range(1, 31):
        m.charge(0, frame_idx=f)
    assert round(m.available(frame_idx=31)) == 10


def test_available_never_negative():
    m = BudgetMeter(budget_inf_per_s=30, fps=30, window_s=1.0)  # 1 tile/frame
    m.charge(100, frame_idx=0)
    assert m.available(frame_idx=1) >= 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_budget.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dynamic_tiling.budget'`

- [ ] **Step 3: Write the implementation**

```python
# dynamic_tiling/budget.py
from __future__ import annotations

from collections import deque


class BudgetMeter:
    """Sliding-window tile-inference accounting.

    Enforces an average spend of `budget_inf_per_s` tile-inferences per second
    over a trailing `window_s` window, while allowing short bursts (e.g. a
    recovery spike) as long as the windowed total stays under the cap.
    """

    def __init__(self, budget_inf_per_s: float, fps: float, window_s: float = 1.0):
        self.budget_inf_per_s = float(budget_inf_per_s)
        self.fps = float(fps)
        self.window_frames = max(1, int(round(window_s * fps)))
        self.window_cap = self.budget_inf_per_s * window_s
        self._spent: deque[tuple[int, int]] = deque()  # (frame_idx, n_tiles)

    def _evict(self, frame_idx: int) -> None:
        lo = frame_idx - self.window_frames
        while self._spent and self._spent[0][0] <= lo:
            self._spent.popleft()

    def _window_total(self, frame_idx: int) -> int:
        self._evict(frame_idx)
        return sum(n for _, n in self._spent)

    def charge(self, n_tiles: int, frame_idx: int) -> None:
        self._evict(frame_idx)
        if n_tiles > 0:
            self._spent.append((frame_idx, int(n_tiles)))

    def available(self, frame_idx: int) -> float:
        """Tiles that may be spent on `frame_idx` to keep the window <= cap."""
        return max(0.0, self.window_cap - self._window_total(frame_idx))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_budget.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/budget.py dynamic_tiling/tests/test_budget.py
git commit -m "dynamic_tiling: sliding-window BudgetMeter"
```

---

## Task 3: Aggregator (crop-local dets -> source px + cross-tile NMS)

**Files:**
- Create: `dynamic_tiling/aggregator.py`
- Test: `dynamic_tiling/tests/test_aggregator.py`

`HefHandle.infer` + `decode_nms_output` give crop-local normalized `Detection(cls,x,y,w,h,score)`. We map each into the full frame and dedup overlapping detections from different crops.

Mapping (crop is `CropRect` in src px, frame is `src_W × src_H`):
```
src_x = crop.x + det.x * crop.w
src_y = crop.y + det.y * crop.h
src_bw = det.w * crop.w
src_bh = det.h * crop.h
# normalized:
nx = src_x / src_W ; ny = src_y / src_H ; nw = src_bw / src_W ; nh = src_bh / src_H
```

- [ ] **Step 1: Write the failing test**

```python
# dynamic_tiling/tests/test_aggregator.py
from dynamic_tiling.types import CropRect, Det
from dynamic_tiling.aggregator import map_to_source, nms


class _CropLocalDet:
    """Mimics probe_phantom_hef.Detection (normalized in crop)."""
    def __init__(self, cls, x, y, w, h, score):
        self.cls, self.x, self.y, self.w, self.h, self.score = cls, x, y, w, h, score


def test_map_to_source_places_box_correctly():
    crop = CropRect(x=1000, y=500, w=640, h=480)
    # Detection centred in the crop, 50% wide/tall.
    d = _CropLocalDet(cls=0, x=0.25, y=0.25, w=0.5, h=0.5, score=0.9)
    out = map_to_source([d], crop, src_w=4000, src_h=3000)
    assert len(out) == 1
    o = out[0]
    # src_x = 1000 + 0.25*640 = 1160 -> /4000
    assert abs(o.x - 1160 / 4000) < 1e-6
    assert abs(o.y - (500 + 0.25 * 480) / 3000) < 1e-6
    assert abs(o.w - (0.5 * 640) / 4000) < 1e-6


def test_nms_merges_overlapping_same_class():
    a = Det(cls=0, score=0.9, x=0.10, y=0.10, w=0.10, h=0.20)
    b = Det(cls=0, score=0.7, x=0.105, y=0.105, w=0.10, h=0.20)  # near-duplicate
    c = Det(cls=0, score=0.8, x=0.60, y=0.60, w=0.10, h=0.20)    # far apart
    kept = nms([a, b, c], iou_thr=0.5)
    assert len(kept) == 2
    assert any(abs(k.x - 0.10) < 1e-6 for k in kept)   # higher-score a survives
    assert any(abs(k.x - 0.60) < 1e-6 for k in kept)


def test_nms_keeps_different_classes():
    a = Det(cls=0, score=0.9, x=0.10, y=0.10, w=0.10, h=0.20)
    b = Det(cls=1, score=0.7, x=0.10, y=0.10, w=0.10, h=0.20)  # same box, other class
    assert len(nms([a, b], iou_thr=0.5)) == 2
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_aggregator.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dynamic_tiling.aggregator'`

- [ ] **Step 3: Write the implementation**

```python
# dynamic_tiling/aggregator.py
from __future__ import annotations

from typing import Sequence

from .types import CropRect, Det


def map_to_source(crop_dets, crop: CropRect, src_w: int, src_h: int) -> list[Det]:
    """Map crop-local normalized detections into normalized full-frame Dets.

    `crop_dets` items expose .cls .x .y .w .h .score normalized within the crop
    (e.g. probe_phantom_hef.Detection)."""
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

- [ ] **Step 4: Run test to verify it passes**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_aggregator.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/aggregator.py dynamic_tiling/tests/test_aggregator.py
git commit -m "dynamic_tiling: aggregator (crop->source mapping + cross-tile NMS)"
```

---

## Task 4: TargetLock over the pipeline ByteTracker

**Files:**
- Create: `dynamic_tiling/target_lock.py`
- Test: `dynamic_tiling/tests/test_target_lock.py`

`TargetLock` builds the real ByteTracker via `create_tracker`, feeds it a `(N,5)` person-detection array (SCALE=1000, xyxy+score), and follows one `track_id`. It derives `LockState` (TRACKING/SEARCHING/LOST) from track presence and `track_buffer`.

- [ ] **Step 1: Write the failing test**

```python
# dynamic_tiling/tests/test_target_lock.py
import numpy as np
import dynamic_tiling  # noqa: F401  (sets sys.path for drone_follow import)
from dynamic_tiling.types import Det
from dynamic_tiling.target_lock import TargetLock, dets_to_array


def _person(x, y, w=0.08, h=0.20, score=0.9):
    return Det(cls=0, score=score, x=x, y=y, w=w, h=h)


def test_dets_to_array_scale_and_columns():
    arr = dets_to_array([_person(0.10, 0.20)])
    assert arr.shape == (1, 5)
    # xmin,ymin,xmax,ymax in 1000-scale; score raw.
    assert abs(arr[0, 0] - 100.0) < 1e-3      # 0.10 * 1000
    assert abs(arr[0, 2] - (0.10 + 0.08) * 1000) < 1e-3
    assert abs(arr[0, 4] - 0.9) < 1e-6


def test_lock_holds_track_id_across_constant_velocity():
    lock = TargetLock()
    # Frame 0: a single moving person; lock onto it.
    s0 = lock.step([_person(0.40, 0.40)], lock_if_unlocked=True)
    assert s0.status == "TRACKING"
    tid = lock.track_id
    assert tid is not None
    # Frames 1..5: same person drifting right -> same track id, TRACKING.
    for k in range(1, 6):
        s = lock.step([_person(0.40 + 0.01 * k, 0.40)])
        assert s.status == "TRACKING"
        assert lock.track_id == tid
    assert lock.state.last_velocity[0] > 0  # moving right


def test_lock_enters_searching_then_recovers():
    lock = TargetLock()
    lock.step([_person(0.40, 0.40)], lock_if_unlocked=True)
    tid = lock.track_id
    # A few empty frames -> SEARCHING (within track_buffer), id retained.
    for _ in range(3):
        s = lock.step([])
        assert s.status in ("SEARCHING", "LOST")
    assert lock.track_id == tid
    # Person reappears near last position -> re-acquired as same track.
    s = lock.step([_person(0.43, 0.40)])
    assert s.status == "TRACKING"
    assert lock.track_id == tid
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_target_lock.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dynamic_tiling.target_lock'`

- [ ] **Step 3: Write the implementation**

```python
# dynamic_tiling/target_lock.py
from __future__ import annotations

from typing import Sequence

import numpy as np

import dynamic_tiling  # noqa: F401  ensures _vendor_paths ran
from drone_follow.pipeline_adapter.tracker_factory import create_tracker

from .types import Det, LockState

_SCALE = 1000.0


def dets_to_array(person_dets: Sequence[Det]) -> np.ndarray:
    """Build the (N,5) [xmin,ymin,xmax,ymax,score] SCALE=1000 array ByteTracker
    expects (mirrors hailo_drone_detection_manager.py)."""
    arr = np.empty((len(person_dets), 5), dtype=np.float32)
    for i, d in enumerate(person_dets):
        arr[i, 0] = d.x * _SCALE
        arr[i, 1] = d.y * _SCALE
        arr[i, 2] = (d.x + d.w) * _SCALE
        arr[i, 3] = (d.y + d.h) * _SCALE
        arr[i, 4] = d.score
    return arr


def _iou_tlwh(a: tuple, b: tuple) -> float:
    ax1, ay1 = a[0], a[1]; ax2, ay2 = a[0] + a[2], a[1] + a[3]
    bx1, by1 = b[0], b[1]; bx2, by2 = b[0] + b[2], b[1] + b[3]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = a[2] * a[3] + b[2] * b[3] - inter
    return inter / ua if ua > 0 else 0.0


class TargetLock:
    """Follows one ByteTracker track_id; derives a LockState each frame."""

    def __init__(self, track_buffer: int = 90, **tracker_kwargs):
        self._tracker = create_tracker("byte", track_buffer=track_buffer,
                                       **tracker_kwargs)
        self.track_buffer = track_buffer
        self.track_id: int | None = None
        self.state = LockState()

    def lock_from_gt(self, gt_bbox_norm: tuple, tracks) -> None:
        """Pick the activated track best matching a GT bbox (x,y,w,h)."""
        best, best_iou = None, 0.0
        for t in tracks:
            if not t.is_activated or not t.filtered_tlwh:
                continue
            i = _iou_tlwh(gt_bbox_norm, t.filtered_tlwh)
            if i > best_iou:
                best, best_iou = t, i
        if best is not None and best_iou > 0.3:
            self.track_id = best.track_id

    def step(self, person_dets: Sequence[Det], *, lock_if_unlocked: bool = False,
             gt_bbox_norm: tuple | None = None) -> LockState:
        tracks = self._tracker.update(dets_to_array(person_dets))

        if self.track_id is None:
            if gt_bbox_norm is not None:
                self.lock_from_gt(gt_bbox_norm, tracks)
            elif lock_if_unlocked:
                # Lock the largest activated track.
                acts = [t for t in tracks if t.is_activated and t.filtered_tlwh]
                if acts:
                    big = max(acts, key=lambda t: t.filtered_tlwh[2] * t.filtered_tlwh[3])
                    self.track_id = big.track_id

        cur = next((t for t in tracks
                    if t.track_id == self.track_id and t.is_activated
                    and t.filtered_tlwh), None)

        s = self.state
        if cur is not None:
            cx_old = s.bbox_norm[0] + s.bbox_norm[2] / 2 if s.frames_since_seen == 0 else None
            cy_old = s.bbox_norm[1] + s.bbox_norm[3] / 2 if s.frames_since_seen == 0 else None
            s.bbox_norm = tuple(cur.filtered_tlwh)
            cx_new = s.bbox_norm[0] + s.bbox_norm[2] / 2
            cy_new = s.bbox_norm[1] + s.bbox_norm[3] / 2
            if cx_old is not None:
                s.last_velocity = (cx_new - cx_old, cy_new - cy_old)
            s.status = "TRACKING"
            s.frames_since_seen = 0
        else:
            s.frames_since_seen += 1
            s.status = "SEARCHING" if s.frames_since_seen <= self.track_buffer else "LOST"
        s.track_id = self.track_id
        return s
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_target_lock.py -v`
Expected: PASS (3 passed). If `create_tracker` import fails, confirm `source setup_env.sh` was run so `drone_follow` is importable.

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/target_lock.py dynamic_tiling/tests/test_target_lock.py
git commit -m "dynamic_tiling: TargetLock over pipeline ByteTracker"
```

---

## Task 5: Inference backends (Replay + Hef)

**Files:**
- Create: `dynamic_tiling/inference.py`
- Test: `dynamic_tiling/tests/test_inference.py`

`ReplayBackend` returns canned detections so scheduler/replay tests run without the chip. `HefBackend` wraps `HefHandle` (crops the frame, resizes to 640×480, BGR→RGB, infers, decodes). Only `ReplayBackend` is unit-tested; `HefBackend` is exercised in the manual real-chip run (Task 10).

- [ ] **Step 1: Write the failing test**

```python
# dynamic_tiling/tests/test_inference.py
import numpy as np
from dynamic_tiling.types import CropRect
from dynamic_tiling.inference import ReplayBackend


def test_replay_backend_returns_canned_crop_local_dets():
    # Keyed by (frame_idx, crop signature). Returns crop-local normalized dets.
    canned = {(0, (1000, 500, 640, 480)): [
        type("D", (), dict(cls=0, x=0.25, y=0.25, w=0.5, h=0.5, score=0.9))()]}
    be = ReplayBackend(canned)
    crop = CropRect(x=1000, y=500, w=640, h=480)
    dets = be.infer(frame=np.zeros((3000, 4000, 3), np.uint8), crop=crop, frame_idx=0)
    assert len(dets) == 1 and dets[0].cls == 0


def test_replay_backend_empty_for_unknown_crop():
    be = ReplayBackend({})
    crop = CropRect(x=0, y=0, w=640, h=480)
    assert be.infer(frame=np.zeros((480, 640, 3), np.uint8), crop=crop, frame_idx=5) == []
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_inference.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dynamic_tiling.inference'`

- [ ] **Step 3: Write the implementation**

```python
# dynamic_tiling/inference.py
from __future__ import annotations

from typing import Protocol

import cv2
import numpy as np

from .types import CropRect, MODEL_W, MODEL_H


class InferenceBackend(Protocol):
    def infer(self, frame: np.ndarray, crop: CropRect, frame_idx: int) -> list:
        """Return crop-local normalized detections (.cls .x .y .w .h .score)."""
        ...


class ReplayBackend:
    """Deterministic backend: returns canned crop-local dets for tests.

    `canned` maps (frame_idx, (x,y,w,h)) -> list of objects exposing
    .cls .x .y .w .h .score (crop-local normalized)."""

    def __init__(self, canned: dict):
        self._canned = canned

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:
        return list(self._canned.get((frame_idx, (crop.x, crop.y, crop.w, crop.h)), []))


class HefBackend:
    """Real on-chip backend wrapping tiling_benchmark HefHandle."""

    def __init__(self, hef_path: str, nms_score_threshold: float = 0.25):
        from probe_phantom_hef import HefHandle  # via _vendor_paths
        self._HefHandle = HefHandle
        self._handle = HefHandle.open(hef_path, nms_score_threshold=nms_score_threshold)

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:
        from probe_phantom_hef import decode_nms_output
        sub = frame[crop.y:crop.y + crop.h, crop.x:crop.x + crop.w]
        resized = cv2.resize(sub, (MODEL_W, MODEL_H), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        return decode_nms_output(self._handle.infer(rgb))

    def close(self) -> None:
        self._handle.close()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_inference.py -v`
Expected: PASS (2 passed)

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/inference.py dynamic_tiling/tests/test_inference.py
git commit -m "dynamic_tiling: inference backends (replay + hef)"
```

---

## Task 6: TileScheduler (levers 1-5)

**Files:**
- Create: `dynamic_tiling/scheduler.py`
- Test: `dynamic_tiling/tests/test_scheduler.py`

Levers: (1) decimated discovery grid only when `frame_idx % discovery_period == 0`; (2) ROI tile on the locked bbox; (3) adaptive zoom sizing clamped to scale ≤ 2.0; (4) placement uses the lock bbox (TRACKING) or last-known+velocity extrapolation (gap); (5) recovery grid around last-known position when SEARCHING/LOST, bounded by `meter.available`.

Adaptive ROI sizing (lever 3): pick `crop_w` so the target maps to `target_model_h` px tall, then clamp.
```
src_h_px   = bbox_h_norm * src_h
crop_w     = src_h_px * MODEL_H * MODEL_ASPECT / target_model_h   # so model height ~= target_model_h
crop_w     = clamp(crop_w, MODEL_W / max_zoom, MODEL_W)           # scale in [1.0, max_zoom]
crop_w     = max(crop_w, (bbox_w_norm*src_w + 2*margin_px))       # whole target must fit
```

- [ ] **Step 1: Write the failing test**

```python
# dynamic_tiling/tests/test_scheduler.py
from dynamic_tiling.types import LockState
from dynamic_tiling.budget import BudgetMeter
from dynamic_tiling.scheduler import TileScheduler


def _meter():
    return BudgetMeter(budget_inf_per_s=300, fps=30, window_s=1.0)


def test_discovery_grid_only_on_cadence():
    sched = TileScheduler(src_w=4000, src_h=3000, discovery_period=15,
                          discovery_grid=(3, 2))
    lock = LockState(track_id=7, bbox_norm=(0.4, 0.4, 0.08, 0.2), status="TRACKING")
    # Frame 0 -> discovery cadence hit: grid (6) + ROI (1).
    crops0 = sched.decide(lock, frame_idx=0, meter=_meter())
    assert len(crops0) == 3 * 2 + 1
    # Frame 1 -> off cadence: ROI only.
    crops1 = sched.decide(lock, frame_idx=1, meter=_meter())
    assert len(crops1) == 1


def test_roi_centered_on_target_and_zoom_clamped():
    sched = TileScheduler(src_w=4000, src_h=3000, discovery_period=15,
                          max_zoom=2.0, target_model_h=40)
    lock = LockState(track_id=7, bbox_norm=(0.50, 0.50, 0.05, 0.10), status="TRACKING")
    crops = sched.decide(lock, frame_idx=1, meter=_meter())
    roi = crops[0]
    assert 1.0 <= roi.scale <= 2.0 + 1e-6
    # Centre of ROI ~ centre of target (0.525, 0.55) in src px.
    cx = roi.x + roi.w / 2
    assert abs(cx - 0.525 * 4000) < roi.w  # within the crop


def test_recovery_grid_when_searching():
    sched = TileScheduler(src_w=4000, src_h=3000, discovery_period=15,
                          recovery_grid=(3, 3))
    lock = LockState(track_id=7, bbox_norm=(0.4, 0.4, 0.08, 0.2),
                     status="SEARCHING", frames_since_seen=2)
    crops = sched.decide(lock, frame_idx=1, meter=_meter())  # off discovery cadence
    assert len(crops) == 3 * 3  # local recovery grid, no ROI/discovery


def test_recovery_grid_throttled_by_budget():
    sched = TileScheduler(src_w=4000, src_h=3000, discovery_period=15,
                          recovery_grid=(3, 3))
    lock = LockState(track_id=7, bbox_norm=(0.4, 0.4, 0.08, 0.2),
                     status="SEARCHING", frames_since_seen=2)
    tight = BudgetMeter(budget_inf_per_s=120, fps=30, window_s=1.0)  # 4 tiles/frame
    tight.charge(4, frame_idx=0)  # window already full
    crops = sched.decide(lock, frame_idx=1, meter=tight)
    assert len(crops) <= 4
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_scheduler.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dynamic_tiling.scheduler'`

- [ ] **Step 3: Write the implementation**

```python
# dynamic_tiling/scheduler.py
from __future__ import annotations

from .types import CropRect, LockState, MODEL_W, MODEL_H, MODEL_ASPECT


class TileScheduler:
    def __init__(self, src_w: int, src_h: int, *,
                 discovery_period: int = 15, discovery_grid: tuple = (3, 2),
                 recovery_grid: tuple = (3, 3), max_zoom: float = 2.0,
                 target_model_h: float = 40.0, roi_margin_frac: float = 0.25):
        self.src_w = src_w
        self.src_h = src_h
        self.discovery_period = discovery_period
        self.discovery_grid = discovery_grid
        self.recovery_grid = recovery_grid
        self.max_zoom = max_zoom
        self.target_model_h = target_model_h
        self.roi_margin_frac = roi_margin_frac

    def _grid(self, gx: int, gy: int, x0: float, y0: float, w: float, h: float,
              mode: str) -> list[CropRect]:
        """gx*gy grid of CropRects covering the [x0,y0,w,h] src-px region."""
        out = []
        cw = w / gx
        ch = h / gy
        # Square-ish tiles resized to 4:3 model input; width drives the rect.
        for j in range(gy):
            for i in range(gx):
                cx = x0 + (i + 0.5) * cw
                cy = y0 + (j + 0.5) * ch
                r = CropRect.from_center_width(cx, cy, int(round(cw)), mode=mode)
                out.append(r.clamp(self.src_w, self.src_h))
        return out

    def _roi(self, lock: LockState) -> CropRect:
        bx, by, bw, bh = lock.bbox_norm
        # Lever 4: extrapolate placement during a gap.
        if lock.status != "TRACKING":
            bx += lock.last_velocity[0] * lock.frames_since_seen
            by += lock.last_velocity[1] * lock.frames_since_seen
        cx = (bx + bw / 2) * self.src_w
        cy = (by + bh / 2) * self.src_h
        # Lever 3: adaptive zoom sizing.
        src_h_px = max(1.0, bh * self.src_h)
        crop_w = src_h_px * MODEL_H * MODEL_ASPECT / self.target_model_h
        lo = MODEL_W / self.max_zoom            # smallest crop -> max zoom
        crop_w = max(lo, min(float(MODEL_W) * 4, crop_w))  # cap absurd sizes
        crop_w = min(crop_w, float(MODEL_W))    # never < scale 1.0 (no downscale ROI)
        # whole target must fit with margin
        need_w = bw * self.src_w * (1 + 2 * self.roi_margin_frac)
        crop_w = max(crop_w, need_w)
        return CropRect.from_center_width(cx, cy, int(round(crop_w))).clamp(
            self.src_w, self.src_h)

    def decide(self, lock: LockState, frame_idx: int, meter) -> list[CropRect]:
        crops: list[CropRect] = []
        on_cadence = (frame_idx % self.discovery_period == 0)

        if lock.status in ("SEARCHING", "LOST") and lock.track_id is not None:
            # Lever 5: local recovery grid around last-known position.
            gx, gy = self.recovery_grid
            bx, by, bw, bh = lock.bbox_norm
            span = 0.4  # 40% of frame around last-known centre
            x0 = max(0.0, (bx + bw / 2 - span / 2)) * self.src_w
            y0 = max(0.0, (by + bh / 2 - span / 2)) * self.src_h
            crops = self._grid(gx, gy, x0, y0, span * self.src_w, span * self.src_h, "s")
        else:
            if on_cadence:
                gx, gy = self.discovery_grid
                crops += self._grid(gx, gy, 0, 0, self.src_w, self.src_h, "m")
            if lock.status == "TRACKING":
                crops.append(self._roi(lock))

        # Budget throttle (lever shared): never exceed what's available.
        budget = int(meter.available(frame_idx))
        if budget >= 0 and len(crops) > budget:
            crops = crops[:max(0, budget)]
        return crops
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_scheduler.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/scheduler.py dynamic_tiling/tests/test_scheduler.py
git commit -m "dynamic_tiling: TileScheduler (decimated grid + adaptive ROI + recovery)"
```

---

## Task 7: GT target trajectory builder

**Files:**
- Create: `dynamic_tiling/gt_track.py`
- Test: `dynamic_tiling/tests/test_gt_track.py`

The GT `frames.json` is per-frame boxes without identities. To score a single target we stitch a trajectory by greedy IoU association across consecutive frames, then select one trajectory (largest person at the start frame, or a chosen anchor).

- [ ] **Step 1: Write the failing test**

```python
# dynamic_tiling/tests/test_gt_track.py
from dynamic_tiling.gt_track import build_target_trajectory


def _doc():
    # Two persons; person A drifts right, person B static. A is larger.
    frames = []
    for f in range(5):
        frames.append({"frame": f, "detections": [
            {"label": "person", "confidence": 0.9,
             "bbox": [0.40 + 0.02 * f, 0.40, 0.10, 0.25]},   # A (large, moving)
            {"label": "person", "confidence": 0.9,
             "bbox": [0.10, 0.10, 0.05, 0.12]},              # B (small, static)
        ]})
    return {"frames": frames}


def test_builds_trajectory_for_largest_person():
    traj = build_target_trajectory(_doc(), label="person", anchor="largest")
    assert set(traj.keys()) == {0, 1, 2, 3, 4}
    # Each frame's bbox should be the large/moving person A.
    assert abs(traj[0][0] - 0.40) < 1e-6
    assert abs(traj[4][0] - 0.48) < 1e-6  # moved right


def test_handles_missing_frame_gap():
    doc = _doc()
    doc["frames"][2]["detections"] = []  # gap on frame 2
    traj = build_target_trajectory(doc, label="person", anchor="largest")
    assert 2 not in traj            # no GT box on frame 2
    assert traj[3][0] > traj[1][0]  # re-associates A after the gap
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_gt_track.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dynamic_tiling.gt_track'`

- [ ] **Step 3: Write the implementation**

```python
# dynamic_tiling/gt_track.py
from __future__ import annotations


def _iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = a[0], a[1], a[0] + a[2], a[1] + a[3]
    bx1, by1, bx2, by2 = b[0], b[1], b[0] + b[2], b[1] + b[3]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = a[2] * a[3] + b[2] * b[3] - inter
    return inter / ua if ua > 0 else 0.0


def _centre_dist(a, b) -> float:
    acx, acy = a[0] + a[2] / 2, a[1] + a[3] / 2
    bcx, bcy = b[0] + b[2] / 2, b[1] + b[3] / 2
    return ((acx - bcx) ** 2 + (acy - bcy) ** 2) ** 0.5


def build_target_trajectory(doc: dict, label: str = "person",
                            anchor: str = "largest",
                            min_iou: float = 0.1,
                            max_centre_jump: float = 0.15) -> dict:
    """Return {frame_idx: (x,y,w,h)} for ONE target stitched by greedy
    association. Picks the `anchor` ("largest") box on the first frame that has
    one, then follows it; on a gap, re-associates by nearest centre."""
    frames = sorted(doc.get("frames", []), key=lambda fr: fr["frame"])

    def boxes(fr):
        return [tuple(d["bbox"]) for d in fr.get("detections", [])
                if d.get("label") == label]

    # Seed.
    cur = None
    out: dict = {}
    for fr in frames:
        bs = boxes(fr)
        if not bs:
            continue
        if cur is None:
            cur = max(bs, key=lambda b: b[2] * b[3]) if anchor == "largest" else bs[0]
            out[fr["frame"]] = cur
            continue
        # Associate: best IoU, else nearest centre within jump bound.
        best = max(bs, key=lambda b: _iou(cur, b))
        if _iou(cur, best) < min_iou:
            best = min(bs, key=lambda b: _centre_dist(cur, b))
            if _centre_dist(cur, best) > max_centre_jump:
                continue  # no plausible continuation this frame
        out[fr["frame"]] = best
        cur = best
    return out
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_gt_track.py -v`
Expected: PASS (2 passed)

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/gt_track.py dynamic_tiling/tests/test_gt_track.py
git commit -m "dynamic_tiling: GT single-target trajectory builder"
```

---

## Task 8: Scoring

**Files:**
- Create: `dynamic_tiling/score.py`
- Test: `dynamic_tiling/tests/test_score.py`

Per frame where the GT target exists: detected = (best lock IoU ≥ 0.5). Aggregate recall and mean IoU over matched frames. `compare()` returns a small dict for dynamic vs a baseline.

- [ ] **Step 1: Write the failing test**

```python
# dynamic_tiling/tests/test_score.py
from dynamic_tiling.score import score_run, RunScore


def test_score_run_recall_and_iou():
    gt = {0: (0.40, 0.40, 0.10, 0.25), 1: (0.42, 0.40, 0.10, 0.25),
          2: (0.44, 0.40, 0.10, 0.25)}
    # Predicted lock bbox per frame (None == target not held this frame).
    pred = {0: (0.40, 0.40, 0.10, 0.25),   # perfect
            1: (0.42, 0.41, 0.10, 0.25),   # good overlap
            2: None}                       # missed
    s = score_run(gt, pred, iou_thr=0.5)
    assert isinstance(s, RunScore)
    assert s.n_gt_frames == 3
    assert s.n_hit == 2
    assert abs(s.recall - 2 / 3) < 1e-9
    assert s.mean_iou > 0.8  # over the two hit frames


def test_score_run_empty_gt():
    s = score_run({}, {}, iou_thr=0.5)
    assert s.n_gt_frames == 0 and s.recall == 0.0 and s.mean_iou == 0.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_score.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dynamic_tiling.score'`

- [ ] **Step 3: Write the implementation**

```python
# dynamic_tiling/score.py
from __future__ import annotations

from dataclasses import dataclass


def _iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = a[0], a[1], a[0] + a[2], a[1] + a[3]
    bx1, by1, bx2, by2 = b[0], b[1], b[0] + b[2], b[1] + b[3]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = a[2] * a[3] + b[2] * b[3] - inter
    return inter / ua if ua > 0 else 0.0


@dataclass
class RunScore:
    n_gt_frames: int
    n_hit: int
    recall: float
    mean_iou: float
    per_frame_iou: dict  # frame_idx -> iou (0.0 if missed)


def score_run(gt_traj: dict, pred_traj: dict, iou_thr: float = 0.5) -> RunScore:
    """gt_traj/pred_traj: {frame_idx: (x,y,w,h) normalized} (pred may have None)."""
    per_frame = {}
    n_hit = 0
    iou_sum = 0.0
    for f, gt_box in gt_traj.items():
        pred_box = pred_traj.get(f)
        iou = _iou(gt_box, pred_box) if pred_box else 0.0
        per_frame[f] = iou
        if iou >= iou_thr:
            n_hit += 1
            iou_sum += iou
    n = len(gt_traj)
    return RunScore(
        n_gt_frames=n, n_hit=n_hit,
        recall=(n_hit / n) if n else 0.0,
        mean_iou=(iou_sum / n_hit) if n_hit else 0.0,
        per_frame_iou=per_frame,
    )


def compare(name_a: str, a: RunScore, name_b: str, b: RunScore) -> dict:
    return {
        name_a: dict(recall=a.recall, mean_iou=a.mean_iou, hits=a.n_hit),
        name_b: dict(recall=b.recall, mean_iou=b.mean_iou, hits=b.n_hit),
    }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_score.py -v`
Expected: PASS (2 passed)

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/score.py dynamic_tiling/tests/test_score.py
git commit -m "dynamic_tiling: per-target recall/IoU scoring"
```

---

## Task 9: Replay harness (integration)

**Files:**
- Create: `dynamic_tiling/replay.py`
- Test: `dynamic_tiling/tests/test_replay_integration.py`

`run()` drives the full loop on a frame iterator (so tests inject synthetic frames; `run_dynamic.py` feeds real video). It returns the predicted target trajectory + the achieved average tiles/frame, and can emit a viewer-compatible `frames.json`.

- [ ] **Step 1: Write the failing test**

```python
# dynamic_tiling/tests/test_replay_integration.py
import numpy as np
from dynamic_tiling.types import CropRect
from dynamic_tiling.budget import BudgetMeter
from dynamic_tiling.scheduler import TileScheduler
from dynamic_tiling.target_lock import TargetLock
from dynamic_tiling.inference import ReplayBackend
from dynamic_tiling.replay import run


def test_replay_tracks_target_with_replay_backend():
    src_w, src_h = 4000, 3000
    n_frames = 6
    # GT: one person drifting right.
    gt_traj = {f: (0.40 + 0.01 * f, 0.40, 0.08, 0.20) for f in range(n_frames)}

    # Canned crop-local dets: whatever crop the scheduler asks for, return the
    # target re-centred in that crop (so mapping reproduces ~the GT box).
    class _AnyCropBackend:
        def infer(self, frame, crop: CropRect, frame_idx):
            gx, gy, gw, gh = gt_traj[frame_idx]
            # GT box centre in src px:
            gcx = (gx + gw / 2) * src_w
            gcy = (gy + gh / 2) * src_h
            # Only "detect" if the target centre falls inside this crop.
            if not (crop.x <= gcx <= crop.x + crop.w and
                    crop.y <= gcy <= crop.y + crop.h):
                return []
            lx = (gcx - crop.x) / crop.w
            ly = (gcy - crop.y) / crop.h
            lw = gw * src_w / crop.w
            lh = gh * src_h / crop.h
            return [type("D", (), dict(cls=0, x=lx - lw / 2, y=ly - lh / 2,
                                       w=lw, h=lh, score=0.9))()]

    frames = [np.zeros((src_h, src_w, 3), np.uint8) for _ in range(n_frames)]
    result = run(
        frames=frames, src_w=src_w, src_h=src_h,
        scheduler=TileScheduler(src_w, src_h, discovery_period=2),
        lock=TargetLock(),
        backend=_AnyCropBackend(),
        meter=BudgetMeter(budget_inf_per_s=300, fps=30),
        gt_traj=gt_traj,
    )
    # Target is held on most frames and average tiles/frame is well under budget.
    assert result.pred_traj.get(n_frames - 1) is not None
    assert result.avg_tiles_per_frame < 10
    assert sum(1 for f in range(n_frames) if result.pred_traj.get(f)) >= n_frames - 2
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_replay_integration.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dynamic_tiling.replay'`

- [ ] **Step 3: Write the implementation**

```python
# dynamic_tiling/replay.py
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from .aggregator import map_to_source, nms
from .scheduler import TileScheduler
from .target_lock import TargetLock


@dataclass
class RunResult:
    pred_traj: dict = field(default_factory=dict)   # frame_idx -> (x,y,w,h) | absent
    frame_dets: dict = field(default_factory=dict)  # frame_idx -> list[Det]
    total_tiles: int = 0
    n_frames: int = 0

    @property
    def avg_tiles_per_frame(self) -> float:
        return self.total_tiles / self.n_frames if self.n_frames else 0.0


def run(frames, src_w: int, src_h: int, scheduler: TileScheduler,
        lock: TargetLock, backend, meter, gt_traj: dict,
        person_cls: int = 0) -> RunResult:
    res = RunResult()
    for frame_idx, frame in enumerate(frames):
        # Decide crops from the current lock state.
        crops = scheduler.decide(lock.state, frame_idx, meter)
        meter.charge(len(crops), frame_idx)
        res.total_tiles += len(crops)

        # Real (or replay) inference per crop, mapped to source + merged.
        dets = []
        for crop in crops:
            local = backend.infer(frame, crop, frame_idx)
            dets += map_to_source(local, crop, src_w, src_h)
        dets = nms(dets, iou_thr=0.5)
        res.frame_dets[frame_idx] = dets

        # Feed persons to the tracker; lock on the first GT-present frame.
        persons = [d for d in dets if d.cls == person_cls]
        gt_box = gt_traj.get(frame_idx)
        if lock.track_id is None and gt_box is not None:
            state = lock.step(persons, gt_bbox_norm=gt_box)
        else:
            state = lock.step(persons)

        if state.status == "TRACKING":
            res.pred_traj[frame_idx] = tuple(state.bbox_norm)
    res.n_frames = frame_idx + 1 if frames else 0
    return res


def emit_frames_json(res: RunResult, label: str, out_path: Path,
                     class_labels=("person", "vehicle", "face", "license_plate")) -> None:
    """Write a frames.json the existing overlay_viewer can load."""
    frames = []
    for f, dets in sorted(res.frame_dets.items()):
        frames.append({"frame": f, "detections": [
            {"label": class_labels[d.cls] if d.cls < len(class_labels) else str(d.cls),
             "confidence": d.score, "bbox": [d.x, d.y, d.w, d.h]} for d in dets]})
    out_path.write_text(json.dumps({"label": label, "frames": frames}))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_replay_integration.py -v`
Expected: PASS (1 passed)

- [ ] **Step 5: Run the whole suite**

Run: `./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/ -v`
Expected: PASS (all tasks' tests green)

- [ ] **Step 6: Commit**

```bash
git add dynamic_tiling/replay.py dynamic_tiling/tests/test_replay_integration.py
git commit -m "dynamic_tiling: offline replay harness + viewer-compatible output"
```

---

## Task 10: CLI driver + real-chip validation

**Files:**
- Create: `dynamic_tiling/run_dynamic.py`
- Create: `dynamic_tiling/run_dynamic.sh` (launcher, like `tiling_benchmark/run_viewer.sh`)

This task wires real video + real chip. There is no unit test (it needs the HEF + video); validate by running it and inspecting the printed table + emitted frames.json in the viewer.

- [ ] **Step 1: Write the CLI driver**

```python
# dynamic_tiling/run_dynamic.py
"""Run the dynamic tile scheduler over a recorded video with real inference,
score the locked target vs the GT, and emit a viewer-compatible frames.json.

Example:
    source setup_env.sh
    python -m dynamic_tiling.run_dynamic \
        --video /home/giladn/Videos/Drone/Training/DJI_20260430103421_0010_D_rotated.MP4 \
        --gt tiling_benchmark/pxt_runs/pxt_GT-12x9-25-multi.frames.json \
        --hef /usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef \
        --budget 300 --fps 30 --discovery-fps 2 \
        --out dynamic_tiling/runs/dynamic_run.frames.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2

import dynamic_tiling  # noqa: F401
from .budget import BudgetMeter
from .scheduler import TileScheduler
from .target_lock import TargetLock
from .inference import HefBackend
from .gt_track import build_target_trajectory
from .score import score_run
from .replay import run, emit_frames_json


def _frame_iter(cap, max_frames):
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok or (max_frames and n >= max_frames):
            break
        yield frame
        n += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--gt", required=True, type=Path)
    ap.add_argument("--hef",
                    default="/usr/local/hailo/resources/models/hailo10h/"
                            "hailo_yolov8n_4_classes_vga.hef")
    ap.add_argument("--budget", type=float, default=300.0)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--discovery-fps", type=float, default=2.0)
    ap.add_argument("--max-zoom", type=float, default=2.0)
    ap.add_argument("--target-model-h", type=float, default=40.0)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--nms-thresh", type=float, default=0.25)
    ap.add_argument("--out", type=Path,
                    default=Path("dynamic_tiling/runs/dynamic_run.frames.json"))
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"cannot open video: {args.video}")
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    gt_doc = json.loads(args.gt.read_text())
    gt_traj = build_target_trajectory(gt_doc, label="person", anchor="largest")

    discovery_period = max(1, int(round(args.fps / args.discovery_fps)))
    scheduler = TileScheduler(src_w, src_h, discovery_period=discovery_period,
                              max_zoom=args.max_zoom,
                              target_model_h=args.target_model_h)
    lock = TargetLock(frame_rate=int(args.fps))
    backend = HefBackend(args.hef, nms_score_threshold=args.nms_thresh)
    meter = BudgetMeter(budget_inf_per_s=args.budget, fps=args.fps)

    try:
        res = run(_frame_iter(cap, args.max_frames), src_w, src_h,
                  scheduler, lock, backend, meter, gt_traj)
    finally:
        backend.close()
        cap.release()

    sc = score_run(gt_traj, res.pred_traj, iou_thr=0.5)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    emit_frames_json(res, label=f"dynamic-b{int(args.budget)}", out_path=args.out)

    print(f"\nframes processed : {res.n_frames}")
    print(f"avg tiles/frame  : {res.avg_tiles_per_frame:.2f}  "
          f"(rt_factor {args.budget / (res.avg_tiles_per_frame * args.fps):.2f})"
          if res.avg_tiles_per_frame else "avg tiles/frame  : 0")
    print(f"GT target frames : {sc.n_gt_frames}")
    print(f"target recall    : {sc.recall:.3f}")
    print(f"target mean IoU  : {sc.mean_iou:.3f}")
    print(f"frames.json      : {args.out}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Write the launcher**

```bash
# dynamic_tiling/run_dynamic.sh
#!/usr/bin/env bash
# Run the dynamic tile scheduler over the benchmark video with real inference.
#   ./dynamic_tiling/run_dynamic.sh [--budget 300] [--max-frames 300]
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck disable=SC1091
source "$REPO_ROOT/setup_env.sh" >/dev/null 2>&1
cd "$REPO_ROOT"
exec python -m dynamic_tiling.run_dynamic \
  --video "/home/giladn/Videos/Drone/Training/DJI_20260430103421_0010_D_rotated.MP4" \
  --gt "tiling_benchmark/pxt_runs/pxt_GT-12x9-25-multi.frames.json" \
  "$@"
```

- [ ] **Step 3: Make the launcher executable**

```bash
chmod +x dynamic_tiling/run_dynamic.sh
```

- [ ] **Step 4: Real-chip smoke run (short clip)**

Run: `source setup_env.sh && ./dynamic_tiling/run_dynamic.sh --max-frames 120`
Expected: prints `avg tiles/frame` (well under 10 → rt_factor ≥ 1.0), a non-zero `target recall`, and writes `dynamic_tiling/runs/dynamic_run.frames.json`. The VDevice must be free (only one HEF backend at a time).

- [ ] **Step 5: Visual check in the viewer**

Run: `./tiling_benchmark/run_viewer.sh -- --frames dynamic_tiling/runs/dynamic_run.frames.json:dynamic`
Expected: the dynamic run overlays on the video; the locked target stays boxed as it moves.

- [ ] **Step 6: Commit**

```bash
git add dynamic_tiling/run_dynamic.py dynamic_tiling/run_dynamic.sh
git commit -m "dynamic_tiling: CLI driver + launcher for real-chip dynamic runs"
```

---

## Task 11: Compare dynamic vs static baselines

**Files:**
- Create: `dynamic_tiling/compare_baselines.py`

Score the static baseline runs (already produced by `tiling_benchmark`, e.g. `pxt_3x3-native+vga3x.frames.json`, `pxt_8x6-native.frames.json`) against the SAME GT target trajectory, then print a table dynamic vs each baseline at their respective avg inf/s.

- [ ] **Step 1: Write the comparison script**

```python
# dynamic_tiling/compare_baselines.py
"""Score static baseline frames.json files against the GT target trajectory and
tabulate them next to a dynamic run, all on the same single-target metric.

Static baselines have no track ids, so we reduce each frame to the single
detection best matching the GT target box that frame (the same single-target
question the dynamic harness answers)."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import dynamic_tiling  # noqa: F401
from .gt_track import build_target_trajectory
from .score import score_run, _iou


def baseline_pred_traj(frames_json: Path, gt_traj: dict, label="person") -> dict:
    doc = json.loads(frames_json.read_text())
    by_frame = {int(fr["frame"]): fr["detections"] for fr in doc["frames"]}
    pred = {}
    for f, gt_box in gt_traj.items():
        cands = [tuple(d["bbox"]) for d in by_frame.get(f, [])
                 if d.get("label") == label]
        if not cands:
            continue
        best = max(cands, key=lambda b: _iou(gt_box, b))
        if _iou(gt_box, best) > 0:
            pred[f] = best
    return pred


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True, type=Path)
    ap.add_argument("--dynamic", required=True, type=Path,
                    help="dynamic run frames.json")
    ap.add_argument("--baseline", nargs="+", type=Path, required=True,
                    help="static baseline frames.json files")
    args = ap.parse_args()

    gt_doc = json.loads(args.gt.read_text())
    gt_traj = build_target_trajectory(gt_doc, label="person", anchor="largest")

    rows = []
    for path in [args.dynamic, *args.baseline]:
        pred = baseline_pred_traj(path, gt_traj)
        sc = score_run(gt_traj, pred, iou_thr=0.5)
        rows.append((path.stem, sc.recall, sc.mean_iou, sc.n_hit, sc.n_gt_frames))

    print(f"{'run':40s} {'recall':>8s} {'mean_iou':>9s} {'hits':>6s}/{'gt':<6s}")
    for name, recall, miou, hits, ngt in rows:
        print(f"{name:40s} {recall:8.3f} {miou:9.3f} {hits:6d}/{ngt:<6d}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run the comparison**

Run:
```bash
source setup_env.sh
python -m dynamic_tiling.compare_baselines \
  --gt tiling_benchmark/pxt_runs/pxt_GT-12x9-25-multi.frames.json \
  --dynamic dynamic_tiling/runs/dynamic_run.frames.json \
  --baseline tiling_benchmark/pxt_runs/pxt_3x3-native+vga3x.frames.json \
             tiling_benchmark/pxt_runs/pxt_8x6-native.frames.json
```
Expected: a table; **success criterion = the dynamic run's recall/mean_iou ≥ the static baselines** at equal-or-lower avg inf/s.

- [ ] **Step 3: Commit**

```bash
git add dynamic_tiling/compare_baselines.py
git commit -m "dynamic_tiling: single-target comparison vs static baselines"
```

---

## Self-review notes

- **Spec coverage:** budget (Task 2), ByteTracker reuse (Task 4), levers 1-5 (Task 6), real re-inference (Task 5/9), GT single-target scoring (Tasks 7-8), viewer-compatible output (Task 9), CLI + validation + baseline comparison (Tasks 10-11). All spec sections map to tasks.
- **Type consistency:** `Det(cls,score,x,y,w,h)`, `CropRect`, `LockState`, `RunScore`, `RunResult` are defined once and reused with the same field names across tasks; the ByteTracker array format matches `hailo_drone_detection_manager.py:112-120`.
- **Open items from the spec** (non-blocking, resolved here): validation clip = the GT's source video (`DJI_...rotated.MP4`); decimated discovery pass = a 3×2 grid (`discovery_grid=(3,2)`), matching the GT recipe's medium grid rather than the full 12×9.
- **Risk:** `target_model_h`, `discovery_period`, recovery `span`, and `recovery_grid` are tuning knobs; Task 10/11 are where they get calibrated against the baselines. Defaults chosen from PERF_REPORT (≥32 model-px floor → target_model_h=40; 2 fps discovery).
