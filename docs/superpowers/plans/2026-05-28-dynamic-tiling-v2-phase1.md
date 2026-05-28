# Dynamic Tiling v2 Phase 1 — Multi-Target Lock + Per-Target ROIs

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `MultiTargetLock` and `MultiTargetTileScheduler` so every ByteTracker-activated track (person + vehicle by default) gets its own ROI each frame, while the user-selected target still gets recovery grid priority.

**Architecture:** `TargetState` dataclass lives in `types.py`. `MultiTargetLock` in `target_lock.py` holds one `ByteTrackerAdapter` per allowed class and maintains a `dict[(cls, track_id), TargetState]`. `MultiTargetTileScheduler` in `scheduler.py` emits one ROI per TRACKING target (selected first), recovery grid only for the selected target, and discovery on cadence. `run_multi` in `replay.py` is a sibling of `run`. `run_dynamic.py` grows a `--multi-target` flag.

**Tech Stack:** Python 3.10, ByteTrackerAdapter (from drone_follow), pytest, existing `types.py / scheduler.py / replay.py` patterns.

---

### Task 1: Add `TargetState` dataclass to `types.py`

**Files:**
- Modify: `dynamic_tiling/types.py`
- Test: `dynamic_tiling/tests/test_types.py`

- [ ] **Step 1: Write the failing test**

Add to `dynamic_tiling/tests/test_types.py`:

```python
from dynamic_tiling.types import TargetState

def test_target_state_defaults():
    ts = TargetState(key=(0, 1), cls=0)
    assert ts.status == "LOST"
    assert ts.frames_since_seen == 0
    assert ts.selected is False
    assert ts.bbox_norm == (0.0, 0.0, 0.0, 0.0)
    assert ts.last_velocity == (0.0, 0.0)
```

- [ ] **Step 2: Run test to verify it fails**

```
./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_types.py::test_target_state_defaults -v
```
Expected: `ImportError` or `FAILED` — `TargetState` not yet defined.

- [ ] **Step 3: Add `TargetState` to `dynamic_tiling/types.py`**

Append after the `LockState` dataclass:

```python
@dataclass
class TargetState:
    """Per-track state for multi-target tracking."""
    key: tuple                           # (cls, track_id) composite lock id
    cls: int
    bbox_norm: tuple = (0.0, 0.0, 0.0, 0.0)  # (x,y,w,h) normalized
    status: str = "LOST"                  # TRACKING | SEARCHING | LOST
    frames_since_seen: int = 0
    last_velocity: tuple = (0.0, 0.0)
    selected: bool = False               # True if this is the user-selected target
```

- [ ] **Step 4: Run test to verify it passes**

```
./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_types.py -v
```
Expected: all 5 tests PASS (4 existing + new).

---

### Task 2: Add `MultiTargetLock` to `target_lock.py`

**Files:**
- Modify: `dynamic_tiling/target_lock.py`
- Create: `dynamic_tiling/tests/test_multi_target_lock.py`

- [ ] **Step 1: Write the failing tests**

Create `dynamic_tiling/tests/test_multi_target_lock.py`:

```python
import dynamic_tiling  # noqa: F401
from dynamic_tiling.types import Det, TargetState
from dynamic_tiling.target_lock import MultiTargetLock


def _person(x, y, w=0.08, h=0.20, score=0.9):
    return Det(cls=0, score=score, x=x, y=y, w=w, h=h)


def _vehicle(x, y, w=0.15, h=0.10, score=0.85):
    return Det(cls=1, score=score, x=x, y=y, w=w, h=h)


def test_multi_target_lock_tracks_one_person_one_vehicle():
    lock = MultiTargetLock(target_classes={0, 1})
    dets = [_person(0.20, 0.40), _vehicle(0.70, 0.50)]
    states = None
    for _ in range(3):
        states = lock.step(dets)
    # Both should be TRACKING after 3 frames.
    assert any(s.cls == 0 and s.status == "TRACKING" for s in states)
    assert any(s.cls == 1 and s.status == "TRACKING" for s in states)
    # Keys are (cls, track_id) tuples.
    keys = [s.key for s in states]
    assert all(isinstance(k, tuple) and len(k) == 2 for k in keys)
    # Person and vehicle have different class in their key.
    cls_set = {s.key[0] for s in states}
    assert cls_set == {0, 1}


def test_multi_target_lock_selects_from_gt():
    lock = MultiTargetLock(target_classes={0, 1})
    dets = [_person(0.20, 0.40), _vehicle(0.70, 0.50)]
    # Feed for 2 frames without GT to establish tracks.
    lock.step(dets)
    lock.step(dets)
    # Feed with GT bbox matching the person.
    person_bbox = (0.20, 0.40, 0.08, 0.20)
    states = lock.step(dets, gt_bbox_norm=person_bbox, gt_cls=0)
    assert lock.selected_key is not None
    assert lock.selected_key[0] == 0   # class 0 = person
    selected = next(s for s in states if s.selected)
    assert selected.cls == 0


def test_multi_target_lock_loses_track_after_buffer():
    track_buffer = 5
    lock = MultiTargetLock(target_classes={0}, track_buffer=track_buffer)
    # Establish a track.
    for _ in range(2):
        lock.step([_person(0.40, 0.40)])
    # Confirm TRACKING.
    states = lock.step([_person(0.40, 0.40)])
    assert any(s.status == "TRACKING" for s in states)
    # Remove detections for track_buffer + 1 frames.
    last_states = None
    for _ in range(track_buffer + 1):
        last_states = lock.step([])
    # Target should be LOST (not returned from step, or returned as LOST).
    # step() returns only non-LOST states; so either empty OR contains LOST.
    alive = [s for s in last_states if s.status != "LOST"]
    assert len(alive) == 0
```

- [ ] **Step 2: Run tests to verify they fail**

```
./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_multi_target_lock.py -v
```
Expected: `ImportError` — `MultiTargetLock` not yet defined.

- [ ] **Step 3: Implement `MultiTargetLock` in `target_lock.py`**

Append after the `TargetLock` class:

```python
from .types import Det, LockState, TargetState

_REACQ_IOU = 0.3  # already defined above; reuse


class MultiTargetLock:
    """Tracks ALL activated ByteTracker tracks across multiple classes.

    One ByteTrackerAdapter per allowed class avoids cross-class IoU id
    collisions.  A single ``selected_key`` (composite key ``(cls, track_id)``)
    marks the user-selected target; it is set once via GT IoU matching and
    never cleared automatically.
    """

    def __init__(self, target_classes: set | frozenset = frozenset({0, 1}),
                 track_buffer: int = 90, **tracker_kwargs):
        self._trackers = {c: create_tracker("byte", track_buffer=track_buffer,
                                            **tracker_kwargs)
                          for c in target_classes}
        self.target_classes = set(target_classes)
        self.track_buffer = track_buffer
        self.targets: dict[tuple, TargetState] = {}
        self.selected_key: tuple | None = None

    def step(self, dets: "Sequence[Det]", *,
             gt_bbox_norm: tuple | None = None,
             gt_cls: int | None = None) -> "list[TargetState]":
        """Feed one frame of detections; return non-LOST TargetStates."""
        # --- 1. partition dets by class and run each tracker ---
        active_keys: set[tuple] = set()
        for cls, tracker in self._trackers.items():
            cls_dets = [d for d in dets if d.cls == cls]
            tracks = tracker.update(dets_to_array(cls_dets))
            for t in tracks:
                if not t.is_activated or not t.filtered_tlwh:
                    continue
                key = (cls, t.track_id)
                active_keys.add(key)
                s = self.targets.get(key)
                if s is None:
                    s = TargetState(key=key, cls=cls)
                    self.targets[key] = s
                # --- velocity update (same rule as v1 TargetLock) ---
                had_prev = s.frames_since_seen == 0 and s.bbox_norm[2] > 0
                cx_old = (s.bbox_norm[0] + s.bbox_norm[2] / 2) if had_prev else None
                cy_old = (s.bbox_norm[1] + s.bbox_norm[3] / 2) if had_prev else None
                s.bbox_norm = tuple(t.filtered_tlwh)
                cx_new = s.bbox_norm[0] + s.bbox_norm[2] / 2
                cy_new = s.bbox_norm[1] + s.bbox_norm[3] / 2
                if cx_old is not None:
                    s.last_velocity = (cx_new - cx_old, cy_new - cy_old)
                s.status = "TRACKING"
                s.frames_since_seen = 0

        # --- 2. age inactive tracks ---
        for key, s in list(self.targets.items()):
            if key not in active_keys:
                s.frames_since_seen += 1
                if s.frames_since_seen > self.track_buffer:
                    s.status = "LOST"
                elif s.status != "LOST":
                    s.status = "SEARCHING"

        # --- 3. GT-seeded selection (once only) ---
        if self.selected_key is None and gt_bbox_norm is not None:
            best_key, best_iou = None, _REACQ_IOU
            for key, s in self.targets.items():
                if s.status == "TRACKING":
                    if gt_cls is not None and s.cls != gt_cls:
                        continue
                    iou = _iou_tlwh(gt_bbox_norm, s.bbox_norm)
                    if iou > best_iou:
                        best_iou, best_key = iou, key
            if best_key is not None:
                self.selected_key = best_key

        # --- 4. propagate selected flag ---
        for key, s in self.targets.items():
            s.selected = (key == self.selected_key)

        # --- 5. return non-LOST, sorted TRACKING first, selected first ---
        alive = [s for s in self.targets.values() if s.status != "LOST"]
        alive.sort(key=lambda s: (0 if s.status == "TRACKING" else 1,
                                  0 if s.selected else 1))
        return alive
```

Note: the `from .types import TargetState` import needs to be added to the existing `from .types import Det, LockState` line at the top of the file.

- [ ] **Step 4: Run tests to verify they pass**

```
./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_multi_target_lock.py -v
```
Expected: all 3 tests PASS.

- [ ] **Step 5: Confirm existing tests still pass**

```
./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/ -v
```
Expected: 35 passed (32 old + 3 new).

---

### Task 3: Add `MultiTargetTileScheduler` to `scheduler.py`

**Files:**
- Modify: `dynamic_tiling/scheduler.py`
- Create: `dynamic_tiling/tests/test_multi_target_scheduler.py`

- [ ] **Step 1: Write the failing tests**

Create `dynamic_tiling/tests/test_multi_target_scheduler.py`:

```python
from dynamic_tiling.types import TargetState
from dynamic_tiling.budget import BudgetMeter
from dynamic_tiling.scheduler import MultiTargetTileScheduler


def _meter():
    return BudgetMeter(budget_inf_per_s=300, fps=30, window_s=1.0)


def _tracking(cls, track_id, x=0.4, y=0.4, selected=False):
    s = TargetState(key=(cls, track_id), cls=cls,
                    bbox_norm=(x, y, 0.08, 0.20), status="TRACKING")
    s.selected = selected
    return s


def _searching(cls, track_id, x=0.4, y=0.4, selected=False):
    s = TargetState(key=(cls, track_id), cls=cls,
                    bbox_norm=(x, y, 0.08, 0.20), status="SEARCHING",
                    frames_since_seen=3)
    s.selected = selected
    return s


def test_multi_target_scheduler_emits_roi_per_tracking_target():
    sched = MultiTargetTileScheduler(src_w=4000, src_h=3000,
                                     discovery_period=15)
    targets = [
        _tracking(0, 1, x=0.2),
        _tracking(0, 2, x=0.5),
        _tracking(1, 3, x=0.8),
    ]
    crops = sched.decide(targets, frame_idx=1, meter=_meter())
    # Off discovery cadence, no selected+SEARCHING => 3 per-target ROIs.
    assert len(crops) == 3


def test_multi_target_scheduler_recovery_only_for_selected():
    sched = MultiTargetTileScheduler(src_w=4000, src_h=3000,
                                     discovery_period=15,
                                     recovery_grid=(3, 3))
    targets = [
        _tracking(0, 1, x=0.2),
        _tracking(0, 2, x=0.5),
        _searching(1, 3, x=0.7, selected=True),
    ]
    crops = sched.decide(targets, frame_idx=1, meter=_meter())
    # Recovery branch: selected target is SEARCHING => 9-tile recovery grid.
    # No per-target ROIs when recovery is active.
    assert len(crops) == 9


def test_multi_target_scheduler_selected_roi_first():
    sched = MultiTargetTileScheduler(src_w=4000, src_h=3000,
                                     discovery_period=15)
    selected_state = _tracking(0, 1, x=0.2, selected=True)
    other_state = _tracking(0, 2, x=0.7, selected=False)
    targets = [other_state, selected_state]  # selected passed second
    crops = sched.decide(targets, frame_idx=1, meter=_meter())
    assert len(crops) >= 2
    # First crop must be centered near selected target's x=0.2.
    sel_cx = (selected_state.bbox_norm[0] + selected_state.bbox_norm[2] / 2) * 4000
    first_cx = crops[0].x + crops[0].w / 2
    other_cx = (other_state.bbox_norm[0] + other_state.bbox_norm[2] / 2) * 4000
    assert abs(first_cx - sel_cx) < abs(first_cx - other_cx)
```

- [ ] **Step 2: Run tests to verify they fail**

```
./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_multi_target_scheduler.py -v
```
Expected: `ImportError` — `MultiTargetTileScheduler` not yet defined.

- [ ] **Step 3: Implement `MultiTargetTileScheduler` in `scheduler.py`**

Append after `TileScheduler`:

```python
from .types import CropRect, LockState, TargetState, MODEL_W, MODEL_H, MODEL_ASPECT


class MultiTargetTileScheduler:
    """Emits one ROI per TRACKING target + recovery grid for the selected
    target + discovery on cadence.  Phase 1: no merging, no aging counter.
    """

    def __init__(self, src_w: int, src_h: int, *,
                 discovery_period: int = 15, discovery_grid: tuple = (3, 2),
                 recovery_grid: tuple = (3, 3), max_zoom: float = 2.0,
                 target_model_h: float = 40.0, roi_margin_frac: float = 0.25,
                 recovery_span: float = 0.4):
        self.src_w = src_w
        self.src_h = src_h
        self.discovery_period = discovery_period
        self.discovery_grid = discovery_grid
        self.recovery_grid = recovery_grid
        self.max_zoom = max_zoom
        self.target_model_h = target_model_h
        self.roi_margin_frac = roi_margin_frac
        self.recovery_span = recovery_span
        # Reuse TileScheduler helpers via composition.
        self._v1 = TileScheduler(src_w, src_h,
                                 discovery_period=discovery_period,
                                 discovery_grid=discovery_grid,
                                 recovery_grid=recovery_grid,
                                 max_zoom=max_zoom,
                                 target_model_h=target_model_h,
                                 roi_margin_frac=roi_margin_frac,
                                 recovery_span=recovery_span)

    def _roi_for_target(self, s: TargetState) -> CropRect:
        """Same adaptive-zoom ROI logic as v1 TileScheduler._roi, applied to a
        TargetState instead of a LockState."""
        lock = LockState(track_id=s.key[1],
                         bbox_norm=s.bbox_norm,
                         status=s.status,
                         frames_since_seen=s.frames_since_seen,
                         last_velocity=s.last_velocity)
        return self._v1._roi(lock)

    def decide(self, targets: "list[TargetState]", frame_idx: int,
               meter) -> "list[CropRect]":
        on_cadence = (frame_idx % self.discovery_period == 0)

        # --- recovery branch: SELECTED target is SEARCHING / LOST ---
        selected = next((s for s in targets if s.selected), None)
        if (selected is not None
                and selected.status in ("SEARCHING", "LOST")
                and selected.bbox_norm[2] > 0):
            lock = LockState(track_id=selected.key[1],
                             bbox_norm=selected.bbox_norm,
                             status=selected.status,
                             frames_since_seen=selected.frames_since_seen,
                             last_velocity=selected.last_velocity)
            gx, gy = self.recovery_grid
            bx, by, bw, bh = lock.bbox_norm
            ecx = bx + bw / 2 + lock.last_velocity[0] * lock.frames_since_seen
            ecy = by + bh / 2 + lock.last_velocity[1] * lock.frames_since_seen
            span = self.recovery_span
            half = span / 2
            x0_n = max(0.0, min(1.0 - span, ecx - half))
            y0_n = max(0.0, min(1.0 - span, ecy - half))
            x0 = x0_n * self.src_w
            y0 = y0_n * self.src_h
            crops = self._v1._grid(gx, gy, x0, y0,
                                   span * self.src_w, span * self.src_h, "s")
            budget = int(meter.available(frame_idx))
            if budget >= 0 and len(crops) > budget:
                crops = crops[:max(0, budget)]
            return crops

        # --- normal path: per-target ROIs + discovery ---
        crops: list[CropRect] = []

        # Selected target's ROI first (survives budget squeeze).
        tracking_targets = [s for s in targets if s.status == "TRACKING"]
        if selected is not None and selected.status == "TRACKING":
            crops.append(self._roi_for_target(selected))

        # Other TRACKING targets.
        for s in tracking_targets:
            if not s.selected:
                crops.append(self._roi_for_target(s))

        # Discovery grid on cadence.
        if on_cadence:
            gx, gy = self.discovery_grid
            crops += self._v1._grid(gx, gy, 0, 0,
                                    self.src_w, self.src_h, "m")

        budget = int(meter.available(frame_idx))
        if budget >= 0 and len(crops) > budget:
            crops = crops[:max(0, budget)]
        return crops
```

- [ ] **Step 4: Run tests to verify they pass**

```
./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_multi_target_scheduler.py -v
```
Expected: all 3 tests PASS.

- [ ] **Step 5: Confirm all tests still pass**

```
./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/ -v
```
Expected: 38 passed.

---

### Task 4: Add `run_multi` to `replay.py`

**Files:**
- Modify: `dynamic_tiling/replay.py`
- Modify: `dynamic_tiling/tests/test_replay_integration.py`

- [ ] **Step 1: Write the failing test**

Add to `dynamic_tiling/tests/test_replay_integration.py`:

```python
import numpy as np
from dynamic_tiling.types import CropRect
from dynamic_tiling.budget import BudgetMeter
from dynamic_tiling.scheduler import MultiTargetTileScheduler
from dynamic_tiling.target_lock import MultiTargetLock
from dynamic_tiling.replay import run_multi


def test_run_multi_records_per_target_rois():
    src_w, src_h = 4000, 3000
    n_frames = 6
    # Two persons moving slightly.
    gt_traj = {f: (0.40 + 0.01 * f, 0.40, 0.08, 0.20) for f in range(n_frames)}
    person2 = {f: (0.70 + 0.005 * f, 0.50, 0.08, 0.20) for f in range(n_frames)}

    class _TwoPersonBackend:
        def infer(self, frame, crop: CropRect, frame_idx):
            results = []
            for traj in [gt_traj, person2]:
                gx, gy, gw, gh = traj[frame_idx]
                gcx = (gx + gw / 2) * src_w
                gcy = (gy + gh / 2) * src_h
                if not (crop.x <= gcx <= crop.x + crop.w and
                        crop.y <= gcy <= crop.y + crop.h):
                    continue
                lx = (gcx - crop.x) / crop.w
                ly = (gcy - crop.y) / crop.h
                lw = gw * src_w / crop.w
                lh = gh * src_h / crop.h
                results.append(
                    type("D", (), dict(cls=0, x=lx - lw / 2, y=ly - lh / 2,
                                       w=lw, h=lh, score=0.9))()
                )
            return results

    frames = [np.zeros((src_h, src_w, 3), np.uint8) for _ in range(n_frames)]
    lock = MultiTargetLock(target_classes={0, 1})
    sched = MultiTargetTileScheduler(src_w, src_h, discovery_period=2)
    result = run_multi(
        frames=frames, src_w=src_w, src_h=src_h,
        scheduler=sched,
        lock=lock,
        backend=_TwoPersonBackend(),
        meter=BudgetMeter(budget_inf_per_s=300, fps=30),
        gt_traj=gt_traj,
        gt_cls=0,
    )
    assert result.n_frames == n_frames
    # After a couple of frames to establish tracks, expect ≥ 2 "dynamic" tiles.
    dynamic_frames = [
        f for f in range(2, n_frames)
        if any(t[4] == "dynamic" for t in result.frame_tiles.get(f, []))
    ]
    assert len(dynamic_frames) >= 2, (
        f"expected >=2 frames with 'dynamic' tiles, got {len(dynamic_frames)}: "
        f"{result.frame_tiles}"
    )
```

- [ ] **Step 2: Run test to verify it fails**

```
./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_replay_integration.py::test_run_multi_records_per_target_rois -v
```
Expected: `ImportError` — `run_multi` not defined.

- [ ] **Step 3: Implement `run_multi` in `replay.py`**

Add imports at the top of `replay.py`:
```python
from .scheduler import TileScheduler, MultiTargetTileScheduler
from .target_lock import TargetLock, MultiTargetLock
```

Append after `emit_frames_json`:

```python
def run_multi(frames, src_w: int, src_h: int,
              scheduler: MultiTargetTileScheduler,
              lock: MultiTargetLock, backend, meter, gt_traj: dict,
              gt_cls: int = 0) -> RunResult:
    """Multi-target replay loop.  Feeds all allowed-class dets to the lock,
    asks the scheduler for per-target ROIs, and records the selected target's
    bbox in pred_traj (same schema as run() for score_run compatibility)."""
    res = RunResult()
    gt_seeded = False
    frame_idx = -1

    for frame_idx, frame in enumerate(frames):
        # Query targets from the lock (starts empty).
        targets = lock.targets  # dict at this point; expose as list from last step
        # Actually call decide with the current target list.
        current_targets = [s for s in lock.targets.values()
                           if s.status != "LOST"]
        crops = scheduler.decide(current_targets, frame_idx, meter)
        meter.charge(len(crops), frame_idx)
        res.total_tiles += len(crops)

        # Inference.
        dets = []
        for crop in crops:
            local = backend.infer(frame, crop, frame_idx)
            dets += map_to_source(local, crop, src_w, src_h)
        dets = nms(dets, iou_thr=0.5)
        res.frame_dets[frame_idx] = dets

        # Feed all allowed-class dets to the lock.
        dets_for_lock = [d for d in dets if d.cls in lock.target_classes]
        gt_box = gt_traj.get(frame_idx)
        if not gt_seeded and gt_box is not None:
            targets = lock.step(dets_for_lock,
                                gt_bbox_norm=gt_box, gt_cls=gt_cls)
            gt_seeded = True
        else:
            targets = lock.step(dets_for_lock)

        # Tag tiles.
        is_recovery = (
            any(s.selected and s.status in ("SEARCHING", "LOST")
                for s in targets)
        )
        tagged: list = []
        if is_recovery:
            for c in crops:
                tagged.append((c.x / src_w, c.y / src_h,
                               c.w / src_w, c.h / src_h, "single-scale"))
        else:
            # Per-target ROIs are mode "s"; discovery is mode "m".
            roi_count = 0
            for c in crops:
                if c.mode == "m":
                    cat = "multi-scale"
                else:
                    cat = "dynamic"
                    roi_count += 1
                tagged.append((c.x / src_w, c.y / src_h,
                               c.w / src_w, c.h / src_h, cat))
        res.frame_tiles[frame_idx] = tagged

        # Record selected target bbox for single-target scoring compatibility.
        sel = next((s for s in targets if s.selected
                    and s.status == "TRACKING"), None)
        if sel is not None:
            res.pred_traj[frame_idx] = tuple(sel.bbox_norm)

    res.n_frames = frame_idx + 1
    return res
```

- [ ] **Step 4: Run test to verify it passes**

```
./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/test_replay_integration.py -v
```
Expected: all 4 tests PASS.

- [ ] **Step 5: Confirm all tests still pass**

```
./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/ -v
```
Expected: 39 passed.

---

### Task 5: Add `--multi-target` flag to `run_dynamic.py`

**Files:**
- Modify: `dynamic_tiling/run_dynamic.py`

- [ ] **Step 1: Add imports and flags**

Add to the imports block:

```python
from .scheduler import TileScheduler, MultiTargetTileScheduler
from .target_lock import TargetLock, MultiTargetLock
from .replay import run, run_multi, emit_frames_json
```

Add to `ap.add_argument` block:

```python
ap.add_argument("--multi-target", action="store_true",
                help="Use multi-target dynamic tiling (v2) instead of single-target (v1).")
ap.add_argument("--target-classes", default="0,1",
                help="Comma-separated class ids for multi-target mode (default: 0,1).")
```

Replace the scheduler/lock construction and `run()` call with a branch:

```python
if args.multi_target:
    target_classes = {int(c) for c in args.target_classes.split(",")}
    scheduler = MultiTargetTileScheduler(src_w, src_h,
                                         discovery_period=discovery_period,
                                         max_zoom=args.max_zoom,
                                         target_model_h=args.target_model_h)
    lock = MultiTargetLock(target_classes=target_classes,
                           track_buffer=int(args.fps))
    try:
        res = run_multi(_frame_iter(cap, args.max_frames), src_w, src_h,
                        scheduler, lock, backend, meter, gt_traj,
                        gt_cls=0)
    finally:
        backend.close()
        cap.release()
else:
    scheduler = TileScheduler(src_w, src_h,
                              discovery_period=discovery_period,
                              max_zoom=args.max_zoom,
                              target_model_h=args.target_model_h)
    lock = TargetLock(frame_rate=int(args.fps))
    try:
        res = run(_frame_iter(cap, args.max_frames), src_w, src_h,
                  scheduler, lock, backend, meter, gt_traj)
    finally:
        backend.close()
        cap.release()
```

- [ ] **Step 2: Verify imports still work**

```
./hailo-apps/venv_hailo_apps/bin/python -c "from dynamic_tiling.run_dynamic import main; print('OK')"
```
Expected: `OK`

- [ ] **Step 3: Confirm all tests still pass**

```
./hailo-apps/venv_hailo_apps/bin/python -m pytest dynamic_tiling/tests/ -v
```
Expected: 39 passed.

---

### Task 6: Commit

- [ ] **Step 1: Stage and commit**

```bash
git add dynamic_tiling/types.py \
        dynamic_tiling/target_lock.py \
        dynamic_tiling/scheduler.py \
        dynamic_tiling/replay.py \
        dynamic_tiling/run_dynamic.py \
        dynamic_tiling/tests/test_multi_target_lock.py \
        dynamic_tiling/tests/test_multi_target_scheduler.py \
        dynamic_tiling/tests/test_replay_integration.py \
        dynamic_tiling/tests/test_types.py \
        docs/superpowers/specs/2026-05-28-dynamic-tiling-v2-multi-target.md \
        docs/superpowers/plans/2026-05-28-dynamic-tiling-v2-phase1.md
git commit -m "dynamic_tiling: v2 Phase 1 — multi-target lock + per-target ROIs"
```

---

## Self-Review

### Spec Coverage
- ✅ `MultiTargetLock` with one tracker per class — Task 2
- ✅ `TargetState` in `types.py` — Task 1
- ✅ `selected_key` set via GT IoU — Task 2
- ✅ `MultiTargetTileScheduler.decide` — Task 3
- ✅ Recovery grid only for selected target — Task 3
- ✅ Per-target ROIs, selected first — Task 3
- ✅ `run_multi` in `replay.py` — Task 4
- ✅ `--multi-target` / `--target-classes` CLI flags — Task 5
- ✅ Tests: `test_multi_target_lock.py` (3 tests) — Task 2
- ✅ Tests: `test_multi_target_scheduler.py` (3 tests) — Task 3
- ✅ Tests: `test_run_multi_records_per_target_rois` — Task 4
- ✅ NO merging, NO aging counter, NO new viewer category (Phase 2)

### Type Consistency
- `TargetState.key` is `tuple` throughout; `selected_key` is `tuple | None`
- `dets_to_array` reused from existing `target_lock.py` (no duplication)
- `_roi_for_target` converts `TargetState` → `LockState` to delegate to v1 `_roi`

### Phase 1 Scope
- Tagging: `"dynamic"` for per-target ROIs, `"multi-scale"` for discovery, `"single-scale"` for recovery — no `"dynamic-merged"` (Phase 2)
