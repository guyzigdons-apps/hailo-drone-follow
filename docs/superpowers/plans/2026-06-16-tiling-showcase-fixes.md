# Dynamic-Tiling Showcase Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the dynamic-tiling showcase on the DJI car clips run its full flat dense load every frame, show the live tracked target (distinct colour) plus all other detections carried-forward correctly, with no stale duplicate boxes, and render a polished realtime-60fps demo video.

**Architecture:** The live runner (`run_showcase.py`) drives a `DynamicTilingController` (seed-tracked SOT via `SeedTracker` + flat rolling dense sweep via `StripedDenseScheduler` + `DetectionPersistence` carry-forward). Detections are recorded per-frame to `frames.json` and rendered by `overlay_viewer.py`. The fixes are: (1) stop the budget meter trimming the flat `ROI + dense_per_frame` load; (2) record each frame with the tiles that actually produced its detections; (3) run a callback-side full-frame NMS over the carried-forward union (target + aged persisted detections) so the target has no duplicate "friend" and stale boxes decay — the GStreamer aggregator keeps doing only within-frame NMS; (4) recolour the `target` box and keep native tiles (no zoom); (5) make the seed a *user selection* that snaps to a real dense detection at a chosen frame (emulates click-to-track; the dense grid bootstraps detection, the seed just picks which box to follow), with a tight gate so it cannot grab a neighbour; (6) re-run + export the demo video and verify realtime.

**Tech Stack:** Python 3.10, GStreamer (`hailotilecropper_dynamic` + `hailotileaggregator` + `hailonet`), HAILO10H, pytest. Run tests with `source setup_env.sh && python -m pytest`.

---

## Background facts (verified against the current `showcase_0007` run)

- Source `DJI_20260614160255_0007_D.MP4` is 3840×2160 @ 59.94 fps, no prepare step needed.
- `BudgetMeter.available()` returns `remaining / window_frames` (a per-frame *average share*). With `--budget 300` at 60 fps the trailing-second window fills to ~150 tiles, so `available ≈ (300−150)/60 = 2.5 → int → 2`. The scheduler then trims `ROI(1) + dense(2) = 3` to 2, dropping one dense tile every TRACKING frame. Measured tile-count distribution: `{2: 3493, 3: 116}`; dense-per-frame `{1: 1432, 2: 2177}`. **This is the "only 1 dense tile, skip one" bug.**
- `available()` is shared by `TileScheduler`, `MultiTargetTileScheduler`, `StripedDenseScheduler`, and `modifiers/budget_trim.py`, and its semantics are locked by `hailo_tiling/tests/test_budget.py` + `tiling_lab/tests/test_budget.py`. **Do NOT change `available()`** — fix the showcase by giving it enough budget that the flat load is never trimmed (the `dense_per_frame` count is itself the real cap).
- In `run_showcase.py` the probe reads detections from the current buffer (produced by the tiles pushed during the *previous* probe) but records the tiles it computes for the *next* frame. So recorded tiles lead their detections by one frame. **This is the "tiles don't match the boxes / look skipped" bug.**
- The seed-tracked target's own detection is also re-published by `DetectionPersistence` as a generic `vehicle`; when the target moves, the old box lingers in a not-yet-re-swept cell next to the live target box. **This is the "tracked bbox got a friend from the dense tile" bug.**
- **Verified frame-0 detectability:** in the last run the dynamic ROI was a native 640×480 crop centred exactly on the car for frames 1–27 yet produced **0 detections**; it only detected from frame 28 once bound to a real bbox and `_roi` zoomed to a 388px crop (1.65×). A tiny far car (~20px) is not detectable in a native crop. Seeding at frame 0 also let the acquisition gate grow until it bound to a detection 0.15 (≈576px) away — likely a different car.
- **Decision (confirmed by user):** native tiles, **no extra zoom**; carry-forward persistence ("saved until the tile's next iteration"); keep 2 dense/frame + rely on persistence; the **`target` box must be a distinct colour**; deliverable is a polished demo video that sustains realtime 60 fps. **The seed emulates a user selection and is applied at a frame where the dense grid has ALREADY detected the target** — it snaps to that real dense detection and tracks it. This solves bootstrapping (no need to detect a tiny car from a fake seed box) and avoids the wrong-car bind.

## File Structure

- `hailo_tiling/dynamic/striped.py` — flat scheduler; already returns `ROI + dense_per_frame`. No logic change; covered by a new test that proves it is not trimmed under the showcase budget.
- `hailo_tiling/dynamic/nms.py` — **new**; `nms_merge()` full-frame NMS with target/age/confidence priority (Task 3a).
- `hailo_tiling/dynamic/persistence.py` — add `age` + `tick()` for decay-aware merge (Task 3b).
- `tiling_lab/live/controller.py` — `DynamicTilingController`: raise the default budget; publish `nms_merge([target] + aged persisted)` in `step_showcase` (Task 3c); delete the old IoU dedup.
- `tiling_lab/harness/target_lock.py` — `SeedTracker`: tight, non-growing pre-bind selection gate so the click snaps only to a dense detection at the click (Task 5).
- `tiling_lab/live/run_showcase.py` — raise `--budget` default; record the previously-pushed tiles with the current detections (Task 2).
- `tiling_lab/viewer/overlay_viewer.py` — give the `target` label a distinct colour.
- Tests: `tiling_lab/tests/test_showcase_controller.py`, `tiling_lab/tests/test_seed_tracker.py`, `hailo_tiling/tests/test_striped_scheduler.py`, `hailo_tiling/tests/test_run_showcase_tiles.py`, plus new `hailo_tiling/tests/test_nms_merge.py`, `hailo_tiling/tests/test_persistence.py`, `tiling_lab/tests/test_viewer_colors.py`.

---

## Task 1: Stop the budget meter trimming the flat dense load

**Files:**
- Modify: `tiling_lab/live/controller.py` (default `budget_inf_per_s`)
- Modify: `tiling_lab/live/run_showcase.py:114` (`--budget` default)
- Test: `hailo_tiling/tests/test_striped_scheduler.py`

- [ ] **Step 1: Write the failing test** — under a showcase-sized budget the flat load is never trimmed across a full dense sweep.

```python
# Append to hailo_tiling/tests/test_striped_scheduler.py
from hailo_tiling.budget import BudgetMeter


def test_flat_load_not_trimmed_under_showcase_budget():
    s = StripedDenseScheduler(3840, 2160, fps=60.0, dense_per_frame=2)
    meter = BudgetMeter(budget_inf_per_s=600.0, fps=60.0)  # showcase default
    lock = _tracking_lock()
    counts = []
    for f in range(120):                       # two full dense sweeps
        crops = s.decide(lock, f, meter)
        meter.charge(len(crops), f)            # charge like the controller does
        counts.append(len(crops))
    # Every TRACKING frame keeps ROI(1) + dense(2) = 3; never trimmed to 2.
    assert min(counts) == 3 and max(counts) == 3
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && python -m pytest hailo_tiling/tests/test_striped_scheduler.py::test_flat_load_not_trimmed_under_showcase_budget -v`
Expected: FAIL — counts drop to 2 once the window fills (budget 600 still trims at fps 60? verify: steady share = 600/60 − spend; with spend 3 the share is 7 ≥ 3, so this should PASS at 600). If it PASSES immediately, lower the test budget to 300 first to demonstrate the trim, confirm FAIL, then restore 600.

- [ ] **Step 3: Raise the showcase budget defaults**

```python
# tiling_lab/live/run_showcase.py — change the --budget default
ap.add_argument("--budget", type=float, default=600.0,
                help="tile-inference/s cap. The showcase load is already flat "
                     "(ROI + --dense-per-frame); this must be high enough not to "
                     "trim it: budget/fps must exceed 2*(1+dense_per_frame). "
                     "600 @ 60fps leaves headroom for recovery bursts.")
```

```python
# tiling_lab/live/controller.py — DynamicTilingController.__init__ default
def __init__(self, src_w: int, src_h: int, *, fps: float = 30.0,
             budget_inf_per_s: float = 600.0, track_buffer: int = 90,
```

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && python -m pytest hailo_tiling/tests/test_striped_scheduler.py -v`
Expected: PASS (all striped tests).

- [ ] **Step 5: Commit**

```bash
git add hailo_tiling/tests/test_striped_scheduler.py tiling_lab/live/run_showcase.py tiling_lab/live/controller.py
git commit -m "fix(tiling): showcase budget no longer trims the flat ROI+dense load"
```

---

## Task 2: Record each frame with the tiles that produced its detections

**Files:**
- Modify: `tiling_lab/live/run_showcase.py` (probe recording; helper extraction)
- Test: `tiling_lab/tests/test_run_showcase_tiles.py`

The probe currently records the tiles it just computed (for the *next* frame) alongside detections from the *previous* frame's tiles. Track the previously-pushed tiles and record those with the current detections.

- [ ] **Step 1: Write the failing test** — a small pure helper that pairs detections with the tiles that produced them.

```python
# Append to tiling_lab/tests/test_run_showcase_tiles.py
from tiling_lab.live.run_showcase import build_frame_record, tiles_static_to_dicts


def test_frame_record_uses_previous_tiles_not_next():
    # Frame 5's detections were produced by the tiles applied to frame 5, which
    # were pushed during frame 4's probe (= prev_pushed). The record must carry
    # prev_pushed, NOT the tiles just computed for frame 6.
    prev_pushed = "0.1,0.1,0.1,0.1,m"     # tiles that produced these dets
    next_pushed = "0.8,0.8,0.1,0.1,m"     # tiles for the NEXT frame
    dets = [{"label": "vehicle", "confidence": 0.9, "bbox": [0.12, 0.12, 0.03, 0.02]}]
    rec = build_frame_record(frame=5, detections=dets, applied_tiles=prev_pushed)
    assert rec["frame"] == 5
    assert rec["detections"] == dets
    assert rec["tiles"] == tiles_static_to_dicts(prev_pushed)
    # sanity: it is NOT the next-frame tiles
    assert rec["tiles"] != tiles_static_to_dicts(next_pushed)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && python -m pytest tiling_lab/tests/test_run_showcase_tiles.py::test_frame_record_uses_previous_tiles_not_next -v`
Expected: FAIL with `ImportError: cannot import name 'build_frame_record'`.

- [ ] **Step 3: Add the helper and use previously-pushed tiles in the probe**

```python
# tiling_lab/live/run_showcase.py — add near tiles_static_to_dicts()
def build_frame_record(frame, detections, applied_tiles):
    """Pair a frame's detections with the tiles that ACTUALLY produced them
    (the tiles applied to this frame = the ones pushed on the previous probe),
    so the viewer draws each box inside the tile that detected it."""
    return {"frame": frame, "detections": detections,
            "tiles": tiles_static_to_dicts(applied_tiles)}
```

```python
# tiling_lab/live/run_showcase.py — in the probe, replace the record append.
# Add `state["applied"] = INITIAL_TILES` to the state dict initialiser, then:
        tiles, records = ctrl.step_showcase(target_dets, all_dets)
        pushed = tiles or INITIAL_TILES
        # Record THIS frame's detections against the tiles that produced them
        # (pushed on the PREVIOUS probe), then push `pushed` for the next frame.
        frame_records.append(build_frame_record(state["frame"], records,
                                                 state["applied"]))
        cropper.set_property("tiles-static", pushed)
        state["applied"] = pushed
        n_tiles = pushed.count(";") + 1 if pushed else 0
        tile_counts.append(n_tiles)
        f = state["frame"]
```

(Remove the old `frame_records.append({...tiles_static_to_dicts(pushed)...})` line.)

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && python -m pytest tiling_lab/tests/test_run_showcase_tiles.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tiling_lab/live/run_showcase.py tiling_lab/tests/test_run_showcase_tiles.py
git commit -m "fix(tiling): align recorded tiles with the detections they produced"
```

---

## Task 3: Callback-side full-frame NMS over the carried-forward union (with aging)

**Files:**
- Create: `hailo_tiling/dynamic/nms.py` (`nms_merge`)
- Modify: `hailo_tiling/dynamic/persistence.py` (add `age` to stored detections)
- Modify: `tiling_lab/live/controller.py` (`step_showcase` publishes `nms_merge([target] + persisted)`)
- Test: `hailo_tiling/tests/test_nms_merge.py` (new), `tiling_lab/tests/test_showcase_controller.py`

**Design (Option B — chosen over injecting into the GStreamer aggregator):** the `hailotileaggregator` keeps doing tile-local flatten + **within-frame** NMS. The **cross-frame** merge happens in the Python callback: each frame we run one greedy NMS over the union of the live `target` box and the carried-forward persisted detections. Each persisted detection carries an `age` (frames since it was last detected); on overlap NMS keeps the preferred box by `target` first, then **lowest age (freshest)**, then highest confidence. This removes the target's stale "friend," collapses a moving car's trail of stale boxes, and dedups dense-vs-dense — all in one principled pass. `DetectionPersistence` still bounds each box's lifetime to its cell's next sweep (preserving "saved until the tile's next iteration"); `age` adds decay-based preference and a TTL backstop.

### 3a. `nms_merge` helper

- [ ] **Step 1: Write the failing test**

```python
# hailo_tiling/tests/test_nms_merge.py
from hailo_tiling.dynamic.nms import nms_merge


def _d(label, x, y, w=0.05, h=0.04, conf=0.9, age=0):
    return {"label": label, "confidence": conf, "bbox": [x, y, w, h], "age": age}


def test_target_wins_over_overlapping_stale_box():
    target = _d("target", 0.50, 0.50, age=0)
    stale = _d("vehicle", 0.505, 0.505, age=12)         # same car, old, overlaps
    out = nms_merge([target, stale], iou_thresh=0.3)
    assert len(out) == 1 and out[0]["label"] == "target"


def test_fresher_box_wins_when_neither_is_target():
    fresh = _d("vehicle", 0.50, 0.50, age=0)
    stale = _d("vehicle", 0.505, 0.505, age=15)
    out = nms_merge([stale, fresh], iou_thresh=0.3)
    assert len(out) == 1 and out[0]["age"] == 0          # freshest survives


def test_non_overlapping_boxes_all_survive():
    a = _d("vehicle", 0.10, 0.10)
    b = _d("vehicle", 0.80, 0.80)
    out = nms_merge([a, b], iou_thresh=0.3)
    assert len(out) == 2
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && python -m pytest hailo_tiling/tests/test_nms_merge.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'hailo_tiling.dynamic.nms'`.

- [ ] **Step 3: Implement `nms_merge`**

```python
# hailo_tiling/dynamic/nms.py
from __future__ import annotations

__all__ = ["nms_merge"]


def _iou(a, b):
    ax, ay, aw, ah = a; bx, by, bw, bh = b
    ix1, iy1 = max(ax, bx), max(ay, by)
    ix2, iy2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = aw * ah + bw * bh - inter
    return inter / ua if ua > 0 else 0.0


def _priority(d):
    # Higher tuple sorts first: target beats non-target; then freshest (lowest
    # age); then highest confidence.
    return (1 if d.get("label") == "target" else 0,
            -d.get("age", 0),
            d.get("confidence", 0.0))


def nms_merge(detections, iou_thresh=0.3):
    """Greedy full-frame NMS over a union of detections (visualizer-schema dicts
    with an optional `age`). On overlap >= iou_thresh, keep the higher-priority
    box (target > freshest > most-confident) and drop the others."""
    kept = []
    for d in sorted(detections, key=_priority, reverse=True):
        if any(_iou(d["bbox"], k["bbox"]) >= iou_thresh for k in kept):
            continue
        kept.append(d)
    return kept
```

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && python -m pytest hailo_tiling/tests/test_nms_merge.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add hailo_tiling/dynamic/nms.py hailo_tiling/tests/test_nms_merge.py
git commit -m "feat(tiling): nms_merge — full-frame NMS with target/age/conf priority"
```

### 3b. Age the persisted detections

- [ ] **Step 1: Write the failing test**

```python
# Append to hailo_tiling/tests/test_persistence.py (create if absent)
from hailo_tiling.dynamic.persistence import DetectionPersistence


def _d(x, y):
    return {"label": "vehicle", "confidence": 0.9, "bbox": [x, y, 0.04, 0.03]}


def test_published_detections_carry_increasing_age():
    p = DetectionPersistence(dense_grid=(7, 6))
    cell = p.cell_of(_d(0.50, 0.50))
    p.update([cell], [_d(0.50, 0.50)])            # fresh: age 0
    assert p.published()[0]["age"] == 0
    p.tick()                                       # one frame passes
    p.tick()
    assert p.published()[0]["age"] == 2            # aged, not re-detected
    p.update([cell], [_d(0.50, 0.50)])            # re-detected: age resets
    assert p.published()[0]["age"] == 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && python -m pytest hailo_tiling/tests/test_persistence.py -v`
Expected: FAIL — `KeyError: 'age'` (or `AttributeError: 'DetectionPersistence' object has no attribute 'tick'`).

- [ ] **Step 3: Add `age` + `tick()` to `DetectionPersistence`**

```python
# hailo_tiling/dynamic/persistence.py — in update(), stamp age=0 on stored copies:
    def update(self, run_cells, dets) -> None:
        run = set(run_cells)
        for c in run:
            self._cells[c] = []
        for d in dets:
            c = self.cell_of(d)
            if c in run:
                self._cells[c].append({**d, "age": 0})   # fresh copy, age 0

    def tick(self) -> None:
        """Advance one frame: age every carried-forward detection."""
        for ds in self._cells.values():
            for d in ds:
                d["age"] = d.get("age", 0) + 1
```

(Call `tick()` once per frame from the controller — see 3c.)

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && python -m pytest hailo_tiling/tests/test_persistence.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add hailo_tiling/dynamic/persistence.py hailo_tiling/tests/test_persistence.py
git commit -m "feat(tiling): persisted detections carry an age for decay-aware NMS"
```

### 3c. Publish the NMS-merged union in `step_showcase`

- [ ] **Step 1: Write the failing test**

```python
# Append to tiling_lab/tests/test_showcase_controller.py
def test_seed_mode_nms_merges_stale_friend_into_target():
    ctrl = DynamicTilingController(3840, 2160, fps=60.0, budget_inf_per_s=600.0,
                                   striped=True, persist=True, dense_grid=(7, 6),
                                   grid_overlap=0.15, acquire_mode="seed")
    ctrl.seed((0.50, 0.50, 0.04, 0.04))
    real = _target_det(0.521, 0.518, w=0.05, h=0.035)
    real_dict = {"label": "vehicle", "confidence": 0.9,
                 "bbox": [0.521, 0.518, 0.05, 0.035]}
    far = _det_dict(0.05, 0.90, 0.06, 0.05, label="vehicle")
    # Run enough frames that the target's own detection is also persisted as a
    # dense vehicle (the "friend"), plus a far car that must survive.
    records = []
    for _ in range(30):
        _tiles, records = ctrl.step_showcase([real], [real_dict, far])
    near = [r for r in records
            if abs(r["bbox"][0]-0.521) < 0.03 and abs(r["bbox"][1]-0.518) < 0.03]
    assert len(near) == 1 and near[0]["label"] == "target"   # friend merged away
    assert any(abs(r["bbox"][0]-0.05) < 0.03 for r in records)  # far car kept
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && python -m pytest tiling_lab/tests/test_showcase_controller.py::test_seed_mode_nms_merges_stale_friend_into_target -v`
Expected: FAIL — two boxes near the target (live `target` + persisted `vehicle`).

- [ ] **Step 3: Replace the persisted-record loop with the NMS-merged union**

```python
# tiling_lab/live/controller.py
# top of file:
from hailo_tiling.dynamic.nms import nms_merge

# inside step_showcase, replace the `if self._persist is not None:` block with:
        if self._persist is not None:
            self._persist.update(self._sched.stripe_indices(self._frame),
                                 list(all_dets))
            self._persist.tick()
            union = list(records) + self._persist.published()   # target + carried
            records = nms_merge(union, iou_thresh=0.3)
```

(Delete the now-unused `_iou`/`_SOT_DEDUP_IOU` dedup helper from `controller.py`. The `target` record added earlier in `step_showcase` has no `age` key — `nms_merge` treats missing age as 0, i.e. freshest, which is correct.)

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && python -m pytest tiling_lab/tests/test_showcase_controller.py -v`
Expected: PASS. (The pre-existing `test_step_showcase_dedups_persisted_target_box` still holds: the overlapping persisted box is NMS-merged into the single `target` record.)

- [ ] **Step 5: Commit**

```bash
git add tiling_lab/live/controller.py tiling_lab/tests/test_showcase_controller.py
git commit -m "fix(tiling): publish callback-side NMS-merged union so target has no stale friend"
```

---

## Task 4: Recolour the tracked target box distinctly in the viewer/export

**Files:**
- Modify: `tiling_lab/viewer/overlay_viewer.py` (label→colour map)
- Test: `tiling_lab/tests/test_viewer_colors.py` (new)

The viewer must draw the `target` box in a distinct colour from the dense `vehicle`/`person` detections. Locate the existing label→colour logic in `overlay_viewer.py` (search for where box colours are chosen) and add an explicit `target` entry.

- [ ] **Step 1: Write the failing test**

```python
# tiling_lab/tests/test_viewer_colors.py
from tiling_lab.viewer.overlay_viewer import color_for_label


def test_target_has_distinct_color():
    target = color_for_label("target")
    vehicle = color_for_label("vehicle")
    person = color_for_label("person")
    assert target != vehicle and target != person
    # 3-tuple BGR/RGB in 0..255
    assert len(target) == 3 and all(0 <= c <= 255 for c in target)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && python -m pytest tiling_lab/tests/test_viewer_colors.py -v`
Expected: FAIL with `ImportError: cannot import name 'color_for_label'` (or AssertionError if a differently-named function exists — then rename the test import to match and assert the `target` distinctness).

- [ ] **Step 3: Add an explicit colour function used by the draw path**

```python
# tiling_lab/viewer/overlay_viewer.py — add (and call it from the box-draw code)
_LABEL_COLORS = {
    "target": (0, 0, 255),       # red — the tracked SOT, stands out
    "vehicle": (0, 200, 0),      # green — dense detections
    "person": (255, 160, 0),     # orange
}
_DEFAULT_COLOR = (200, 200, 200)


def color_for_label(label):
    """BGR colour for a detection label; the tracked `target` is always red."""
    return _LABEL_COLORS.get(label, _DEFAULT_COLOR)
```

Then replace the inline colour choice in the box-drawing loop with `color_for_label(det["label"])`. (Verify the channel order matches the existing draw calls — if the viewer draws RGB rather than BGR, swap the `target` tuple accordingly so it renders red.)

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && python -m pytest tiling_lab/tests/test_viewer_colors.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tiling_lab/viewer/overlay_viewer.py tiling_lab/tests/test_viewer_colors.py
git commit -m "feat(tiling): draw tracked target box in a distinct colour"
```

---

## Task 5: Seed = user selection that snaps to a real dense detection (tight gate)

**Files:**
- Modify: `tiling_lab/harness/target_lock.py` (`SeedTracker`)
- Test: `tiling_lab/tests/test_seed_tracker.py`

**Design:** the seed emulates a user clicking an object that the **dense grid has already detected**. So the seed must (a) bind to the nearest real detection within a **tight, non-growing selection radius** (it can only pick a box right at the click, never a neighbour 0.15 away), and (b) never emit a synthetic box. Once bound it tracks the real bbox exactly as today (the bound ROI then zooms via `_roi`, which is what made frames 28+ detect). The runner applies seeds at frames chosen *because the dense grid detected the target there* (see Task 6).

Currently `SeedTracker` uses `acq_radius=0.10` plus `radius_growth * frames_since_seen` *before* binding — so an un-bound seed's gate balloons and can grab a far car. Split the gate: tight + fixed before binding, growing only after binding (for re-acquisition through brief misses).

- [ ] **Step 1: Write the failing test**

```python
# Append to tiling_lab/tests/test_seed_tracker.py
def test_seed_binds_immediately_to_detection_at_selection():
    st = SeedTracker()
    st.seed((0.50, 0.30, 0.04, 0.04))             # selection centre (0.52, 0.32)
    real = _det(0.518, 0.317, 0.03, 0.02)         # dense detection right there
    assert st.step([real]) is True                # binds on the selection frame
    assert st.state.bbox_norm == (0.518, 0.317, 0.03, 0.02)


def test_seed_tight_gate_never_grabs_far_neighbour_even_after_waiting():
    st = SeedTracker()
    st.seed((0.50, 0.30, 0.04, 0.04))             # centre (0.52, 0.32)
    far = _det(0.64, 0.27, 0.03, 0.02)            # a different car ~0.15 away
    for _ in range(40):                            # wait many frames
        assert st.step([far]) is False             # gate must NOT grow to reach it
    assert st.state.bbox_norm[2] == 0.04 or st.state.status in ("TRACKING", "LOST")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && python -m pytest tiling_lab/tests/test_seed_tracker.py::test_seed_tight_gate_never_grabs_far_neighbour_even_after_waiting -v`
Expected: FAIL — the pre-bind gate grows with `frames_since_seen` and eventually binds the far car.

- [ ] **Step 3: Make the pre-bind selection gate tight and non-growing**

```python
# tiling_lab/harness/target_lock.py — SeedTracker.__init__ signature:
    def __init__(self, *, track_buffer: int = 90, select_radius: float = 0.04,
                 track_radius: float = 0.06, radius_growth: float = 0.004):
        self.track_buffer = track_buffer
        self.select_radius = select_radius   # tight, FIXED gate for the click-to-select
        self.track_radius = track_radius     # gate once bound
        self.radius_growth = radius_growth   # growth per consecutive miss, POST-bind only
```

```python
# tiling_lab/harness/target_lock.py — SeedTracker.step(): replace the gate calc
        pcx, pcy = self._pred_center
        if self._bound:
            gate = self.track_radius + self.radius_growth * s.frames_since_seen
        else:
            gate = self.select_radius            # fixed: select only a box AT the click
```

(Remove the old `base = self.track_radius if self._bound else self.acq_radius` / `gate = base + ...` lines. `acq_radius` is gone; no other code references it.)

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && python -m pytest tiling_lab/tests/test_seed_tracker.py -v`
Expected: PASS (all SeedTracker tests, including the pre-existing ones).

- [ ] **Step 5: Commit**

```bash
git add tiling_lab/harness/target_lock.py tiling_lab/tests/test_seed_tracker.py
git commit -m "fix(tiling): tight non-growing seed selection gate (click-to-track on a dense detection)"
```

---

## Task 5b: Click-to-acquire via pyramid (zoom) search — finds the target even before dense detects it

**Files:**
- Create: `hailo_tiling/dynamic/acquisition.py` (`pyramid_crops`)
- Modify: `tiling_lab/harness/target_lock.py` (`SeedTracker.bound` property)
- Modify: `tiling_lab/live/controller.py` (`seed(mode=...)`; emit pyramid while click-acquiring)
- Modify: `tiling_lab/live/run_showcase.py` (`--select-mode`)
- Test: `hailo_tiling/tests/test_acquisition.py` (new), `tiling_lab/tests/test_showcase_controller.py`

**Design:** a `click` seed is a raw screen location — the user clicks where they *see* the object, before the dense grid has detected it. While un-bound, the controller emits a **pyramid of tiles centred on the click at decreasing crop widths** (native 640 → progressively zoomed, e.g. 640, 448, 320, 224). A tiny far car that is invisible at native (proven: frames 1–27) is magnified by a zoomed pyramid level and detected. The `SeedTracker` then binds to the detection nearest the click (tight gate from Task 5) and tracking proceeds; once bound the pyramid stops and the normal tracking ROI takes over. The pyramid is a transient burst the Task-1 budget absorbs. Dense tiles stay native; zoom is used only transiently to *acquire* (and by `_roi` to track), per the user's "native dense grid" decision.

### 5b-i. `pyramid_crops` helper

- [ ] **Step 1: Write the failing test**

```python
# hailo_tiling/tests/test_acquisition.py
from hailo_tiling.dynamic.acquisition import pyramid_crops


def test_pyramid_has_increasing_zoom_centred_on_click():
    crops = pyramid_crops(0.5, 0.5, 3840, 2160, widths=(640, 320))
    assert len(crops) == 2
    assert abs(crops[0].scale - 1.0) < 1e-6        # native level
    assert abs(crops[1].scale - 2.0) < 1e-6        # 2x zoom level
    for c in crops:                                 # all centred on the click
        assert abs((c.x + c.w / 2) / 3840 - 0.5) < 0.01
        assert abs((c.y + c.h / 2) / 2160 - 0.5) < 0.01
    assert all(c.mode == "s" for c in crops)


def test_pyramid_skips_widths_larger_than_source():
    crops = pyramid_crops(0.5, 0.5, 600, 480, widths=(640, 320))
    assert [c.w for c in crops] == [320]            # 640 > src_w → skipped
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && python -m pytest hailo_tiling/tests/test_acquisition.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'hailo_tiling.dynamic.acquisition'`.

- [ ] **Step 3: Implement `pyramid_crops`**

```python
# hailo_tiling/dynamic/acquisition.py
from __future__ import annotations

from ..types import CropRect

__all__ = ["pyramid_crops", "DEFAULT_PYRAMID_WIDTHS"]

# Native 640 down to ~2.9x digital zoom — the zoom regime where zoom_probe.py
# shows small targets become detectable before upscale blur dominates.
DEFAULT_PYRAMID_WIDTHS = (640, 448, 320, 224)


def pyramid_crops(cx_norm, cy_norm, src_w, src_h, widths=DEFAULT_PYRAMID_WIDTHS):
    """A pyramid of single-scale crops centred on a normalized click point, at
    decreasing source-pixel widths (native -> increasing zoom). Lets the detector
    find an object at the click at whatever scale works — even a tiny far target
    a native crop misses. Widths larger than the source are skipped."""
    cx = cx_norm * src_w
    cy = cy_norm * src_h
    out = []
    for w in widths:
        if w > src_w:
            continue
        out.append(CropRect.from_center_width(cx, cy, int(w), mode="s").clamp(src_w, src_h))
    return out
```

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && python -m pytest hailo_tiling/tests/test_acquisition.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add hailo_tiling/dynamic/acquisition.py hailo_tiling/tests/test_acquisition.py
git commit -m "feat(tiling): pyramid_crops — multi-zoom acquisition crops centred on a click"
```

### 5b-ii. Emit the pyramid while click-acquiring (controller)

- [ ] **Step 1: Write the failing test**

```python
# Append to tiling_lab/tests/test_showcase_controller.py
from hailo_tiling.dynamic.acquisition import DEFAULT_PYRAMID_WIDTHS


def test_click_seed_emits_pyramid_before_detection_then_stops_after_bind():
    ctrl = DynamicTilingController(3840, 2160, fps=60.0, budget_inf_per_s=600.0,
                                   striped=True, persist=True, dense_grid=(7, 6),
                                   grid_overlap=0.15, acquire_mode="seed")
    ctrl.seed((0.50, 0.30, 0.04, 0.04), mode="click")    # raw click, nothing detected yet
    # No detection yet: the pyramid (multiple "s" tiles at the click) must be emitted.
    tiles, _ = ctrl.step_showcase([], [])
    dyn = [seg for seg in tiles.split(";") if seg.endswith(",s")]
    assert len(dyn) >= len([w for w in DEFAULT_PYRAMID_WIDTHS if w <= 3840])
    # Now a zoomed level detects the car at the click → bind → pyramid stops.
    real = _target_det(0.518, 0.317, w=0.03, h=0.02)
    ctrl.step_showcase([real], [{"label": "vehicle", "confidence": 0.9,
                                 "bbox": [0.518, 0.317, 0.03, 0.02]}])
    tiles2, _ = ctrl.step_showcase([real], [{"label": "vehicle", "confidence": 0.9,
                                             "bbox": [0.518, 0.317, 0.03, 0.02]}])
    dyn2 = [seg for seg in tiles2.split(";") if seg.endswith(",s")]
    assert len(dyn2) == 1                                 # single tracking ROI, no pyramid
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && python -m pytest tiling_lab/tests/test_showcase_controller.py::test_click_seed_emits_pyramid_before_detection_then_stops_after_bind -v`
Expected: FAIL — `seed()` takes no `mode` arg (TypeError) / no pyramid emitted.

- [ ] **Step 3: Add `bound` to `SeedTracker`, `mode` to `controller.seed`, and pyramid emission**

```python
# tiling_lab/harness/target_lock.py — add to SeedTracker:
    @property
    def bound(self) -> bool:
        return self._bound
```

```python
# tiling_lab/live/controller.py
from hailo_tiling.dynamic.acquisition import pyramid_crops

# in __init__: self._select_mode = "detection"; self._click = None

    def seed(self, bbox_norm, mode: str = "detection") -> None:
        """Seed the target. mode='detection' snaps to an existing nearby
        detection (tight gate); mode='click' runs a zoom pyramid at the click
        until a detection is found (works before dense has detected it)."""
        self._select_mode = mode
        self._click = (bbox_norm[0] + bbox_norm[2] / 2.0,
                       bbox_norm[1] + bbox_norm[3] / 2.0)
        self._lock.seed(bbox_norm)

# in step_showcase, AFTER `crops = self._sched.decide(...)` and BEFORE charging:
        if (self._seed_mode and self._select_mode == "click"
                and self._click is not None and not self._lock.bound):
            dense = [c for c in crops if c.mode == "m"]
            crops = pyramid_crops(self._click[0], self._click[1],
                                  self.src_w, self.src_h) + dense
```

(The pyramid replaces the single pre-bind ROI; dense tiles are preserved. `self._meter.charge(len(crops), ...)` then accounts for the burst.)

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && python -m pytest tiling_lab/tests/test_showcase_controller.py tiling_lab/tests/test_seed_tracker.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tiling_lab/harness/target_lock.py tiling_lab/live/controller.py tiling_lab/tests/test_showcase_controller.py
git commit -m "feat(tiling): click-seed runs a zoom pyramid until the target is acquired"
```

### 5b-iii. `--select-mode` flag in the runner

- [ ] **Step 1: Write the failing test**

```python
# Append to hailo_tiling/tests/test_run_showcase_tiles.py (argparse smoke)
from tiling_lab.live.run_showcase import build_arg_parser


def test_select_mode_flag_defaults_to_click():
    args = build_arg_parser().parse_args(["--video", "x", "--out", "y"])
    assert args.select_mode == "click"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && python -m pytest hailo_tiling/tests/test_run_showcase_tiles.py::test_select_mode_flag_defaults_to_click -v`
Expected: FAIL — `build_arg_parser` not exported / `--select-mode` unknown.

- [ ] **Step 3: Add the flag and thread it into `ctrl.seed`**

Refactor the `argparse` setup in `run_showcase.main()` into `build_arg_parser()` (returns the parser) so it is testable, then add:

```python
    ap.add_argument("--select-mode", default="click", choices=["click", "detection"],
                    help="how a --seed acts: 'click' runs a zoom pyramid at the "
                         "location until the target is found (works before dense "
                         "detects it); 'detection' snaps to an existing nearby "
                         "detection (tight gate, no zoom).")
```

In the probe, pass the mode when seeding:

```python
        if state["frame"] in seeds_by_frame:
            ctrl.seed(seeds_by_frame[state["frame"]], mode=args.select_mode)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && python -m pytest hailo_tiling/tests/test_run_showcase_tiles.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tiling_lab/live/run_showcase.py hailo_tiling/tests/test_run_showcase_tiles.py
git commit -m "feat(tiling): --select-mode {click,detection} for the showcase seed"
```

---

## Task 6: Re-run the showcase, render the demo video, verify realtime + all requirements

**Files:**
- Output: `tiling_lab/runs/showcase_0007/frames.json`, `tiling_lab/runs/showcase_0007/showcase_0007.mp4`

This is the integration + verification task. No new product code; it proves Tasks 1–5 deliver what the user asked for.

- [ ] **Step 1a: Dense-only pass to discover where the target is detected (seed-frame selection)**

Per the user's flow, seeds are applied at frames where the dense grid has ALREADY detected the target. First run with NO seed so only the dense grid produces detections:

```bash
source setup_env.sh
python -m tiling_lab.live.run_showcase \
  --video /home/giladn/Videos/Drone/Training/Car/DJI_20260614160255_0007_D.MP4 \
  --out tiling_lab/runs/showcase_0007_dense --fps 60 --target-class vehicle \
  --acquire-mode largest   # no seed; dense grid + auto-lock only, just to surface dense dets
```

Then identify candidate (frame, x, y) selections — the first dense `vehicle` detection on each trail of interest. Either pick them by eye in the viewer (true user-selection emulation):

```bash
DISPLAY=:1 XAUTHORITY=/run/user/10615/gdm/Xauthority \
python -m tiling_lab.viewer.overlay_viewer \
  --video /home/giladn/Videos/Drone/Training/Car/DJI_20260614160255_0007_D.MP4 \
  --frames tiling_lab/runs/showcase_0007_dense/frames.json:dense --conf-min 0.1
```

…or list them programmatically (first dense detection in a region/time window):

```bash
source setup_env.sh
python - <<'PY'
import json
F = json.load(open("tiling_lab/runs/showcase_0007_dense/frames.json"))["frames"]
for f in F:
    for r in f["detections"]:
        if r["label"] == "vehicle":
            b = r["bbox"]
            print(f"frame {f['frame']:>4}  center=({b[0]+b[2]/2:.4f},{b[1]+b[3]/2:.4f})  conf={r['confidence']:.2f}")
PY
```

Record the chosen `FRAME:x,y,w,h` selections (centre on the detection; `w,h` small, e.g. 0.04) for Step 1b. **Confirm the selections with the user before the seeded run** — they emulate the operator's clicks.

- [ ] **Step 1b: Re-run the live showcase WITH the chosen selections (records frames.json, validates realtime)**

With `--select-mode click` (default), seeds can be placed at the moment the operator *sees* the car — including frame 0 — and the zoom pyramid finds it even before dense detection. The original frame-0/1878 clicks therefore work directly:

```bash
source setup_env.sh
python -m tiling_lab.live.run_showcase \
  --video /home/giladn/Videos/Drone/Training/Car/DJI_20260614160255_0007_D.MP4 \
  --out tiling_lab/runs/showcase_0007 --fps 60 --target-class vehicle \
  --acquire-mode seed --select-mode click \
  --seed 0:0.4688,0.2957,0.04,0.04 \
  --seed 1878:0.0876,0.5795,0.04,0.04
```

(For `--select-mode detection`, use the (frame, x, y) confirmed in Step 1a instead.) Expected stdout tail: `sustains_60fps=True`. Note `max_t/f` will spike during the brief click-acquisition pyramid (≈ ROI-replacement + dense ≈ 6) then settle to 3 once bound; the burst is transient and must not drop sustained fps below 60.

- [ ] **Step 2: Assert the fixes landed in the recorded data**

```bash
source setup_env.sh
python - <<'PY'
import json
from collections import Counter
F = json.load(open("tiling_lab/runs/showcase_0007/frames.json"))["frames"]
dense = Counter(len([t for t in f["tiles"] if t["category"]=="multi-scale"]) for f in F)
tracking = [f for f in F if any(r["label"]=="target" for r in f["detections"])]
dense_when_tracking = Counter(
    len([t for t in f["tiles"] if t["category"]=="multi-scale"]) for f in tracking)
print("dense/frame overall:", dict(sorted(dense.items())))
print("dense/frame while TRACKING:", dict(sorted(dense_when_tracking.items())))
# No frame should have a 'target' AND a second box overlapping it (no stale friend)
def iou(a,b):
    ax,ay,aw,ah=a; bx,by,bw,bh=b
    ix1,iy1=max(ax,bx),max(ay,by); ix2,iy2=min(ax+aw,bx+bw),min(ay+ah,by+bh)
    iw,ih=max(0,ix2-ix1),max(0,iy2-iy1); inter=iw*ih
    ua=aw*ah+bw*bh-inter; return inter/ua if ua>0 else 0
friends=0
for f in F:
    tg=[r["bbox"] for r in f["detections"] if r["label"]=="target"]
    ot=[r["bbox"] for r in f["detections"] if r["label"]!="target"]
    for t in tg:
        if any(iou(t,o)>0.3 for o in ot): friends+=1
print("frames with a duplicate box overlapping the target:", friends)
assert dense_when_tracking and min(dense_when_tracking)==2, "dense load still trimmed"
assert friends==0, "stale duplicate boxes near target remain"
print("OK: 2 dense tiles every tracking frame; no stale friend near target")
PY
```

Expected: prints `OK: ...`; assertions pass.

- [ ] **Step 3: Render the polished demo video (recoloured target, tiles, persisted detections)**

```bash
source setup_env.sh
python -m tiling_lab.viewer.overlay_viewer \
  --video /home/giladn/Videos/Drone/Training/Car/DJI_20260614160255_0007_D.MP4 \
  --frames tiling_lab/runs/showcase_0007/frames.json:showcase \
  --export tiling_lab/runs/showcase_0007/showcase_0007.mp4 \
  --export-tiles showcase --export-fps 60 --conf-min 0.1
```

Expected: writes `showcase_0007.mp4`; the `target` box renders red, dense detections green, tiles drawn, boxes persist between sweeps.

- [ ] **Step 4: Full test suite green**

Run: `source setup_env.sh && python -m pytest hailo_tiling/tests tiling_lab/tests -q`
Expected: all pass (≈ 510+ tests).

- [ ] **Step 5: Commit the run artifacts metadata (not the mp4 unless requested)**

```bash
git add tiling_lab/runs/showcase_0007/metrics.json
git commit -m "chore(tiling): showcase_0007 metrics after flat-load + alignment + colour fixes"
```

(Confirm with the user before committing large binaries like the `.mp4` or `frames.json`.)

---

## Self-Review

**Spec coverage:**
- "Track from the first frame / see the car at frame 0" → Task 5b: a `click` seed runs a zoom **pyramid** at the location, so a tiny far car invisible at native is magnified and detected even before dense finds it; the tight gate (Task 5) then binds the real bbox.
- "Seed emulates a user selection applied after dense detection" → Task 5 (tight-gate snap to an existing detection) and Task 6 Step 1a (choose seed frames from a dense-only pass). The two acquisition modes are selectable via `--select-mode {click,detection}` (Task 5b-iii).
- "Show all detections incl. dense, saved until the tile's next iteration" → `DetectionPersistence` carry-forward preserved (cell re-sweep bounds lifetime); Task 3c's `nms_merge` only collapses *overlapping* duplicates, so non-overlapping persisted boxes all survive (verified in Task 6 Step 2 by the far-car survival).
- "NMS should be merged between dynamic and dense tiles; dynamic tile updates every frame" → within-frame merge stays in `hailotileaggregator` (iou 0.3); the **cross-frame** merge is the callback-side `nms_merge` over target + aged persisted detections (Task 3, Option B), preferring target > freshest > confidence; ROI runs every frame by design.
- "Not running 2 dense tiles, skips one" → Task 1 (budget no longer trims), verified in Task 6 Step 2.
- "Tiles per frame look like (1),(3),(5)…" → caused by both the trim (Task 1) and the recording misalignment (Task 2); both fixed.
- "Target bbox in a distinct colour" → Task 4.
- "Sustains realtime 60fps" + "polished demo video" → Task 6 Steps 1 & 3.

**Placeholder scan:** none — every step has concrete code/commands.

**Type consistency:** `nms_merge(detections, iou_thresh=0.3)`, `DetectionPersistence.tick()`, `color_for_label`, `build_frame_record`, `initial_tiles_for_seed` defined where first used; persisted detections are visualizer-schema dicts with an added `age` key consumed by `nms_merge._priority`; `CropRect.from_center_width`/`.clamp`/`.scale` and `MODEL_W` match `hailo_tiling/types.py`; `stripe_indices`/`decide` signatures match `striped.py`; `BudgetMeter(budget_inf_per_s, fps)` matches `hailo_tiling/budget.py`.
