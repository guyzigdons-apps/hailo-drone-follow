# Block 1 — Re-acquisition Motion Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix `TargetLock`'s frozen-anchor re-acquisition so moving targets can be re-locked after long losses (velocity-extrapolated anchor + time-growing distance gate).

**Architecture:** Two opt-in knobs on `TargetLock` (`reacq_motion`, `reacq_radius_growth`); the anchor becomes an internal, motion-updated state; the gate adds a centre-distance fallback after the existing IoU path. CLI plumb through `run_trials`. Validation replays the three known failing walker trials (tile caches are warm → near chip-free).

**Tech Stack:** Python, pytest, existing `dynamic_tiling` harness. Spec: `docs/superpowers/specs/2026-06-04-weekend-recovery-reid-sweep-mot-design.md`.

**Context for the implementer:**
- `dynamic_tiling/target_lock.py` — `TargetLock.step()` is the heart. Today: when the
  locked ByteTracker id vanishes, re-lock happens ONLY if a new activated track has
  `IoU(track, state.bbox_norm) > _REACQ_IOU`, and `state.bbox_norm` is FROZEN at the
  loss location. `state.last_velocity` (normalized cx,cy delta per frame) is already
  retained during loss.
- Proven failure (BASELINE.md): walker fov50-ov0 — activated track on the walker for
  135 frames during a 1243-frame loss, never adopted, anchor IoU 0.
- Run all tests with: `source hailo-apps/venv_hailo_apps/bin/activate && python -m pytest dynamic_tiling/tests/ -q`
- Git: branch `tiling-benchmark`, never push, commit per task.

---

### Task 1: Anchor state + velocity extrapolation in TargetLock

**Files:**
- Modify: `dynamic_tiling/target_lock.py`
- Test: `dynamic_tiling/tests/test_target_lock.py`

- [ ] **Step 1: Write the failing tests**

Append to `dynamic_tiling/tests/test_target_lock.py` (follow that file's existing
helpers for building `Det`s; a det is `hailo_tiling.types.Det(cls, score, x, y, w, h)`
normalized; `lock.step(dets)` drives the tracker — seed the lock the way existing tests
do, via `gt_bbox_norm=` on the first step):

```python
def _walk_dets(n, x0=0.10, dx=0.01, y=0.50, w=0.04, h=0.10, score=0.9):
    """Person walking right: one det per frame."""
    from hailo_tiling.types import Det
    return [Det(cls=1, score=score, x=x0 + i * dx, y=y, w=w, h=h) for i in range(n)]


def test_velocity_anchor_tracks_motion_during_loss():
    from dynamic_tiling.target_lock import TargetLock
    lock = TargetLock(frame_rate=30, reacq_motion="velocity")
    dets = _walk_dets(10)
    lock.step([dets[0]], gt_bbox_norm=(dets[0].x, dets[0].y, dets[0].w, dets[0].h))
    for d in dets[1:]:
        lock.step([d])                      # establishes velocity ~ (0.01, 0)
    a0 = lock.reacq_anchor
    for _ in range(10):
        lock.step([])                       # 10 lost frames, anchor should advance
    a1 = lock.reacq_anchor
    assert a1[0] - a0[0] > 0.05             # moved right ~10 * 0.01 (Kalman-smoothed, allow slack)
    assert abs(a1[1] - a0[1]) < 0.02        # no vertical drift


def test_frozen_mode_keeps_legacy_anchor():
    from dynamic_tiling.target_lock import TargetLock
    lock = TargetLock(frame_rate=30)        # default reacq_motion="frozen"
    dets = _walk_dets(10)
    lock.step([dets[0]], gt_bbox_norm=(dets[0].x, dets[0].y, dets[0].w, dets[0].h))
    for d in dets[1:]:
        lock.step([d])
    a0 = lock.reacq_anchor
    for _ in range(10):
        lock.step([])
    assert lock.reacq_anchor == a0          # frozen
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest dynamic_tiling/tests/test_target_lock.py -q`
Expected: FAIL — `TypeError: unexpected keyword 'reacq_motion'` / `AttributeError: reacq_anchor`.

- [ ] **Step 3: Implement anchor state**

In `TargetLock.__init__`, add params and state (keep existing params untouched):

```python
def __init__(self, track_buffer: int = 90, reacq_motion: str = "frozen",
             reacq_radius_growth: float = 0.0, **tracker_kwargs):
    ...existing body...
    self.reacq_motion = reacq_motion
    self.reacq_radius_growth = reacq_radius_growth
    self._anchor: tuple | None = None   # (x, y, w, h) normalized, motion-updated
```

In `step()`:
- whenever `cur is not None` (TRACKING branch): `self._anchor = tuple(cur.filtered_tlwh)`.
- in the lost branch (where `frames_since_seen` increments), BEFORE the reacq gate:

```python
if self._anchor is not None and self.reacq_motion == "velocity":
    vx, vy = self.state.last_velocity
    ax, ay, aw, ah = self._anchor
    self._anchor = (ax + vx, ay + vy, aw, ah)
```

(Note the ordering subtlety: the reacq gate runs before the status update in the
current code — restructure minimally so the anchor advance happens once per lost
frame before the gate evaluates. Keep `state.bbox_norm` behaviour EXACTLY as today —
other consumers (scheduler recovery placement, replay ANCHOR dump fallback) read it.)

Expose:

```python
@property
def reacq_anchor(self) -> tuple | None:
    return self._anchor if self._anchor is not None else \
        (tuple(self.state.bbox_norm) if self.state.bbox_norm[2] > 0 else None)
```

Switch the existing IoU reacq gate to compare against `self._anchor` instead of
`s.bbox_norm` (identical behaviour in frozen mode since the anchor equals the last
tracked bbox).

- [ ] **Step 4: Run tests**

Run: `python -m pytest dynamic_tiling/tests/test_target_lock.py -q` — Expected: PASS (all, including pre-existing).

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/target_lock.py dynamic_tiling/tests/test_target_lock.py
git commit -m "feat(reacq): motion-updated reacquisition anchor (reacq_motion=velocity)"
```

---

### Task 2: Time-growing centre-distance gate

**Files:**
- Modify: `dynamic_tiling/target_lock.py`
- Test: `dynamic_tiling/tests/test_target_lock.py`

- [ ] **Step 1: Write the failing tests**

```python
def test_radius_growth_relocks_distant_target():
    from dynamic_tiling.target_lock import TargetLock
    lock = TargetLock(frame_rate=30, reacq_radius_growth=0.005)
    dets = _walk_dets(10)
    lock.step([dets[0]], gt_bbox_norm=(dets[0].x, dets[0].y, dets[0].w, dets[0].h))
    for d in dets[1:]:
        lock.step([d])
    for _ in range(40):
        lock.step([])                       # long loss; radius grows 40*0.005=0.2
    # target reappears 0.15 away from the frozen anchor — IoU=0, distance inside radius
    from hailo_tiling.types import Det
    far = Det(cls=1, score=0.9, x=dets[-1].x + 0.15, y=0.50, w=0.04, h=0.10)
    st = None
    for _ in range(5):                      # let ByteTracker activate the new track
        st = lock.step([far])
    assert st.status == "TRACKING"


def test_radius_zero_keeps_strict_behavior():
    from dynamic_tiling.target_lock import TargetLock
    lock = TargetLock(frame_rate=30)        # growth 0.0
    dets = _walk_dets(10)
    lock.step([dets[0]], gt_bbox_norm=(dets[0].x, dets[0].y, dets[0].w, dets[0].h))
    for d in dets[1:]:
        lock.step([d])
    for _ in range(40):
        lock.step([])
    from hailo_tiling.types import Det
    far = Det(cls=1, score=0.9, x=dets[-1].x + 0.15, y=0.50, w=0.04, h=0.10)
    st = None
    for _ in range(5):
        st = lock.step([far])
    assert st.status != "TRACKING"          # legacy: never re-locks


def test_no_swap_when_distractor_is_outside_radius():
    from dynamic_tiling.target_lock import TargetLock
    from hailo_tiling.types import Det
    lock = TargetLock(frame_rate=30, reacq_radius_growth=0.002)
    d0 = Det(cls=1, score=0.9, x=0.10, y=0.50, w=0.04, h=0.10)
    lock.step([d0], gt_bbox_norm=(0.10, 0.50, 0.04, 0.10))
    for _ in range(9):
        lock.step([d0])
    for _ in range(5):                      # short loss: radius ~ r0 + 5*0.002
        lock.step([])
    distractor = Det(cls=1, score=0.9, x=0.80, y=0.20, w=0.04, h=0.10)  # far away
    st = None
    for _ in range(5):
        st = lock.step([distractor])
    assert st.status != "TRACKING"          # distant person must NOT be adopted
```

- [ ] **Step 2: Run to verify the first test fails** (`pytest ... -q`; expected: first test FAILS, others may pass)

- [ ] **Step 3: Implement the distance gate**

In `step()`, extend the reacq block: after the existing IoU pass finds nothing and
`self.reacq_radius_growth > 0`:

```python
if cur is None and self._bt_track_id is not None and self._anchor is not None \
        and self.reacq_radius_growth > 0:
    ax, ay, aw, ah = self._anchor
    acx, acy = ax + aw / 2, ay + ah / 2
    r0 = max(aw, ah)
    r = r0 + self.reacq_radius_growth * s.frames_since_seen
    best_d, best_track = r, None
    for t in tracks:
        if t.is_activated and t.filtered_tlwh and t.track_id != self._bt_track_id:
            tx, ty, tw, th = t.filtered_tlwh
            d = ((tx + tw / 2 - acx) ** 2 + (ty + th / 2 - acy) ** 2) ** 0.5
            if d < best_d:
                best_d, best_track = d, t
    if best_track is not None:
        self._bt_track_id = best_track.track_id
        cur = best_track
```

IoU path keeps priority (runs first, unchanged). Both paths now use `self._anchor`.

- [ ] **Step 4: Run the full suite** — `python -m pytest dynamic_tiling/tests/ -q` — Expected: ALL PASS.

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/target_lock.py dynamic_tiling/tests/test_target_lock.py
git commit -m "feat(reacq): time-growing centre-distance reacquisition gate (reacq_radius_growth)"
```

---

### Task 3: CLI plumb + replay ANCHOR dump uses the moving anchor

**Files:**
- Modify: `dynamic_tiling/run_trials.py`, `dynamic_tiling/trials.py`, `dynamic_tiling/replay.py`
- Test: `dynamic_tiling/tests/test_trials.py`

- [ ] **Step 1: Failing test** — append to `test_trials.py`:

```python
def test_reacq_knobs_reach_target_lock(monkeypatch):
    captured = {}
    import dynamic_tiling.trials as trials_mod

    class FakeLock:
        def __init__(self, **kw):
            captured.update(kw)
            self.track_id = None
            from dynamic_tiling.types import LockState
            self.state = LockState()

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj, person_cls=1):
        from dynamic_tiling.replay import RunResult
        r = RunResult(); r.n_frames = 1; return r

    monkeypatch.setattr(trials_mod, "TargetLock", FakeLock)
    monkeypatch.setattr(trials_mod, "run", fake_run)
    from dynamic_tiling.gt_clean import GtTrack
    run_all_trials(frames_factory=lambda: iter([]), src_w=1920, src_h=1080,
                   gt_tracks=[GtTrack(cls=1, track_id=1, frames={0: (0.1, 0.1, 0.2, 0.2)})],
                   backend_factory=lambda: _NullBackend(),
                   budget=300.0, fps=30.0, discovery_fps=2.0,
                   reacq_motion="velocity", reacq_radius_growth=0.002)
    assert captured["reacq_motion"] == "velocity"
    assert captured["reacq_radius_growth"] == 0.002
```

- [ ] **Step 2: Run to verify it fails** (unexpected kwargs in `run_all_trials`).

- [ ] **Step 3: Implement**

`trials.py`: add `reacq_motion="frozen", reacq_radius_growth=0.0` params to
`run_all_trials`, pass into `TargetLock(frame_rate=int(fps), reacq_motion=...,
reacq_radius_growth=...)`.

`run_trials.py`: add `--reacq-motion` (choices `frozen`/`velocity`, default `frozen`)
and `--reacq-radius-growth` (float, default 0.0); forward to `run_all_trials`; add both
to the `params` dict written by `--out`.

`replay.py`: in the tracker-debug block, prefer the moving anchor:

```python
anchor = getattr(lock, "reacq_anchor", None)
if state.status != "TRACKING" and anchor is not None:
    dbg["anchor"] = list(anchor)
```

(replacing the `state.bbox_norm`-based anchor line; keep the same `"anchor"` key).

- [ ] **Step 4: Full suite** — Expected: ALL PASS.

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/run_trials.py dynamic_tiling/trials.py dynamic_tiling/replay.py dynamic_tiling/tests/test_trials.py
git commit -m "feat(reacq): plumb reacq knobs through trials CLI; replay dumps the moving anchor"
```

---

### Task 4: Gate validation on the three known failures (chip/cache)

**Files:**
- Create: `dynamic_tiling/runs/block1_validation/` (results, untracked)
- Modify: `dynamic_tiling/runs/baseline_0025/BASELINE.md` (append validation section)

- [ ] **Step 1: Run a small reacq grid on the three failing cells** (caches are warm:
`dynamic_tiling/runs/cache/0025_fov{50,60,70}__yolov8n4c_vga.sqlite3`):

For each (fov, ov) in {(50, 0), (60, 0.25), (70, 0.25)} × each combo of
`--reacq-motion {frozen,velocity}` × `--reacq-radius-growth {0, 0.001, 0.002, 0.005}`:

```bash
source hailo-apps/venv_hailo_apps/bin/activate
python -m dynamic_tiling.run_trials \
  --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov<FOV>.mp4 \
  --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov<FOV>/gt_tracks.verified.json \
  --budget 3000 --fps 30 --discovery-fps 2 --discovery-grid 8x6 --max-zoom 2.0 \
  --discovery-overlap <OV> \
  --reacq-motion <M> --reacq-radius-growth <K> \
  --cache dynamic_tiling/runs/cache/0025_fov<FOV>__yolov8n4c_vga.sqlite3 \
  --out dynamic_tiling/runs/block1_validation/fov<FOV>_ov<OV>_<M>_k<K>.json
```

(24 runs; with warm caches each is minutes. Identical tile schedules to prior runs are
fully cache-served — recovery happening EARLIER changes subsequent ROI tiles, so some
chip misses are expected; that warms the caches further.)

- [ ] **Step 2: Check the gate** — for the best (M, K): walker (track 3) coverage > 0.8 on
ALL THREE cells, and the other six baseline cells (re-run them with the same best M,K)
show no metric regression > 0.02 coverage. Summarize per-cell coverage/losses/recovery
into the validation section.

- [ ] **Step 3: If the gate fails** — debug with a tagged dump (`--dump-frames`) on the
worst cell (the ANCHOR box now moves; verify it tracks the walker's path) before
touching code. Iterate Task 1/2 logic ONLY with a failing unit test reproducing the
issue.

- [ ] **Step 4: Append results table + chosen default recommendation to BASELINE.md, commit**

```bash
git add dynamic_tiling/runs/baseline_0025/BASELINE.md
git commit -m "docs(reacq): Block-1 validation — reacq grid on the three failing walker cells"
```

(Result JSONs stay untracked.)
