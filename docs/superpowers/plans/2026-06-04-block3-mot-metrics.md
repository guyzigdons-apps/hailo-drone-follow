# Block 3 — MOT Metrics + First Multi-Target Scorecard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** In-repo MOT metrics (IDF1, MOTA, ID switches, Frag, MT/ML) scored per frame via greedy IoU matching, applied to a multi-target dynamic run on 0026-fov50 vs its verified GT, with an equal-budget dense-static comparison row → `MOT_BASELINE.md`.

**Architecture:** `mot_metrics.py` is pure Python over `{frame: {track_id: bbox}}` dicts — no chip, no new deps (greedy matching; IDF1 identity assignment via greedy-on-counts). `run_multi` gains per-track recording (`multi_traj`); `run_dynamic --multi-target` gains `--dump-mot` to persist predictions; `run_mot_eval.py` scores a predictions file against GT.

**Tech Stack:** Python, pytest. CPU-only until the final scored run — **this plan's Tasks 1–3 can run in parallel with Block 2's chip time.** Spec: `docs/superpowers/specs/2026-06-04-weekend-recovery-reid-sweep-mot-design.md`.

---

### Task 1: mot_metrics.py — matching + MOTA family

**Files:**
- Create: `dynamic_tiling/mot_metrics.py`
- Test: `dynamic_tiling/tests/test_mot_metrics.py`

- [ ] **Step 1: Failing tests**

```python
def _traj(tid, frames, x0=0.1, dx=0.0, y=0.5, w=0.05, h=0.1):
    return {f: (x0 + dx * i, y, w, h) for i, f in enumerate(frames)}


def test_perfect_tracking_scores_perfectly():
    from dynamic_tiling.mot_metrics import score_mot
    gt = {1: _traj(1, range(10)), 2: _traj(2, range(10), x0=0.7)}
    pred = {10: _traj(10, range(10)), 20: _traj(20, range(10), x0=0.7)}
    m = score_mot(gt, pred, iou_thr=0.5)
    assert m["MOTA"] == 1.0 and m["IDF1"] == 1.0
    assert m["IDsw"] == 0 and m["FP"] == 0 and m["FN"] == 0
    assert m["MT"] == 2 and m["ML"] == 0


def test_identity_swap_costs_idsw_and_idf1():
    from dynamic_tiling.mot_metrics import score_mot
    gt = {1: _traj(1, range(10))}
    pred = {10: _traj(10, range(5)),          # first half id 10
            11: {f: (0.1, 0.5, 0.05, 0.1) for f in range(5, 10)}}  # second half id 11
    m = score_mot(gt, pred, iou_thr=0.5)
    assert m["IDsw"] == 1
    assert m["MOTA"] < 1.0                     # IDsw penalised
    assert 0.4 < m["IDF1"] <= 0.6              # best identity covers half


def test_fp_flood_and_misses():
    from dynamic_tiling.mot_metrics import score_mot
    gt = {1: _traj(1, range(10))}
    pred = {10: _traj(10, range(6)),                       # 4 FN
            99: {f: (0.9, 0.9, 0.05, 0.1) for f in range(10)}}  # 10 FP
    m = score_mot(gt, pred, iou_thr=0.5)
    assert m["FN"] == 4 and m["FP"] == 10
    assert m["MOTA"] == 1 - (4 + 10 + 0) / 10


def test_fragmentation_counted():
    from dynamic_tiling.mot_metrics import score_mot
    gt = {1: _traj(1, range(12))}
    pred = {10: {f: (0.1, 0.5, 0.05, 0.1) for f in (0, 1, 2, 3, 6, 7, 8, 11)}}
    m = score_mot(gt, pred, iou_thr=0.5)
    assert m["Frag"] == 2                      # two gaps inside a matched GT track


def test_empty_pred_is_all_misses():
    from dynamic_tiling.mot_metrics import score_mot
    gt = {1: _traj(1, range(5))}
    m = score_mot(gt, {}, iou_thr=0.5)
    assert m["FN"] == 5 and m["MOTA"] == 0.0 and m["IDF1"] == 0.0 and m["ML"] == 1
```

- [ ] **Step 2: Run, expect FAIL (module missing).**

- [ ] **Step 3: Implement** `dynamic_tiling/mot_metrics.py`:

```python
"""Minimal in-repo MOT metrics: MOTA (FP/FN/IDsw), IDF1, Frag, MT/ML.

Inputs are {gt_id: {frame: (x,y,w,h)}} and {pred_id: {frame: (x,y,w,h)}}
(normalized). Per-frame association: GREEDY best-IoU matching above iou_thr
(deterministic: sort candidate pairs by IoU desc, then ids). IDF1 identity
assignment: greedy on global per-(gt,pred) match counts (Hungarian-free; exact
enough for our small id counts — documented limitation)."""
from __future__ import annotations


def _iou(a, b):
    ax2, ay2 = a[0] + a[2], a[1] + a[3]
    bx2, by2 = b[0] + b[2], b[1] + b[3]
    iw = max(0.0, min(ax2, bx2) - max(a[0], b[0]))
    ih = max(0.0, min(ay2, by2) - max(a[1], b[1]))
    i = iw * ih
    return i / (a[2] * a[3] + b[2] * b[3] - i) if i > 0 else 0.0


def _frames_union(*track_dicts):
    fs = set()
    for tracks in track_dicts:
        for traj in tracks.values():
            fs.update(traj)
    return sorted(fs)


def score_mot(gt: dict, pred: dict, *, iou_thr: float = 0.5) -> dict:
    frames = _frames_union(gt, pred)
    n_gt = sum(len(t) for t in gt.values())
    fp = fn = idsw = 0
    matches_prev: dict = {}            # gt_id -> pred_id matched on previous frame
    pair_counts: dict = {}             # (gt_id, pred_id) -> matched frames
    gt_matched_frames = {g: set() for g in gt}

    for f in frames:
        g_boxes = {g: traj[f] for g, traj in gt.items() if f in traj}
        p_boxes = {p: traj[f] for p, traj in pred.items() if f in traj}
        cand = sorted(
            ((_iou(gb, pb), g, p) for g, gb in g_boxes.items()
             for p, pb in p_boxes.items()),
            key=lambda t: (-t[0], t[1], t[2]))
        used_g, used_p, matches = set(), set(), {}
        for iou, g, p in cand:
            if iou < iou_thr or g in used_g or p in used_p:
                continue
            used_g.add(g); used_p.add(p); matches[g] = p
        fn += len(g_boxes) - len(matches)
        fp += len(p_boxes) - len(matches)
        for g, p in matches.items():
            pair_counts[(g, p)] = pair_counts.get((g, p), 0) + 1
            gt_matched_frames[g].add(f)
            if g in matches_prev and matches_prev[g] != p:
                idsw += 1
        # carry forward last known assignment for IDsw across gaps (MOT convention)
        matches_prev.update(matches)

    # Frag: gaps inside each GT track's matched-frame set
    frag = 0
    for g, traj in gt.items():
        fs = sorted(set(traj) & gt_matched_frames[g])
        track_fs = sorted(traj)
        in_run = False
        seen_first = False
        for f in track_fs:
            m = f in gt_matched_frames[g]
            if m and seen_first and not in_run:
                frag += 1
            if m:
                seen_first = True
            in_run = m
        _ = fs

    # MT/ML on matched-coverage ratio
    mt = ml = 0
    for g, traj in gt.items():
        r = len(gt_matched_frames[g]) / len(traj) if traj else 0.0
        mt += r >= 0.8
        ml += r <= 0.2

    # IDF1 — greedy identity assignment by global overlap counts
    idtp = 0
    pairs = sorted(pair_counts.items(), key=lambda kv: (-kv[1], kv[0]))
    used_g, used_p = set(), set()
    for (g, p), c in pairs:
        if g in used_g or p in used_p:
            continue
        used_g.add(g); used_p.add(p); idtp += c
    n_pred = sum(len(t) for t in pred.values())
    idf1 = (2 * idtp / (n_gt + n_pred)) if (n_gt + n_pred) else 0.0

    mota = 1.0 - (fn + fp + idsw) / n_gt if n_gt else 0.0
    return {"MOTA": mota, "IDF1": idf1, "IDsw": idsw, "FP": fp, "FN": fn,
            "Frag": frag, "MT": mt, "ML": ml,
            "n_gt": n_gt, "n_pred": n_pred, "n_frames": len(frames)}
```

- [ ] **Step 4: Run tests; iterate until ALL PASS** (the Frag walk and the
swap-IDF1 bound are the fiddly bits — fix implementation, not tests, unless a test
contradicts the documented MOT definition).
- [ ] **Step 5: Commit** — `feat(mot): in-repo MOT metrics (MOTA/IDF1/IDsw/Frag/MT-ML)`

---

### Task 2: Per-track recording in run_multi + --dump-mot

**Files:**
- Modify: `dynamic_tiling/replay.py` (`run_multi`), `dynamic_tiling/run_dynamic.py`
- Test: `dynamic_tiling/tests/test_replay_integration.py`

- [ ] **Step 1: Failing test** — in `run_multi`'s existing integration test setup
(see `test_replay_integration.py` / `test_multi_target_lock.py` for the fake
backend/scheduler pattern used there), assert the new field:

```python
# inside a run_multi test: after res = run_multi(...)
assert hasattr(res, "multi_traj")
# multi_traj: {frame_idx: {(cls, track_id): (x, y, w, h)}} for every CONFIRMED target
some_frame = max(res.multi_traj)
assert all(len(k) == 2 for k in res.multi_traj[some_frame])
```

- [ ] **Step 2: Implement** — add `multi_traj: dict = field(default_factory=dict)` to
`RunResult`; in `run_multi`'s per-frame loop record every current target:
`res.multi_traj[frame_idx] = {key: tuple(t.bbox_norm) for key, t in lock.targets.items() if t.bbox_norm[2] > 0}`.
In `run_dynamic`, add `--dump-mot PATH`: after a `--multi-target` run, write

```python
import json
by_track: dict = {}
for f, d in res.multi_traj.items():
    for (cls, tid), bb in d.items():
        if cls != 1:        # MOT scorecard is person-only unless told otherwise
            continue
        by_track.setdefault(tid, {})[f] = bb
json.dump({"tracks": {str(t): {str(f): list(b) for f, b in traj.items()}
                      for t, traj in by_track.items()}}, open(args.dump_mot, "w"))
```

(keep vehicles in `multi_traj` itself — the dump filter is the policy point; add a
`--dump-mot-classes 1,2` option, default `1`.)

- [ ] **Step 3: Suite green; commit** — `feat(mot): per-track multi_traj recording + run_dynamic --dump-mot`

---

### Task 3: run_mot_eval.py

**Files:**
- Create: `dynamic_tiling/run_mot_eval.py`
- Test: `dynamic_tiling/tests/test_run_mot_eval.py` (loader + score round-trip on tiny fixtures written to tmp_path)

- [ ] **Step 1:** CLI: `--gt-tracks <gt_tracks.verified.json> --pred <dump-mot json> --classes 1 --iou-thr 0.5 [--out report.json]`.
Loads GT (same format as `run_trials._load_tracks`, filter by `--classes`), loads pred,
calls `score_mot`, prints an aligned table and optionally writes JSON. TDD on fixtures.

- [ ] **Step 2: Commit** — `feat(mot): run_mot_eval CLI`

---

### Task 4: First scorecard — 0026-fov50 dynamic multi-target vs dense static (chip)

**Files:**
- Output: `dynamic_tiling/runs/MOT_BASELINE.md` (committed); run artifacts untracked

- [ ] **Step 1:** Dynamic run (after Block 2 finishes chip work):

```bash
python -m dynamic_tiling.run_dynamic \
  --video /home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4 \
  --multi-target --target-classes 0,1 --discovery-grid 8x6 --discovery-fps 2 \
  --budget 3000 --dump-mot dynamic_tiling/runs/mot/dyn_0026_fov50.json \
  --out dynamic_tiling/runs/mot/dyn_0026_fov50.frames.json
```

(Use the Block-2 winning grid/overlap if the flags exist on run_dynamic; otherwise note
the config used. Check `run_dynamic --help` first — its flag set may lag run_trials;
extend it with `--discovery-overlap` if missing, same plumb as run_trials.)

- [ ] **Step 2:** Static comparison: build per-frame "tracks" from the dense static
cache by running ByteTracker over the cached dense 8x6 detections (write a 30-line
helper in `run_mot_eval.py`: `--from-static-cache <db> --grid 8x6` that replays cached
dets per frame through `MultiTargetLock` — no chip needed). Equal budget note: dense
8x6@every-frame is 48 tiles/frame vs dynamic's measured tiles/frame — report both
tiles/frame columns; that asymmetry IS the headline.

- [ ] **Step 3:** Score both vs `dynamic_tiling/runs/gt_verify_0026_fov50/gt_tracks.verified.json`
(person tracks), render `MOT_BASELINE.md`: table (system, tiles/frame, MOTA, IDF1,
IDsw, FP, FN, Frag, MT/ML), repro commands, and 2-paragraph interpretation
(where dynamic loses ids vs static; connect to Block-1/R recovery findings).

- [ ] **Step 4: Commit** — `docs(mot): first multi-target MOT scorecard — dynamic vs dense static on 0026-fov50`
