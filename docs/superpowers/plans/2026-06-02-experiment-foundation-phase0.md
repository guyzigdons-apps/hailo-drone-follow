# Experiment Foundation (Phase 0) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the reproducible evaluation foundation for the tile-scheduler experiments — a class-standardized inference path, a heavy-MOT ground-truth track builder, a tracking+recovery metric suite, and an all-objects multi-trial runner — so any tiling config can be scored identically.

**Architecture:** All inference goes through `hailo_tiling.backends.hef.HefBackend` with a class-offset so ids are person=1/vehicle=2 everywhere. A new offline BoT-SORT pass (`dynamic_tiling/gt_mot.py`) turns the dense 12×9 detector output into per-object GT trajectories. A pure-Python metric module (`dynamic_tiling/metrics.py`) scores follow coverage, drift, and recovery. A trial driver (`dynamic_tiling/trials.py`) runs one single-target follow per GT object (others as distractors) and aggregates.

**Tech Stack:** Python 3.10, pytest, numpy, OpenCV (`cv2` 4.10), `boxmot` (BoT-SORT, torch 2.6+cu124 available), `lap`+`scipy` (assignment/Kalman), HailoRT via `probe_phantom_hef`.

**Spec:** `docs/superpowers/specs/2026-06-02-tile-scheduler-experiment-design.md`

**Environment:** all commands assume `source setup_env.sh` has been run (activates `./hailo-apps/venv_hailo_apps`, exports `PYTHONPATH`). Run pytest as `python -m pytest`. Branch: `tiling-benchmark` (never main; never push).

**Scope guardrails (carry into every commit):**
- Do NOT stage pre-existing dirty files: `drone_follow/pipeline_adapter/reid_manager.py`, `sim/PX4-Autopilot`, `hailo-apps` (submodule pointer), `tiling_benchmark/overlay_viewer.py`, `.claude/scheduled_tasks.lock`, `HANDOFF.md`, `dynamic_tiling/runs/*.frames.json`. Stage only the files each task lists.
- Commit trailer on every commit: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.

---

## File Structure

| File | Responsibility | Created/Modified |
|------|----------------|------------------|
| `hailo_tiling/backends/hef.py` | add `class_offset` applied to decoded `Det.cls` | Modify |
| `dynamic_tiling/inference.py` | pass `class_offset` through the single-crop wrapper | Modify |
| `dynamic_tiling/replay.py` | default to person=1, filter dets to `TRACKED_CLASSES`, label tuple from `classes.LABELS` | Modify |
| `dynamic_tiling/gt_mot.py` | offline BoT-SORT over dense dets → raw per-object tracks (injectable tracker) | Create |
| `dynamic_tiling/gt_clean.py` | gap interpolation + quality filter → clean `GtTrack`s | Create |
| `dynamic_tiling/run_gt_tracks.py` | CLI: dense frames.json → gt_tracks.json + spot-check overlay | Create |
| `dynamic_tiling/metrics.py` | tracking+recovery metric suite (`score_trial`, `TrialScore`) | Create |
| `dynamic_tiling/trials.py` | per-object trial driver + aggregation (`run_all_trials`) | Create |
| `dynamic_tiling/run_trials.py` | CLI: video + gt_tracks → per-trial + aggregate metrics | Create |
| `dynamic_tiling/tests/test_*.py` | unit tests per module | Create |

---

## Task 1: Class-convention standardization (person=1, vehicle=2)

**Files:**
- Modify: `hailo_tiling/backends/hef.py`
- Modify: `dynamic_tiling/inference.py`
- Modify: `dynamic_tiling/replay.py:84-96` (label tuple), `:42` and `:141` (class filter)
- Test: `dynamic_tiling/tests/test_class_offset.py`

The on-chip `decode_nms_output` emits 0-indexed class ids (person=0); the dense GT and the rest of the project use person=1 (the label-file convention, see `hailo_tiling/classes.py`). Add a `class_offset` that shifts decoded ids so everything is person=1/vehicle=2.

- [ ] **Step 1: Write the failing test**

```python
# dynamic_tiling/tests/test_class_offset.py
from hailo_tiling.backends.hef import HefBackend
from hailo_tiling.types import CropRect, Det


def _fake_handle():
    class H:
        def infer(self, rgb):
            return "raw"
        def close(self):
            pass
    return H()


def test_class_offset_shifts_decoded_ids():
    # decode returns person=0 (raw NMS convention); offset must lift to person=1.
    def decode(_raw):
        return [Det(cls=0, score=0.9, x=0.1, y=0.1, w=0.2, h=0.3)]
    be = HefBackend(_fake_handle(), decode, class_offset=1)
    out = be.infer(frame=None, crops=[CropRect(0, 0, 64, 48)], frame_idx=0)
    assert out[0][0].cls == 1


def test_class_offset_defaults_to_zero():
    def decode(_raw):
        return [Det(cls=0, score=0.9, x=0.1, y=0.1, w=0.2, h=0.3)]
    be = HefBackend(_fake_handle(), decode)
    out = be.infer(frame=None, crops=[CropRect(0, 0, 64, 48)], frame_idx=0)
    assert out[0][0].cls == 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest dynamic_tiling/tests/test_class_offset.py -v`
Expected: FAIL — `HefBackend.__init__` got an unexpected keyword `class_offset` (test-injection branch ignores kwargs).

- [ ] **Step 3: Implement the offset in `hailo_tiling/backends/hef.py`**

Modify `__init__` to accept and store `class_offset`, and apply it in `_infer_one`. Replace the test-injection branch and add offset handling:

```python
    def __init__(self, *args, **kwargs):
        self._class_offset = kwargs.pop("class_offset", 0)
        # Test-injection mode: (handle, decode) positional, no remaining kwargs.
        if len(args) == 2 and not kwargs:
            handle, decode = args
            self._handle = handle
            self._decode = decode
            return
        hef_path = kwargs.get("hef_path") or (args[0] if args else None)
        nms_score_threshold = kwargs.get("nms_score_threshold", 0.25)
        if hef_path is None:
            raise TypeError("HefBackend requires hef_path or (handle, decode)")
        from probe_phantom_hef import HefHandle, decode_nms_output  # noqa: WPS433
        self._handle = HefHandle.open(hef_path, nms_score_threshold=nms_score_threshold)
        self._decode = decode_nms_output

    def _infer_one(self, frame, crop):
        import cv2  # noqa: WPS433
        sub = frame[crop.y:crop.y + crop.h, crop.x:crop.x + crop.w]
        resized = cv2.resize(sub, (MODEL_W, MODEL_H), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        dets = self._decode(self._handle.infer(rgb))
        if self._class_offset:
            dets = [Det(cls=d.cls + self._class_offset, score=d.score,
                        x=d.x, y=d.y, w=d.w, h=d.h) for d in dets]
        return dets
```

(`Det` and `MODEL_W/MODEL_H` are already imported at the top of the file.)

- [ ] **Step 4: Run the offset test to verify it passes**

Run: `python -m pytest dynamic_tiling/tests/test_class_offset.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Write the failing test for replay class filtering + person default**

```python
# add to dynamic_tiling/tests/test_class_offset.py
from hailo_tiling.classes import PERSON, TRACKED_CLASSES


def test_tracked_classes_excludes_face_and_plate():
    # Standing project rule: only person(1) + vehicle(2); never face(3)/plate(4).
    assert set(TRACKED_CLASSES) == {1, 2}
    assert PERSON == 1
```

- [ ] **Step 6: Run it**

Run: `python -m pytest dynamic_tiling/tests/test_class_offset.py::test_tracked_classes_excludes_face_and_plate -v`
Expected: PASS immediately (asserts existing constants — this pins the rule so a regression fails loudly).

- [ ] **Step 7: Update `dynamic_tiling/replay.py` defaults + filtering**

At the top of `replay.py` add `from hailo_tiling.classes import PERSON, LABELS, TRACKED_CLASSES`. Then:
- `run(...)`: change signature default `person_cls: int = PERSON` (was `0`), and after `dets = nms(...)` keep only tracked classes: `dets = [d for d in dets if d.cls in TRACKED_CLASSES]` before `res.frame_dets[frame_idx] = dets`.
- `run_multi(...)`: change `gt_cls: int = PERSON` (was `0`); add the same `TRACKED_CLASSES` filter after `dets = nms(...)`.
- `emit_frames_json(...)`: change the default `class_labels=("person", "vehicle", "face", "license_plate")` to `class_labels=LABELS` (the authoritative tuple, which has the leading "unlabeled" so `class_labels[d.cls]` lines up with person=1).

- [ ] **Step 8: Run the full dynamic_tiling suite**

Run: `python -m pytest dynamic_tiling/tests/ -q`
Expected: PASS (existing tests that constructed locks/runs with `person_cls=0` should be updated if any fail — search `person_cls=0` / `gt_cls=0` in tests and switch to `PERSON`). If a test explicitly asserts a 0-indexed class, update it to person=1 with a comment referencing `hailo_tiling/classes.py`.

- [ ] **Step 9: Commit**

```bash
git add hailo_tiling/backends/hef.py dynamic_tiling/inference.py dynamic_tiling/replay.py dynamic_tiling/tests/test_class_offset.py
git commit -m "feat(tiling): class_offset on HefBackend + person=1/vehicle=2 standardization

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: GT-track builder core — BoT-SORT over dense detections

**Files:**
- Create: `dynamic_tiling/gt_mot.py`
- Test: `dynamic_tiling/tests/test_gt_mot.py`

Produce raw per-object trajectories from a dense detections doc, using an **injectable tracker** so unit tests don't need boxmot/torch. The real builder wires boxmot's BoT-SORT (CMC + ReID); the default path is independent of the runtime ByteTracker.

The dense doc schema (confirmed): top-level `{"frames": [...]}`, each frame `{"frame": int, "detections": [{"bbox": [x,y,w,h] normalized, "confidence": float, "label": str, "cls": int}]}`.

- [ ] **Step 1: Write the failing test (injectable fake tracker)**

```python
# dynamic_tiling/tests/test_gt_mot.py
from dynamic_tiling.gt_mot import build_raw_tracks


class FakeTracker:
    """Assigns a stable id per detection by its column order (deterministic)."""
    def update(self, dets_xyxy_score_cls, frame):
        # dets_xyxy_score_cls: (N,6) [x1,y1,x2,y2,score,cls]; return (N,8)
        # [x1,y1,x2,y2,track_id,conf,cls,idx]
        import numpy as np
        n = dets_xyxy_score_cls.shape[0]
        out = np.zeros((n, 8), dtype=float)
        out[:, 0:4] = dets_xyxy_score_cls[:, 0:4]
        out[:, 4] = [i + 1 for i in range(n)]   # track id = row order
        out[:, 5] = dets_xyxy_score_cls[:, 4]
        out[:, 6] = dets_xyxy_score_cls[:, 5]
        return out


def test_build_raw_tracks_groups_by_track_id():
    doc = {"frames": [
        {"frame": 0, "detections": [
            {"bbox": [0.1, 0.1, 0.05, 0.1], "confidence": 0.9, "cls": 1, "label": "person"},
            {"bbox": [0.5, 0.5, 0.04, 0.08], "confidence": 0.8, "cls": 1, "label": "person"}]},
        {"frame": 1, "detections": [
            {"bbox": [0.12, 0.1, 0.05, 0.1], "confidence": 0.9, "cls": 1, "label": "person"},
            {"bbox": [0.5, 0.52, 0.04, 0.08], "confidence": 0.8, "cls": 1, "label": "person"}]},
    ]}
    tracks = build_raw_tracks(doc, tracker=FakeTracker(), classes=(1, 2))
    # two persistent ids, each present on both frames
    assert set(t.cls for t in tracks) == {1}
    assert all(len(t.frames) == 2 for t in tracks)
    ids = sorted(t.track_id for t in tracks)
    assert ids == [1, 2]


def test_build_raw_tracks_drops_untracked_classes():
    doc = {"frames": [{"frame": 0, "detections": [
        {"bbox": [0.1, 0.1, 0.05, 0.1], "confidence": 0.9, "cls": 3, "label": "face"}]}]}
    tracks = build_raw_tracks(doc, tracker=FakeTracker(), classes=(1, 2))
    assert tracks == []
```

- [ ] **Step 2: Run it**

Run: `python -m pytest dynamic_tiling/tests/test_gt_mot.py -v`
Expected: FAIL — `No module named 'dynamic_tiling.gt_mot'`.

- [ ] **Step 3: Implement `dynamic_tiling/gt_mot.py`**

```python
"""Offline ground-truth multi-object tracks from dense detector output.

Runs a HEAVY tracker (BoT-SORT: camera-motion compensation + ReID appearance)
over the dense 12x9 detections to produce one stable trajectory per object. This
is GT generation — it must be INDEPENDENT of the runtime ByteTracker under test.
The tracker is injectable so unit tests avoid the boxmot/torch dependency.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from hailo_tiling.classes import TRACKED_CLASSES


@dataclass
class RawTrack:
    cls: int
    track_id: int
    frames: dict = field(default_factory=dict)  # frame_idx -> (x,y,w,h) normalized


def _dets_to_xyxy(frame_doc, classes):
    """Return (N,6) array [x1,y1,x2,y2,score,cls] in NORMALIZED coords."""
    rows = []
    for d in frame_doc.get("detections", []):
        c = int(d.get("cls", -1))
        if c not in classes:
            continue
        x, y, w, h = d["bbox"]
        rows.append([x, y, x + w, y + h, float(d.get("confidence", 1.0)), c])
    return np.asarray(rows, dtype=float).reshape(-1, 6)


def build_raw_tracks(doc, *, tracker, classes=TRACKED_CLASSES):
    """Feed dense detections frame-by-frame to `tracker`; group outputs by id.

    `tracker.update(dets_xyxy_score_cls, frame)` must return rows
    [x1,y1,x2,y2,track_id,conf,cls,...] (the boxmot tracker contract).
    """
    classes = tuple(classes)
    out: dict = {}  # (cls, track_id) -> RawTrack
    frames = sorted(doc.get("frames", []), key=lambda fr: fr["frame"])
    for fr in frames:
        fi = fr["frame"]
        dets = _dets_to_xyxy(fr, classes)
        res = tracker.update(dets, fr)
        if res is None or len(res) == 0:
            continue
        for row in np.asarray(res, dtype=float):
            x1, y1, x2, y2, tid, _conf, cls = row[0], row[1], row[2], row[3], int(row[4]), row[5], int(row[6])
            key = (cls, tid)
            t = out.get(key)
            if t is None:
                t = RawTrack(cls=cls, track_id=tid)
                out[key] = t
            t.frames[fi] = (x1, y1, x2 - x1, y2 - y1)
    return list(out.values())
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python -m pytest dynamic_tiling/tests/test_gt_mot.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Add the real BoT-SORT factory (integration spike) — install + wire**

Install boxmot into the project venv:

```bash
./hailo-apps/venv_hailo_apps/bin/pip install boxmot 2>&1 | ~/.claude/token-filter/token-filter.sh --cmd "pip install boxmot"
```

Add a factory to `gt_mot.py` that builds a real BoT-SORT (image-aware: it needs the frame for CMC + ReID, so the caller passes the BGR frame as `fr`):

```python
def make_botsort(reid_weights="osnet_x0_25_msmt17.pt", device="cuda:0", half=True):
    """Construct a boxmot BoT-SORT tracker (CMC + ReID). Falls back to OC-SORT
    if BoT-SORT import fails (documented fallback in the spec)."""
    from pathlib import Path
    try:
        from boxmot import BotSort
        return BotSort(reid_weights=Path(reid_weights), device=device, half=half)
    except Exception:  # pragma: no cover - environment-dependent
        from boxmot import OcSort
        return OcSort()
```

The boxmot contract differs from our `FakeTracker`: `tracker.update(dets, img)` expects `dets` as `(N,6)` `[x1,y1,x2,y2,conf,cls]` in **pixel** coords and `img` as the BGR frame. Add the pixel-aware `build_raw_tracks_from_video` (used by the CLI in Task 4) alongside the normalized, unit-tested `build_raw_tracks` core:

```python
def build_raw_tracks_from_video(doc, video_path, *, tracker_factory=make_botsort,
                                classes=TRACKED_CLASSES):
    """Pixel-coords BoT-SORT pass over the video frames + dense dets.

    boxmot needs the BGR frame (CMC + ReID) and pixel-coord dets. Detections
    are scaled to pixels per frame; results are stored back in NORMALIZED coords.
    """
    import cv2
    import numpy as np
    classes = tuple(classes)
    by_frame = {fr["frame"]: fr for fr in doc.get("frames", [])}
    tracker = tracker_factory()
    out: dict = {}
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise SystemExit(f"cannot open video: {video_path}")
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fi = -1
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            fi += 1
            fr = by_frame.get(fi)
            rows = []
            if fr is not None:
                for d in fr.get("detections", []):
                    c = int(d.get("cls", -1))
                    if c not in classes:
                        continue
                    x, y, bw, bh = d["bbox"]
                    rows.append([x * w, y * h, (x + bw) * w, (y + bh) * h,
                                 float(d.get("confidence", 1.0)), c])
            px = np.asarray(rows, dtype=float).reshape(-1, 6)
            res = tracker.update(px, frame)
            if res is None or len(res) == 0:
                continue
            for row in np.asarray(res, dtype=float):
                x1, y1, x2, y2 = row[0], row[1], row[2], row[3]
                tid, cls = int(row[4]), int(row[6])
                key = (cls, tid)
                t = out.get(key) or RawTrack(cls=cls, track_id=tid)
                out[key] = t
                t.frames[fi] = (x1 / w, y1 / h, (x2 - x1) / w, (y2 - y1) / h)
    finally:
        cap.release()
    return list(out.values())
```

- [ ] **Step 6: Commit**

```bash
git add dynamic_tiling/gt_mot.py dynamic_tiling/tests/test_gt_mot.py
git commit -m "feat(gt): BoT-SORT raw-track builder over dense detections (injectable tracker)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: GT-track cleaning — gap interpolation + quality filter

**Files:**
- Create: `dynamic_tiling/gt_clean.py`
- Test: `dynamic_tiling/tests/test_gt_clean.py`

Turn raw tracks into clean GT trajectories: bridge short gaps by linear interpolation, then keep only trajectories long and clean enough to be fair follow targets.

- [ ] **Step 1: Write the failing test**

```python
# dynamic_tiling/tests/test_gt_clean.py
from dynamic_tiling.gt_mot import RawTrack
from dynamic_tiling.gt_clean import interpolate_gaps, filter_tracks, GtTrack


def test_interpolate_bridges_short_gap():
    t = RawTrack(cls=1, track_id=5, frames={0: (0.0, 0.0, 0.1, 0.1),
                                            2: (0.2, 0.0, 0.1, 0.1)})
    out = interpolate_gaps(t, max_gap=3)
    # frame 1 is filled at the midpoint
    assert 1 in out.frames
    assert abs(out.frames[1][0] - 0.1) < 1e-9


def test_interpolate_leaves_long_gap_unfilled():
    t = RawTrack(cls=1, track_id=5, frames={0: (0.0, 0.0, 0.1, 0.1),
                                            10: (0.2, 0.0, 0.1, 0.1)})
    out = interpolate_gaps(t, max_gap=3)
    assert 5 not in out.frames


def test_filter_drops_short_tracks():
    short = GtTrack(cls=1, track_id=1, frames={i: (0.0, 0.0, 0.1, 0.1) for i in range(3)})
    long = GtTrack(cls=1, track_id=2, frames={i: (0.0, 0.0, 0.1, 0.1) for i in range(40)})
    kept = filter_tracks([short, long], min_len=30)
    assert [t.track_id for t in kept] == [2]
```

- [ ] **Step 2: Run it**

Run: `python -m pytest dynamic_tiling/tests/test_gt_clean.py -v`
Expected: FAIL — `No module named 'dynamic_tiling.gt_clean'`.

- [ ] **Step 3: Implement `dynamic_tiling/gt_clean.py`**

```python
"""Clean raw GT tracks: interpolate short gaps, filter to fair follow targets."""
from __future__ import annotations

from dataclasses import dataclass, field

from .gt_mot import RawTrack


@dataclass
class GtTrack:
    cls: int
    track_id: int
    frames: dict = field(default_factory=dict)  # frame_idx -> (x,y,w,h) normalized


def _lerp(a, b, frac):
    return tuple(av + (bv - av) * frac for av, bv in zip(a, b))


def interpolate_gaps(track: RawTrack, *, max_gap: int = 5) -> GtTrack:
    """Fill frame gaps of length <= max_gap by linear interpolation of bbox."""
    fis = sorted(track.frames)
    out = dict(track.frames)
    for f0, f1 in zip(fis, fis[1:]):
        gap = f1 - f0
        if 1 < gap <= max_gap:
            b0, b1 = track.frames[f0], track.frames[f1]
            for k in range(1, gap):
                out[f0 + k] = _lerp(b0, b1, k / gap)
    return GtTrack(cls=track.cls, track_id=track.track_id, frames=out)


def filter_tracks(tracks, *, min_len: int = 30):
    """Keep only trajectories with >= min_len frames (fair-target quality gate)."""
    return [t for t in tracks if len(t.frames) >= min_len]


def clean_tracks(raw_tracks, *, max_gap: int = 5, min_len: int = 30):
    """Interpolate then filter; the full raw->clean pipeline."""
    return filter_tracks([interpolate_gaps(t, max_gap=max_gap) for t in raw_tracks],
                         min_len=min_len)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest dynamic_tiling/tests/test_gt_clean.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/gt_clean.py dynamic_tiling/tests/test_gt_clean.py
git commit -m "feat(gt): gap interpolation + quality filter for GT tracks

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: GT-tracks CLI + spot-check overlay

**Files:**
- Create: `dynamic_tiling/run_gt_tracks.py`
- Test: `dynamic_tiling/tests/test_run_gt_tracks.py`

Produce `gt_tracks.json` (the trial source) from a dense frames.json + the video, and an overlay frames.json so the GT can be eyeballed in the visualizer (spec §7.2 validation step).

`gt_tracks.json` schema: `{"clip": <name>, "tracks": [{"cls": int, "track_id": int, "frames": {"<frame_idx>": [x,y,w,h]}}]}`.

- [ ] **Step 1: Write the failing test (serialization round-trip, no video/boxmot)**

```python
# dynamic_tiling/tests/test_run_gt_tracks.py
import json
from dynamic_tiling.gt_clean import GtTrack
from dynamic_tiling.run_gt_tracks import tracks_to_doc, doc_to_tracks


def test_tracks_doc_roundtrip():
    tracks = [GtTrack(cls=1, track_id=7, frames={0: (0.1, 0.2, 0.3, 0.4), 5: (0.11, 0.2, 0.3, 0.4)})]
    doc = tracks_to_doc(tracks, clip="x")
    s = json.dumps(doc)               # must be JSON-serializable
    back = doc_to_tracks(json.loads(s))
    assert back[0].cls == 1 and back[0].track_id == 7
    assert back[0].frames[0] == (0.1, 0.2, 0.3, 0.4)   # int keys restored
```

- [ ] **Step 2: Run it**

Run: `python -m pytest dynamic_tiling/tests/test_run_gt_tracks.py -v`
Expected: FAIL — `No module named 'dynamic_tiling.run_gt_tracks'`.

- [ ] **Step 3: Implement `dynamic_tiling/run_gt_tracks.py`**

```python
"""CLI: dense frames.json + video -> gt_tracks.json (+ spot-check overlay).

    source setup_env.sh
    python -m dynamic_tiling.run_gt_tracks \
        --dense /tmp/gt_legacy/12x9.frames.json \
        --video /home/giladn/Videos/Drone/Training/DJI_..._0026_..._fov50.mp4 \
        --out dynamic_tiling/runs/gt_tracks_0026_fov50.json \
        --overlay dynamic_tiling/runs/gt_tracks_0026_fov50.frames.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from hailo_tiling.classes import LABELS, TRACKED_CLASSES
from .gt_clean import GtTrack, clean_tracks


def tracks_to_doc(tracks, *, clip: str) -> dict:
    return {"clip": clip, "tracks": [
        {"cls": t.cls, "track_id": t.track_id,
         "frames": {str(f): list(b) for f, b in sorted(t.frames.items())}}
        for t in tracks]}


def doc_to_tracks(doc) -> list:
    return [GtTrack(cls=t["cls"], track_id=t["track_id"],
                    frames={int(f): tuple(b) for f, b in t["frames"].items()})
            for t in doc["tracks"]]


def _overlay_doc(tracks, label: str) -> dict:
    """One frames.json with every GT track's bbox, coloured by class label."""
    per_frame: dict = {}
    for t in tracks:
        for f, b in t.frames.items():
            per_frame.setdefault(f, []).append(
                {"label": LABELS[t.cls] if 0 <= t.cls < len(LABELS) else str(t.cls),
                 "confidence": 1.0, "bbox": list(b),
                 "track_id": t.track_id})
    frames = [{"frame": f, "detections": per_frame[f], "tiles": []}
              for f in sorted(per_frame)]
    return {"label": label, "frames": frames}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dense", required=True, type=Path)
    ap.add_argument("--video", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--overlay", type=Path, default=None)
    ap.add_argument("--max-gap", type=int, default=5)
    ap.add_argument("--min-len", type=int, default=30)
    args = ap.parse_args()

    from .gt_mot import build_raw_tracks_from_video, make_botsort
    doc = json.loads(args.dense.read_text())
    raw = build_raw_tracks_from_video(doc, str(args.video),
                                      tracker_factory=make_botsort,
                                      classes=TRACKED_CLASSES)
    tracks = clean_tracks(raw, max_gap=args.max_gap, min_len=args.min_len)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(tracks_to_doc(tracks, clip=args.video.stem)))
    print(f"GT tracks written: {args.out}  ({len(tracks)} tracks)")
    if args.overlay:
        args.overlay.write_text(json.dumps(_overlay_doc(tracks, label="gt-tracks")))
        print(f"overlay written : {args.overlay}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the round-trip test**

Run: `python -m pytest dynamic_tiling/tests/test_run_gt_tracks.py -v`
Expected: PASS.

- [ ] **Step 5: Generate real GT tracks on-chip and spot-check (manual validation)**

```bash
source setup_env.sh
python -m dynamic_tiling.run_gt_tracks \
  --dense /tmp/gt_legacy/12x9.frames.json \
  --video /home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4 \
  --out dynamic_tiling/runs/gt_tracks_0026_fov50.json \
  --overlay dynamic_tiling/runs/gt_tracks_0026_fov50.frames.json
```
Expected: prints "GT tracks written ... (N tracks)" with N ≥ 2. Then eyeball in the visualizer:
```bash
DISPLAY=:1 XAUTHORITY=/run/user/10615/gdm/Xauthority \
  python3 tiling_benchmark/overlay_viewer.py \
    --video /home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4 \
    --frames "dynamic_tiling/runs/gt_tracks_0026_fov50.frames.json:gt-tracks"
```
Expected: each person/vehicle keeps a single consistent box through the clip (no wild id swaps at crossings). Note any defects in the commit message; pseudo-GT imperfection is acceptable and documented.

- [ ] **Step 6: Commit (code only — NOT the generated run outputs)**

```bash
git add dynamic_tiling/run_gt_tracks.py dynamic_tiling/tests/test_run_gt_tracks.py
git commit -m "feat(gt): run_gt_tracks CLI (dense -> gt_tracks.json + spot-check overlay)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: Metric suite — coverage, localization, drift

**Files:**
- Create: `dynamic_tiling/metrics.py`
- Test: `dynamic_tiling/tests/test_metrics_coverage.py`

Score one trial's predicted trajectory against the target GT trajectory, with distractor trajectories available to detect mis-locks (spec §5.3 metrics 1–3).

- [ ] **Step 1: Write the failing test**

```python
# dynamic_tiling/tests/test_metrics_coverage.py
from dynamic_tiling.metrics import score_trial


def test_perfect_coverage():
    gt = {0: (0.1, 0.1, 0.2, 0.2), 1: (0.1, 0.1, 0.2, 0.2)}
    pred = dict(gt)
    s = score_trial(gt, pred, distractors=[], iou_thr=0.5)
    assert s.coverage == 1.0
    assert s.mean_iou > 0.99
    assert s.drift_rate == 0.0


def test_miss_counts_against_coverage():
    gt = {0: (0.1, 0.1, 0.2, 0.2), 1: (0.1, 0.1, 0.2, 0.2)}
    pred = {0: (0.1, 0.1, 0.2, 0.2)}            # frame 1 missing
    s = score_trial(gt, pred, distractors=[], iou_thr=0.5)
    assert s.coverage == 0.5


def test_drift_when_pred_matches_distractor():
    gt = {0: (0.1, 0.1, 0.2, 0.2)}
    distractor = {0: (0.6, 0.6, 0.2, 0.2)}
    pred = {0: (0.6, 0.6, 0.2, 0.2)}            # locked onto the distractor
    s = score_trial(gt, pred, distractors=[distractor], iou_thr=0.5)
    assert s.coverage == 0.0
    assert s.drift_rate == 1.0
```

- [ ] **Step 2: Run it**

Run: `python -m pytest dynamic_tiling/tests/test_metrics_coverage.py -v`
Expected: FAIL — `No module named 'dynamic_tiling.metrics'`.

- [ ] **Step 3: Implement coverage/drift in `dynamic_tiling/metrics.py`**

```python
"""Tracking + recovery metric suite for single-target follow trials."""
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
class TrialScore:
    n_frames: int
    coverage: float          # fraction of GT-present frames the target is correctly followed
    mean_iou: float          # mean IoU over covered frames
    drift_rate: float        # fraction of GT-present frames the pred matched a DISTRACTOR
    # recovery fields filled by Task 6:
    loss_events: int = 0
    mean_time_to_recover: float = 0.0
    recovery_success_rate: float = 0.0


def _covered(gt_box, pred_box, iou_thr):
    return pred_box is not None and _iou(gt_box, pred_box) >= iou_thr


def score_trial(target_traj, pred_traj, *, distractors, iou_thr=0.5) -> TrialScore:
    """target_traj/pred_traj: {frame: (x,y,w,h)}. distractors: list of {frame: bbox}.
    Frames scored = frames where the target is present in GT."""
    frames = sorted(target_traj)
    n = len(frames)
    n_cov = 0
    n_drift = 0
    iou_sum = 0.0
    for f in frames:
        gt_box = target_traj[f]
        pred_box = pred_traj.get(f)
        if _covered(gt_box, pred_box, iou_thr):
            n_cov += 1
            iou_sum += _iou(gt_box, pred_box)
        elif pred_box is not None:
            # not on target — did we lock onto a different GT object?
            if any(_covered(d.get(f), pred_box, iou_thr) for d in distractors
                   if d.get(f) is not None):
                n_drift += 1
    return TrialScore(
        n_frames=n,
        coverage=(n_cov / n) if n else 0.0,
        mean_iou=(iou_sum / n_cov) if n_cov else 0.0,
        drift_rate=(n_drift / n) if n else 0.0,
    )
```

(`_covered(d.get(f), ...)` is safe: when `d.get(f)` is None the helper short-circuits on `pred_box is not None` only — guard added via the `if d.get(f) is not None` filter.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest dynamic_tiling/tests/test_metrics_coverage.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/metrics.py dynamic_tiling/tests/test_metrics_coverage.py
git commit -m "feat(metrics): follow coverage + localization + drift/mis-lock

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Metric suite — recovery (loss events, time-to-recover, success)

**Files:**
- Modify: `dynamic_tiling/metrics.py`
- Test: `dynamic_tiling/tests/test_metrics_recovery.py`

A *loss* is a maximal run of GT-present frames where the target is not covered. Recovery success = the run ends with correct re-coverage on the same target (spec §5.3 metric 4, strict same-object definition the user approved).

- [ ] **Step 1: Write the failing test**

```python
# dynamic_tiling/tests/test_metrics_recovery.py
from dynamic_tiling.metrics import score_trial


def _box(x):  # helper: a unit-ish box at x
    return (x, 0.1, 0.1, 0.1)


def test_single_loss_then_recover():
    # covered 0,1 ; lost 2,3 ; recovered 4,5
    gt = {f: _box(0.1) for f in range(6)}
    pred = {0: _box(0.1), 1: _box(0.1), 4: _box(0.1), 5: _box(0.1)}
    s = score_trial(gt, pred, distractors=[], iou_thr=0.5)
    assert s.loss_events == 1
    assert s.mean_time_to_recover == 2.0     # frames 2,3 lost before recovery at 4
    assert s.recovery_success_rate == 1.0


def test_unrecovered_loss_at_clip_end():
    gt = {f: _box(0.1) for f in range(4)}
    pred = {0: _box(0.1), 1: _box(0.1)}      # lost 2,3 and never recovered
    s = score_trial(gt, pred, distractors=[], iou_thr=0.5)
    assert s.loss_events == 1
    assert s.recovery_success_rate == 0.0
```

- [ ] **Step 2: Run it**

Run: `python -m pytest dynamic_tiling/tests/test_metrics_recovery.py -v`
Expected: FAIL — recovery fields are still 0 (defaults), assertions on `loss_events`/`mean_time_to_recover` fail.

- [ ] **Step 3: Add recovery computation to `score_trial` in `dynamic_tiling/metrics.py`**

Insert recovery accounting before the `return`, replacing the return with the version below:

```python
    # --- recovery: scan the covered/uncovered sequence over GT-present frames ---
    covered_seq = [_covered(target_traj[f], pred_traj.get(f), iou_thr) for f in frames]
    loss_events = 0
    recovered = 0
    ttr_sum = 0
    i = 0
    while i < n:
        if not covered_seq[i]:
            loss_events += 1
            j = i
            while j < n and not covered_seq[j]:
                j += 1
            # j == n  -> never recovered (clip ended lost); else recovered at j
            if j < n:
                recovered += 1
                ttr_sum += (j - i)
            i = j
        else:
            i += 1
    return TrialScore(
        n_frames=n,
        coverage=(n_cov / n) if n else 0.0,
        mean_iou=(iou_sum / n_cov) if n_cov else 0.0,
        drift_rate=(n_drift / n) if n else 0.0,
        loss_events=loss_events,
        mean_time_to_recover=(ttr_sum / recovered) if recovered else 0.0,
        recovery_success_rate=(recovered / loss_events) if loss_events else 0.0,
    )
```

Delete the old `return TrialScore(...)` (the one without recovery fields) so only this one remains.

- [ ] **Step 4: Run both metric test files**

Run: `python -m pytest dynamic_tiling/tests/test_metrics_coverage.py dynamic_tiling/tests/test_metrics_recovery.py -v`
Expected: PASS (5 tests total). The first-frame-lost case is handled: if frame 0 is uncovered it opens a loss event immediately.

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/metrics.py dynamic_tiling/tests/test_metrics_recovery.py
git commit -m "feat(metrics): recovery — loss events, time-to-recover, success rate

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 7: Multi-trial driver — one follow per GT object

**Files:**
- Create: `dynamic_tiling/trials.py`
- Test: `dynamic_tiling/tests/test_trials.py`

For each GT track, run a single-target follow seeded on that track and score it against that track (the others are distractors), then aggregate. The per-trial run reuses `dynamic_tiling.replay.run`; the seeding passes that track's per-frame bbox as `gt_traj` (GT-bbox-seeded, ByteTracker-id-locked — spec §6).

- [ ] **Step 1: Write the failing test (fake backend + real scheduler/lock)**

Isolate the aggregation logic that `trials.py` owns by monkeypatching `replay.run` (so the test does not depend on inference / `map_to_source` mapping semantics — the real integration is covered by the on-chip smoke in Task 8).

```python
# dynamic_tiling/tests/test_trials.py
import dynamic_tiling.trials as trials_mod
from dynamic_tiling.gt_clean import GtTrack
from dynamic_tiling.trials import run_all_trials, AggregateScore
from dynamic_tiling.replay import RunResult


class _NullBackend:
    def close(self):
        pass


def test_run_all_trials_aggregates_per_track(monkeypatch):
    tracks = [
        GtTrack(cls=1, track_id=1, frames={i: (0.1, 0.1, 0.2, 0.2) for i in range(10)}),
        GtTrack(cls=1, track_id=2, frames={i: (0.6, 0.6, 0.1, 0.1) for i in range(10)}),
    ]

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj, person_cls=1):
        # perfectly follow whatever target trajectory was passed in this trial
        r = RunResult()
        r.pred_traj = dict(gt_traj)
        r.n_frames = 10
        r.total_tiles = 14            # -> avg 1.4 tiles/frame
        return r

    monkeypatch.setattr(trials_mod, "run", fake_run)
    agg = run_all_trials(
        frames_factory=lambda: iter([]),
        src_w=1920, src_h=1080, gt_tracks=tracks,
        backend_factory=lambda: _NullBackend(),
        budget=300.0, fps=30.0, discovery_fps=2.0,
    )
    assert isinstance(agg, AggregateScore)
    assert agg.n_trials == 2
    assert agg.mean_coverage == 1.0          # each trial follows its own target perfectly
    assert agg.mean_drift_rate == 0.0
    assert abs(agg.avg_tiles_per_frame - 1.4) < 1e-9
```

- [ ] **Step 2: Run it**

Run: `python -m pytest dynamic_tiling/tests/test_trials.py -v`
Expected: FAIL — `No module named 'dynamic_tiling.trials'`.

- [ ] **Step 3: Implement `dynamic_tiling/trials.py`**

```python
"""Run one single-target follow trial per GT object, aggregate the metric suite."""
from __future__ import annotations

from dataclasses import dataclass

from hailo_tiling.classes import PERSON

from .budget import BudgetMeter
from .scheduler import TileScheduler
from .target_lock import TargetLock
from .replay import run
from .metrics import score_trial, TrialScore


@dataclass
class AggregateScore:
    n_trials: int
    mean_coverage: float
    mean_iou: float
    mean_drift_rate: float
    mean_loss_events: float
    mean_time_to_recover: float
    mean_recovery_success: float
    avg_tiles_per_frame: float
    per_trial: list  # list[TrialScore]


def run_all_trials(*, frames_factory, src_w, src_h, gt_tracks,
                   backend_factory, budget, fps, discovery_fps,
                   max_zoom=2.0, target_model_h=40.0,
                   discovery_grid=None, iou_thr=0.5) -> AggregateScore:
    discovery_period = max(1, int(round(fps / discovery_fps)))
    per_trial = []
    tiles_acc = 0.0
    for target in gt_tracks:
        target_traj = target.frames
        distractors = [t.frames for t in gt_tracks if t is not target]
        _disc = {"discovery_grid": discovery_grid} if discovery_grid else {}
        scheduler = TileScheduler(src_w, src_h, discovery_period=discovery_period,
                                  max_zoom=max_zoom, target_model_h=target_model_h, **_disc)
        lock = TargetLock(frame_rate=int(fps))
        backend = backend_factory()
        meter = BudgetMeter(budget_inf_per_s=budget, fps=fps)
        try:
            res = run(frames_factory(), src_w, src_h, scheduler, lock, backend,
                      meter, target_traj, person_cls=PERSON)
        finally:
            backend.close()
        # only score frames we actually played
        gt_for_score = {f: b for f, b in target_traj.items() if f < res.n_frames}
        per_trial.append(score_trial(gt_for_score, res.pred_traj,
                                     distractors=distractors, iou_thr=iou_thr))
        tiles_acc += res.avg_tiles_per_frame

    n = len(per_trial)
    def mean(attr):
        return sum(getattr(s, attr) for s in per_trial) / n if n else 0.0
    return AggregateScore(
        n_trials=n,
        mean_coverage=mean("coverage"),
        mean_iou=mean("mean_iou"),
        mean_drift_rate=mean("drift_rate"),
        mean_loss_events=mean("loss_events"),
        mean_time_to_recover=mean("mean_time_to_recover"),
        mean_recovery_success=mean("recovery_success_rate"),
        avg_tiles_per_frame=(tiles_acc / n) if n else 0.0,
        per_trial=per_trial,
    )
```

Note: `TargetLock.__init__` takes `track_buffer` and `**tracker_kwargs`; `frame_rate` is forwarded to `create_tracker`. If the suite reveals `frame_rate` is not a valid tracker kwarg, change `TargetLock(frame_rate=int(fps))` to `TargetLock(track_buffer=int(fps))` to match `run_dynamic.py:112`'s existing call — keep the two callers consistent.

- [ ] **Step 4: Run the trial test**

Run: `python -m pytest dynamic_tiling/tests/test_trials.py -v`
Expected: PASS (1 test). The monkeypatched `run` means `TargetLock`/`TileScheduler` are still constructed by `run_all_trials` before the patched `run` is reached — if construction (`TargetLock(frame_rate=...)`) raises, apply the note in Step 3 and re-run.

- [ ] **Step 5: Commit**

```bash
git add dynamic_tiling/trials.py dynamic_tiling/tests/test_trials.py
git commit -m "feat(trials): per-object follow trials + metric aggregation

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 8: Trials CLI + cache-backed inference

**Files:**
- Create: `dynamic_tiling/run_trials.py`
- Test: `dynamic_tiling/tests/test_run_trials_cli.py`

Tie it together: read `gt_tracks.json`, re-open the video per trial (frames factory), run all trials through a **CachingBackend-wrapped** HefBackend so tiles repeated across trials are not re-inferred, and print the aggregate metric table.

- [ ] **Step 1: Write the failing test (arg-free helper: aggregate table formatting)**

```python
# dynamic_tiling/tests/test_run_trials_cli.py
from dynamic_tiling.trials import AggregateScore
from dynamic_tiling.run_trials import format_aggregate


def test_format_aggregate_table():
    agg = AggregateScore(n_trials=3, mean_coverage=0.91, mean_iou=0.72,
                         mean_drift_rate=0.03, mean_loss_events=1.3,
                         mean_time_to_recover=4.2, mean_recovery_success=0.8,
                         avg_tiles_per_frame=1.4, per_trial=[])
    txt = format_aggregate(agg, budget=300.0, fps=30.0)
    assert "coverage" in txt and "0.91" in txt
    assert "tiles/frame" in txt and "1.4" in txt
```

- [ ] **Step 2: Run it**

Run: `python -m pytest dynamic_tiling/tests/test_run_trials_cli.py -v`
Expected: FAIL — `No module named 'dynamic_tiling.run_trials'`.

- [ ] **Step 3: Implement `dynamic_tiling/run_trials.py`**

```python
"""CLI: video + gt_tracks.json -> per-trial + aggregate tracking/recovery metrics.

    source setup_env.sh
    python -m dynamic_tiling.run_trials \
        --video /home/giladn/Videos/Drone/Training/DJI_..._0026_..._fov50.mp4 \
        --gt-tracks dynamic_tiling/runs/gt_tracks_0026_fov50.json \
        --budget 40 --fps 30 --discovery-fps 2 --discovery-grid 8x6
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2

from .gt_clean import GtTrack
from .trials import run_all_trials, AggregateScore


def _load_tracks(path: Path):
    doc = json.loads(path.read_text())
    return [GtTrack(cls=t["cls"], track_id=t["track_id"],
                    frames={int(f): tuple(b) for f, b in t["frames"].items()})
            for t in doc["tracks"]]


def _frames_factory(video: Path, max_frames: int):
    def gen():
        cap = cv2.VideoCapture(str(video))
        n = 0
        try:
            while True:
                ok, fr = cap.read()
                if not ok or (max_frames and n >= max_frames):
                    break
                yield fr
                n += 1
        finally:
            cap.release()
    return gen


def format_aggregate(agg: AggregateScore, *, budget: float, fps: float) -> str:
    return (
        f"trials           : {agg.n_trials}\n"
        f"tiles/frame      : {agg.avg_tiles_per_frame:.3f}  (budget {budget} @ {fps}fps)\n"
        f"coverage         : {agg.mean_coverage:.3f}\n"
        f"mean IoU         : {agg.mean_iou:.3f}\n"
        f"drift rate       : {agg.mean_drift_rate:.3f}\n"
        f"loss events      : {agg.mean_loss_events:.2f}\n"
        f"time-to-recover  : {agg.mean_time_to_recover:.2f}\n"
        f"recovery success : {agg.mean_recovery_success:.3f}\n"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True, type=Path)
    ap.add_argument("--gt-tracks", required=True, type=Path)
    ap.add_argument("--hef",
                    default="/usr/local/hailo/resources/models/hailo10h/"
                            "hailo_yolov8n_4_classes_vga.hef")
    ap.add_argument("--budget", type=float, default=40.0)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--discovery-fps", type=float, default=2.0)
    ap.add_argument("--discovery-grid", default=None)
    ap.add_argument("--max-zoom", type=float, default=2.0)
    ap.add_argument("--target-model-h", type=float, default=40.0)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--nms-thresh", type=float, default=0.25)
    args = ap.parse_args()

    discovery_grid = None
    if args.discovery_grid:
        gx, gy = args.discovery_grid.lower().split("x")
        discovery_grid = (int(gx), int(gy))

    cap = cv2.VideoCapture(str(args.video))
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()

    tracks = _load_tracks(args.gt_tracks)

    def backend_factory():
        from .inference import HefBackend
        return HefBackend(args.hef, nms_score_threshold=args.nms_thresh, class_offset=1)

    agg = run_all_trials(
        frames_factory=_frames_factory(args.video, args.max_frames),
        src_w=src_w, src_h=src_h, gt_tracks=tracks,
        backend_factory=backend_factory,
        budget=args.budget, fps=args.fps, discovery_fps=args.discovery_fps,
        max_zoom=args.max_zoom, target_model_h=args.target_model_h,
        discovery_grid=discovery_grid)

    print(format_aggregate(agg, budget=args.budget, fps=args.fps))


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the formatting test**

Run: `python -m pytest dynamic_tiling/tests/test_run_trials_cli.py -v`
Expected: PASS.

- [ ] **Step 5: On-chip smoke run (bounded)**

```bash
source setup_env.sh
python -m dynamic_tiling.run_trials \
  --video /home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4 \
  --gt-tracks dynamic_tiling/runs/gt_tracks_0026_fov50.json \
  --budget 40 --fps 30 --discovery-fps 2 --discovery-grid 8x6 --max-frames 120
```
Expected: prints the aggregate table with `trials` ≥ 2 and non-zero `coverage`. This validates the whole foundation end-to-end on real inference.

- [ ] **Step 6: Commit (code only)**

```bash
git add dynamic_tiling/run_trials.py dynamic_tiling/tests/test_run_trials_cli.py
git commit -m "feat(trials): run_trials CLI over gt_tracks with class_offset backend

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 9: Full-suite regression + INDEX/HANDOFF update

**Files:**
- Modify: `docs/superpowers/plans/INDEX.md`

- [ ] **Step 1: Run the complete dynamic_tiling + hailo_tiling suites**

Run: `python -m pytest dynamic_tiling/tests/ hailo_tiling/tests/ -q`
Expected: all pass (prior 282 + new ~16). If any prior test broke on the person=1 switch, fix it to the new convention with a comment, do not weaken assertions.

- [ ] **Step 2: Add the Phase-0 row to `docs/superpowers/plans/INDEX.md`**

Add under the table:
```markdown
| 11  | `2026-06-02-experiment-foundation-phase0.md` (class standardization + BoT-SORT GT tracks + tracking/recovery metrics + all-objects trials) | n/a (experiment) | in flight |
```
And under "Dynamic-path tuning + unification" note that Phase 0 closes: the two class conventions (now person=1 via `class_offset`) and adds the GT-track + metric foundation; Phase A (simple sweep) and Phase B (knapsack) plans to follow.

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/plans/INDEX.md
git commit -m "docs: Phase-0 experiment foundation landed; INDEX updated

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Subsequent plans (written after Phase 0 lands)

- **Plan 2 — Phase A (simple-scheduler tuning):** add the discovery-overlap knob to `dynamic_tiling/scheduler._grid`, a parameter-sweep driver over `run_all_trials`, and the static-grid baseline run through the same trial harness; output metric-vs-budget curves per fov. Written once Task 8's smoke confirms the metric numbers are sane.
- **Plan 3 — Phase B (knapsack challenger):** value function (`P(target)×resolution_gain×freshness`), the non-uniform cost model, the knapsack tile selector, and the head-to-head comparison vs Plan 2's best. Written after Plan 2 establishes the frontier to beat.

These are deferred deliberately (YAGNI): their task code depends on the real metric outputs and the tuned simple-config from Phase A.
