# Human-Verified GT Generation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Turn the auto-only GT-track builder into an offline, human-in-the-loop pipeline that (1) de-duplicates fragmented dense detections, (2) merges co-located/fragmented tracks with high confidence, (3) flags uncertain cases as reviewable annotated images, and (4) finalizes GT by applying human verdicts — producing trustworthy pseudo-GT.

**Architecture:** Offline (non-causal), quality-first. A per-frame fragment-NMS pass cleans the dense detections; the existing BoT-SORT pass tracks them; an offline track-graph pass auto-merges near-certain duplicate/fragment tracks and emits `ReviewCase`s for the gray zone; each case renders to an annotated PNG; human verdicts in `review_decisions.json` are applied in a finalize pass producing verified `gt_tracks.json` and a track-ID-coloured overlay.

**Tech Stack:** Python 3.10, pytest, numpy, OpenCV. boxmot (in isolated `.venv_gt`) for the tracking pass only. Pure-Python for dedup/merge/decisions (testable in the shared venv).

**Spec:** `docs/superpowers/specs/2026-06-02-tile-scheduler-experiment-design.md` §7.2 (GT builder), extended here with the human-verification loop discussed 2026-06-02.

**Environment:** Unit tests run in the SHARED venv: `./hailo-apps/venv_hailo_apps/bin/python -m pytest`. The full GT run (Task 6) runs in `.venv_gt` (boxmot) + reads the video. Branch: `tiling-benchmark` (never main; never push).

**Scope guardrails:** Stage only each task's listed files. NEVER stage pre-existing dirty files (`drone_follow/pipeline_adapter/reid_manager.py`, `sim/PX4-Autopilot`, `hailo-apps`, `tiling_benchmark/overlay_viewer.py`, `.claude/scheduled_tasks.lock`, `HANDOFF.md`, `dynamic_tiling/runs/*`) or the untracked `.venv_gt/`. Commit trailer: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.

**Design choices (approved):** inline human review (render PNGs, show to human); conservative auto-merge (auto only near-certain; flag the gray zone).

---

## File Structure

| File | Responsibility | C/M |
|------|----------------|-----|
| `dynamic_tiling/gt_dedup.py` | per-frame same-class fragment NMS over dense dets | Create |
| `dynamic_tiling/gt_review.py` | track-graph: auto-merge + flag (`ReviewCase`), short-track recovery flags, apply-decisions | Create |
| `dynamic_tiling/gt_render_review.py` | render one annotated PNG per `ReviewCase` | Create |
| `dynamic_tiling/run_gt_verify.py` | CLI: dedup→track→merge→flag→write clean GT + review_queue.json + PNGs; `--apply-decisions` finalize | Create |
| `dynamic_tiling/gt_mot.py` | call dedup before tracking (small change) | Modify |
| `dynamic_tiling/run_gt_tracks.py` | overlay can colour by track_id | Modify |
| `dynamic_tiling/tests/test_*.py` | unit tests per module | Create |

---

## Task 1: Per-frame fragment dedup

**Files:** Create `dynamic_tiling/gt_dedup.py`; Test `dynamic_tiling/tests/test_gt_dedup.py`

Fragmented dense tiling emits multiple overlapping boxes for one object (frame 868: 2 person + 5 vehicle boxes for ~3 objects). Collapse same-class boxes with high IoU into one (keep the highest-confidence box) BEFORE tracking.

- [ ] **Step 1: Failing test**
```python
# dynamic_tiling/tests/test_gt_dedup.py
from dynamic_tiling.gt_dedup import dedup_frame, dedup_doc


def test_dedup_merges_overlapping_same_class():
    dets = [
        {"bbox": [0.42, 0.64, 0.017, 0.054], "confidence": 0.90, "cls": 1},
        {"bbox": [0.424, 0.633, 0.011, 0.045], "confidence": 0.84, "cls": 1},  # dup of above
        {"bbox": [0.326, 0.388, 0.008, 0.031], "confidence": 0.68, "cls": 1},  # separate person
    ]
    out = dedup_frame(dets, iou_thr=0.5)
    assert len(out) == 2                       # two distinct persons
    assert out[0]["confidence"] == 0.90        # kept the higher-confidence box


def test_dedup_keeps_different_classes_separate():
    dets = [
        {"bbox": [0.5, 0.47, 0.05, 0.05], "confidence": 0.9, "cls": 1},
        {"bbox": [0.5, 0.47, 0.05, 0.05], "confidence": 0.8, "cls": 2},  # same place, diff class
    ]
    out = dedup_frame(dets, iou_thr=0.5)
    assert len(out) == 2


def test_dedup_doc_processes_all_frames():
    doc = {"frames": [
        {"frame": 0, "detections": [
            {"bbox": [0.4, 0.6, 0.02, 0.05], "confidence": 0.9, "cls": 1},
            {"bbox": [0.405, 0.6, 0.02, 0.05], "confidence": 0.8, "cls": 1}]}]}
    out = dedup_doc(doc, iou_thr=0.5)
    assert len(out["frames"][0]["detections"]) == 1
```

- [ ] **Step 2: Run — FAIL** (`python -m pytest dynamic_tiling/tests/test_gt_dedup.py -v`).

- [ ] **Step 3: Implement `dynamic_tiling/gt_dedup.py`**
```python
"""Per-frame fragment de-duplication of dense detections (offline GT prep).

Dense tiling fragments one object into several overlapping boxes. Greedy
same-class NMS collapses them: sort by confidence, keep a box, drop later
same-class boxes whose IoU with it exceeds the threshold.
"""
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


def dedup_frame(dets, *, iou_thr: float = 0.5):
    """Greedy same-class NMS. dets: list of {bbox:[x,y,w,h], confidence, cls}.
    Returns the kept detections (highest-confidence representative per cluster)."""
    order = sorted(dets, key=lambda d: -float(d.get("confidence", 0.0)))
    kept = []
    for d in order:
        b, c = d["bbox"], int(d.get("cls", -1))
        if any(int(k.get("cls", -1)) == c and _iou(b, k["bbox"]) > iou_thr for k in kept):
            continue
        kept.append(d)
    return kept


def dedup_doc(doc, *, iou_thr: float = 0.5) -> dict:
    """Return a new doc with each frame's detections de-duplicated."""
    return {**{k: v for k, v in doc.items() if k != "frames"},
            "frames": [{**fr, "detections": dedup_frame(fr.get("detections", []),
                                                         iou_thr=iou_thr)}
                       for fr in doc.get("frames", [])]}
```

- [ ] **Step 4: Run — PASS** (3 tests).
- [ ] **Step 5: Commit** `dynamic_tiling/gt_dedup.py dynamic_tiling/tests/test_gt_dedup.py` — `feat(gt): per-frame fragment dedup (same-class NMS) for offline GT prep`.

---

## Task 2: Offline track-graph merge + flag

**Files:** Create `dynamic_tiling/gt_review.py`; Test `dynamic_tiling/tests/test_gt_review.py`

Given clean tracks (list of `GtTrack`-like with `cls`, `track_id`, `frames`), build the pairwise temporal-overlap graph and: AUTO-merge near-certain duplicates (high shared-frame IoU); FLAG the gray zone as `ReviewCase`s; FLAG short tracks for keep/drop. Conservative bias.

- [ ] **Step 1: Failing test**
```python
# dynamic_tiling/tests/test_gt_review.py
from dynamic_tiling.gt_clean import GtTrack
from dynamic_tiling.gt_review import merge_and_flag, ReviewCase, apply_decisions


def _t(tid, cls, frames):
    return GtTrack(cls=cls, track_id=tid, frames=frames)


def test_auto_merges_near_certain_duplicate():
    # two tracks on the SAME object across 40 shared frames (IoU ~1.0) -> auto-merge
    box = (0.42, 0.64, 0.02, 0.05)
    a = _t(1, 1, {i: box for i in range(40)})
    b = _t(2, 1, {i: (0.421, 0.64, 0.02, 0.05) for i in range(40)})
    merged, cases = merge_and_flag([a, b], auto_iou=0.7, flag_iou=0.3, min_len=30)
    assert len(merged) == 1
    assert not any(c.kind == "merge" for c in cases)   # certainty -> no review needed


def test_flags_gray_zone_merge():
    a = _t(1, 1, {i: (0.40, 0.60, 0.05, 0.05) for i in range(40)})
    b = _t(2, 1, {i: (0.43, 0.62, 0.05, 0.05) for i in range(40)})  # ~0.3-0.7 overlap
    merged, cases = merge_and_flag([a, b], auto_iou=0.7, flag_iou=0.3, min_len=30)
    assert any(c.kind == "merge" for c in cases)
    # gray-zone pair is NOT auto-merged (conservative)
    assert len(merged) == 2


def test_flags_short_track_for_keep_drop():
    short = _t(9, 2, {i: (0.5, 0.47, 0.05, 0.05) for i in range(12)})  # < min_len
    long = _t(1, 1, {i: (0.1, 0.1, 0.05, 0.05) for i in range(40)})
    merged, cases = merge_and_flag([short, long], auto_iou=0.7, flag_iou=0.3, min_len=30)
    assert any(c.kind == "keep_short" and 9 in c.track_ids for c in cases)
    # short track is retained for now (decision deferred to human)
    assert any(t.track_id == 9 for t in merged)


def test_apply_decisions_merge_and_drop():
    a = _t(1, 1, {i: (0.40, 0.60, 0.05, 0.05) for i in range(40)})
    b = _t(2, 1, {i: (0.43, 0.62, 0.05, 0.05) for i in range(40)})
    short = _t(9, 2, {i: (0.5, 0.47, 0.05, 0.05) for i in range(12)})
    cases = [ReviewCase(kind="merge", frame=0, track_ids=(1, 2), boxes=[], reason="", score=0.5),
             ReviewCase(kind="keep_short", frame=0, track_ids=(9,), boxes=[], reason="", score=0.0)]
    decisions = {0: "merge", 1: "drop"}    # case index -> verdict
    out = apply_decisions([a, b, short], cases, decisions)
    ids = sorted(t.track_id for t in out)
    assert ids == [1]            # 1&2 merged into 1; short(9) dropped
```

- [ ] **Step 2: Run — FAIL**.

- [ ] **Step 3: Implement `dynamic_tiling/gt_review.py`**
```python
"""Offline track-graph cleanup with confidence tiers + human-decision apply.

Conservative: auto-merge only near-certain duplicate/fragment tracks (high IoU
over their shared frames); everything in the gray zone becomes a ReviewCase for
human verdict. Short tracks (< min_len) are flagged keep_short (not silently
dropped) so real but briefly-tracked objects (e.g. a distant car) survive review.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .gt_clean import GtTrack


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
class ReviewCase:
    kind: str                 # "merge" | "keep_short" | "swap"
    frame: int                # representative frame to render
    track_ids: tuple          # involved track ids
    boxes: list = field(default_factory=list)  # [(cls,id,bbox), ...] for rendering
    reason: str = ""
    score: float = 0.0        # overlap / confidence of the proposal


def _shared_overlap(a: GtTrack, b: GtTrack):
    """(n_shared_frames, mean_iou_over_shared, representative_frame)."""
    shared = sorted(set(a.frames) & set(b.frames))
    if not shared:
        return 0, 0.0, None
    ious = [_iou(a.frames[f], b.frames[f]) for f in shared]
    mean_iou = sum(ious) / len(ious)
    rep = shared[max(range(len(ious)), key=lambda i: ious[i])]
    return len(shared), mean_iou, rep


def _union(a: GtTrack, b: GtTrack) -> GtTrack:
    frames = dict(b.frames); frames.update(a.frames)  # prefer a on conflicts
    return GtTrack(cls=a.cls, track_id=min(a.track_id, b.track_id), frames=frames)


def merge_and_flag(tracks, *, auto_iou=0.7, flag_iou=0.3, min_len=30,
                   min_shared=5):
    """Return (merged_tracks, review_cases). Conservative auto-merge."""
    tracks = list(tracks)
    cases: list[ReviewCase] = []
    # --- auto-merge near-certain same-class duplicates (union-find style) ---
    parent = {t.track_id: t.track_id for t in tracks}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]; x = parent[x]
        return x

    by_id = {t.track_id: t for t in tracks}
    ids = [t.track_id for t in tracks]
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a, b = by_id[ids[i]], by_id[ids[j]]
            if a.cls != b.cls:
                continue
            n, miou, rep = _shared_overlap(a, b)
            if n < min_shared:
                continue
            if miou >= auto_iou:
                ra, rb = find(a.track_id), find(b.track_id)
                if ra != rb:
                    parent[max(ra, rb)] = min(ra, rb)
            elif miou >= flag_iou:
                cases.append(ReviewCase(
                    kind="merge", frame=rep, track_ids=(a.track_id, b.track_id),
                    boxes=[(a.cls, a.track_id, a.frames[rep]),
                           (b.cls, b.track_id, b.frames[rep])],
                    reason=f"shared={n} mean_iou={miou:.2f}", score=miou))
    # collapse auto-merge groups
    groups: dict = {}
    for tid in ids:
        groups.setdefault(find(tid), []).append(by_id[tid])
    merged = []
    for root, grp in groups.items():
        acc = grp[0]
        for other in grp[1:]:
            acc = _union(acc, other)
        merged.append(acc)
    # --- flag short tracks for keep/drop (don't auto-drop) ---
    for t in merged:
        if len(t.frames) < min_len:
            rep = min(t.frames)
            cases.append(ReviewCase(
                kind="keep_short", frame=rep, track_ids=(t.track_id,),
                boxes=[(t.cls, t.track_id, t.frames[rep])],
                reason=f"len={len(t.frames)} < min_len={min_len}",
                score=float(len(t.frames))))
    return merged, cases


def apply_decisions(tracks, cases, decisions):
    """decisions: {case_index: verdict}. verdict per kind:
      merge      -> 'merge' (apply) | 'keep'  (leave separate)
      keep_short -> 'keep'  (retain) | 'drop'  (remove track)
    Returns the final track list."""
    by_id = {t.track_id: t for t in tracks}
    drop: set = set()
    # process merges first
    for idx, case in enumerate(cases):
        verdict = decisions.get(idx)
        if case.kind == "merge" and verdict == "merge":
            a_id, b_id = case.track_ids
            if a_id in by_id and b_id in by_id:
                merged = _union(by_id[a_id], by_id[b_id])
                by_id.pop(b_id, None)
                by_id[merged.track_id] = merged
    for idx, case in enumerate(cases):
        verdict = decisions.get(idx)
        if case.kind == "keep_short" and verdict == "drop":
            drop.add(case.track_ids[0])
    return [t for tid, t in by_id.items() if tid not in drop]
```

- [ ] **Step 4: Run — PASS** (4 tests).
- [ ] **Step 5: Commit** `dynamic_tiling/gt_review.py dynamic_tiling/tests/test_gt_review.py` — `feat(gt): offline track-graph auto-merge + flag + apply-decisions`.

---

## Task 3: Review-case rendering (annotated PNG)

**Files:** Create `dynamic_tiling/gt_render_review.py`; Test `dynamic_tiling/tests/test_gt_render_review.py`

Render one annotated image per `ReviewCase`: the video frame with the candidate box(es) highlighted and a caption describing the proposed action. Unit-test the pure caption/geometry helper; the cv2 draw is exercised by the CLI run (Task 5/6), not unit tests.

- [ ] **Step 1: Failing test**
```python
# dynamic_tiling/tests/test_gt_render_review.py
from dynamic_tiling.gt_review import ReviewCase
from dynamic_tiling.gt_render_review import case_caption, norm_to_px


def test_case_caption_merge():
    c = ReviewCase(kind="merge", frame=868, track_ids=(168, 193), boxes=[],
                   reason="shared=40 mean_iou=0.55", score=0.55)
    cap = case_caption(c, index=0)
    assert "MERGE" in cap.upper() and "168" in cap and "193" in cap and "868" in cap


def test_case_caption_keep_short():
    c = ReviewCase(kind="keep_short", frame=12, track_ids=(9,), boxes=[],
                   reason="len=12 < min_len=30", score=12.0)
    cap = case_caption(c, index=3)
    assert "9" in cap and ("KEEP" in cap.upper() or "REAL" in cap.upper())


def test_norm_to_px():
    assert norm_to_px((0.5, 0.5, 0.1, 0.2), 1920, 1080) == (960, 540, 192, 216)
```

- [ ] **Step 2: Run — FAIL**.

- [ ] **Step 3: Implement `dynamic_tiling/gt_render_review.py`**
```python
"""Render annotated review images for human GT verification."""
from __future__ import annotations

from pathlib import Path

from .gt_review import ReviewCase


def norm_to_px(bbox, w: int, h: int):
    x, y, bw, bh = bbox
    return (int(round(x * w)), int(round(y * h)),
            int(round(bw * w)), int(round(bh * h)))


def case_caption(case: ReviewCase, *, index: int) -> str:
    ids = "+".join(str(i) for i in case.track_ids)
    if case.kind == "merge":
        return (f"[{index}] PROPOSED MERGE  ids {ids}  frame {case.frame}  "
                f"({case.reason}) -> verdict: merge / keep-separate")
    if case.kind == "keep_short":
        return (f"[{index}] REAL OBJECT? short track id {ids}  frame {case.frame}  "
                f"({case.reason}) -> verdict: keep / drop")
    return (f"[{index}] {case.kind.upper()} ids {ids} frame {case.frame} ({case.reason})")


_COLORS = [(0, 0, 255), (0, 200, 0), (255, 128, 0), (200, 0, 200)]


def render_case(frame_bgr, case: ReviewCase, *, index: int):
    """Draw the candidate boxes + caption onto a copy of frame_bgr. Returns the image."""
    import cv2
    img = frame_bgr.copy()
    h, w = img.shape[:2]
    for k, (cls, tid, bbox) in enumerate(case.boxes):
        x, y, bw, bh = norm_to_px(bbox, w, h)
        col = _COLORS[k % len(_COLORS)]
        cv2.rectangle(img, (x, y), (x + bw, y + bh), col, 2)
        cv2.putText(img, f"id{tid}", (x, max(0, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    cap = case_caption(case, index=index)
    cv2.rectangle(img, (0, 0), (w, 26), (0, 0, 0), -1)
    cv2.putText(img, cap[:140], (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    return img


def render_queue(frames_by_idx, cases, out_dir: Path):
    """frames_by_idx: {frame_idx: bgr image}. Writes case_NNNN.png per case.
    Returns list of written paths."""
    import cv2
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for i, case in enumerate(cases):
        frame = frames_by_idx.get(case.frame)
        if frame is None:
            continue
        img = render_case(frame, case, index=i)
        p = out_dir / f"case_{i:04d}.png"
        cv2.imwrite(str(p), img)
        paths.append(p)
    return paths
```

- [ ] **Step 4: Run — PASS** (3 tests).
- [ ] **Step 5: Commit** `dynamic_tiling/gt_render_review.py dynamic_tiling/tests/test_gt_render_review.py` — `feat(gt): render annotated review images for human GT verification`.

---

## Task 4: Track-ID-coloured overlay

**Files:** Modify `dynamic_tiling/run_gt_tracks.py`; Test `dynamic_tiling/tests/test_run_gt_tracks.py` (extend)

The overlay must show *tracking*: colour + label each box by `track_id` (deterministic colour per id) so splits/merges are visible. Add `_overlay_doc_by_id(tracks)` that puts `track_id` into the label and a per-id colour hint, keeping the existing class overlay too.

- [ ] **Step 1: Failing test** (append to `test_run_gt_tracks.py`)
```python
def test_overlay_by_id_labels_track_id():
    from dynamic_tiling.gt_clean import GtTrack
    from dynamic_tiling.run_gt_tracks import overlay_doc_by_id
    tracks = [GtTrack(cls=1, track_id=7, frames={0: (0.1, 0.2, 0.3, 0.4)})]
    doc = overlay_doc_by_id(tracks)
    det = doc["frames"][0]["detections"][0]
    assert det["track_id"] == 7
    assert "7" in det["label"]          # label carries the id so the viewer shows it
```

- [ ] **Step 2: Run — FAIL**.

- [ ] **Step 3: Implement** — add to `run_gt_tracks.py`:
```python
def overlay_doc_by_id(tracks, label: str = "gt-by-id") -> dict:
    """Overlay where each box's label is '<class>#<track_id>' so the viewer shows
    track identity (not just class)."""
    from hailo_tiling.classes import label as cls_label
    per_frame: dict = {}
    for t in tracks:
        name = f"{cls_label(t.cls)}#{t.track_id}"
        for f, b in t.frames.items():
            per_frame.setdefault(f, []).append(
                {"label": name, "confidence": 1.0, "bbox": list(b),
                 "track_id": t.track_id})
    frames = [{"frame": f, "detections": per_frame[f], "tiles": []}
              for f in sorted(per_frame)]
    return {"label": label, "frames": frames}
```

- [ ] **Step 4: Run — PASS**.
- [ ] **Step 5: Commit** `dynamic_tiling/run_gt_tracks.py dynamic_tiling/tests/test_run_gt_tracks.py` — `feat(gt): track-id-labelled overlay so tracking is visible`.

---

## Task 5: `run_gt_verify` CLI (build queue + finalize)

**Files:** Create `dynamic_tiling/run_gt_verify.py`; Modify `dynamic_tiling/gt_mot.py`; Test `dynamic_tiling/tests/test_run_gt_verify.py`

Orchestrate the loop. Two modes: default (build) runs dedup→track→clean→merge_and_flag→write `gt_tracks.json` (current clean) + `review_queue.json` + render PNGs + write the by-id overlay. `--apply-decisions FILE` runs the finalize pass: load tracks + cases + decisions, `apply_decisions`, write `gt_tracks.verified.json` + verified overlay.

`review_queue.json` schema: `{"cases":[{"kind","frame","track_ids":[...],"reason","score"}]}`. `review_decisions.json` schema: `{"<case_index>": "<verdict>"}`.

- [ ] **Step 1: Failing test** (pure serialization + decisions round-trip, no boxmot/video)
```python
# dynamic_tiling/tests/test_run_gt_verify.py
import json
from dynamic_tiling.gt_review import ReviewCase
from dynamic_tiling.run_gt_verify import cases_to_doc, doc_to_cases


def test_cases_doc_roundtrip():
    cases = [ReviewCase(kind="merge", frame=868, track_ids=(168, 193),
                        boxes=[], reason="x", score=0.55)]
    doc = cases_to_doc(cases)
    back = doc_to_cases(json.loads(json.dumps(doc)))
    assert back[0].kind == "merge" and back[0].track_ids == (168, 193)
    assert back[0].frame == 868
```

- [ ] **Step 2: Run — FAIL**.

- [ ] **Step 3: Implement `dynamic_tiling/gt_mot.py` change** — add an optional dedup hook so the video pass can dedup each frame's dense dets before tracking:
In `build_raw_tracks_from_video`, accept `dedup_iou: float | None = None`; when set, replace the per-frame `for d in fr.get("detections", [])` extraction by first running `from .gt_dedup import dedup_frame; dets_in = dedup_frame(fr.get("detections", []), iou_thr=dedup_iou)` and iterating `dets_in`. Leave behaviour unchanged when `dedup_iou is None`.

- [ ] **Step 4: Implement `dynamic_tiling/run_gt_verify.py`**
```python
"""CLI: human-in-the-loop GT verification.

Build mode (default):
    .venv_gt/bin/python -m dynamic_tiling.run_gt_verify \
        --dense /tmp/gt_legacy/12x9.frames.json --video <clip.mp4> \
        --outdir dynamic_tiling/runs/gt_verify_0026_fov50

Finalize mode:
    python -m dynamic_tiling.run_gt_verify --finalize \
        --outdir dynamic_tiling/runs/gt_verify_0026_fov50 \
        --decisions dynamic_tiling/runs/gt_verify_0026_fov50/review_decisions.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .gt_clean import clean_tracks
from .gt_review import merge_and_flag, apply_decisions, ReviewCase
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id


def cases_to_doc(cases) -> dict:
    return {"cases": [{"kind": c.kind, "frame": c.frame,
                       "track_ids": list(c.track_ids), "reason": c.reason,
                       "score": c.score} for c in cases]}


def doc_to_cases(doc) -> list:
    return [ReviewCase(kind=c["kind"], frame=c["frame"],
                       track_ids=tuple(c["track_ids"]), boxes=[],
                       reason=c.get("reason", ""), score=c.get("score", 0.0))
            for c in doc["cases"]]


def _read_frames(video, frame_idxs):
    import cv2
    want = set(frame_idxs)
    out = {}
    cap = cv2.VideoCapture(str(video))
    fi = -1
    try:
        while want:
            ok, fr = cap.read()
            if not ok:
                break
            fi += 1
            if fi in want:
                out[fi] = fr
                want.discard(fi)
    finally:
        cap.release()
    return out


def _build(args):
    from .gt_mot import build_raw_tracks_from_video, make_botsort
    from .gt_render_review import render_queue
    doc = json.loads(Path(args.dense).read_text())
    raw = build_raw_tracks_from_video(doc, str(args.video),
                                      tracker_factory=make_botsort,
                                      dedup_iou=args.dedup_iou)
    clean = clean_tracks(raw, max_gap=args.max_gap, min_len=1)  # keep short; flag later
    merged, cases = merge_and_flag(clean, auto_iou=args.auto_iou,
                                   flag_iou=args.flag_iou, min_len=args.min_len)
    # attach boxes for rendering (rep frame already chosen)
    by_id = {t.track_id: t for t in merged}
    for c in cases:
        c.boxes = [(by_id[i].cls, i, by_id[i].frames[c.frame])
                   for i in c.track_ids if i in by_id and c.frame in by_id[i].frames]
    out = Path(args.outdir); out.mkdir(parents=True, exist_ok=True)
    (out / "gt_tracks.json").write_text(json.dumps(tracks_to_doc(merged, clip=Path(args.video).stem)))
    (out / "review_queue.json").write_text(json.dumps(cases_to_doc(cases), indent=2))
    (out / "overlay_by_id.frames.json").write_text(json.dumps(overlay_doc_by_id(merged)))
    frames = _read_frames(args.video, [c.frame for c in cases])
    paths = render_queue(frames, cases, out / "review")
    print(f"tracks: {len(merged)} | review cases: {len(cases)} | images: {len(paths)}")
    print(f"queue : {out/'review_queue.json'}")
    print(f"images: {out/'review'}")


def _finalize(args):
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / "gt_tracks.json").read_text()))
    cases = doc_to_cases(json.loads((out / "review_queue.json").read_text()))
    decisions = {int(k): v for k, v in json.loads(Path(args.decisions).read_text()).items()}
    final = apply_decisions(tracks, cases, decisions)
    # drop tracks still under min_len that the human did not explicitly keep
    final_doc = tracks_to_doc(final, clip=out.name)
    (out / "gt_tracks.verified.json").write_text(json.dumps(final_doc))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(final)))
    print(f"verified tracks: {len(final)}")
    print(f"verified GT : {out/'gt_tracks.verified.json'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--dense", type=Path)
    ap.add_argument("--video", type=Path)
    ap.add_argument("--dedup-iou", type=float, default=0.5)
    ap.add_argument("--max-gap", type=int, default=5)
    ap.add_argument("--min-len", type=int, default=30)
    ap.add_argument("--auto-iou", type=float, default=0.7)
    ap.add_argument("--flag-iou", type=float, default=0.3)
    ap.add_argument("--finalize", action="store_true")
    ap.add_argument("--decisions", type=Path)
    args = ap.parse_args()
    if args.finalize:
        _finalize(args)
    else:
        _build(args)


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: Run the round-trip test — PASS**. Then full suite `python -m pytest dynamic_tiling/tests/ -q` — all pass.
- [ ] **Step 6: Commit** `dynamic_tiling/run_gt_verify.py dynamic_tiling/gt_mot.py dynamic_tiling/tests/test_run_gt_verify.py` — `feat(gt): run_gt_verify CLI — build review queue + finalize with human decisions`.

---

## Task 6: Run the build pass on 0026 fov50 (controller, on `.venv_gt`)

Not a code task — the controller runs this after Tasks 1–5 land, to produce the review batch for the human.

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
.venv_gt/bin/python -m dynamic_tiling.run_gt_verify \
  --dense /tmp/gt_legacy/12x9.frames.json \
  --video /home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4 \
  --outdir dynamic_tiling/runs/gt_verify_0026_fov50
```
Expected: prints track count + review-case count + image count; writes `review_queue.json`, `review/case_*.png`, `overlay_by_id.frames.json`. The controller then shows the case images to the human, collects verdicts into `review_decisions.json`, and runs `--finalize`.

---

## Self-Review notes
- Dedup (Task 1) attacks the duplicate boxes at the source; track-merge (Task 2) cleans residual; both conservative. The white-car miss is handled by `keep_short` flags (min_len=1 in clean, flag < min_len) so real short objects reach human review instead of being dropped.
- `apply_decisions` processes merges before drops; case indices are the contract between `review_queue.json` and `review_decisions.json`.
- Unit tests stay in the shared venv (pure Python); only Task 6 needs `.venv_gt` + chip-free boxmot.
