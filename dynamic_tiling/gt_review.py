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
    frame: int
    track_ids: tuple
    boxes: list = field(default_factory=list)  # [(cls,id,bbox), ...] for rendering
    reason: str = ""
    score: float = 0.0


def _shared_overlap(a: GtTrack, b: GtTrack):
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


def merge_and_flag(tracks, *, auto_iou=0.7, flag_iou=0.3, min_len=30, min_shared=5):
    """Return (merged_tracks, review_cases). Conservative auto-merge."""
    tracks = list(tracks)
    cases: list[ReviewCase] = []
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
    groups: dict = {}
    for tid in ids:
        groups.setdefault(find(tid), []).append(by_id[tid])
    merged = []
    for root, grp in groups.items():
        acc = grp[0]
        for other in grp[1:]:
            acc = _union(acc, other)
        merged.append(acc)
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
    """decisions: {case_index: verdict}.
      merge      -> 'merge' (apply) | 'keep' (leave separate)
      keep_short -> 'keep' (retain) | 'drop' (remove track)
    Returns the final track list."""
    by_id = {t.track_id: t for t in tracks}
    drop: set = set()
    for idx, case in enumerate(cases):
        if case.kind == "merge" and decisions.get(idx) == "merge":
            a_id, b_id = case.track_ids
            if a_id in by_id and b_id in by_id:
                merged = _union(by_id[a_id], by_id[b_id])
                by_id.pop(b_id, None)
                by_id[merged.track_id] = merged
    for idx, case in enumerate(cases):
        if case.kind == "keep_short" and decisions.get(idx) == "drop":
            drop.add(case.track_ids[0])
    return [t for tid, t in by_id.items() if tid not in drop]
