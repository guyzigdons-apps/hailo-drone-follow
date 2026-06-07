"""Reproducible GT corrections (replayable, not hand-edits)."""
from __future__ import annotations

import statistics
from dataclasses import replace

from .gt_clean import GtTrack, interpolate_gaps


def clip_frame_range(tracks):
    """(min_frame, max_frame) inclusive over all tracks' frames."""
    frames = [f for t in tracks for f in t.frames]
    return (min(frames), max(frames)) if frames else (0, 0)


def interp_track_gaps(tracks, *, track_ids, max_gap):
    """Linearly interpolate in-span frame gaps of length <= max_gap for each
    track in track_ids (fixes detection-dropout flicker on a track that is
    genuinely present throughout its span). Tracks outside track_ids pass
    through unchanged. Out-of-span absence (before a track's first frame /
    after its last) is never filled."""
    ids = set(track_ids)
    return [interpolate_gaps(t, max_gap=max_gap) if t.track_id in ids else t
            for t in tracks]


def despike_track_heights(tracks, *, track_ids, min_ratio=0.7, window=31,
                          max_iter=20, anchor="top"):
    """Restore detector-truncated bbox heights on each track in track_ids.

    When a detection boxes only part of an object its height collapses while one
    edge stays put. For each frame whose height is below ``min_ratio`` * the
    local windowed-median height, the height is reset to that local median while
    x and width are preserved and one vertical edge is held:

    - ``anchor="top"`` (default) holds the top edge (ymin) -- the legs were cut,
      so the box grows back DOWNWARD to the feet.
    - ``anchor="bottom"`` holds the bottom edge (ymin + h) -- the head was cut,
      so the box grows back UPWARD over the head.

    ``window`` (odd) sets the centered median window so genuine scale change
    (object walking nearer/farther) is tracked rather than flattened.

    Applied iteratively to convergence: restoring one frame raises the local
    median, which can expose the shoulders of a gradual dip, so the pass repeats
    until no frame is below the threshold (or ``max_iter`` is hit). Heights only
    ever rise (capped at the local median), so this terminates. Tracks outside
    track_ids pass through unchanged. Assumes (xmin, ymin, w, h) top-left
    normalized bboxes."""
    ids = set(track_ids)
    half = window // 2
    out = []
    for t in tracks:
        if t.track_id not in ids:
            out.append(t)
            continue
        fis = sorted(t.frames)
        frames = dict(t.frames)
        for _ in range(max_iter):
            heights = [frames[f][3] for f in fis]
            changed = False
            for i, f in enumerate(fis):
                local_med = statistics.median(heights[max(0, i - half):i + half + 1])
                x, y, w, h = frames[f]
                if h < min_ratio * local_med:
                    ny = y + h - local_med if anchor == "bottom" else y
                    frames[f] = (x, ny, w, local_med)
                    changed = True
            if not changed:
                break
        out.append(replace(t, frames=frames))
    return out


def remap_track_ids(tracks, *, mapping):
    """Rename track ids per ``mapping`` (old_id -> new_id); ids absent from the
    mapping keep their value. Used to align ids to a canonical scheme across
    clips/fovs (same physical object -> same id everywhere). Raises ValueError
    if the result would have duplicate ids."""
    out = [replace(t, track_id=mapping.get(t.track_id, t.track_id)) for t in tracks]
    ids = [t.track_id for t in out]
    if len(set(ids)) != len(ids):
        raise ValueError(f"remap produced duplicate track ids: {sorted(ids)}")
    return out


def project_bbox(bbox, sx, sy):
    """Project a normalized (xmin, ymin, w, h) bbox from one center-crop FOV to
    another. Both FOVs are centered crops of the same source scaled to the same
    output, so normalized coords map affinely about the center (0.5): a point
    moves toward/away from 0.5 by the scale factor and the box scales likewise.
    ``sx = crop_w(src)/crop_w(dst)``, ``sy = crop_h(src)/crop_h(dst)``."""
    x, y, w, h = bbox
    return (0.5 + (x - 0.5) * sx, 0.5 + (y - 0.5) * sy, w * sx, h * sy)


def crossfov_fill_track(tracks, *, track_id, source_frames, sx, sy):
    """Replace the target track's frames with ``source_frames`` (a frame->bbox
    dict from another FOV's GT) projected into this FOV via project_bbox. Used
    to fill an unstable/short track from the same object's clean track in a
    different FOV of the same clip (same frames, exact geometry). Raises
    ValueError if track_id is not present."""
    projected = {f: project_bbox(b, sx, sy) for f, b in source_frames.items()}
    out, found = [], False
    for t in tracks:
        if t.track_id == track_id:
            out.append(replace(t, frames=projected))
            found = True
        else:
            out.append(t)
    if not found:
        raise ValueError(f"crossfov_fill: target track {track_id} not found")
    return out


def drift_extend_track(tracks, *, track_id, ref_frames, frame_range):
    """Extend a static object's track into frames outside its detected span by
    propagating camera drift from a reference track. For each frame in
    frame_range not already present, the target's nearest-end bbox is translated
    by the reference track's motion between that frame and the anchor frame
    (x, y shifted; w, h kept). Used when a parked object is present but
    undetected for part of the clip and a frozen box would mis-track the drift.
    A frame is skipped if the reference lacks it or the anchor frame. Raises
    ValueError if track_id is absent."""
    target = next((t for t in tracks if t.track_id == track_id), None)
    if target is None:
        raise ValueError(f"drift_extend: target track {track_id} not found")
    fis = sorted(target.frames)
    first, last = fis[0], fis[-1]
    new = dict(target.frames)
    lo, hi = frame_range
    for f in range(lo, hi + 1):
        if f in new:
            continue
        anchor = first if f < first else last
        if f not in ref_frames or anchor not in ref_frames:
            continue
        ax, ay, aw, ah = target.frames[anchor]
        dx = ref_frames[f][0] - ref_frames[anchor][0]
        dy = ref_frames[f][1] - ref_frames[anchor][1]
        new[f] = (ax + dx, ay + dy, aw, ah)
    return [replace(t, frames=new) if t.track_id == track_id else t for t in tracks]


def plan_gap_recovery(frames):
    """Predict (cx, cy, w, h) for every frame missing INSIDE a track's span, by
    linear interpolation of the bounding present detections. Used to seed a
    zoomed re-detection pass: each predicted centre is where to place the 2x ROI
    so the detector can recover a small target the dense pass missed. Returns
    {frame -> (cx, cy, w, h)} (centre-normalized). Empty if <2 present frames."""
    fis = sorted(frames)
    plan = {}
    for a, b in zip(fis, fis[1:]):
        if b - a <= 1:
            continue
        xa, ya, wa, ha = frames[a]
        xb, yb, wb, hb = frames[b]
        cax, cay = xa + wa / 2, ya + ha / 2
        cbx, cby = xb + wb / 2, yb + hb / 2
        for f in range(a + 1, b):
            t = (f - a) / (b - a)
            plan[f] = (cax + (cbx - cax) * t, cay + (cby - cay) * t,
                       wa + (wb - wa) * t, ha + (hb - ha) * t)
    return plan


def merge_tracks(tracks, *, groups):
    """Merge each group of track ids into a single track (same physical object
    that the tracker ID-split). The merged track keeps the min id of its group,
    the cls of the first-listed member, and the union of frames; on a frame
    conflict the earlier-listed member wins. Tracks in no group pass through.
    Order is not preserved."""
    by_id = {t.track_id: t for t in tracks}
    grouped = {tid for g in groups for tid in g}
    out = [t for t in tracks if t.track_id not in grouped]
    for group in groups:
        members = [by_id[i] for i in group if i in by_id]
        if not members:
            continue
        frames = {}
        for m in members:
            for f, b in m.frames.items():
                frames.setdefault(f, b)  # first-listed member wins on conflict
        out.append(replace(members[0], track_id=min(group), frames=frames))
    return out


def drop_tracks(tracks, *, track_ids):
    """Remove the tracks whose ids are in track_ids (spurious / false-positive
    trajectories rejected during human review). All others pass through."""
    ids = set(track_ids)
    return [t for t in tracks if t.track_id not in ids]


def hold_track_tail(tracks, *, track_ids, until_frame):
    """For each track in track_ids, repeat its last-frame bbox forward through
    ``until_frame`` inclusive (the object is still present at the clip end but
    the detector dropped it). No-op for a track whose last frame is already at
    or past until_frame. Tracks outside track_ids pass through unchanged."""
    ids = set(track_ids)
    out = []
    for t in tracks:
        if t.track_id not in ids or not t.frames:
            out.append(t)
            continue
        last = max(t.frames)
        box = t.frames[last]
        frames = dict(t.frames)
        for f in range(last + 1, until_frame + 1):
            frames[f] = box
        out.append(replace(t, frames=frames))
    return out


def _percentile(values, p):
    """Linear-interpolated p-th percentile (0..100) of a non-empty list."""
    vals = sorted(values)
    if len(vals) == 1:
        return vals[0]
    k = (len(vals) - 1) * p / 100.0
    lo = int(k)
    frac = k - lo
    hi = min(lo + 1, len(vals) - 1)
    return vals[lo] + (vals[hi] - vals[lo]) * frac


def restore_track_width(tracks, *, track_ids, anchor="left", percentile=90,
                        min_ratio=0.85, window=151, max_iter=20):
    """Restore the width of a track that is intermittently occluded on one side
    (e.g. a parked car partly hidden behind another). For each frame whose width
    is below ``min_ratio`` * the local windowed ``percentile``-th width (the
    unoccluded width), the width is reset to that target while the stable edge
    is held: ``anchor='left'`` keeps xmin, ``'right'`` keeps xmax, ``'center'``
    keeps the centre; y and h are untouched. Applied iteratively to convergence.
    Tracks outside track_ids pass through unchanged. (xmin, ymin, w, h) bboxes."""
    ids = set(track_ids)
    half = window // 2
    out = []
    for t in tracks:
        if t.track_id not in ids:
            out.append(t)
            continue
        fis = sorted(t.frames)
        frames = dict(t.frames)
        for _ in range(max_iter):
            widths = [frames[f][2] for f in fis]
            changed = False
            for i, f in enumerate(fis):
                target = _percentile(widths[max(0, i - half):i + half + 1], percentile)
                x, y, w, h = frames[f]
                if w < min_ratio * target:
                    if anchor == "right":
                        nx = x + w - target
                    elif anchor == "center":
                        nx = x + w / 2.0 - target / 2.0
                    else:  # left
                        nx = x
                    frames[f] = (nx, y, target, h)
                    changed = True
            if not changed:
                break
        out.append(replace(t, frames=frames))
    return out


def smooth_track_bbox(tracks, *, track_ids, window=15, dims=("x", "w")):
    """Centered moving-average smoothing of selected bbox components over time,
    to remove per-frame detector jitter while preserving slow motion (e.g. a
    static car's camera drift). ``dims`` selects which of x/y/w/h to smooth
    (default horizontal only: x position + width). The window shrinks at the
    track ends. Assumes a (near-)contiguous track (window is by frame-index
    position). Tracks outside track_ids pass through unchanged."""
    comp = {"x": 0, "y": 1, "w": 2, "h": 3}
    sel = [comp[d] for d in dims]
    half = window // 2
    ids = set(track_ids)
    out = []
    for t in tracks:
        if t.track_id not in ids:
            out.append(t)
            continue
        fis = sorted(t.frames)
        raw = [list(t.frames[f]) for f in fis]
        smoothed = [list(b) for b in raw]
        for i in range(len(fis)):
            lo, hi = max(0, i - half), min(len(fis), i + half + 1)
            n = hi - lo
            for c in sel:
                smoothed[i][c] = sum(raw[j][c] for j in range(lo, hi)) / n
        out.append(replace(t, frames={f: tuple(smoothed[i]) for i, f in enumerate(fis)}))
    return out


def pin_static_tracks(tracks, *, track_ids, ref_frame, frame_range):
    """For each track in track_ids, replace its frames with the ref_frame bbox
    held constant across frame_range inclusive (for static objects under a fixed
    camera). Other tracks pass through unchanged. Raises if ref_frame absent
    from a targeted track."""
    ids = set(track_ids)
    lo, hi = frame_range
    out = []
    for t in tracks:
        if t.track_id in ids:
            if ref_frame not in t.frames:
                raise ValueError(f"track {t.track_id} has no ref_frame {ref_frame}")
            box = t.frames[ref_frame]
            out.append(replace(t, frames={f: box for f in range(lo, hi + 1)}))
        else:
            out.append(t)
    return out
