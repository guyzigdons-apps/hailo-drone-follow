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


def despike_track_heights(tracks, *, track_ids, min_ratio=0.7, window=31, max_iter=20):
    """Restore detector-truncated bbox heights on each track in track_ids.

    When a detection boxes only the top of an object (legs cut off), its height
    collapses while the top edge (ymin) stays put. For each frame whose height
    is below ``min_ratio`` * the local windowed-median height, the height is
    reset to that local median while x, ymin and width are preserved -- so the
    box grows back DOWNWARD to the feet. ``window`` (odd) sets the centered
    median window so genuine scale change (object walking nearer/farther) is
    tracked rather than flattened.

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
                    frames[f] = (x, y, w, local_med)
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
