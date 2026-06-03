"""Reproducible GT corrections (replayable, not hand-edits)."""
from __future__ import annotations

from dataclasses import replace

from .gt_clean import GtTrack


def clip_frame_range(tracks):
    """(min_frame, max_frame) inclusive over all tracks' frames."""
    frames = [f for t in tracks for f in t.frames]
    return (min(frames), max(frames)) if frames else (0, 0)


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
