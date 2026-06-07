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
