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
