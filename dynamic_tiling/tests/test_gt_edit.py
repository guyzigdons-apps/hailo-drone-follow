from dynamic_tiling.gt_clean import GtTrack
from dynamic_tiling.gt_edit import clip_frame_range, interp_track_gaps, pin_static_tracks


def _t(tid, cls, frames):
    return GtTrack(cls=cls, track_id=tid, frames=frames)


def test_clip_frame_range_spans_all_tracks():
    a = _t(1, 2, {0: (0, 0, 1, 1), 5: (0, 0, 1, 1)})
    b = _t(2, 1, {3: (0, 0, 1, 1), 9: (0, 0, 1, 1)})
    assert clip_frame_range([a, b]) == (0, 9)


def test_pin_fills_constant_bbox_over_full_range():
    car = _t(1, 2, {0: (0.1, 0.1, 0.2, 0.2), 5: (0.11, 0.1, 0.2, 0.2)})  # has a gap (1-4) + drifts
    person = _t(3, 1, {i: (0.5, 0.5, 0.1, 0.1) for i in range(6)})
    out = pin_static_tracks([car, person], track_ids=[1], ref_frame=5, frame_range=(0, 5))
    pinned = next(t for t in out if t.track_id == 1)
    # every frame 0..5 present, all equal to the ref-frame (5) bbox
    assert sorted(pinned.frames) == [0, 1, 2, 3, 4, 5]
    assert all(b == (0.11, 0.1, 0.2, 0.2) for b in pinned.frames.values())
    # untouched track unchanged
    assert next(t for t in out if t.track_id == 3).frames == person.frames


def test_interp_fills_in_span_gaps_for_selected_tracks():
    # person 3: one 5-frame gap (5->11); person 4: untouched-id stays raw
    p3 = _t(3, 1, {0: (0.0, 0.0, 0.1, 0.1), 5: (0.5, 0.5, 0.1, 0.1),
                   11: (1.0, 1.0, 0.1, 0.1)})
    p4 = _t(4, 1, {0: (0.2, 0.2, 0.1, 0.1), 8: (0.9, 0.9, 0.1, 0.1)})  # gap of 8
    out = interp_track_gaps([p3, p4], track_ids=[3], max_gap=8)
    got3 = next(t for t in out if t.track_id == 3)
    # gap 5->11 (size 6) filled; gap 0->5 (size 5) filled too (both <= 8)
    assert sorted(got3.frames) == list(range(12))
    # linear midpoint check at frame 8 (between 5 and 11): frac 3/6 -> 0.75
    assert got3.frames[8] == (0.75, 0.75, 0.1, 0.1)
    # track 4 NOT in track_ids -> untouched (gap remains)
    got4 = next(t for t in out if t.track_id == 4)
    assert sorted(got4.frames) == [0, 8]


def test_interp_respects_max_gap():
    p = _t(3, 1, {0: (0.0, 0.0, 0.1, 0.1), 10: (1.0, 1.0, 0.1, 0.1)})  # gap 10
    out = interp_track_gaps([p], track_ids=[3], max_gap=5)
    # gap (10) exceeds max_gap (5) -> not filled
    assert sorted(out[0].frames) == [0, 10]


def test_pin_requires_ref_frame_present():
    car = _t(1, 2, {0: (0.1, 0.1, 0.2, 0.2)})
    try:
        pin_static_tracks([car], track_ids=[1], ref_frame=99, frame_range=(0, 3))
        assert False, "expected error"
    except (KeyError, ValueError):
        pass
