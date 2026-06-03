from dynamic_tiling.gt_clean import GtTrack
from dynamic_tiling.gt_edit import (
    clip_frame_range,
    despike_track_heights,
    drop_tracks,
    hold_track_tail,
    interp_track_gaps,
    pin_static_tracks,
)


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


def test_despike_restores_truncated_heights_anchored_at_top():
    # steady person h=0.07 (top-left ymin=0.7), two truncated frames (legs cut)
    frames = {i: (0.5, 0.7, 0.02, 0.07) for i in range(41)}
    frames[20] = (0.5, 0.7, 0.02, 0.03)   # truncated: only top boxed
    frames[21] = (0.5, 0.7, 0.02, 0.025)
    out = despike_track_heights([_t(3, 1, frames)], track_ids=[3],
                                min_ratio=0.7, window=31)
    got = out[0].frames
    # truncated frames restored to local median height (0.07)
    assert got[20] == (0.5, 0.7, 0.02, 0.07)
    assert got[21] == (0.5, 0.7, 0.02, 0.07)
    # top edge (ymin) and width preserved -> box grows DOWNWARD
    assert got[20][1] == 0.7 and got[20][2] == 0.02
    # a healthy frame is untouched
    assert got[10] == (0.5, 0.7, 0.02, 0.07)


def test_despike_converges_on_gradual_dip():
    # a gradual V-shaped truncation: fixing the bottom raises the local median
    # and would expose the shoulders on a single pass -> must converge.
    frames = {i: (0.5, 0.7, 0.02, 0.07) for i in range(61)}
    dip = {28: 0.060, 29: 0.050, 30: 0.038, 31: 0.050, 32: 0.060}
    for f, h in dip.items():
        frames[f] = (0.5, 0.7, 0.02, h)
    out = despike_track_heights([_t(3, 1, frames)], track_ids=[3],
                                min_ratio=0.8, window=31)
    fr = out[0].frames
    import statistics as _st
    fis = sorted(fr); H = [fr[f][3] for f in fis]; half = 15
    # after convergence NO frame is below 0.8 * its local median
    assert all(fr[f][3] >= 0.8 * _st.median(H[max(0, i - half):i + half + 1])
               for i, f in enumerate(fis))


def test_despike_only_touches_selected_tracks_and_respects_ratio():
    short = {i: (0.5, 0.7, 0.02, 0.07) for i in range(41)}
    short[20] = (0.5, 0.7, 0.02, 0.062)   # 0.886*median -> above 0.7 ratio, keep
    p3 = _t(3, 1, short)
    p4 = _t(4, 1, {i: (0.1, 0.1, 0.02, 0.03) for i in range(41)})  # not selected
    out = despike_track_heights([p3, p4], track_ids=[3], min_ratio=0.7, window=31)
    got3 = next(t for t in out if t.track_id == 3).frames
    assert got3[20] == (0.5, 0.7, 0.02, 0.062)  # within ratio -> unchanged
    got4 = next(t for t in out if t.track_id == 4).frames
    assert got4 == p4.frames  # untouched track identical


def test_drop_tracks_removes_only_listed_ids():
    a = _t(1, 2, {0: (0, 0, 1, 1)})
    b = _t(6, 2, {0: (0, 0, 1, 1)})
    c = _t(2, 1, {0: (0, 0, 1, 1)})
    out = drop_tracks([a, b, c], track_ids=[6])
    assert [t.track_id for t in out] == [1, 2]


def test_drop_tracks_empty_list_is_noop():
    a = _t(1, 2, {0: (0, 0, 1, 1)})
    out = drop_tracks([a], track_ids=[])
    assert [t.track_id for t in out] == [1]


def test_hold_track_tail_repeats_last_bbox_to_end():
    last = (0.4, 0.3, 0.05, 0.06)
    p4 = _t(4, 1, {5: (0.1, 0.1, 0.05, 0.06), 8: last})  # ends at frame 8
    other = _t(3, 1, {0: (0.2, 0.2, 0.05, 0.06)})
    out = hold_track_tail([p4, other], track_ids=[4], until_frame=11)
    got = next(t for t in out if t.track_id == 4).frames
    # frames 9,10,11 added, all equal to the last (frame-8) bbox
    assert sorted(got) == [5, 8, 9, 10, 11]
    assert got[9] == last and got[10] == last and got[11] == last
    # original frames untouched; out-of-ids track untouched
    assert got[5] == (0.1, 0.1, 0.05, 0.06)
    assert next(t for t in out if t.track_id == 3).frames == other.frames


def test_hold_track_tail_noop_when_until_not_past_last():
    p4 = _t(4, 1, {5: (0.1, 0.1, 0.05, 0.06), 8: (0.4, 0.3, 0.05, 0.06)})
    out = hold_track_tail([p4], track_ids=[4], until_frame=8)
    assert sorted(out[0].frames) == [5, 8]  # nothing added


def test_pin_requires_ref_frame_present():
    car = _t(1, 2, {0: (0.1, 0.1, 0.2, 0.2)})
    try:
        pin_static_tracks([car], track_ids=[1], ref_frame=99, frame_range=(0, 3))
        assert False, "expected error"
    except (KeyError, ValueError):
        pass
