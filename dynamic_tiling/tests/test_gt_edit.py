from dynamic_tiling.gt_clean import GtTrack
from dynamic_tiling.gt_edit import (
    clip_frame_range,
    crossfov_fill_track,
    despike_track_heights,
    drift_extend_track,
    drop_tracks,
    hold_track_tail,
    interp_track_gaps,
    pin_static_tracks,
    project_bbox,
    remap_track_ids,
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


def test_remap_track_ids_renames_per_mapping():
    a = _t(1, 2, {0: (0, 0, 1, 1)})
    b = _t(3, 2, {0: (0, 0, 1, 1)})
    c = _t(2, 1, {0: (0, 0, 1, 1)})
    out = remap_track_ids([a, b, c], mapping={1: 1, 3: 2, 2: 3})
    assert sorted((t.track_id, t.cls) for t in out) == [(1, 2), (2, 2), (3, 1)]


def test_remap_track_ids_rejects_id_collision():
    a = _t(1, 2, {0: (0, 0, 1, 1)})
    b = _t(2, 2, {0: (0, 0, 1, 1)})
    try:
        remap_track_ids([a, b], mapping={1: 5, 2: 5})  # both -> 5
        assert False, "expected collision error"
    except ValueError:
        pass


def test_project_bbox_affine_about_center():
    # at the exact center, projection is identity for position; size scales
    assert project_bbox((0.5, 0.5, 0.10, 0.20), sx=0.5, sy=0.5) == (0.5, 0.5, 0.05, 0.10)
    # off-center point pulled toward 0.5 by the scale factor
    x, y, w, h = project_bbox((0.7, 0.3, 0.10, 0.10), sx=0.5, sy=0.5)
    assert abs(x - (0.5 + 0.2 * 0.5)) < 1e-12  # 0.6
    assert abs(y - (0.5 - 0.2 * 0.5)) < 1e-12  # 0.4
    assert abs(w - 0.05) < 1e-12 and abs(h - 0.05) < 1e-12


def test_crossfov_fill_replaces_target_with_projected_source():
    # target track 4 has jittery native frames; replace with projected source
    tgt = _t(4, 1, {10: (0.9, 0.9, 0.02, 0.02)})
    keep = _t(3, 1, {0: (0.1, 0.1, 0.05, 0.05)})
    src_frames = {10: (0.7, 0.3, 0.10, 0.10), 11: (0.7, 0.3, 0.10, 0.10)}
    out = crossfov_fill_track([tgt, keep], track_id=4,
                              source_frames=src_frames, sx=0.5, sy=0.5)
    got = next(t for t in out if t.track_id == 4).frames
    assert sorted(got) == [10, 11]
    assert got[10] == project_bbox(src_frames[10], 0.5, 0.5)
    # other track untouched
    assert next(t for t in out if t.track_id == 3).frames == keep.frames


def test_crossfov_fill_requires_target_present():
    try:
        crossfov_fill_track([_t(3, 1, {0: (0, 0, 1, 1)})], track_id=99,
                            source_frames={0: (0.5, 0.5, 0.1, 0.1)}, sx=1.0, sy=1.0)
        assert False, "expected error"
    except ValueError:
        pass


def test_drift_extend_track_follows_reference_drift():
    # target present only 10..11; reference (a dense static object) drifts each frame
    tgt = _t(5, 2, {10: (0.40, 0.90, 0.05, 0.07), 11: (0.40, 0.90, 0.05, 0.07)})
    # ref moves left+up over time (camera drift); at f10 it's at (0.70,0.50)
    ref_frames = {f: (0.70 - 0.001 * (10 - f), 0.50 - 0.002 * (10 - f), 0.04, 0.04)
                  for f in range(0, 12)}
    out = drift_extend_track([tgt], track_id=5, ref_frames=ref_frames, frame_range=(8, 11))
    fr = next(t for t in out if t.track_id == 5).frames
    assert sorted(fr) == [8, 9, 10, 11]
    # frame 9 = anchor(10) box shifted by ref(9)-ref(10) drift
    dx = ref_frames[9][0] - ref_frames[10][0]
    dy = ref_frames[9][1] - ref_frames[10][1]
    assert abs(fr[9][0] - (0.40 + dx)) < 1e-12
    assert abs(fr[9][1] - (0.90 + dy)) < 1e-12
    assert fr[9][2] == 0.05 and fr[9][3] == 0.07  # size unchanged
    # existing native frames untouched
    assert fr[10] == (0.40, 0.90, 0.05, 0.07)


def test_drift_extend_track_skips_frames_missing_in_reference():
    tgt = _t(5, 2, {10: (0.4, 0.9, 0.05, 0.07)})
    ref_frames = {10: (0.7, 0.5, 0.04, 0.04)}  # ref has no frame 8/9
    out = drift_extend_track([tgt], track_id=5, ref_frames=ref_frames, frame_range=(8, 10))
    fr = out[0].frames
    assert sorted(fr) == [10]  # nothing added (no ref to derive drift)


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
