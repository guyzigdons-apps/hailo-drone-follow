from dynamic_tiling.gt_clean import GtTrack
from dynamic_tiling.gt_edit import clip_frame_range, pin_static_tracks


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


def test_pin_requires_ref_frame_present():
    car = _t(1, 2, {0: (0.1, 0.1, 0.2, 0.2)})
    try:
        pin_static_tracks([car], track_ids=[1], ref_frame=99, frame_range=(0, 3))
        assert False, "expected error"
    except (KeyError, ValueError):
        pass
