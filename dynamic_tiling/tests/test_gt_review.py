from dynamic_tiling.gt_clean import GtTrack
from dynamic_tiling.gt_review import merge_and_flag, ReviewCase, apply_decisions


def _t(tid, cls, frames):
    return GtTrack(cls=cls, track_id=tid, frames=frames)


def test_auto_merges_near_certain_duplicate():
    box = (0.42, 0.64, 0.02, 0.05)
    a = _t(1, 1, {i: box for i in range(40)})
    b = _t(2, 1, {i: (0.421, 0.64, 0.02, 0.05) for i in range(40)})
    merged, cases = merge_and_flag([a, b], auto_iou=0.7, flag_iou=0.3, min_len=30)
    assert len(merged) == 1
    assert not any(c.kind == "merge" for c in cases)


def test_flags_gray_zone_merge():
    a = _t(1, 1, {i: (0.40, 0.60, 0.05, 0.05) for i in range(40)})
    b = _t(2, 1, {i: (0.41, 0.61, 0.05, 0.05) for i in range(40)})  # IoU ~0.47 → gray zone
    merged, cases = merge_and_flag([a, b], auto_iou=0.7, flag_iou=0.3, min_len=30)
    assert any(c.kind == "merge" for c in cases)
    assert len(merged) == 2


def test_flags_short_track_for_keep_drop():
    short = _t(9, 2, {i: (0.5, 0.47, 0.05, 0.05) for i in range(12)})
    long = _t(1, 1, {i: (0.1, 0.1, 0.05, 0.05) for i in range(40)})
    merged, cases = merge_and_flag([short, long], auto_iou=0.7, flag_iou=0.3, min_len=30)
    assert any(c.kind == "keep_short" and 9 in c.track_ids for c in cases)
    assert any(t.track_id == 9 for t in merged)


def test_apply_decisions_merge_and_drop():
    a = _t(1, 1, {i: (0.40, 0.60, 0.05, 0.05) for i in range(40)})
    b = _t(2, 1, {i: (0.43, 0.62, 0.05, 0.05) for i in range(40)})
    short = _t(9, 2, {i: (0.5, 0.47, 0.05, 0.05) for i in range(12)})
    cases = [ReviewCase(kind="merge", frame=0, track_ids=(1, 2), boxes=[], reason="", score=0.5),
             ReviewCase(kind="keep_short", frame=0, track_ids=(9,), boxes=[], reason="", score=0.0)]
    decisions = {0: "merge", 1: "drop"}
    out = apply_decisions([a, b, short], cases, decisions)
    ids = sorted(t.track_id for t in out)
    assert ids == [1]
