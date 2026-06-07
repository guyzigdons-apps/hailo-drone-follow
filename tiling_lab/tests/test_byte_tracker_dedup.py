"""Tests for ByteTracker.remove_duplicate_stracks duplicate pruning.

Regression coverage for the inverted threshold bug: the vendored tracker
swapped upstream's iou_distance (1 - IoU) for raw iou_batch but kept the
`pdist < 0.15` test, so it pruned near-DISJOINT pairs as "duplicates"
instead of heavily-overlapping ones. During a track loss this killed every
new recovery track spawned away from the buffered lost track.
"""
import numpy as np
from drone_follow.pipeline_adapter.byte_tracker import remove_duplicate_stracks
from hailo_tiling.types import Det
from tiling_lab.harness.target_lock import TargetLock


class _FakeTrack:
    """Minimal STrack stand-in: only the attributes remove_duplicate_stracks reads."""

    def __init__(self, tlbr, start_frame, frame_id):
        self.tlbr = np.asarray(tlbr, dtype=float)  # [x1, y1, x2, y2]
        self.start_frame = start_frame
        self.frame_id = frame_id

    @property
    def age(self):
        return self.frame_id - self.start_frame


def test_overlapping_tracks_younger_removed():
    """Two heavily-overlapping tracks (IoU > 0.9): the younger one is pruned."""
    older = _FakeTrack([100, 100, 200, 200], start_frame=1, frame_id=50)   # age 49
    younger = _FakeTrack([102, 102, 201, 201], start_frame=40, frame_id=50)  # age 10
    res_a, res_b = remove_duplicate_stracks([older], [younger])
    assert len(res_a) == 1 and res_a[0] is older       # older kept
    assert res_b == []                                  # younger pruned


def test_far_apart_tracks_both_survive():
    """Two non-overlapping tracks (IoU == 0) must BOTH survive.

    This is the recovery-killer case: pre-fix, raw IoU 0 < 0.15 flagged them
    as duplicates and killed one.
    """
    a = _FakeTrack([0, 0, 50, 50], start_frame=1, frame_id=50)
    b = _FakeTrack([900, 900, 950, 950], start_frame=40, frame_id=50)
    res_a, res_b = remove_duplicate_stracks([a], [b])
    assert len(res_a) == 1 and res_a[0] is a
    assert len(res_b) == 1 and res_b[0] is b


def test_short_loss_relocks_distant_target():
    """Integration: a target lost for only 30 frames (< track_buffer) then
    reappearing 0.15 away IS re-locked within a few frames.

    Mirrors test_radius_growth_relocks_distant_target but with a SHORT loss.
    Pre-fix this was impossible: the lingering lost track is still buffered, so
    the reappearing distant det spawned a NEW track that remove_duplicate_stracks
    immediately killed (raw IoU 0 < 0.15). With the 1 - IoU distance fix, the
    distant new track survives and the radius-growth gate can adopt it.
    """
    lock = TargetLock(frame_rate=30, reacq_radius_growth=0.005)
    dets = []
    x = 0.10
    for _ in range(10):
        dets.append(Det(cls=1, score=0.9, x=x, y=0.50, w=0.04, h=0.10))
        x += 0.01
    lock.step([dets[0]], gt_bbox_norm=(dets[0].x, dets[0].y, dets[0].w, dets[0].h))
    for d in dets[1:]:
        lock.step([d])
    # Short loss: 30 frames, well under track_buffer (90). The lost track is
    # STILL in ByteTracker's buffer when the target reappears far away.
    for _ in range(30):
        lock.step([])                          # radius grows 30*0.005 = 0.15
    far = Det(cls=1, score=0.9, x=dets[-1].x + 0.15, y=0.50, w=0.04, h=0.10)
    st = None
    for _ in range(5):
        st = lock.step([far])
    assert st.status == "TRACKING"
