import numpy as np
from hailo_tiling.types import Det
from tiling_lab.harness.target_lock import TargetLock, dets_to_array


def _person(x, y, w=0.08, h=0.20, score=0.9):
    return Det(cls=0, score=score, x=x, y=y, w=w, h=h)


def test_dets_to_array_scale_and_columns():
    arr = dets_to_array([_person(0.10, 0.20)])
    assert arr.shape == (1, 5)
    assert abs(arr[0, 0] - 100.0) < 1e-3      # 0.10 * 1000
    assert abs(arr[0, 2] - (0.10 + 0.08) * 1000) < 1e-3
    assert abs(arr[0, 4] - 0.9) < 1e-6


def test_lock_holds_track_id_across_constant_velocity():
    lock = TargetLock()
    s0 = lock.step([_person(0.40, 0.40)], lock_if_unlocked=True)
    assert s0.status == "TRACKING"
    tid = lock.track_id
    assert tid is not None
    for k in range(1, 6):
        s = lock.step([_person(0.40 + 0.01 * k, 0.40)])
        assert s.status == "TRACKING"
        assert lock.track_id == tid
    assert lock.state.last_velocity[0] > 0  # moving right


def test_lock_enters_searching_then_recovers():
    lock = TargetLock()
    lock.step([_person(0.40, 0.40)], lock_if_unlocked=True)
    tid = lock.track_id
    for _ in range(3):
        s = lock.step([])
        assert s.status in ("SEARCHING", "LOST")
    assert lock.track_id == tid
    s = lock.step([_person(0.43, 0.40)])
    assert s.status == "TRACKING"
    assert lock.track_id == tid


def test_no_spurious_velocity_on_first_lock():
    lock = TargetLock()
    s = lock.step([_person(0.40, 0.40)], lock_if_unlocked=True)
    assert s.status == "TRACKING"
    assert abs(s.last_velocity[0]) < 1e-6 and abs(s.last_velocity[1]) < 1e-6


def _walk_dets(n, x0=0.10, dx=0.01, y=0.50, w=0.04, h=0.10, score=0.9):
    """Person walking right: one det per frame."""
    return [Det(cls=1, score=score, x=x0 + i * dx, y=y, w=w, h=h) for i in range(n)]


def test_velocity_anchor_tracks_motion_during_loss():
    lock = TargetLock(frame_rate=30, reacq_motion="velocity")
    dets = _walk_dets(10)
    lock.step([dets[0]], gt_bbox_norm=(dets[0].x, dets[0].y, dets[0].w, dets[0].h))
    for d in dets[1:]:
        lock.step([d])                      # establishes velocity ~ (0.01, 0)
    a0 = lock.reacq_anchor
    for _ in range(10):
        lock.step([])                       # 10 lost frames, anchor should advance
    a1 = lock.reacq_anchor
    assert a1[0] - a0[0] > 0.05             # moved right ~10 * 0.01 (Kalman-smoothed, allow slack)
    assert abs(a1[1] - a0[1]) < 0.02        # no vertical drift


def test_radius_growth_relocks_distant_target():
    lock = TargetLock(frame_rate=30, reacq_radius_growth=0.005)
    dets = _walk_dets(10)
    lock.step([dets[0]], gt_bbox_norm=(dets[0].x, dets[0].y, dets[0].w, dets[0].h))
    for d in dets[1:]:
        lock.step([d])
    # Loss must exceed track_buffer (90) so the lingering lost track is purged
    # from ByteTracker's buffer; otherwise the reappearing distant det can never
    # spawn a surviving NEW track for the distance gate to adopt (see report).
    for _ in range(100):
        lock.step([])                       # long loss; radius grows 100*0.005=0.5
    # target reappears 0.15 away from the frozen anchor — IoU=0, distance inside radius
    far = Det(cls=1, score=0.9, x=dets[-1].x + 0.15, y=0.50, w=0.04, h=0.10)
    st = None
    for _ in range(5):                      # let ByteTracker activate the new track
        st = lock.step([far])
    assert st.status == "TRACKING"


def test_radius_zero_keeps_strict_behavior():
    lock = TargetLock(frame_rate=30)        # growth 0.0
    dets = _walk_dets(10)
    lock.step([dets[0]], gt_bbox_norm=(dets[0].x, dets[0].y, dets[0].w, dets[0].h))
    for d in dets[1:]:
        lock.step([d])
    for _ in range(100):
        lock.step([])
    far = Det(cls=1, score=0.9, x=dets[-1].x + 0.15, y=0.50, w=0.04, h=0.10)
    st = None
    for _ in range(5):
        st = lock.step([far])
    assert st.status != "TRACKING"          # legacy: never re-locks


def test_no_swap_when_distractor_is_outside_radius():
    lock = TargetLock(frame_rate=30, reacq_radius_growth=0.002)
    d0 = Det(cls=1, score=0.9, x=0.10, y=0.50, w=0.04, h=0.10)
    lock.step([d0], gt_bbox_norm=(0.10, 0.50, 0.04, 0.10))
    for _ in range(9):
        lock.step([d0])
    for _ in range(5):                      # short loss: radius ~ r0 + 5*0.002
        lock.step([])
    distractor = Det(cls=1, score=0.9, x=0.80, y=0.20, w=0.04, h=0.10)  # far away
    st = None
    for _ in range(5):
        st = lock.step([distractor])
    assert st.status != "TRACKING"          # distant person must NOT be adopted


def test_frozen_mode_keeps_legacy_anchor():
    lock = TargetLock(frame_rate=30)        # default reacq_motion="frozen"
    dets = _walk_dets(10)
    lock.step([dets[0]], gt_bbox_norm=(dets[0].x, dets[0].y, dets[0].w, dets[0].h))
    for d in dets[1:]:
        lock.step([d])
    a0 = lock.reacq_anchor
    for _ in range(10):
        lock.step([])
    assert lock.reacq_anchor == a0          # frozen
