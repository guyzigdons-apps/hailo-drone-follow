"""Central-target acquisition in TargetLock: lock the most-central track (held
in the centre for several frames), not merely the largest."""
from hailo_tiling.types import Det
from tiling_lab.harness.target_lock import TargetLock


def _step_n(lock, dets, n):
    st = None
    for _ in range(n):
        st = lock.step(dets, lock_if_unlocked=True)
    return st


# A small vehicle dead-centre vs a bigger one off to the right.
CENTRAL = Det(cls=2, score=0.9, x=0.47, y=0.47, w=0.06, h=0.06)   # centre (0.50, 0.50)
BIG_OFF = Det(cls=2, score=0.9, x=0.78, y=0.40, w=0.20, h=0.20)   # centre (0.88, 0.50), larger


def test_central_mode_locks_central_not_largest():
    lock = TargetLock(acquire_mode="central", center_frac=0.2, central_frames=3)
    st = _step_n(lock, [CENTRAL, BIG_OFF], 8)
    assert lock.track_id is not None
    assert st.status == "TRACKING"
    cx = st.bbox_norm[0] + st.bbox_norm[2] / 2
    cy = st.bbox_norm[1] + st.bbox_norm[3] / 2
    assert abs(cx - 0.5) < 0.15 and abs(cy - 0.5) < 0.15  # locked the CENTRAL one


def test_largest_mode_still_locks_offcenter_big():
    lock = TargetLock(acquire_mode="largest")
    st = _step_n(lock, [CENTRAL, BIG_OFF], 6)
    assert lock.track_id is not None
    cx = st.bbox_norm[0] + st.bbox_norm[2] / 2
    assert cx > 0.7  # legacy behaviour: locked the off-centre BIG one


def test_central_requires_debounce_frames():
    # Central target present for fewer than central_frames must NOT lock yet.
    lock = TargetLock(acquire_mode="central", center_frac=0.2, central_frames=20)
    _step_n(lock, [CENTRAL], 4)
    assert lock.track_id is None


def test_offcenter_target_never_acquired_in_central_mode():
    # A target that is large but never near the centre is never locked.
    lock = TargetLock(acquire_mode="central", center_frac=0.15, central_frames=3)
    _step_n(lock, [BIG_OFF], 10)
    assert lock.track_id is None


def test_manual_seed_binds_offcenter_target():
    # seed() fallback locks a target by location even when it's NOT central
    # (auto-central would never pick it). central_frames huge so only seed acts.
    lock = TargetLock(acquire_mode="central", center_frac=0.15, central_frames=999)
    lock.seed((0.78, 0.40, 0.20, 0.20))   # BIG_OFF location (off-centre)
    st = _step_n(lock, [BIG_OFF], 6)
    assert lock.track_id is not None       # bound to the seeded target
    cx = st.bbox_norm[0] + st.bbox_norm[2] / 2
    assert cx > 0.7 and st.status == "TRACKING"


def test_seed_times_out_if_target_absent():
    # A seed pointing where nothing is detected is given up after seed_timeout.
    lock = TargetLock(acquire_mode="central", central_frames=999, seed_timeout=5)
    lock.seed((0.5, 0.5, 0.05, 0.05))
    _step_n(lock, [], 8)                    # nothing ever detected there
    assert lock.track_id is None
    assert lock._pending_seed is None       # seed abandoned


def test_central_acquisition_tolerates_detection_gaps():
    # The striped dense grid samples a not-yet-locked target only intermittently.
    # Central observations must accumulate ACROSS gaps (no detection at all on the
    # gap frames), not require consecutive frames.
    lock = TargetLock(acquire_mode="central", center_frac=0.2, central_frames=4)
    # Warm up the track so ByteTracker keeps a stable activated id across gaps.
    _step_n(lock, [CENTRAL], 3)
    # Now alternate: central detection, then several empty (gap) frames.
    for _ in range(4):
        lock.step([CENTRAL], lock_if_unlocked=True)      # central observation
        _step_n(lock, [], 5)                              # gap: nothing detected
    assert lock.track_id is not None                      # accumulated -> locked
    assert lock.state.bbox_norm[2] > 0
