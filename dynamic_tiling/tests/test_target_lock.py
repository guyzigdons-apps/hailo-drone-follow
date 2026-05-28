import numpy as np
import dynamic_tiling  # noqa: F401  (sets sys.path for drone_follow import)
from dynamic_tiling.types import Det
from dynamic_tiling.target_lock import TargetLock, dets_to_array


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
