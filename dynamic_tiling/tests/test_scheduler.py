from dynamic_tiling.types import LockState
from dynamic_tiling.budget import BudgetMeter
from dynamic_tiling.scheduler import TileScheduler


def _meter():
    return BudgetMeter(budget_inf_per_s=300, fps=30, window_s=1.0)


def test_discovery_grid_only_on_cadence():
    sched = TileScheduler(src_w=4000, src_h=3000, discovery_period=15,
                          discovery_grid=(3, 2))
    lock = LockState(track_id=7, bbox_norm=(0.4, 0.4, 0.08, 0.2), status="TRACKING")
    crops0 = sched.decide(lock, frame_idx=0, meter=_meter())
    assert len(crops0) == 3 * 2 + 1
    crops1 = sched.decide(lock, frame_idx=1, meter=_meter())
    assert len(crops1) == 1


def test_roi_centered_on_target_and_zoom_clamped():
    sched = TileScheduler(src_w=4000, src_h=3000, discovery_period=15,
                          max_zoom=2.0, target_model_h=40)
    lock = LockState(track_id=7, bbox_norm=(0.50, 0.50, 0.05, 0.10), status="TRACKING")
    crops = sched.decide(lock, frame_idx=1, meter=_meter())
    roi = crops[0]
    assert 1.0 <= roi.scale <= 2.0 + 1e-6
    cx = roi.x + roi.w / 2
    assert abs(cx - 0.525 * 4000) < roi.w


def test_recovery_grid_when_searching():
    sched = TileScheduler(src_w=4000, src_h=3000, discovery_period=15,
                          recovery_grid=(3, 3))
    lock = LockState(track_id=7, bbox_norm=(0.4, 0.4, 0.08, 0.2),
                     status="SEARCHING", frames_since_seen=2)
    crops = sched.decide(lock, frame_idx=1, meter=_meter())
    assert len(crops) == 3 * 3


def test_recovery_grid_throttled_by_budget():
    sched = TileScheduler(src_w=4000, src_h=3000, discovery_period=15,
                          recovery_grid=(3, 3))
    lock = LockState(track_id=7, bbox_norm=(0.4, 0.4, 0.08, 0.2),
                     status="SEARCHING", frames_since_seen=2)
    tight = BudgetMeter(budget_inf_per_s=120, fps=30, window_s=1.0)
    tight.charge(4, frame_idx=0)
    crops = sched.decide(lock, frame_idx=1, meter=tight)
    assert len(crops) <= 4


def test_recovery_grid_follows_predicted_motion():
    sched = TileScheduler(src_w=4000, src_h=3000, discovery_period=15,
                          recovery_grid=(3, 3))
    # Same last-known bbox; one stationary, one moving right while lost for 5 frames.
    still = LockState(track_id=7, bbox_norm=(0.40, 0.40, 0.08, 0.20),
                      status="SEARCHING", frames_since_seen=5, last_velocity=(0.0, 0.0))
    moving = LockState(track_id=7, bbox_norm=(0.40, 0.40, 0.08, 0.20),
                       status="SEARCHING", frames_since_seen=5, last_velocity=(0.02, 0.0))
    cs = sched.decide(still, frame_idx=1, meter=_meter())
    cm = sched.decide(moving, frame_idx=1, meter=_meter())
    # Mean tile centre x should shift right for the moving target.
    mean_x_still = sum(c.x + c.w / 2 for c in cs) / len(cs)
    mean_x_moving = sum(c.x + c.w / 2 for c in cm) / len(cm)
    assert mean_x_moving > mean_x_still
