from dynamic_tiling.types import TargetState
from dynamic_tiling.budget import BudgetMeter
from dynamic_tiling.scheduler import MultiTargetTileScheduler


def _meter():
    return BudgetMeter(budget_inf_per_s=300, fps=30, window_s=1.0)


def _tracking(cls, track_id, x=0.4, y=0.4, selected=False):
    s = TargetState(key=(cls, track_id), cls=cls,
                    bbox_norm=(x, y, 0.08, 0.20), status="TRACKING")
    s.selected = selected
    return s


def _searching(cls, track_id, x=0.4, y=0.4, selected=False):
    s = TargetState(key=(cls, track_id), cls=cls,
                    bbox_norm=(x, y, 0.08, 0.20), status="SEARCHING",
                    frames_since_seen=3)
    s.selected = selected
    return s


def test_multi_target_scheduler_emits_roi_per_tracking_target():
    sched = MultiTargetTileScheduler(src_w=4000, src_h=3000,
                                     discovery_period=15)
    targets = [
        _tracking(0, 1, x=0.2),
        _tracking(0, 2, x=0.5),
        _tracking(1, 3, x=0.8),
    ]
    crops = sched.decide(targets, frame_idx=1, meter=_meter())
    # Off discovery cadence, no selected+SEARCHING => 3 per-target ROIs.
    assert len(crops) == 3


def test_multi_target_scheduler_recovery_only_for_selected():
    sched = MultiTargetTileScheduler(src_w=4000, src_h=3000,
                                     discovery_period=15,
                                     recovery_grid=(3, 3))
    targets = [
        _tracking(0, 1, x=0.2),
        _tracking(0, 2, x=0.5),
        _searching(1, 3, x=0.7, selected=True),
    ]
    crops = sched.decide(targets, frame_idx=1, meter=_meter())
    # Recovery branch: selected target is SEARCHING => 9-tile recovery grid.
    # No per-target ROIs when recovery is active.
    assert len(crops) == 9


def test_multi_target_scheduler_selected_roi_first():
    sched = MultiTargetTileScheduler(src_w=4000, src_h=3000,
                                     discovery_period=15)
    selected_state = _tracking(0, 1, x=0.2, selected=True)
    other_state = _tracking(0, 2, x=0.7, selected=False)
    targets = [other_state, selected_state]  # selected passed second
    crops = sched.decide(targets, frame_idx=1, meter=_meter())
    assert len(crops) >= 2
    # First crop must be centered near selected target's x=0.2.
    sel_cx = (selected_state.bbox_norm[0] + selected_state.bbox_norm[2] / 2) * 4000
    first_cx = crops[0].x + crops[0].w / 2
    other_cx = (other_state.bbox_norm[0] + other_state.bbox_norm[2] / 2) * 4000
    assert abs(first_cx - sel_cx) < abs(first_cx - other_cx)
