from hailo_tiling.types import LockState
from hailo_tiling.budget import BudgetMeter
from hailo_tiling.dynamic.scheduler import TileScheduler


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


def test_discovery_grid_covers_full_frame():
    sched = TileScheduler(src_w=4000, src_h=3000, discovery_period=1,
                          discovery_grid=(3, 2))
    lock = LockState(track_id=None, status="LOST")  # no ROI, no recovery
    crops = sched.decide(lock, frame_idx=0, meter=_meter())
    assert len(crops) == 6
    assert min(c.x for c in crops) == 0
    assert max(c.x + c.w for c in crops) == 4000
    assert min(c.y for c in crops) == 0
    assert max(c.y + c.h for c in crops) == 3000


def test_tight_budget_keeps_roi_drops_discovery():
    sched = TileScheduler(src_w=4000, src_h=3000, discovery_period=1,
                          discovery_grid=(3, 2), max_zoom=2.0, target_model_h=40)
    lock = LockState(track_id=7, bbox_norm=(0.50, 0.50, 0.05, 0.10),
                     status="TRACKING")
    # Budget allows only ~1 tile/frame. Discovery alone is 6 tiles.
    tight = BudgetMeter(budget_inf_per_s=30, fps=30, window_s=1.0)
    crops = sched.decide(lock, frame_idx=0, meter=tight)
    assert len(crops) == 1
    # The surviving crop must be the ROI: scale >= 1.0 (zoom-in or native),
    # centred near the target. A discovery cell over a 4000x3000 frame would
    # have scale < 1 (cw ~ 1333 -> scale 0.48 / or cw=2000 -> scale 0.32).
    assert crops[0].scale >= 1.0


def test_recovery_grid_stays_inside_frame_at_right_edge():
    sched = TileScheduler(src_w=4000, src_h=3000, discovery_period=15,
                          recovery_grid=(3, 3), recovery_span=0.4)
    # Last-known at right edge + large rightward velocity over 5 frames pushes
    # the extrapolated centre well past the right edge.
    lock = LockState(track_id=7, bbox_norm=(0.95, 0.50, 0.05, 0.10),
                     status="SEARCHING", frames_since_seen=5,
                     last_velocity=(0.05, 0.0))
    crops = sched.decide(lock, frame_idx=1, meter=_meter())
    assert len(crops) == 3 * 3
    # All tile centres must lie inside the frame.
    for c in crops:
        cx = c.x + c.w / 2
        cy = c.y + c.h / 2
        assert 0 <= cx <= 4000
        assert 0 <= cy <= 3000
    # The recovery region's right edge must not exceed src_w.
    assert max(c.x + c.w for c in crops) <= 4000


def test_grid_overlap_makes_adjacent_tiles_share_area():
    s0 = TileScheduler(3840, 2160)
    s = TileScheduler(3840, 2160, grid_overlap=0.25)
    t0 = s0._grid(4, 3, 0, 0, 3840, 2160, "d")
    t = s._grid(4, 3, 0, 0, 3840, 2160, "d")
    assert len(t) == len(t0) == 12
    # horizontal neighbours overlap by ~25% of a tile width
    a, b = t[0], t[1]
    shared = (a.x + a.w) - b.x
    assert shared > 0.2 * a.w
    # zero-overlap grid: edge-to-edge cells (no horizontal sharing beyond aspect growth)
    a0, b0 = t0[0], t0[1]
    assert (a0.x + a0.w) - b0.x <= 0.05 * a0.w + 1
    # coverage preserved: last tile still reaches the right/bottom edge
    assert t[3].x + t[3].w >= 3839
    assert t[-1].y + t[-1].h >= 2159
