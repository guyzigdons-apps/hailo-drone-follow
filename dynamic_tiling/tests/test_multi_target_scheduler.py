from dynamic_tiling.types import TargetState, ScheduledTile
from dynamic_tiling.budget import BudgetMeter
from dynamic_tiling.scheduler import MultiTargetTileScheduler


def _meter(budget=300):
    return BudgetMeter(budget_inf_per_s=budget, fps=30, window_s=1.0)


def _tracking(cls, track_id, x=0.4, y=0.4, w=0.08, h=0.20, selected=False):
    s = TargetState(key=(cls, track_id), cls=cls,
                    bbox_norm=(x, y, w, h), status="TRACKING")
    s.selected = selected
    return s


def _searching(cls, track_id, x=0.4, y=0.4, selected=False):
    s = TargetState(key=(cls, track_id), cls=cls,
                    bbox_norm=(x, y, 0.08, 0.20), status="SEARCHING",
                    frames_since_seen=3)
    s.selected = selected
    return s


# ---------------------------------------------------------------------------
# Phase 1 tests — adapted for ScheduledTile return type
# ---------------------------------------------------------------------------

def test_multi_target_scheduler_emits_roi_per_tracking_target():
    sched = MultiTargetTileScheduler(src_w=4000, src_h=3000,
                                     discovery_period=15)
    targets = [
        _tracking(0, 1, x=0.2),
        _tracking(0, 2, x=0.5),
        _tracking(1, 3, x=0.8),
    ]
    tiles = sched.decide(targets, frame_idx=1, meter=_meter())
    # Off discovery cadence, no selected+SEARCHING => 3 per-target ROIs (no merge expected
    # since targets are well separated).
    assert len(tiles) == 3
    # All should be ScheduledTile instances.
    assert all(isinstance(t, ScheduledTile) for t in tiles)


def test_multi_target_scheduler_recovery_only_for_selected():
    sched = MultiTargetTileScheduler(src_w=4000, src_h=3000,
                                     discovery_period=15,
                                     recovery_grid=(3, 3))
    targets = [
        _tracking(0, 1, x=0.2),
        _tracking(0, 2, x=0.5),
        _searching(1, 3, x=0.7, selected=True),
    ]
    tiles = sched.decide(targets, frame_idx=1, meter=_meter())
    # Recovery branch: selected target is SEARCHING => 9-tile recovery grid.
    assert len(tiles) == 9
    assert all(t.category == "single-scale" for t in tiles)


def test_multi_target_scheduler_selected_roi_first():
    sched = MultiTargetTileScheduler(src_w=4000, src_h=3000,
                                     discovery_period=15)
    selected_state = _tracking(0, 1, x=0.2, selected=True)
    other_state = _tracking(0, 2, x=0.7, selected=False)
    sched.set_selected(selected_state.key)
    targets = [other_state, selected_state]  # selected passed second
    tiles = sched.decide(targets, frame_idx=1, meter=_meter())
    assert len(tiles) >= 2
    # Selected key must appear in one of the first tiles served.
    served_keys = set()
    for t in tiles:
        for k in t.target_keys:
            served_keys.add(k)
    assert selected_state.key in served_keys


# ---------------------------------------------------------------------------
# Phase 2 new tests
# ---------------------------------------------------------------------------

def test_merge_close_small_targets():
    """Two small targets close together should be merged into one dynamic-merged tile."""
    sched = MultiTargetTileScheduler(
        src_w=4000, src_h=3000,
        discovery_period=15,
        merge_iou_threshold=0.3,   # lenient so the test is robust
        merge_pad_frac=0.5,        # large pad brings small close targets into overlap
        merge_union_inflate_max=2.0,
    )
    # Two small targets almost on top of each other.
    t1 = _tracking(0, 1, x=0.40, y=0.40, w=0.04, h=0.10)
    t2 = _tracking(0, 2, x=0.42, y=0.40, w=0.04, h=0.10)
    tiles = sched.decide([t1, t2], frame_idx=1, meter=_meter())
    # Should have been merged into one tile.
    dynamic_tiles = [t for t in tiles if t.category in ("dynamic", "dynamic-merged")]
    assert len(dynamic_tiles) == 1, (
        f"Expected 1 merged tile, got {len(dynamic_tiles)}: {[t.category for t in tiles]}"
    )
    assert dynamic_tiles[0].category == "dynamic-merged"
    assert set(dynamic_tiles[0].target_keys) == {(0, 1), (0, 2)}


def test_no_merge_large_separated_targets():
    """Two targets far apart should stay as separate ROIs."""
    sched = MultiTargetTileScheduler(
        src_w=4000, src_h=3000,
        discovery_period=15,
        merge_iou_threshold=0.5,
        merge_pad_frac=0.25,
        merge_union_inflate_max=1.5,
    )
    # Targets on opposite sides of the frame.
    t1 = _tracking(0, 1, x=0.05, y=0.40, w=0.10, h=0.20)
    t2 = _tracking(0, 2, x=0.80, y=0.40, w=0.10, h=0.20)
    tiles = sched.decide([t1, t2], frame_idx=1, meter=_meter())
    dynamic_tiles = [t for t in tiles if t.category in ("dynamic", "dynamic-merged")]
    assert len(dynamic_tiles) == 2
    assert all(t.category == "dynamic" for t in dynamic_tiles)


def test_no_merge_when_union_inflates_too_much():
    """Targets that are near each other but have very different sizes should NOT merge
    when the union crop_w exceeds merge_union_inflate_max * max(crop_w_a, crop_w_b)."""
    sched = MultiTargetTileScheduler(
        src_w=4000, src_h=3000,
        discovery_period=15,
        merge_iou_threshold=0.1,    # low threshold so IoU condition passes
        merge_pad_frac=0.5,
        merge_union_inflate_max=1.0,  # very tight: almost no extra size allowed
    )
    # Two targets at almost the same position but one is large, one small.
    t1 = _tracking(0, 1, x=0.40, y=0.40, w=0.30, h=0.40)  # large
    t2 = _tracking(0, 2, x=0.41, y=0.40, w=0.04, h=0.05)  # small, overlapping
    tiles = sched.decide([t1, t2], frame_idx=1, meter=_meter())
    dynamic_tiles = [t for t in tiles if t.category in ("dynamic", "dynamic-merged")]
    # With inflate_max=1.0 the union should fail condition B.
    assert all(t.category == "dynamic" for t in dynamic_tiles), (
        f"No merge expected with inflate_max=1.0, got: {[t.category for t in tiles]}"
    )


def test_aging_counter_promotes_deferred_target():
    """With budget=1 and no selected target, 3 TRACKING targets should each
    get served at least once over 3 frames (aging round-robin)."""
    sched = MultiTargetTileScheduler(
        src_w=4000, src_h=3000,
        discovery_period=100,   # suppress discovery tiles
        merge_iou_threshold=0.9,  # suppress merges (targets are separated)
    )
    sched.clear_aging()
    # Three well-separated targets (no merge).
    targets = [
        _tracking(0, 1, x=0.10, y=0.40),
        _tracking(0, 2, x=0.50, y=0.40),
        _tracking(0, 3, x=0.85, y=0.40),
    ]
    meter = _meter(budget=30)  # ~1 tile/frame at 30fps

    served_counts = {t.key: 0 for t in targets}
    for frame_idx in range(9):  # 3× the number of targets
        tiles = sched.decide(targets, frame_idx=frame_idx, meter=meter)
        for tile in tiles:
            for k in tile.target_keys:
                if k in served_counts:
                    served_counts[k] += 1

    # Each target should have been served at least once.
    for key, count in served_counts.items():
        assert count >= 1, (
            f"Target {key} was never served in 9 frames (counts: {served_counts})"
        )


def test_selected_target_always_served():
    """With budget=1, the selected (smallest) target must be served every frame."""
    sched = MultiTargetTileScheduler(
        src_w=4000, src_h=3000,
        discovery_period=100,   # suppress discovery
        merge_iou_threshold=0.9,  # suppress merges
    )
    sched.clear_aging()
    # Three targets — selected is the smallest (by area).
    selected = _tracking(0, 99, x=0.50, y=0.50, w=0.02, h=0.05, selected=True)
    big1 = _tracking(0, 1, x=0.10, y=0.40, w=0.15, h=0.30)
    big2 = _tracking(0, 2, x=0.80, y=0.40, w=0.15, h=0.30)
    sched.set_selected(selected.key)
    targets = [big1, big2, selected]

    meter = _meter(budget=30)  # ~1 tile/frame
    for frame_idx in range(6):
        tiles = sched.decide(targets, frame_idx=frame_idx, meter=meter)
        served_keys = {k for t in tiles for k in t.target_keys}
        assert selected.key in served_keys, (
            f"Selected target not served on frame {frame_idx} "
            f"(tiles: {[(t.category, t.target_keys) for t in tiles]})"
        )
