from hailo_tiling.dynamic.striped import StripedDenseScheduler
from hailo_tiling.types import LockState


def test_stripes_partition_full_grid_with_no_overlap():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0)
    # 60/2 = 30 frames per full refresh.
    assert s.K == 30
    # 8*6 = 48 dense crops built once.
    assert len(s._dense) == 48
    # Union of all stripe index-sets over one cycle == every dense cell, once.
    seen = []
    for f in range(s.K):
        seen.extend(s.stripe_indices(f))
    assert sorted(seen) == list(range(48))


def test_stripes_are_interleaved_and_balanced():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0)
    sizes = [len(s.stripe_indices(f)) for f in range(s.K)]
    # 48 tiles over 30 stripes => sizes differ by at most 1.
    assert max(sizes) - min(sizes) <= 1
    # Interleaved: stripe 0 starts at cell 0, stripe 1 at cell 1, etc.
    assert s.stripe_indices(0)[0] == 0
    assert s.stripe_indices(1)[0] == 1


class _Unlimited:
    def available(self, frame_idx):
        return 9999.0


def _tracking_lock():
    return LockState(track_id=7, bbox_norm=(0.5, 0.5, 0.04, 0.03),
                     status="TRACKING", frames_since_seen=0,
                     last_velocity=(0.0, 0.0))


def test_decide_emits_roi_first_then_one_stripe():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0)
    lock = _tracking_lock()
    crops = s.decide(lock, frame_idx=0, meter=_Unlimited())
    # ROI tile (single-scale) + stripe-0 dense tiles (multi-scale).
    n_stripe = len(s.stripe_indices(0))
    assert len(crops) == 1 + n_stripe
    assert crops[0].mode == "s"           # ROI first
    assert all(c.mode == "m" for c in crops[1:])  # dense tiles multi-scale


def test_decide_per_frame_count_is_flat_across_cycle():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0)
    lock = _tracking_lock()
    counts = [len(s.decide(lock, f, _Unlimited())) for f in range(s.K)]
    # ROI(1) + ~1.6 dense/frame => counts differ by at most 1 (no spike).
    assert max(counts) - min(counts) <= 1


def test_budget_trims_dense_stripe_before_roi():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0)
    lock = _tracking_lock()

    class _OneTile:
        def available(self, frame_idx):
            return 1.0
    crops = s.decide(lock, frame_idx=0, meter=_OneTile())
    assert len(crops) == 1
    assert crops[0].mode == "s"           # the ROI survives, dense dropped


def test_recovery_owns_frame_no_dense_stripe():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0,
                              recovery_grid=(3, 3))
    lock = LockState(track_id=7, bbox_norm=(0.5, 0.5, 0.04, 0.03),
                     status="SEARCHING", frames_since_seen=3,
                     last_velocity=(0.0, 0.0))
    crops = s.decide(lock, frame_idx=0, meter=_Unlimited())
    # 3x3 recovery grid, all single-scale, no multi-scale dense tiles.
    assert len(crops) == 9
    assert all(c.mode == "s" for c in crops)


def test_recovery_respects_zero_budget():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0, recovery_grid=(3, 3))
    lock = LockState(track_id=7, bbox_norm=(0.5, 0.5, 0.04, 0.03),
                     status="SEARCHING", frames_since_seen=3,
                     last_velocity=(0.0, 0.0))

    class _ZeroBudget:
        def available(self, frame_idx):
            return 0.0
    crops = s.decide(lock, frame_idx=0, meter=_ZeroBudget())
    assert crops == []
