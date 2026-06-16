from hailo_tiling.dynamic.striped import StripedDenseScheduler
from hailo_tiling.types import LockState, MODEL_W


# ---- native-resolution overlapping dense grid (default) --------------------

def test_default_dense_grid_is_native_overlapping():
    # Defaults must give ~native 640x480 crops WITH overlap on a 4K source.
    s = StripedDenseScheduler(3840, 2160)
    assert s.dense_grid == (7, 6)
    assert s._v1.grid_overlap == 0.15
    assert len(s._dense) == 42
    # Every dense crop runs at ~scale 1.0 (native) — no >~5% up/downscale.
    for c in s._dense:
        assert 0.95 <= c.scale <= 1.10, (c.w, c.scale)


def test_default_dense_tiles_overlap_neighbours():
    s = StripedDenseScheduler(3840, 2160)
    # First two cells in row 0 share horizontal extent (right edge of cell 0
    # past the left edge of cell 1) => real overlap, not a gap.
    c0, c1 = s._dense[0], s._dense[1]
    assert (c0.x + c0.w) > c1.x


# ---- interleaved mode (legacy, opt-in) -------------------------------------

def test_interleaved_stripes_partition_full_grid():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6), fps=60.0,
                              cadence_fps=2.0, stripe_mode="interleaved")
    assert s.K == 30
    assert len(s._dense) == 48
    # Union of all stripe index-sets over one cycle == every dense cell, once.
    seen = []
    for f in range(s.K):
        seen.extend(s.stripe_indices(f))
    assert sorted(seen) == list(range(48))


def test_interleaved_stripes_are_balanced():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6), fps=60.0,
                              cadence_fps=2.0, stripe_mode="interleaved")
    sizes = [len(s.stripe_indices(f)) for f in range(s.K)]
    assert max(sizes) - min(sizes) <= 1
    assert s.stripe_indices(0)[0] == 0
    assert s.stripe_indices(1)[0] == 1


# ---- rolling mode (default): flat N cells/frame, row-major sweep -----------

def test_rolling_emits_fixed_cells_per_frame_rowmajor():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6), fps=60.0,
                              dense_per_frame=2)  # default stripe_mode="rolling"
    # Exactly dense_per_frame consecutive row-major cells, advancing each frame.
    assert s.stripe_indices(0) == [0, 1]
    assert s.stripe_indices(1) == [2, 3]
    assert s.stripe_indices(2) == [4, 5]


def test_rolling_covers_whole_frame_each_sweep_and_wraps():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6), fps=60.0,
                              dense_per_frame=2)
    n = len(s._dense)                       # 48
    sweep = n // s.dense_per_frame          # 24 frames per full sweep
    seen = []
    for f in range(sweep):
        seen.extend(s.stripe_indices(f))
    assert sorted(seen) == list(range(n))   # whole frame covered once per sweep
    assert s.stripe_indices(sweep) == s.stripe_indices(0)  # wraps


class _Unlimited:
    def available(self, frame_idx):
        return 9999.0


def _tracking_lock():
    return LockState(track_id=7, bbox_norm=(0.5, 0.5, 0.04, 0.03),
                     status="TRACKING", frames_since_seen=0,
                     last_velocity=(0.0, 0.0))


def test_decide_roi_first_then_dense():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6), fps=60.0,
                              dense_per_frame=2)
    crops = s.decide(_tracking_lock(), frame_idx=0, meter=_Unlimited())
    assert len(crops) == 1 + 2              # ROI + 2 dense cells
    assert crops[0].mode == "s"             # ROI first (single-scale)
    # Dense tiles are single-scale too: each native ~640x480 cell is already
    # model-sized, and multi-scale ("m") makes the aggregator drop their dets.
    assert all(c.mode == "s" for c in crops[1:])


def test_decide_per_frame_count_is_flat():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6), fps=60.0,
                              dense_per_frame=2)
    lock = _tracking_lock()
    counts = [len(s.decide(lock, f, _Unlimited())) for f in range(48)]
    assert max(counts) - min(counts) == 0   # ROI(1)+dense(2) every frame, no spike


def test_budget_trims_dense_before_roi():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6), fps=60.0,
                              dense_per_frame=2)

    class _OneTile:
        def available(self, frame_idx):
            return 1.0
    crops = s.decide(_tracking_lock(), frame_idx=0, meter=_OneTile())
    assert len(crops) == 1
    assert crops[0].mode == "s"             # ROI survives, dense dropped


def test_search_when_lost_runs_dense_sweep_not_recovery_grid():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6), fps=60.0,
                              dense_per_frame=2)
    lock = LockState(track_id=7, bbox_norm=(0.5, 0.5, 0.04, 0.03),
                     status="SEARCHING", frames_since_seen=3,
                     last_velocity=(0.0, 0.0))
    crops = s.decide(lock, frame_idx=0, meter=_Unlimited())
    # No ROI (not TRACKING); the rolling dense sweep IS the whole-frame search.
    assert len(crops) == 2
    assert all(c.mode == "s" for c in crops)   # dense is single-scale (native)


def test_search_respects_zero_budget():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6), fps=60.0,
                              dense_per_frame=2)
    lock = LockState(track_id=7, bbox_norm=(0.5, 0.5, 0.04, 0.03),
                     status="SEARCHING", frames_since_seen=3,
                     last_velocity=(0.0, 0.0))

    class _ZeroBudget:
        def available(self, frame_idx):
            return 0.0
    assert s.decide(lock, frame_idx=0, meter=_ZeroBudget()) == []


def test_flat_load_not_trimmed_under_showcase_budget():
    """With the showcase default budget (600/s @ 60 fps), the flat ROI+dense
    load of 3 tiles/frame must never be trimmed.  BudgetMeter.available()
    returns a per-frame average share; at steady state that is
    (600 - 3*60) / 60 = 7.0 >> 3, so no tile should be dropped."""
    from hailo_tiling.budget import BudgetMeter
    s = StripedDenseScheduler(3840, 2160, fps=60.0, dense_per_frame=2)
    meter = BudgetMeter(budget_inf_per_s=600.0, fps=60.0)  # showcase default
    lock = _tracking_lock()
    counts = []
    for f in range(120):                       # two full dense sweeps
        crops = s.decide(lock, f, meter)
        meter.charge(len(crops), f)            # charge like the controller does
        counts.append(len(crops))
    # Every TRACKING frame keeps ROI(1) + dense(2) = 3; never trimmed to 2.
    assert min(counts) == 3 and max(counts) == 3
