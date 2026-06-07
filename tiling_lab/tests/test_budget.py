from hailo_tiling.budget import BudgetMeter


def test_steady_budget_allows_average_tiles_per_frame():
    # 300 inf/s at 30 fps -> 10 tiles/frame average.
    m = BudgetMeter(budget_inf_per_s=300, fps=30, window_s=1.0)
    assert round(m.available(frame_idx=0)) == 10


def test_burst_then_throttle_keeps_window_average():
    m = BudgetMeter(budget_inf_per_s=300, fps=30, window_s=1.0)
    m.charge(40, frame_idx=0)
    avail_after_burst = m.available(frame_idx=1)
    assert avail_after_burst < 10  # throttled below steady average
    for f in range(1, 31):
        m.charge(0, frame_idx=f)
    assert round(m.available(frame_idx=31)) == 10


def test_available_never_negative():
    m = BudgetMeter(budget_inf_per_s=30, fps=30, window_s=1.0)  # 1 tile/frame
    m.charge(100, frame_idx=0)
    assert m.available(frame_idx=1) >= 0
