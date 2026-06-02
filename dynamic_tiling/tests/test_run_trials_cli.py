from dynamic_tiling.trials import AggregateScore
from dynamic_tiling.run_trials import format_aggregate


def test_format_aggregate_table():
    agg = AggregateScore(n_trials=3, mean_coverage=0.91, mean_iou=0.72,
                         mean_drift_rate=0.03, mean_loss_events=1.3,
                         mean_time_to_recover=4.2, mean_recovery_success=0.8,
                         avg_tiles_per_frame=1.4, per_trial=[])
    txt = format_aggregate(agg, budget=300.0, fps=30.0)
    assert "coverage" in txt and "0.91" in txt
    assert "tiles/frame" in txt and "1.4" in txt
