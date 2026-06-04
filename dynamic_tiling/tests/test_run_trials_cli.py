from dynamic_tiling.metrics import TrialScore
from dynamic_tiling.trials import AggregateScore
from dynamic_tiling.run_trials import format_aggregate, results_doc


def test_format_aggregate_table():
    agg = AggregateScore(n_trials=3, mean_coverage=0.91, mean_iou=0.72,
                         mean_drift_rate=0.03, mean_loss_events=1.3,
                         mean_time_to_recover=4.2, mean_recovery_success=0.8,
                         avg_tiles_per_frame=1.4, per_trial=[])
    txt = format_aggregate(agg, budget=300.0, fps=30.0)
    assert "coverage" in txt and "0.91" in txt
    assert "tiles/frame" in txt and "1.4" in txt


def test_results_doc_round_trips_per_trial_with_track_ids():
    t1 = TrialScore(n_frames=100, coverage=0.9, mean_iou=0.7, drift_rate=0.0,
                    loss_events=1, mean_time_to_recover=3.0, recovery_success_rate=1.0)
    t2 = TrialScore(n_frames=50, coverage=0.4, mean_iou=0.5, drift_rate=0.1,
                    loss_events=2, mean_time_to_recover=8.0, recovery_success_rate=0.5)
    agg = AggregateScore(n_trials=2, mean_coverage=0.65, mean_iou=0.6,
                         mean_drift_rate=0.05, mean_loss_events=1.5,
                         mean_time_to_recover=5.5, mean_recovery_success=0.75,
                         avg_tiles_per_frame=2.0, per_trial=[t1, t2])
    doc = results_doc(agg, person_track_ids=[3, 4], params={"budget": 3000.0})
    assert doc["params"]["budget"] == 3000.0
    assert doc["aggregate"]["mean_coverage"] == 0.65
    assert "per_trial" not in doc["aggregate"]  # per-trial detail lives in its own list
    assert [t["track_id"] for t in doc["per_trial"]] == [3, 4]
    assert doc["per_trial"][0]["coverage"] == 0.9
    assert doc["per_trial"][1]["loss_events"] == 2
