import dynamic_tiling.trials as trials_mod
from dynamic_tiling.gt_clean import GtTrack
from dynamic_tiling.trials import run_all_trials, AggregateScore
from dynamic_tiling.replay import RunResult


class _NullBackend:
    def close(self):
        pass


def test_run_all_trials_aggregates_per_track(monkeypatch):
    tracks = [
        GtTrack(cls=1, track_id=1, frames={i: (0.1, 0.1, 0.2, 0.2) for i in range(10)}),
        GtTrack(cls=1, track_id=2, frames={i: (0.6, 0.6, 0.1, 0.1) for i in range(10)}),
    ]

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj, person_cls=1):
        # perfectly follow whatever target trajectory was passed in this trial
        r = RunResult()
        r.pred_traj = dict(gt_traj)
        r.n_frames = 10
        r.total_tiles = 14            # -> avg 1.4 tiles/frame
        return r

    monkeypatch.setattr(trials_mod, "run", fake_run)
    agg = run_all_trials(
        frames_factory=lambda: iter([]),
        src_w=1920, src_h=1080, gt_tracks=tracks,
        backend_factory=lambda: _NullBackend(),
        budget=300.0, fps=30.0, discovery_fps=2.0,
    )
    assert isinstance(agg, AggregateScore)
    assert agg.n_trials == 2
    assert agg.mean_coverage == 1.0          # each trial follows its own target perfectly
    assert agg.mean_drift_rate == 0.0
    assert abs(agg.avg_tiles_per_frame - 1.4) < 1e-9


def test_vehicle_tracks_not_trialed_but_persons_are(monkeypatch):
    tracks = [
        GtTrack(cls=1, track_id=1, frames={i: (0.1, 0.1, 0.2, 0.2) for i in range(10)}),
        GtTrack(cls=2, track_id=9, frames={i: (0.6, 0.6, 0.1, 0.1) for i in range(10)}),  # vehicle
    ]

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj, person_cls=1):
        r = RunResult()
        r.pred_traj = dict(gt_traj)
        r.n_frames = 10
        r.total_tiles = 10
        return r

    monkeypatch.setattr(trials_mod, "run", fake_run)
    agg = run_all_trials(
        frames_factory=lambda: iter([]),
        src_w=1920, src_h=1080, gt_tracks=tracks,
        backend_factory=lambda: _NullBackend(),
        budget=300.0, fps=30.0, discovery_fps=2.0,
    )
    assert agg.n_trials == 1          # only the person track is trialed


def test_on_result_callback_receives_track_id_and_runresult(monkeypatch):
    tracks = [
        GtTrack(cls=1, track_id=3, frames={i: (0.1, 0.1, 0.2, 0.2) for i in range(10)}),
        GtTrack(cls=1, track_id=4, frames={i: (0.6, 0.6, 0.1, 0.1) for i in range(10)}),
    ]

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj, person_cls=1):
        r = RunResult()
        r.pred_traj = dict(gt_traj)
        r.n_frames = 10
        r.total_tiles = 10
        return r

    monkeypatch.setattr(trials_mod, "run", fake_run)
    seen = []
    run_all_trials(
        frames_factory=lambda: iter([]),
        src_w=1920, src_h=1080, gt_tracks=tracks,
        backend_factory=lambda: _NullBackend(),
        budget=300.0, fps=30.0, discovery_fps=2.0,
        on_result=lambda tid, res: seen.append((tid, type(res).__name__)),
    )
    assert seen == [(3, "RunResult"), (4, "RunResult")]


def test_reacq_knobs_reach_target_lock(monkeypatch):
    captured = {}

    class FakeLock:
        def __init__(self, **kw):
            captured.update(kw)
            self.track_id = None
            from dynamic_tiling.types import LockState
            self.state = LockState()

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj, person_cls=1):
        r = RunResult()
        r.n_frames = 1
        return r

    monkeypatch.setattr(trials_mod, "TargetLock", FakeLock)
    monkeypatch.setattr(trials_mod, "run", fake_run)
    run_all_trials(
        frames_factory=lambda: iter([]), src_w=1920, src_h=1080,
        gt_tracks=[GtTrack(cls=1, track_id=1, frames={0: (0.1, 0.1, 0.2, 0.2)})],
        backend_factory=lambda: _NullBackend(),
        budget=300.0, fps=30.0, discovery_fps=2.0,
        reacq_motion="velocity", reacq_radius_growth=0.002)
    assert captured["reacq_motion"] == "velocity"
    assert captured["reacq_radius_growth"] == 0.002
