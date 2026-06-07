import tiling_lab.harness.trials as trials_mod
from tiling_lab.gt.gt_clean import GtTrack
from tiling_lab.harness.trials import run_all_trials, AggregateScore
from tiling_lab.harness.replay import RunResult


class _NullBackend:
    def close(self):
        pass


def test_run_all_trials_aggregates_per_track(monkeypatch):
    tracks = [
        GtTrack(cls=1, track_id=1, frames={i: (0.1, 0.1, 0.2, 0.2) for i in range(10)}),
        GtTrack(cls=1, track_id=2, frames={i: (0.6, 0.6, 0.1, 0.1) for i in range(10)}),
    ]

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj, person_cls=1, reid_assist=None):
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

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj, person_cls=1, reid_assist=None):
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

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj, person_cls=1, reid_assist=None):
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
            from hailo_tiling.types import LockState
            self.state = LockState()

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj, person_cls=1, reid_assist=None):
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


def test_fresh_reid_assist_per_trial_reaches_run(monkeypatch):
    """A reid_assist_factory builds one ReidAssist per trial; each carries a FRESH
    gallery (two person tracks -> two distinct gallery objects). The assist reaches
    run() and its per-trial stats surface in the aggregate/per-trial rows."""
    from tiling_lab.reid.reid_gallery import ReidGallery
    from tiling_lab.reid.reid_policy import GenerousPolicy, ReidAssist

    tracks = [
        GtTrack(cls=1, track_id=1, frames={i: (0.1, 0.1, 0.2, 0.2) for i in range(10)}),
        GtTrack(cls=1, track_id=2, frames={i: (0.6, 0.6, 0.1, 0.1) for i in range(10)}),
    ]

    class _FakeEmbedder:
        def __init__(self):
            self.stats = {"embeds": 0, "chip_embeds": 0}

    seen_assists = []

    def reid_assist_factory():
        a = ReidAssist(_FakeEmbedder(), ReidGallery(), GenerousPolicy())
        return a

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj,
                 person_cls=1, reid_assist=None):
        assert reid_assist is not None       # the assist reaches run()
        seen_assists.append(reid_assist)
        # simulate some embedding work for this trial
        reid_assist.embedder.stats["embeds"] += 4
        reid_assist.embedder.stats["chip_embeds"] += 1
        reid_assist.person_dets_seen += 8
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
        reid_assist_factory=reid_assist_factory,
    )
    # one assist per person trial, each with a DISTINCT gallery (fresh per trial)
    assert len(seen_assists) == 2
    assert seen_assists[0].gallery is not seen_assists[1].gallery
    # per-trial reid stats recorded
    assert agg.per_trial[0].reid_embeds == 4
    assert agg.per_trial[0].reid_chip_embeds == 1
    assert agg.per_trial[0].person_dets_seen == 8
    assert abs(agg.per_trial[0].frac_dets_embedded - 0.5) < 1e-9  # 4 / 8
    # aggregate means
    assert abs(agg.mean_frac_dets_embedded - 0.5) < 1e-9


def test_frame_weighted_coverage_vs_unweighted(monkeypatch):
    """Two very-different-length tracks: a 300-frame track fully covered and a
    15-frame track fully uncovered. Frame-weighted coverage ~ 300/315 while the
    legacy unweighted (per-track mean) coverage ~ 0.5."""
    tracks = [
        GtTrack(cls=1, track_id=1, frames={i: (0.1, 0.1, 0.2, 0.2) for i in range(300)}),
        GtTrack(cls=1, track_id=2, frames={i: (0.6, 0.6, 0.1, 0.1) for i in range(15)}),
    ]

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj,
                 person_cls=1, reid_assist=None):
        r = RunResult()
        # long track (300 frames): perfectly followed; short track (15): never found
        if len(gt_traj) == 300:
            r.pred_traj = dict(gt_traj)
        else:
            r.pred_traj = {}      # totally uncovered
        r.n_frames = len(gt_traj)
        r.total_tiles = r.n_frames
        return r

    monkeypatch.setattr(trials_mod, "run", fake_run)
    agg = run_all_trials(
        frames_factory=lambda: iter([]),
        src_w=1920, src_h=1080, gt_tracks=tracks,
        backend_factory=lambda: _NullBackend(),
        budget=300.0, fps=30.0, discovery_fps=2.0,
    )
    # legacy unweighted: (1.0 + 0.0) / 2 = 0.5
    assert abs(agg.mean_coverage - 0.5) < 1e-9
    # frame-weighted: 300 covered / 315 gt frames
    assert abs(agg.coverage_fw - (300 / 315)) < 1e-9
    assert agg.n_gt_frames_total == 315


def test_event_weighted_recovery_vs_unweighted(monkeypatch):
    """One track recovers 3 of 3 losses (rate 1.0), another recovers 0 of 1
    (rate 0.0). Legacy unweighted recovery = 0.5; event-weighted = 3/4."""
    # long track: covered, lost, covered, lost, covered, lost, covered (3 recovered losses)
    long_frames = {}
    long_pred = {}
    box = (0.1, 0.1, 0.2, 0.2)
    f = 0
    # build a covered/lost/covered/... pattern: 3 isolated single-frame losses
    pattern = [True, False, True, False, True, False, True, True]
    for hit in pattern:
        long_frames[f] = box
        if hit:
            long_pred[f] = box
        f += 1
    # short track: lost at the final frame and never recovers -> 1 loss, 0 recovered
    short_frames = {0: box, 1: box}
    short_pred = {0: box}   # frame 1 uncovered, clip ends lost

    tracks = [
        GtTrack(cls=1, track_id=1, frames=long_frames),
        GtTrack(cls=1, track_id=2, frames=short_frames),
    ]

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj,
                 person_cls=1, reid_assist=None):
        r = RunResult()
        if len(gt_traj) == len(long_frames):
            r.pred_traj = dict(long_pred)
            r.n_frames = len(long_frames)
        else:
            r.pred_traj = dict(short_pred)
            r.n_frames = len(short_frames)
        r.total_tiles = r.n_frames
        return r

    monkeypatch.setattr(trials_mod, "run", fake_run)
    agg = run_all_trials(
        frames_factory=lambda: iter([]),
        src_w=1920, src_h=1080, gt_tracks=tracks,
        backend_factory=lambda: _NullBackend(),
        budget=300.0, fps=30.0, discovery_fps=2.0,
    )
    # sanity: long track has 3 losses all recovered, short has 1 loss not recovered
    assert agg.per_trial[0].loss_events == 3
    assert abs(agg.per_trial[0].recovery_success_rate - 1.0) < 1e-9
    assert agg.per_trial[1].loss_events == 1
    assert abs(agg.per_trial[1].recovery_success_rate - 0.0) < 1e-9
    # legacy unweighted: (1.0 + 0.0) / 2 = 0.5
    assert abs(agg.mean_recovery_success - 0.5) < 1e-9
    # event-weighted: 3 recovered / 4 total loss events
    assert abs(agg.recovery_success_ew - 0.75) < 1e-9


def test_event_weighted_recovery_none_when_no_losses(monkeypatch):
    """No loss events anywhere -> recovery_success_ew is None."""
    tracks = [GtTrack(cls=1, track_id=1, frames={i: (0.1, 0.1, 0.2, 0.2) for i in range(10)})]

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj,
                 person_cls=1, reid_assist=None):
        r = RunResult()
        r.pred_traj = dict(gt_traj)   # perfectly covered -> zero losses
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
    assert agg.recovery_success_ew is None


def test_no_reid_assist_factory_means_no_reid_stats(monkeypatch):
    """policy=none path: no reid_assist_factory -> run() gets reid_assist=None and
    per-trial reid stats are zero (frac_dets_embedded == 0)."""
    tracks = [GtTrack(cls=1, track_id=1, frames={i: (0.1, 0.1, 0.2, 0.2) for i in range(10)})]

    def fake_run(frames, src_w, src_h, scheduler, lock, backend, meter, gt_traj,
                 person_cls=1, reid_assist=None):
        assert reid_assist is None
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
    assert agg.per_trial[0].reid_embeds == 0
    assert agg.per_trial[0].frac_dets_embedded == 0.0
    assert agg.mean_frac_dets_embedded == 0.0
