"""Tests for the Phase A coordinate-descent sweep driver. NONE touch the chip or
real videos: `coordinate_descent` is pure and exercised with a stubbed score_fn;
the main-loop tests monkeypatch `run_all_trials` (and the loaders) so no inference
happens, then assert configs are evaluated, JSONs written, the memo prevents
duplicate evaluation, and the budget frontier writes its 15 files."""
import json

import pytest

from dynamic_tiling import run_sweep as rs


# --------------------------------------------------------------------------- #
# coordinate_descent (pure)                                                   #
# --------------------------------------------------------------------------- #

def test_coordinate_descent_walks_axes_and_keeps_best():
    calls = []

    def fake_score(cfg):                      # higher = better
        calls.append(dict(cfg))
        return -abs(cfg["a"] - 3) - abs(cfg["b"] - 20)

    axes = {"a": [1, 2, 3], "b": [10, 20]}
    best, history = rs.coordinate_descent({"a": 1, "b": 10}, axes, fake_score, passes=2)
    assert best == {"a": 3, "b": 20}
    assert all("score" in h for h in history)
    # pass 2 re-sweeps each axis around the updated best
    assert len(calls) >= len(axes["a"]) + len(axes["b"])


def test_coordinate_descent_memoizes_repeat_configs():
    calls = []

    def fake_score(cfg):
        calls.append(dict(cfg))
        return -abs(cfg["a"] - 1) - abs(cfg["b"] - 10)  # base is already best

    axes = {"a": [1, 2, 3], "b": [10, 20]}
    base = {"a": 1, "b": 10}
    best, history = rs.coordinate_descent(base, axes, fake_score, passes=2)
    assert best == base
    # history holds only the unique configs (memoized), not every evaluation
    keys = [tuple(sorted({k: v for k, v in h.items() if k != "score"}.items()))
            for h in history]
    assert len(keys) == len(set(keys))
    # each history entry corresponds to exactly one score_fn call
    assert len(calls) == len(history)
    # the base config (a=1,b=10) is evaluated once even though both passes revisit it
    base_evals = sum(1 for c in calls if c == base)
    assert base_evals == 1


def test_coordinate_descent_history_records_scores():
    def fake_score(cfg):
        return float(cfg["a"])

    best, history = rs.coordinate_descent(
        {"a": 1}, {"a": [1, 2, 5]}, fake_score, passes=1)
    assert best == {"a": 5}
    scored = {h["a"]: h["score"] for h in history}
    assert scored == {1: 1.0, 2: 2.0, 5: 5.0}


# --------------------------------------------------------------------------- #
# cfg slug                                                                    #
# --------------------------------------------------------------------------- #

def test_cfg_slug_is_deterministic_and_fs_safe():
    cfg = {"discovery_grid": "8x6", "discovery_fps": 2, "discovery_overlap": 0.25,
           "max_zoom": 2.0, "target_model_h": 40.0}
    slug = rs.cfg_slug(cfg)
    assert slug == rs.cfg_slug(dict(cfg))           # deterministic
    # filesystem-safe: no path separators / spaces / colons
    for bad in ("/", "\\", " ", ":"):
        assert bad not in slug
    # readable + carries each knob
    assert "grid8x6" in slug
    assert "fps2" in slug
    assert "ov0.25" in slug
    assert "zoom2.0" in slug
    assert "h40" in slug


# --------------------------------------------------------------------------- #
# AXES / BASE registry                                                        #
# --------------------------------------------------------------------------- #

def test_axes_and_base_match_spec():
    assert rs.AXES == {
        "discovery_grid": ["4x3", "6x4", "8x6", "12x9"],
        "discovery_fps": [1, 2, 4],
        "discovery_overlap": [0.0, 0.15, 0.25],
        "max_zoom": [1.5, 2.0, 3.0],
        "target_model_h": [30.0, 40.0, 60.0],
    }
    assert rs.BASE == {
        "discovery_grid": "8x6", "discovery_fps": 2, "discovery_overlap": 0.25,
        "max_zoom": 2.0, "target_model_h": 40.0}
    # BASE values are members of their axes (so BASE is inside the search space)
    for k, v in rs.BASE.items():
        assert v in rs.AXES[k]


def test_clip_registry_has_three_0025_fovs():
    for key in ("0025:fov50", "0025:fov60", "0025:fov70"):
        assert key in rs.CLIPS
        clip = rs.CLIPS[key]
        assert str(clip.video).endswith(".mp4")
        assert "__yolov8n4c_vga.sqlite3" in str(clip.cache)


# --------------------------------------------------------------------------- #
# score_fn / main loop (monkeypatched run_all_trials -> no chip)              #
# --------------------------------------------------------------------------- #

def _fake_agg(coverage=0.9, tiles=1.3):
    from dynamic_tiling.trials import AggregateScore
    return AggregateScore(
        n_trials=1, mean_coverage=coverage, mean_iou=0.7, mean_drift_rate=0.0,
        mean_loss_events=1.0, mean_time_to_recover=3.0,
        mean_recovery_success=1.0, avg_tiles_per_frame=tiles, per_trial=[],
        mean_frac_dets_embedded=0.5)


def _stub_loaders(monkeypatch):
    monkeypatch.setattr(rs, "_load_tracks", lambda p: [])
    monkeypatch.setattr(rs, "_probe_dims", lambda v: (1920, 1080))
    monkeypatch.setattr(rs, "_frames_factory", lambda v, n: (lambda: iter([])))


def test_score_fn_runs_three_fovs_and_writes_per_fov_json(tmp_path, monkeypatch):
    seen = []

    def fake_run_all_trials(**kwargs):
        seen.append(kwargs)
        return _fake_agg(coverage=0.9, tiles=1.5)

    monkeypatch.setattr(rs, "run_all_trials", fake_run_all_trials)
    _stub_loaders(monkeypatch)

    outdir = tmp_path / "phase_a"
    score = rs.make_score_fn(
        outdir=outdir, budget=3000.0, reacq_motion="velocity",
        reacq_radius_growth=0.002, reid_policy="none")
    cfg = dict(rs.BASE)
    val = score(cfg)

    # three fovs evaluated
    assert len(seen) == 3
    # score == mean(coverage) - 0.005*mean(tiles)
    assert val == pytest.approx(0.9 - 0.005 * 1.5)
    # one JSON per fov, named <slug>_fov<F>.json
    slug = rs.cfg_slug(cfg)
    written = sorted(p.name for p in outdir.glob("*.json"))
    assert written == sorted(f"{slug}_fov{f}.json" for f in (50, 60, 70))
    # the cfg knobs propagated into run_all_trials
    assert seen[0]["discovery_grid"] == (8, 6)
    assert seen[0]["discovery_fps"] == 2
    assert seen[0]["grid_overlap"] == 0.25
    assert seen[0]["max_zoom"] == 2.0
    assert seen[0]["target_model_h"] == 40.0
    assert seen[0]["reacq_motion"] == "velocity"
    assert seen[0]["reacq_radius_growth"] == 0.002
    # reid_policy none -> no reid factory
    assert seen[0]["reid_assist_factory"] is None


def test_score_fn_reid_policy_wires_factory(tmp_path, monkeypatch):
    seen = []
    monkeypatch.setattr(rs, "run_all_trials",
                        lambda **kw: seen.append(kw) or _fake_agg())
    _stub_loaders(monkeypatch)
    # a fake embedder (with the .close() the real one has) so we don't touch the
    # chip building the reid factory
    class _StubEmbedder:
        def close(self):
            pass

    monkeypatch.setattr(rs, "_build_reid_factory",
                        lambda **kw: ((lambda: object()), _StubEmbedder()))

    score = rs.make_score_fn(
        outdir=tmp_path / "phase_a", budget=3000.0, reacq_motion="velocity",
        reacq_radius_growth=0.002, reid_policy="motion")
    score(dict(rs.BASE))
    assert seen[0]["reid_assist_factory"] is not None


def test_main_loop_evaluates_configs_writes_json_and_memoizes(tmp_path, monkeypatch):
    """Run the real coordinate_descent through a monkeypatched run_all_trials: each
    distinct config writes 3 fov JSONs, and the memo means no config's 3 fovs are
    run twice across the two passes."""
    seen = []

    def fake_run_all_trials(**kwargs):
        seen.append(kwargs)
        # vary coverage by grid so the descent actually moves (bigger grid better)
        gx, gy = kwargs["discovery_grid"]
        return _fake_agg(coverage=0.5 + 0.01 * (gx + gy), tiles=1.0 + 0.05 * gx)

    monkeypatch.setattr(rs, "run_all_trials", fake_run_all_trials)
    _stub_loaders(monkeypatch)

    outdir = tmp_path / "phase_a"
    best, history = rs.run_sweep(
        outdir=outdir, budget=3000.0, reacq_motion="velocity",
        reacq_radius_growth=0.002, reid_policy="none", passes=2)

    # every history config produced exactly 3 fov JSONs, and no JSON written twice
    slugs = {rs.cfg_slug({k: v for k, v in h.items() if k != "score"})
             for h in history}
    written = sorted(p.name for p in outdir.glob("*.json"))
    expected = sorted(f"{s}_fov{f}.json" for s in slugs for f in (50, 60, 70))
    assert written == expected
    # memo: exactly 3 run_all_trials calls per distinct config (no double-run)
    assert len(seen) == 3 * len(slugs)
    # best is a full config dict
    assert set(best) == set(rs.BASE)


def test_budget_frontier_writes_fifteen_files(tmp_path, monkeypatch):
    monkeypatch.setattr(rs, "run_all_trials", lambda **kw: _fake_agg())
    _stub_loaders(monkeypatch)

    outdir = tmp_path / "phase_a"
    written = rs.run_budget_frontier(
        cfg=dict(rs.BASE), outdir=outdir, reacq_motion="velocity",
        reacq_radius_growth=0.002, reid_policy="none")
    # 5 budgets x 3 fovs = 15
    assert len(written) == 15
    files = sorted(p.name for p in outdir.glob("frontier_b*_fov*.json"))
    assert len(files) == 15
    for b in (300, 600, 1000, 1500, 3000):
        for f in (50, 60, 70):
            assert (outdir / f"frontier_b{b}_fov{f}.json").exists()
