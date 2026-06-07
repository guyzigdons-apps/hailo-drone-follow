"""Tests for the ReID ablation driver. NONE of these touch the chip or real
videos: registry paths are checked exist-or-skip, arm->policy mapping is pure,
render_report runs on synthetic JSON fixtures, and the driver loop monkeypatches
run_all_trials so no inference happens."""
import json

import pytest

from tiling_lab.cli import run_reid_ablation as rra


# --------------------------------------------------------------------------- #
# Clip registry                                                               #
# --------------------------------------------------------------------------- #

def test_clip_registry_has_expected_keys():
    for key in ("0025:fov50", "0025:fov60", "0025:fov70", "0026:fov50"):
        assert key in rra.CLIPS
        clip = rra.CLIPS[key]
        # video, gt, cache paths present and shaped right
        assert str(clip.video).endswith(".mp4")
        assert str(clip.gt_tracks).endswith("gt_tracks.verified.json")
        assert "__yolov8n4c_vga.sqlite3" in str(clip.cache)


def test_registry_video_and_gt_paths_match_disk_or_skip():
    """The registered video + GT paths should exist on this machine. Skip (don't
    fail) when the data isn't present so the suite stays chip/data independent."""
    missing = [str(p) for clip in rra.CLIPS.values()
               for p in (clip.video, clip.gt_tracks) if not p.exists()]
    if missing:
        pytest.skip(f"clip data not present: {missing}")
    for clip in rra.CLIPS.values():
        assert clip.video.exists()
        assert clip.gt_tracks.exists()


# --------------------------------------------------------------------------- #
# Arm -> policy mapping                                                       #
# --------------------------------------------------------------------------- #

def test_arm_none_has_no_policy():
    from tiling_lab.reid.reid_policy import (GenerousPolicy, ProdPolicy,
                                            AmbiguityPolicy, MotionGatedPolicy,
                                            HistogramPolicy)
    assert rra.ARMS["none"] is None
    assert isinstance(rra.make_policy("generous"), GenerousPolicy)
    assert isinstance(rra.make_policy("prod"), ProdPolicy)
    assert isinstance(rra.make_policy("ambiguity"), AmbiguityPolicy)
    assert isinstance(rra.make_policy("motion"), MotionGatedPolicy)
    assert isinstance(rra.make_policy("histogram"), HistogramPolicy)


def test_motion_and_histogram_arms_carry_block1_radius_growth():
    """motion + histogram are MotionGatedPolicy subclasses whose radius_growth
    must be threaded through from the driver's --reacq-radius-growth (so the gate
    geometry matches the Block-1 best)."""
    m = rra.make_policy("motion", radius_growth=0.005)
    assert m.radius_growth == 0.005
    h = rra.make_policy("histogram", radius_growth=0.005)
    assert h.radius_growth == 0.005
    assert h.keep_top == 2  # histogram default top-2 color screen


def test_make_policy_rejects_unknown_arm():
    with pytest.raises(KeyError):
        rra.make_policy("bogus")


# --------------------------------------------------------------------------- #
# render_report on synthetic JSON fixtures                                    #
# --------------------------------------------------------------------------- #

def _fake_result_json(arm, *, coverage, frac, reacq_motion="velocity",
                       reacq_radius_growth=0.002):
    return {
        "params": {
            "video": "/v/clip.mp4",
            "gt_tracks": "/g/gt_tracks.verified.json",
            "budget": 3000.0,
            "reacq_motion": reacq_motion,
            "reacq_radius_growth": reacq_radius_growth,
            "reid_policy": arm,
            "reid_cache": "/c/clip.sqlite3",
            "cache": "/c/clip.sqlite3",
            "hef": "/h/det.hef",
            "reid_hef": "/h/reid.hef",
            "discovery_grid": "8x6",
            "discovery_fps": 2.0,
            "discovery_overlap": 0.25,
        },
        "aggregate": {
            "n_trials": 2,
            "mean_coverage": coverage,
            "mean_iou": 0.7,
            "mean_drift_rate": 0.02,
            "mean_loss_events": 1.0,
            "mean_time_to_recover": 4.0,
            "mean_recovery_success": 0.8,
            "avg_tiles_per_frame": 1.3,
            "mean_frac_dets_embedded": frac,
        },
        "per_trial": [],
    }


def _write_ablation_dir(tmp_path):
    out = tmp_path / "reid_ablation"
    out.mkdir()
    # two clips x a few arms
    fixtures = {
        ("0025_fov50", "none"): (0.80, 0.0),
        ("0025_fov50", "generous"): (0.95, 1.0),
        ("0025_fov50", "motion"): (0.93, 0.25),
        ("0026_fov50", "none"): (0.70, 0.0),
        ("0026_fov50", "generous"): (0.90, 1.0),
        ("0026_fov50", "motion"): (0.88, 0.30),
    }
    for (clip, arm), (cov, frac) in fixtures.items():
        (out / f"{clip}__{arm}.json").write_text(
            json.dumps(_fake_result_json(arm, coverage=cov, frac=frac)))
    return out


def test_render_report_builds_markdown_with_tables_and_pareto(tmp_path):
    out = _write_ablation_dir(tmp_path)
    md = rra.render_report(out)
    # one table per clip
    assert "0025_fov50" in md and "0026_fov50" in md
    # arm rows present
    for arm in ("none", "generous", "motion"):
        assert arm in md
    # quality-vs-fraction (Pareto) section
    assert "fraction" in md.lower() or "frac" in md.lower()
    # repro command(s)
    assert "run_trials" in md or "run_reid_ablation" in md
    # chosen-default paragraph present
    assert "default" in md.lower()


def test_render_report_chosen_default_reflects_best_coverage(tmp_path):
    out = _write_ablation_dir(tmp_path)
    md = rra.render_report(out)
    # generous wins on mean coverage across both clips -> named as the data-driven
    # best in the chosen-default discussion.
    assert "generous" in md


# --------------------------------------------------------------------------- #
# Driver loop (monkeypatched run_all_trials -> no chip)                       #
# --------------------------------------------------------------------------- #

def test_driver_runs_one_trial_per_clip_x_arm_and_writes_json(tmp_path, monkeypatch):
    from tiling_lab.harness.trials import AggregateScore

    calls = []

    def fake_run_all_trials(**kwargs):
        calls.append(kwargs)
        return AggregateScore(
            n_trials=1, mean_coverage=0.9, mean_iou=0.7, mean_drift_rate=0.0,
            mean_loss_events=1.0, mean_time_to_recover=3.0,
            mean_recovery_success=1.0, avg_tiles_per_frame=1.2, per_trial=[],
            mean_frac_dets_embedded=0.5)

    # No video read / no track load: stub the loaders too.
    monkeypatch.setattr(rra, "run_all_trials", fake_run_all_trials)
    monkeypatch.setattr(rra, "_load_tracks", lambda p: [])
    monkeypatch.setattr(rra, "_probe_dims", lambda v: (1920, 1080))
    monkeypatch.setattr(rra, "_frames_factory", lambda v, n: (lambda: iter([])))

    clips = ["0025:fov50", "0026:fov50"]
    arms = ["none", "generous", "motion"]
    outdir = tmp_path / "abl"
    rra.run_ablation(clips=clips, arms=arms, budget=3000.0,
                     reacq_motion="velocity", reacq_radius_growth=0.002,
                     outdir=outdir)

    # one run per clip x arm
    assert len(calls) == len(clips) * len(arms)
    # one JSON per clip x arm
    written = sorted(p.name for p in outdir.glob("*.json"))
    assert written == sorted(
        f"{c.replace(':', '_')}__{a}.json" for c in clips for a in arms)
    # the JSONs are valid result docs
    doc = json.loads((outdir / "0025_fov50__generous.json").read_text())
    assert doc["params"]["reid_policy"] == "generous"
    assert doc["aggregate"]["mean_coverage"] == 0.9


def test_driver_threads_motion_config_into_run_all_trials(tmp_path, monkeypatch):
    from tiling_lab.harness.trials import AggregateScore
    seen = []

    def fake_run_all_trials(**kwargs):
        seen.append(kwargs)
        return AggregateScore(
            n_trials=1, mean_coverage=0.9, mean_iou=0.7, mean_drift_rate=0.0,
            mean_loss_events=1.0, mean_time_to_recover=3.0,
            mean_recovery_success=1.0, avg_tiles_per_frame=1.2, per_trial=[],
            mean_frac_dets_embedded=0.5)

    monkeypatch.setattr(rra, "run_all_trials", fake_run_all_trials)
    monkeypatch.setattr(rra, "_load_tracks", lambda p: [])
    monkeypatch.setattr(rra, "_probe_dims", lambda v: (1920, 1080))
    monkeypatch.setattr(rra, "_frames_factory", lambda v, n: (lambda: iter([])))

    rra.run_ablation(clips=["0025:fov50"], arms=["motion"], budget=3000.0,
                     reacq_motion="velocity", reacq_radius_growth=0.007,
                     outdir=tmp_path / "abl")
    assert seen[0]["reacq_motion"] == "velocity"
    assert seen[0]["reacq_radius_growth"] == 0.007
    assert seen[0]["budget"] == 3000.0
    # P0 vs reid arm: motion arm must pass a reid_assist_factory through
    assert seen[0]["reid_assist_factory"] is not None


def test_driver_none_arm_passes_no_reid_factory(tmp_path, monkeypatch):
    from tiling_lab.harness.trials import AggregateScore
    seen = []
    monkeypatch.setattr(rra, "run_all_trials",
                        lambda **kw: seen.append(kw) or AggregateScore(
                            n_trials=0, mean_coverage=0.0, mean_iou=0.0,
                            mean_drift_rate=0.0, mean_loss_events=0.0,
                            mean_time_to_recover=0.0, mean_recovery_success=0.0,
                            avg_tiles_per_frame=0.0, per_trial=[]))
    monkeypatch.setattr(rra, "_load_tracks", lambda p: [])
    monkeypatch.setattr(rra, "_probe_dims", lambda v: (1920, 1080))
    monkeypatch.setattr(rra, "_frames_factory", lambda v, n: (lambda: iter([])))
    rra.run_ablation(clips=["0025:fov50"], arms=["none"], budget=3000.0,
                     reacq_motion="velocity", reacq_radius_growth=0.002,
                     outdir=tmp_path / "abl")
    assert seen[0]["reid_assist_factory"] is None


def test_gallery_sub_ablation_runs_winning_arm_variants(tmp_path, monkeypatch):
    """--gallery-sub-ablation runs FIFO vs ema=0.1 vs both for the winning arm on
    0025:fov50 + 0026:fov50. Winner = highest mean coverage across the main runs."""
    from tiling_lab.harness.trials import AggregateScore

    # Make 'generous' the coverage winner.
    cov_by_arm = {"none": 0.7, "generous": 0.95, "motion": 0.9}
    seen = []

    def fake_run_all_trials(**kwargs):
        seen.append(kwargs)
        # ema knob is read off the factory; just report a fixed coverage by arm.
        return AggregateScore(
            n_trials=1, mean_coverage=0.9, mean_iou=0.7, mean_drift_rate=0.0,
            mean_loss_events=1.0, mean_time_to_recover=3.0,
            mean_recovery_success=1.0, avg_tiles_per_frame=1.2, per_trial=[],
            mean_frac_dets_embedded=0.5)

    monkeypatch.setattr(rra, "run_all_trials", fake_run_all_trials)
    monkeypatch.setattr(rra, "_load_tracks", lambda p: [])
    monkeypatch.setattr(rra, "_probe_dims", lambda v: (1920, 1080))
    monkeypatch.setattr(rra, "_frames_factory", lambda v, n: (lambda: iter([])))

    outdir = tmp_path / "abl"
    # pre-seed main-run JSONs so the winner can be determined from data
    outdir.mkdir()
    for clip in ("0025_fov50", "0026_fov50"):
        for arm, cov in cov_by_arm.items():
            (outdir / f"{clip}__{arm}.json").write_text(
                json.dumps(_fake_result_json(arm, coverage=cov, frac=0.5)))

    rra.run_gallery_sub_ablation(
        arms=list(cov_by_arm), budget=3000.0, reacq_motion="velocity",
        reacq_radius_growth=0.002, outdir=outdir)

    # winner=generous, 2 clips x {fifo, ema, both} = 6 runs
    assert len(seen) == 6
    names = sorted(p.name for p in outdir.glob("*__generous__*.json"))
    assert names == sorted(
        f"{c}__generous__{v}.json"
        for c in ("0025_fov50", "0026_fov50")
        for v in ("fifo", "ema", "both"))
