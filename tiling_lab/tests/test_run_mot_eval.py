import json
from pathlib import Path

import pytest

from tiling_lab.cli.run_mot_eval import (
    load_gt_as_mot,
    load_pred,
    format_metrics,
    main,
    replay_static_cache,
    by_track_to_pred_doc,
    _parse_wh,
)


def _write(path: Path, doc: dict) -> Path:
    path.write_text(json.dumps(doc))
    return path


def _gt_doc():
    # one person track (cls=1) + one vehicle track (cls=2)
    return {
        "clip": "fixture",
        "tracks": [
            {"cls": 1, "track_id": 1,
             "frames": {str(f): [0.1, 0.5, 0.05, 0.1] for f in range(5)}},
            {"cls": 2, "track_id": 2,
             "frames": {str(f): [0.7, 0.5, 0.05, 0.1] for f in range(5)}},
        ],
    }


def _pred_perfect_person():
    # pred matches the person track exactly, with a string id
    return {"tracks": {"10": {str(f): [0.1, 0.5, 0.05, 0.1] for f in range(5)}}}


def test_load_gt_filters_by_class_and_converts_keys(tmp_path):
    gt_path = _write(tmp_path / "gt.json", _gt_doc())
    gt = load_gt_as_mot(gt_path, classes={1})
    # vehicle (cls=2, track_id=2) excluded; only person track_id=1 remains
    assert set(gt) == {1}
    traj = gt[1]
    # string frame keys -> int frames; bbox -> tuple
    assert set(traj) == set(range(5))
    assert all(isinstance(f, int) for f in traj)
    assert traj[0] == (0.1, 0.5, 0.05, 0.1)


def test_load_gt_multi_class(tmp_path):
    gt_path = _write(tmp_path / "gt.json", _gt_doc())
    gt = load_gt_as_mot(gt_path, classes={1, 2})
    assert set(gt) == {1, 2}


def test_load_pred_converts_string_keys(tmp_path):
    pred_path = _write(tmp_path / "pred.json", _pred_perfect_person())
    pred = load_pred(pred_path)
    assert set(pred) == {10}
    assert all(isinstance(p, int) for p in pred)
    traj = pred[10]
    assert all(isinstance(f, int) for f in traj)
    assert traj[3] == (0.1, 0.5, 0.05, 0.1)


def test_round_trip_perfect_prediction_scores_perfectly(tmp_path):
    gt_path = _write(tmp_path / "gt.json", _gt_doc())
    pred_path = _write(tmp_path / "pred.json", _pred_perfect_person())
    gt = load_gt_as_mot(gt_path, classes={1})
    pred = load_pred(pred_path)
    from tiling_lab.harness.mot_metrics import score_mot
    m = score_mot(gt, pred, iou_thr=0.5)
    assert m["MOTA"] == 1.0
    assert m["IDF1"] == 1.0
    assert m["FP"] == 0 and m["FN"] == 0 and m["IDsw"] == 0


def test_class_filter_excludes_vehicles_affects_score(tmp_path):
    # If we DON'T filter, the vehicle GT track is unmatched -> FN, MOTA<1.
    gt_path = _write(tmp_path / "gt.json", _gt_doc())
    pred_path = _write(tmp_path / "pred.json", _pred_perfect_person())
    pred = load_pred(pred_path)
    from tiling_lab.harness.mot_metrics import score_mot

    gt_person = load_gt_as_mot(gt_path, classes={1})
    gt_both = load_gt_as_mot(gt_path, classes={1, 2})
    m_person = score_mot(gt_person, pred, iou_thr=0.5)
    m_both = score_mot(gt_both, pred, iou_thr=0.5)
    assert m_person["MOTA"] == 1.0
    assert m_both["MOTA"] < 1.0  # vehicle frames are misses
    assert m_both["FN"] == 5


def test_format_metrics_is_aligned_table():
    metrics = {"MOTA": 1.0, "IDF1": 1.0, "IDsw": 0, "FP": 0, "FN": 0,
               "Frag": 0, "MT": 1, "ML": 0, "n_gt": 5, "n_pred": 5,
               "n_frames": 5}
    txt = format_metrics(metrics)
    assert "MOTA" in txt and "IDF1" in txt and "IDsw" in txt
    assert "1.000" in txt  # floats formatted


def test_main_writes_out_file_with_expected_keys(tmp_path, monkeypatch):
    gt_path = _write(tmp_path / "gt.json", _gt_doc())
    pred_path = _write(tmp_path / "pred.json", _pred_perfect_person())
    out_path = tmp_path / "report.json"
    monkeypatch.setattr(
        "sys.argv",
        ["run_mot_eval", "--gt-tracks", str(gt_path), "--pred", str(pred_path),
         "--classes", "1", "--iou-thr", "0.5", "--out", str(out_path)],
    )
    main()
    assert out_path.exists()
    doc = json.loads(out_path.read_text())
    assert set(doc) >= {"metrics", "gt_tracks", "pred", "params"}
    assert doc["gt_tracks"] == str(gt_path)
    assert doc["pred"] == str(pred_path)
    assert doc["params"]["classes"] == [1]
    assert doc["params"]["iou_thr"] == 0.5
    assert doc["metrics"]["MOTA"] == 1.0
    assert doc["metrics"]["IDF1"] == 1.0


def test_main_runs_without_out(tmp_path, monkeypatch, capsys):
    gt_path = _write(tmp_path / "gt.json", _gt_doc())
    pred_path = _write(tmp_path / "pred.json", _pred_perfect_person())
    monkeypatch.setattr(
        "sys.argv",
        ["run_mot_eval", "--gt-tracks", str(gt_path), "--pred", str(pred_path),
         "--classes", "1"],
    )
    main()
    out = capsys.readouterr().out
    assert "MOTA" in out


# --- static-cache replay (chip-free) -------------------------------------

_SRC_W, _SRC_H = 3840, 2160


def _build_cache(tmp_path, *, with_meta=True, frames=range(6)):
    """Build a tiny SqliteCacheStore: 2 tiles tiling the top half horizontally,
    with a STATIONARY person in tile 0 across all frames + a moving second
    person in tile 1. Crop-local normalized dets. ppv=0."""
    from hailo_tiling.cache.store import SqliteCacheStore
    from hailo_tiling.types import CropRect, Det

    db = tmp_path / "dense.db"
    store = SqliteCacheStore.open(db)
    if with_meta:
        store.meta_put("video_w", str(_SRC_W))
        store.meta_put("video_h", str(_SRC_H))

    # Two 4:3 tiles side by side in the top-left quadrant region.
    tile0 = CropRect(x=0, y=0, w=1280, h=960)
    tile1 = CropRect(x=1280, y=0, w=1280, h=960)

    rows = []
    for f in frames:
        # Stationary person centred in tile0 (same crop-local coords every frame).
        d0 = Det(cls=1, score=0.9, x=0.40, y=0.40, w=0.10, h=0.20)
        # A second person in tile1 that drifts slowly rightward inside the tile.
        dx = 0.02 * (f - min(frames))
        d1 = Det(cls=1, score=0.9, x=0.30 + dx, y=0.40, w=0.10, h=0.20)
        rows.append({"frame_idx": int(f), "crop_rect": tile0, "ppv": 0, "dets": [d0]})
        rows.append({"frame_idx": int(f), "crop_rect": tile1, "ppv": 0, "dets": [d1]})
    store.put_many(rows)
    store.close()
    return db


def test_parse_wh():
    assert _parse_wh("3840x2160") == (3840, 2160)
    assert _parse_wh("1920,1080") == (1920, 1080)
    with pytest.raises(ValueError):
        _parse_wh("nope")


def test_replay_static_cache_uses_meta_src_dims(tmp_path):
    db = _build_cache(tmp_path)
    by_track, mean_tiles, n_frames = replay_static_cache(
        db, classes={1}, fps=30)
    assert n_frames == 6
    # two distinct crops per frame
    assert mean_tiles == pytest.approx(2.0)
    assert len(by_track) >= 1
    # every track entry is a {frame_int: (x,y,w,h)} dict
    for tid, traj in by_track.items():
        assert isinstance(tid, int)
        for f, bb in traj.items():
            assert isinstance(f, int)
            assert len(bb) == 4


def test_replay_static_cache_pred_doc_shape(tmp_path):
    db = _build_cache(tmp_path)
    by_track, _, _ = replay_static_cache(db, classes={1}, fps=30)
    doc = by_track_to_pred_doc(by_track)
    assert set(doc) == {"tracks"}
    # round-trips through load_pred
    p = tmp_path / "pred.json"
    p.write_text(json.dumps(doc))
    pred = load_pred(p)
    assert pred  # non-empty
    for tid, traj in pred.items():
        assert isinstance(tid, int)


def test_stationary_person_yields_one_stable_track(tmp_path):
    db = _build_cache(tmp_path)
    by_track, _, _ = replay_static_cache(db, classes={1}, fps=30)
    # The stationary person (tile0) maps to source x ~ 0.40*1280/3840 = 0.133.
    # Find the track that stays put across all frames.
    stable = None
    for tid, traj in by_track.items():
        if len(traj) < 5:
            continue
        xs = [bb[0] for bb in traj.values()]
        if max(xs) - min(xs) < 1e-6:  # truly stationary
            stable = (tid, traj)
    assert stable is not None, "expected one stationary person track"
    tid, traj = stable
    # One stable id spanning (nearly) all frames.
    assert len(traj) >= 5
    # source x for the stationary person: crop.x + 0.40*crop.w over src_w
    exp_x = (0 + 0.40 * 1280) / _SRC_W
    assert all(abs(bb[0] - exp_x) < 1e-6 for bb in traj.values())


def test_replay_requires_src_wh_when_meta_absent(tmp_path):
    db = _build_cache(tmp_path, with_meta=False)
    with pytest.raises(ValueError):
        replay_static_cache(db, classes={1}, fps=30, src_wh=None)
    # explicit src_wh works
    by_track, _, n = replay_static_cache(
        db, classes={1}, fps=30, src_wh=(_SRC_W, _SRC_H))
    assert n == 6 and by_track


def test_main_from_static_cache_writes_pred_and_scores(tmp_path, monkeypatch, capsys):
    db = _build_cache(tmp_path)
    # Build GT matching the stationary person at the mapped source coord.
    exp_x = (0 + 0.40 * 1280) / _SRC_W
    exp_y = (0 + 0.40 * 960) / _SRC_H
    exp_w = (0.10 * 1280) / _SRC_W
    exp_h = (0.20 * 960) / _SRC_H
    gt_doc = {"tracks": [
        {"cls": 1, "track_id": 1,
         "frames": {str(f): [exp_x, exp_y, exp_w, exp_h] for f in range(6)}},
    ]}
    gt_path = _write(tmp_path / "gt.json", gt_doc)
    pred_out = tmp_path / "static_pred.json"
    report = tmp_path / "report.json"
    monkeypatch.setattr(
        "sys.argv",
        ["run_mot_eval", "--from-static-cache", str(db),
         "--classes", "1", "--src-wh", "3840x2160", "--fps", "30",
         "--pred-out", str(pred_out), "--gt-tracks", str(gt_path),
         "--out", str(report)],
    )
    main()
    out = capsys.readouterr().out
    assert "mean tiles/frame" in out
    assert "MOTA" in out
    assert pred_out.exists()
    pred_doc = json.loads(pred_out.read_text())
    assert "tracks" in pred_doc
    rep = json.loads(report.read_text())
    assert rep["static_cache"] == str(db)
    assert rep["mean_tiles_per_frame"] == pytest.approx(2.0)
    # the stationary GT person should be well covered -> high recall
    assert rep["metrics"]["FN"] <= 1


def test_main_static_cache_dump_only(tmp_path, monkeypatch, capsys):
    db = _build_cache(tmp_path)
    pred_out = tmp_path / "pred.json"
    monkeypatch.setattr(
        "sys.argv",
        ["run_mot_eval", "--from-static-cache", str(db),
         "--classes", "1", "--pred-out", str(pred_out)],
    )
    main()  # no --gt-tracks -> dump only, must not raise
    assert pred_out.exists()
    out = capsys.readouterr().out
    assert "MOTA" not in out  # scoring skipped


def test_main_static_cache_and_pred_mutually_exclusive(tmp_path, monkeypatch):
    db = _build_cache(tmp_path)
    pred_path = _write(tmp_path / "pred.json", _pred_perfect_person())
    monkeypatch.setattr(
        "sys.argv",
        ["run_mot_eval", "--from-static-cache", str(db), "--pred", str(pred_path)],
    )
    with pytest.raises(SystemExit):
        main()
