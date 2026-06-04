import json
from pathlib import Path

from dynamic_tiling.run_mot_eval import (
    load_gt_as_mot,
    load_pred,
    format_metrics,
    main,
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
    from dynamic_tiling.mot_metrics import score_mot
    m = score_mot(gt, pred, iou_thr=0.5)
    assert m["MOTA"] == 1.0
    assert m["IDF1"] == 1.0
    assert m["FP"] == 0 and m["FN"] == 0 and m["IDsw"] == 0


def test_class_filter_excludes_vehicles_affects_score(tmp_path):
    # If we DON'T filter, the vehicle GT track is unmatched -> FN, MOTA<1.
    gt_path = _write(tmp_path / "gt.json", _gt_doc())
    pred_path = _write(tmp_path / "pred.json", _pred_perfect_person())
    pred = load_pred(pred_path)
    from dynamic_tiling.mot_metrics import score_mot

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
