import pytest


def _traj(tid, frames, x0=0.1, dx=0.0, y=0.5, w=0.05, h=0.1):
    return {f: (x0 + dx * i, y, w, h) for i, f in enumerate(frames)}


def test_perfect_tracking_scores_perfectly():
    from tiling_lab.harness.mot_metrics import score_mot
    gt = {1: _traj(1, range(10)), 2: _traj(2, range(10), x0=0.7)}
    pred = {10: _traj(10, range(10)), 20: _traj(20, range(10), x0=0.7)}
    m = score_mot(gt, pred, iou_thr=0.5)
    assert m["MOTA"] == 1.0 and m["IDF1"] == 1.0
    assert m["IDsw"] == 0 and m["FP"] == 0 and m["FN"] == 0
    assert m["MT"] == 2 and m["ML"] == 0


def test_identity_swap_costs_idsw_and_idf1():
    from tiling_lab.harness.mot_metrics import score_mot
    gt = {1: _traj(1, range(10))}
    pred = {10: _traj(10, range(5)),          # first half id 10
            11: {f: (0.1, 0.5, 0.05, 0.1) for f in range(5, 10)}}  # second half id 11
    m = score_mot(gt, pred, iou_thr=0.5)
    assert m["IDsw"] == 1
    assert m["MOTA"] < 1.0                     # IDsw penalised
    assert 0.4 < m["IDF1"] <= 0.6              # best identity covers half


def test_fp_flood_and_misses():
    from tiling_lab.harness.mot_metrics import score_mot
    gt = {1: _traj(1, range(10))}
    pred = {10: _traj(10, range(6)),                       # 4 FN
            99: {f: (0.9, 0.9, 0.05, 0.1) for f in range(10)}}  # 10 FP
    m = score_mot(gt, pred, iou_thr=0.5)
    assert m["FN"] == 4 and m["FP"] == 10
    assert m["MOTA"] == 1 - (4 + 10 + 0) / 10


def test_fragmentation_counted():
    from tiling_lab.harness.mot_metrics import score_mot
    gt = {1: _traj(1, range(12))}
    pred = {10: {f: (0.1, 0.5, 0.05, 0.1) for f in (0, 1, 2, 3, 6, 7, 8, 11)}}
    m = score_mot(gt, pred, iou_thr=0.5)
    assert m["Frag"] == 2                      # two gaps inside a matched GT track


def test_empty_pred_is_all_misses():
    from tiling_lab.harness.mot_metrics import score_mot
    gt = {1: _traj(1, range(5))}
    m = score_mot(gt, {}, iou_thr=0.5)
    assert m["FN"] == 5 and m["MOTA"] == 0.0 and m["IDF1"] == 0.0 and m["ML"] == 1


def test_empty_gt_raises():
    from tiling_lab.harness.mot_metrics import score_mot
    with pytest.raises(ValueError, match="score_mot: empty GT"):
        score_mot({}, {10: _traj(10, range(5))}, iou_thr=0.5)
