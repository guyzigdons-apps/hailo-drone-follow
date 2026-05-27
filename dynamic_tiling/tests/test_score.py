from dynamic_tiling.score import score_run, RunScore


def test_score_run_recall_and_iou():
    gt = {0: (0.40, 0.40, 0.10, 0.25), 1: (0.42, 0.40, 0.10, 0.25),
          2: (0.44, 0.40, 0.10, 0.25)}
    pred = {0: (0.40, 0.40, 0.10, 0.25),   # perfect
            1: (0.42, 0.41, 0.10, 0.25),   # good overlap
            2: None}                       # missed
    s = score_run(gt, pred, iou_thr=0.5)
    assert isinstance(s, RunScore)
    assert s.n_gt_frames == 3
    assert s.n_hit == 2
    assert abs(s.recall - 2 / 3) < 1e-9
    assert s.mean_iou > 0.8  # over the two hit frames


def test_score_run_empty_gt():
    s = score_run({}, {}, iou_thr=0.5)
    assert s.n_gt_frames == 0 and s.recall == 0.0 and s.mean_iou == 0.0
