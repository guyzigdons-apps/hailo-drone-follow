from tiling_lab.harness.score import score_run, RunScore, compare


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
    assert abs(s.mean_iou - 0.9615) < 1e-3  # over the two hit frames
    assert s.per_frame_iou[2] is None


def test_score_run_empty_gt():
    s = score_run({}, {}, iou_thr=0.5)
    assert s.n_gt_frames == 0 and s.recall == 0.0 and s.mean_iou == 0.0


def test_compare_carries_fields():
    gt = {0: (0.4, 0.4, 0.1, 0.25)}
    s = score_run(gt, {0: (0.4, 0.4, 0.1, 0.25)}, iou_thr=0.5)
    out = compare("dyn", s, "static", s)
    assert out["dyn"]["recall"] == s.recall
    assert out["dyn"]["n_gt_frames"] == 1
    assert out["static"]["hits"] == s.n_hit
