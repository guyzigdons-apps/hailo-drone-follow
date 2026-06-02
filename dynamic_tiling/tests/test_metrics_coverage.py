from dynamic_tiling.metrics import score_trial


def test_perfect_coverage():
    gt = {0: (0.1, 0.1, 0.2, 0.2), 1: (0.1, 0.1, 0.2, 0.2)}
    pred = dict(gt)
    s = score_trial(gt, pred, distractors=[], iou_thr=0.5)
    assert s.coverage == 1.0
    assert s.mean_iou > 0.99
    assert s.drift_rate == 0.0


def test_miss_counts_against_coverage():
    gt = {0: (0.1, 0.1, 0.2, 0.2), 1: (0.1, 0.1, 0.2, 0.2)}
    pred = {0: (0.1, 0.1, 0.2, 0.2)}            # frame 1 missing
    s = score_trial(gt, pred, distractors=[], iou_thr=0.5)
    assert s.coverage == 0.5


def test_drift_when_pred_matches_distractor():
    gt = {0: (0.1, 0.1, 0.2, 0.2)}
    distractor = {0: (0.6, 0.6, 0.2, 0.2)}
    pred = {0: (0.6, 0.6, 0.2, 0.2)}            # locked onto the distractor
    s = score_trial(gt, pred, distractors=[distractor], iou_thr=0.5)
    assert s.coverage == 0.0
    assert s.drift_rate == 1.0
