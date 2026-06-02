from dynamic_tiling.metrics import score_trial


def _box(x):  # helper: a unit-ish box at x
    return (x, 0.1, 0.1, 0.1)


def test_single_loss_then_recover():
    # covered 0,1 ; lost 2,3 ; recovered 4,5
    gt = {f: _box(0.1) for f in range(6)}
    pred = {0: _box(0.1), 1: _box(0.1), 4: _box(0.1), 5: _box(0.1)}
    s = score_trial(gt, pred, distractors=[], iou_thr=0.5)
    assert s.loss_events == 1
    assert s.mean_time_to_recover == 2.0     # frames 2,3 lost before recovery at 4
    assert s.recovery_success_rate == 1.0


def test_unrecovered_loss_at_clip_end():
    gt = {f: _box(0.1) for f in range(4)}
    pred = {0: _box(0.1), 1: _box(0.1)}      # lost 2,3 and never recovered
    s = score_trial(gt, pred, distractors=[], iou_thr=0.5)
    assert s.loss_events == 1
    assert s.recovery_success_rate == 0.0
