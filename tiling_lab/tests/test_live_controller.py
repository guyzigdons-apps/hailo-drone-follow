from hailo_tiling.types import Det
from tiling_lab.live.controller import DynamicTilingController


def _person(cx, cy, w=0.05, h=0.12, score=0.9):
    # Det stores top-left x,y normalized
    return Det(cls=0, score=score, x=cx - w / 2, y=cy - h / 2, w=w, h=h)


def test_first_update_returns_a_string():
    ctrl = DynamicTilingController(src_w=1280, src_h=720, fps=30.0,
                                   budget_inf_per_s=40.0)
    out = ctrl.update([])
    assert isinstance(out, str)


def test_acquires_then_follows_target():
    ctrl = DynamicTilingController(src_w=1280, src_h=720, fps=30.0,
                                   budget_inf_per_s=60.0)
    last = ""
    for f in range(60):
        last = ctrl.update([_person(0.5, 0.5)])
    assert ctrl.status == "TRACKING"
    assert last != ""  # tiles emitted while tracking
    for rect in last.split(";"):
        x, y, w, h, mode = rect.split(",")
        assert 0.0 <= float(x) <= 1.0 and 0.0 <= float(y) <= 1.0
        assert float(w) > 0.0 and float(h) > 0.0
        assert mode in ("s", "m")


def test_budget_is_charged_and_tiles_per_frame_tracked():
    ctrl = DynamicTilingController(src_w=1280, src_h=720, fps=30.0,
                                   budget_inf_per_s=60.0)
    for f in range(30):
        ctrl.update([_person(0.5, 0.5)])
    assert 0.0 < ctrl.mean_tiles_per_frame < 10.0
    assert ctrl.frame_count == 30


def test_no_detections_eventually_reports_searching_or_lost():
    ctrl = DynamicTilingController(src_w=1280, src_h=720, fps=30.0,
                                   budget_inf_per_s=60.0)
    for f in range(10):
        ctrl.update([_person(0.5, 0.5)])  # acquire
    for f in range(5):
        ctrl.update([])  # target gone
    assert ctrl.status in ("SEARCHING", "LOST")
