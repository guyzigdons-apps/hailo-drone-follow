from hailo_tiling.dynamic.nms import nms_merge


def _d(label, x, y, w=0.05, h=0.04, conf=0.9, age=0):
    return {"label": label, "confidence": conf, "bbox": [x, y, w, h], "age": age}


def test_target_wins_over_overlapping_stale_box():
    target = _d("target", 0.50, 0.50, age=0)
    stale = _d("vehicle", 0.505, 0.505, age=12)
    out = nms_merge([target, stale], iou_thresh=0.3)
    assert len(out) == 1 and out[0]["label"] == "target"


def test_fresher_box_wins_when_neither_is_target():
    fresh = _d("vehicle", 0.50, 0.50, age=0)
    stale = _d("vehicle", 0.505, 0.505, age=15)
    out = nms_merge([stale, fresh], iou_thresh=0.3)
    assert len(out) == 1 and out[0]["age"] == 0


def test_non_overlapping_boxes_all_survive():
    a = _d("vehicle", 0.10, 0.10)
    b = _d("vehicle", 0.80, 0.80)
    out = nms_merge([a, b], iou_thresh=0.3)
    assert len(out) == 2
