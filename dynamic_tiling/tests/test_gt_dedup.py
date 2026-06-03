from dynamic_tiling.gt_dedup import dedup_frame, dedup_doc, det_cls


def test_dedup_merges_overlapping_same_class():
    dets = [
        {"bbox": [0.42, 0.64, 0.017, 0.054], "confidence": 0.90, "cls": 1},
        {"bbox": [0.424, 0.633, 0.011, 0.045], "confidence": 0.84, "cls": 1},  # dup of above
        {"bbox": [0.326, 0.388, 0.008, 0.031], "confidence": 0.68, "cls": 1},  # separate person
    ]
    out = dedup_frame(dets, iou_thr=0.5)
    assert len(out) == 2                       # two distinct persons
    assert out[0]["confidence"] == 0.90        # kept the higher-confidence box


def test_dedup_keeps_different_classes_separate():
    dets = [
        {"bbox": [0.5, 0.47, 0.05, 0.05], "confidence": 0.9, "cls": 1},
        {"bbox": [0.5, 0.47, 0.05, 0.05], "confidence": 0.8, "cls": 2},  # same place, diff class
    ]
    out = dedup_frame(dets, iou_thr=0.5)
    assert len(out) == 2


def test_dedup_doc_processes_all_frames():
    doc = {"frames": [
        {"frame": 0, "detections": [
            {"bbox": [0.4, 0.6, 0.02, 0.05], "confidence": 0.9, "cls": 1},
            {"bbox": [0.405, 0.6, 0.02, 0.05], "confidence": 0.8, "cls": 1}]}]}
    out = dedup_doc(doc, iou_thr=0.5)
    assert len(out["frames"][0]["detections"]) == 1


def test_dedup_reads_class_id_key():
    assert det_cls({"class_id": 2}) == 2
    assert det_cls({"cls": 1}) == 1
    assert det_cls({"label": "x"}) == -1
    # two same-class (class_id) overlapping dets dedup to one
    dets = [{"bbox": [0.4, 0.6, 0.02, 0.05], "confidence": 0.9, "class_id": 1},
            {"bbox": [0.405, 0.6, 0.02, 0.05], "confidence": 0.8, "class_id": 1}]
    assert len(dedup_frame(dets, iou_thr=0.5)) == 1
