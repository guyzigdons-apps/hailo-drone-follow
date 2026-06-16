"""Pure unit tests for run_showcase.tiles_static_to_dicts (no device needed)."""
from tiling_lab.live.run_showcase import tiles_static_to_dicts


def test_parses_segments_into_viewer_schema():
    s = "0.0,0.0,0.125,0.166667,m;0.5,0.5,0.1,0.1,s"
    out = tiles_static_to_dicts(s)
    assert out == [
        {"x": 0.0, "y": 0.0, "w": 0.125, "h": 0.166667,
         "category": "multi-scale"},
        {"x": 0.5, "y": 0.5, "w": 0.1, "h": 0.1, "category": "dynamic"},
    ]


def test_mode_maps_to_category():
    # 'm' (dense discovery) -> multi-scale; anything else (ROI/recovery) -> dynamic.
    assert tiles_static_to_dicts("0,0,0.1,0.1,m")[0]["category"] == "multi-scale"
    assert tiles_static_to_dicts("0,0,0.1,0.1,s")[0]["category"] == "dynamic"


def test_empty_and_malformed_segments_are_skipped():
    assert tiles_static_to_dicts("") == []
    # trailing ';' and a too-short segment are ignored, valid ones kept.
    out = tiles_static_to_dicts("0.1,0.2,0.3,0.4,m;;1,2,3")
    assert len(out) == 1
    assert out[0]["x"] == 0.1


from tiling_lab.live.run_showcase import build_frame_record, tiles_static_to_dicts


def test_frame_record_uses_previous_tiles_not_next():
    # Frame 5's detections were produced by the tiles applied to frame 5, which
    # were pushed during frame 4's probe (= prev_pushed). The record must carry
    # prev_pushed, NOT the tiles just computed for frame 6.
    prev_pushed = "0.1,0.1,0.1,0.1,m"     # tiles that produced these dets
    next_pushed = "0.8,0.8,0.1,0.1,m"     # tiles for the NEXT frame
    dets = [{"label": "vehicle", "confidence": 0.9, "bbox": [0.12, 0.12, 0.03, 0.02]}]
    rec = build_frame_record(frame=5, detections=dets, applied_tiles=prev_pushed)
    assert rec["frame"] == 5
    assert rec["detections"] == dets
    assert rec["tiles"] == tiles_static_to_dicts(prev_pushed)
    assert rec["tiles"] != tiles_static_to_dicts(next_pushed)
