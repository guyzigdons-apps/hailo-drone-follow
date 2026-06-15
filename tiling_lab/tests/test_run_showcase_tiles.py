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
