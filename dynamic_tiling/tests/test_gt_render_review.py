from dynamic_tiling.gt_review import ReviewCase
from dynamic_tiling.gt_render_review import case_caption, norm_to_px


def test_case_caption_merge():
    c = ReviewCase(kind="merge", frame=868, track_ids=(168, 193), boxes=[],
                   reason="shared=40 mean_iou=0.55", score=0.55)
    cap = case_caption(c, index=0)
    assert "MERGE" in cap.upper() and "168" in cap and "193" in cap and "868" in cap


def test_case_caption_keep_short():
    c = ReviewCase(kind="keep_short", frame=12, track_ids=(9,), boxes=[],
                   reason="len=12 < min_len=30", score=12.0)
    cap = case_caption(c, index=3)
    assert "9" in cap and ("KEEP" in cap.upper() or "REAL" in cap.upper())


def test_norm_to_px():
    assert norm_to_px((0.5, 0.5, 0.1, 0.2), 1920, 1080) == (960, 540, 192, 216)
