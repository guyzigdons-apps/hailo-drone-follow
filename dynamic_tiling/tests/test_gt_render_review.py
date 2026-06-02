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


def test_crop_region_covers_boxes_with_margin_and_min_size():
    from dynamic_tiling.gt_render_review import crop_region
    boxes = [(2, 10, (0.50, 0.47, 0.02, 0.02)), (2, 17, (0.49, 0.47, 0.03, 0.02))]
    x, y, w, h = crop_region(boxes, 1920, 1080, pad_frac=3.0, min_frac=0.18)
    # crop stays in-frame
    assert 0 <= x and 0 <= y and x + w <= 1920 and y + h <= 1080
    # min size enforced (>= 0.18 of each dim)
    assert w >= int(0.18 * 1920) - 1 and h >= int(0.18 * 1080) - 1
    # union center (~0.505, 0.48) is inside the crop
    cx, cy = 0.505 * 1920, 0.48 * 1080
    assert x <= cx <= x + w and y <= cy <= y + h


def test_crop_region_empty_boxes_returns_full_frame():
    from dynamic_tiling.gt_render_review import crop_region
    assert crop_region([], 1920, 1080) == (0, 0, 1920, 1080)
