from hailo_tiling.types import CropRect, Det, LockState, TargetState, MODEL_W, MODEL_H, MODEL_ASPECT


def test_model_constants():
    assert (MODEL_W, MODEL_H) == (640, 480)
    assert abs(MODEL_ASPECT - 640 / 480) < 1e-9


def test_croprect_scale_and_aspect():
    assert CropRect.from_center_width(cx=1000, cy=800, crop_w=640).scale == 1.0
    assert CropRect.from_center_width(cx=1000, cy=800, crop_w=320).scale == 2.0
    r = CropRect.from_center_width(cx=1000, cy=800, crop_w=640)
    assert r.h == round(640 / MODEL_ASPECT)  # 480


def test_croprect_clamp_keeps_inside_bounds():
    r = CropRect.from_center_width(cx=10, cy=10, crop_w=640).clamp(src_w=4000, src_h=3000)
    assert r.x >= 0 and r.y >= 0
    assert r.x + r.w <= 4000 and r.y + r.h <= 3000


def test_lockstate_defaults():
    s = LockState()
    assert s.track_id is None and s.status == "LOST" and s.frames_since_seen == 0


def test_target_state_defaults():
    ts = TargetState(key=(0, 1), cls=0)
    assert ts.status == "LOST"
    assert ts.frames_since_seen == 0
    assert ts.selected is False
    assert ts.bbox_norm == (0.0, 0.0, 0.0, 0.0)
    assert ts.last_velocity == (0.0, 0.0)
