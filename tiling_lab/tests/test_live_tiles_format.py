import math
from hailo_tiling.types import CropRect
from tiling_lab.live.tiles_format import crops_to_tiles_static


def test_empty_crops_returns_empty_string():
    assert crops_to_tiles_static([], 1280, 720) == ""


def test_single_crop_normalized_with_mode():
    # 256x144 px crop at (128,72) in a 1280x720 frame -> 0.1,0.1,0.2,0.2
    c = CropRect(x=128, y=72, w=256, h=144, mode="s")
    out = crops_to_tiles_static([c], 1280, 720)
    assert out == "0.100000,0.100000,0.200000,0.200000,s"


def test_multiple_crops_semicolon_joined():
    c1 = CropRect(x=0, y=0, w=640, h=360, mode="m")
    c2 = CropRect(x=640, y=360, w=640, h=360, mode="s")
    out = crops_to_tiles_static([c1, c2], 1280, 720)
    assert out == "0.000000,0.000000,0.500000,0.500000,m;0.500000,0.500000,0.500000,0.500000,s"


def test_crop_overflowing_frame_is_clamped_into_unit_square():
    # crop runs past the right/bottom edge -> clamp so x+w<=1, y+h<=1
    c = CropRect(x=1152, y=648, w=256, h=144, mode="s")  # x/w=0.9..1.1
    out = crops_to_tiles_static([c], 1280, 720)
    x, y, w, h, mode = out.split(",")
    assert math.isclose(float(x) + float(w), 1.0, abs_tol=1e-6)
    assert math.isclose(float(y) + float(h), 1.0, abs_tol=1e-6)
    assert float(x) >= 0.0 and float(y) >= 0.0


def test_negative_origin_is_clamped_to_zero():
    c = CropRect(x=-64, y=-36, w=256, h=144, mode="s")
    out = crops_to_tiles_static([c], 1280, 720)
    x, y, w, h, _ = out.split(",")
    assert float(x) == 0.0 and float(y) == 0.0
    assert float(w) > 0.0 and float(h) > 0.0
