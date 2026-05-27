import numpy as np
from dynamic_tiling.types import CropRect
from dynamic_tiling.inference import ReplayBackend


class _CropLocalDet:
    def __init__(self, cls, x, y, w, h, score):
        self.cls, self.x, self.y, self.w, self.h, self.score = cls, x, y, w, h, score


def test_replay_backend_returns_canned_crop_local_dets():
    canned = {(0, (1000, 500, 640, 480)): [
        _CropLocalDet(cls=0, x=0.25, y=0.25, w=0.5, h=0.5, score=0.9)]}
    be = ReplayBackend(canned)
    crop = CropRect(x=1000, y=500, w=640, h=480)
    dets = be.infer(frame=np.zeros((3000, 4000, 3), np.uint8), crop=crop, frame_idx=0)
    assert len(dets) == 1 and dets[0].cls == 0


def test_replay_backend_empty_for_unknown_crop():
    be = ReplayBackend({})
    crop = CropRect(x=0, y=0, w=640, h=480)
    assert be.infer(frame=np.zeros((480, 640, 3), np.uint8), crop=crop, frame_idx=5) == []
