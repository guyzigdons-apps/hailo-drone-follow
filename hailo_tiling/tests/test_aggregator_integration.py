"""Top-level Aggregator.aggregate — composes NMS + boundary strip + memory."""
from __future__ import annotations

from hailo_tiling.aggregator import (
    Aggregator,
    BoundaryStripFilter,
    NoOpMemory,
)
from hailo_tiling.types import CropRect, Det


class _CropLocalDet:
    def __init__(self, cls, x, y, w, h, score):
        self.cls, self.x, self.y, self.w, self.h, self.score = cls, x, y, w, h, score


def test_aggregate_maps_dedups_and_emits_source_normalized():
    src_w, src_h = 4000, 3000
    # Two overlapping 's'-mode crops that both cover the same person near
    # x=608..640. Each crop emits its own copy of the detection; the source
    # bboxes coincide, so per-class NMS must keep only the higher-scoring one.
    crops = [
        CropRect(x=0,   y=0, w=640, h=480, mode="s"),
        CropRect(x=600, y=0, w=640, h=480, mode="s"),
    ]
    dets_per_crop = [
        [_CropLocalDet(cls=0, x=0.95,   y=0.1, w=0.05, h=0.1, score=0.9)],
        [_CropLocalDet(cls=0, x=0.0125, y=0.1, w=0.05, h=0.1, score=0.7)],
    ]
    agg = Aggregator(boundary_strip=BoundaryStripFilter(border_threshold=0.0),
                     memory=NoOpMemory(), iou_thr=0.5)
    out = agg.aggregate(frame_idx=0, crops=crops, dets_per_crop=dets_per_crop,
                         src_w=src_w, src_h=src_h)
    assert len(out) == 1
    assert out[0].cls == 0
    assert out[0].score == 0.9


def test_aggregate_applies_boundary_strip():
    src_w, src_h = 3840, 2160
    tile_left = CropRect(x=0, y=0, w=1280, h=960, mode="m")
    tile_right = CropRect(x=1280, y=0, w=1280, h=960, mode="m")
    det_local = _CropLocalDet(cls=0, x=0.98, y=0.1, w=0.02, h=0.05, score=0.9)
    agg = Aggregator(boundary_strip=BoundaryStripFilter(border_threshold=0.005))
    out = agg.aggregate(frame_idx=0,
                         crops=[tile_left, tile_right],
                         dets_per_crop=[[det_local], []],
                         src_w=src_w, src_h=src_h)
    assert out == []


def test_aggregate_with_backend_e2e(make_mock_backend):
    src_w, src_h = 4000, 3000
    crops = [CropRect(x=1000, y=500, w=640, h=480, mode="s")]
    canned = {
        (0, (1000, 500, 640, 480)): [
            Det(cls=0, score=0.9, x=0.25, y=0.25, w=0.5, h=0.5),
        ],
    }
    be = make_mock_backend(canned)
    dets_per_crop = be.infer(frame=None, crops=crops, frame_idx=0)
    agg = Aggregator()
    out = agg.aggregate(frame_idx=0, crops=crops, dets_per_crop=dets_per_crop,
                         src_w=src_w, src_h=src_h)
    assert len(out) == 1
    o = out[0]
    assert abs(o.x - 1160 / 4000) < 1e-6
    assert o.cls == 0


def test_aggregate_invokes_memory_observe_and_predict():
    class _RecordingMemory:
        def __init__(self):
            self.observed = []
            self._to_inject = [Det(cls=1, score=0.5, x=0.5, y=0.5, w=0.01, h=0.01)]
        def observe(self, dets, frame_idx):
            self.observed.append((frame_idx, list(dets)))
        def predict(self, frame_idx):
            return list(self._to_inject)
        def reset(self):
            pass

    mem = _RecordingMemory()
    src_w, src_h = 4000, 3000
    crops = [CropRect(x=0, y=0, w=640, h=480, mode="s")]
    dets = [[_CropLocalDet(cls=0, x=0.25, y=0.25, w=0.1, h=0.1, score=0.9)]]
    agg = Aggregator(memory=mem)
    out = agg.aggregate(frame_idx=0, crops=crops, dets_per_crop=dets,
                         src_w=src_w, src_h=src_h)
    assert len(out) == 2
    assert any(d.cls == 1 for d in out)
    assert mem.observed[0][0] == 0
