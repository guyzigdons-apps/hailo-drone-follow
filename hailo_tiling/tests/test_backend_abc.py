"""InferenceBackend ABC contract + MockBackend fixture sanity."""
from __future__ import annotations

import numpy as np
import pytest

from hailo_tiling.backends import InferenceBackend, MockBackend
from hailo_tiling.types import CropRect, Det


def test_cannot_instantiate_bare_abc():
    with pytest.raises(TypeError):
        InferenceBackend()  # type: ignore[abstract]


def test_subclass_without_infer_method_fails():
    class _Bad(InferenceBackend):
        pass
    with pytest.raises(TypeError):
        _Bad()  # type: ignore[abstract]


def test_mock_backend_returns_one_list_per_crop():
    canned = {
        (0, (0, 0, 640, 480)): [Det(cls=0, score=0.9, x=0.1, y=0.1, w=0.05, h=0.1)],
        (0, (640, 0, 640, 480)): [],
    }
    be = MockBackend(canned)
    crops = [
        CropRect(x=0, y=0, w=640, h=480, mode="s"),
        CropRect(x=640, y=0, w=640, h=480, mode="s"),
    ]
    frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
    out = be.infer(frame, crops, frame_idx=0)
    assert len(out) == 2
    assert len(out[0]) == 1 and out[0][0].cls == 0
    assert out[1] == []


def test_mock_backend_unknown_crop_returns_empty_list():
    be = MockBackend({})
    crops = [CropRect(x=0, y=0, w=640, h=480, mode="s")]
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    out = be.infer(frame, crops, frame_idx=0)
    assert out == [[]]


def test_mock_backend_records_calls_for_assertions():
    be = MockBackend({})
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    crops_a = [CropRect(x=0, y=0, w=640, h=480, mode="s")]
    crops_b = [CropRect(x=10, y=10, w=640, h=480, mode="s")]
    be.infer(frame, crops_a, frame_idx=0)
    be.infer(frame, crops_b, frame_idx=1)
    assert be.call_count == 2
    assert be.calls[0]["frame_idx"] == 0
    assert be.calls[1]["crops"] == crops_b
