"""HefBackend — shim contract.

The test mocks the HailoRT-touching parts of HefBackend so it runs without a
Hailo chip, asserting the batched `infer(frame, crops, frame_idx)` shape.

The legacy single-crop shim tests moved to
``tiling_lab/tests/test_legacy_inference_shim.py`` (the lab's inference wrapper
must not be imported from hailo_tiling — tiling-lab restructure, 2026-06-07).
"""
from __future__ import annotations

import numpy as np
import pytest

from hailo_tiling.backends import HefBackend
from hailo_tiling.types import CropRect, Det


class _FakeHandle:
    """Stand-in for tiling_benchmark HefHandle."""

    def __init__(self):
        self.calls = []
        self.closed = False

    def infer(self, rgb_chw_or_hwc):
        self.calls.append(rgb_chw_or_hwc.shape)
        return "RAW"

    def close(self):
        self.closed = True


class _FakeDet:
    def __init__(self, cls=0, x=0.25, y=0.25, w=0.5, h=0.5, score=0.9):
        self.cls, self.x, self.y, self.w, self.h, self.score = cls, x, y, w, h, score


def _fake_decode(raw):
    assert raw == "RAW"
    return [_FakeDet()]


@pytest.fixture
def patched_hef(monkeypatch):
    """Patch HefBackend internals so __init__ does no HailoRT work."""
    handle = _FakeHandle()

    def _from_handle(self, handle_obj, decode, class_offset=0):
        self._handle = handle_obj
        self._decode = decode
        self._class_offset = class_offset

    monkeypatch.setattr(HefBackend, "__init__", _from_handle, raising=False)
    backend = HefBackend(handle, _fake_decode)  # type: ignore[call-arg]
    return backend, handle


def test_batched_infer_returns_list_per_crop(patched_hef):
    be, _ = patched_hef
    crops = [
        CropRect(x=0, y=0, w=640, h=480, mode="s"),
        CropRect(x=10, y=10, w=640, h=480, mode="s"),
    ]
    frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
    out = be.infer(frame, crops, frame_idx=0)
    assert len(out) == 2
    assert all(len(dets) == 1 for dets in out)
