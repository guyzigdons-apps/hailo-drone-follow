"""HefBackend — shim contract.

The test mocks the HailoRT-touching parts of HefBackend so it runs without a
Hailo chip, asserting the batched `infer(frame, crops, frame_idx)` shape and
the legacy single-crop shim path used by dynamic_tiling.
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


def test_legacy_dynamic_tiling_inference_hef_backend_is_single_crop_wrapper():
    """dynamic_tiling.inference.HefBackend must expose the legacy single-crop API.

    It wraps the new batched hailo_tiling.backends.hef.HefBackend internally so
    dynamic_tiling.run_dynamic / dynamic_tiling.replay keep working through the
    shim (they call `backend.infer(frame, crop, frame_idx)` with a single CropRect).
    """
    import numpy as np

    from dynamic_tiling.inference import HefBackend as LegacyHefBackend
    from hailo_tiling.backends.hef import HefBackend as BatchedHefBackend

    # Different classes — wrapper, not identity.
    assert LegacyHefBackend is not BatchedHefBackend

    # Test-injection construction round-trips through the wrapper.
    handle = _FakeHandle()
    legacy = LegacyHefBackend(handle, _fake_decode)

    crop = CropRect(x=0, y=0, w=640, h=480, mode="s")
    frame = np.zeros((1080, 1920, 3), dtype=np.uint8)

    # Legacy single-crop API: returns a flat list (not list-of-lists).
    out = legacy.infer(frame, crop, frame_idx=0)
    assert len(out) == 1  # _fake_decode returns one _FakeDet


def test_legacy_replay_backend_still_works():
    """dynamic_tiling.inference.ReplayBackend keeps its single-crop API."""
    from dynamic_tiling.inference import ReplayBackend
    canned = {(0, (0, 0, 640, 480)): [_FakeDet()]}
    be = ReplayBackend(canned)
    out = be.infer(frame=np.zeros((480, 640, 3), dtype=np.uint8),
                    crop=CropRect(x=0, y=0, w=640, h=480, mode="s"),
                    frame_idx=0)
    assert len(out) == 1
