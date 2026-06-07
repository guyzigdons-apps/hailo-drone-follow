"""Legacy single-crop inference shim — contract tests.

``tiling_lab.harness.inference.{HefBackend, ReplayBackend}`` expose the legacy
single-crop API (``infer(frame, crop, frame_idx)`` -> flat list) that
``tiling_lab.cli.run_dynamic`` / ``tiling_lab.harness.replay`` rely on. HefBackend
wraps the batched ``hailo_tiling.backends.hef.HefBackend`` internally.

These tests used to live in ``hailo_tiling/tests/test_backend_hef_shim.py`` but
moved here when the harness left ``hailo_tiling``'s allowed import set
(tiling-lab restructure, 2026-06-07): ``hailo_tiling`` must not import
``tiling_lab``, so the lab-shim coverage belongs in the lab's own suite.
"""
from __future__ import annotations

import numpy as np

from hailo_tiling.types import CropRect


class _FakeHandle:
    """Stand-in for the HefHandle the backend wraps."""

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


def test_legacy_inference_hef_backend_is_single_crop_wrapper():
    """tiling_lab.harness.inference.HefBackend exposes the legacy single-crop API.

    It wraps the new batched hailo_tiling.backends.hef.HefBackend internally so
    run_dynamic / replay keep working through the shim (they call
    ``backend.infer(frame, crop, frame_idx)`` with a single CropRect).
    """
    from tiling_lab.harness.inference import HefBackend as LegacyHefBackend
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
    """tiling_lab.harness.inference.ReplayBackend keeps its single-crop API."""
    from tiling_lab.harness.inference import ReplayBackend

    canned = {(0, (0, 0, 640, 480)): [_FakeDet()]}
    be = ReplayBackend(canned)
    out = be.infer(frame=np.zeros((480, 640, 3), dtype=np.uint8),
                   crop=CropRect(x=0, y=0, w=640, h=480, mode="s"),
                   frame_idx=0)
    assert len(out) == 1
