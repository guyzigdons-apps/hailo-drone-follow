"""Compatibility shim: backends live in hailo_tiling.backends.

`HefBackend` is the same class as `hailo_tiling.backends.hef.HefBackend`.
`ReplayBackend` is kept here as a legacy single-crop helper used by
`dynamic_tiling.tests.test_inference`. The Plan-1-style per-crop API
(infer(frame, crop, frame_idx)) is preserved exactly.

This shim disappears in Plan 8 (drone-follow migration) once all callers
move to hailo_tiling.
"""
from __future__ import annotations

from typing import Protocol

from hailo_tiling.backends.hef import HefBackend  # noqa: F401

from .types import CropRect


class InferenceBackend(Protocol):
    """Legacy single-crop protocol; preserved for dynamic_tiling callers."""

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:  # noqa: D401
        ...


class ReplayBackend:
    """Legacy deterministic backend keyed on (frame_idx, (x, y, w, h)).

    Single-crop API. The batched ReplayBackend lands in Plan 4.
    """

    def __init__(self, canned: dict):
        self._canned = canned

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:
        return list(self._canned.get((frame_idx, (crop.x, crop.y, crop.w, crop.h)), []))
