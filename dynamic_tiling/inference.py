"""Compatibility shim: backends live in hailo_tiling.backends.

`HefBackend` here is a **thin wrapper** around `hailo_tiling.backends.hef.HefBackend`
that exposes the legacy single-crop `infer(frame, crop, frame_idx) -> list` API
used by `dynamic_tiling.run_dynamic` and `dynamic_tiling.replay`. The wrapped
class uses the new batched `infer(frame, crops, frame_idx) -> list[list[Det]]`
API internally.

`ReplayBackend` is kept here as a legacy single-crop helper used by
`dynamic_tiling.tests.test_inference`. The Plan-1-style per-crop API
(infer(frame, crop, frame_idx)) is preserved exactly.

This shim disappears in Plan 8 (drone-follow migration) once all callers
move to hailo_tiling.
"""
from __future__ import annotations

from typing import Protocol

from hailo_tiling.backends.hef import HefBackend as _BatchedHefBackend

from .types import CropRect


class InferenceBackend(Protocol):
    """Legacy single-crop protocol; preserved for dynamic_tiling callers.

    The batched ABC lives in `hailo_tiling.backends.backend.InferenceBackend`.
    """

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:  # noqa: D401
        ...


class HefBackend:
    """Single-crop adapter over the batched `hailo_tiling.backends.hef.HefBackend`.

    Forwards `infer(frame, crop, frame_idx)` to the underlying backend's
    `infer(frame, [crop], frame_idx)` and unwraps the single result list.
    Constructor args are passed through verbatim.
    """

    def __init__(self, *args, **kwargs):
        self._inner = _BatchedHefBackend(*args, **kwargs)

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:
        return self._inner.infer(frame, [crop], frame_idx)[0]

    def close(self) -> None:
        self._inner.close()


class ReplayBackend:
    """Legacy deterministic backend keyed on (frame_idx, (x, y, w, h)).

    Single-crop API. The batched ReplayBackend (`hailo_tiling.backends`) lands
    in Plan 4 alongside the SQLite cache layer.
    """

    def __init__(self, canned: dict):
        self._canned = canned

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:
        return list(self._canned.get((frame_idx, (crop.x, crop.y, crop.w, crop.h)), []))
