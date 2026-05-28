"""InferenceBackend ABC + MockBackend.

The spec (§3.4) defines `infer(frame, crops) -> list[list[Det]]`: one call
covers an ordered batch of crops and returns one detection-list per crop in
the same order. This batched signature is what GstCropperBackend (Plan 6)
needs; HefBackend (Task 10) implements it by looping over crops internally
so the on-chip code path stays per-crop.

`MockBackend` is the chip-free fixture used throughout the rest of the
hailo_tiling test suite — no Hailo hardware, no HailoRT import.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Sequence

from ..types import CropRect, Det


class InferenceBackend(ABC):
    """Run inference on an ordered batch of crops, return one list per crop."""

    @abstractmethod
    def infer(
        self,
        frame: Any,
        crops: Sequence[CropRect],
        frame_idx: int,
    ) -> list[list[Det]]:
        """Return crop-local normalized detections, one list per input crop, in order."""

    def close(self) -> None:  # pragma: no cover - default no-op
        return None


class MockBackend(InferenceBackend):
    """Test double. Looks up canned `(frame_idx, (x,y,w,h)) -> list[Det]` entries.

    Records every call into `self.calls` for assertion-by-test patterns.
    """

    def __init__(self, canned: dict[tuple[int, tuple[int, int, int, int]], list[Det]] | None = None):
        self.canned = dict(canned or {})
        self.calls: list[dict] = []

    @property
    def call_count(self) -> int:
        return len(self.calls)

    def infer(self, frame, crops, frame_idx):
        self.calls.append({"frame_idx": frame_idx, "crops": list(crops)})
        out: list[list[Det]] = []
        for c in crops:
            key = (frame_idx, (c.x, c.y, c.w, c.h))
            out.append(list(self.canned.get(key, [])))
        return out
