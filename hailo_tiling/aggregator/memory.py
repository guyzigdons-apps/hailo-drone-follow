"""DetectionMemory — interface for carry-forward of unscheduled-tile detections.

Plan 2 ships only the ABC + a NoOpMemory default. The full v2 implementation
(CarryForwardMemory) tracks (det → tile) mappings and resurfaces "missed"
detections on frames where the corresponding tile was not scheduled. See
spec §3.1 and `docs/research/2026-05-27-industry-tiling-drone-tracking.md`
§1.7 (Selective Tile Processing with Memory).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Iterable

from ..types import Det


class DetectionMemory(ABC):
    """Maintains a memory of recent detections; predicts forward when a tile is skipped."""

    @abstractmethod
    def observe(self, dets: Iterable[Det], frame_idx: int) -> None:
        """Update memory state with this frame's confirmed detections."""

    @abstractmethod
    def predict(self, frame_idx: int) -> list[Det]:
        """Return memory-carried detections to inject into this frame's results.

        For NoOpMemory this is always `[]`. CarryForwardMemory (v2) returns
        the last-known detections whose tiles were NOT scheduled this frame.
        """

    def reset(self) -> None:  # pragma: no cover - default no-op
        """Optional: clear all stored state."""


class NoOpMemory(DetectionMemory):
    """Default memory implementation — never carries detections forward."""

    name = "noop_memory"

    def observe(self, dets, frame_idx: int) -> None:
        return None

    def predict(self, frame_idx: int) -> list[Det]:
        return []

    def reset(self) -> None:
        return None
