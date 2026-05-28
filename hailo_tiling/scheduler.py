"""TileScheduler — composes TileEmitters and TileModifiers.

Plan 2 widens the protocols by one keyword argument: `telemetry: TelemetrySnapshot`.
Existing emitters/modifiers from Plan 1 accept-and-ignore the new arg; new
modifiers (AltitudeZoom, AdaptiveSliceSizing) read it.

This module deliberately contains no Hailo / GStreamer / OpenCV imports.
"""
from __future__ import annotations

from typing import Protocol, Sequence, runtime_checkable

from .telemetry import NULL_SNAPSHOT, TelemetrySnapshot
from .types import CropRect, LockState


@runtime_checkable
class TileEmitter(Protocol):
    """Produces a list of CropRects for the current frame."""

    name: str

    def emit(
        self,
        src_w: int,
        src_h: int,
        lock: LockState,
        frame_idx: int,
        meter,
        telemetry: TelemetrySnapshot = NULL_SNAPSHOT,
    ) -> list[CropRect]: ...


@runtime_checkable
class TileModifier(Protocol):
    """Mutates the working tile list before submission."""

    name: str

    def modify(
        self,
        tiles: list[CropRect],
        src_w: int,
        src_h: int,
        lock: LockState,
        frame_idx: int,
        meter,
        telemetry: TelemetrySnapshot = NULL_SNAPSHOT,
    ) -> list[CropRect]: ...


class TileScheduler:
    """Composable scheduler. Pure orchestration; emitters/modifiers do the work."""

    def __init__(
        self,
        emitters: Sequence[TileEmitter],
        modifiers: Sequence[TileModifier],
    ):
        self.emitters = list(emitters)
        self.modifiers = list(modifiers)

    def decide(
        self,
        src_w: int,
        src_h: int,
        lock: LockState,
        frame_idx: int,
        meter,
        telemetry: TelemetrySnapshot | None = None,
    ) -> list[CropRect]:
        tel = NULL_SNAPSHOT if telemetry is None else telemetry
        tiles: list[CropRect] = []
        for e in self.emitters:
            tiles.extend(e.emit(src_w, src_h, lock, frame_idx, meter, tel))
        for m in self.modifiers:
            tiles = m.modify(tiles, src_w, src_h, lock, frame_idx, meter, tel)
        return tiles
