"""TileScheduler — composes TileEmitters and TileModifiers.

The scheduler runs each emitter in order, concatenates the resulting CropRects,
then runs each modifier in order, threading the working tile list through. The
last modifier should typically be `BudgetTrimModifier`, which enforces the
per-frame inference budget.

This module deliberately contains no Hailo / GStreamer / OpenCV imports.
"""
from __future__ import annotations

from typing import Protocol, Sequence, runtime_checkable

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
    ) -> list[CropRect]:
        tiles: list[CropRect] = []
        for e in self.emitters:
            tiles.extend(e.emit(src_w, src_h, lock, frame_idx, meter))
        for m in self.modifiers:
            tiles = m.modify(tiles, src_w, src_h, lock, frame_idx, meter)
        return tiles
