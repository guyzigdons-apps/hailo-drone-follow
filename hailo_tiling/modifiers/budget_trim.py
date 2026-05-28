"""Final-stage modifier that enforces the per-frame inference budget."""
from __future__ import annotations

from ..types import CropRect, LockState


class BudgetTrimModifier:
    """Truncate the tile list to fit `meter.available(frame_idx)`.

    Mirrors the budget-enforcement tail in `dynamic_tiling.TileScheduler.decide`:
    a negative `available()` is treated as 'unlimited'; otherwise the list is
    truncated to the floor of the available per-frame share. Tiles are kept
    from the head — emitter order is the priority order, so the upstream
    composition (ROI before discovery) decides what survives a tight budget.
    """

    name = "budget_trim"

    def modify(
        self,
        tiles: list[CropRect],
        src_w: int,
        src_h: int,
        lock: LockState,
        frame_idx: int,
        meter,
        telemetry=None,  # accepted for Protocol compatibility; not used
    ) -> list[CropRect]:
        budget = int(meter.available(frame_idx))
        if budget < 0:
            return tiles
        if len(tiles) <= budget:
            return tiles
        return tiles[: max(0, budget)]
