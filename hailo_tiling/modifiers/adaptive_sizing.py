"""AdaptiveSliceSizingModifier — ASAHI-style discovery grid resizing.

When the locked target's apparent height (bbox_norm[3]) is small, use a
finer discovery grid (more, smaller tiles) so small targets are found; when
the apparent height is large, use a coarser grid so cycles aren't wasted on
redundant tiles.

Reference: Akyon et al., Remote Sensing 15(5):1249, MDPI 2023 (ASAHI).
docs/research/2026-05-27-industry-tiling-drone-tracking.md §1.2.

Plan 2 implementation uses ONLY the bbox height as the scale signal. A
secondary altitude-based signal (telemetry.altitude_agl_m → expected pixel
height per target class) is a Plan 3+ follow-up.
"""
from __future__ import annotations

from ..emitters.discovery_grid import _grid_full
from ..telemetry import NULL_SNAPSHOT, TelemetrySnapshot
from ..types import CropRect, LockState


class AdaptiveSliceSizingModifier:
    """Reshape the discovery grid based on target apparent size."""

    name = "adaptive_slice_sizing"

    def __init__(
        self,
        target_h_thresholds: tuple[float, float] = (0.05, 0.15),
        small_grid: tuple[int, int] = (6, 4),
        medium_grid: tuple[int, int] = (3, 2),
        large_grid: tuple[int, int] = (2, 1),
        discovery_mode_tag: str = "m",
    ):
        """target_h_thresholds = (T_small, T_large):
            bbox_h < T_small  -> small_grid
            T_small <= bbox_h < T_large -> medium_grid
            bbox_h >= T_large -> large_grid
        """
        self.t_small, self.t_large = target_h_thresholds
        self.small_grid = small_grid
        self.medium_grid = medium_grid
        self.large_grid = large_grid
        self.discovery_mode_tag = discovery_mode_tag

    def _pick_grid(self, lock: LockState) -> tuple[int, int]:
        bh = lock.bbox_norm[3]
        if bh < self.t_small:
            return self.small_grid
        if bh < self.t_large:
            return self.medium_grid
        return self.large_grid

    def modify(
        self,
        tiles: list[CropRect],
        src_w: int,
        src_h: int,
        lock: LockState,
        frame_idx: int,
        meter,
        telemetry: TelemetrySnapshot = NULL_SNAPSHOT,
    ) -> list[CropRect]:
        if lock.status != "TRACKING":
            return tiles

        gx, gy = self._pick_grid(lock)
        non_discovery = [t for t in tiles if t.mode != self.discovery_mode_tag]
        any_discovery = any(t.mode == self.discovery_mode_tag for t in tiles)
        if not any_discovery:
            return tiles
        new_disc = _grid_full(src_w, src_h, gx, gy, self.discovery_mode_tag)
        return non_discovery + new_disc
