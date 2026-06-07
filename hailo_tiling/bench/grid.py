"""Grid-definition shim for the ablation bench (Plan 6 B2/B3).

The canonical normalized-grid -> tiles definition now lives in
``hailo_tiling.geometry.grid_to_static_tiles`` (absorbed from the frozen
``tiling_benchmark/tiling_record.py:_grid_to_static_tiles`` @ 7d9a8d9). It is
the SINGLE grid definition shared by the warmer and the replay-key computation.
This module re-exports it under the name the bench has always imported.
"""
from __future__ import annotations

from hailo_tiling.geometry import grid_to_static_tiles  # noqa: F401

__all__ = ["grid_to_static_tiles"]
