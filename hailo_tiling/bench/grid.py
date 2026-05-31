"""Grid-definition shim for the ablation bench (Plan 6 B2/B3).

The canonical normalized-grid -> tiles definition lives in
``tiling_benchmark/tiling_record.py:_grid_to_static_tiles`` and the plan
mandates reusing it as the SINGLE grid definition shared by the warmer and the
replay-key computation.

``tiling_benchmark`` is a top-level research package in the repo that is NOT
pip-installed (only ``hailo_tiling`` / ``drone_follow`` are packaged). When
``hailo-tiling-bench`` runs as an installed console script, the repo root is
not on ``sys.path``, so a bare ``import tiling_benchmark`` fails. This shim
locates the repo root (the directory containing ``tiling_benchmark/``) by
walking up from this file and puts it on ``sys.path`` before importing — so the
bench works both under pytest (rootdir injected) and as an installed script.
"""
from __future__ import annotations

import sys
from pathlib import Path


def _ensure_tiling_benchmark_importable() -> None:
    try:
        import tiling_benchmark  # noqa: F401
        return
    except ModuleNotFoundError:
        pass
    # hailo_tiling/bench/grid.py -> repo root is two parents up from hailo_tiling.
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "tiling_benchmark" / "tiling_record.py").exists():
            if str(parent) not in sys.path:
                sys.path.insert(0, str(parent))
            return
    # Fall through: the import below will raise a clear ModuleNotFoundError.


_ensure_tiling_benchmark_importable()

from tiling_benchmark.tiling_record import _grid_to_static_tiles  # noqa: E402


def grid_to_static_tiles(
    tiles_x: int, tiles_y: int, overlap_x: float, overlap_y: float, mode: str = ""
) -> list[str]:
    """Thin re-export of the canonical ``_grid_to_static_tiles`` so the rest of
    the bench imports one stable name."""
    return _grid_to_static_tiles(tiles_x, tiles_y, overlap_x, overlap_y, mode)
