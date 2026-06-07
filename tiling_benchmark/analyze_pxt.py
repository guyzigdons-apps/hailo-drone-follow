"""Back-compat shim — moved to ``tiling_lab.viewer.analyze_pxt``.

This module was relocated into the ``tiling_lab`` package by the tiling-lab
restructure (2026-06-07). This shim re-exports the new location so the one
remaining frozen-legacy consumer (``tiling_benchmark/clean_frames.py``, which
does a bare same-dir ``from analyze_pxt import ...``) keeps working.
"""
from tiling_lab.viewer.analyze_pxt import *  # noqa: F401,F403

# ``import *`` skips underscore-prefixed names; re-export the private helper the
# frozen ``clean_frames.py`` consumer imports by name.
from tiling_lab.viewer.analyze_pxt import _compute_tile_rects  # noqa: F401
