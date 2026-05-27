"""Make tiling_benchmark and the repo root importable from this package.

tiling_benchmark/ holds loose scripts (HefHandle in probe_phantom_hef.py) that
are not a package; the drone_follow package lives at the repo root."""
from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
_TILING = _REPO_ROOT / "tiling_benchmark"

for p in (str(_REPO_ROOT), str(_TILING)):
    if p not in sys.path:
        sys.path.insert(0, p)
