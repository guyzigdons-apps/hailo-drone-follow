"""Hashing helpers for the tile cache.

`file_sha256(path)` — streams a file in 1 MiB chunks and returns the SHA-256
hex digest. Used for `video_sha` / `hef_sha` in the cache filename and `meta`
table.

`canonicalize_crop(crop_rect, quantise=None)` — returns the 4-tuple cache key
`(x, y, w, h)`. If `quantise` is set, each component is rounded **down** to a
multiple of `quantise` px. Quantisation is spec §7.3's "slightly fuzzy" mode;
the unquantised default (`quantise=None` or `quantise=1`) is paper-correct.

Note: this duplicates `tiling_benchmark/prepare_video.py:sha256_of_file` by
design. The cache layer is a published library surface; we don't want it to
depend on tiling_benchmark internals. See the plan's Open Questions §1.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

from ..types import CropRect

_SHA_CHUNK = 1024 * 1024  # 1 MiB


def file_sha256(path: str | Path) -> str:
    """Return the SHA-256 hex digest of `path`'s bytes (chunked read)."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(_SHA_CHUNK), b""):
            h.update(chunk)
    return h.hexdigest()


def canonicalize_crop(
    crop_rect: CropRect,
    quantise: int | None = None,
) -> tuple[int, int, int, int]:
    """Return the cache key `(x, y, w, h)`.

    If `quantise` is set and > 1, each component is rounded **down** to a
    multiple of `quantise`. This increases cache hit rate when float→int
    rounding in an emitter is unstable across runs at the cost of slightly
    wrong (typically 1–3 px) crop coordinates. The default behaviour
    (`quantise=None` or `quantise=1`) is identity.
    """
    if quantise is None or quantise <= 1:
        return (crop_rect.x, crop_rect.y, crop_rect.w, crop_rect.h)
    q = quantise
    return (
        (crop_rect.x // q) * q,
        (crop_rect.y // q) * q,
        (crop_rect.w // q) * q,
        (crop_rect.h // q) * q,
    )
