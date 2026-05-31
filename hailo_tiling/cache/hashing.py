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


def tile_norm_to_source_px(
    xmin: float,
    ymin: float,
    width: float,
    height: float,
    src_w: int,
    src_h: int,
    mode: str = "s",
) -> CropRect:
    """Convert a normalized tile (xmin, ymin, width, height in [0,1]) to a
    source-pixel :class:`CropRect`, mirroring the C++ cropper / cache-key rule
    EXACTLY (truncate-then-clamp — the TAPPAS convention):

        x = int(xmin  * src_w)
        y = int(ymin  * src_h)
        w = clamp(int(width  * src_w), 0, src_w - x)
        h = clamp(int(height * src_h), 0, src_h - y)

    This is the Python counterpart of ``cache_keys::tile_crop_to_source_px``
    (and of ``scripts/cache_gst_replay_gate.py:_tile_crop_px``). Plan-5b proved
    the cropper's recorded source-pixel crop equals this computation with 0
    deviations, so feeding the SAME normalized tiles through this helper
    reproduces exactly the keys the GStreamer warmer wrote — the foundation of
    the chip-free replay path (Plan 6 B2).
    """
    if src_w <= 0 or src_h <= 0:
        raise ValueError(f"src_w/src_h must be positive (got {src_w}x{src_h})")
    x = int(xmin * src_w)
    y = int(ymin * src_h)
    w = int(width * src_w)
    h = int(height * src_h)
    w = max(0, min(w, src_w - x))
    h = max(0, min(h, src_h - y))
    return CropRect(x=x, y=y, w=w, h=h, mode=mode)


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
