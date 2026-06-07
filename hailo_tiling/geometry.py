"""Tile/FOV geometry helpers for the tiling library.

These two helpers were absorbed verbatim from the frozen ``tiling_benchmark``
research package (provenance commit ``7d9a8d9``) so that ``hailo_tiling`` — the
installed, prod-bound library — no longer reaches into a non-packaged research
dir for its grid/FOV math:

- ``grid_to_static_tiles`` ← ``tiling_benchmark/tiling_record.py``
  (``_grid_to_static_tiles``, de-underscored; its private helpers
  ``_suffix_tile`` / ``_VALID_TILE_MODES`` come along too).
- ``fov_to_crop_dims`` ← ``tiling_benchmark/prepare_video.py`` (same name).

The bodies are copied verbatim from ``7d9a8d9``; the parity tests in
``hailo_tiling/tests/test_geometry.py`` assert byte-identical results vs the
frozen originals.
"""
from __future__ import annotations

import math


# --- grid_to_static_tiles (from tiling_benchmark/tiling_record.py @ 7d9a8d9) ---

_VALID_TILE_MODES = ("", "m", "s", "multi-scale", "single-scale", "0", "1")


def _suffix_tile(rect: str, mode: str) -> str:
    """Append ',<mode>' to a 'x,y,w,h' rect string if mode is non-empty.
    `mode` is the cropper's per-tile mode override (see hailotilecropper_dynamic's
    tiles-static property docstring). Empty string ⇒ inherit cropper default."""
    if not mode:
        return rect
    if mode not in _VALID_TILE_MODES:
        raise ValueError(
            f"tile mode must be one of {_VALID_TILE_MODES!r}, got {mode!r}")
    return f"{rect},{mode}"


def grid_to_static_tiles(tiles_x: int, tiles_y: int,
                         overlap_x: float, overlap_y: float,
                         mode: str = "") -> list[str]:
    """Convert a regular tiles_x*tiles_y grid into a list of normalized
    'x,y,w,h[,mode]' rectangles. Math: T = 1/(N - (N-1)*o); step = T*(1-o).
    For N==1 returns a single full-axis tile.

    `mode` (optional, default "") is a per-tile mode override for
    hailotilecropper_dynamic's tiles-static parser. Accepts ``"m"`` /
    ``"multi-scale"`` / ``"1"`` (boundary-strip ON) or ``"s"`` /
    ``"single-scale"`` / ``"0"`` (boundary-strip OFF). Empty ⇒ inherit the
    cropper-level ``tiling-mode`` default.
    """
    if tiles_x < 1 or tiles_y < 1:
        return []
    def axis(n: int, o: float):
        if n == 1:
            return [(0.0, 1.0)]
        T = 1.0 / (n - (n - 1) * o)
        S = T * (1.0 - o)
        return [(i * S, T) for i in range(n)]
    rects = []
    for (y, h) in axis(tiles_y, overlap_y):
        for (x, w) in axis(tiles_x, overlap_x):
            rects.append(_suffix_tile(
                f"{x:.6f},{y:.6f},{w:.6f},{h:.6f}", mode))
    return rects


# --- fov_to_crop_dims (from tiling_benchmark/prepare_video.py @ 7d9a8d9) ---

# 6K source dimensions (DJI Mavic 4 Pro main camera; spec §8.1)
SRC_6K_W = 6016
SRC_6K_H = 3384
SRC_NATIVE_FOV_DEG = 70

# Allowed FOV variants. Anything outside this set would require upscaling
# from the 6K source, which is disallowed (spec §8.2).
ALLOWED_FOVS = (70, 60, 50)


def fov_to_crop_dims(fov_deg: int) -> tuple[int, int]:
    """Return the (crop_w, crop_h) needed to emulate `fov_deg` from a 6K source.

    Derived from `crop_ratio = tan(fov_deg/2) / tan(70°/2)` per spec §8.2.
    `crop_h` is the smallest even integer >= `SRC_6K_H * crop_ratio` (h264/h265
    encoders require even dims), and `crop_w` is derived from `crop_h` so the
    true source aspect ratio is preserved exactly
    (`int(crop_h * SRC_6K_W / SRC_6K_H)`, i.e. 6016/3384 — not nominal 16:9).

    The published spec §8.2 table values fall out of this construction:
        FOV-70: (6016, 3384)
        FOV-60: (4963, 2792)
        FOV-50: (4007, 2254)

    Verifies the result is <= source dims and >= 4K output dims (3840x2160);
    raises ValueError on either violation (we never upscale).
    """
    if fov_deg not in ALLOWED_FOVS:
        raise ValueError(
            f"fov_deg must be one of {ALLOWED_FOVS}; got {fov_deg}"
        )
    ratio = (
        math.tan(math.radians(fov_deg) / 2.0)
        / math.tan(math.radians(SRC_NATIVE_FOV_DEG) / 2.0)
    )
    # Round crop_h up to the nearest even integer (encoder-friendly), then
    # derive crop_w from crop_h to preserve the 16:9 source aspect ratio.
    crop_h = math.ceil(SRC_6K_H * ratio / 2.0) * 2
    crop_w = int(crop_h * SRC_6K_W / SRC_6K_H)
    if crop_w > SRC_6K_W or crop_h > SRC_6K_H:
        raise ValueError(
            f"fov_deg={fov_deg} requires crop {crop_w}x{crop_h} > source "
            f"{SRC_6K_W}x{SRC_6K_H}; refusing to upscale"
        )
    if crop_w < 3840 or crop_h < 2160:
        raise ValueError(
            f"fov_deg={fov_deg} requires crop {crop_w}x{crop_h} < 4K output "
            f"3840x2160; refusing to upscale"
        )
    return crop_w, crop_h
