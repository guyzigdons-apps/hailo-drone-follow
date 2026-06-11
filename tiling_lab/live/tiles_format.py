"""Serialise scheduler CropRects (source pixels) to the normalized
`tiles-static` string consumed by hailotilecropper_dynamic.

Format per rect: 'x,y,w,h,mode' with x,y,w,h in [0,1]; rects are ';'-joined.
Crops are clamped into the unit square (origin >= 0, origin+extent <= 1) so a
scheduler ROI that ran past a frame edge never produces an invalid tile.
"""
from collections.abc import Sequence

from hailo_tiling.types import CropRect


def _clamp_axis(origin: float, extent: float) -> tuple[float, float]:
    if extent <= 0.0:
        return 0.0, 0.0
    if origin < 0.0:
        origin = 0.0
    if extent > 1.0:
        extent = 1.0
    if origin + extent > 1.0:
        origin = 1.0 - extent
    return origin, extent


def crops_to_tiles_static(crops: Sequence[CropRect], src_w: int,
                          src_h: int) -> str:
    """Return the ';'-joined normalized tiles-static string for `crops`."""
    parts: list[str] = []
    for c in crops:
        nx, nw = _clamp_axis(c.x / src_w, c.w / src_w)
        ny, nh = _clamp_axis(c.y / src_h, c.h / src_h)
        if nw <= 0.0 or nh <= 0.0:
            continue
        mode = c.mode or "s"
        parts.append(f"{nx:.6f},{ny:.6f},{nw:.6f},{nh:.6f},{mode}")
    return ";".join(parts)
