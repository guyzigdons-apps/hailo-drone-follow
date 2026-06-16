from __future__ import annotations

from ..types import CropRect

__all__ = ["pyramid_crops", "DEFAULT_PYRAMID_WIDTHS"]

# Native 640 down to ~2.9x digital zoom.
DEFAULT_PYRAMID_WIDTHS = (640, 448, 320, 224)


def pyramid_crops(cx_norm, cy_norm, src_w, src_h, widths=DEFAULT_PYRAMID_WIDTHS):
    """A pyramid of single-scale crops centred on a normalized click point, at
    decreasing source-pixel widths (native -> increasing zoom). Lets the detector
    find an object at the click at whatever scale works — even a tiny far target
    a native crop misses. Widths larger than the source are skipped."""
    cx = cx_norm * src_w
    cy = cy_norm * src_h
    out = []
    for w in widths:
        if w > src_w:
            continue
        out.append(CropRect.from_center_width(cx, cy, int(w), mode="s").clamp(src_w, src_h))
    return out
