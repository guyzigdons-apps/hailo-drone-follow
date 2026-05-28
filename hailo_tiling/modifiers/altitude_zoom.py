"""AltitudeZoomModifier — gates ROI tile zoom from drone altitude telemetry.

Reads `telemetry.altitude_agl_m` and linearly interpolates an effective
`max_zoom` between `zoom_at_low_agl` (when at or below `low_agl_m`) and
`zoom_at_high_agl` (when at or above `high_agl_m`). The first ROI tile in
the working list — defined as "the first tile with mode == 's' when
lock.status == 'TRACKING'" — is rebuilt with the new zoom factor.

Falls back to `fallback_max_zoom` (or leaves the tile untouched if that is
None) when `telemetry.altitude_agl_m is None`.

Reference: arXiv:2511.19728 (altitude-aware dynamic tiling).
"""
from __future__ import annotations

from typing import Optional

from ..telemetry import NULL_SNAPSHOT, TelemetrySnapshot
from ..types import CropRect, LockState, MODEL_W


def _interp_zoom(agl_m: float, low_agl: float, high_agl: float,
                 z_low: float, z_high: float) -> float:
    """Linear interp of zoom factor between low_agl and high_agl. Clamped."""
    if high_agl <= low_agl:
        return z_low
    if agl_m <= low_agl:
        return z_low
    if agl_m >= high_agl:
        return z_high
    f = (agl_m - low_agl) / (high_agl - low_agl)
    return z_low + f * (z_high - z_low)


class AltitudeZoomModifier:
    """Gate the ROI tile's effective max-zoom by drone AGL altitude."""

    name = "altitude_zoom"

    def __init__(
        self,
        zoom_at_low_agl: float = 1.0,
        zoom_at_high_agl: float = 2.0,
        low_agl_m: float = 5.0,
        high_agl_m: float = 40.0,
        fallback_max_zoom: Optional[float] = None,
    ):
        self.zoom_at_low_agl = zoom_at_low_agl
        self.zoom_at_high_agl = zoom_at_high_agl
        self.low_agl_m = low_agl_m
        self.high_agl_m = high_agl_m
        self.fallback_max_zoom = fallback_max_zoom

    def _rescale_roi(self, roi: CropRect, lock: LockState, src_w: int, src_h: int,
                      zoom: float) -> CropRect:
        """Rebuild the ROI tile centered on the locked bbox at the new zoom."""
        bx, by, bw, bh = lock.bbox_norm
        cx = (bx + bw / 2) * src_w
        cy = (by + bh / 2) * src_h
        crop_w = int(round(MODEL_W / max(0.01, zoom)))
        need_w = int(round(bw * src_w * 1.5))  # 1 + 2*0.25 margin
        crop_w = max(crop_w, need_w)
        return CropRect.from_center_width(cx, cy, crop_w, mode="s").clamp(src_w, src_h)

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

        agl = telemetry.altitude_agl_m
        if agl is None:
            if self.fallback_max_zoom is None:
                return tiles
            zoom = self.fallback_max_zoom
        else:
            zoom = _interp_zoom(agl, self.low_agl_m, self.high_agl_m,
                                self.zoom_at_low_agl, self.zoom_at_high_agl)

        out = list(tiles)
        for i, t in enumerate(out):
            if t.mode == "s":
                out[i] = self._rescale_roi(t, lock, src_w, src_h, zoom)
                break
        return out
