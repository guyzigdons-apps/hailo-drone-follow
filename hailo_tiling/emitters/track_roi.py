# hailo_tiling/emitters/track_roi.py
"""Track-guided ROI tile centered on the currently locked bbox."""
from __future__ import annotations

from ..types import CropRect, LockState, MODEL_W, MODEL_H, MODEL_ASPECT


class TrackROIEmitter:
    """Emits a single ROI tile while `lock.status == 'TRACKING'`.

    Crop width is sized so the locked target occupies `target_model_h` pixels
    of vertical extent after the crop is rescaled to model input. Width is
    clamped to [MODEL_W/max_zoom, MODEL_W] (don't upscale; don't zoom past
    max_zoom) but always grown to contain the *whole* target plus margin.

    This is the inlined equivalent of `dynamic_tiling.TileScheduler._roi(lock)`
    and must produce byte-identical output.
    """

    name = "track_roi"

    def __init__(
        self,
        max_zoom: float = 2.0,
        target_model_h: float = 40.0,
        roi_margin_frac: float = 0.25,
    ):
        self.max_zoom = max_zoom
        self.target_model_h = target_model_h
        self.roi_margin_frac = roi_margin_frac

    def emit(self, src_w: int, src_h: int, lock: LockState,
             frame_idx: int, meter) -> list[CropRect]:
        if lock.status != "TRACKING":
            return []
        bx, by, bw, bh = lock.bbox_norm
        cx = (bx + bw / 2) * src_w
        cy = (by + bh / 2) * src_h
        src_h_px = max(1.0, bh * src_h)
        crop_w = src_h_px * MODEL_H * MODEL_ASPECT / self.target_model_h
        lo = MODEL_W / self.max_zoom
        crop_w = max(lo, crop_w)
        crop_w = min(crop_w, float(MODEL_W))
        need_w = bw * src_w * (1 + 2 * self.roi_margin_frac)
        crop_w = max(crop_w, need_w)
        rect = CropRect.from_center_width(cx, cy, int(round(crop_w))).clamp(src_w, src_h)
        return [rect]
