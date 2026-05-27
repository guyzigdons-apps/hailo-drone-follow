from __future__ import annotations

from .types import CropRect, LockState, MODEL_W, MODEL_H, MODEL_ASPECT


class TileScheduler:
    def __init__(self, src_w: int, src_h: int, *,
                 discovery_period: int = 15, discovery_grid: tuple = (3, 2),
                 recovery_grid: tuple = (3, 3), max_zoom: float = 2.0,
                 target_model_h: float = 40.0, roi_margin_frac: float = 0.25):
        self.src_w = src_w
        self.src_h = src_h
        self.discovery_period = discovery_period
        self.discovery_grid = discovery_grid
        self.recovery_grid = recovery_grid
        self.max_zoom = max_zoom
        self.target_model_h = target_model_h
        self.roi_margin_frac = roi_margin_frac

    def _grid(self, gx: int, gy: int, x0: float, y0: float, w: float, h: float,
              mode: str) -> list[CropRect]:
        """gx*gy grid of CropRects covering the [x0,y0,w,h] src-px region."""
        out = []
        cw = w / gx
        ch = h / gy
        for j in range(gy):
            for i in range(gx):
                cx = x0 + (i + 0.5) * cw
                cy = y0 + (j + 0.5) * ch
                r = CropRect.from_center_width(cx, cy, int(round(cw)), mode=mode)
                out.append(r.clamp(self.src_w, self.src_h))
        return out

    def _roi(self, lock: LockState) -> CropRect:
        bx, by, bw, bh = lock.bbox_norm
        if lock.status != "TRACKING":
            bx += lock.last_velocity[0] * lock.frames_since_seen
            by += lock.last_velocity[1] * lock.frames_since_seen
        cx = (bx + bw / 2) * self.src_w
        cy = (by + bh / 2) * self.src_h
        src_h_px = max(1.0, bh * self.src_h)
        crop_w = src_h_px * MODEL_H * MODEL_ASPECT / self.target_model_h
        lo = MODEL_W / self.max_zoom
        crop_w = max(lo, min(float(MODEL_W) * 4, crop_w))
        crop_w = min(crop_w, float(MODEL_W))
        need_w = bw * self.src_w * (1 + 2 * self.roi_margin_frac)
        crop_w = max(crop_w, need_w)
        return CropRect.from_center_width(cx, cy, int(round(crop_w))).clamp(
            self.src_w, self.src_h)

    def decide(self, lock: LockState, frame_idx: int, meter) -> list[CropRect]:
        crops: list[CropRect] = []
        on_cadence = (frame_idx % self.discovery_period == 0)

        if lock.status in ("SEARCHING", "LOST") and lock.track_id is not None:
            gx, gy = self.recovery_grid
            bx, by, bw, bh = lock.bbox_norm
            span = 0.4
            x0 = max(0.0, (bx + bw / 2 - span / 2)) * self.src_w
            y0 = max(0.0, (by + bh / 2 - span / 2)) * self.src_h
            crops = self._grid(gx, gy, x0, y0, span * self.src_w, span * self.src_h, "s")
        else:
            if on_cadence:
                gx, gy = self.discovery_grid
                crops += self._grid(gx, gy, 0, 0, self.src_w, self.src_h, "m")
            if lock.status == "TRACKING":
                crops.append(self._roi(lock))

        budget = int(meter.available(frame_idx))
        if budget >= 0 and len(crops) > budget:
            crops = crops[:max(0, budget)]
        return crops
