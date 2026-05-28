from __future__ import annotations

from .types import CropRect, LockState, MODEL_W, MODEL_H, MODEL_ASPECT


class TileScheduler:
    def __init__(self, src_w: int, src_h: int, *,
                 discovery_period: int = 15, discovery_grid: tuple = (3, 2),
                 recovery_grid: tuple = (3, 3), max_zoom: float = 2.0,
                 target_model_h: float = 40.0, roi_margin_frac: float = 0.25,
                 recovery_span: float = 0.4):
        self.src_w = src_w
        self.src_h = src_h
        self.discovery_period = discovery_period
        self.discovery_grid = discovery_grid
        self.recovery_grid = recovery_grid
        self.max_zoom = max_zoom
        self.target_model_h = target_model_h
        self.roi_margin_frac = roi_margin_frac
        self.recovery_span = recovery_span

    def _grid(self, gx: int, gy: int, x0: float, y0: float, w: float, h: float,
              mode: str) -> list[CropRect]:
        """gx*gy grid covering the [x0,y0,w,h] src-px region. Each tile is 4:3
        (model aspect); the tile width is grown to the larger of the cell width
        and the aspect-scaled cell height so cells are fully covered (tiles may
        overlap) rather than leaving vertical/horizontal gaps."""
        out = []
        cw = w / gx
        ch = h / gy
        crop_w = max(cw, ch * MODEL_ASPECT)
        for j in range(gy):
            for i in range(gx):
                cx = x0 + (i + 0.5) * cw
                cy = y0 + (j + 0.5) * ch
                r = CropRect.from_center_width(cx, cy, int(round(crop_w)), mode=mode)
                out.append(r.clamp(self.src_w, self.src_h))
        return out

    def _roi(self, lock: LockState) -> CropRect:
        # Only called when TRACKING; placement uses the current locked bbox.
        bx, by, bw, bh = lock.bbox_norm
        cx = (bx + bw / 2) * self.src_w
        cy = (by + bh / 2) * self.src_h
        src_h_px = max(1.0, bh * self.src_h)
        crop_w = src_h_px * MODEL_H * MODEL_ASPECT / self.target_model_h
        lo = MODEL_W / self.max_zoom            # crop_w floor => scale <= max_zoom (cap zoom-in)
        crop_w = max(lo, crop_w)
        crop_w = min(crop_w, float(MODEL_W))    # scale >= 1.0: don't upscale a target that already fits
        need_w = bw * self.src_w * (1 + 2 * self.roi_margin_frac)
        # Always contain the WHOLE target; a target wider than 640 src-px
        # intentionally downscales (scale < 1) to stay whole — it needs no zoom.
        crop_w = max(crop_w, need_w)
        return CropRect.from_center_width(cx, cy, int(round(crop_w))).clamp(
            self.src_w, self.src_h)

    def decide(self, lock: LockState, frame_idx: int, meter) -> list[CropRect]:
        crops: list[CropRect] = []
        on_cadence = (frame_idx % self.discovery_period == 0)

        if lock.status in ("SEARCHING", "LOST") and lock.track_id is not None:
            gx, gy = self.recovery_grid
            bx, by, bw, bh = lock.bbox_norm
            # Lever 4: extrapolate the last-known centre by predicted motion during the gap.
            ecx = bx + bw / 2 + lock.last_velocity[0] * lock.frames_since_seen
            ecy = by + bh / 2 + lock.last_velocity[1] * lock.frames_since_seen
            span = self.recovery_span
            half = span / 2
            x0_n = max(0.0, min(1.0 - span, ecx - half))
            y0_n = max(0.0, min(1.0 - span, ecy - half))
            x0 = x0_n * self.src_w
            y0 = y0_n * self.src_h
            crops = self._grid(gx, gy, x0, y0, span * self.src_w, span * self.src_h, "s")
        else:
            # ROI first so a tight budget keeps the locked-target tile and
            # drops discovery tiles from the tail, not the other way around.
            if lock.status == "TRACKING":
                crops.append(self._roi(lock))
            if on_cadence:
                gx, gy = self.discovery_grid
                crops += self._grid(gx, gy, 0, 0, self.src_w, self.src_h, "m")

        budget = int(meter.available(frame_idx))
        if budget >= 0 and len(crops) > budget:
            crops = crops[:max(0, budget)]
        return crops
