from __future__ import annotations

from ..types import CropRect, LockState, TargetState, ScheduledTile, MODEL_W, MODEL_H, MODEL_ASPECT

__all__ = ["TileScheduler", "MultiTargetTileScheduler"]


class TileScheduler:
    def __init__(self, src_w: int, src_h: int, *,
                 discovery_period: int = 15, discovery_grid: tuple = (3, 2),
                 recovery_grid: tuple = (3, 3), max_zoom: float = 2.0,
                 target_model_h: float = 40.0, roi_margin_frac: float = 0.25,
                 recovery_span: float = 0.4, grid_overlap: float = 0.0):
        self.grid_overlap = grid_overlap
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
        overlap) rather than leaving vertical/horizontal gaps.

        `grid_overlap` is the fraction of a cell shared with each neighbour
        (static-ablation convention): cells grow to cell = span/(g - (g-1)*o)
        and the stride shrinks to cell*(1-o), so boundary objects appear whole
        in at least one tile while coverage of [x0,y0,w,h] is preserved."""
        out = []
        o = self.grid_overlap
        cw = w / (gx - (gx - 1) * o)
        ch = h / (gy - (gy - 1) * o)
        sx = cw * (1 - o)
        sy = ch * (1 - o)
        crop_w = max(cw, ch * MODEL_ASPECT)
        for j in range(gy):
            for i in range(gx):
                cx = x0 + cw / 2 + i * sx
                cy = y0 + ch / 2 + j * sy
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

    def _roi_proportional(self, lock: LockState) -> CropRect:
        """Dynamic ROI sized to the target bbox + margin on BOTH axes (4:3), so
        the window always sits a bit larger than the bbox and tracks its size:
        it GROWS past native (640) for a large/near target (downscales to keep
        it whole) and tightens (zooms in, capped at ``max_zoom``) for a small/
        far one. Unlike ``_roi`` it does not pin a small target to native 640 —
        the tile hugs the bbox. (Used by the live showcase; ``_roi`` is kept for
        the benchmark/parity-tested paths.)"""
        bx, by, bw, bh = lock.bbox_norm
        cx = (bx + bw / 2) * self.src_w
        cy = (by + bh / 2) * self.src_h
        m = 1.0 + 2.0 * self.roi_margin_frac
        need_w = bw * self.src_w * m
        need_h = bh * self.src_h * m
        crop_w = max(need_w, need_h * MODEL_ASPECT)   # contain both axes in 4:3
        crop_w = max(crop_w, MODEL_W / self.max_zoom)  # cap zoom-in (upscale blur)
        crop_w = min(crop_w, float(self.src_w))        # never exceed the frame
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


class MultiTargetTileScheduler:
    """Emits one ROI per TRACKING target + recovery grid for the selected
    target + discovery on cadence.  Phase 2: size-aware ROI merge, aging-counter
    budget allocation, selected-target always-served.
    """

    def __init__(self, src_w: int, src_h: int, *,
                 discovery_period: int = 15, discovery_grid: tuple = (3, 2),
                 recovery_grid: tuple = (3, 3), max_zoom: float = 2.0,
                 target_model_h: float = 40.0, roi_margin_frac: float = 0.25,
                 recovery_span: float = 0.4,
                 merge_iou_threshold: float = 0.5,
                 merge_pad_frac: float = 0.25,
                 merge_union_inflate_max: float = 1.5):
        self.src_w = src_w
        self.src_h = src_h
        self.discovery_period = discovery_period
        self.discovery_grid = discovery_grid
        self.recovery_grid = recovery_grid
        self.max_zoom = max_zoom
        self.target_model_h = target_model_h
        self.roi_margin_frac = roi_margin_frac
        self.recovery_span = recovery_span
        self.merge_iou_threshold = merge_iou_threshold
        self.merge_pad_frac = merge_pad_frac
        self.merge_union_inflate_max = merge_union_inflate_max
        # Delegate grid/ROI helpers to a v1 TileScheduler instance.
        self._v1 = TileScheduler(src_w, src_h,
                                 discovery_period=discovery_period,
                                 discovery_grid=discovery_grid,
                                 recovery_grid=recovery_grid,
                                 max_zoom=max_zoom,
                                 target_model_h=target_model_h,
                                 roi_margin_frac=roi_margin_frac,
                                 recovery_span=recovery_span)
        # Phase 2 state.
        self._selected_key: tuple | None = None
        self._aging: dict = {}    # key -> frames deferred since last served
        self._areas: dict = {}    # key -> bbox area (bw*bh) for tiebreak

    # ------------------------------------------------------------------
    # External API (called by run_multi each frame)
    # ------------------------------------------------------------------

    def set_selected(self, key: tuple | None) -> None:
        """Set the user-selected target key; persists across frames."""
        self._selected_key = key

    def clear_aging(self) -> None:
        """Reset aging counters (for tests)."""
        self._aging.clear()
        self._areas.clear()

    # ------------------------------------------------------------------
    # Merge helpers
    # ------------------------------------------------------------------

    def _padded_det_iou(self, a_bbox: tuple, b_bbox: tuple, pad: float) -> float:
        """IoU of two detection bboxes (x,y,w,h normalized) after expanding
        both by `pad` fraction of their own size on each side."""
        def expand(b, p):
            x, y, w, h = b
            px = w * p; py = h * p
            return (x - px, y - py, w + 2 * px, h + 2 * py)
        pa = expand(a_bbox, pad)
        pb = expand(b_bbox, pad)
        ax1, ay1, ax2, ay2 = pa[0], pa[1], pa[0] + pa[2], pa[1] + pa[3]
        bx1, by1, bx2, by2 = pb[0], pb[1], pb[0] + pb[2], pb[1] + pb[3]
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
        inter = iw * ih
        if inter <= 0:
            return 0.0
        ua = pa[2] * pa[3] + pb[2] * pb[3] - inter
        return inter / ua if ua > 0 else 0.0

    def _union_crop_w(self, a_bbox: tuple, b_bbox: tuple) -> int:
        """Smallest crop_w (src px) needed to contain both detection bboxes
        with roi_margin_frac margin, respecting 4:3 model aspect."""
        ax1, ay1, aw, ah = a_bbox
        bx1, by1, bw, bh = b_bbox
        ux1 = min(ax1, bx1); uy1 = min(ay1, by1)
        ux2 = max(ax1 + aw, bx1 + bw); uy2 = max(ay1 + ah, by1 + bh)
        uw_src = (ux2 - ux1) * self.src_w
        uh_src = (uy2 - uy1) * self.src_h
        # crop_w must contain the wider axis (4:3 model aspect).
        need_w = max(uw_src, uh_src * MODEL_ASPECT) * (1 + 2 * self.roi_margin_frac)
        return int(round(need_w))

    def _merged_crop(self, a_bbox: tuple, b_bbox: tuple) -> CropRect:
        """Build the merged CropRect centred on the union midpoint, sized to
        contain both bboxes, clamped to frame bounds."""
        ax1, ay1, aw, ah = a_bbox
        bx1, by1, bw, bh = b_bbox
        ux1 = min(ax1, bx1); uy1 = min(ay1, by1)
        ux2 = max(ax1 + aw, bx1 + bw); uy2 = max(ay1 + ah, by1 + bh)
        cx = ((ux1 + ux2) / 2) * self.src_w
        cy = ((uy1 + uy2) / 2) * self.src_h
        crop_w = self._union_crop_w(a_bbox, b_bbox)
        lo = MODEL_W / self.max_zoom
        crop_w = max(int(round(lo)), crop_w)
        crop_w = min(crop_w, MODEL_W)
        return CropRect.from_center_width(cx, cy, crop_w, mode="s").clamp(
            self.src_w, self.src_h)

    # ------------------------------------------------------------------
    # Internal ROI builder (same as v1)
    # ------------------------------------------------------------------

    def _roi_for_target(self, s: TargetState) -> CropRect:
        lock = LockState(track_id=s.key[1],
                         bbox_norm=s.bbox_norm,
                         status=s.status,
                         frames_since_seen=s.frames_since_seen,
                         last_velocity=s.last_velocity)
        return self._v1._roi(lock)

    def _crop_w_for_target(self, s: TargetState) -> int:
        """Return the crop_w (src px) the v1 ROI rule would produce."""
        return self._roi_for_target(s).w

    # ------------------------------------------------------------------
    # Main decision
    # ------------------------------------------------------------------

    def decide(self, targets: list, frame_idx: int, meter) -> list:
        """Return list[ScheduledTile].  Also updates _aging/_areas state."""
        on_cadence = (frame_idx % self.discovery_period == 0)

        # Update per-key area cache from live targets.
        for s in targets:
            bw, bh = s.bbox_norm[2], s.bbox_norm[3]
            self._areas[s.key] = bw * bh

        # --- recovery branch: SELECTED target is SEARCHING / LOST ---
        selected_state = next((s for s in targets if s.selected), None)
        if (selected_state is not None
                and selected_state.status in ("SEARCHING", "LOST")
                and selected_state.bbox_norm[2] > 0):
            gx, gy = self.recovery_grid
            bx, by, bw, bh = selected_state.bbox_norm
            ecx = (bx + bw / 2
                   + selected_state.last_velocity[0] * selected_state.frames_since_seen)
            ecy = (by + bh / 2
                   + selected_state.last_velocity[1] * selected_state.frames_since_seen)
            span = self.recovery_span
            half = span / 2
            x0_n = max(0.0, min(1.0 - span, ecx - half))
            y0_n = max(0.0, min(1.0 - span, ecy - half))
            x0 = x0_n * self.src_w
            y0 = y0_n * self.src_h
            crops = self._v1._grid(gx, gy, x0, y0,
                                   span * self.src_w, span * self.src_h, "s")
            budget = int(meter.available(frame_idx))
            tiles = [ScheduledTile(crop=c, category="single-scale",
                                   target_keys=()) for c in crops]
            if budget >= 0 and len(tiles) > budget:
                tiles = tiles[:max(0, budget)]
            return tiles

        # --- normal path: build per-target ROI candidates ---
        tracking_targets = [s for s in targets if s.status == "TRACKING"]

        # Build one ScheduledTile per TRACKING target (pre-merge).
        per_target: list[tuple] = []  # (ScheduledTile, TargetState)
        for s in tracking_targets:
            crop = self._roi_for_target(s)
            tile = ScheduledTile(crop=crop, category="dynamic",
                                 target_keys=(s.key,))
            per_target.append((tile, s))

        # --- merge pass: greedy pairwise, one pass ---
        merged_tiles: list[ScheduledTile] = []
        merged_indices: set = set()
        for i, (tile_a, state_a) in enumerate(per_target):
            if i in merged_indices:
                continue
            merged_with = None
            for j, (tile_b, state_b) in enumerate(per_target):
                if j <= i or j in merged_indices:
                    continue
                # Condition A: padded det IoU >= threshold.
                iou = self._padded_det_iou(state_a.bbox_norm, state_b.bbox_norm,
                                           self.merge_pad_frac)
                if iou < self.merge_iou_threshold:
                    continue
                # Condition B: union crop_w <= inflate_max * max(individual crop_ws).
                union_w = self._union_crop_w(state_a.bbox_norm, state_b.bbox_norm)
                max_w = max(tile_a.crop.w, tile_b.crop.w)
                if union_w > self.merge_union_inflate_max * max_w:
                    continue
                # Merge accepted.
                merged_crop = self._merged_crop(state_a.bbox_norm, state_b.bbox_norm)
                merged_tile = ScheduledTile(
                    crop=merged_crop,
                    category="dynamic-merged",
                    target_keys=(state_a.key, state_b.key),
                )
                merged_tiles.append(merged_tile)
                merged_indices.add(i)
                merged_indices.add(j)
                merged_with = j
                break  # one merge partner per tile
            if merged_with is None and i not in merged_indices:
                merged_tiles.append(tile_a)

        # Discovery grid on cadence — tagged multi-scale, no target keys.
        discovery_tiles: list[ScheduledTile] = []
        if on_cadence:
            gx, gy = self.discovery_grid
            disc_crops = self._v1._grid(gx, gy, 0, 0,
                                        self.src_w, self.src_h, "m")
            discovery_tiles = [ScheduledTile(crop=c, category="multi-scale",
                                             target_keys=()) for c in disc_crops]

        all_tiles = merged_tiles + discovery_tiles

        # --- aging-counter budget allocation ---
        budget = int(meter.available(frame_idx))

        # Always keep tiles that serve the selected key.
        sel_key = self._selected_key
        if sel_key is not None:
            mandatory = [t for t in all_tiles if sel_key in t.target_keys]
            others = [t for t in all_tiles if sel_key not in t.target_keys]
        else:
            mandatory = []
            others = list(all_tiles)

        # Sort others: dynamic/dynamic-merged by aging desc then area desc first;
        # discovery/recovery tiles come last (lowest priority).
        def sort_key(t: ScheduledTile):
            if t.target_keys and t.category in ("dynamic", "dynamic-merged"):
                ages = [self._aging.get(k, 0) for k in t.target_keys]
                areas = [self._areas.get(k, 0.0) for k in t.target_keys]
                return (0, -max(ages), -max(areas))
            return (1, 0, 0.0)

        others.sort(key=sort_key)

        kept = (mandatory + others)[:max(0, budget)] if budget >= 0 else (mandatory + others)

        # Update aging counters.
        served_keys: set = set()
        for t in kept:
            for k in t.target_keys:
                served_keys.add(k)
        deferred_keys: set = set()
        for t in all_tiles:
            for k in t.target_keys:
                if k not in served_keys:
                    deferred_keys.add(k)
        for k in served_keys:
            self._aging[k] = 0
        for k in deferred_keys:
            self._aging[k] = self._aging.get(k, 0) + 1

        return kept
