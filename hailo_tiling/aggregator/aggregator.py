"""Top-level Aggregator that composes the per-stage filters.

Order:
1. map_to_source — convert crop-local tile-local dets to source-frame normalized.
2. boundary_strip — drop dets touching tile interior edges (if enabled).
3. nms — per-class greedy NMS.
4. memory.observe(final_dets) — update memory state.
5. memory.predict() — inject carry-forward dets (NoOpMemory returns []).

Returns a single flat list of source-frame normalized `Det`s for the tracker
to consume. See spec §4 (Data Flow).
"""
from __future__ import annotations

from typing import Iterable, Optional, Sequence

from ..telemetry import NULL_SNAPSHOT, TelemetrySnapshot
from ..types import CropRect, Det
from .boundary_strip import BoundaryStripFilter
from .memory import DetectionMemory, NoOpMemory
from .nms import map_to_source, nms


class Aggregator:
    """Composes map → boundary-strip → NMS → memory."""

    def __init__(
        self,
        boundary_strip: Optional[BoundaryStripFilter] = None,
        memory: Optional[DetectionMemory] = None,
        iou_thr: float = 0.5,
    ):
        self.boundary_strip = boundary_strip if boundary_strip is not None else BoundaryStripFilter()
        self.memory = memory if memory is not None else NoOpMemory()
        self.iou_thr = iou_thr

    def aggregate(
        self,
        frame_idx: int,
        crops: Sequence[CropRect],
        dets_per_crop: Sequence[Iterable],
        src_w: int,
        src_h: int,
        telemetry: TelemetrySnapshot = NULL_SNAPSHOT,
    ) -> list[Det]:
        if len(crops) != len(dets_per_crop):
            raise ValueError(
                f"crops and dets_per_crop length mismatch: {len(crops)} vs {len(dets_per_crop)}"
            )
        flat: list[Det] = []
        for crop, dets in zip(crops, dets_per_crop):
            flat.extend(map_to_source(dets, crop, src_w, src_h))
        stripped = self.boundary_strip.filter(flat, crops, src_w, src_h)
        kept = nms(stripped, iou_thr=self.iou_thr)
        self.memory.observe(kept, frame_idx)
        injected = self.memory.predict(frame_idx)
        if injected:
            kept = nms(kept + list(injected), iou_thr=self.iou_thr)
        return kept
