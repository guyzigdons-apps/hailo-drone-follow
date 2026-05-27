from __future__ import annotations

from typing import Protocol

import cv2
import numpy as np

from .types import CropRect, MODEL_W, MODEL_H


class InferenceBackend(Protocol):
    def infer(self, frame: np.ndarray, crop: CropRect, frame_idx: int) -> list:
        """Return crop-local normalized detections (.cls .x .y .w .h .score)."""
        ...


class ReplayBackend:
    """Deterministic backend: returns canned crop-local dets for tests.

    `canned` maps (frame_idx, (x,y,w,h)) -> list of objects exposing
    .cls .x .y .w .h .score (crop-local normalized)."""

    def __init__(self, canned: dict):
        self._canned = canned

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:
        return list(self._canned.get((frame_idx, (crop.x, crop.y, crop.w, crop.h)), []))


class HefBackend:
    """Real on-chip backend wrapping tiling_benchmark HefHandle."""

    def __init__(self, hef_path: str, nms_score_threshold: float = 0.25):
        from probe_phantom_hef import HefHandle  # via _vendor_paths
        self._HefHandle = HefHandle
        self._handle = HefHandle.open(hef_path, nms_score_threshold=nms_score_threshold)

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:
        from probe_phantom_hef import decode_nms_output
        sub = frame[crop.y:crop.y + crop.h, crop.x:crop.x + crop.w]
        resized = cv2.resize(sub, (MODEL_W, MODEL_H), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        return decode_nms_output(self._handle.infer(rgb))

    def close(self) -> None:
        self._handle.close()
