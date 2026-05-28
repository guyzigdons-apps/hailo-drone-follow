"""HefBackend — direct HailoRT inference, lifted from dynamic_tiling/inference.py.

Widens the per-crop API to a batched `(frame, crops, frame_idx) -> list[list[Det]]`
shape required by the hailo_tiling InferenceBackend ABC (see backends/backend.py).
On-chip, the implementation loops over crops internally so the HailoRT call
sequence is unchanged from the legacy dynamic_tiling.inference.HefBackend.
"""
from __future__ import annotations

from typing import Sequence

import numpy as np

from ..types import CropRect, Det, MODEL_H, MODEL_W
from .backend import InferenceBackend


class HefBackend(InferenceBackend):
    """Real on-chip backend wrapping tiling_benchmark HefHandle.

    Two construction modes:
    1. Production: ``HefBackend(hef_path="...", nms_score_threshold=0.25)``
       lazy-imports the HailoRT-touching ``probe_phantom_hef`` module.
    2. Test injection: ``HefBackend(handle, decode)`` — for unit tests that
       want to assert on the batched infer loop without touching HailoRT.
    """

    def __init__(self, *args, **kwargs):
        # Test-injection mode: (handle, decode) positional, no kwargs.
        if len(args) == 2 and not kwargs:
            handle, decode = args
            self._handle = handle
            self._decode = decode
            return
        hef_path = kwargs.get("hef_path") or (args[0] if args else None)
        nms_score_threshold = kwargs.get("nms_score_threshold", 0.25)
        if hef_path is None:
            raise TypeError("HefBackend requires hef_path or (handle, decode)")
        # Same lazy-import path as the legacy dynamic_tiling/inference.py
        # (via _vendor_paths in the tiling-benchmark layout).
        from probe_phantom_hef import HefHandle, decode_nms_output  # noqa: WPS433
        self._handle = HefHandle.open(hef_path, nms_score_threshold=nms_score_threshold)
        self._decode = decode_nms_output

    def _infer_one(self, frame: np.ndarray, crop: CropRect) -> list:
        import cv2  # noqa: WPS433 — lazy so headless tests don't import cv2
        sub = frame[crop.y:crop.y + crop.h, crop.x:crop.x + crop.w]
        resized = cv2.resize(sub, (MODEL_W, MODEL_H), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        return self._decode(self._handle.infer(rgb))

    def infer(self, frame, crops: Sequence[CropRect], frame_idx: int) -> list[list[Det]]:
        return [self._infer_one(frame, c) for c in crops]

    def close(self) -> None:
        self._handle.close()
