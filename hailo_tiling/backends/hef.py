"""HefBackend — direct HailoRT inference, lifted from tiling_lab/harness/inference.py.

Widens the per-crop API to a batched `(frame, crops, frame_idx) -> list[list[Det]]`
shape required by the hailo_tiling InferenceBackend ABC (see backends/backend.py).
On-chip, the implementation loops over crops internally so the HailoRT call
sequence is unchanged from the legacy tiling_lab.harness.inference.HefBackend.
"""
from __future__ import annotations

from typing import Sequence

import numpy as np

from ..types import CropRect, Det, MODEL_H, MODEL_W
from .backend import InferenceBackend

# Duplicated from hailo_apps.python.core.common.defines.SHARED_VDEVICE_GROUP_ID
# (value "SHARED"), the same group id reid_analysis/reid_embedding_extractor.py
# uses. hailo_tiling must not import from reid_analysis / hailo_apps, so the
# string is duplicated here; keep both in sync. Joining this shared VDevice
# group lets the HailoRT scheduler multiplex the detection HEF and the ReID HEF
# onto one physical device instead of each grabbing an exclusive VDevice.
SHARED_VDEVICE_GROUP_ID = "SHARED"


def make_shared_vdevice_params():
    """Build VDevice params that join the shared, scheduler-multiplexed group.

    Sets ROUND_ROBIN scheduling and the shared ``group_id`` so this backend's
    VDevice coexists with the ReID extractor's VDevice on one physical device.
    Imports hailo_platform inside the function to match this module's deferred
    HailoRT-import style (so headless/chip-free tests can import the module).
    """
    from hailo_platform import HailoSchedulingAlgorithm, VDevice  # noqa: WPS433

    params = VDevice.create_params()
    params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
    params.group_id = SHARED_VDEVICE_GROUP_ID
    return params


class HefBackend(InferenceBackend):
    """Real on-chip backend wrapping tiling_benchmark HefHandle.

    Two construction modes:
    1. Production: ``HefBackend(hef_path="...", nms_score_threshold=0.25)``
       lazy-imports the HailoRT-touching ``hef_runtime`` module.
    2. Test injection: ``HefBackend(handle, decode)`` — for unit tests that
       want to assert on the batched infer loop without touching HailoRT.

    Pass ``class_offset=N`` to shift every decoded class id by N (default 0;
    use 1 to map the 0-indexed NMS decode to the person=1 label convention).
    """

    def __init__(self, *args, **kwargs):
        self._class_offset = kwargs.pop("class_offset", 0)
        # Test-injection mode: (handle, decode) positional, no remaining kwargs.
        if len(args) == 2 and not kwargs:
            handle, decode = args
            self._handle = handle
            self._decode = decode
            return
        hef_path = kwargs.get("hef_path") or (args[0] if args else None)
        nms_score_threshold = kwargs.get("nms_score_threshold", 0.25)
        if hef_path is None:
            raise TypeError("HefBackend requires hef_path or (handle, decode)")
        # Lazy import of the HailoRT-touching helper (kept off the import path
        # so the module loads chip-free); imported only when a real on-chip
        # backend is actually constructed.
        from .hef_runtime import HefHandle, decode_nms_output  # noqa: WPS433
        self._handle = HefHandle.open(
            hef_path,
            nms_score_threshold=nms_score_threshold,
            vdevice_params=make_shared_vdevice_params(),
        )
        self._decode = decode_nms_output

    def _infer_one(self, frame: np.ndarray, crop: CropRect) -> list:
        import cv2  # noqa: WPS433 — lazy so headless tests don't import cv2
        sub = frame[crop.y:crop.y + crop.h, crop.x:crop.x + crop.w]
        resized = cv2.resize(sub, (MODEL_W, MODEL_H), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        raw = self._handle.infer(rgb)
        dets = self._decode(raw)
        if self._class_offset:
            dets = [Det(cls=d.cls + self._class_offset, score=d.score,
                        x=d.x, y=d.y, w=d.w, h=d.h) for d in dets]
        return dets

    def infer(self, frame, crops: Sequence[CropRect], frame_idx: int) -> list[list[Det]]:
        return [self._infer_one(frame, c) for c in crops]

    def close(self) -> None:
        self._handle.close()
