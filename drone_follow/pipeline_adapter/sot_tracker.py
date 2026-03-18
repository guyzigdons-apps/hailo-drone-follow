"""Single-Object Tracker (SOT) using OpenCV NanoTrack.

Wraps cv2.TrackerNano to provide frame-by-frame tracking of a selected
person.  The tracker is initialized with a bounding box crop and updated
every frame with the full image.

NanoTrack is a lightweight siamese tracker (~1 MB) that runs at 25-40 FPS
on a Raspberry Pi 5 CPU — no NPU needed.
"""

import logging
import os
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

LOGGER = logging.getLogger("drone_follow.sot_tracker")

# Default model paths (relative to this file)
_MODELS_DIR = os.path.join(os.path.dirname(__file__), "models")
_DEFAULT_BACKBONE = os.path.join(_MODELS_DIR, "nanotrack_backbone_sim.onnx")
_DEFAULT_NECKHEAD = os.path.join(_MODELS_DIR, "nanotrack_head_sim.onnx")

# Confidence threshold below which we consider the target lost
DEFAULT_CONFIDENCE_THRESHOLD = 0.5


@dataclass
class SOTResult:
    """Result of a single SOT tracker update."""
    ok: bool                    # True if target is being tracked
    bbox: Optional[Tuple[int, int, int, int]] = None  # (x, y, w, h) in pixels
    confidence: float = 0.0     # NanoTrack tracking score
    center_norm: Optional[Tuple[float, float]] = None  # (cx, cy) normalized 0-1
    bbox_height_norm: float = 0.0  # bbox height normalized 0-1


class SOTracker:
    """Single-object tracker using OpenCV NanoTrack.

    Usage::

        sot = SOTracker()
        sot.init(frame, bbox_xywh)      # Initialize on a target crop
        result = sot.update(frame)       # Update every frame
        sot.reset()                      # Clear when switching to MOT
    """

    def __init__(self, backbone_path: str = None, neckhead_path: str = None,
                 confidence_threshold: float = DEFAULT_CONFIDENCE_THRESHOLD):
        self._backbone_path = backbone_path or _DEFAULT_BACKBONE
        self._neckhead_path = neckhead_path or _DEFAULT_NECKHEAD
        self._confidence_threshold = confidence_threshold
        self._tracker: Optional[cv2.TrackerNano] = None
        self._initialized = False
        self._frame_size: Optional[Tuple[int, int]] = None  # (width, height)

        if not os.path.isfile(self._backbone_path):
            raise FileNotFoundError(
                f"NanoTrack backbone model not found: {self._backbone_path}")
        if not os.path.isfile(self._neckhead_path):
            raise FileNotFoundError(
                f"NanoTrack neckhead model not found: {self._neckhead_path}")

    def _create_tracker(self) -> cv2.TrackerNano:
        params = cv2.TrackerNano.Params()
        params.backbone = self._backbone_path
        params.neckhead = self._neckhead_path
        return cv2.TrackerNano.create(params)

    def init(self, frame: np.ndarray, bbox_xywh: Tuple[int, int, int, int]):
        """Initialize the tracker on a target bounding box.

        Args:
            frame: BGR image (H, W, 3) uint8.
            bbox_xywh: (x, y, width, height) in pixel coordinates.
        """
        self._tracker = self._create_tracker()
        h, w = frame.shape[:2]
        self._frame_size = (w, h)
        self._tracker.init(frame, bbox_xywh)
        was_initialized = self._initialized
        self._initialized = True
        if not was_initialized:
            LOGGER.info("[SOT] NanoTrack initialized bbox=%s frame=%dx%d",
                        bbox_xywh, w, h)
        else:
            LOGGER.debug("[SOT] NanoTrack re-init bbox=%s", bbox_xywh)

    def update(self, frame: np.ndarray) -> SOTResult:
        """Update the tracker with a new frame.

        Args:
            frame: BGR image (H, W, 3) uint8.

        Returns:
            SOTResult with tracking status, bbox, and confidence.
        """
        if not self._initialized or self._tracker is None:
            return SOTResult(ok=False)

        t0 = time.monotonic()
        ok, bbox = self._tracker.update(frame)
        score = self._tracker.getTrackingScore()
        elapsed_ms = (time.monotonic() - t0) * 1000.0

        h, w = frame.shape[:2]

        if not ok or score < self._confidence_threshold:
            LOGGER.debug("[SOT] Target lost (ok=%s, score=%.3f, %.1fms)",
                         ok, score, elapsed_ms)
            return SOTResult(ok=False, confidence=score)

        x, y, bw, bh = [int(v) for v in bbox]
        cx_norm = (x + bw / 2) / w
        cy_norm = (y + bh / 2) / h
        bh_norm = bh / h

        LOGGER.debug("[SOT] Tracking ok  score=%.3f  bbox=(%d,%d,%d,%d)  %.1fms",
                     score, x, y, bw, bh, elapsed_ms)

        return SOTResult(
            ok=True,
            bbox=(x, y, bw, bh),
            confidence=score,
            center_norm=(cx_norm, cy_norm),
            bbox_height_norm=bh_norm,
        )

    def reset(self):
        """Reset the tracker (e.g., when switching back to MOT mode)."""
        self._tracker = None
        self._initialized = False
        self._frame_size = None
        LOGGER.info("[SOT] Reset")

    @property
    def is_initialized(self) -> bool:
        return self._initialized
