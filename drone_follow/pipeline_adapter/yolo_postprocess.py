"""YOLOv8 post-processing for HailoRT NMS output.

The YOLOv8 HEF on Hailo15 includes built-in NMS (hailortpp), so the output
is already a list of per-class detection arrays. This module converts that
to drone-follow Detection objects.

No manual anchor decoding or NMS is needed.
"""

import time
from typing import List

import numpy as np

from drone_follow.follow_api.types import Detection

# COCO class index for "person"
PERSON_CLASS_ID = 0


def extract_person_detections(
    nms_output: list,
    confidence_threshold: float = 0.4,
) -> List[Detection]:
    """Extract person detections from HailoRT NMS output.

    Args:
        nms_output: List of numpy arrays, one per class. Each array has shape
                    (num_detections, 5) with columns [ymin, xmin, ymax, xmax, score].
                    Coordinates are normalized to [0, 1].
        confidence_threshold: Minimum confidence to keep a detection.

    Returns:
        List of Detection objects for persons found.
    """
    detections = []
    now = time.monotonic()

    if PERSON_CLASS_ID >= len(nms_output):
        return detections

    person_dets = nms_output[PERSON_CLASS_ID]
    if len(person_dets) == 0:
        return detections

    for det in person_dets:
        ymin, xmin, ymax, xmax, score = det[:5]
        if score < confidence_threshold:
            continue

        bbox_height = ymax - ymin
        cx = (xmin + xmax) / 2
        cy = (ymin + ymax) / 2

        detections.append(Detection(
            label="person",
            confidence=float(score),
            center_x=float(cx),
            center_y=float(cy),
            bbox_height=float(bbox_height),
            timestamp=now,
        ))

    return detections
