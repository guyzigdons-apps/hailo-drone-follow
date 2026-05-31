"""YOLOv8 post-processing for HailoRT NMS output.

The YOLOv8 HEF on Hailo15 includes built-in NMS (hailortpp), so the output
is already a list of per-class detection arrays. This module converts that
to drone-follow Detection objects.

No manual anchor decoding or NMS is needed.
"""

import time
from typing import List

import numpy as np

from robot_follow.follow_api.types import Detection

# COCO class index for "person"
PERSON_CLASS_ID = 0


def _parse_nms_by_class_buffer(flat_buffer: np.ndarray):
    """Parse HAILO_NMS_BY_CLASS flat float32 buffer into per-class detection arrays.

    The buffer is organized as num_classes blocks of fixed stride:
        [count, det0_y1, det0_x1, det0_y2, det0_x2, det0_score, det1_y1, ...]

    stride_per_class = 1 (count) + max_proposals_per_class * 5 (bbox params)

    Returns list of (N, 5) arrays, one per class.
    """
    total = len(flat_buffer)
    bbox_params = 5

    # Find (num_classes, max_proposals) that divide evenly
    num_classes = None
    max_proposals = None
    for mp in [100, 64, 200, 50, 300, 10, 20]:
        stride = 1 + mp * bbox_params
        if total % stride == 0:
            num_classes = total // stride
            max_proposals = mp
            break

    if num_classes is None:
        raise ValueError(f"Cannot determine NMS structure from buffer size {total}")

    per_class = []
    stride = 1 + max_proposals * bbox_params
    for c in range(num_classes):
        offset = c * stride
        count = int(flat_buffer[offset])
        count = max(0, min(count, max_proposals))
        if count > 0:
            start = offset + 1
            dets = flat_buffer[start:start + count * bbox_params].reshape(count, bbox_params)
        else:
            dets = np.empty((0, bbox_params), dtype=np.float32)
        per_class.append(dets)

    return per_class


def extract_person_detections(
    nms_output,
    confidence_threshold: float = 0.4,
) -> List[Detection]:
    """Extract person detections from HailoRT NMS output.

    Args:
        nms_output: Either a flat numpy float32 array (HAILO_NMS_BY_CLASS format)
                    or a list of per-class (N, 5) arrays.
                    Bbox columns: [ymin, xmin, ymax, xmax, score], normalized [0, 1].
        confidence_threshold: Minimum confidence to keep a detection.

    Returns:
        List of Detection objects for persons found.
    """
    # Parse flat NMS buffer into per-class arrays if needed
    if isinstance(nms_output, np.ndarray) and nms_output.ndim == 1:
        nms_output = _parse_nms_by_class_buffer(nms_output)

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
        bbox_width = xmax - xmin
        cx = (xmin + xmax) / 2
        cy = (ymin + ymax) / 2

        detections.append(Detection(
            label="person",
            confidence=float(score),
            center_x=float(cx),
            center_y=float(cy),
            bbox_height=float(bbox_height),
            bbox_width=float(bbox_width),
            timestamp=now,
        ))

    return detections
