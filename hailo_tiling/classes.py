"""Authoritative class mapping for the ``hailo_yolov8n_4_classes_vga`` HEF.

The model's label file (``/usr/local/hailo/resources/json/hailo_4_classes.json``)
is::

    ["unlabeled", "person", "vehicle", "face", "license_plate"]

i.e. there is a leading ``unlabeled`` placeholder at index 0 and the network
emits class ids **1..4**. Earlier code assumed ``person=0`` (off by one), which
mislabelled persons as "vehicle", vehicles as "face", etc., and caused the
single-target tracker to be seeded with the wrong class. This module is the one
place that mapping lives — import from here, never hard-code the order.
"""
from __future__ import annotations

# Index == class id emitted by the network (0 is the json's "unlabeled" slot).
LABELS: tuple[str, ...] = ("unlabeled", "person", "vehicle", "face", "license_plate")

UNLABELED = 0
PERSON = 1
VEHICLE = 2
FACE = 3
LICENSE_PLATE = 4

#: Single-target follow / tracker-lock class.
TARGET_CLASS = PERSON

#: Classes we keep for tracking + paper metrics. Face + license_plate are
#: dropped per project decision (we follow people, not faces/plates).
TRACKED_CLASSES: tuple[int, ...] = (PERSON, VEHICLE)

#: Path to the installed label file (for the cross-check test / tooling).
LABELS_JSON_PATH = "/usr/local/hailo/resources/json/hailo_4_classes.json"


def label(cls: int) -> str:
    """Human-readable label for a class id (``clsN`` for out-of-range ids)."""
    return LABELS[cls] if 0 <= cls < len(LABELS) else f"cls{cls}"
