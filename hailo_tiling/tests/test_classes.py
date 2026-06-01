"""The authoritative class mapping must match the installed HEF label file.

Regression guard for the off-by-one bug: the network's label file has a leading
"unlabeled" slot, so person is class id 1 (not 0). Earlier code assumed person=0,
which mislabelled persons as "vehicle" and seeded the tracker with the wrong
class. These tests pin the mapping and cross-check it against the installed json
when present.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from hailo_tiling import classes


def test_label_order_pins_person_at_one():
    assert classes.LABELS == (
        "unlabeled", "person", "vehicle", "face", "license_plate",
    )
    assert classes.PERSON == 1
    assert classes.VEHICLE == 2
    assert classes.FACE == 3
    assert classes.LICENSE_PLATE == 4
    assert classes.TARGET_CLASS == classes.PERSON
    # Face + license_plate are intentionally dropped from tracking.
    assert classes.TRACKED_CLASSES == (classes.PERSON, classes.VEHICLE)
    assert classes.FACE not in classes.TRACKED_CLASSES
    assert classes.LICENSE_PLATE not in classes.TRACKED_CLASSES


def test_label_helper():
    assert classes.label(1) == "person"
    assert classes.label(2) == "vehicle"
    assert classes.label(0) == "unlabeled"
    assert classes.label(99) == "cls99"


def test_matches_installed_hef_label_json():
    """Cross-check against the deployed label file if it is installed.

    Skipped on hosts without the HEF resources (CI laptop) so the suite stays
    portable; on the chip host this catches a future relabel/reorder.
    """
    p = Path(classes.LABELS_JSON_PATH)
    if not p.exists():
        pytest.skip(f"{p} not installed on this host")
    labels = json.loads(p.read_text())["labels"]
    assert tuple(labels) == classes.LABELS
