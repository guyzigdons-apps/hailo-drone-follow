"""Tests for tiling_lab.bench.metrics (matched-compute delta — Night-2 B3)."""
from __future__ import annotations

from tiling_lab.bench.metrics import (
    matched_compute_delta,
    recall_precision_vs_reference,
)
from hailo_tiling.classes import FACE, PERSON, VEHICLE
from hailo_tiling.types import Det


_STATIC = [
    {"name": "1x1", "mean_tiles": 1.0, "recall": 0.28},
    {"name": "2x2", "mean_tiles": 4.0, "recall": 0.34},
    {"name": "6x4", "mean_tiles": 24.0, "recall": 0.44},
    {"name": "12x9", "mean_tiles": 108.0, "recall": 1.0},
]


def test_matched_compute_picks_closest_static_budget():
    # A dynamic row at 0.4 tiles/frame matches the 1x1 static grid (closest).
    name, delta = matched_compute_delta(0.4, 0.05, _STATIC)
    assert name == "1x1"
    assert delta == 0.05 - 0.28


def test_matched_compute_mid_budget():
    # 5 tiles/frame is closest to 2x2 (4.0) not 6x4 (24.0).
    name, delta = matched_compute_delta(5.0, 0.40, _STATIC)
    assert name == "2x2"
    assert abs(delta - (0.40 - 0.34)) < 1e-9


def test_matched_compute_empty_static():
    assert matched_compute_delta(1.0, 0.5, []) == (None, None)


def test_matched_compute_skips_none_recall():
    rows = [{"name": "x", "mean_tiles": 1.0, "recall": None},
            {"name": "y", "mean_tiles": 2.0, "recall": 0.3}]
    name, delta = matched_compute_delta(1.5, 0.4, rows)
    assert name == "y"  # the None-recall row is skipped
    assert abs(delta - 0.1) < 1e-9


def _box(cls):
    # One box per class at a fixed location so pred/ref overlap perfectly.
    return Det(cls=cls, score=0.9, x=0.4, y=0.4, w=0.2, h=0.2)


def test_keep_classes_excludes_face_and_plate():
    # Reference + prediction each carry person, vehicle and face boxes.
    preds = {0: [_box(PERSON), _box(VEHICLE), _box(FACE)]}
    refs = {0: [_box(PERSON), _box(VEHICLE), _box(FACE)]}

    # Unfiltered: all three classes count (perfect recall over 3 boxes).
    recall, precision, n_ref = recall_precision_vs_reference(preds, refs)
    assert n_ref == 3
    assert recall == 1.0 and precision == 1.0

    # keep_classes drops face: only person + vehicle are scored.
    recall, precision, n_ref = recall_precision_vs_reference(
        preds, refs, keep_classes=(PERSON, VEHICLE)
    )
    assert n_ref == 2
    assert recall == 1.0 and precision == 1.0


def test_keep_classes_filters_false_positives():
    # Prediction has an extra face box the reference lacks; with face kept it
    # is a false positive, with face dropped precision is perfect.
    preds = {0: [_box(PERSON), _box(FACE)]}
    refs = {0: [_box(PERSON)]}

    recall, precision, _ = recall_precision_vs_reference(preds, refs)
    assert recall == 1.0 and precision == 0.5  # the face is an FP

    recall, precision, _ = recall_precision_vs_reference(
        preds, refs, keep_classes=(PERSON, VEHICLE)
    )
    assert recall == 1.0 and precision == 1.0  # face dropped from both sides
