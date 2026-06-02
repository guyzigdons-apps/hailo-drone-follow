import numpy as np

from hailo_tiling.backends.hef import HefBackend
from hailo_tiling.classes import (PERSON, VEHICLE, FACE, LICENSE_PLATE,
                                   TRACKED_CLASSES, LABELS)
from hailo_tiling.types import CropRect, Det


def _fake_handle():
    class H:
        def infer(self, rgb):
            return "raw"
        def close(self):
            pass
    return H()


def test_class_offset_shifts_decoded_ids():
    # decode returns person=0 (raw NMS convention); offset must lift to person=1.
    def decode(_raw):
        return [Det(cls=0, score=0.9, x=0.1, y=0.1, w=0.2, h=0.3)]
    be = HefBackend(_fake_handle(), decode, class_offset=1)
    frame = np.zeros((48, 64, 3), np.uint8)
    out = be.infer(frame=frame, crops=[CropRect(0, 0, 64, 48)], frame_idx=0)
    assert out[0][0].cls == 1


def test_class_offset_defaults_to_zero():
    def decode(_raw):
        return [Det(cls=0, score=0.9, x=0.1, y=0.1, w=0.2, h=0.3)]
    be = HefBackend(_fake_handle(), decode)
    frame = np.zeros((48, 64, 3), np.uint8)
    out = be.infer(frame=frame, crops=[CropRect(0, 0, 64, 48)], frame_idx=0)
    assert out[0][0].cls == 0


def test_tracked_classes_contains_person_and_vehicle():
    assert PERSON in TRACKED_CLASSES
    assert VEHICLE in TRACKED_CLASSES


def test_tracked_classes_excludes_face_and_plate():
    assert FACE not in TRACKED_CLASSES
    assert LICENSE_PLATE not in TRACKED_CLASSES


def test_person_label_matches_labels_tuple():
    assert LABELS[PERSON] == "person"
    assert LABELS[VEHICLE] == "vehicle"
