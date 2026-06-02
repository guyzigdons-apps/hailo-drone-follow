import numpy as np

from dynamic_tiling.gt_mot import build_raw_tracks


class FakeTracker:
    """Assigns a stable id per detection by its column order (deterministic)."""
    def update(self, dets_xyxy_score_cls, frame):
        # dets_xyxy_score_cls: (N,6) [x1,y1,x2,y2,score,cls]; return (N,8)
        # [x1,y1,x2,y2,track_id,conf,cls,idx]
        n = dets_xyxy_score_cls.shape[0]
        out = np.zeros((n, 8), dtype=float)
        out[:, 0:4] = dets_xyxy_score_cls[:, 0:4]
        out[:, 4] = [i + 1 for i in range(n)]   # track id = row order
        out[:, 5] = dets_xyxy_score_cls[:, 4]
        out[:, 6] = dets_xyxy_score_cls[:, 5]
        return out


def test_build_raw_tracks_groups_by_track_id():
    doc = {"frames": [
        {"frame": 0, "detections": [
            {"bbox": [0.1, 0.1, 0.05, 0.1], "confidence": 0.9, "cls": 1, "label": "person"},
            {"bbox": [0.5, 0.5, 0.04, 0.08], "confidence": 0.8, "cls": 1, "label": "person"}]},
        {"frame": 1, "detections": [
            {"bbox": [0.12, 0.1, 0.05, 0.1], "confidence": 0.9, "cls": 1, "label": "person"},
            {"bbox": [0.5, 0.52, 0.04, 0.08], "confidence": 0.8, "cls": 1, "label": "person"}]},
    ]}
    tracks = build_raw_tracks(doc, tracker=FakeTracker(), classes=(1, 2))
    assert set(t.cls for t in tracks) == {1}
    assert all(len(t.frames) == 2 for t in tracks)
    ids = sorted(t.track_id for t in tracks)
    assert ids == [1, 2]


def test_build_raw_tracks_drops_untracked_classes():
    doc = {"frames": [{"frame": 0, "detections": [
        {"bbox": [0.1, 0.1, 0.05, 0.1], "confidence": 0.9, "cls": 3, "label": "face"}]}]}
    tracks = build_raw_tracks(doc, tracker=FakeTracker(), classes=(1, 2))
    assert tracks == []
