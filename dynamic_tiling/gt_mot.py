"""Offline ground-truth multi-object tracks from dense detector output.

Runs a HEAVY tracker (BoT-SORT: camera-motion compensation + ReID appearance)
over the dense 12x9 detections to produce one stable trajectory per object. This
is GT generation — it must be INDEPENDENT of the runtime ByteTracker under test.
The tracker is injectable so unit tests avoid the boxmot/torch dependency.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from hailo_tiling.classes import TRACKED_CLASSES
from .gt_dedup import dedup_frame, det_cls


@dataclass
class RawTrack:
    cls: int
    track_id: int
    frames: dict = field(default_factory=dict)  # frame_idx -> (x,y,w,h) normalized


def _dets_to_xyxy(frame_doc, classes):
    """Return (N,6) array [x1,y1,x2,y2,score,cls] in NORMALIZED coords."""
    rows = []
    for d in frame_doc.get("detections", []):
        c = det_cls(d)
        if c not in classes:
            continue
        x, y, w, h = d["bbox"]
        rows.append([x, y, x + w, y + h, float(d.get("confidence", 1.0)), c])
    return np.asarray(rows, dtype=float).reshape(-1, 6)


def build_raw_tracks(doc, *, tracker, classes=TRACKED_CLASSES):
    """Feed dense detections frame-by-frame to `tracker`; group outputs by id.

    `tracker.update(dets_xyxy_score_cls, frame)` must return rows
    [x1,y1,x2,y2,track_id,conf,cls,...] (the legacy numpy-row contract).
    This is the injection path used by unit tests (``FakeTracker``); it expects
    each row to be indexable as ``row[4]`` = track_id, ``row[6]`` = cls.
    The REAL boxmot-19 path (which parses ``TrackResults`` with ``.xyxy``/``.id``/
    ``.cls``) is ``build_raw_tracks_from_video``.

    Note: ``frame`` here is the raw frame *dict*, not a BGR image — this
    function is for injection-testing with trackers that ignore the image
    argument. For real BoT-SORT (CMC + ReID) use ``build_raw_tracks_from_video``.
    """
    classes = tuple(classes)
    out: dict = {}  # (cls, track_id) -> RawTrack
    frames = sorted(doc.get("frames", []), key=lambda fr: fr["frame"])
    for fr in frames:
        fi = fr["frame"]
        dets = _dets_to_xyxy(fr, classes)
        res = tracker.update(dets, fr)
        if res is None or len(res) == 0:
            continue
        for row in np.asarray(res, dtype=float):
            x1, y1, x2, y2 = row[0], row[1], row[2], row[3]
            tid, cls = int(row[4]), int(row[6])
            key = (cls, tid)
            t = out.get(key)
            if t is None:
                t = RawTrack(cls=cls, track_id=tid)
                out[key] = t
            t.frames[fi] = (x1, y1, x2 - x1, y2 - y1)
    return list(out.values())


def make_botsort(*, with_reid=False, cmc_method="ecc", frame_rate=30, track_buffer=90):
    """Construct a boxmot 19 BoT-SORT tracker for offline GT building.

    Camera-motion compensation (``cmc_method='ecc'``) is the dominant association
    fix for moving-camera drone footage and runs with no extra weights. ReID
    appearance is OFF by default: boxmot 19 requires a separately-constructed
    ``reid_model`` object, so it is deferred (CMC + motion is a strong GT builder;
    enable ReID later if crossings prove problematic). Falls back to OC-SORT only
    if BoT-SORT cannot be imported.
    """
    try:
        from boxmot.trackers import BotSort
    except ImportError:  # pragma: no cover - environment-dependent
        from boxmot.trackers import OcSort
        return OcSort()
    return BotSort(with_reid=with_reid, cmc_method=cmc_method,
                   frame_rate=frame_rate, track_buffer=track_buffer)


def build_raw_tracks_from_video(doc, video_path, *, tracker_factory=make_botsort,
                                classes=TRACKED_CLASSES, dedup_iou: float | None = None):
    """Pixel-coords BoT-SORT pass over the video frames + dense dets.

    boxmot needs the BGR frame (CMC + ReID) and pixel-coord dets. Detections
    are scaled to pixels per frame; results are stored back in NORMALIZED coords.
    """
    import cv2
    classes = tuple(classes)
    by_frame = {fr["frame"]: fr for fr in doc.get("frames", [])}
    tracker = tracker_factory()
    out: dict = {}
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise FileNotFoundError(f"cannot open video: {video_path}")
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fi = -1
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            fi += 1
            fr = by_frame.get(fi)
            rows = []
            if fr is not None:
                raw_dets = fr.get("detections", [])
                dets_in = dedup_frame(raw_dets, iou_thr=dedup_iou) if dedup_iou is not None else raw_dets
                for d in dets_in:
                    c = det_cls(d)
                    if c not in classes:
                        continue
                    x, y, bw, bh = d["bbox"]
                    rows.append([x * w, y * h, (x + bw) * w, (y + bh) * h,
                                 float(d.get("confidence", 1.0)), c])
            px = np.asarray(rows, dtype=float).reshape(-1, 6)
            res = tracker.update(px, frame)
            if res is None or len(res) == 0:
                continue
            xyxy = res.xyxy
            ids = res.id
            clss = res.cls
            for k in range(len(xyxy)):
                x1, y1, x2, y2 = (float(xyxy[k][0]), float(xyxy[k][1]),
                                  float(xyxy[k][2]), float(xyxy[k][3]))
                tid, cls = int(ids[k]), int(clss[k])
                key = (cls, tid)
                t = out.get(key)
                if t is None:
                    t = RawTrack(cls=cls, track_id=tid)
                    out[key] = t
                t.frames[fi] = (x1 / w, y1 / h, (x2 - x1) / w, (y2 - y1) / h)
    finally:
        cap.release()
    return list(out.values())
