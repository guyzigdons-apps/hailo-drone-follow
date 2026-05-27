from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from .aggregator import map_to_source, nms
from .scheduler import TileScheduler
from .target_lock import TargetLock


@dataclass
class RunResult:
    pred_traj: dict = field(default_factory=dict)   # frame_idx -> (x,y,w,h) | absent
    frame_dets: dict = field(default_factory=dict)  # frame_idx -> list[Det]
    total_tiles: int = 0
    n_frames: int = 0

    @property
    def avg_tiles_per_frame(self) -> float:
        return self.total_tiles / self.n_frames if self.n_frames else 0.0


def run(frames, src_w: int, src_h: int, scheduler: TileScheduler,
        lock: TargetLock, backend, meter, gt_traj: dict,
        person_cls: int = 0) -> RunResult:
    res = RunResult()
    frame_idx = -1
    for frame_idx, frame in enumerate(frames):
        crops = scheduler.decide(lock.state, frame_idx, meter)
        meter.charge(len(crops), frame_idx)
        res.total_tiles += len(crops)

        dets = []
        for crop in crops:
            local = backend.infer(frame, crop, frame_idx)
            dets += map_to_source(local, crop, src_w, src_h)
        dets = nms(dets, iou_thr=0.5)
        res.frame_dets[frame_idx] = dets

        persons = [d for d in dets if d.cls == person_cls]
        gt_box = gt_traj.get(frame_idx)
        if lock.track_id is None and gt_box is not None:
            state = lock.step(persons, gt_bbox_norm=gt_box)
        else:
            state = lock.step(persons)

        if state.status == "TRACKING":
            res.pred_traj[frame_idx] = tuple(state.bbox_norm)
    res.n_frames = frame_idx + 1
    return res


def emit_frames_json(res: RunResult, label: str, out_path: Path,
                     class_labels=("person", "vehicle", "face", "license_plate")) -> None:
    """Write a frames.json the existing overlay_viewer can load."""
    frames = []
    for f, dets in sorted(res.frame_dets.items()):
        frames.append({"frame": f, "detections": [
            {"label": class_labels[d.cls] if d.cls < len(class_labels) else str(d.cls),
             "confidence": d.score, "bbox": [d.x, d.y, d.w, d.h]} for d in dets]})
    out_path.write_text(json.dumps({"label": label, "frames": frames}))
