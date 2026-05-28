from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from .aggregator import map_to_source, nms
from .scheduler import TileScheduler, MultiTargetTileScheduler
from .target_lock import TargetLock, MultiTargetLock


@dataclass
class RunResult:
    pred_traj: dict = field(default_factory=dict)   # frame_idx -> (x,y,w,h) | absent
    frame_dets: dict = field(default_factory=dict)  # frame_idx -> list[Det]
    frame_tiles: dict = field(default_factory=dict)  # frame_idx -> list[(x_n,y_n,w_n,h_n,category)]
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
        # `lock.track_id` never clears in the replay harness (single-target,
        # set once), so this gates GT-seeded locking to the pre-lock phase only.
        pre_status = lock.state.status
        if lock.track_id is None and gt_box is not None:
            state = lock.step(persons, gt_bbox_norm=gt_box)
        else:
            state = lock.step(persons)

        # Tag each crop with its visualization category.
        tagged: list = []
        if pre_status in ("SEARCHING", "LOST") and lock.track_id is not None:
            # All recovery tiles.
            for c in crops:
                tagged.append((c.x / src_w, c.y / src_h,
                               c.w / src_w, c.h / src_h, "single-scale"))
        else:
            # TRACKING (or initial pre-lock): scheduler emits ROI first (mode "s")
            # then discovery grid (mode "m"). Tag the first "s" crop as the
            # tracker ROI ("dynamic"); "m" crops are the discovery grid
            # ("multi-scale"); any further "s" crops fall back to "single-scale".
            roi_tagged = False
            for c in crops:
                if c.mode == "m":
                    cat = "multi-scale"
                else:  # "s"
                    if pre_status == "TRACKING" and not roi_tagged:
                        cat = "dynamic"
                        roi_tagged = True
                    else:
                        cat = "single-scale"
                tagged.append((c.x / src_w, c.y / src_h,
                               c.w / src_w, c.h / src_h, cat))
        res.frame_tiles[frame_idx] = tagged

        if state.status == "TRACKING":
            res.pred_traj[frame_idx] = tuple(state.bbox_norm)
    res.n_frames = frame_idx + 1
    return res


def emit_frames_json(res: RunResult, label: str, out_path: Path,
                     class_labels=("person", "vehicle", "face", "license_plate")) -> None:
    """Write a frames.json the existing overlay_viewer can load."""
    frames = []
    for f, dets in sorted(res.frame_dets.items()):
        tiles_out = []
        for (tx, ty, tw, th, tcat) in res.frame_tiles.get(f, []):
            tiles_out.append({"x": tx, "y": ty, "w": tw, "h": th, "category": tcat})
        frames.append({"frame": f, "detections": [
            {"label": class_labels[d.cls] if 0 <= d.cls < len(class_labels) else str(d.cls),
             "confidence": d.score, "bbox": [d.x, d.y, d.w, d.h]} for d in dets],
            "tiles": tiles_out})
    out_path.write_text(json.dumps({"label": label, "frames": frames}))


def run_multi(frames, src_w: int, src_h: int,
              scheduler: MultiTargetTileScheduler,
              lock: MultiTargetLock, backend, meter, gt_traj: dict,
              gt_cls: int = 0) -> RunResult:
    """Multi-target replay loop.

    Feeds all allowed-class dets to the lock, asks the scheduler for
    per-target ROIs, and records the selected target's bbox in pred_traj
    (same schema as run() for score_run compatibility).

    GT bbox is passed every frame while lock.selected_key is None — the
    lock's internal guard prevents double-seeding.  This fixes the Phase 1
    deviation where the GT seed fired on frame 0 before ByteTracker had
    activated any tracks.
    """
    res = RunResult()
    frame_idx = -1

    for frame_idx, frame in enumerate(frames):
        # Build current target list from lock state (empty on frame 0).
        current_targets = [s for s in lock.targets.values()
                           if s.status != "LOST"]

        # Inform the scheduler which target is selected (for always-served logic).
        scheduler.set_selected(lock.selected_key)

        # scheduler.decide() now returns list[ScheduledTile].
        scheduled = scheduler.decide(current_targets, frame_idx, meter)
        meter.charge(len(scheduled), frame_idx)
        res.total_tiles += len(scheduled)

        # Inference over all requested crops.
        dets = []
        for tile in scheduled:
            local = backend.infer(frame, tile.crop, frame_idx)
            dets += map_to_source(local, tile.crop, src_w, src_h)
        dets = nms(dets, iou_thr=0.5)
        res.frame_dets[frame_idx] = dets

        # Feed all allowed-class dets to the multi-target lock.
        # Pass gt_bbox_norm every frame while selected_key is None so the
        # lock can seed as soon as ByteTracker activates the first track.
        dets_for_lock = [d for d in dets if d.cls in lock.target_classes]
        gt_box = gt_traj.get(frame_idx)
        if lock.selected_key is None and gt_box is not None:
            targets = lock.step(dets_for_lock,
                                gt_bbox_norm=gt_box, gt_cls=gt_cls)
        else:
            targets = lock.step(dets_for_lock)

        # Tag tiles with visualization categories (scheduler provides them).
        tagged: list = []
        for tile in scheduled:
            tagged.append((tile.crop.x / src_w, tile.crop.y / src_h,
                           tile.crop.w / src_w, tile.crop.h / src_h,
                           tile.category))
        res.frame_tiles[frame_idx] = tagged

        # Record selected target bbox for single-target scoring compatibility.
        sel = next(
            (s for s in targets if s.selected and s.status == "TRACKING"),
            None
        )
        if sel is not None:
            res.pred_traj[frame_idx] = tuple(sel.bbox_norm)

    res.n_frames = frame_idx + 1
    return res
