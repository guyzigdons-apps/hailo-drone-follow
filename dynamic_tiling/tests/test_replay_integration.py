import numpy as np
from dynamic_tiling.types import CropRect
from dynamic_tiling.budget import BudgetMeter
from dynamic_tiling.scheduler import TileScheduler, MultiTargetTileScheduler
from dynamic_tiling.target_lock import TargetLock, MultiTargetLock
from dynamic_tiling.replay import run, run_multi
from hailo_tiling.classes import PERSON, TRACKED_CLASSES


def test_replay_tracks_target_with_replay_backend():
    src_w, src_h = 4000, 3000
    n_frames = 6
    gt_traj = {f: (0.40 + 0.01 * f, 0.40, 0.08, 0.20) for f in range(n_frames)}

    class _AnyCropBackend:
        def infer(self, frame, crop: CropRect, frame_idx):
            gx, gy, gw, gh = gt_traj[frame_idx]
            gcx = (gx + gw / 2) * src_w
            gcy = (gy + gh / 2) * src_h
            if not (crop.x <= gcx <= crop.x + crop.w and
                    crop.y <= gcy <= crop.y + crop.h):
                return []
            lx = (gcx - crop.x) / crop.w
            ly = (gcy - crop.y) / crop.h
            lw = gw * src_w / crop.w
            lh = gh * src_h / crop.h
            # cls=PERSON (1) — aligned to person=1 convention (was cls=0, off-by-one).
            return [type("D", (), dict(cls=PERSON, x=lx - lw / 2, y=ly - lh / 2,
                                       w=lw, h=lh, score=0.9))()]

    frames = [np.zeros((src_h, src_w, 3), np.uint8) for _ in range(n_frames)]
    result = run(
        frames=frames, src_w=src_w, src_h=src_h,
        scheduler=TileScheduler(src_w, src_h, discovery_period=2),
        lock=TargetLock(),
        backend=_AnyCropBackend(),
        meter=BudgetMeter(budget_inf_per_s=300, fps=30),
        gt_traj=gt_traj,
    )
    assert result.pred_traj.get(n_frames - 1) is not None
    assert result.avg_tiles_per_frame < 10
    assert sum(1 for f in range(n_frames) if result.pred_traj.get(f)) >= n_frames - 2


def test_replay_never_locked_path():
    src_w, src_h = 4000, 3000
    n_frames = 4
    gt_traj = {f: (0.40, 0.40, 0.08, 0.20) for f in range(n_frames)}

    class _NoDetBackend:
        def infer(self, frame, crop, frame_idx):
            return []

    frames = [np.zeros((src_h, src_w, 3), np.uint8) for _ in range(n_frames)]
    result = run(
        frames=frames, src_w=src_w, src_h=src_h,
        scheduler=TileScheduler(src_w, src_h, discovery_period=2),
        lock=TargetLock(),
        backend=_NoDetBackend(),
        meter=BudgetMeter(budget_inf_per_s=300, fps=30),
        gt_traj=gt_traj,
    )
    assert result.pred_traj == {}
    assert set(result.frame_dets.keys()) == set(range(n_frames))
    assert result.n_frames == n_frames


def test_replay_records_per_frame_tagged_tiles():
    src_w, src_h = 4000, 3000
    n_frames = 3
    gt_traj = {f: (0.40 + 0.01 * f, 0.40, 0.08, 0.20) for f in range(n_frames)}

    class _AlwaysHitBackend:
        def infer(self, frame, crop, frame_idx):
            gx, gy, gw, gh = gt_traj[frame_idx]
            gcx = (gx + gw / 2) * src_w
            gcy = (gy + gh / 2) * src_h
            if not (crop.x <= gcx <= crop.x + crop.w and
                    crop.y <= gcy <= crop.y + crop.h):
                return []
            lx = (gcx - crop.x) / crop.w
            ly = (gcy - crop.y) / crop.h
            lw = gw * src_w / crop.w
            lh = gh * src_h / crop.h
            # cls=PERSON (1) — aligned to person=1 convention (was cls=0, off-by-one).
            return [type("D", (), dict(cls=PERSON, x=lx - lw / 2, y=ly - lh / 2,
                                       w=lw, h=lh, score=0.9))()]

    frames = [np.zeros((src_h, src_w, 3), np.uint8) for _ in range(n_frames)]
    result = run(
        frames=frames, src_w=src_w, src_h=src_h,
        scheduler=TileScheduler(src_w, src_h, discovery_period=1,
                                discovery_grid=(3, 2)),
        lock=TargetLock(),
        backend=_AlwaysHitBackend(),
        meter=BudgetMeter(budget_inf_per_s=300, fps=30),
        gt_traj=gt_traj,
    )
    # Per-frame tiles recorded for every processed frame.
    assert set(result.frame_tiles.keys()) == set(range(n_frames))
    # Once TRACKING (frames 1+), the first "s" crop is the ROI -> tagged "dynamic".
    # Frame 0 builds the lock from GT; status starts LOST -> may or may not get
    # the dynamic tag (depends on lock activation timing), so check >= frame 1.
    seen_dynamic = any(
        any(t[4] == "dynamic" for t in result.frame_tiles[f])
        for f in range(1, n_frames)
    )
    assert seen_dynamic, f"no 'dynamic' tile tag found: {result.frame_tiles}"
    # And every per-frame tile is normalized 0..1 (within float error).
    for f, tiles in result.frame_tiles.items():
        for (tx, ty, tw, th, cat) in tiles:
            assert -1e-6 <= tx <= 1.0 + 1e-6
            assert -1e-6 <= ty <= 1.0 + 1e-6
            assert tw > 0 and th > 0
            assert cat in ("dynamic", "multi-scale", "single-scale")


def test_run_multi_records_per_target_rois():
    src_w, src_h = 4000, 3000
    n_frames = 6
    # Two persons moving slightly.
    gt_traj = {f: (0.40 + 0.01 * f, 0.40, 0.08, 0.20) for f in range(n_frames)}
    person2 = {f: (0.70 + 0.005 * f, 0.50, 0.08, 0.20) for f in range(n_frames)}

    class _TwoPersonBackend:
        def infer(self, frame, crop: CropRect, frame_idx):
            results = []
            for traj in [gt_traj, person2]:
                gx, gy, gw, gh = traj[frame_idx]
                gcx = (gx + gw / 2) * src_w
                gcy = (gy + gh / 2) * src_h
                if not (crop.x <= gcx <= crop.x + crop.w and
                        crop.y <= gcy <= crop.y + crop.h):
                    continue
                lx = (gcx - crop.x) / crop.w
                ly = (gcy - crop.y) / crop.h
                lw = gw * src_w / crop.w
                lh = gh * src_h / crop.h
                # cls=PERSON (1) — aligned to person=1 convention (was cls=0, off-by-one).
                results.append(
                    type("D", (), dict(cls=PERSON, x=lx - lw / 2, y=ly - lh / 2,
                                       w=lw, h=lh, score=0.9))()
                )
            return results

    frames = [np.zeros((src_h, src_w, 3), np.uint8) for _ in range(n_frames)]
    # target_classes uses TRACKED_CLASSES convention: PERSON=1, VEHICLE=2.
    lock = MultiTargetLock(target_classes=set(TRACKED_CLASSES))
    sched = MultiTargetTileScheduler(src_w, src_h, discovery_period=2)
    result = run_multi(
        frames=frames, src_w=src_w, src_h=src_h,
        scheduler=sched,
        lock=lock,
        backend=_TwoPersonBackend(),
        meter=BudgetMeter(budget_inf_per_s=300, fps=30),
        gt_traj=gt_traj,
        gt_cls=PERSON,
    )
    assert result.n_frames == n_frames
    # After a couple of frames to establish tracks, expect >= 2 frames with "dynamic" tiles.
    dynamic_frames = [
        f for f in range(2, n_frames)
        if any(t[4] == "dynamic" for t in result.frame_tiles.get(f, []))
    ]
    assert len(dynamic_frames) >= 2, (
        f"expected >=2 frames with 'dynamic' tiles, got {len(dynamic_frames)}: "
        f"{result.frame_tiles}"
    )


def test_emit_frames_json_keeps_tiles_and_appends_lock_box(tmp_path):
    import json
    from dynamic_tiling.replay import RunResult, emit_frames_json

    res = RunResult()
    res.frame_dets = {0: [], 1: []}
    res.frame_tiles = {0: [(0.0, 0.0, 0.5, 0.5, "discovery")],
                       1: [(0.2, 0.2, 0.3, 0.3, "recovery")]}
    res.pred_traj = {1: (0.25, 0.25, 0.1, 0.2)}  # locked only on frame 1
    res.n_frames = 2
    out = tmp_path / "t.frames.json"
    emit_frames_json(res, label="trial", out_path=out)
    doc = json.loads(out.read_text())
    f0, f1 = doc["frames"]
    assert f0["tiles"] == [{"x": 0.0, "y": 0.0, "w": 0.5, "h": 0.5, "category": "discovery"}]
    assert f1["tiles"][0]["category"] == "recovery"
    assert f0["detections"] == []  # no lock on frame 0 -> no LOCK box
    assert f1["detections"][-1] == {"label": "LOCK", "confidence": 1.0,
                                    "bbox": [0.25, 0.25, 0.1, 0.2]}


def test_emit_frames_json_renders_tracker_tags_and_search_anchor(tmp_path):
    import json
    from dynamic_tiling.replay import RunResult, emit_frames_json

    res = RunResult()
    res.frame_dets = {0: [], 1: []}
    res.frame_lock = {
        0: {"status": "TRACKING", "bt_id": 7,
            "tracks": [{"id": 7, "bbox": [0.1, 0.1, 0.05, 0.1], "activated": True}]},
        1: {"status": "SEARCHING", "bt_id": 7, "anchor": [0.1, 0.1, 0.05, 0.1],
            "tracks": [{"id": 9, "bbox": [0.4, 0.4, 0.05, 0.1], "activated": True},
                       {"id": 11, "bbox": [0.6, 0.6, 0.05, 0.1], "activated": False}]},
    }
    res.n_frames = 2
    out = tmp_path / "t.frames.json"
    emit_frames_json(res, label="trial", out_path=out)
    doc = json.loads(out.read_text())
    f0, f1 = doc["frames"]
    labels0 = [d["label"] for d in f0["detections"]]
    labels1 = [d["label"] for d in f1["detections"]]
    assert "trk7*" in labels0                      # lock's own track marked with *
    assert "trk9" in labels1 and "trk11" in labels1
    assert "ANCHOR[SEARCHING]" in labels1          # stale reacq anchor visible during loss
    assert "ANCHOR[TRACKING]" not in labels0       # no anchor box while tracking
    trk11 = next(d for d in f1["detections"] if d["label"] == "trk11")
    assert trk11["confidence"] < 0.5               # non-activated tracks render faint
