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


def test_run_multi_records_multi_traj_per_track():
    src_w, src_h = 4000, 3000
    n_frames = 6
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
                results.append(
                    type("D", (), dict(cls=PERSON, x=lx - lw / 2, y=ly - lh / 2,
                                       w=lw, h=lh, score=0.9))()
                )
            return results

    frames = [np.zeros((src_h, src_w, 3), np.uint8) for _ in range(n_frames)]
    lock = MultiTargetLock(target_classes=set(TRACKED_CLASSES))
    sched = MultiTargetTileScheduler(src_w, src_h, discovery_period=2)
    res = run_multi(
        frames=frames, src_w=src_w, src_h=src_h,
        scheduler=sched,
        lock=lock,
        backend=_TwoPersonBackend(),
        meter=BudgetMeter(budget_inf_per_s=300, fps=30),
        gt_traj=gt_traj,
        gt_cls=PERSON,
    )
    # New field: {frame_idx: {(cls, track_id): (x, y, w, h)}} for confirmed targets.
    assert hasattr(res, "multi_traj")
    assert res.multi_traj, "multi_traj should be populated once tracks confirm"
    some_frame = max(res.multi_traj)
    # Keys are (cls, track_id) tuples.
    assert all(isinstance(k, tuple) and len(k) == 2 for k in res.multi_traj[some_frame])
    # Each recorded bbox is a 4-tuple with positive size.
    for bb in res.multi_traj[some_frame].values():
        assert len(bb) == 4 and bb[2] > 0 and bb[3] > 0
    # Two persons should both be recorded on at least one frame.
    assert any(len(d) >= 2 for d in res.multi_traj.values())


def test_run_dynamic_default_dump_mot_person_matches_input(tmp_path, monkeypatch):
    """End-to-end: `run_dynamic --multi-target --dump-mot` with DEFAULT class
    args must dump the person track and have it IoU-match the input person bbox.

    Guards the person=1 convention in run_dynamic's CLI wiring
    (`--target-classes` default + the `gt_cls` passed to run_multi). A stale
    cls=0 convention makes GT selection never fire (recall 0) and dumps the
    wrong / no person track.
    """
    import json
    import sys
    import numpy as np
    from dynamic_tiling import run_dynamic
    from dynamic_tiling.types import CropRect

    src_w, src_h = 4000, 3000
    n_frames = 8
    # A single moving person, cls=PERSON (the network's true person id == 1).
    person = {f: (0.40 + 0.01 * f, 0.40, 0.08, 0.20) for f in range(n_frames)}

    class _FakeBackend:
        def __init__(self, *a, **k):
            pass

        def infer(self, frame, crop: CropRect, frame_idx):
            gx, gy, gw, gh = person[frame_idx]
            gcx = (gx + gw / 2) * src_w
            gcy = (gy + gh / 2) * src_h
            if not (crop.x <= gcx <= crop.x + crop.w and
                    crop.y <= gcy <= crop.y + crop.h):
                return []
            lx = (gcx - crop.x) / crop.w
            ly = (gcy - crop.y) / crop.h
            lw = gw * src_w / crop.w
            lh = gh * src_h / crop.h
            return [type("D", (), dict(cls=PERSON, x=lx - lw / 2, y=ly - lh / 2,
                                       w=lw, h=lh, score=0.9))()]

        def close(self):
            pass

    class _FakeCap:
        def __init__(self):
            self._i = 0

        def isOpened(self):
            return True

        def get(self, prop):
            import cv2
            return {cv2.CAP_PROP_FRAME_WIDTH: src_w,
                    cv2.CAP_PROP_FRAME_HEIGHT: src_h}.get(prop, 0)

        def read(self):
            if self._i >= n_frames:
                return False, None
            self._i += 1
            return True, np.zeros((src_h, src_w, 3), np.uint8)

        def release(self):
            pass

    # GT doc consumed by build_target_trajectory(label="person", anchor="largest").
    gt_doc = {"frames": [
        {"frame": f, "detections": [
            {"label": "person", "bbox": list(person[f]), "confidence": 1.0}]}
        for f in range(n_frames)]}
    gt_path = tmp_path / "gt.frames.json"
    gt_path.write_text(json.dumps(gt_doc))

    out_path = tmp_path / "out.frames.json"
    mot_path = tmp_path / "mot.json"

    monkeypatch.setattr(run_dynamic.cv2, "VideoCapture", lambda _p: _FakeCap())
    monkeypatch.setattr(run_dynamic, "HefBackend", _FakeBackend)
    monkeypatch.setattr(
        sys, "argv",
        ["run_dynamic", "--video", "fake.mp4", "--gt", str(gt_path),
         "--out", str(out_path), "--multi-target",
         "--dump-mot", str(mot_path)],  # DEFAULTS: target-classes, dump-mot-classes
    )
    run_dynamic.main()

    dump = json.loads(mot_path.read_text())["tracks"]
    assert dump, "person MOT dump must not be empty with default class args"

    last = str(n_frames - 1)
    px, py, pw, ph = person[n_frames - 1]

    def _iou(a, b):
        ax2, ay2 = a[0] + a[2], a[1] + a[3]
        bx2, by2 = b[0] + b[2], b[1] + b[3]
        ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
        inter = iw * ih
        if inter <= 0:
            return 0.0
        return inter / (a[2] * a[3] + b[2] * b[3] - inter)

    matched = max(
        (_iou(traj[last], (px, py, pw, ph))
         for traj in dump.values() if last in traj),
        default=0.0,
    )
    assert matched > 0.8, f"dumped person box must match input det, IoU={matched:.3f}"

    # GT-seeded selection must actually fire: the person is the GT target, so a
    # LOCK box (pred_traj entry) must appear in the emitted frames.json. With the
    # stale gt_cls=0 wiring the lock never selects the cls=1 person -> no LOCK box.
    out_doc = json.loads(out_path.read_text())
    lock_frames = sum(
        1 for fr in out_doc["frames"]
        if any(d["label"] == "LOCK" for d in fr["detections"])
    )
    assert lock_frames >= n_frames - 2, (
        f"person GT target must be selected/locked; got {lock_frames} LOCK frames")


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


def test_run_dynamic_backend_uses_class_offset_one(monkeypatch, tmp_path):
    """run_dynamic must build its HEF backend with class_offset=1 (person=1,
    vehicle=2 — the Phase-0 unified convention). Regression: it was omitted,
    so dets stayed 0-indexed and the multi-target trackers followed vehicles."""
    import sys
    import json as _json
    import dynamic_tiling.run_dynamic as rd

    captured = {}

    class _FakeBackend:
        def __init__(self, *a, **kw):
            captured.update(kw)
        def infer(self, frame, crop, frame_idx):
            return []
        def close(self):
            pass

    class _FakeCap:
        def __init__(self, *a): self.n = 0
        def get(self, prop): return {3: 640, 4: 480, 5: 30.0, 7: 2}.get(int(prop), 0)
        def read(self):
            self.n += 1
            import numpy as np
            return (self.n <= 2, np.zeros((480, 640, 3), dtype=np.uint8))
        def release(self): pass

    gt = tmp_path / "gt.frames.json"
    gt.write_text(_json.dumps({"frames": [{"frame": 0, "detections": [
        {"label": "person", "confidence": 1.0, "bbox": [0.4, 0.4, 0.1, 0.2]}], "tiles": []}]}))
    monkeypatch.setattr(rd, "HefBackend", _FakeBackend)
    import cv2
    monkeypatch.setattr(cv2, "VideoCapture", _FakeCap)
    monkeypatch.setattr(sys, "argv", ["run_dynamic", "--video", "v.mp4", "--gt", str(gt),
                                      "--multi-target", "--max-frames", "2",
                                      "--out", str(tmp_path / "o.json")])
    rd.main()
    assert captured.get("class_offset") == 1
