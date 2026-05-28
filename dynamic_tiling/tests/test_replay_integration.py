import numpy as np
from dynamic_tiling.types import CropRect
from dynamic_tiling.budget import BudgetMeter
from dynamic_tiling.scheduler import TileScheduler
from dynamic_tiling.target_lock import TargetLock
from dynamic_tiling.replay import run


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
            return [type("D", (), dict(cls=0, x=lx - lw / 2, y=ly - lh / 2,
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
            return [type("D", (), dict(cls=0, x=lx - lw / 2, y=ly - lh / 2,
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
