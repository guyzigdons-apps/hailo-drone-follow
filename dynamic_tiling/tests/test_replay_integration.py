import numpy as np
from dynamic_tiling.types import CropRect
from dynamic_tiling.budget import BudgetMeter
from dynamic_tiling.scheduler import TileScheduler
from dynamic_tiling.target_lock import TargetLock
from dynamic_tiling.inference import ReplayBackend
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
