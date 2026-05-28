# scripts/smoke_hailo_tiling.py
"""End-to-end smoke test for hailo_tiling.

Runs a composed TileScheduler over a fixed scenario and prints the resulting
CropRects. Intended to be re-runnable as a CI sanity check.

Usage:
    python scripts/smoke_hailo_tiling.py
"""
from __future__ import annotations

from hailo_tiling import (
    BudgetMeter,
    BudgetTrimModifier,
    DiscoveryGridEmitter,
    LockState,
    RecoveryGridEmitter,
    TileScheduler,
    TrackROIEmitter,
)


def main() -> int:
    src_w, src_h = 3840, 2160
    scheduler = TileScheduler(
        emitters=[
            TrackROIEmitter(max_zoom=2.0, target_model_h=40.0, roi_margin_frac=0.25),
            DiscoveryGridEmitter(grid=(3, 2), period=15, mode="m"),
            RecoveryGridEmitter(grid=(3, 3), span=0.4, mode="s"),
        ],
        modifiers=[BudgetTrimModifier()],
    )
    lock = LockState(
        track_id=42,
        bbox_norm=(0.45, 0.40, 0.05, 0.15),
        status="TRACKING",
        frames_since_seen=0,
        last_velocity=(0.0, 0.0),
    )
    meter = BudgetMeter(budget_inf_per_s=300.0, fps=30.0)
    crops = scheduler.decide(src_w, src_h, lock, frame_idx=0, meter=meter)
    print(f"emitted {len(crops)} crops:")
    for c in crops:
        print(f"  {c}")
    return 0 if crops else 1


if __name__ == "__main__":
    raise SystemExit(main())
