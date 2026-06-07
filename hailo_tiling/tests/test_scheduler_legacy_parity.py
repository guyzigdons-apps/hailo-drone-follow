# hailo_tiling/tests/test_scheduler_legacy_parity.py
"""End-to-end golden-file parity vs hailo_tiling.dynamic.TileScheduler.

For each parameterised scenario, build a composed hailo_tiling.TileScheduler
with the same parameters and assert that `decide()` returns the byte-identical
list[CropRect] as the legacy scheduler.

Once Plan 8 (drone-follow migration) retires the promoted-scheduler parity
seam, this test will be replaced by static golden files generated from this
test's expected outputs.
"""
from __future__ import annotations

import pytest

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.emitters import (
    DiscoveryGridEmitter,
    RecoveryGridEmitter,
    TrackROIEmitter,
)
from hailo_tiling.modifiers import BudgetTrimModifier
from hailo_tiling.scheduler import TileScheduler
from hailo_tiling.types import LockState


# (description, src_w, src_h, scheduler_kwargs, lock, frame_idx, meter_args)
SCENARIOS = [
    (
        "tracking_on_cadence_4k",
        3840, 2160,
        dict(discovery_period=15, discovery_grid=(3, 2),
             recovery_grid=(3, 3), max_zoom=2.0, target_model_h=40.0,
             roi_margin_frac=0.25, recovery_span=0.4),
        LockState(track_id=42, bbox_norm=(0.45, 0.40, 0.05, 0.15),
                  status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0)),
        0,
        dict(budget_inf_per_s=1000.0, fps=30.0),
    ),
    (
        "tracking_off_cadence_4k",
        3840, 2160,
        dict(discovery_period=15, discovery_grid=(3, 2),
             recovery_grid=(3, 3), max_zoom=2.0, target_model_h=40.0,
             roi_margin_frac=0.25, recovery_span=0.4),
        LockState(track_id=42, bbox_norm=(0.45, 0.40, 0.05, 0.15),
                  status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0)),
        7,
        dict(budget_inf_per_s=1000.0, fps=30.0),
    ),
    (
        "searching_4k",
        3840, 2160,
        dict(discovery_period=15, discovery_grid=(3, 2),
             recovery_grid=(3, 3), max_zoom=2.0, target_model_h=40.0,
             roi_margin_frac=0.25, recovery_span=0.4),
        LockState(track_id=7, bbox_norm=(0.60, 0.55, 0.04, 0.12),
                  status="SEARCHING", frames_since_seen=5,
                  last_velocity=(0.001, -0.002)),
        12,
        dict(budget_inf_per_s=1000.0, fps=30.0),
    ),
    (
        "lost_with_track_id_4k",
        3840, 2160,
        dict(discovery_period=15, discovery_grid=(3, 2),
             recovery_grid=(3, 3), max_zoom=2.0, target_model_h=40.0,
             roi_margin_frac=0.25, recovery_span=0.4),
        LockState(track_id=99, bbox_norm=(0.50, 0.50, 0.05, 0.15),
                  status="LOST", frames_since_seen=20, last_velocity=(0.0, 0.0)),
        0,
        dict(budget_inf_per_s=1000.0, fps=30.0),
    ),
    (
        "lost_no_track_id_4k_off_cadence",
        3840, 2160,
        dict(discovery_period=15, discovery_grid=(3, 2),
             recovery_grid=(3, 3), max_zoom=2.0, target_model_h=40.0,
             roi_margin_frac=0.25, recovery_span=0.4),
        LockState(track_id=None, bbox_norm=(0.0, 0.0, 0.0, 0.0),
                  status="LOST", frames_since_seen=999, last_velocity=(0.0, 0.0)),
        3,
        dict(budget_inf_per_s=1000.0, fps=30.0),
    ),
    (
        "tracking_tight_budget_4k",
        3840, 2160,
        dict(discovery_period=15, discovery_grid=(3, 2),
             recovery_grid=(3, 3), max_zoom=2.0, target_model_h=40.0,
             roi_margin_frac=0.25, recovery_span=0.4),
        LockState(track_id=42, bbox_norm=(0.45, 0.40, 0.05, 0.15),
                  status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0)),
        0,
        dict(budget_inf_per_s=90.0, fps=30.0),   # 3 tiles/frame: ROI + first 2 discovery
    ),
    (
        "tracking_2k_source",
        1920, 1080,
        dict(discovery_period=10, discovery_grid=(2, 2),
             recovery_grid=(2, 2), max_zoom=2.0, target_model_h=40.0,
             roi_margin_frac=0.25, recovery_span=0.5),
        LockState(track_id=1, bbox_norm=(0.30, 0.30, 0.10, 0.20),
                  status="TRACKING", frames_since_seen=0, last_velocity=(0.0, 0.0)),
        0,
        dict(budget_inf_per_s=1000.0, fps=30.0),
    ),
]


@pytest.mark.parametrize(
    "desc,src_w,src_h,sched_kwargs,lock,frame_idx,meter_args",
    SCENARIOS,
    ids=[s[0] for s in SCENARIOS],
)
def test_composed_scheduler_matches_legacy(
    desc, src_w, src_h, sched_kwargs, lock, frame_idx, meter_args,
):
    from hailo_tiling.dynamic.scheduler import TileScheduler as LegacyScheduler

    legacy = LegacyScheduler(src_w, src_h, **sched_kwargs)
    legacy_out = legacy.decide(lock, frame_idx, BudgetMeter(**meter_args))

    new = TileScheduler(
        emitters=[
            TrackROIEmitter(
                max_zoom=sched_kwargs["max_zoom"],
                target_model_h=sched_kwargs["target_model_h"],
                roi_margin_frac=sched_kwargs["roi_margin_frac"],
            ),
            DiscoveryGridEmitter(
                grid=sched_kwargs["discovery_grid"],
                period=sched_kwargs["discovery_period"],
                mode="m",
            ),
            RecoveryGridEmitter(
                grid=sched_kwargs["recovery_grid"],
                span=sched_kwargs["recovery_span"],
                mode="s",
            ),
        ],
        modifiers=[BudgetTrimModifier()],
    )
    new_out = new.decide(src_w, src_h, lock, frame_idx, BudgetMeter(**meter_args))

    assert new_out == legacy_out, (
        f"[{desc}] scheduler outputs diverged.\n"
        f"  legacy: {legacy_out}\n"
        f"  new   : {new_out}"
    )
