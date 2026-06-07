# hailo_tiling/tests/test_budget.py
"""BudgetMeter unit tests.

The former dynamic-tiling ``budget`` re-export shim was removed by the
tiling-lab restructure (2026-06-07); its identity-parity test went with it.
"""
from __future__ import annotations

from hailo_tiling.budget import BudgetMeter


def test_available_at_start_returns_per_frame_share():
    meter = BudgetMeter(budget_inf_per_s=300.0, fps=30.0, window_s=1.0)
    # At start, full window cap available; per-frame share = 300/30 = 10.0
    assert meter.available(0) == 10.0


def test_charge_and_available_after_spend():
    meter = BudgetMeter(budget_inf_per_s=300.0, fps=30.0, window_s=1.0)
    meter.charge(n_tiles=5, frame_idx=0)
    # 300 cap - 5 spent = 295 remaining over 30 frames = 9.833...
    assert meter.available(1) == 295.0 / 30.0


def test_eviction_after_window():
    meter = BudgetMeter(budget_inf_per_s=300.0, fps=30.0, window_s=1.0)
    meter.charge(n_tiles=300, frame_idx=0)
    # After window_frames + 1, the old charge is evicted: full budget back.
    assert meter.available(31) == 10.0
