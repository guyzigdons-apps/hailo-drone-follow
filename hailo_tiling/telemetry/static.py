"""Constant-valued telemetry provider for tests and offline runs."""
from __future__ import annotations

from dataclasses import replace
from typing import Optional, Tuple

from .provider import TelemetryProvider, TelemetrySnapshot


class StaticTelemetry(TelemetryProvider):
    """Provider that returns the same snapshot for every t.

    The `timestamp` of the returned snapshot is the requested `t`, so callers
    that key on timestamp (e.g., for caching) still see distinct snapshots.
    """

    def __init__(
        self,
        altitude_agl_m: Optional[float] = None,
        yaw_rate_rad_s: Optional[float] = None,
        velocity_world: Optional[Tuple[float, float, float]] = None,
        attitude_quat: Optional[Tuple[float, float, float, float]] = None,
    ):
        self._template = TelemetrySnapshot(
            altitude_agl_m=altitude_agl_m,
            yaw_rate_rad_s=yaw_rate_rad_s,
            velocity_world=velocity_world,
            attitude_quat=attitude_quat,
            timestamp=0.0,
        )

    def snapshot(self, t: float) -> TelemetrySnapshot:
        return replace(self._template, timestamp=t)
