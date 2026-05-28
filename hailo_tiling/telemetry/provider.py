"""TelemetryProvider ABC + TelemetrySnapshot dataclass.

Fields match the spec §3.3 — every domain field is Optional so consumers
gracefully degrade when a stream is missing (offline runs, MAVSDK absent,
sensor dropout). The `timestamp` field is always populated (defaults to 0.0
for the NULL sentinel).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class TelemetrySnapshot:
    """Per-frame telemetry slice consumed by tiling modifiers and tracker components.

    See spec §3.3. Every field except `timestamp` is Optional.
    """
    altitude_agl_m: Optional[float] = None
    yaw_rate_rad_s: Optional[float] = None
    velocity_world: Optional[Tuple[float, float, float]] = None
    attitude_quat: Optional[Tuple[float, float, float, float]] = None
    timestamp: float = 0.0


# Sentinel used by the scheduler when no provider is wired.
NULL_SNAPSHOT = TelemetrySnapshot()


class TelemetryProvider(ABC):
    """Returns a TelemetrySnapshot for a given monotonic time."""

    @abstractmethod
    def snapshot(self, t: float) -> TelemetrySnapshot:
        """Return the snapshot at (or nearest to) monotonic time `t` seconds."""
