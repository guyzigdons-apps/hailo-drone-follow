"""MAVSDK-backed telemetry provider.

The `mavsdk` package is imported lazily on first `_pull_snapshot_sync()` call so
the library installs cleanly on machines without it. To use this provider,
install with the optional extras: `pip install hailo_tiling[mavsdk]`.
"""
from __future__ import annotations

from dataclasses import replace
from typing import Optional

from .provider import TelemetryProvider, TelemetrySnapshot

_MAVSDK_INSTALL_HINT = (
    "MavsdkTelemetry requires the optional `mavsdk` package. "
    "Install it with: pip install 'hailo_tiling[mavsdk]'"
)


class MavsdkTelemetry(TelemetryProvider):
    """Pulls telemetry from a PX4 (or other MAVLink) endpoint via MAVSDK.

    Connection is lazy: the MAVSDK system is created and connected on first
    `snapshot()` call, not at construction time. Subsequent calls reuse the
    same connection. If the connection fails or mavsdk is not installed, the
    error is raised lazily on the first `snapshot()` call.
    """

    def __init__(self, system_address: str = "udp://:14540"):
        self.system_address = system_address
        self._system = None
        self._last_pull: Optional[TelemetrySnapshot] = None

    def _pull_snapshot_sync(self) -> Optional[TelemetrySnapshot]:
        """Pull one synchronous-feeling snapshot. Returns None if no data yet.

        Real implementation:
        1. On first call, lazy-import `mavsdk` (ImportError surfaces here).
        2. If `self._system is None`, run a short asyncio task that connects
           and subscribes to position/attitude/velocity streams; cache the
           latest values in `self._last_pull` via the stream callbacks.
        3. Return `self._last_pull` (may be None until the first stream tick).

        Plan 8 (drone-follow migration) replaces this skeleton with the real
        asyncio impl. Plan 2 ships only the seam + ImportError plumbing so
        downstream code (AltitudeZoomModifier integration tests) can run.
        """
        try:
            import mavsdk  # noqa: F401
        except ImportError as e:
            raise ImportError(_MAVSDK_INSTALL_HINT) from e
        return self._last_pull

    def snapshot(self, t: float) -> TelemetrySnapshot:
        pulled = self._pull_snapshot_sync()
        if pulled is None:
            return TelemetrySnapshot(timestamp=t)
        return replace(pulled, timestamp=t)
