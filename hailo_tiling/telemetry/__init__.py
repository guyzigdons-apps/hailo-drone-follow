"""Telemetry abstraction — the only seam that knows what MAVSDK is.

See docs/superpowers/specs/2026-05-28-tiling-library-design.md §3.3.
"""
from .provider import TelemetryProvider, TelemetrySnapshot, NULL_SNAPSHOT  # noqa: F401
from .static import StaticTelemetry  # noqa: F401
from .recorded import RecordedTelemetry  # noqa: F401
from .mavsdk import MavsdkTelemetry  # noqa: F401
