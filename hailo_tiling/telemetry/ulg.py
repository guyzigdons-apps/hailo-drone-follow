"""PX4 ULog (.ulg) → JSONL row adapter.

Reads a PX4 ULog flight log and emits a list of dicts shaped like a
`RecordedTelemetry` JSONL timeline (one row per `vehicle_local_position`
message, with last-value-carried-forward joins from the other topics).

Plan 7 Task 1 ships only the stub; Task 2 lands the implementation.
See `docs/superpowers/plans/2026-05-28-telemetry-import-visualizer.md`.
"""
from __future__ import annotations

from pathlib import Path


def parse_ulg(path: Path) -> list[dict]:
    """Parse a PX4 ULog file into a list of telemetry row dicts.

    Each row has keys: ``timestamp``, ``altitude_agl_m``, ``yaw_rate_rad_s``,
    ``velocity_world``, ``attitude_quat``, plus a ``_geo`` sidecar.

    Time base: monotonic seconds, with the first message at ``timestamp=0.0``.

    Stub: implemented in Plan 7 Task 2.
    """
    raise NotImplementedError(__name__)
