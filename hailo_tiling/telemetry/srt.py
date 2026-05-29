"""DJI SRT sidecar → JSONL row adapter.

Reads a DJI-style SRT sidecar file (one block per video frame, bracketed
key-value payload) and emits a list of dicts shaped like a
`RecordedTelemetry` JSONL timeline.

Plan 7 Task 1 ships only the stub; Task 3 lands the implementation.
See `docs/superpowers/plans/2026-05-28-telemetry-import-visualizer.md`.
"""
from __future__ import annotations

from pathlib import Path


def parse_srt(path: Path) -> list[dict]:
    """Parse a DJI SRT sidecar file into a list of telemetry row dicts.

    Each row has keys: ``timestamp``, ``altitude_agl_m``, ``yaw_rate_rad_s``,
    ``velocity_world``, ``attitude_quat``, plus a ``_geo`` sidecar. SRT carries
    no inertial data, so ``velocity_world``, ``attitude_quat``, and
    ``yaw_rate_rad_s`` are always ``None``.

    Stub: implemented in Plan 7 Task 3.
    """
    raise NotImplementedError(__name__)
