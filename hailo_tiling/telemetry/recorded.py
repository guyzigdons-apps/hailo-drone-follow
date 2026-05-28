"""JSONL-replay telemetry provider for offline / paper-reproducibility runs."""
from __future__ import annotations

import bisect
import json
from dataclasses import replace
from pathlib import Path
from typing import Iterable

from .provider import TelemetryProvider, TelemetrySnapshot


def _row_to_snapshot(row: dict) -> TelemetrySnapshot:
    def _tuple_or_none(v):
        if v is None:
            return None
        return tuple(float(x) for x in v)
    return TelemetrySnapshot(
        altitude_agl_m=row.get("altitude_agl_m"),
        yaw_rate_rad_s=row.get("yaw_rate_rad_s"),
        velocity_world=_tuple_or_none(row.get("velocity_world")),
        attitude_quat=_tuple_or_none(row.get("attitude_quat")),
        timestamp=float(row.get("timestamp", 0.0)),
    )


class RecordedTelemetry(TelemetryProvider):
    """Provider backed by an in-memory list of TelemetrySnapshots sorted by timestamp."""

    def __init__(self, snapshots: Iterable[TelemetrySnapshot]):
        snaps = sorted(snapshots, key=lambda s: s.timestamp)
        self._timestamps = [s.timestamp for s in snaps]
        self._snaps = snaps

    @classmethod
    def from_path(cls, path: Path | str) -> "RecordedTelemetry":
        snaps: list[TelemetrySnapshot] = []
        path = Path(path)
        if path.exists():
            for line in path.read_text(encoding="utf-8").splitlines():
                line = line.strip()
                if not line:
                    continue
                try:
                    row = json.loads(line)
                except json.JSONDecodeError:
                    continue
                snaps.append(_row_to_snapshot(row))
        return cls(snaps)

    def snapshot(self, t: float) -> TelemetrySnapshot:
        if not self._snaps:
            return TelemetrySnapshot(timestamp=t)
        idx = bisect.bisect_right(self._timestamps, t) - 1
        if idx < 0:
            return TelemetrySnapshot(timestamp=t)
        return replace(self._snaps[idx], timestamp=t)
