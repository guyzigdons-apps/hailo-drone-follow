"""TelemetrySnapshot — frozen dataclass with optional fields and a NULL sentinel."""
from __future__ import annotations

import dataclasses

import pytest

from hailo_tiling.telemetry import TelemetrySnapshot, NULL_SNAPSHOT


def test_snapshot_is_frozen():
    s = TelemetrySnapshot(timestamp=0.0)
    with pytest.raises(dataclasses.FrozenInstanceError):
        s.timestamp = 1.0  # type: ignore[misc]


def test_snapshot_defaults_are_none_except_timestamp():
    s = TelemetrySnapshot(timestamp=12.5)
    assert s.timestamp == 12.5
    assert s.altitude_agl_m is None
    assert s.yaw_rate_rad_s is None
    assert s.velocity_world is None
    assert s.attitude_quat is None


def test_snapshot_accepts_all_fields():
    s = TelemetrySnapshot(
        altitude_agl_m=12.5,
        yaw_rate_rad_s=0.1,
        velocity_world=(1.0, -2.0, 0.5),
        attitude_quat=(1.0, 0.0, 0.0, 0.0),
        timestamp=3.14,
    )
    assert s.altitude_agl_m == 12.5
    assert s.velocity_world == (1.0, -2.0, 0.5)


def test_null_snapshot_is_a_singleton_constant():
    assert NULL_SNAPSHOT.timestamp == 0.0
    assert NULL_SNAPSHOT.altitude_agl_m is None
    from hailo_tiling.telemetry import NULL_SNAPSHOT as also_null
    assert also_null is NULL_SNAPSHOT
