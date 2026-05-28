"""StaticTelemetry — returns a fixed snapshot, timestamp substituted in."""
from __future__ import annotations

from hailo_tiling.telemetry import StaticTelemetry, TelemetrySnapshot


def test_static_returns_configured_values():
    p = StaticTelemetry(altitude_agl_m=25.0, yaw_rate_rad_s=0.0)
    s = p.snapshot(0.0)
    assert s.altitude_agl_m == 25.0
    assert s.yaw_rate_rad_s == 0.0
    assert s.timestamp == 0.0


def test_static_substitutes_timestamp_on_each_call():
    p = StaticTelemetry(altitude_agl_m=10.0)
    s0 = p.snapshot(0.0)
    s5 = p.snapshot(5.5)
    assert s0.timestamp == 0.0 and s5.timestamp == 5.5
    # Domain fields unchanged across calls.
    assert s0.altitude_agl_m == s5.altitude_agl_m == 10.0


def test_static_defaults_all_none():
    p = StaticTelemetry()
    s = p.snapshot(0.0)
    assert s.altitude_agl_m is None
    assert s.yaw_rate_rad_s is None
    assert s.velocity_world is None
    assert s.attitude_quat is None


def test_static_snapshot_is_telemetry_snapshot():
    p = StaticTelemetry(altitude_agl_m=1.0)
    assert isinstance(p.snapshot(0.0), TelemetrySnapshot)
