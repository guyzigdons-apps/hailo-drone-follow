"""MavsdkTelemetry — lazy MAVSDK import + graceful degradation when absent."""
from __future__ import annotations

import importlib
import sys
import types

import pytest

from hailo_tiling.telemetry import MavsdkTelemetry, TelemetrySnapshot


def _mavsdk_installed() -> bool:
    try:
        import mavsdk  # noqa: F401
        return True
    except ImportError:
        return False


def test_import_works_without_mavsdk():
    """The class must be importable on a machine that doesn't have mavsdk."""
    assert MavsdkTelemetry is not None


def test_construct_without_mavsdk_does_not_raise():
    """Construction is a no-op pending lazy connect; no ImportError yet."""
    t = MavsdkTelemetry(system_address="udp://:14540")
    assert isinstance(t, MavsdkTelemetry)


@pytest.mark.skipif(_mavsdk_installed(),
                    reason="mavsdk IS installed; this test verifies the absent-path")
def test_snapshot_raises_clear_error_when_mavsdk_absent():
    t = MavsdkTelemetry(system_address="udp://:14540")
    with pytest.raises(ImportError) as exc:
        t.snapshot(0.0)
    msg = str(exc.value).lower()
    assert "mavsdk" in msg
    assert "pip install" in msg or "extras" in msg


def test_snapshot_with_mocked_mavsdk(monkeypatch):
    """With a faked seam, snapshot() returns a populated TelemetrySnapshot."""
    captured = TelemetrySnapshot(
        altitude_agl_m=12.3,
        yaw_rate_rad_s=0.05,
        velocity_world=(1.0, -1.0, 0.2),
        attitude_quat=(1.0, 0.0, 0.0, 0.0),
        timestamp=0.0,
    )
    t = MavsdkTelemetry(system_address="udp://:14540")
    monkeypatch.setattr(t, "_pull_snapshot_sync", lambda: captured)

    s = t.snapshot(7.0)
    assert s.altitude_agl_m == 12.3
    assert s.timestamp == 7.0
    assert s.velocity_world == (1.0, -1.0, 0.2)


def test_snapshot_returns_null_fields_when_pull_returns_none(monkeypatch):
    """If the internal pull returns None (e.g., no link yet), degrade to NULL fields."""
    t = MavsdkTelemetry(system_address="udp://:14540")
    monkeypatch.setattr(t, "_pull_snapshot_sync", lambda: None)
    s = t.snapshot(3.0)
    assert s.altitude_agl_m is None
    assert s.timestamp == 3.0
