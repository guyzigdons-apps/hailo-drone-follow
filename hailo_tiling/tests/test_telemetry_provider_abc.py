"""TelemetryProvider — ABC contract."""
from __future__ import annotations

import pytest

from hailo_tiling.telemetry import TelemetryProvider, TelemetrySnapshot


def test_cannot_instantiate_bare_abc():
    with pytest.raises(TypeError):
        TelemetryProvider()  # type: ignore[abstract]


def test_subclass_without_snapshot_method_fails():
    class _BadProvider(TelemetryProvider):
        pass  # missing snapshot()
    with pytest.raises(TypeError):
        _BadProvider()  # type: ignore[abstract]


def test_subclass_returns_snapshot():
    class _GoodProvider(TelemetryProvider):
        def snapshot(self, t):
            return TelemetrySnapshot(timestamp=t, altitude_agl_m=10.0)

    p = _GoodProvider()
    s = p.snapshot(1.5)
    assert isinstance(s, TelemetrySnapshot)
    assert s.timestamp == 1.5
    assert s.altitude_agl_m == 10.0
