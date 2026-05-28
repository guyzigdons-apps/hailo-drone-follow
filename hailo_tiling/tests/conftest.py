"""Shared fixtures for hailo_tiling tests."""
from __future__ import annotations

import pytest

from hailo_tiling.types import LockState


@pytest.fixture
def src_dims():
    """Standard 4K source dimensions used across scheduler tests."""
    return (3840, 2160)


@pytest.fixture
def tracking_lock():
    """A TRACKING lock state with a person bbox roughly centered."""
    return LockState(
        track_id=42,
        bbox_norm=(0.45, 0.40, 0.05, 0.15),
        status="TRACKING",
        frames_since_seen=0,
        last_velocity=(0.0, 0.0),
    )


@pytest.fixture
def searching_lock():
    """A SEARCHING lock state with last-known bbox and small velocity."""
    return LockState(
        track_id=7,
        bbox_norm=(0.60, 0.55, 0.04, 0.12),
        status="SEARCHING",
        frames_since_seen=5,
        last_velocity=(0.001, -0.002),
    )


@pytest.fixture
def lost_lock():
    """A LOST lock state."""
    return LockState(
        track_id=None,
        bbox_norm=(0.0, 0.0, 0.0, 0.0),
        status="LOST",
        frames_since_seen=999,
        last_velocity=(0.0, 0.0),
    )


# ----------------------------------------------------------------------
# Telemetry fixtures (added in Plan 2)
# ----------------------------------------------------------------------
from hailo_tiling.telemetry import (
    NULL_SNAPSHOT,
    StaticTelemetry,
    TelemetrySnapshot,
)


@pytest.fixture
def null_snapshot():
    return NULL_SNAPSHOT


@pytest.fixture
def low_altitude_snapshot():
    """5 m AGL — close to subject, modifiers should widen the ROI."""
    return TelemetrySnapshot(altitude_agl_m=5.0, timestamp=0.0)


@pytest.fixture
def high_altitude_snapshot():
    """40 m AGL — far from subject, modifiers should narrow the ROI."""
    return TelemetrySnapshot(altitude_agl_m=40.0, timestamp=0.0)


@pytest.fixture
def static_low_telemetry():
    return StaticTelemetry(altitude_agl_m=5.0)


@pytest.fixture
def static_high_telemetry():
    return StaticTelemetry(altitude_agl_m=40.0)


# ----------------------------------------------------------------------
# Backend fixtures (added in Plan 2)
# ----------------------------------------------------------------------
from hailo_tiling.backends import MockBackend


@pytest.fixture
def make_mock_backend():
    """Factory: returns a callable that builds a MockBackend with given canned dets."""
    def _make(canned=None):
        return MockBackend(canned or {})
    return _make
