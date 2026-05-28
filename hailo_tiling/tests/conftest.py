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
