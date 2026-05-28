"""DetectionMemory ABC + NoOpMemory."""
from __future__ import annotations

import pytest

from hailo_tiling.aggregator import DetectionMemory, NoOpMemory
from hailo_tiling.types import Det


def test_cannot_instantiate_abc():
    with pytest.raises(TypeError):
        DetectionMemory()  # type: ignore[abstract]


def test_noop_memory_observe_then_predict_returns_empty():
    mem = NoOpMemory()
    mem.observe([Det(cls=0, score=0.9, x=0.1, y=0.1, w=0.1, h=0.1)], frame_idx=0)
    assert mem.predict(frame_idx=1) == []


def test_noop_memory_reset_is_safe():
    mem = NoOpMemory()
    mem.observe([Det(cls=0, score=0.9, x=0.1, y=0.1, w=0.1, h=0.1)], frame_idx=0)
    mem.reset()
    assert mem.predict(frame_idx=1) == []


def test_noop_memory_implements_abc():
    assert isinstance(NoOpMemory(), DetectionMemory)
