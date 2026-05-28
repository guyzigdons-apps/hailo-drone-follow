"""ReplayBackend — chip-free cache reader. Misses are loud."""
from __future__ import annotations

from pathlib import Path

import pytest

from hailo_tiling.backends import ReplayBackend
from hailo_tiling.cache import CacheMissError
from hailo_tiling.cache.store import SqliteCacheStore
from hailo_tiling.types import CropRect, Det


def _det(s: float = 0.9) -> Det:
    return Det(cls=0, score=s, x=0.1, y=0.1, w=0.1, h=0.1)


def _store(tmp_path: Path) -> SqliteCacheStore:
    return SqliteCacheStore.open(tmp_path / "r.sqlite3")


def test_all_hit_returns_dets_in_order(tmp_path):
    store = _store(tmp_path)
    crops = [CropRect(x=i * 100, y=0, w=640, h=480, mode="m") for i in range(3)]
    store.put_many([
        {"frame_idx": 0, "crop_rect": c, "ppv": 1, "dets": [_det(0.5 + 0.1 * i)]}
        for i, c in enumerate(crops)
    ])
    be = ReplayBackend(store=store, ppv=1)
    out = be.infer(frame=None, crops=crops, frame_idx=0)
    assert [d[0].score for d in out] == pytest.approx([0.5, 0.6, 0.7])
    store.close()


def test_miss_raises_cache_miss_error(tmp_path):
    store = _store(tmp_path)
    c = CropRect(x=0, y=0, w=640, h=480, mode="m")
    be = ReplayBackend(store=store, ppv=1)
    with pytest.raises(CacheMissError) as exc:
        be.infer(frame=None, crops=[c], frame_idx=42)
    msg = str(exc.value)
    assert "frame_idx=42" in msg
    assert "0,0,640,480" in msg or "(0, 0, 640, 480)" in msg
    store.close()


def test_partial_miss_still_raises_loudly(tmp_path):
    store = _store(tmp_path)
    crops = [CropRect(x=i * 100, y=0, w=640, h=480, mode="m") for i in range(3)]
    store.put_many([
        {"frame_idx": 0, "crop_rect": crops[0], "ppv": 1, "dets": [_det()]},
    ])
    be = ReplayBackend(store=store, ppv=1)
    with pytest.raises(CacheMissError):
        be.infer(frame=None, crops=crops, frame_idx=0)
    store.close()


def test_empty_crops_returns_empty(tmp_path):
    store = _store(tmp_path)
    be = ReplayBackend(store=store, ppv=1)
    assert be.infer(frame=None, crops=[], frame_idx=0) == []
    store.close()


def test_ppv_isolation(tmp_path):
    store = _store(tmp_path)
    c = CropRect(x=0, y=0, w=640, h=480, mode="m")
    store.put_many([{"frame_idx": 0, "crop_rect": c, "ppv": 1, "dets": [_det()]}])
    be = ReplayBackend(store=store, ppv=2)
    with pytest.raises(CacheMissError):
        be.infer(frame=None, crops=[c], frame_idx=0)
    store.close()
