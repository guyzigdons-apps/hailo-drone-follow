"""CachingBackend — wraps any InferenceBackend with an SQLite cache."""
from __future__ import annotations

from pathlib import Path

import pytest

from hailo_tiling.backends import MockBackend
from hailo_tiling.backends.caching import CachingBackend
from hailo_tiling.cache.store import SqliteCacheStore
from hailo_tiling.types import CropRect, Det


def _det(score: float = 0.9) -> Det:
    return Det(cls=0, score=score, x=0.1, y=0.2, w=0.3, h=0.4)


def _crops(n: int):
    return [CropRect(x=i * 100, y=0, w=640, h=480, mode="m") for i in range(n)]


def _store(tmp_path: Path) -> SqliteCacheStore:
    return SqliteCacheStore.open(tmp_path / "c.sqlite3")


def test_all_miss_forwards_to_wrapped_then_stores(tmp_path):
    store = _store(tmp_path)
    crops = _crops(3)
    canned = {(0, (c.x, c.y, c.w, c.h)): [_det(0.5 + 0.1 * i)] for i, c in enumerate(crops)}
    mock = MockBackend(canned=canned)
    cb = CachingBackend(wrapped=mock, store=store, ppv=1)
    out = cb.infer(frame=None, crops=crops, frame_idx=0)
    assert len(out) == 3
    assert mock.call_count == 1
    assert mock.calls[0]["crops"] == crops
    out2 = cb.infer(frame=None, crops=crops, frame_idx=0)
    assert out2 == out
    assert mock.call_count == 1
    store.close()


def test_partial_hit_forwards_only_misses(tmp_path):
    store = _store(tmp_path)
    crops = _crops(4)
    canned = {(0, (c.x, c.y, c.w, c.h)): [_det(0.1 * i)] for i, c in enumerate(crops)}
    mock = MockBackend(canned=canned)
    cb = CachingBackend(wrapped=mock, store=store, ppv=1)
    store.put_many([
        {"frame_idx": 0, "crop_rect": crops[0], "ppv": 1, "dets": [_det(0.99)]},
        {"frame_idx": 0, "crop_rect": crops[2], "ppv": 1, "dets": [_det(0.88)]},
    ])
    out = cb.infer(frame=None, crops=crops, frame_idx=0)
    assert mock.call_count == 1
    assert mock.calls[0]["crops"] == [crops[1], crops[3]]
    assert out[0][0].score == pytest.approx(0.99)
    assert out[2][0].score == pytest.approx(0.88)
    assert out[1] == canned[(0, (crops[1].x, crops[1].y, crops[1].w, crops[1].h))]
    assert out[3] == canned[(0, (crops[3].x, crops[3].y, crops[3].w, crops[3].h))]
    store.close()


def test_all_hit_skips_wrapped(tmp_path):
    store = _store(tmp_path)
    crops = _crops(2)
    store.put_many([
        {"frame_idx": 0, "crop_rect": c, "ppv": 1, "dets": [_det(0.5 + 0.1 * i)]}
        for i, c in enumerate(crops)
    ])
    mock = MockBackend(canned={})
    cb = CachingBackend(wrapped=mock, store=store, ppv=1)
    out = cb.infer(frame=None, crops=crops, frame_idx=0)
    assert mock.call_count == 0
    assert all(len(d) == 1 for d in out)
    store.close()


def test_misses_are_stored_in_one_transaction(tmp_path, monkeypatch):
    store = _store(tmp_path)
    crops = _crops(5)
    canned = {(0, (c.x, c.y, c.w, c.h)): [_det()] for c in crops}
    mock = MockBackend(canned=canned)
    cb = CachingBackend(wrapped=mock, store=store, ppv=1)

    # sqlite3.Connection.execute is read-only in Python 3.10+ so we can't
    # monkeypatch it directly; use set_trace_callback to spy on SQL instead.
    seen_calls = []

    def _trace(sql):
        # set_trace_callback fires on every SQL statement executed.
        seen_calls.append(sql.strip().split()[0].upper())

    store._con.set_trace_callback(_trace)
    try:
        cb.infer(frame=None, crops=crops, frame_idx=0)
    finally:
        store._con.set_trace_callback(None)
    begins = seen_calls.count("BEGIN")
    commits = seen_calls.count("COMMIT")
    assert begins == 1
    assert commits == 1
    store.close()


def test_empty_crops_returns_empty(tmp_path):
    store = _store(tmp_path)
    mock = MockBackend(canned={})
    cb = CachingBackend(wrapped=mock, store=store, ppv=1)
    assert cb.infer(frame=None, crops=[], frame_idx=0) == []
    assert mock.call_count == 0
    store.close()


def test_duplicate_crops_collapse_to_one_query(tmp_path):
    store = _store(tmp_path)
    c = CropRect(x=0, y=0, w=640, h=480, mode="m")
    canned = {(0, (c.x, c.y, c.w, c.h)): [_det(0.7)]}
    mock = MockBackend(canned=canned)
    cb = CachingBackend(wrapped=mock, store=store, ppv=1)
    out = cb.infer(frame=None, crops=[c, c, c], frame_idx=0)
    assert mock.call_count == 1
    assert mock.calls[0]["crops"] == [c]
    assert len(out) == 3
    assert out[0] == out[1] == out[2]
    store.close()


def test_ppv_isolation(tmp_path):
    store = _store(tmp_path)
    c = CropRect(x=0, y=0, w=640, h=480, mode="m")
    canned = {(0, (c.x, c.y, c.w, c.h)): [_det(0.5)]}
    mock = MockBackend(canned=canned)
    cb1 = CachingBackend(wrapped=mock, store=store, ppv=1)
    cb2 = CachingBackend(wrapped=mock, store=store, ppv=2)
    cb1.infer(frame=None, crops=[c], frame_idx=0)
    assert mock.call_count == 1
    cb2.infer(frame=None, crops=[c], frame_idx=0)
    assert mock.call_count == 2
    store.close()


def test_canonicalize_respected_when_quantise_set(tmp_path):
    store = _store(tmp_path)
    # With floor-quantise q=4, both c_orig=(100,100,640,480) and
    # c_off=(101,102,641,482) canonicalise to (100,100,640,480).
    c_orig = CropRect(x=100, y=100, w=640, h=480, mode="m")
    c_off = CropRect(x=101, y=102, w=641, h=482, mode="m")
    canned = {(0, (c_orig.x, c_orig.y, c_orig.w, c_orig.h)): [_det(0.7)]}
    mock = MockBackend(canned=canned)
    cb = CachingBackend(wrapped=mock, store=store, ppv=1, quantise=4)
    cb.infer(frame=None, crops=[c_orig], frame_idx=0)
    out = cb.infer(frame=None, crops=[c_off], frame_idx=0)
    assert mock.call_count == 1
    assert out[0][0].score == pytest.approx(0.7)
    store.close()
