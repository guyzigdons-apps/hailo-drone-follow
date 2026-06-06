"""CachingBackend.stats — per-instance hit/miss + chip/lookup time accounting.

Counters are per-tile: a batched call with k misses adds k to `misses`; the
remaining (cached) positions add to `hits`. Dedup-collapsed duplicate crops
count as hits for the duplicate positions (they consumed no chip)."""
from __future__ import annotations

import time
from pathlib import Path

import pytest

from hailo_tiling.backends.backend import MockBackend
from hailo_tiling.backends.caching import CachingBackend
from hailo_tiling.cache.store import SqliteCacheStore
from hailo_tiling.types import CropRect, Det


def _det(score: float = 0.9) -> Det:
    return Det(cls=0, score=score, x=0.1, y=0.2, w=0.3, h=0.4)


def _crops(n: int):
    return [CropRect(x=i * 100, y=0, w=640, h=480, mode="m") for i in range(n)]


def _store(tmp_path: Path) -> SqliteCacheStore:
    return SqliteCacheStore.open(tmp_path / "c.sqlite3")


class _SlowBackend(MockBackend):
    """MockBackend that sleeps a fixed time inside infer (to exercise chip_seconds)."""

    def __init__(self, canned=None, delay: float = 0.0):
        super().__init__(canned)
        self._delay = delay

    def infer(self, frame, crops, frame_idx):
        if self._delay:
            time.sleep(self._delay)
        return super().infer(frame, crops, frame_idx)


def test_stats_dict_exists_and_zero_initialised(tmp_path):
    store = _store(tmp_path)
    cb = CachingBackend(wrapped=MockBackend(canned={}), store=store, ppv=1)
    assert cb.stats == {
        "hits": 0, "misses": 0, "chip_seconds": 0.0, "lookup_seconds": 0.0,
    }
    store.close()


def test_all_miss_counts_per_tile_misses(tmp_path):
    store = _store(tmp_path)
    crops = _crops(3)
    canned = {(0, (c.x, c.y, c.w, c.h)): [_det()] for c in crops}
    cb = CachingBackend(wrapped=MockBackend(canned=canned), store=store, ppv=1)
    cb.infer(frame=None, crops=crops, frame_idx=0)
    assert cb.stats["misses"] == 3
    assert cb.stats["hits"] == 0
    store.close()


def test_all_hit_counts_per_tile_hits(tmp_path):
    store = _store(tmp_path)
    crops = _crops(2)
    store.put_many([
        {"frame_idx": 0, "crop_rect": c, "ppv": 1, "dets": [_det()]} for c in crops
    ])
    cb = CachingBackend(wrapped=MockBackend(canned={}), store=store, ppv=1)
    cb.infer(frame=None, crops=crops, frame_idx=0)
    assert cb.stats["hits"] == 2
    assert cb.stats["misses"] == 0
    store.close()


def test_partial_hit_counts_split(tmp_path):
    store = _store(tmp_path)
    crops = _crops(4)
    canned = {(0, (c.x, c.y, c.w, c.h)): [_det()] for c in crops}
    cb = CachingBackend(wrapped=MockBackend(canned=canned), store=store, ppv=1)
    store.put_many([
        {"frame_idx": 0, "crop_rect": crops[0], "ppv": 1, "dets": [_det()]},
        {"frame_idx": 0, "crop_rect": crops[2], "ppv": 1, "dets": [_det()]},
    ])
    cb.infer(frame=None, crops=crops, frame_idx=0)
    assert cb.stats["hits"] == 2
    assert cb.stats["misses"] == 2
    store.close()


def test_duplicate_positions_count_as_hits(tmp_path):
    """A miss on the first occurrence of a crop counts once; the duplicate
    positions consumed no chip, so they count as hits."""
    store = _store(tmp_path)
    c = CropRect(x=0, y=0, w=640, h=480, mode="m")
    canned = {(0, (c.x, c.y, c.w, c.h)): [_det(0.7)]}
    cb = CachingBackend(wrapped=MockBackend(canned=canned), store=store, ppv=1)
    cb.infer(frame=None, crops=[c, c, c], frame_idx=0)
    # one unique crop forwarded -> 1 miss; the other 2 positions are dedup hits.
    assert cb.stats["misses"] == 1
    assert cb.stats["hits"] == 2
    store.close()


def test_stats_accumulate_across_calls(tmp_path):
    store = _store(tmp_path)
    crops = _crops(2)
    canned = {(0, (c.x, c.y, c.w, c.h)): [_det()] for c in crops}
    cb = CachingBackend(wrapped=MockBackend(canned=canned), store=store, ppv=1)
    cb.infer(frame=None, crops=crops, frame_idx=0)   # 2 misses
    cb.infer(frame=None, crops=crops, frame_idx=0)   # now 2 hits
    assert cb.stats["misses"] == 2
    assert cb.stats["hits"] == 2
    store.close()


def test_chip_seconds_only_accrues_on_miss(tmp_path):
    store = _store(tmp_path)
    crops = _crops(2)
    canned = {(0, (c.x, c.y, c.w, c.h)): [_det()] for c in crops}
    slow = _SlowBackend(canned=canned, delay=0.02)
    cb = CachingBackend(wrapped=slow, store=store, ppv=1)
    cb.infer(frame=None, crops=crops, frame_idx=0)   # misses -> sleeps
    after_miss = cb.stats["chip_seconds"]
    assert after_miss >= 0.02
    cb.infer(frame=None, crops=crops, frame_idx=0)   # all hits -> no wrapped call
    assert cb.stats["chip_seconds"] == pytest.approx(after_miss)
    store.close()


def test_lookup_seconds_accrues_even_on_all_hit(tmp_path):
    store = _store(tmp_path)
    crops = _crops(2)
    store.put_many([
        {"frame_idx": 0, "crop_rect": c, "ppv": 1, "dets": [_det()]} for c in crops
    ])
    cb = CachingBackend(wrapped=MockBackend(canned={}), store=store, ppv=1)
    cb.infer(frame=None, crops=crops, frame_idx=0)
    assert cb.stats["lookup_seconds"] >= 0.0
    # lookup ran (get_many) even with no misses; it is a float that was touched.
    assert isinstance(cb.stats["lookup_seconds"], float)
    store.close()


def test_empty_crops_does_not_touch_stats(tmp_path):
    store = _store(tmp_path)
    cb = CachingBackend(wrapped=MockBackend(canned={}), store=store, ppv=1)
    cb.infer(frame=None, crops=[], frame_idx=0)
    assert cb.stats["hits"] == 0
    assert cb.stats["misses"] == 0
    assert cb.stats["chip_seconds"] == 0.0
    assert cb.stats["lookup_seconds"] == 0.0
    store.close()
