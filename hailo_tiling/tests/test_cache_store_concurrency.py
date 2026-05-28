"""WAL behaviour smoke test — one writer does not block one reader.

This is the lock-contention proof the spec calls out (§7.7). We do not test
multi-writer contention; the spec mandates "one bench process at a time per
cache file" and the warmer takes an advisory file lock in Plan 5.
"""
from __future__ import annotations

import time
from pathlib import Path

import pytest

from hailo_tiling.cache.store import SqliteCacheStore
from hailo_tiling.types import CropRect, Det


def _det(score: float) -> Det:
    return Det(cls=0, score=score, x=0.1, y=0.1, w=0.1, h=0.1)


def test_reader_sees_committed_rows_while_writer_is_open(tmp_path: Path):
    db = tmp_path / "wal.sqlite3"

    writer = SqliteCacheStore.open(db)
    try:
        c = CropRect(x=0, y=0, w=640, h=480, mode="m")
        writer.put_many([{
            "frame_idx": 0, "crop_rect": c, "ppv": 1, "dets": [_det(0.9)],
        }])

        reader = SqliteCacheStore.open(db)
        try:
            got = reader.get(0, c, ppv=1)
            assert got is not None
            assert got[0].score == pytest.approx(0.9)
        finally:
            reader.close()
    finally:
        writer.close()


def test_wal_persistent_across_reopen(tmp_path: Path):
    db = tmp_path / "wal2.sqlite3"
    s1 = SqliteCacheStore.open(db)
    s1.close()
    s2 = SqliteCacheStore.open(db)
    try:
        jm = s2._con.execute("PRAGMA journal_mode").fetchone()[0].lower()
        assert jm == "wal"
    finally:
        s2.close()
