"""SqliteCacheStore unit tests."""
from __future__ import annotations

import sqlite3
from pathlib import Path

import pytest


def test_schema_sql_creates_expected_tables(tmp_path: Path):
    """schema.sql, when executed against an empty DB, produces the spec tables."""
    schema = (Path(__file__).resolve().parent.parent / "cache" / "schema.sql").read_text()
    db = tmp_path / "t.sqlite3"
    con = sqlite3.connect(db)
    try:
        con.executescript(schema)
        con.commit()
        cur = con.execute("PRAGMA user_version")
        assert cur.fetchone()[0] == 1
        names = {r[0] for r in con.execute(
            "SELECT name FROM sqlite_master WHERE type='table'"
        )}
        assert "detections" in names
        assert "meta" in names
        info = list(con.execute("PRAGMA table_info(detections)"))
        cols = {row[1] for row in info}
        assert cols == {
            "frame_idx", "crop_x", "crop_y", "crop_w", "crop_h",
            "ppv", "dets_json", "ts_epoch",
        }
    finally:
        con.close()


from hailo_tiling.cache.store import SqliteCacheStore


def test_store_open_creates_file_and_schema(tmp_path):
    db = tmp_path / "v__h.sqlite3"
    store = SqliteCacheStore.open(db)
    try:
        assert db.is_file()
        cur = store._con.execute("PRAGMA user_version")
        assert cur.fetchone()[0] == 1
        jm = store._con.execute("PRAGMA journal_mode").fetchone()[0].lower()
        assert jm == "wal"
    finally:
        store.close()


def test_store_open_existing_is_idempotent(tmp_path):
    db = tmp_path / "v__h.sqlite3"
    s1 = SqliteCacheStore.open(db)
    s1.close()
    s2 = SqliteCacheStore.open(db)
    assert s2 is not None
    s2.close()


def test_store_rejects_mismatched_schema_version(tmp_path):
    db = tmp_path / "old.sqlite3"
    con = sqlite3.connect(db)
    con.execute("PRAGMA user_version = 999")
    con.commit()
    con.close()
    with pytest.raises(ValueError, match="schema_version"):
        SqliteCacheStore.open(db)


def test_meta_put_and_get(tmp_path):
    store = SqliteCacheStore.open(tmp_path / "m.sqlite3")
    try:
        store.meta_put("video_sha256", "abc123")
        store.meta_put("video_fps", "30.0")
        assert store.meta_get("video_sha256") == "abc123"
        assert store.meta_get("video_fps") == "30.0"
        assert store.meta_get("missing") is None
    finally:
        store.close()


def test_meta_put_upserts(tmp_path):
    store = SqliteCacheStore.open(tmp_path / "u.sqlite3")
    try:
        store.meta_put("ppv", "1")
        store.meta_put("ppv", "2")
        assert store.meta_get("ppv") == "2"
    finally:
        store.close()


import json

from hailo_tiling.types import CropRect, Det


def _det(cls: int, score: float, x: float, y: float, w: float, h: float) -> Det:
    return Det(cls=cls, score=score, x=x, y=y, w=w, h=h)


def _row(frame_idx: int, crop: CropRect, ppv: int, dets):
    return {
        "frame_idx": frame_idx,
        "crop_rect": crop,
        "ppv": ppv,
        "dets": list(dets),
    }


def test_put_many_then_get_roundtrips(tmp_path):
    store = SqliteCacheStore.open(tmp_path / "rt.sqlite3")
    try:
        c1 = CropRect(x=0, y=0, w=640, h=480, mode="m")
        c2 = CropRect(x=640, y=0, w=640, h=480, mode="m")
        dets1 = [_det(0, 0.91, 0.1, 0.2, 0.3, 0.4)]
        dets2 = []
        store.put_many([
            _row(0, c1, 1, dets1),
            _row(0, c2, 1, dets2),
        ])
        got1 = store.get(0, c1, ppv=1)
        got2 = store.get(0, c2, ppv=1)
        assert got1 == dets1
        assert got2 == dets2
    finally:
        store.close()


def test_get_miss_returns_none(tmp_path):
    store = SqliteCacheStore.open(tmp_path / "miss.sqlite3")
    try:
        c = CropRect(x=10, y=10, w=640, h=480, mode="m")
        assert store.get(0, c, ppv=1) is None
    finally:
        store.close()


def test_put_many_duplicate_key_within_batch_is_first_writer_wins(tmp_path):
    """Plan 6 A1: a duplicate (frame_idx, crop, ppv) key inside one batch no
    longer raises (INSERT OR IGNORE) — the first row for the key wins and the
    batch commits. The previous behaviour (raise + roll back the whole batch)
    is intentionally replaced to make warming idempotent / re-runnable."""
    store = SqliteCacheStore.open(tmp_path / "tx.sqlite3")
    try:
        c_good = CropRect(x=0, y=0, w=640, h=480, mode="m")
        first = [_det(0, 0.9, 0, 0, 0.1, 0.1)]
        rows = [
            _row(0, c_good, 1, first),
            _row(0, c_good, 1, [_det(0, 0.1, 0, 0, 0.1, 0.1)]),  # dup key, ignored
        ]
        store.put_many(rows)  # must not raise
        # First-writer-wins: the stored dets are the first row's, count stays 1.
        assert store.get(0, c_good, ppv=1) == first
        assert store.stats()["n_rows"] == 1
    finally:
        store.close()


def test_put_many_double_insert_is_idempotent(tmp_path):
    """Plan 6 A1: re-recording the same row via a SECOND put_many call is a
    no-op — the row count stays 1 and no exception is raised. This is the
    re-runnable-warming guarantee on the Python side, mirroring the C++ test."""
    store = SqliteCacheStore.open(tmp_path / "idem.sqlite3")
    try:
        c = CropRect(x=10, y=20, w=100, h=200, mode="m")
        dets = [_det(0, 0.91, 0.1, 0.2, 0.3, 0.4)]
        store.put_many([_row(0, c, 1, dets)])
        assert store.stats()["n_rows"] == 1
        # Second insert of the identical key — idempotent no-op.
        store.put_many([_row(0, c, 1, dets)])
        assert store.stats()["n_rows"] == 1
        assert store.get(0, c, ppv=1) == dets
    finally:
        store.close()


def test_put_many_empty_is_noop(tmp_path):
    store = SqliteCacheStore.open(tmp_path / "noop.sqlite3")
    try:
        store.put_many([])
        assert store.stats()["n_rows"] == 0
    finally:
        store.close()


def test_get_many_preserves_order(tmp_path):
    store = SqliteCacheStore.open(tmp_path / "om.sqlite3")
    try:
        crops = [CropRect(x=i * 100, y=0, w=640, h=480, mode="m") for i in range(5)]
        store.put_many([
            _row(0, c, 1, [_det(0, 0.5 + 0.05 * i, 0, 0, 0.1, 0.1)])
            for i, c in enumerate(crops)
        ])
        rev = list(reversed(crops))
        out = store.get_many(0, rev, ppv=1)
        assert len(out) == 5
        for i, (c, dets) in enumerate(zip(rev, out)):
            assert dets is not None
            assert dets[0].score == pytest.approx(0.5 + 0.05 * (4 - i))
    finally:
        store.close()


def test_get_many_mixed_hit_miss(tmp_path):
    store = SqliteCacheStore.open(tmp_path / "mh.sqlite3")
    try:
        c_hit = CropRect(x=0, y=0, w=640, h=480, mode="m")
        c_miss = CropRect(x=100, y=0, w=640, h=480, mode="m")
        store.put_many([_row(0, c_hit, 1, [_det(0, 0.9, 0, 0, 0.1, 0.1)])])
        out = store.get_many(0, [c_hit, c_miss, c_hit], ppv=1)
        assert out[0] is not None and out[2] is not None
        assert out[1] is None
    finally:
        store.close()


def test_stats_reports_n_rows(tmp_path):
    store = SqliteCacheStore.open(tmp_path / "stats.sqlite3")
    try:
        assert store.stats()["n_rows"] == 0
        crops = [CropRect(x=i * 100, y=0, w=640, h=480, mode="m") for i in range(4)]
        store.put_many([_row(0, c, 1, []) for c in crops])
        s = store.stats()
        assert s["n_rows"] == 4
        assert s["schema_version"] == 1
        assert "path" in s
    finally:
        store.close()


def test_vacuum_runs(tmp_path):
    store = SqliteCacheStore.open(tmp_path / "vac.sqlite3")
    try:
        c = CropRect(x=0, y=0, w=640, h=480, mode="m")
        store.put_many([_row(0, c, 1, [])])
        store.vacuum()
        assert store.get(0, c, ppv=1) == []
    finally:
        store.close()


def test_embedding_roundtrip(tmp_path):
    import numpy as np
    from hailo_tiling.cache.store import SqliteCacheStore
    from hailo_tiling.types import CropRect
    s = SqliteCacheStore.open(tmp_path / "c.sqlite3")
    crop = CropRect(x=10, y=20, w=64, h=128)
    vec = np.arange(8, dtype=np.float32) / 10.0
    assert s.get_embedding(5, crop, model="repvgg") is None
    s.put_embedding(5, crop, model="repvgg", vec=vec)
    got = s.get_embedding(5, crop, model="repvgg")
    assert got is not None and got.dtype == np.float32
    assert np.allclose(got, vec)
    assert s.get_embedding(5, crop, model="osnet") is None   # model-keyed
    s.close()


def test_quantise_is_applied_at_put_time_when_enabled(tmp_path):
    """The store is canonicalisation-agnostic — put/get round-trip on exact key."""
    store = SqliteCacheStore.open(tmp_path / "q.sqlite3")
    try:
        c = CropRect(x=123, y=457, w=789, h=321, mode="m")
        store.put_many([_row(0, c, 1, [])])
        assert store.get(0, c, ppv=1) == []
        c2 = CropRect(x=120, y=456, w=788, h=320, mode="m")
        assert store.get(0, c2, ppv=1) is None
    finally:
        store.close()
