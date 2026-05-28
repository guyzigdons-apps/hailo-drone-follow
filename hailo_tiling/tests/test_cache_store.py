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
