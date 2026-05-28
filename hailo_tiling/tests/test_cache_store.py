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
