"""SqliteCacheStore — Python wrapper over the §7.2 schema.

Lifetime: `open()` returns an open store; `close()` flushes and closes.
Reads use point-lookups against the composite PRIMARY KEY (no full scans).
Writes batch through a single transaction.
"""
from __future__ import annotations

import sqlite3
import time
from pathlib import Path
from typing import Iterable, Sequence

_SCHEMA_VERSION = 1
_SCHEMA_FILE = Path(__file__).resolve().parent / "schema.sql"


class SqliteCacheStore:
    """SQLite-backed tile cache. One file per (video_sha, hef_sha) pair."""

    def __init__(self, path: Path, con: sqlite3.Connection):
        self.path = path
        self._con = con

    @classmethod
    def open(cls, path: str | Path) -> "SqliteCacheStore":
        """Open an existing cache or create a new one at `path`.

        Verifies `PRAGMA user_version` matches `_SCHEMA_VERSION` on an
        existing file; raises `ValueError` on mismatch (no auto-migrate).
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        existed = path.exists()
        con = sqlite3.connect(path, isolation_level=None)
        con.execute("PRAGMA journal_mode = WAL")
        con.execute("PRAGMA synchronous = NORMAL")
        if existed:
            uv = con.execute("PRAGMA user_version").fetchone()[0]
            if uv == 0:
                cls._apply_schema(con)
            elif uv != _SCHEMA_VERSION:
                con.close()
                raise ValueError(
                    f"{path}: cache schema_version mismatch "
                    f"(file={uv}, expected={_SCHEMA_VERSION}). "
                    "Delete the file or use a matching hailo_tiling version."
                )
        else:
            cls._apply_schema(con)
        return cls(path, con)

    @staticmethod
    def _apply_schema(con: sqlite3.Connection) -> None:
        sql = _SCHEMA_FILE.read_text()
        con.executescript(sql)

    def close(self) -> None:
        try:
            self._con.commit()
        except sqlite3.Error:
            pass
        self._con.close()

    def __enter__(self) -> "SqliteCacheStore":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def meta_get(self, k: str) -> str | None:
        row = self._con.execute("SELECT v FROM meta WHERE k = ?", (k,)).fetchone()
        return row[0] if row else None

    def meta_put(self, k: str, v: str) -> None:
        self._con.execute(
            "INSERT INTO meta (k, v) VALUES (?, ?) "
            "ON CONFLICT(k) DO UPDATE SET v = excluded.v",
            (k, str(v)),
        )
