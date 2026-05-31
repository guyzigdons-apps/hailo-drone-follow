"""SqliteCacheStore — Python wrapper over the §7.2 schema.

Lifetime: `open()` returns an open store; `close()` flushes and closes.
Reads use point-lookups against the composite PRIMARY KEY (no full scans).
Writes batch through a single transaction.
"""
from __future__ import annotations

import json
import sqlite3
import time
from pathlib import Path
from typing import Iterable, Sequence

from ..types import Det

_SCHEMA_VERSION = 1
_SCHEMA_FILE = Path(__file__).resolve().parent / "schema.sql"


def _dets_to_json(dets: Iterable) -> str:
    """Serialise a list[Det] to compact JSON."""
    return json.dumps(
        [{"cls": d.cls, "score": d.score, "x": d.x, "y": d.y, "w": d.w, "h": d.h}
         for d in dets],
        separators=(",", ":"),
    )


def _json_to_dets(s: str) -> list:
    arr = json.loads(s)
    return [Det(cls=int(o["cls"]), score=float(o["score"]),
                x=float(o["x"]), y=float(o["y"]), w=float(o["w"]), h=float(o["h"]))
            for o in arr]


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

    def put_many(self, rows: Sequence[dict]) -> None:
        """Insert many rows in a single transaction."""
        if not rows:
            return
        now = time.time()
        prepared: list[tuple] = []
        for r in rows:
            cr = r["crop_rect"]
            dets = r["dets"]
            prepared.append((
                int(r["frame_idx"]),
                int(cr.x), int(cr.y), int(cr.w), int(cr.h),
                int(r["ppv"]),
                _dets_to_json(dets),
                float(r.get("ts_epoch", now)),
            ))
        try:
            self._con.execute("BEGIN")
            # INSERT OR IGNORE: warming may re-record the same
            # (frame_idx, crop, ppv) key across overlapping grids / re-runs.
            # First-writer-wins; identical content per key, so this is
            # semantics-preserving and makes warming idempotent (Plan 6 A1).
            self._con.executemany(
                "INSERT OR IGNORE INTO detections "
                "(frame_idx, crop_x, crop_y, crop_w, crop_h, ppv, dets_json, ts_epoch) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                prepared,
            )
            self._con.execute("COMMIT")
        except Exception:
            self._con.execute("ROLLBACK")
            raise

    def get(self, frame_idx: int, crop_rect, *, ppv: int) -> "list | None":
        row = self._con.execute(
            "SELECT dets_json FROM detections WHERE "
            "frame_idx=? AND crop_x=? AND crop_y=? AND crop_w=? AND crop_h=? AND ppv=?",
            (frame_idx, crop_rect.x, crop_rect.y, crop_rect.w, crop_rect.h, ppv),
        ).fetchone()
        if row is None:
            return None
        return _json_to_dets(row[0])

    def get_many(self, frame_idx: int, crop_rects: Sequence, *, ppv: int) -> "list[list | None]":
        """Return `[dets_or_None, ...]` matching `crop_rects` in input order."""
        out: list = []
        stmt = (
            "SELECT dets_json FROM detections WHERE "
            "frame_idx=? AND crop_x=? AND crop_y=? AND crop_w=? AND crop_h=? AND ppv=?"
        )
        for c in crop_rects:
            row = self._con.execute(
                stmt, (frame_idx, c.x, c.y, c.w, c.h, ppv),
            ).fetchone()
            out.append(_json_to_dets(row[0]) if row else None)
        return out

    def stats(self) -> dict:
        n = self._con.execute("SELECT COUNT(*) FROM detections").fetchone()[0]
        uv = self._con.execute("PRAGMA user_version").fetchone()[0]
        return {
            "path": str(self.path),
            "n_rows": int(n),
            "schema_version": int(uv),
        }

    def vacuum(self) -> None:
        """Reclaim free pages. Holds exclusive lock briefly."""
        self._con.execute("VACUUM")
