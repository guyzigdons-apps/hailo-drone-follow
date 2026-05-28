# Plan 4: Cache Schema + Python Cache Layer — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Land spec phase 7 (Inference Cache) of `docs/superpowers/specs/2026-05-28-tiling-library-design.md`. After this plan, `hailo_tiling/` ships a pure-Python SQLite-backed tile cache: an `SqliteCacheStore` over the spec §7.2 WITHOUT-ROWID schema, hashing helpers, a `CachingBackend` decorator that wraps any `InferenceBackend`, a `ReplayBackend` that raises on miss (chip-free reruns), and a `hailo-tiling-warm-cache` CLI. No GStreamer plugins, no `hailo-apps-core` changes, no `HefBackend` exercise in tests — those are Plan 5. `MockBackend` from Plan 2 is the inference fake throughout.

**Architecture:**
- `SqliteCacheStore` owns the SQLite handle. Opens with WAL mode, checks `PRAGMA user_version` on first open, creates the schema on first write. One DB per `(video_sha, hef_sha)` pair (the filename encodes both). All writes go through `put_many(rows)` in a single transaction; all reads through `get(frame_idx, crop_rect, ppv)` and `get_many(...)`.
- `CachingBackend(wrapped, store, ppv)` decorates any `InferenceBackend`. On `infer(frame, crops, frame_idx)` it splits crops into cache hits (served from SQLite) and misses (forwarded to `wrapped`), wraps the miss results back into a SQLite transaction, then **merges in original input order** so downstream code is oblivious to the cache. The decorator is invisible to the scheduler — same code path with or without it.
- `ReplayBackend(store)` is the chip-free fork: a `cache hit → cached dets`, `cache miss → raise CacheMissError` backend. Used in CI and by external reviewers running the published cache without a Hailo chip.
- `hailo-tiling-warm-cache` is a standalone CLI that walks a video with a fixed crop set (e.g. `--grid 1x1 --grid 2x2 --grid 3x2`), runs inference (any registered backend; default `MockBackend` in Plan 4, swappable to `HefBackend`/`GstCropperBackend` later), and writes into the cache. Crops are deterministic functions of the video size, so the warmer's output is reproducible.
- Cache **schema is the API**: the same SQLite file produced here will be consumed by `hailonet_cache` (Plan 5) and the GStreamer recorder (Plan 5) — no FFI layer, no shared library binding. Both ends read/write the same schema.

**Tech Stack:** Python 3.10+, stdlib `sqlite3` (no new deps), pytest. The existing project venv at `./hailo-apps/venv_hailo_apps` is reused. All code is pure Python with no HailoRT / GStreamer / OpenCV imports — those land in Plan 5. Video walking in the CLI uses OpenCV (`cv2.VideoCapture`), which is already a transitive dependency via `hailo-apps`; the CLI lazy-imports it so library import stays cv2-free.

**Spec reference:** `docs/superpowers/specs/2026-05-28-tiling-library-design.md` §7 in full — especially §7.2 (schema), §7.3 (canonicalisation), §7.4 (flow), §7.5 (warming), §7.6 (replay-only), §7.7 (lifecycle), §11 phase 7.

**Branch / starting HEAD:** `tiling-benchmark` @ `ab67415` (post Plan 3; 164 tests passing).

---

## File Structure

**Files this plan creates:**

```
hailo_tiling/
  cache/
    __init__.py                       # public re-exports: SqliteCacheStore, CacheMissError, file_sha256, canonicalize_crop
    schema.sql                        # CREATE TABLE detections (WITHOUT ROWID) + CREATE TABLE meta + PRAGMA user_version
    hashing.py                        # file_sha256(path), canonicalize_crop(crop_rect, quantise=None)
    store.py                          # SqliteCacheStore: open/get/get_many/put_many/stats/vacuum
  backends/
    caching.py                        # CachingBackend(wrapped, store, ppv, quantise=None)
    replay.py                         # ReplayBackend(store); raises CacheMissError on miss
  cli/
    __init__.py                       # empty
    warm.py                           # hailo-tiling-warm-cache entry point
  tests/
    test_cache_hashing.py             # file_sha256 + canonicalize_crop unit tests
    test_cache_store.py               # SqliteCacheStore CRUD + WAL + schema-version check
    test_cache_store_concurrency.py   # one-reader-one-writer WAL behaviour
    test_backend_caching.py           # CachingBackend split-merge + transactional put
    test_backend_replay.py            # ReplayBackend hit/miss semantics + error message
    test_cli_warm.py                  # CLI argparse + end-to-end small-video warm cycle
```

**Files this plan modifies:**

- `hailo_tiling/__init__.py` — re-export the new public surface: `SqliteCacheStore`, `CacheMissError`, `CachingBackend`, `ReplayBackend`, `file_sha256`, `canonicalize_crop`.
- `hailo_tiling/backends/__init__.py` — add `CachingBackend` and `ReplayBackend` to the re-exports.
- `pyproject.toml` — register the `hailo-tiling-warm-cache` script in `[project.scripts]`.
- `docs/superpowers/plans/INDEX.md` — flip Plan 4 status from `not started` → `in flight` at the start of work; → `done` at the end.

**Files this plan does NOT touch:**

- `hailo_tiling/scheduler.py`, `hailo_tiling/emitters/`, `hailo_tiling/modifiers/`, `hailo_tiling/aggregator/`, `hailo_tiling/telemetry/`, `hailo_tiling/types.py`, `hailo_tiling/budget.py` — Plan 1 and Plan 2 surface stays untouched. The cache lives at the backend seam, below the scheduler.
- `hailo_tiling/backends/backend.py`, `hailo_tiling/backends/hef.py` — the `InferenceBackend` ABC and `HefBackend` are stable from Plan 2. `CachingBackend` and `ReplayBackend` are added as new files alongside.
- `dynamic_tiling/` — no shim work in this plan. The legacy `dynamic_tiling.inference.ReplayBackend` (single-crop, per-frame-canned) stays as-is; the new `hailo_tiling.backends.ReplayBackend` is a separate, batched, cache-keyed implementation.
- `hailo-apps-core/`, GStreamer plugins, `libhailotile_cache.so` — Plan 5.
- `tiling_benchmark/prepare_video.py:sha256_of_file` — left in place. The plan re-implements an independent `file_sha256` in `hailo_tiling/cache/hashing.py` rather than reaching across into `tiling_benchmark/`; rationale in Open Questions §1.

---

## Pre-flight: virtual environment + SQLite version check

All commands assume the project venv is active. If you've never set it up, run `source setup_env.sh` once at the start of the session. After that, use the direct binaries — `/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo-apps/venv_hailo_apps/bin/python` and `.../bin/pytest` — because shell state doesn't persist between tool calls.

For brevity in this plan, paths are written as `python` and `pytest`; the executor should resolve them to the venv binaries.

**Quick sanity check before starting:**
```bash
python -c "import hailo_tiling; print(hailo_tiling.__version__)"
pytest hailo_tiling/tests -q
python -c "import sqlite3; print(sqlite3.sqlite_version)"
```
Expected:
1. `0.1.0.dev0`
2. 164 tests passing (Plan 3 baseline).
3. SQLite ≥ 3.31 — required for `WITHOUT ROWID` + WAL behaviour the spec relies on. The dev machine reports `3.37.2`, which is fine. If a future runner reports < 3.31, the warmer should refuse to start with a clear error.

**Flip the INDEX status to `in flight` as the first action.**

```bash
sed -i 's|^| 4   | Cache schema + Python cache layer                     | 7           | not started |$| 4   | Cache schema + Python cache layer                     | 7           | in flight   |g' docs/superpowers/plans/INDEX.md
```
(or just edit the file by hand — one cell change). Commit with `plans: flip Plan 4 status to in flight in INDEX`.

---

## Task 1: Cache package scaffold + `schema.sql` + `PRAGMA user_version = 1`

**Files:**
- Create: `hailo_tiling/cache/__init__.py`
- Create: `hailo_tiling/cache/schema.sql`
- Create: `hailo_tiling/tests/test_cache_store.py` (just the schema check piece)

- [ ] **Step 1: Create the cache package directory.**

```bash
mkdir -p hailo_tiling/cache
```

- [ ] **Step 2: Write `hailo_tiling/cache/__init__.py` with a placeholder.**

```python
# hailo_tiling/cache/__init__.py
"""hailo_tiling.cache — SQLite-backed tile cache.

Same schema as `hailonet_cache` / `hailodet_record` (Plan 5); the schema is
the API — no FFI binding to the shared C library is needed.
"""
# Filled in by later tasks as the public surface lands.
__all__ = []
```

- [ ] **Step 3: Write the schema file. Exact §7.2 content.**

```sql
-- hailo_tiling/cache/schema.sql
--
-- Tile-cache schema. Same file Plan 5's libhailotile_cache.so will produce.
-- One row per (frame_idx, crop_rect, ppv); coordinates are source-pixel ints.

PRAGMA user_version = 1;
PRAGMA journal_mode = WAL;

CREATE TABLE IF NOT EXISTS detections (
    frame_idx    INTEGER NOT NULL,
    crop_x       INTEGER NOT NULL,
    crop_y       INTEGER NOT NULL,
    crop_w       INTEGER NOT NULL,
    crop_h       INTEGER NOT NULL,
    ppv          INTEGER NOT NULL,
    dets_json    TEXT    NOT NULL,
    ts_epoch     REAL    NOT NULL,
    PRIMARY KEY (frame_idx, crop_x, crop_y, crop_w, crop_h, ppv)
) WITHOUT ROWID;

CREATE TABLE IF NOT EXISTS meta (
    k TEXT PRIMARY KEY,
    v TEXT NOT NULL
);
```

The `IF NOT EXISTS` clauses are deliberate: opening an existing DB should be a no-op, not raise.

- [ ] **Step 4: Write a failing schema-load smoke test.**

```python
# hailo_tiling/tests/test_cache_store.py
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
        # user_version
        cur = con.execute("PRAGMA user_version")
        assert cur.fetchone()[0] == 1
        # tables
        names = {r[0] for r in con.execute(
            "SELECT name FROM sqlite_master WHERE type='table'"
        )}
        assert "detections" in names
        assert "meta" in names
        # detections is WITHOUT ROWID — primary key shape is composite
        info = list(con.execute("PRAGMA table_info(detections)"))
        cols = {row[1] for row in info}
        assert cols == {
            "frame_idx", "crop_x", "crop_y", "crop_w", "crop_h",
            "ppv", "dets_json", "ts_epoch",
        }
    finally:
        con.close()
```

- [ ] **Step 5: Run the test.**

```bash
pytest hailo_tiling/tests/test_cache_store.py -v
```
Expected: 1 passed.

- [ ] **Step 6: Commit.**

```bash
git add hailo_tiling/cache/__init__.py hailo_tiling/cache/schema.sql \
        hailo_tiling/tests/test_cache_store.py
git commit -m "hailo_tiling: cache schema.sql (detections WITHOUT ROWID + meta + user_version=1)"
```

---

## Task 2: `file_sha256` + `canonicalize_crop` hashing helpers

**Files:**
- Create: `hailo_tiling/cache/hashing.py`
- Create: `hailo_tiling/tests/test_cache_hashing.py`

- [ ] **Step 1: Write the failing hashing tests.**

```python
# hailo_tiling/tests/test_cache_hashing.py
"""hashing helpers — file SHA-256 + crop-rect canonicalisation."""
from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from hailo_tiling.cache.hashing import canonicalize_crop, file_sha256
from hailo_tiling.types import CropRect


def test_file_sha256_matches_hashlib(tmp_path: Path):
    p = tmp_path / "a.bin"
    payload = b"hello-cache-" * 1024
    p.write_bytes(payload)
    assert file_sha256(p) == hashlib.sha256(payload).hexdigest()


def test_file_sha256_empty_file(tmp_path: Path):
    p = tmp_path / "empty"
    p.write_bytes(b"")
    assert file_sha256(p) == hashlib.sha256(b"").hexdigest()


def test_file_sha256_streams_large_file(tmp_path: Path):
    """Larger than the 1 MiB chunk — verify chunked read is correct."""
    p = tmp_path / "big.bin"
    p.write_bytes(b"x" * (3 * 1024 * 1024 + 17))
    expected = hashlib.sha256(p.read_bytes()).hexdigest()
    assert file_sha256(p) == expected


def test_canonicalize_crop_no_quantise_passes_through():
    r = CropRect(x=123, y=456, w=789, h=321, mode="s")
    assert canonicalize_crop(r) == (123, 456, 789, 321)


def test_canonicalize_crop_quantise_4_rounds_down_to_multiple():
    r = CropRect(x=123, y=457, w=790, h=322, mode="s")
    # quantise=4 → x=120, y=456, w=788, h=320
    assert canonicalize_crop(r, quantise=4) == (120, 456, 788, 320)


def test_canonicalize_crop_quantise_none_equivalent_to_default():
    r = CropRect(x=100, y=100, w=640, h=480, mode="s")
    assert canonicalize_crop(r, quantise=None) == canonicalize_crop(r)


def test_canonicalize_crop_quantise_one_is_identity():
    r = CropRect(x=7, y=13, w=11, h=5, mode="s")
    assert canonicalize_crop(r, quantise=1) == (7, 13, 11, 5)
```

- [ ] **Step 2: Run the test, see failures (import error).**

```bash
pytest hailo_tiling/tests/test_cache_hashing.py -v
```
Expected: FAIL — `ImportError: cannot import name 'canonicalize_crop' from 'hailo_tiling.cache.hashing'`.

- [ ] **Step 3: Implement the hashing module.**

```python
# hailo_tiling/cache/hashing.py
"""Hashing helpers for the tile cache.

`file_sha256(path)` — streams a file in 1 MiB chunks and returns the SHA-256
hex digest. Used for `video_sha` / `hef_sha` in the cache filename and `meta`
table.

`canonicalize_crop(crop_rect, quantise=None)` — returns the 4-tuple cache key
`(x, y, w, h)`. If `quantise` is set, each component is rounded **down** to a
multiple of `quantise` px. Quantisation is spec §7.3's "slightly fuzzy" mode;
the unquantised default (`quantise=None` or `quantise=1`) is paper-correct.

Note: this duplicates `tiling_benchmark/prepare_video.py:sha256_of_file` by
design. The cache layer is a published library surface; we don't want it to
depend on tiling_benchmark internals. See the plan's Open Questions §1.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

from ..types import CropRect

_SHA_CHUNK = 1024 * 1024  # 1 MiB


def file_sha256(path: str | Path) -> str:
    """Return the SHA-256 hex digest of `path`'s bytes (chunked read)."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(_SHA_CHUNK), b""):
            h.update(chunk)
    return h.hexdigest()


def canonicalize_crop(
    crop_rect: CropRect,
    quantise: int | None = None,
) -> tuple[int, int, int, int]:
    """Return the cache key `(x, y, w, h)`.

    If `quantise` is set and > 1, each component is rounded **down** to a
    multiple of `quantise`. This increases cache hit rate when float→int
    rounding in an emitter is unstable across runs at the cost of slightly
    wrong (typically 1–3 px) crop coordinates. The default behaviour
    (`quantise=None` or `quantise=1`) is identity.
    """
    if quantise is None or quantise <= 1:
        return (crop_rect.x, crop_rect.y, crop_rect.w, crop_rect.h)
    q = quantise
    return (
        (crop_rect.x // q) * q,
        (crop_rect.y // q) * q,
        (crop_rect.w // q) * q,
        (crop_rect.h // q) * q,
    )
```

- [ ] **Step 4: Run the tests, expect pass.**

```bash
pytest hailo_tiling/tests/test_cache_hashing.py -v
```
Expected: 7 passed.

- [ ] **Step 5: Commit.**

```bash
git add hailo_tiling/cache/hashing.py hailo_tiling/tests/test_cache_hashing.py
git commit -m "hailo_tiling: cache.hashing — file_sha256 + canonicalize_crop"
```

---

## Task 3: `SqliteCacheStore` open + schema bootstrap + `meta` accessors

This task lands the constructor, the `open()` classmethod, schema bootstrap, and `meta_get`/`meta_put` accessors. CRUD on `detections` lands in Task 4.

**Files:**
- Create: `hailo_tiling/cache/store.py`
- Modify: `hailo_tiling/tests/test_cache_store.py` (append open + meta tests)
- Modify: `hailo_tiling/cache/__init__.py` (export `SqliteCacheStore`)

- [ ] **Step 1: Append failing tests to `test_cache_store.py`.**

```python
# (append to hailo_tiling/tests/test_cache_store.py)
from hailo_tiling.cache.store import SqliteCacheStore


def test_store_open_creates_file_and_schema(tmp_path):
    db = tmp_path / "v__h.sqlite3"
    store = SqliteCacheStore.open(db)
    try:
        # The DB file exists.
        assert db.is_file()
        # user_version is 1.
        cur = store._con.execute("PRAGMA user_version")
        assert cur.fetchone()[0] == 1
        # WAL is on (journal_mode returns 'wal' once it sticks).
        jm = store._con.execute("PRAGMA journal_mode").fetchone()[0].lower()
        assert jm == "wal"
    finally:
        store.close()


def test_store_open_existing_is_idempotent(tmp_path):
    db = tmp_path / "v__h.sqlite3"
    s1 = SqliteCacheStore.open(db)
    s1.close()
    # Re-open the same file; must not raise.
    s2 = SqliteCacheStore.open(db)
    assert s2 is not None
    s2.close()


def test_store_rejects_mismatched_schema_version(tmp_path):
    db = tmp_path / "old.sqlite3"
    # Create a DB with user_version=999 — simulating a newer schema.
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
```

- [ ] **Step 2: Implement the store skeleton.**

```python
# hailo_tiling/cache/store.py
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

    # ------------------------------------------------------------ lifecycle

    @classmethod
    def open(cls, path: str | Path) -> "SqliteCacheStore":
        """Open an existing cache or create a new one at `path`.

        Verifies `PRAGMA user_version` matches `_SCHEMA_VERSION` on an
        existing file; raises `ValueError` on mismatch (no auto-migrate).
        """
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        existed = path.exists()
        con = sqlite3.connect(path, isolation_level=None)  # autocommit; we BEGIN explicitly
        # WAL must be persistent — set early.
        con.execute("PRAGMA journal_mode = WAL")
        con.execute("PRAGMA synchronous = NORMAL")  # WAL pairs well with NORMAL
        if existed:
            uv = con.execute("PRAGMA user_version").fetchone()[0]
            if uv == 0:
                # Empty/initialised-but-no-schema file. Treat as fresh.
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

    # ---------------------------------------------------------------- meta

    def meta_get(self, k: str) -> str | None:
        row = self._con.execute("SELECT v FROM meta WHERE k = ?", (k,)).fetchone()
        return row[0] if row else None

    def meta_put(self, k: str, v: str) -> None:
        self._con.execute(
            "INSERT INTO meta (k, v) VALUES (?, ?) "
            "ON CONFLICT(k) DO UPDATE SET v = excluded.v",
            (k, str(v)),
        )

    # --------------------------------------------------------- detections
    # (Task 4 adds get / get_many / put_many / stats / vacuum)
```

- [ ] **Step 3: Add `SqliteCacheStore` to the cache `__init__.py`.**

```python
# hailo_tiling/cache/__init__.py
"""hailo_tiling.cache — SQLite-backed tile cache."""
from .hashing import canonicalize_crop, file_sha256
from .store import SqliteCacheStore

__all__ = ["SqliteCacheStore", "canonicalize_crop", "file_sha256"]
```

- [ ] **Step 4: Run the tests, expect pass.**

```bash
pytest hailo_tiling/tests/test_cache_store.py -v
```
Expected: 6 passed (1 from Task 1 + 5 new).

- [ ] **Step 5: Commit.**

```bash
git add hailo_tiling/cache/store.py hailo_tiling/cache/__init__.py \
        hailo_tiling/tests/test_cache_store.py
git commit -m "hailo_tiling: SqliteCacheStore.open + meta_get/meta_put + WAL bootstrap"
```

---

## Task 4: `SqliteCacheStore` — `put_many` / `get` / `get_many` / `stats` / `vacuum`

**Files:**
- Modify: `hailo_tiling/cache/store.py` (extend the class)
- Modify: `hailo_tiling/tests/test_cache_store.py` (append CRUD tests)

- [ ] **Step 1: Append failing CRUD tests.**

```python
# (append to hailo_tiling/tests/test_cache_store.py)
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


def test_put_many_is_one_transaction(tmp_path, monkeypatch):
    """A failing row inside put_many rolls back ALL rows in that call."""
    store = SqliteCacheStore.open(tmp_path / "tx.sqlite3")
    try:
        c_good = CropRect(x=0, y=0, w=640, h=480, mode="m")
        # Force a constraint violation by reusing the same key twice in one call.
        rows = [
            _row(0, c_good, 1, [_det(0, 0.9, 0, 0, 0.1, 0.1)]),
            _row(0, c_good, 1, [_det(0, 0.1, 0, 0, 0.1, 0.1)]),  # duplicate PK
        ]
        with pytest.raises(sqlite3.IntegrityError):
            store.put_many(rows)
        # First row must NOT be present — the transaction rolled back.
        assert store.get(0, c_good, ppv=1) is None
    finally:
        store.close()


def test_put_many_empty_is_noop(tmp_path):
    store = SqliteCacheStore.open(tmp_path / "noop.sqlite3")
    try:
        store.put_many([])  # must not raise
        assert store.stats()["n_rows"] == 0
    finally:
        store.close()


def test_get_many_preserves_order(tmp_path):
    store = SqliteCacheStore.open(tmp_path / "om.sqlite3")
    try:
        crops = [
            CropRect(x=i * 100, y=0, w=640, h=480, mode="m") for i in range(5)
        ]
        store.put_many([
            _row(0, c, 1, [_det(0, 0.5 + 0.05 * i, 0, 0, 0.1, 0.1)])
            for i, c in enumerate(crops)
        ])
        # Query in reverse order; result must come back in the queried order.
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
        assert out[1] is None  # miss in the middle is None


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
    """vacuum() must complete cleanly on a populated DB."""
    store = SqliteCacheStore.open(tmp_path / "vac.sqlite3")
    try:
        c = CropRect(x=0, y=0, w=640, h=480, mode="m")
        store.put_many([_row(0, c, 1, [])])
        store.vacuum()  # smoke test — must not raise
        assert store.get(0, c, ppv=1) == []
    finally:
        store.close()


def test_quantise_is_applied_at_put_time_when_enabled(tmp_path):
    """If callers pass an already-canonicalised key, the row keeps it verbatim.

    Quantisation is the responsibility of `CachingBackend` (Task 6); the store
    is canonicalisation-agnostic. This test merely asserts that put_many does
    not re-canonicalise behind our back.
    """
    store = SqliteCacheStore.open(tmp_path / "q.sqlite3")
    try:
        c = CropRect(x=123, y=457, w=789, h=321, mode="m")
        store.put_many([_row(0, c, 1, [])])
        # Hit with the SAME values.
        assert store.get(0, c, ppv=1) == []
        # Hit with a slightly different crop must miss.
        c2 = CropRect(x=120, y=456, w=788, h=320, mode="m")
        assert store.get(0, c2, ppv=1) is None
    finally:
        store.close()
```

- [ ] **Step 2: Extend `SqliteCacheStore` with the CRUD methods.**

Append to `hailo_tiling/cache/store.py`:

```python
# (append to hailo_tiling/cache/store.py; methods on SqliteCacheStore)

    # --------------------------------------------------------- detections

    def put_many(self, rows: Sequence[dict]) -> None:
        """Insert many rows in a single transaction.

        Each row is a dict with keys:
          - frame_idx: int
          - crop_rect: CropRect
          - ppv: int
          - dets: list[Det]
        """
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
            self._con.executemany(
                "INSERT INTO detections "
                "(frame_idx, crop_x, crop_y, crop_w, crop_h, ppv, dets_json, ts_epoch) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                prepared,
            )
            self._con.execute("COMMIT")
        except Exception:
            self._con.execute("ROLLBACK")
            raise

    def get(
        self,
        frame_idx: int,
        crop_rect,  # CropRect
        *,
        ppv: int,
    ) -> "list | None":
        row = self._con.execute(
            "SELECT dets_json FROM detections WHERE "
            "frame_idx=? AND crop_x=? AND crop_y=? AND crop_w=? AND crop_h=? AND ppv=?",
            (frame_idx, crop_rect.x, crop_rect.y, crop_rect.w, crop_rect.h, ppv),
        ).fetchone()
        if row is None:
            return None
        return _json_to_dets(row[0])

    def get_many(
        self,
        frame_idx: int,
        crop_rects: Sequence,
        *,
        ppv: int,
    ) -> "list[list | None]":
        """Return `[dets_or_None, ...]` matching `crop_rects` in input order."""
        out: list = []
        # Single prepared statement, one query per crop. The composite PK is
        # an index; each lookup is ~10–100 µs (see spec §7.9).
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
        """Reclaim free pages after large deletions. Holds an exclusive lock."""
        # VACUUM cannot run inside a transaction. autocommit (isolation_level=None)
        # already prevents implicit transactions; this is safe.
        self._con.execute("VACUUM")
```

Add the JSON helpers near the top of the file (after the schema constants, before the class):

```python
# (insert near the top of hailo_tiling/cache/store.py)
import json
from ..types import Det


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
```

(The `from typing import Iterable` import already exists in `store.py` from Task 3; if not, add it.)

- [ ] **Step 3: Run the tests, expect pass.**

```bash
pytest hailo_tiling/tests/test_cache_store.py -v
```
Expected: 14 passed.

- [ ] **Step 4: Commit.**

```bash
git add hailo_tiling/cache/store.py hailo_tiling/tests/test_cache_store.py
git commit -m "hailo_tiling: SqliteCacheStore CRUD — put_many/get/get_many/stats/vacuum"
```

---

## Task 5: WAL concurrency smoke test (1 reader + 1 writer)

A tiny test that asserts the documented WAL behaviour from spec §7.7 / §7.9: while one connection holds an open transaction writing rows, a second connection can still read previously-committed rows. Documents the lock-contention story called out in Open Question §3.

**Files:**
- Create: `hailo_tiling/tests/test_cache_store_concurrency.py`

- [ ] **Step 1: Write the test.**

```python
# hailo_tiling/tests/test_cache_store_concurrency.py
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

        # Open a separate reader connection. WAL allows this without blocking.
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
```

- [ ] **Step 2: Run the tests, expect pass.**

```bash
pytest hailo_tiling/tests/test_cache_store_concurrency.py -v
```
Expected: 2 passed.

- [ ] **Step 3: Commit.**

```bash
git add hailo_tiling/tests/test_cache_store_concurrency.py
git commit -m "hailo_tiling: WAL concurrency smoke (reader + writer; reopen persistence)"
```

---

## Task 6: `CachingBackend` — split-merge + transactional put

**Files:**
- Create: `hailo_tiling/backends/caching.py`
- Create: `hailo_tiling/tests/test_backend_caching.py`
- Modify: `hailo_tiling/backends/__init__.py` (add `CachingBackend` export)

- [ ] **Step 1: Write the failing test suite.**

```python
# hailo_tiling/tests/test_backend_caching.py
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
    assert mock.call_count == 1  # one batched call
    assert mock.calls[0]["crops"] == crops  # all forwarded (all miss)

    # Subsequent call must serve from cache: mock not called again.
    out2 = cb.infer(frame=None, crops=crops, frame_idx=0)
    assert out2 == out
    assert mock.call_count == 1  # unchanged
    store.close()


def test_partial_hit_forwards_only_misses(tmp_path):
    store = _store(tmp_path)
    crops = _crops(4)
    canned = {(0, (c.x, c.y, c.w, c.h)): [_det(0.1 * i)] for i, c in enumerate(crops)}
    mock = MockBackend(canned=canned)
    cb = CachingBackend(wrapped=mock, store=store, ppv=1)

    # Pre-warm crops 0 and 2.
    store.put_many([
        {"frame_idx": 0, "crop_rect": crops[0], "ppv": 1, "dets": [_det(0.99)]},
        {"frame_idx": 0, "crop_rect": crops[2], "ppv": 1, "dets": [_det(0.88)]},
    ])

    out = cb.infer(frame=None, crops=crops, frame_idx=0)
    # Misses (1, 3) forwarded; hits (0, 2) served from cache.
    assert mock.call_count == 1
    assert mock.calls[0]["crops"] == [crops[1], crops[3]]
    # Order is preserved: out[i] matches crops[i].
    assert out[0][0].score == pytest.approx(0.99)
    assert out[2][0].score == pytest.approx(0.88)
    # And the wrapped results are at positions 1 and 3.
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
    """One CachingBackend.infer() == one BEGIN/COMMIT on the store."""
    store = _store(tmp_path)
    crops = _crops(5)
    canned = {(0, (c.x, c.y, c.w, c.h)): [_det()] for c in crops}
    mock = MockBackend(canned=canned)
    cb = CachingBackend(wrapped=mock, store=store, ppv=1)

    seen_calls = []
    original_execute = store._con.execute

    def _spy(sql, *a, **kw):
        seen_calls.append(sql.strip().split()[0].upper())
        return original_execute(sql, *a, **kw)

    monkeypatch.setattr(store._con, "execute", _spy)

    cb.infer(frame=None, crops=crops, frame_idx=0)
    # Count the BEGIN / COMMIT pairs from put_many's transaction.
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
    """If the input has the SAME crop twice, the second resolves from the
    in-flight result of the first — we don't call wrapped() twice and we
    don't violate the PK on store."""
    store = _store(tmp_path)
    c = CropRect(x=0, y=0, w=640, h=480, mode="m")
    canned = {(0, (c.x, c.y, c.w, c.h)): [_det(0.7)]}
    mock = MockBackend(canned=canned)
    cb = CachingBackend(wrapped=mock, store=store, ppv=1)
    out = cb.infer(frame=None, crops=[c, c, c], frame_idx=0)
    assert mock.call_count == 1
    # The wrapped MockBackend received de-duplicated crops.
    assert mock.calls[0]["crops"] == [c]
    # The output is broadcast back to all three input positions.
    assert len(out) == 3
    assert out[0] == out[1] == out[2]
    store.close()


def test_ppv_isolation(tmp_path):
    """Same crop, different ppv → different cache entries."""
    store = _store(tmp_path)
    c = CropRect(x=0, y=0, w=640, h=480, mode="m")
    canned = {(0, (c.x, c.y, c.w, c.h)): [_det(0.5)]}
    mock = MockBackend(canned=canned)
    cb1 = CachingBackend(wrapped=mock, store=store, ppv=1)
    cb2 = CachingBackend(wrapped=mock, store=store, ppv=2)
    cb1.infer(frame=None, crops=[c], frame_idx=0)
    # ppv=2 must NOT see the ppv=1 row.
    assert mock.call_count == 1
    cb2.infer(frame=None, crops=[c], frame_idx=0)
    assert mock.call_count == 2
    store.close()


def test_canonicalize_respected_when_quantise_set(tmp_path):
    """quantise=4 means slightly-off crops still hit. Documented behaviour."""
    store = _store(tmp_path)
    c_orig = CropRect(x=100, y=100, w=640, h=480, mode="m")
    c_off = CropRect(x=101, y=99, w=641, h=481, mode="m")  # 1 px off in all
    canned = {(0, (c_orig.x, c_orig.y, c_orig.w, c_orig.h)): [_det(0.7)]}
    mock = MockBackend(canned=canned)
    cb = CachingBackend(wrapped=mock, store=store, ppv=1, quantise=4)
    cb.infer(frame=None, crops=[c_orig], frame_idx=0)
    out = cb.infer(frame=None, crops=[c_off], frame_idx=0)
    # quantise=4 rounds both to (100, 96, 640, 480) — same cache key.
    assert mock.call_count == 1  # the off crop did NOT forward to wrapped
    assert out[0][0].score == pytest.approx(0.7)
    store.close()
```

- [ ] **Step 2: Implement `CachingBackend`.**

```python
# hailo_tiling/backends/caching.py
"""CachingBackend — decorator that serves repeated `(frame_idx, crop)` lookups
from an SQLite cache.

Wrap any `InferenceBackend`; the cache is invisible to the scheduler and the
aggregator. On `infer(frame, crops, frame_idx)`:

  1. Canonicalise each crop (optional quantise).
  2. Split `crops` into hits (served from `store`) and misses (forwarded).
  3. Forward misses to the wrapped backend as ONE batched call.
  4. Persist new (crop, dets) rows in ONE transaction.
  5. Merge hits + new results back into the original input order.

Duplicate crops in the input collapse to one wrapped call; the result is
broadcast back to every input position.
"""
from __future__ import annotations

from typing import Any, Sequence

from ..cache.hashing import canonicalize_crop
from ..cache.store import SqliteCacheStore
from ..types import CropRect, Det
from .backend import InferenceBackend


class CachingBackend(InferenceBackend):
    def __init__(
        self,
        wrapped: InferenceBackend,
        store: SqliteCacheStore,
        ppv: int,
        quantise: int | None = None,
    ):
        self._wrapped = wrapped
        self._store = store
        self._ppv = int(ppv)
        self._quantise = quantise

    def _key(self, c: CropRect) -> tuple[int, int, int, int]:
        return canonicalize_crop(c, quantise=self._quantise)

    def _canon_crop(self, c: CropRect) -> CropRect:
        if self._quantise is None or self._quantise <= 1:
            return c
        x, y, w, h = self._key(c)
        return CropRect(x=x, y=y, w=w, h=h, mode=c.mode)

    def infer(
        self,
        frame: Any,
        crops: Sequence[CropRect],
        frame_idx: int,
    ) -> list[list[Det]]:
        if not crops:
            return []

        # Phase 1 — canonicalise; build unique-crop bucket preserving first-seen order.
        canon = [self._canon_crop(c) for c in crops]
        keys = [self._key(c) for c in canon]

        unique_keys: list[tuple] = []
        unique_crops: list[CropRect] = []
        key_to_unique_idx: dict[tuple, int] = {}
        for k, c in zip(keys, canon):
            if k not in key_to_unique_idx:
                key_to_unique_idx[k] = len(unique_keys)
                unique_keys.append(k)
                unique_crops.append(c)

        # Phase 2 — cache lookup against the unique set.
        cached = self._store.get_many(frame_idx, unique_crops, ppv=self._ppv)
        miss_indices = [i for i, d in enumerate(cached) if d is None]
        miss_crops = [unique_crops[i] for i in miss_indices]

        # Phase 3 — forward misses (one batched call).
        if miss_crops:
            new_results = self._wrapped.infer(frame, miss_crops, frame_idx)
            assert len(new_results) == len(miss_crops), (
                "wrapped backend returned wrong number of det-lists"
            )
            # Phase 4 — persist new rows in one transaction.
            self._store.put_many([
                {
                    "frame_idx": frame_idx,
                    "crop_rect": miss_crops[i],
                    "ppv": self._ppv,
                    "dets": new_results[i],
                }
                for i in range(len(miss_crops))
            ])
            # Patch the cached array with the fresh results.
            for slot, dets in zip(miss_indices, new_results):
                cached[slot] = dets

        # Phase 5 — broadcast back to the original input order.
        return [cached[key_to_unique_idx[k]] for k in keys]

    def close(self) -> None:
        self._wrapped.close()
        # The store outlives this backend in typical usage; do not close it here.
```

- [ ] **Step 3: Add `CachingBackend` to the backends re-exports.**

```python
# hailo_tiling/backends/__init__.py
"""Inference backends — the seam between scheduler policy and execution mechanism."""
from .backend import InferenceBackend, MockBackend  # noqa: F401
from .caching import CachingBackend  # noqa: F401
from .hef import HefBackend  # noqa: F401
```

- [ ] **Step 4: Run the test suite.**

```bash
pytest hailo_tiling/tests/test_backend_caching.py -v
```
Expected: 8 passed.

- [ ] **Step 5: Commit.**

```bash
git add hailo_tiling/backends/caching.py hailo_tiling/backends/__init__.py \
        hailo_tiling/tests/test_backend_caching.py
git commit -m "hailo_tiling: CachingBackend (decorator; split-merge; one-tx put)"
```

---

## Task 7: `ReplayBackend` + `CacheMissError`

**Files:**
- Create: `hailo_tiling/backends/replay.py`
- Create: `hailo_tiling/tests/test_backend_replay.py`
- Modify: `hailo_tiling/backends/__init__.py` (add `ReplayBackend`, `CacheMissError`)
- Modify: `hailo_tiling/cache/__init__.py` (add `CacheMissError`)

- [ ] **Step 1: Write the failing test.**

```python
# hailo_tiling/tests/test_backend_replay.py
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
```

- [ ] **Step 2: Implement `ReplayBackend` and `CacheMissError`.**

```python
# hailo_tiling/backends/replay.py
"""ReplayBackend — chip-free cache reader.

For chip-free reruns over a published cache (CI, external reviewers). A
cache miss is a loud error — the spec (§7.6) is explicit: "Mixing live
inference into a replay path would muddy paper results; if the cache is
incomplete, the user re-runs warming."
"""
from __future__ import annotations

from typing import Any, Sequence

from ..cache.store import SqliteCacheStore
from ..types import CropRect, Det
from .backend import InferenceBackend


class CacheMissError(LookupError):
    """Raised by ReplayBackend when a requested crop is not in the cache."""


class ReplayBackend(InferenceBackend):
    def __init__(self, store: SqliteCacheStore, ppv: int = 1):
        self._store = store
        self._ppv = int(ppv)

    def infer(
        self,
        frame: Any,
        crops: Sequence[CropRect],
        frame_idx: int,
    ) -> list[list[Det]]:
        if not crops:
            return []
        results = self._store.get_many(frame_idx, list(crops), ppv=self._ppv)
        for c, dets in zip(crops, results):
            if dets is None:
                raise CacheMissError(
                    f"cache miss: frame_idx={frame_idx} "
                    f"crop=({c.x},{c.y},{c.w},{c.h}) ppv={self._ppv}. "
                    f"Re-warm the cache or use a live backend."
                )
        return results  # type: ignore[return-value]

    def close(self) -> None:
        # store lifetime is owned by the caller.
        return None
```

- [ ] **Step 3: Wire `CacheMissError` into the cache `__init__.py` and backends `__init__.py`.**

```python
# hailo_tiling/cache/__init__.py
"""hailo_tiling.cache — SQLite-backed tile cache."""
from ..backends.replay import CacheMissError  # noqa: F401 (re-export for convenience)
from .hashing import canonicalize_crop, file_sha256
from .store import SqliteCacheStore

__all__ = [
    "CacheMissError",
    "SqliteCacheStore",
    "canonicalize_crop",
    "file_sha256",
]
```

```python
# hailo_tiling/backends/__init__.py
"""Inference backends — the seam between scheduler policy and execution mechanism."""
from .backend import InferenceBackend, MockBackend  # noqa: F401
from .caching import CachingBackend  # noqa: F401
from .hef import HefBackend  # noqa: F401
from .replay import CacheMissError, ReplayBackend  # noqa: F401
```

- [ ] **Step 4: Run the test suite.**

```bash
pytest hailo_tiling/tests/test_backend_replay.py -v
```
Expected: 5 passed.

- [ ] **Step 5: Commit.**

```bash
git add hailo_tiling/backends/replay.py hailo_tiling/backends/__init__.py \
        hailo_tiling/cache/__init__.py hailo_tiling/tests/test_backend_replay.py
git commit -m "hailo_tiling: ReplayBackend + CacheMissError (chip-free reruns)"
```

---

## Task 8: `hailo-tiling-warm-cache` CLI

**Files:**
- Create: `hailo_tiling/cli/__init__.py`
- Create: `hailo_tiling/cli/warm.py`
- Create: `hailo_tiling/tests/test_cli_warm.py`

- [ ] **Step 1: Stub the CLI package.**

```python
# hailo_tiling/cli/__init__.py
"""hailo_tiling.cli — command-line entry points."""
```

- [ ] **Step 2: Write the failing CLI tests.**

```python
# hailo_tiling/tests/test_cli_warm.py
"""hailo-tiling-warm-cache CLI — end-to-end on a tiny canned video."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from hailo_tiling.backends import MockBackend
from hailo_tiling.cache.store import SqliteCacheStore
from hailo_tiling.cli import warm
from hailo_tiling.types import CropRect, Det


def _det(score: float = 0.9) -> Det:
    return Det(cls=0, score=score, x=0.2, y=0.3, w=0.1, h=0.2)


def test_parse_grids_accepts_NxM():
    assert warm.parse_grids(["1x1", "2x2", "3x2"]) == [(1, 1), (2, 2), (3, 2)]


def test_parse_grids_rejects_garbage():
    with pytest.raises(SystemExit):
        warm.parse_grids(["1"])
    with pytest.raises(SystemExit):
        warm.parse_grids(["axb"])


def test_grid_crops_for_dims_matches_discovery_emitter():
    """The CLI's crop generator and DiscoveryGridEmitter must agree on a fixed (W, H)."""
    from hailo_tiling.emitters import DiscoveryGridEmitter
    from hailo_tiling.budget import BudgetMeter
    from hailo_tiling.types import LockState

    crops = warm.crops_for_grids(src_w=3840, src_h=2160, grids=[(3, 2)])
    e = DiscoveryGridEmitter(grid=(3, 2), period=1, mode="m")
    expected = e.emit(3840, 2160, LockState(), frame_idx=0,
                       meter=BudgetMeter(budget_inf_per_s=1e9, fps=30))
    assert crops == expected


def test_warm_video_writes_one_row_per_crop_per_frame(tmp_path):
    """Drive `warm_video()` with a fake VideoFrames + a MockBackend; assert
    cache rows = n_frames * n_crops."""
    db = tmp_path / "w.sqlite3"
    store = SqliteCacheStore.open(db)
    crops = warm.crops_for_grids(src_w=3840, src_h=2160, grids=[(2, 1)])  # 2 crops
    # Three frames of black 4K pixels (we don't actually decode; the backend
    # is a mock).
    frames = [(i, None) for i in range(3)]
    backend = MockBackend(canned={
        (i, (c.x, c.y, c.w, c.h)): [_det()] for i in range(3) for c in crops
    })
    warm.warm_video(frames=frames, crops=crops, backend=backend,
                     store=store, ppv=1, batch_rows=8)
    assert store.stats()["n_rows"] == 3 * len(crops)
    store.close()


def test_warm_video_skips_already_cached(tmp_path):
    """Second pass over the same (frame, crop, ppv) tuples does not re-infer."""
    db = tmp_path / "w2.sqlite3"
    store = SqliteCacheStore.open(db)
    crops = warm.crops_for_grids(src_w=3840, src_h=2160, grids=[(2, 1)])
    canned = {(i, (c.x, c.y, c.w, c.h)): [_det()] for i in range(2) for c in crops}
    backend = MockBackend(canned=canned)
    frames = [(i, None) for i in range(2)]
    warm.warm_video(frames=frames, crops=crops, backend=backend,
                     store=store, ppv=1, batch_rows=8)
    n_calls_first = backend.call_count
    warm.warm_video(frames=frames, crops=crops, backend=backend,
                     store=store, ppv=1, batch_rows=8)
    # All hits → no new wrapped calls.
    assert backend.call_count == n_calls_first
    store.close()


def test_main_argparse_round_trip(tmp_path, monkeypatch):
    """`hailo-tiling-warm-cache --help` must exit cleanly."""
    with pytest.raises(SystemExit) as exc:
        warm.main(["--help"])
    assert exc.value.code == 0


def test_main_writes_meta_table(tmp_path, monkeypatch):
    """End-to-end CLI invocation populates the meta table with video_sha256
    and hef_sha256 (computed from the input files), even when the backend is
    mocked."""
    # Stand up a tiny fake video file and fake HEF file.
    vid = tmp_path / "v.mp4"
    vid.write_bytes(b"\x00fake-mp4\x00" * 1024)
    hef = tmp_path / "y.hef"
    hef.write_bytes(b"\x00fake-hef\x00" * 1024)
    cache_dir = tmp_path / ".cache"

    # Inject a deterministic frame-walker so we don't import cv2.
    def fake_frames(path):
        return [(0, None), (1, None)]

    monkeypatch.setattr(warm, "iter_video_frames", fake_frames)
    # And a deterministic backend factory.
    monkeypatch.setattr(
        warm, "make_backend",
        lambda kind, **kw: MockBackend(canned={
            (i, (c.x, c.y, c.w, c.h)): [_det()]
            for i in range(2)
            for c in warm.crops_for_grids(3840, 2160, [(1, 1)])
        }),
    )

    rc = warm.main([
        "--video", str(vid),
        "--hef", str(hef),
        "--src-w", "3840", "--src-h", "2160",
        "--grid", "1x1",
        "--cache-dir", str(cache_dir),
        "--ppv", "1",
        "--backend", "mock",
    ])
    assert rc == 0
    # The CLI writes to `<cache-dir>/<video_sha[:16]>__<hef_sha[:16]>.sqlite3`.
    matches = list(cache_dir.glob("*.sqlite3"))
    assert len(matches) == 1
    store = SqliteCacheStore.open(matches[0])
    try:
        assert store.meta_get("video_sha256") is not None
        assert store.meta_get("hef_sha256") is not None
        assert store.meta_get("ppv") == "1"
        assert int(store.meta_get("video_w") or 0) == 3840
        assert int(store.meta_get("video_h") or 0) == 2160
        assert store.stats()["n_rows"] == 2 * 1  # 2 frames × 1 crop
    finally:
        store.close()
```

- [ ] **Step 3: Implement `warm.py`.**

```python
# hailo_tiling/cli/warm.py
"""hailo-tiling-warm-cache — walk a video and pre-compute crops to the cache.

Walks `--video` frame by frame, builds a fixed crop set from `--grid` flags
(each `--grid NxM` adds an N×M discovery grid), runs inference through the
selected backend, and writes the results into a `(video_sha, hef_sha)`-keyed
SQLite file in `--cache-dir`.

The backend defaults to `mock` (a `MockBackend` that returns empty det-lists
unless canned) so the CLI is unit-testable without HailoRT. Plan 5 will add
`--backend hef` (live HefBackend) and `--backend gst` (GstCropperBackend)
options.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Iterable, Sequence

from ..backends import CachingBackend, MockBackend
from ..backends.backend import InferenceBackend
from ..budget import BudgetMeter
from ..cache.hashing import file_sha256
from ..cache.store import SqliteCacheStore
from ..emitters import DiscoveryGridEmitter
from ..types import CropRect, LockState


# ----------------------------------------------------------- argument parse


def _build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="hailo-tiling-warm-cache",
        description="Pre-compute per-crop detections to an SQLite tile cache.",
    )
    p.add_argument("--video", required=True, type=Path,
                   help="Input video (mp4 / mkv / any cv2.VideoCapture-readable).")
    p.add_argument("--hef", required=True, type=Path,
                   help="HEF file; SHA-256 is part of the cache filename.")
    p.add_argument("--src-w", type=int, default=None,
                   help="Override video width; default: probed from cv2.")
    p.add_argument("--src-h", type=int, default=None,
                   help="Override video height; default: probed from cv2.")
    p.add_argument("--grid", action="append", default=[],
                   help="N×M discovery grid; repeat to warm multiple grids.")
    p.add_argument("--cache-dir", type=Path, default=Path(".tile_cache"),
                   help="Directory holding cache files.")
    p.add_argument("--ppv", type=int, default=1,
                   help="post-process version (spec §7.2). Default 1.")
    p.add_argument("--score-floor", type=float, default=0.01,
                   help="Score floor cached at (spec §7.2). For metadata only.")
    p.add_argument("--backend", choices=["mock"], default="mock",
                   help="Inference backend. Plan 4 ships 'mock' only; "
                        "Plan 5 adds 'hef' and 'gst'.")
    p.add_argument("--batch-rows", type=int, default=128,
                   help="Flush put_many() every N rows.")
    p.add_argument("--max-frames", type=int, default=None,
                   help="Limit number of frames warmed (debugging).")
    return p


def parse_grids(args: Sequence[str]) -> list[tuple[int, int]]:
    """Parse '--grid 3x2' tokens into a list of (gx, gy) tuples."""
    out: list[tuple[int, int]] = []
    for tok in args:
        if "x" not in tok:
            sys.exit(f"--grid: expected NxM, got {tok!r}")
        try:
            a, b = tok.split("x", 1)
            out.append((int(a), int(b)))
        except (ValueError, TypeError):
            sys.exit(f"--grid: expected NxM, got {tok!r}")
    return out


# -------------------------------------------------------------- crop layout


def crops_for_grids(
    src_w: int,
    src_h: int,
    grids: Sequence[tuple[int, int]],
) -> list[CropRect]:
    """Build a deduped crop list by running each grid through DiscoveryGridEmitter.

    Deterministic: identical args → identical output (so the cache key set is
    reproducible across runs).
    """
    out: list[CropRect] = []
    seen: set[tuple[int, int, int, int]] = set()
    meter = BudgetMeter(budget_inf_per_s=1e9, fps=30.0)  # effectively unlimited
    for gx, gy in grids:
        e = DiscoveryGridEmitter(grid=(gx, gy), period=1, mode="m")
        for c in e.emit(src_w, src_h, LockState(), frame_idx=0, meter=meter):
            key = (c.x, c.y, c.w, c.h)
            if key not in seen:
                seen.add(key)
                out.append(c)
    return out


# ------------------------------------------------------------- video walker


def iter_video_frames(path: Path) -> Iterable[tuple[int, "any"]]:
    """Yield `(frame_idx, frame_bgr)` pairs from `path` using cv2.VideoCapture.

    Lazy-imports cv2 so test stubs can monkeypatch this function without
    triggering the cv2 import.
    """
    import cv2  # noqa: WPS433 — lazy
    cap = cv2.VideoCapture(str(path))
    try:
        i = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                return
            yield i, frame
            i += 1
    finally:
        cap.release()


def probe_video_dims(path: Path) -> tuple[int, int]:
    import cv2
    cap = cv2.VideoCapture(str(path))
    try:
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return w, h
    finally:
        cap.release()


# -------------------------------------------------------- backend factories


def make_backend(kind: str, **kwargs) -> InferenceBackend:
    """Build the inference backend selected by --backend.

    Plan 4 supports only `mock` (returns empty lists). Plan 5 will register
    'hef' and 'gst'.
    """
    if kind == "mock":
        return MockBackend()
    raise ValueError(f"Unknown backend: {kind}")  # pragma: no cover


# ------------------------------------------------------------- main loop


def warm_video(
    frames: Iterable[tuple[int, "any"]],
    crops: Sequence[CropRect],
    backend: InferenceBackend,
    store: SqliteCacheStore,
    *,
    ppv: int,
    batch_rows: int = 128,
) -> None:
    """Run `backend.infer(frame, crops, frame_idx)` over `frames`, cache misses.

    Each frame's results land in the store via a `CachingBackend` decorator
    (so we get the cache-skip-on-hit behaviour for free).
    """
    cb = CachingBackend(wrapped=backend, store=store, ppv=ppv)
    for frame_idx, frame in frames:
        cb.infer(frame, crops, frame_idx)
        # `batch_rows` is currently advisory — `put_many` already does one
        # transaction per frame. Larger batches (multi-frame) are a v2
        # optimisation if profiling shows it matters.


def _cache_filename(video_sha: str, hef_sha: str) -> str:
    return f"{video_sha[:16]}__{hef_sha[:16]}.sqlite3"


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_argparser().parse_args(argv)
    grids = parse_grids(args.grid)
    if not grids:
        sys.exit("--grid: at least one grid required")

    video_sha = file_sha256(args.video)
    hef_sha = file_sha256(args.hef)
    args.cache_dir.mkdir(parents=True, exist_ok=True)
    cache_path = args.cache_dir / _cache_filename(video_sha, hef_sha)

    if args.src_w is None or args.src_h is None:
        src_w, src_h = probe_video_dims(args.video)
        src_w = args.src_w or src_w
        src_h = args.src_h or src_h
    else:
        src_w, src_h = args.src_w, args.src_h

    crops = crops_for_grids(src_w, src_h, grids)

    store = SqliteCacheStore.open(cache_path)
    try:
        store.meta_put("video_sha256", video_sha)
        store.meta_put("video_path", str(args.video))
        store.meta_put("video_w", str(src_w))
        store.meta_put("video_h", str(src_h))
        store.meta_put("hef_sha256", hef_sha)
        store.meta_put("hef_path", str(args.hef))
        store.meta_put("ppv", str(args.ppv))
        store.meta_put("score_floor", str(args.score_floor))
        store.meta_put("created_at", str(time.time()))

        backend = make_backend(args.backend)
        frames = iter_video_frames(args.video)
        if args.max_frames is not None:
            def _capped(it, n):
                for i, x in enumerate(it):
                    if i >= n:
                        return
                    yield x
            frames = _capped(frames, args.max_frames)

        warm_video(frames=frames, crops=crops, backend=backend,
                   store=store, ppv=args.ppv, batch_rows=args.batch_rows)
    finally:
        store.close()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
```

- [ ] **Step 4: Run the tests, expect pass.**

```bash
pytest hailo_tiling/tests/test_cli_warm.py -v
```
Expected: 7 passed.

- [ ] **Step 5: Commit.**

```bash
git add hailo_tiling/cli/__init__.py hailo_tiling/cli/warm.py \
        hailo_tiling/tests/test_cli_warm.py
git commit -m "hailo_tiling: hailo-tiling-warm-cache CLI (mock backend; Plan 5 adds hef/gst)"
```

---

## Task 9: `pyproject.toml` script entry + public-API re-exports

**Files:**
- Modify: `pyproject.toml`
- Modify: `hailo_tiling/__init__.py`
- Modify: `hailo_tiling/tests/test_public_api.py` (extend the existing test)

- [ ] **Step 1: Register the script in `pyproject.toml`.**

In `[project.scripts]`, add the new entry alongside `drone-follow`:

```toml
[project.scripts]
drone-follow = "drone_follow.drone_follow_app:main"
hailo-tiling-warm-cache = "hailo_tiling.cli.warm:main"
```

- [ ] **Step 2: Re-export the new public surface.**

Edit `hailo_tiling/__init__.py`; append after the Plan 2 surface (and add to `__all__`):

```python
# --- Plan 4 surface ---
from .backends import CacheMissError, CachingBackend, ReplayBackend
from .cache import SqliteCacheStore, canonicalize_crop, file_sha256
```

And in `__all__`:

```python
    # Plan 4 — cache
    "SqliteCacheStore",
    "CacheMissError",
    "CachingBackend",
    "ReplayBackend",
    "canonicalize_crop",
    "file_sha256",
```

- [ ] **Step 3: Extend the public-API test.**

Append to `hailo_tiling/tests/test_public_api.py`:

```python
def test_top_level_imports_plan4():
    import hailo_tiling as ht
    # Cache
    assert ht.SqliteCacheStore is not None
    assert ht.CacheMissError is not None
    assert ht.CachingBackend is not None
    assert ht.ReplayBackend is not None
    assert callable(ht.file_sha256)
    assert callable(ht.canonicalize_crop)
```

- [ ] **Step 4: Re-install editable (script entry needs re-registration).**

```bash
./hailo-apps/venv_hailo_apps/bin/pip install -e . 2>&1 | tail -5
```
Expected: "Successfully installed hailo_tiling-…" or the equivalent re-install message.

- [ ] **Step 5: Verify the script is on PATH.**

```bash
./hailo-apps/venv_hailo_apps/bin/hailo-tiling-warm-cache --help | head -5
```
Expected: argparse usage line; no exceptions.

- [ ] **Step 6: Run the full test suite.**

```bash
pytest hailo_tiling/tests -q
```
Expected: every test passes — ≥ 200 cases (164 from Plans 1-3 + ~36 new from Plan 4 across the seven new test files).

- [ ] **Step 7: Commit.**

```bash
git add hailo_tiling/__init__.py pyproject.toml hailo_tiling/tests/test_public_api.py
git commit -m "hailo_tiling: register hailo-tiling-warm-cache script + Plan 4 public re-exports"
```

---

## Task 10: Flip Plan 4 status in `INDEX.md` to `done`

**Files:**
- Modify: `docs/superpowers/plans/INDEX.md`

- [ ] **Step 1: Edit the index.**

Change the row for Plan 4 from `in flight` to `done`:

```
| 4   | `2026-05-28-cache-schema-and-python-cache-layer.md`   | 7           | done       |
```

(Keep the filename column populated so the index links to the plan doc.)

- [ ] **Step 2: Commit.**

```bash
git add docs/superpowers/plans/INDEX.md
git commit -m "plans: flip Plan 4 status to done in INDEX"
```

---

## Plan-wide success criteria (self-check before declaring this plan done)

- [ ] `hailo_tiling/cache/` directory exists with `__init__.py`, `schema.sql`, `hashing.py`, `store.py`.
- [ ] `hailo_tiling/backends/caching.py` and `hailo_tiling/backends/replay.py` exist; the backends `__init__.py` re-exports both.
- [ ] `hailo_tiling/cli/warm.py` exists and the script `hailo-tiling-warm-cache` is on PATH after `pip install -e .`.
- [ ] `python -c "from hailo_tiling import SqliteCacheStore, CachingBackend, ReplayBackend, CacheMissError; print('ok')"` prints `ok`.
- [ ] `pytest hailo_tiling/tests -q` reports **all** tests passing — Plan 1/2/3 baseline (164) plus the new Plan 4 suites (≈ 36 cases): `test_cache_hashing.py` (7), `test_cache_store.py` (14), `test_cache_store_concurrency.py` (2), `test_backend_caching.py` (8), `test_backend_replay.py` (5), `test_cli_warm.py` (7), plus the extended `test_public_api.py` case.
- [ ] `hailo-tiling-warm-cache --help` exits 0.
- [ ] `sqlite3 <some-warmed-file>.sqlite3 "SELECT COUNT(*) FROM detections;"` works on a warmed cache without needing the library installed in the sqlite3 shell (the spec §7.1 inspectability bar).
- [ ] `dynamic_tiling/scheduler.py` is **unchanged**.
- [ ] `dynamic_tiling/inference.py` is **unchanged** — the legacy single-crop `ReplayBackend` shim from Plan 2 still works.
- [ ] No new runtime dependencies in `pyproject.toml`.
- [ ] All commits are on the current branch (`tiling-benchmark`); no force-pushes; each task ended with at least one commit.

---

## Out of scope for this plan (handled later)

- **GStreamer plugins** (`hailodet_record` two modes, `hailonet_cache`, `libhailotile_cache.so`) — Plan 5.
- **`hailo-apps-core` patches** (`hailofilter bypass-on-cache-hit`, detiler metadata hooks) — Plan 5.
- **`HefBackend` exercised by warm-cache** — the CLI exposes only `--backend mock` in Plan 4; Plan 5 adds `--backend hef` and `--backend gst`.
- **Cache federation across multiple `(video, HEF)` pairs in one harness run** — the spec mandates one DB per pair; the cache directory holds many files but the warmer addresses one pair at a time.
- **Quantise=4 paper run** — the code path is implemented and unit-tested, but no end-to-end ablation row uses it in Plan 4; the unquantised default is the paper-correct mode (spec §7.3).
- **Cache lifecycle CLI** (`hailo-tiling-cache prune --older-than 30d`) — spec §7.7 calls this out as v2.
- **Determinism CI test** (assert `HefBackend` twice over the same crop produces byte-identical results) — listed in spec phase 7 but blocked on Plan 5's chip-enabled CI lane.
- **Published reference cache fetch script** (`scripts/fetch_reference_cache.sh`) — Plan 9 (paper artifacts).
- **`hailo-tiling-bench` integration with the cache** — Plan 6 wires `--cache .tile_cache/ --backend replay` into the harness.
- **Full-frame `frame_results` table** (spec §7.8 mode `full_frame`) — that's the GStreamer recorder's schema; lives in Plan 5.

---

## Open Questions

1. **Where does `file_sha256` live canonically?** This plan re-implements `file_sha256` inside `hailo_tiling/cache/hashing.py` rather than reaching across to `tiling_benchmark/prepare_video.py:sha256_of_file`. The duplication is intentional: the cache layer is a published library surface that will eventually outlive `tiling_benchmark/` (which is a research consumer), and we don't want the library to depend on the benchmark module. The cost is ~7 lines of duplicated code and the discipline that future hash-related changes update both sites. **Resolution proposal:** keep the duplicate in Plan 4; in a future cleanup plan (after `tiling_benchmark/` itself migrates onto `hailo_tiling/cache.hashing`), drop the `tiling_benchmark` copy in favour of the library one. **Decision:** library-local, justified inline in the module docstring.

2. **Should the `meta` table be writable post-creation, or only at open time?** Plan 4 makes `meta_put` available at any time after `open()` (so the warmer can write `ppv`, `score_floor`, etc. after construction). Plan 5's `hailodet_record` will likely also write `meta` entries during a long-running flight (e.g., HEF SHA discovered from a `hailo-hef-sha` buffer meta on the first frame). **The spec doesn't forbid this** but does say (§7.7) "schema migration is rejected, not auto-migrated" — the *schema* is immutable post-open, but the *meta table contents* are not. This plan documents `meta_put` as upsert-safe and concurrent-safe in WAL mode; if a future plan finds reasons to lock meta after `created_at`, we'll add a `meta_seal()` method then. **Decision:** mutable post-creation, no seal in v1.

3. **Lock contention strategy when warming and inferring concurrently.** SQLite WAL handles N readers + 1 writer atomically without explicit application-level locks (verified in `test_cache_store_concurrency.py`). The spec (§7.7) says "one bench process at a time per cache file" and notes that the harness takes an advisory file lock — *that lock is not implemented in Plan 4*. Plan 4 ships only the warmer (one writer) and the replay path (N readers); Plan 6's bench harness is the natural home for an `fcntl.flock` advisory wrapper. **Decision for Plan 4:** rely on WAL semantics; document that running two `hailo-tiling-warm-cache` invocations against the same DB simultaneously is undefined behaviour, and add the advisory lock in Plan 6 when the harness lands.

4. **Should `CachingBackend` validate that `wrapped`'s output length matches the request length?** Currently a single `assert` (Task 6, Phase 3) checks `len(new_results) == len(miss_crops)` — useful for `MockBackend`, mandatory for `HefBackend`/`GstCropperBackend`. The assert raises `AssertionError`, which is the wrong exception class for production. **Resolution proposal:** in a Plan 5 follow-up, promote the assert to a typed `BackendContractError` once we have a chip-enabled CI lane that can exercise the failure modes. For Plan 4 the bare `assert` is fine because `MockBackend` always honours the contract by construction.

5. **`PRAGMA journal_mode = WAL` vs `WAL2`.** The spec calls for WAL. Modern SQLite supports WAL2 in some builds, but the dev machine reports plain `wal` after `PRAGMA journal_mode = WAL` (which is what the spec actually says). **Decision:** WAL (not WAL2). Stick with the spec.

6. **Where should the per-frame cv2 frame go in `MockBackend.calls`?** Currently `MockBackend.calls` records the literal `frame` argument verbatim. When the warmer passes a real `cv2.VideoCapture` frame (a NumPy array) and the test uses `frame=None`, those code paths diverge. **Resolution:** `MockBackend` already handles `frame=None` fine (Plan 2 design) — the per-crop canned-lookup ignores `frame`. The warmer's `MockBackend` integration in the CLI tests passes `frame=None`/raw bytes interchangeably. No change needed.

---

### Critical Files for Implementation

- /home/giladn/tappas_apps/repos/hailo-drone-follow/hailo_tiling/cache/store.py
- /home/giladn/tappas_apps/repos/hailo-drone-follow/hailo_tiling/cache/schema.sql
- /home/giladn/tappas_apps/repos/hailo-drone-follow/hailo_tiling/backends/caching.py
- /home/giladn/tappas_apps/repos/hailo-drone-follow/hailo_tiling/backends/replay.py
- /home/giladn/tappas_apps/repos/hailo-drone-follow/hailo_tiling/cli/warm.py
