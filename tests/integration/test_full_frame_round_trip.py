"""Full-frame mode end-to-end: hailocachewriter mode=full_frame writes a
SQLite file with the `frame_results` schema documented in spec §7.8/§7.13.

Plan 5 Task 6. The flow:

1. Drive a synthetic `gst-launch-1.0 videotestsrc ! hailocachewriter
   mode=full_frame ...` pipeline against the just-built plugin under
   ``gst-hailo-cache/build/src/``.
2. Open the resulting SQLite via :class:`hailo_tiling.cache.SqliteCacheStore`
   — Task 6 also added the `frame_results` table to
   ``hailo_tiling/cache/schema.sql`` so this MUST succeed without a
   schema_version mismatch.
3. Verify the `frame_results` table layout (column names, types, NOT NULL,
   PRIMARY KEY) matches the spec via ``PRAGMA table_info``.
4. Verify one row per pushed buffer (record-empty default = TRUE).

Skipped (not failed) if the plugin .so or `gst-launch-1.0` is unavailable
— the C++ side may not be built on every checkout, and the Python test
matrix shouldn't fail because of an unrelated missing dependency.
"""
from __future__ import annotations

import shutil
import sqlite3
import subprocess
from pathlib import Path

import pytest

from hailo_tiling.cache.store import SqliteCacheStore


REPO_ROOT = Path(__file__).resolve().parents[2]
PLUGIN_PATH = REPO_ROOT / "gst-hailo-cache" / "build" / "src" / "libgsthailocache.so"


def _gst_launch_available() -> bool:
    return shutil.which("gst-launch-1.0") is not None


def _skip_if_plugin_missing() -> None:
    if not PLUGIN_PATH.exists():
        pytest.skip(
            f"hailocachewriter plugin not built at {PLUGIN_PATH}. "
            "Run `cd gst-hailo-cache && meson setup build && ninja -C build`."
        )
    if not _gst_launch_available():
        pytest.skip("gst-launch-1.0 is not in PATH")


def test_full_frame_schema_round_trip(tmp_path):
    """A full_frame writer must produce a SQLite that SqliteCacheStore opens
    cleanly AND that exposes the `frame_results` table with the documented
    schema."""
    _skip_if_plugin_missing()

    db_path = tmp_path / "ff_round_trip.sqlite3"
    n_buffers = 8

    # Point gstreamer at our local build of the plugin via GST_PLUGIN_PATH
    # so we don't depend on a system install.
    env = {
        "GST_PLUGIN_PATH": str(PLUGIN_PATH.parent),
        "PATH": "/usr/bin:/bin",  # gst-launch-1.0 + sqlite3 live here
    }
    proc = subprocess.run(
        [
            "gst-launch-1.0",
            "-q",
            "videotestsrc",
            f"num-buffers={n_buffers}",
            "!",
            "hailocachewriter",
            "mode=full_frame",
            f"output-file={db_path}",
            "!",
            "fakesink",
        ],
        env=env,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert proc.returncode == 0, (
        f"gst-launch failed: rc={proc.returncode}\n"
        f"stdout: {proc.stdout}\nstderr: {proc.stderr}"
    )
    assert db_path.exists(), "Pipeline reported success but no SQLite file"

    # 1) SqliteCacheStore must open this file without raising. The Task 6
    #    schema update added `frame_results` to schema.sql; both the
    #    Python store and the C++ writer apply the same DDL on open, so
    #    the file's `user_version=1` is the schema version SqliteCacheStore
    #    accepts.
    with SqliteCacheStore.open(db_path) as store:
        assert store.stats()["schema_version"] == 1

    # 2) Verify the frame_results table layout via raw sqlite3 — the
    #    spec §7.8 schema is the contract.
    con = sqlite3.connect(db_path)
    try:
        cols = con.execute("PRAGMA table_info(frame_results)").fetchall()
        # PRAGMA table_info returns: cid, name, type, notnull, dflt_value, pk.
        # PRIMARY KEY(frame_idx, ppv) → pk=1 for frame_idx, pk=2 for ppv.
        expected = [
            # (name,       type,      notnull, pk)
            ("frame_idx",  "INTEGER", 1, 1),
            ("ppv",        "INTEGER", 1, 2),
            ("dets_json",  "TEXT",    1, 0),
            ("tiles_json", "TEXT",    1, 0),
            ("ts_epoch",   "REAL",    1, 0),
        ]
        assert len(cols) == len(expected), (
            f"frame_results has {len(cols)} columns; expected {len(expected)}: "
            f"{cols!r}"
        )
        for (_cid, name, type_, notnull, _dflt, pk), (xn, xt, xnn, xpk) in zip(
            cols, expected
        ):
            assert name == xn, f"column name mismatch: {name!r} vs {xn!r}"
            assert type_ == xt, f"column {name} type mismatch: {type_!r} vs {xt!r}"
            assert notnull == xnn, f"column {name} NOT NULL mismatch"
            assert pk == xpk, f"column {name} PK ordinal mismatch: {pk} vs {xpk}"

        # 3) One row per buffer (record-empty default = TRUE).
        n = con.execute("SELECT COUNT(*) FROM frame_results").fetchone()[0]
        assert n == n_buffers, (
            f"Expected {n_buffers} frame_results rows; got {n}"
        )

        # 4) Detections table is present (schema applies both) but empty.
        n_det = con.execute("SELECT COUNT(*) FROM detections").fetchone()[0]
        assert n_det == 0, (
            "full_frame writer must NOT populate the detections table; "
            f"got {n_det} rows"
        )

        # 5) tiles_json defaults to '[]' for Task 6 (Phase 14 will plumb
        #    real tile-list metadata through). dets_json is also '[]'.
        rows = con.execute(
            "SELECT dets_json, tiles_json FROM frame_results"
        ).fetchall()
        for dets, tiles in rows:
            assert dets == "[]", f"unexpected dets_json: {dets!r}"
            assert tiles == "[]", f"unexpected tiles_json: {tiles!r}"
    finally:
        con.close()
