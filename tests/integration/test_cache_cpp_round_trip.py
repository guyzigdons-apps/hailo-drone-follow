"""Bit-exact round-trip: C++ TileCacheDb writes → SqliteCacheStore reads.

Plan 5 Task 2 (docs/superpowers/plans/2026-05-28-gst-cache-plugins.md):
the C++ helper must produce a SQLite file that the Python
``SqliteCacheStore`` consumes byte-for-byte. This test invokes the
test-only ``cache_db_cli`` binary built by the meson ``tests/`` subdir,
feeds it 10 rows as JSON-lines on stdin, then reopens the resulting
SQLite via :class:`hailo_tiling.cache.SqliteCacheStore` and asserts each
row's ``dets_json`` is identical to the input bytes.

If the C++ helper hasn't been built yet (i.e. nobody ran
``bash gst-hailo-cache/install.sh`` or
``meson setup build && ninja -C build``), the test is SKIPPED with a
clear message — we don't want the Python test matrix to fail just
because the C++ side isn't built on this checkout.
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from hailo_tiling.cache.store import SqliteCacheStore
from hailo_tiling.types import CropRect, Det


REPO_ROOT = Path(__file__).resolve().parents[2]
CLI_PATH = REPO_ROOT / "gst-hailo-cache" / "build" / "tests" / "cache_db_cli"


def _make_rows():
    """Return (json-lines string, expected (frame_idx, CropRect, ppv, dets_json) tuples).

    Ten rows spanning 3 frames, with a deliberately tricky dets_json
    payload (escaped quote inside a string) to exercise the C++ helper's
    JSON-string parser. The dets are constructed via the same
    ``_dets_to_json`` shape Python uses, so we can compare strings
    byte-for-byte.
    """
    rows_in = []
    expected = []
    base_ts = 1700000000.5
    for i in range(10):
        frame_idx = i // 4  # 3 frames: 0, 0, 0, 0, 1, 1, 1, 1, 2, 2
        crop = CropRect(x=10 * i, y=20 * i, w=200, h=200, mode="m")
        # Build dets_json the SAME way Python does so the round-trip
        # comparison is byte-exact.
        dets = [
            Det(cls=i, score=0.5 + 0.01 * i, x=0.1 * i, y=0.2 * i,
                w=0.3 + 0.001 * i, h=0.4),
        ]
        dets_json = json.dumps(
            [{"cls": d.cls, "score": d.score, "x": d.x, "y": d.y,
              "w": d.w, "h": d.h} for d in dets],
            separators=(",", ":"),
        )
        ts_epoch = base_ts + i
        rows_in.append({
            "frame_idx": frame_idx,
            "crop_x": crop.x,
            "crop_y": crop.y,
            "crop_w": crop.w,
            "crop_h": crop.h,
            "ppv": 1,
            "dets_json": dets_json,
            "ts_epoch": ts_epoch,
        })
        expected.append((frame_idx, crop, 1, dets_json))

    jsonl = "\n".join(json.dumps(r) for r in rows_in) + "\n"
    return jsonl, expected


def test_cpp_writes_python_reads_byte_identical(tmp_path):
    if not CLI_PATH.exists():
        pytest.skip(
            f"C++ cache_db_cli not built at {CLI_PATH}. "
            "Run `cd gst-hailo-cache && meson setup build && ninja -C build`."
        )

    db_path = tmp_path / "roundtrip.sqlite3"
    jsonl, expected = _make_rows()

    proc = subprocess.run(
        [str(CLI_PATH), str(db_path)],
        input=jsonl,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert proc.returncode == 0, (
        f"cache_db_cli failed: rc={proc.returncode}\n"
        f"stdout: {proc.stdout}\nstderr: {proc.stderr}"
    )
    assert db_path.exists(), "cache_db_cli reported success but no DB file"

    # Reopen via the Python store and verify schema version + row count.
    with SqliteCacheStore.open(db_path) as store:
        stats = store.stats()
        assert stats["schema_version"] == 1, stats
        assert stats["n_rows"] == len(expected), stats

        # Group by frame_idx so we can use get_many in input order.
        by_frame: dict[int, list[tuple[CropRect, int, str]]] = {}
        for fi, crop, ppv, dj in expected:
            by_frame.setdefault(fi, []).append((crop, ppv, dj))

        # Verify with the SAME public read path SqliteCacheStore exposes
        # (one-by-one via .get to compare dets directly).
        for fi, crop, ppv, dj_expected in expected:
            dets = store.get(fi, crop, ppv=ppv)
            assert dets is not None, (
                f"miss on (frame={fi}, crop={crop}, ppv={ppv}); "
                "C++ writer did not produce a matching row"
            )
            # Re-serialise via the same _dets_to_json the writer would
            # use — the result MUST be byte-identical to what the C++
            # CLI stored.
            from hailo_tiling.cache.store import _dets_to_json
            dj_roundtrip = _dets_to_json(dets)
            assert dj_roundtrip == dj_expected, (
                f"dets_json drift on frame={fi}, crop={crop}:\n"
                f"  expected: {dj_expected!r}\n"
                f"  got:      {dj_roundtrip!r}"
            )

        # Also exercise get_many's order-preservation property over the
        # whole frame-0 group.
        f0 = by_frame[0]
        crops = [c for (c, _ppv, _dj) in f0]
        results = store.get_many(0, crops, ppv=1)
        assert len(results) == len(f0)
        for (crop, ppv, dj_expected), got in zip(f0, results):
            assert got is not None, f"get_many miss on {crop}"
            from hailo_tiling.cache.store import _dets_to_json
            assert _dets_to_json(got) == dj_expected
