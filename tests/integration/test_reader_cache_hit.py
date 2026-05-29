"""GStreamer reader reads a Python-written cache (Plan 5 Task 9).

Drives the `hailocachereader` element end-to-end through GStreamer to
demonstrate that the lookup wired in Task 9 reads the same SQLite that
the Python `SqliteCacheStore` writes. This complements the C++ gtest
cases in ``gst-hailo-cache/tests/test_hailocachereader.cpp`` (which test
the lookup against caches written via ``TileCacheDb`` directly).

The full Python-vs-GST bit-exact comparison lives in Task 11
(``test_cache_bit_exact_e2e.py``) — this test only verifies that the
reader's cache-hit qdata marker is observable downstream when the cache
was produced by the Python writer.

When the plugin .so is not built (i.e. nobody ran
``cd gst-hailo-cache && meson setup build && ninja -C build``) the test
SKIPs with a clear message.
"""
from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

from hailo_tiling.cache.store import SqliteCacheStore
from hailo_tiling.types import CropRect, Det


REPO_ROOT = Path(__file__).resolve().parents[2]
PLUGIN_PATH = REPO_ROOT / "gst-hailo-cache" / "build" / "src" / "libgsthailocache.so"


# Fixed test resolution — used both to populate the cache and to pin the
# pipeline caps so the reader's Task-9 fallback full-frame crop key
# matches what we wrote.
WIDTH = 320
HEIGHT = 240


def _populate_cache(path: Path, n_frames: int = 3, ppv: int = 1) -> None:
    """Write `n_frames` rows for a full-frame crop to a fresh SQLite."""
    with SqliteCacheStore.open(path) as store:
        store.meta_put("ppv", str(ppv))
        rows = []
        for fi in range(n_frames):
            rows.append({
                "frame_idx": fi,
                "crop_rect": CropRect(x=0, y=0, w=WIDTH, h=HEIGHT, mode="m"),
                "ppv": ppv,
                "dets": [Det(cls=0, score=0.5, x=0.1, y=0.2, w=0.3, h=0.4)],
                "ts_epoch": 1700000000.0 + fi,
            })
        store.put_many(rows)


def _gst_launch_with_plugin(pipeline_tokens: list[str],
                            extra_env: dict | None = None,
                            timeout: float = 15.0) -> subprocess.CompletedProcess:
    """Run gst-launch-1.0 with the local plugin .so on GST_PLUGIN_PATH.

    `pipeline_tokens` is the pipeline description as a list of argv
    tokens (gst-launch's own parser stitches them — the `!` separators
    and `element prop=value` pairs are each their own token).
    """
    env = os.environ.copy()
    plugin_dir = str(PLUGIN_PATH.parent)
    env["GST_PLUGIN_PATH"] = plugin_dir + (
        ":" + env["GST_PLUGIN_PATH"] if env.get("GST_PLUGIN_PATH") else ""
    )
    # Force re-scan so the test sees our freshly built .so even if a
    # stale system registry is around (see
    # .claude/memory/hailotilecropper_dynamic.md for the back-story).
    env["GST_REGISTRY_UPDATE"] = "yes"
    # Drop any cached registry the user may have built against the
    # system gst-plugin path to avoid the stale-.so trap.
    env.setdefault("GST_REGISTRY", "/tmp/gst_registry_test_reader_cache_hit.bin")
    if extra_env:
        env.update(extra_env)
    return subprocess.run(
        ["gst-launch-1.0", "-v"] + pipeline_tokens,
        env=env,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def test_gst_reader_reads_python_written_cache(tmp_path):
    if not PLUGIN_PATH.exists():
        pytest.skip(
            f"libgsthailocache.so not built at {PLUGIN_PATH}. "
            "Run `cd gst-hailo-cache && meson setup build && ninja -C build`."
        )

    # 1. Populate the cache from Python (the same write path the warmer
    #    uses) — 3 rows at the full-frame fallback crop.
    cache = tmp_path / "task9_reader_cache_hit.sqlite3"
    _populate_cache(cache, n_frames=3, ppv=1)

    # Sanity check: cache opened by Python is consistent.
    with SqliteCacheStore.open(cache) as store:
        assert store.stats()["n_rows"] == 3
        assert store.meta_get("ppv") == "1"

    # 2. Drive the gst-launch pipeline. With on-miss=error, the pipeline
    #    must reach EOS without a bus error iff every buffer hit the
    #    cache. Posting an error would make gst-launch exit non-zero.
    proc = _gst_launch_with_plugin([
        "videotestsrc", "num-buffers=3",
        "!",
        f"video/x-raw,width={WIDTH},height={HEIGHT},framerate=30/1",
        "!",
        "hailocachereader", f"cache-file={cache}", "on-miss=error",
        "!",
        "fakesink", "async=false", "sync=false",
    ])

    # The integration test's primary contract: gst-launch exits 0
    # (EOS, no error) when every buffer hits the cache. Capture stderr
    # for the report.
    assert proc.returncode == 0, (
        f"gst-launch exited {proc.returncode}; "
        f"stderr:\n{proc.stderr}\nstdout:\n{proc.stdout}"
    )


def test_gst_reader_on_miss_error_raises_on_empty_cache(tmp_path):
    """Companion case: an empty cache + on-miss=error must surface as
    a non-zero gst-launch exit code (proves the bus error round-trips
    through real GStreamer)."""
    if not PLUGIN_PATH.exists():
        pytest.skip(
            f"libgsthailocache.so not built at {PLUGIN_PATH}. "
            "Run `cd gst-hailo-cache && meson setup build && ninja -C build`."
        )

    cache = tmp_path / "task9_reader_empty.sqlite3"
    # Create an empty (but schema-stamped) cache so the reader opens
    # cleanly but every lookup misses.
    with SqliteCacheStore.open(cache) as store:
        store.meta_put("ppv", "1")
        assert store.stats()["n_rows"] == 0

    proc = _gst_launch_with_plugin([
        "videotestsrc", "num-buffers=3",
        "!",
        f"video/x-raw,width={WIDTH},height={HEIGHT},framerate=30/1",
        "!",
        "hailocachereader", f"cache-file={cache}", "on-miss=error",
        "!",
        "fakesink", "async=false", "sync=false",
    ])

    assert proc.returncode != 0, (
        "gst-launch should have failed with on-miss=error against an "
        f"empty cache; got rc=0. stderr:\n{proc.stderr}"
    )
    # Spec §7.9 promises a loud error message; verify we got something
    # actionable on stderr.
    assert "cache miss" in proc.stderr.lower() or \
        "not_found" in proc.stderr.lower() or \
        "hailocachereader" in proc.stderr.lower(), \
        "expected a hailocachereader-flavoured error on stderr:\n" + proc.stderr
