"""Tests for scripts/warm_gst_cache.py (Plan 6 Task A2).

Two layers:
  * No-chip unit tests for grid-spec parsing / expansion + cache naming.
  * A chip-gated smoke (HAILO_CHIP=1) that warms 4 frames of clip 0026 fov50
    into a real source-pixel-keyed cache and checks it is non-empty with the
    expected meta.
"""
from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
_WARMER = _REPO_ROOT / "scripts" / "warm_gst_cache.py"
_PLUGIN_DIR = _REPO_ROOT / "gst-hailo-cache" / "build" / "src"
_VIDEO = Path(
    "/home/giladn/Videos/Drone/Training/"
    "DJI_20260528155239_0026_D_prepared__fov50.mp4"
)
_HEF = Path(
    "/usr/local/hailo/resources/models/hailo10h/"
    "hailo_yolov8n_4_classes_vga.hef"
)
_POST_SO = Path("/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so")


def _load_warmer():
    """Import warm_gst_cache as a module without requiring scripts/ on PATH.

    The module imports cache_gst_replay_gate (which imports gi/hailo). On a
    host without GStreamer those imports fail; in that case the unit tests
    that only need pure-Python parsing are skipped.
    """
    spec = importlib.util.spec_from_file_location("warm_gst_cache", _WARMER)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except Exception as exc:  # pragma: no cover - host without gi/hailo
        pytest.skip(f"warm_gst_cache import failed (gi/hailo unavailable?): {exc}")
    return mod


# --------------------------------------------------------------------------
# No-chip unit tests
# --------------------------------------------------------------------------

def test_grid_spec_expands_to_tiles():
    """Parse "3x2:0.25" -> one (3,2,0.25) grid -> 6 normalized tiles in [0,1]."""
    warmer = _load_warmer()
    grids = warmer.parse_grid_spec("3x2:0.25")
    assert grids == [(3, 2, 0.25)]

    tiles_static = warmer.grid_to_tiles_static(3, 2, 0.25)
    tiles = [t for t in tiles_static.split(";") if t.strip()]
    assert len(tiles) == 6, f"3x2 should expand to 6 tiles, got {tiles}"
    for t in tiles:
        parts = [float(v) for v in t.split(",")[:4]]
        x, y, w, h = parts
        assert 0.0 <= x <= 1.0 and 0.0 <= y <= 1.0
        assert 0.0 < w <= 1.0 and 0.0 < h <= 1.0
        assert x + w <= 1.0 + 1e-6 and y + h <= 1.0 + 1e-6


def test_grid_spec_multi_and_defaults():
    """Multiple grids; missing overlap defaults to 0.0."""
    warmer = _load_warmer()
    grids = warmer.parse_grid_spec("1x1; 3x2:0.25 ;6x4:0.25")
    assert grids == [(1, 1, 0.0), (3, 2, 0.25), (6, 4, 0.25)]


def test_grid_spec_rejects_bad_input():
    warmer = _load_warmer()
    with pytest.raises(ValueError):
        warmer.parse_grid_spec("")
    with pytest.raises(ValueError):
        warmer.parse_grid_spec("not-a-grid")
    with pytest.raises(ValueError):
        warmer.parse_grid_spec("3x2:1.5")  # overlap out of range


def test_default_cache_filename_strips_fov_and_uses_hef_sha(tmp_path):
    warmer = _load_warmer()
    # Fake hef file so file_sha256 has something to hash.
    hef = tmp_path / "model.hef"
    hef.write_bytes(b"abc")
    name = warmer.default_cache_filename(
        "/data/DJI_x_prepared__fov50.mp4", "fov50", hef
    )
    assert name.startswith("DJI_x_prepared__fov50__")
    assert name.endswith(".sqlite3")
    # 16 hex chars of sha between fov50 and .sqlite3
    sha_part = name[len("DJI_x_prepared__fov50__"):-len(".sqlite3")]
    assert len(sha_part) == 16


# --------------------------------------------------------------------------
# Chip-gated smoke
# --------------------------------------------------------------------------

def _chip_requested() -> bool:
    return os.environ.get("HAILO_CHIP") == "1" or "--chip" in sys.argv


def _chip_available() -> tuple[bool, str]:
    import shutil
    if shutil.which("hailortcli") is None:
        return False, "hailortcli not on PATH"
    try:
        proc = subprocess.run(
            ["hailortcli", "scan"], capture_output=True, text=True, timeout=10,
        )
    except subprocess.SubprocessError as exc:
        return False, f"hailortcli failed: {exc}"
    out = (proc.stdout or "") + (proc.stderr or "")
    if proc.returncode == 0 and "Device" in out:
        return True, "OK"
    return False, out.strip()[:200] or "no device found"


@pytest.fixture(scope="module")
def chip_gate():
    if not _chip_requested():
        pytest.skip("set HAILO_CHIP=1 (or pass --chip) to enable chip tests")
    ok, msg = _chip_available()
    if not ok:
        pytest.skip(f"Hailo chip not available: {msg}")
    for p in (_VIDEO, _HEF, _POST_SO):
        if not p.exists():
            pytest.skip(f"missing required path: {p}")
    if not (_PLUGIN_DIR / "libgsthailocache.so").exists():
        pytest.skip(f"cache plugin not built: {_PLUGIN_DIR}")


def test_chip_smoke_warms_nonempty_cache(chip_gate, tmp_path):
    """Warm 4 frames of 0026 fov50 with a single 3x2 grid; assert the cache
    exists, has detection rows, and meta records video_w=3840."""
    out_cache = tmp_path / "smoke_fov50.sqlite3"
    env = dict(os.environ)
    existing = env.get("GST_PLUGIN_PATH", "")
    env["GST_PLUGIN_PATH"] = (
        f"{_PLUGIN_DIR}:{existing}" if existing else str(_PLUGIN_DIR)
    )
    cmd = [
        sys.executable, str(_WARMER),
        "--video", str(_VIDEO),
        "--hef", str(_HEF),
        "--post-so", str(_POST_SO),
        "--out-cache", str(out_cache),
        "--grids", "3x2:0.25",
        "--source-width", "3840",
        "--source-height", "2160",
        "--max-frames", "4",
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600, env=env)
    assert proc.returncode == 0, (
        f"warmer failed ({proc.returncode})\nstdout:\n{proc.stdout}\n"
        f"stderr:\n{proc.stderr}"
    )
    assert out_cache.exists(), "cache file not created"

    from hailo_tiling.cache.store import SqliteCacheStore

    store = SqliteCacheStore.open(out_cache)
    try:
        n = int(store.stats()["n_rows"])
        assert n > 0, "no detection rows written"
        assert store.meta_get("video_w") == "3840"
        assert store.meta_get("video_h") == "2160"
    finally:
        store.close()
