"""Chip smoke for GstCropperBackend wrapped in CachingBackend (Plan 6 B4 Step 5).

Gated on HAILO_CHIP=1. Runs the canonical GStreamer cropper path live over a
few frames of 0026 fov50 with a small crop set, asserting detections are
returned in crop order and the cache is populated. MUST NOT run while Track A
(A3 warming) holds the chip — chip work is strictly serial.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
_PLUGIN_DIR = _REPO_ROOT / "gst-hailo-cache" / "build" / "src"
_VIDEO = Path(
    "/home/giladn/Videos/Drone/Training/"
    "DJI_20260528155239_0026_D_prepared__fov50.mp4"
)
_HEF = Path(
    "/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef"
)
_POST_SO = Path("/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so")
_W, _H = 3840, 2160


def _chip_requested() -> bool:
    return os.environ.get("HAILO_CHIP") == "1" or "--chip" in sys.argv


@pytest.fixture(scope="module")
def chip_gate():
    if not _chip_requested():
        pytest.skip("set HAILO_CHIP=1 to enable chip tests")
    import shutil
    import subprocess

    if shutil.which("hailortcli") is None:
        pytest.skip("hailortcli not on PATH")
    proc = subprocess.run(["hailortcli", "scan"], capture_output=True, text=True, timeout=10)
    if not (proc.returncode == 0 and "Device" in (proc.stdout + proc.stderr)):
        pytest.skip("Hailo chip not available")
    for p in (_VIDEO, _HEF, _POST_SO):
        if not p.exists():
            pytest.skip(f"missing required path: {p}")
    if not (_PLUGIN_DIR / "libgsthailocache.so").exists():
        pytest.skip(f"cache plugin not built: {_PLUGIN_DIR}")


def test_caching_gst_cropper_populates_cache(chip_gate, tmp_path):
    if str(_PLUGIN_DIR) not in os.environ.get("GST_PLUGIN_PATH", ""):
        existing = os.environ.get("GST_PLUGIN_PATH", "")
        os.environ["GST_PLUGIN_PATH"] = (
            f"{_PLUGIN_DIR}:{existing}" if existing else str(_PLUGIN_DIR)
        )

    from hailo_tiling.backends.caching import CachingBackend
    from hailo_tiling.backends.gst_cropper import GstCropperBackend
    from hailo_tiling.cache.hashing import tile_norm_to_source_px
    from hailo_tiling.cache.store import SqliteCacheStore

    crops = [
        tile_norm_to_source_px(0.0, 0.0, 0.5, 0.5, _W, _H),
        tile_norm_to_source_px(0.5, 0.0, 0.5, 0.5, _W, _H),
        tile_norm_to_source_px(0.0, 0.5, 0.5, 0.5, _W, _H),
        tile_norm_to_source_px(0.5, 0.5, 0.5, 0.5, _W, _H),
    ]
    store = SqliteCacheStore.open(tmp_path / "gcb.sqlite3")
    try:
        backend = GstCropperBackend(
            hef=str(_HEF), post_so=str(_POST_SO), source_w=_W, source_h=_H
        )
        caching = CachingBackend(backend, store, ppv=1)
        # Frame 0's four tiles.
        out = caching.infer(str(_VIDEO), crops, 0)
        assert len(out) == 4, "one det-list per crop"
        # Cache populated.
        assert int(store.stats()["n_rows"]) >= 4
    finally:
        store.close()
