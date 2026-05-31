"""Per-tile live-vs-cached bit-exact replay gate (GStreamer, chip).

Plan `2026-05-31-gst-cache-source-pixel-provenance.md` — Task 6.

Drives `scripts/cache_gst_replay_gate.py` through TWO passes of the canonical
tiled GStreamer pipeline:

  - Pass 1 (LIVE): hailotilecropper_dynamic -> [videoconvert ! hailonet !
    hailofilter ! hailocachewriter(tile_cache, source-width/height)] ->
    aggregator. A pad probe AFTER hailofilter records each tile's HailoROI
    detections keyed by (buffer_idx, source-pixel crop). The writer records
    the per-tile crop provenance AND serializes the detections into the cache.

  - Pass 2 (CACHED): same pipeline but the inner branch is
    [videoconvert ! hailocachereader ! hailocachebypass] — there is NO
    hailonet in the graph at all, so the chip is never consulted. The reader
    restores the cached detections into each tile's HailoROI; the same probe
    (now after hailocachebypass) reads them back.

The two per-tile detection streams must be byte-identical (`status == "OK"`),
and pass 2 must have served entirely from cache (`pass2_served_from_cache`,
proven by the absence of any hailonet element in the pass-2 graph).

This test SKIPS unless one of the following is true:
  - the env var HAILO_CHIP=1 is set, OR
  - pytest is invoked with --chip

Run:
    source setup_env.sh
    GST_PLUGIN_PATH=gst-hailo-cache/build/src \
      HAILO_CHIP=1 pytest tests/integration/test_cache_gst_replay_gate.py -v
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
_VIDEO = Path(
    "/home/giladn/Videos/Drone/Training/"
    "DJI_20260528155741_0029_D_prepared__fov50.mp4"
)
_HEF = Path(
    "/usr/local/hailo/resources/models/hailo10h/"
    "hailo_yolov8n_4_classes_vga.hef"
)
_POST_SO = Path("/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so")
_HELPER = _REPO_ROOT / "scripts" / "cache_gst_replay_gate.py"
_PLUGIN_DIR = _REPO_ROOT / "gst-hailo-cache" / "build" / "src"


def pytest_configure(config):  # pragma: no cover - pytest plugin hook
    config.addinivalue_line(
        "markers", "chip: requires a Hailo chip (real-hardware integration test)",
    )


def _chip_requested() -> bool:
    if os.environ.get("HAILO_CHIP") == "1":
        return True
    return "--chip" in sys.argv


def _chip_available() -> tuple[bool, str]:
    """Best-effort hardware probe via hailortcli scan."""
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
        return True, out.strip().splitlines()[0] if out.strip() else "OK"
    return False, out.strip()[:200] or "no device found"


@pytest.fixture(scope="module")
def chip_gate():
    if not _chip_requested():
        pytest.skip("set HAILO_CHIP=1 (or pass --chip) to enable chip integration tests")
    ok, msg = _chip_available()
    if not ok:
        pytest.skip(f"Hailo chip not available: {msg}")
    if not _VIDEO.exists():
        pytest.skip(f"gate video missing: {_VIDEO}")
    if not _HEF.exists():
        pytest.skip(f"gate HEF missing: {_HEF}")
    if not _POST_SO.exists():
        pytest.skip(f"postprocess .so missing: {_POST_SO}")
    if not _HELPER.exists():
        pytest.skip(f"helper missing: {_HELPER}")
    if not (_PLUGIN_DIR / "libgsthailocache.so").exists():
        pytest.skip(f"cache plugin not built: {_PLUGIN_DIR}/libgsthailocache.so")


def _run_helper(out_dir: Path, max_frames: int = 16) -> dict:
    """Invoke the helper as a subprocess; parse diff_report.json."""
    out_dir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    # Make the cache plugin discoverable (hailotilecropper_dynamic is
    # system-installed; the writer/reader/bypass live in the build dir).
    existing = env.get("GST_PLUGIN_PATH", "")
    env["GST_PLUGIN_PATH"] = (
        f"{_PLUGIN_DIR}:{existing}" if existing else str(_PLUGIN_DIR)
    )
    cmd = [
        sys.executable, str(_HELPER),
        "--video", str(_VIDEO),
        "--hef", str(_HEF),
        "--post-so", str(_POST_SO),
        "--out-dir", str(out_dir),
        "--max-frames", str(max_frames),
        "--tiles-static", "3x2",
    ]
    proc = subprocess.run(
        cmd, capture_output=True, text=True, timeout=600, env=env,
    )
    if proc.returncode not in (0, 1):
        pytest.fail(
            "helper exited with unexpected status "
            f"{proc.returncode}\nstdout:\n{proc.stdout}\nstderr:\n{proc.stderr}",
        )
    report_path = out_dir / "diff_report.json"
    if not report_path.exists():
        pytest.fail(
            "helper did not produce diff_report.json\n"
            f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}",
        )
    report = json.loads(report_path.read_text())
    report["_stdout"] = proc.stdout
    report["_stderr"] = proc.stderr
    report["_returncode"] = proc.returncode
    return report


def test_per_tile_live_vs_cached_bit_exact(chip_gate, tmp_path):
    """Per-tile detections from the cached pass must equal the live pass."""
    out_dir = tmp_path / "gst_gate"
    report = _run_helper(out_dir, max_frames=16)

    assert report["status"] == "OK", (
        f"per-tile live-vs-cached deviations: {report.get('deviations')}\n"
        f"stdout:\n{report['_stdout']}\nstderr:\n{report['_stderr']}"
    )
    # Pass 2 must have served entirely from cache — proven by the absence of
    # any hailonet element in the pass-2 graph (the chip is never opened).
    assert report["pass2_served_from_cache"] is True, (
        "pass 2 was not chip-free: "
        f"pass2_pipeline_has_hailonet={report.get('pass2_pipeline_has_hailonet')}"
    )
    assert report["pass2_pipeline_has_hailonet"] is False
    # Sanity: we actually compared tiles and saw real detections (otherwise
    # an all-empty run would trivially "match").
    assert report["n_tiles_compared"] > 0, "no tiles compared"
    assert report["pass1_total_dets"] == report["pass2_total_dets"]
    assert report["pass1_total_dets"] > 0, (
        "no detections fired — gate would be vacuous; "
        "check HEF / postprocess wiring"
    )
