"""Pre-flight cache bit-exact E2E validator.

Three-mode comparison gating Plan 5 (GStreamer cache plugins):

  - Mode A: HefBackend only (chip).
  - Mode B: CachingBackend(HefBackend), two passes — pass 2 must serve every
    crop from the SQLite cache without touching the chip and must return
    detections identical to pass 1.
  - Mode C: ReplayBackend reading the populated cache (chip-free) — must
    match Mode A exactly.

Plus two regression-style sanity checks:
  - floor-quantise hit (CachingBackend with quantise=N>1)
  - CacheMissError surfaced loudly by ReplayBackend on an empty cache

This test SKIPS unless one of the following is true:
  - the env var HAILO_CHIP=1 is set, OR
  - pytest is invoked with --chip

The chip-free unit-test suite already covers MockBackend wiring. This file
exists to prove the LIVE chip path is deterministic across the cache layer,
which can only be checked against real hardware.

Run:
    source setup_env.sh
    HAILO_CHIP=1 pytest tests/integration/test_cache_bit_exact_e2e.py -v
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
_BASELINE_DIR = _REPO_ROOT / "tiling_benchmark" / "runs" / "cache_e2e_baseline"
_HELPER = _REPO_ROOT / "scripts" / "cache_bit_exact_e2e.py"


def pytest_configure(config):  # pragma: no cover - pytest plugin hook
    # Allow `pytest --chip` as a synonym for HAILO_CHIP=1.
    config.addinivalue_line(
        "markers", "chip: requires a Hailo chip (real-hardware integration test)",
    )


def _chip_requested() -> bool:
    if os.environ.get("HAILO_CHIP") == "1":
        return True
    # Inspect raw argv since --chip is a custom flag we don't register on the
    # parser (avoids polluting the project's pytest config).
    return "--chip" in sys.argv


def _chip_available() -> tuple[bool, str]:
    """Best-effort hardware probe via hailortcli."""
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
        pytest.skip(f"validator video missing: {_VIDEO}")
    if not _HEF.exists():
        pytest.skip(f"validator HEF missing: {_HEF}")
    if not _HELPER.exists():
        pytest.skip(f"helper missing: {_HELPER}")


def _run_helper(out_dir: Path, max_frames: int = 16) -> dict:
    """Invoke the helper as a subprocess; parse diff_report.json."""
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable, str(_HELPER),
        "--video", str(_VIDEO),
        "--hef", str(_HEF),
        "--out-dir", str(out_dir),
        "--max-frames", str(max_frames),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
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
    # Attach helper output for debugging on failure.
    report["_stdout"] = proc.stdout
    report["_stderr"] = proc.stderr
    report["_returncode"] = proc.returncode
    return report


def test_cache_bit_exact_three_modes(chip_gate, tmp_path):
    """Run the three-mode validator and assert no deviations."""
    out_dir = tmp_path / "cache_e2e"
    report = _run_helper(out_dir, max_frames=16)
    assert report["status"] == "OK", (
        f"validator deviations: {report['deviations']}\n"
        f"stdout:\n{report['_stdout']}\nstderr:\n{report['_stderr']}"
    )
    # Belt-and-braces — re-assert the specific invariants the helper checks.
    assert report["mode_b_pass2_chip_crop_calls"] == 0, (
        "Mode B pass 2 invoked the chip — cache hits should be served without "
        f"chip access (got {report['mode_b_pass2_chip_crop_calls']} crop calls)"
    )
    assert report["floor_quantise_ok"], report["floor_quantise_msg"]
    assert report["cache_miss_ok"], report["cache_miss_msg"]


def test_committed_baseline_matches_fresh_run(chip_gate, tmp_path):
    """The fresh helper run must produce JSONs identical to the committed baseline.

    This catches any regression in HefBackend / CachingBackend / ReplayBackend
    that would shift the per-crop detection outputs between commits.
    """
    out_dir = tmp_path / "cache_e2e"
    report = _run_helper(out_dir, max_frames=16)
    assert report["status"] == "OK", report["deviations"]

    for fname in ("mode_a_hef.json", "mode_b_caching.json", "mode_c_replay.json"):
        baseline = _BASELINE_DIR / fname
        fresh = out_dir / fname
        if not baseline.exists():
            pytest.skip(
                f"baseline {baseline} not committed yet — run the helper "
                f"once and commit the outputs to enable this regression test",
            )
        b_doc = json.loads(baseline.read_text())
        f_doc = json.loads(fresh.read_text())
        # Strip ephemeral fields if any (none currently).
        assert b_doc == f_doc, f"committed baseline {fname} drifted from fresh run"
