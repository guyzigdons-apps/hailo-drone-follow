"""On-chip crop-provenance check for hailocachewriter (Phase 14).

Proves that ``hailocachewriter mode=tile_cache``, sitting on the cropped
branch of a ``hailotilecropper_dynamic`` pipeline (after ``hailofilter``,
before ``hailotileaggregator.sink_1``), records the REAL per-tile source
crop rect read from each tile buffer's ``HailoROI`` — instead of the
pre-Phase-14 full-frame fallback ``(0,0,W,H)``.

The crop key is the cropper's exact pixel rule (TAPPAS
``HailoMat::get_bounding_rect``): ``rect.x = (int)(xmin*W)``, ``rect.w =
(int)(width*W)`` clamped to ``W-x`` (and likewise y/h), where ``W``/``H``
are the dimensions of the buffer the writer sees on the cropped branch.
``hailotilecropper_dynamic`` resizes every tile to the HEF input size, so
the writer sees 640x480 buffers and the recorded crops are the 3x2 grid
rects scaled into that 640x480 space.

This test SKIPS unless HAILO_CHIP=1 (or pytest is invoked with --chip),
mirroring tests/integration/test_cache_bit_exact_e2e.py. It also skips if
the gst-hailo-cache plugin hasn't been built. It is a hardware-only test:
the per-tile ROI bbox only exists once the cropper actually runs.

Run:
    source setup_env.sh
    HAILO_CHIP=1 pytest tests/integration/test_writer_crop_provenance.py -v
"""
from __future__ import annotations

import os
import shutil
import sqlite3
import subprocess
import sys
import textwrap
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
_POST_CFG = Path("/usr/local/hailo/resources/json/hailo_4_classes.json")
_PLUGIN_DIR = _REPO_ROOT / "gst-hailo-cache" / "build" / "src"
_PLUGIN_SO = _PLUGIN_DIR / "libgsthailocache.so"

# Cropped-branch model input size (hailotilecropper_dynamic resizes every
# tile to this before the HEF; the writer's caps therefore report it).
_MODEL_W = 640
_MODEL_H = 480

_TILES_X, _TILES_Y = 3, 2


def pytest_configure(config):  # pragma: no cover - pytest plugin hook
    config.addinivalue_line(
        "markers", "chip: requires a Hailo chip (real-hardware integration test)",
    )


def _chip_requested() -> bool:
    if os.environ.get("HAILO_CHIP") == "1":
        return True
    return "--chip" in sys.argv


def _chip_available() -> tuple[bool, str]:
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
    for p in (_VIDEO, _HEF, _POST_SO, _POST_CFG, _PLUGIN_SO):
        if not p.exists():
            pytest.skip(f"required artifact missing: {p}")


def _expected_grid_crops() -> set[tuple[int, int, int, int]]:
    """Replicate the cropper's exact pixel rule over the 3x2 normalized grid
    in the cropped-branch's 640x480 space.

    Normalized grid (overlap 0): each tile is 1/nx wide, 1/ny tall. The
    cropper truncates ``norm*dim`` toward zero, then clamps width/height to
    the residual frame extent — see read_tile_crop_rect_ in
    gst_hailocachewriter.cpp / TAPPAS HailoMat::get_bounding_rect.
    """
    out: set[tuple[int, int, int, int]] = set()
    tw_n = 1.0 / _TILES_X
    th_n = 1.0 / _TILES_Y
    for gy in range(_TILES_Y):
        for gx in range(_TILES_X):
            x = int((gx * tw_n) * _MODEL_W)
            y = int((gy * th_n) * _MODEL_H)
            w = min(int(tw_n * _MODEL_W), _MODEL_W - x)
            h = min(int(th_n * _MODEL_H), _MODEL_H - y)
            out.add((x, y, w, h))
    return out


def _grid_static() -> str:
    rects = []
    tw, th = 1.0 / _TILES_X, 1.0 / _TILES_Y
    for gy in range(_TILES_Y):
        for gx in range(_TILES_X):
            rects.append(f"{gx*tw:.6f},{gy*th:.6f},{tw:.6f},{th:.6f}")
    return ";".join(rects)


_RUNNER = textwrap.dedent(
    """
    import os, sys
    os.environ.setdefault("GST_PLUGIN_FEATURE_RANK",
        "vaapidecodebin:NONE,vaapidecode:NONE,vaapih264dec:NONE,vaapipostproc:NONE")
    import gi; gi.require_version('Gst','1.0'); from gi.repository import Gst, GLib
    Gst.init(None)
    video, hef, so, cfg, static, out = sys.argv[1:7]
    # Optional extra writer props (Task 4: source-pixel provenance). Passed as
    # a single string of "key=value key=value"; appended verbatim onto the
    # hailocachewriter element so the no-prop test stays byte-identical.
    extra = sys.argv[7] if len(sys.argv) > 7 else ""
    inner = (f"queue ! hailonet hef-path={hef} batch-size=1 nms-score-threshold=0.3 ! "
             f"queue ! hailofilter so-path={so} function-name=filter config-path={cfg} qos=false ! "
             f"hailocachewriter mode=tile_cache output-file={out} batch-size=8 flush-interval-ms=50 {extra}")
    pipe = (f"filesrc location={video} ! decodebin ! videoconvert ! video/x-raw,format=RGB ! "
            f"videoscale ! video/x-raw,width=1280,height=800 ! "
            f"hailotilecropper_dynamic name=cr internal-offset=true tiling-mode=single-scale "
            f"tiles-static=\\"{static}\\" "
            f"hailotileaggregator name=agg flatten-detections=true iou-threshold=0.3 "
            f"cr. ! queue ! agg.sink_0 "
            f"cr. ! video/x-raw,format=RGB ! {inner} ! agg.sink_1 "
            f"agg. ! queue ! fakesink sync=false")
    p = Gst.parse_launch(pipe)
    loop = GLib.MainLoop(); n = {"f": 0}
    def cb(pad, info):
        n["f"] += 1
        if n["f"] >= 12: loop.quit()
        return Gst.PadProbeReturn.OK
    p.get_by_name("agg").get_static_pad("src").add_probe(Gst.PadProbeType.BUFFER, cb)
    b = p.get_bus(); b.add_signal_watch()
    b.connect("message", lambda bb, m: loop.quit()
              if m.type in (Gst.MessageType.ERROR, Gst.MessageType.EOS) else None)
    p.set_state(Gst.State.PLAYING)
    GLib.timeout_add_seconds(30, lambda: loop.quit() or True)
    loop.run()
    # NULL transition runs the writer's stop() -> synchronous final flush
    # + thread join, so the SQLite file is complete once this returns.
    p.set_state(Gst.State.NULL)
    print("frames", n["f"])
    """
)


def _run_pipeline(tmp_path, static: str, out_db: Path, extra: str = "") -> subprocess.CompletedProcess:
    runner = tmp_path / "runner.py"
    runner.write_text(_RUNNER)
    env = dict(os.environ)
    env["GST_PLUGIN_PATH"] = f"{_PLUGIN_DIR}:{env.get('GST_PLUGIN_PATH', '')}"
    argv = [sys.executable, str(runner), str(_VIDEO), str(_HEF), str(_POST_SO),
            str(_POST_CFG), static, str(out_db)]
    if extra:
        argv.append(extra)
    return subprocess.run(
        argv, capture_output=True, text=True, timeout=180, env=env,
    )


def test_writer_records_real_per_tile_crops(chip_gate, tmp_path):
    out_db = tmp_path / "writer_tile.sqlite3"
    proc = _run_pipeline(tmp_path, _grid_static(), out_db)
    assert out_db.exists(), (
        "writer produced no SQLite file\n"
        f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
    )

    con = sqlite3.connect(str(out_db))
    try:
        crops = set(con.execute(
            "SELECT DISTINCT crop_x, crop_y, crop_w, crop_h FROM detections"
        ).fetchall())
        n_rows = con.execute("SELECT COUNT(*) FROM detections").fetchone()[0]
    finally:
        con.close()

    # The full-frame fallback (the pre-Phase-14 behaviour) would record a
    # single (0,0,W,H) crop. Phase 14 must record the per-tile grid.
    assert n_rows > 0, f"no rows written\nstderr:\n{proc.stderr}"
    assert crops != {(0, 0, _MODEL_W, _MODEL_H)}, (
        "writer still recorded only the full-frame fallback crop — "
        "per-tile provenance did not engage"
    )
    expected = _expected_grid_crops()
    assert crops == expected, (
        f"recorded crops {sorted(crops)} != expected 3x2 grid {sorted(expected)}\n"
        f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
    )


# Source-pixel provenance (Task 4): when source-width/height are supplied,
# crop keys must land in SOURCE-video space, NOT the cropped-branch caps
# (640x480). A single tile 0,0,0.4,0.5 over a declared 3840x2160 source
# yields (0, 0, int(0.4*3840)=1536, int(0.5*2160)=1080) by the same TAPPAS
# truncate-then-clamp rule. The resize envelope is stamped into `meta`.
_SOURCE_W, _SOURCE_H = 3840, 2160
_SRC_TILE = "0.000000,0.000000,0.400000,0.500000"


def _expected_source_crop() -> tuple[int, int, int, int]:
    x = int(0.0 * _SOURCE_W)
    y = int(0.0 * _SOURCE_H)
    w = min(int(0.4 * _SOURCE_W), _SOURCE_W - x)
    h = min(int(0.5 * _SOURCE_H), _SOURCE_H - y)
    return (x, y, w, h)


def test_writer_records_source_pixel_crops_and_meta(chip_gate, tmp_path):
    out_db = tmp_path / "writer_source.sqlite3"
    extra = (f"source-width={_SOURCE_W} source-height={_SOURCE_H} "
             f"resize-mode=stretch hef-sha=deadbeef")
    proc = _run_pipeline(tmp_path, _SRC_TILE, out_db, extra=extra)
    assert out_db.exists(), (
        "writer produced no SQLite file\n"
        f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
    )

    con = sqlite3.connect(str(out_db))
    try:
        crops = set(con.execute(
            "SELECT DISTINCT crop_x, crop_y, crop_w, crop_h FROM detections"
        ).fetchall())
        n_rows = con.execute("SELECT COUNT(*) FROM detections").fetchone()[0]
        meta = dict(con.execute("SELECT k, v FROM meta").fetchall())
    finally:
        con.close()

    assert n_rows > 0, f"no rows written\nstderr:\n{proc.stderr}"

    # Crops are in 3840x2160 space, NOT the 640x480 cropped-branch caps.
    expected_crop = _expected_source_crop()
    assert expected_crop in crops, (
        f"expected source-pixel crop {expected_crop} not found; got {sorted(crops)}\n"
        f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
    )
    # Sanity: nothing was recorded in the cropped-branch 640x480 space.
    assert (0, 0, _MODEL_W, _MODEL_H) not in crops, (
        "writer recorded a 640x480-space crop despite source-width/height being set"
    )

    # Resize envelope stamped into the meta table.
    assert meta.get("video_w") == str(_SOURCE_W), f"meta={meta}"
    assert meta.get("video_h") == str(_SOURCE_H), f"meta={meta}"
    assert meta.get("resize_mode") == "stretch", f"meta={meta}"
    assert meta.get("hef_sha") == "deadbeef", f"meta={meta}"
    assert meta.get("interpolation") == "linear", f"meta={meta}"
    # dst dims fall back to the cropped-branch caps (the network input).
    assert meta.get("dst_w") == str(_MODEL_W), f"meta={meta}"
    assert meta.get("dst_h") == str(_MODEL_H), f"meta={meta}"
