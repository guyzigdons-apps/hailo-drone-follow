import json
import os
import shutil
import subprocess
import sys

import pytest

CLIP = "/home/giladn/Videos/Drone/Training/Car/DJI_20260614161040_0013_D.MP4"
HEF = "/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef"


def _have_device():
    if not (os.path.exists(CLIP) and os.path.exists(HEF)):
        return False
    if shutil.which("hailortcli") is None:
        return False
    r = subprocess.run(["hailortcli", "fw-control", "identify"],
                       capture_output=True, text=True)
    return r.returncode == 0


@pytest.mark.skipif(not _have_device(),
                    reason="requires Hailo device + 0013 clip + HEF")
def test_run_showcase_emits_frames_and_metrics(tmp_path):
    out = tmp_path / "showcase"
    cmd = [sys.executable, "-m", "tiling_lab.live.run_showcase",
           "--video", CLIP, "--out", str(out),
           "--frames", "300", "--fps", "60", "--target-class", "vehicle"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stderr

    frames = json.loads((out / "frames.json").read_text())
    assert frames["frames"], "no frames recorded"
    f0 = frames["frames"][0]
    assert set(["frame", "detections", "tiles"]).issubset(f0.keys())

    metrics = json.loads((out / "metrics.json").read_text())
    for k in ["frames", "wall_s", "achieved_fps", "mean_tiles_per_frame",
              "max_tiles_per_frame", "sustains_60fps", "source_fps"]:
        assert k in metrics
    # The spike is gone: max per-frame tiles is close to the mean.
    assert metrics["max_tiles_per_frame"] - metrics["mean_tiles_per_frame"] <= 2
