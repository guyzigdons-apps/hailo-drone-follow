"""Tests for the ``hailo-tiling-visualize`` CLI (Plan 7 Task 8).

Covers the ffmpeg invocation wrapper, ffprobe fps probing, the
``--dry-run`` ASS-emit path, and the missing-ffmpeg error message. The
end-to-end ffmpeg path is gated on ``shutil.which("ffmpeg")`` so the
suite stays runnable on hosts without ffmpeg installed.
"""
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

from hailo_tiling.cli import visualize as vis_mod
from hailo_tiling.cli.visualize import main


FIXTURES = Path(__file__).parent / "fixtures" / "telemetry"
TINY_VIDEO = FIXTURES / "tiny_video_120frames.mp4"
SRT_FIXTURE = FIXTURES / "tiny.srt"


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8") as fh:
        for row in rows:
            fh.write(json.dumps(row, sort_keys=True, separators=(",", ":")))
            fh.write("\n")


def _dialogue_count(ass: str) -> int:
    return sum(1 for ln in ass.splitlines() if ln.startswith("Dialogue:"))


def test_visualize_dry_run_emits_ass(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """``--dry-run`` must build + print the ASS string without invoking ffmpeg."""
    telemetry = tmp_path / "t.jsonl"
    _write_jsonl(telemetry, [
        {"timestamp": 0.0, "altitude_agl_m": 5.0},
        {"timestamp": 1.0, "altitude_agl_m": 10.0},
    ])
    # Block all subprocess calls so a stray invocation would fail the test.
    def _no_subprocess(*args, **kwargs):  # noqa: ANN001
        raise AssertionError("subprocess.run must not be called in --dry-run")
    monkeypatch.setattr(vis_mod.subprocess, "run", _no_subprocess)

    rc = main([
        "--video", str(TINY_VIDEO),
        "--telemetry", str(telemetry),
        "--output", str(tmp_path / "out.mp4"),
        "--fps", "30.0",
        "--dry-run",
    ])
    assert rc == 0
    captured = capsys.readouterr()
    assert captured.out.startswith("[Script Info]")
    assert "[Events]" in captured.out
    assert _dialogue_count(captured.out) > 0
    # Output file must NOT have been written by ffmpeg.
    assert not (tmp_path / "out.mp4").exists()


@pytest.mark.skipif(
    shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None,
    reason="ffmpeg/ffprobe not available on this host",
)
def test_visualize_invokes_ffmpeg_when_present(tmp_path: Path) -> None:
    """End-to-end: telemetry JSONL + video → annotated MP4 via real ffmpeg."""
    # Build a telemetry JSONL inline (skip the SRT importer to keep the test
    # focused on Task 8's surface).
    telemetry = tmp_path / "t.jsonl"
    _write_jsonl(telemetry, [
        {"timestamp": 0.0, "altitude_agl_m": 1.0,
         "_geo": {"lat": 0.0001, "lon": 0.0002}},
        {"timestamp": 1.0, "altitude_agl_m": 2.0,
         "_geo": {"lat": 0.0003, "lon": 0.0004}},
        {"timestamp": 2.0, "altitude_agl_m": 3.0,
         "_geo": {"lat": 0.0005, "lon": 0.0006}},
        {"timestamp": 3.0, "altitude_agl_m": 4.0,
         "_geo": {"lat": 0.0007, "lon": 0.0008}},
    ])
    output = tmp_path / "annotated.mp4"
    rc = main([
        "--video", str(TINY_VIDEO),
        "--telemetry", str(telemetry),
        "--output", str(output),
    ])
    assert rc == 0, "ffmpeg invocation must succeed"
    assert output.exists(), "output file must be written"
    assert output.stat().st_size > 0, "output file must be non-empty"

    # Verify it's a valid MP4 by probing the video codec_type via ffprobe.
    probe = subprocess.run(
        [
            "ffprobe", "-v", "error",
            "-select_streams", "v:0",
            "-show_entries", "stream=codec_type",
            "-of", "default=nw=1:nk=1",
            str(output),
        ],
        capture_output=True, text=True, check=False,
    )
    assert probe.returncode == 0, probe.stderr
    assert probe.stdout.strip() == "video"


def test_visualize_missing_ffmpeg_reports_clearly(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """No ffmpeg on PATH → clear error message, non-zero exit, no stack trace."""
    telemetry = tmp_path / "t.jsonl"
    _write_jsonl(telemetry, [{"timestamp": 0.0, "altitude_agl_m": 1.0}])

    # Force shutil.which to report ffmpeg as missing.
    real_which = shutil.which
    def _fake_which(cmd, *args, **kwargs):  # noqa: ANN001
        if cmd == "ffmpeg":
            return None
        return real_which(cmd, *args, **kwargs)
    monkeypatch.setattr(vis_mod.shutil, "which", _fake_which)

    rc = main([
        "--video", str(TINY_VIDEO),
        "--telemetry", str(telemetry),
        "--output", str(tmp_path / "out.mp4"),
        "--fps", "30.0",
    ])
    assert rc != 0
    captured = capsys.readouterr()
    assert "ffmpeg" in captured.err.lower()
    # Helpful hint, not a stack trace.
    assert "install" in captured.err.lower() or "--ffmpeg-path" in captured.err


def test_visualize_probes_fps_from_video(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """Probed ffprobe ``avg_frame_rate`` (``30/1``) drives ASS cue cadence."""
    telemetry = tmp_path / "t.jsonl"
    _write_jsonl(telemetry, [
        {"timestamp": 0.0, "altitude_agl_m": 1.0},
        {"timestamp": 2.0, "altitude_agl_m": 2.0},
    ])

    # Monkeypatch subprocess.run to fake ffprobe responses:
    #  - avg_frame_rate query returns "30/1" (exact, no float drift)
    #  - duration query returns "2.0"
    # Any other call raises (we shouldn't actually invoke ffmpeg in --dry-run).
    class _FakeResult:
        def __init__(self, stdout: str, returncode: int = 0, stderr: str = ""):
            self.stdout = stdout
            self.returncode = returncode
            self.stderr = stderr

    def _fake_run(cmd, *args, **kwargs):  # noqa: ANN001
        if not isinstance(cmd, (list, tuple)):
            raise AssertionError(f"unexpected cmd shape: {cmd!r}")
        # Identify by the value passed to -show_entries.
        joined = " ".join(str(c) for c in cmd)
        if "stream=avg_frame_rate" in joined:
            return _FakeResult("30/1\n")
        if "stream=duration" in joined:
            return _FakeResult("2.000000\n")
        raise AssertionError(f"unexpected subprocess invocation: {cmd!r}")

    monkeypatch.setattr(vis_mod.subprocess, "run", _fake_run)
    # Make sure shutil.which("ffprobe") returns a truthy path so _probe_fps
    # actually attempts the call.
    real_which = shutil.which
    def _fake_which(cmd, *args, **kwargs):  # noqa: ANN001
        if cmd == "ffprobe":
            return "/usr/bin/ffprobe"
        if cmd == "ffmpeg":
            return "/usr/bin/ffmpeg"
        return real_which(cmd, *args, **kwargs)
    monkeypatch.setattr(vis_mod.shutil, "which", _fake_which)

    rc = main([
        "--video", str(TINY_VIDEO),
        "--telemetry", str(telemetry),
        "--output", str(tmp_path / "out.mp4"),
        "--dry-run",
    ])
    assert rc == 0
    ass_text = capsys.readouterr().out

    # 30 fps * 2.0 s = 60 cues exactly (no float-drift risk).
    n_cues = _dialogue_count(ass_text)
    assert n_cues == 60, (
        f"expected 60 cues for 30 fps × 2.0 s, got {n_cues}"
    )
