"""Tests for `hailo_tiling.telemetry.align.align_to_video`.

Covers the three Plan 7 Task 4 strategies (``video-start``, ``offset:<sec>``,
``video-creation``) plus error paths. The ``video-creation`` tests
monkeypatch ``subprocess.run`` so the suite never invokes ffprobe.
"""
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

from hailo_tiling.telemetry import align as align_module
from hailo_tiling.telemetry.align import align_to_video
from hailo_tiling.telemetry.srt import parse_srt


FIXTURE_SRT = Path(__file__).parent / "fixtures" / "telemetry" / "tiny.srt"


def _ulg_shaped_row(timestamp: float) -> dict:
    """Build a synthetic ULG-shape row (no SRT marker on ``_geo``)."""
    return {
        "timestamp": timestamp,
        "altitude_agl_m": 10.0,
        "yaw_rate_rad_s": 0.0,
        "velocity_world": [0.0, 0.0, 0.0],
        "attitude_quat": [0.0, 0.0, 0.0, 1.0],
        "_geo": {
            "lat": 0.0,
            "lon": 0.0,
            "alt_msl": 100.0,
            "pitch": None,
            "roll": None,
        },
    }


def test_align_video_start_identity():
    rows = [_ulg_shaped_row(0.0), _ulg_shaped_row(0.5), _ulg_shaped_row(1.25)]
    out = align_to_video(rows, Path("/tmp/does_not_exist.mp4"), "video-start")
    # Identity: same list object, no shift applied.
    assert out is rows
    assert [r["timestamp"] for r in out] == [0.0, 0.5, 1.25]


def test_align_offset_positive():
    rows = [_ulg_shaped_row(0.0), _ulg_shaped_row(1.0), _ulg_shaped_row(2.5)]
    out = align_to_video(rows, Path("/tmp/does_not_exist.mp4"), "offset:1.5")
    assert [r["timestamp"] for r in out] == [1.5, 2.5, 4.0]
    # Input must not be mutated.
    assert [r["timestamp"] for r in rows] == [0.0, 1.0, 2.5]


def test_align_offset_negative():
    """Negative timestamps are allowed in the output (snapshot() handles them)."""
    rows = [_ulg_shaped_row(0.0), _ulg_shaped_row(1.0), _ulg_shaped_row(2.5)]
    out = align_to_video(rows, Path("/tmp/does_not_exist.mp4"), "offset:-0.5")
    assert [r["timestamp"] for r in out] == [-0.5, 0.5, 2.0]


def test_align_video_creation_from_srt(monkeypatch, tmp_path):
    """Offset = SRT_first_iso - video_creation_time."""
    # Stage the SRT next to a (fake) video so the sibling-lookup finds it.
    srt_dst = tmp_path / "clip.srt"
    shutil.copy(FIXTURE_SRT, srt_dst)
    video_path = tmp_path / "clip.mp4"
    video_path.write_bytes(b"")  # ffprobe is monkeypatched; content is ignored

    # Known SRT first-block ISO: 2026-04-30 10:47:32.749 (see tiny.srt L4).
    # Pick a video creation_time 2 seconds earlier so the offset is +2.000s.
    fake_creation = "2026-04-30T10:47:30.749000Z"

    def fake_run(cmd, *args, **kwargs):  # noqa: ARG001
        assert cmd[0] == "ffprobe", f"unexpected command {cmd!r}"
        result = subprocess.CompletedProcess(
            args=cmd,
            returncode=0,
            stdout=json.dumps(
                {"format": {"tags": {"creation_time": fake_creation}}}
            ),
            stderr="",
        )
        return result

    monkeypatch.setattr(align_module.subprocess, "run", fake_run)

    rows = parse_srt(srt_dst)
    assert rows, "fixture should yield rows"
    original = [r["timestamp"] for r in rows]

    out = align_to_video(rows, video_path, "video-creation")

    expected_offset = 2.0
    shifted = [r["timestamp"] for r in out]
    assert len(shifted) == len(original)
    for orig, new in zip(original, shifted):
        assert new == pytest.approx(orig + expected_offset, abs=1e-6)


def test_align_video_creation_rejects_ulg(tmp_path):
    """ULG rows (no SRT _agl_source marker) → ValueError."""
    rows = [_ulg_shaped_row(0.0), _ulg_shaped_row(1.0)]
    video_path = tmp_path / "clip.mp4"
    video_path.write_bytes(b"")

    with pytest.raises(ValueError, match="SRT rows with absolute timestamps"):
        align_to_video(rows, video_path, "video-creation")


def test_align_invalid_strategy():
    rows = [_ulg_shaped_row(0.0)]
    with pytest.raises(ValueError, match="unknown strategy"):
        align_to_video(rows, Path("/tmp/does_not_exist.mp4"), "bogus")
