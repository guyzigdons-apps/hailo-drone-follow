"""RecordedTelemetry — JSONL replay provider."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from hailo_tiling.telemetry import RecordedTelemetry, TelemetrySnapshot


def _write_jsonl(tmp_path: Path, rows: list[dict]) -> Path:
    p = tmp_path / "telem.jsonl"
    p.write_text("\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8")
    return p


def test_returns_nearest_le_record(tmp_path):
    rows = [
        {"timestamp": 0.0, "altitude_agl_m": 5.0},
        {"timestamp": 1.0, "altitude_agl_m": 7.5},
        {"timestamp": 2.0, "altitude_agl_m": 10.0},
    ]
    p = RecordedTelemetry.from_path(_write_jsonl(tmp_path, rows))
    assert p.snapshot(0.0).altitude_agl_m == 5.0
    assert p.snapshot(0.5).altitude_agl_m == 5.0
    assert p.snapshot(1.0).altitude_agl_m == 7.5
    assert p.snapshot(1.9).altitude_agl_m == 7.5
    assert p.snapshot(2.0).altitude_agl_m == 10.0
    assert p.snapshot(99.0).altitude_agl_m == 10.0


def test_returns_null_before_first_record(tmp_path):
    rows = [{"timestamp": 5.0, "altitude_agl_m": 10.0}]
    p = RecordedTelemetry.from_path(_write_jsonl(tmp_path, rows))
    s = p.snapshot(0.0)
    assert s.altitude_agl_m is None
    assert s.timestamp == 0.0


def test_velocity_and_attitude_round_trip(tmp_path):
    rows = [{
        "timestamp": 0.0,
        "velocity_world": [1.0, -2.0, 0.5],
        "attitude_quat": [1.0, 0.0, 0.0, 0.0],
        "yaw_rate_rad_s": 0.1,
    }]
    p = RecordedTelemetry.from_path(_write_jsonl(tmp_path, rows))
    s = p.snapshot(0.0)
    assert s.velocity_world == (1.0, -2.0, 0.5)
    assert s.attitude_quat == (1.0, 0.0, 0.0, 0.0)
    assert s.yaw_rate_rad_s == 0.1


def test_skips_malformed_lines(tmp_path):
    p = tmp_path / "bad.jsonl"
    p.write_text(
        '{"timestamp": 0.0, "altitude_agl_m": 1.0}\n'
        'not a json line\n'
        '\n'
        '{"timestamp": 1.0, "altitude_agl_m": 2.0}\n',
        encoding="utf-8",
    )
    rt = RecordedTelemetry.from_path(p)
    assert rt.snapshot(0.0).altitude_agl_m == 1.0
    assert rt.snapshot(1.0).altitude_agl_m == 2.0


def test_empty_file_returns_null_snapshot(tmp_path):
    p = tmp_path / "empty.jsonl"
    p.write_text("", encoding="utf-8")
    rt = RecordedTelemetry.from_path(p)
    s = rt.snapshot(1.0)
    assert s.altitude_agl_m is None
    assert s.timestamp == 1.0


def test_requires_sorted_or_sorts_on_load(tmp_path):
    """Out-of-order rows must still produce monotonic-friendly lookups."""
    rows = [
        {"timestamp": 2.0, "altitude_agl_m": 20.0},
        {"timestamp": 0.0, "altitude_agl_m": 5.0},
        {"timestamp": 1.0, "altitude_agl_m": 10.0},
    ]
    rt = RecordedTelemetry.from_path(_write_jsonl(tmp_path, rows))
    assert rt.snapshot(0.5).altitude_agl_m == 5.0
    assert rt.snapshot(1.5).altitude_agl_m == 10.0
