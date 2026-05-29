"""Tests for `hailo_tiling.telemetry.ulg.parse_ulg`.

The committed fixture `tiny.ulg` is a trimmed PX4 SITL log; see
`tiny.ulg.README.md` for provenance.
"""
from __future__ import annotations

from pathlib import Path

import pytest

import pyulog

from hailo_tiling.telemetry.ulg import parse_ulg


FIXTURE = Path(__file__).parent / "fixtures" / "telemetry" / "tiny.ulg"


def test_parse_ulg_smoke():
    rows = parse_ulg(FIXTURE)
    assert len(rows) > 0
    assert rows[0]["timestamp"] == 0.0
    assert rows[-1]["timestamp"] > 0.5


def test_parse_ulg_fields_round_trip():
    rows = parse_ulg(FIXTURE)
    # At least one of each domain field is populated.
    assert any(r["altitude_agl_m"] is not None for r in rows)
    assert any(r["velocity_world"] is not None for r in rows)
    assert any(r["attitude_quat"] is not None for r in rows)
    # Quaternion has 4 components when present.
    for r in rows:
        if r["attitude_quat"] is not None:
            assert len(r["attitude_quat"]) == 4
            break
    # Velocity has 3 components when present.
    for r in rows:
        if r["velocity_world"] is not None:
            assert len(r["velocity_world"]) == 3
            break


def test_parse_ulg_monotonic_timestamps():
    rows = parse_ulg(FIXTURE)
    for i in range(len(rows) - 1):
        assert rows[i]["timestamp"] <= rows[i + 1]["timestamp"], (
            f"row {i} timestamp {rows[i]['timestamp']} > "
            f"row {i+1} timestamp {rows[i+1]['timestamp']}"
        )


def test_parse_ulg_geo_present():
    rows = parse_ulg(FIXTURE)
    assert any(r["_geo"]["lat"] is not None for r in rows)
    assert any(r["_geo"]["lon"] is not None for r in rows)
    # Every row carries a _geo sidecar dict.
    for r in rows:
        assert "_geo" in r
        assert set(r["_geo"].keys()) >= {"lat", "lon", "alt_msl", "pitch", "roll"}


def _write_ulog_without_topic(src: Path, dst: Path, dropped: str) -> None:
    """Re-emit `src` keeping every topic except `dropped`.

    We use pyulog's read/write round-trip: load with a filter list that
    excludes the unwanted topic, then `write_ulog` back out. This is the
    same mechanism the test fixture itself was built with.
    """
    # Topics the parser cares about, minus the one we drop.
    keep = [
        "vehicle_local_position",
        "vehicle_attitude",
        "vehicle_air_data",
        "vehicle_gps_position",
        "vehicle_global_position",
        "vehicle_angular_velocity",
        "sensor_combined",
    ]
    keep = [t for t in keep if t != dropped]
    u = pyulog.ULog(str(src), message_name_filter_list=keep)
    u.write_ulog(str(dst))


def test_parse_ulg_missing_topic_no_crash(tmp_path):
    no_attitude = tmp_path / "no_attitude.ulg"
    _write_ulog_without_topic(FIXTURE, no_attitude, "vehicle_attitude")
    rows = parse_ulg(no_attitude)
    assert len(rows) > 0
    # vehicle_attitude is gone, so attitude_quat must be None for every row.
    for r in rows:
        assert r["attitude_quat"] is None, "expected attitude_quat=None when topic absent"
    # _geo.pitch / _geo.roll also depend on the quaternion -> None.
    for r in rows:
        assert r["_geo"]["pitch"] is None
        assert r["_geo"]["roll"] is None


def test_parse_ulg_malformed_raises_valueerror(tmp_path):
    """A non-ULog file is rejected with `ValueError`, chaining the underlying error."""
    bogus = tmp_path / "bogus.ulg"
    bogus.write_bytes(b"not a real ulog file")
    with pytest.raises(ValueError) as excinfo:
        parse_ulg(bogus)
    # The chained cause should be the pyulog exception.
    assert excinfo.value.__cause__ is not None
