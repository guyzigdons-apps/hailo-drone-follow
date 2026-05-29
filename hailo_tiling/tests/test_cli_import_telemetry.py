"""Tests for the ``hailo-tiling-import-telemetry`` CLI (Plan 7 Task 5).

These exercise the wiring layer between ``parse_ulg`` / ``parse_srt``,
``align_to_video``, and the JSONL writer. The parsers and aligner have
their own unit-test coverage; here we just verify the CLI surface
behaves the way the plan specifies.
"""
from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest

from hailo_tiling.cli.import_telemetry import main


FIXTURES = Path(__file__).parent / "fixtures" / "telemetry"
ULG_FIXTURE = FIXTURES / "tiny.ulg"
SRT_FIXTURE = FIXTURES / "tiny.srt"


def _read_jsonl(path: Path) -> list[dict]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def test_cli_ulg_round_trip(tmp_path: Path) -> None:
    out = tmp_path / "out.jsonl"
    rc = main(["--ulg", str(ULG_FIXTURE), "--output", str(out)])
    assert rc == 0
    rows = _read_jsonl(out)
    assert len(rows) >= 1
    assert any(r.get("altitude_agl_m") is not None for r in rows)


def test_cli_srt_round_trip(tmp_path: Path) -> None:
    out = tmp_path / "out.jsonl"
    rc = main(["--srt", str(SRT_FIXTURE), "--output", str(out)])
    assert rc == 0
    rows = _read_jsonl(out)
    assert len(rows) >= 1
    assert any(r.get("altitude_agl_m") is not None for r in rows)


def test_cli_requires_one_source(tmp_path: Path) -> None:
    # argparse exits with SystemExit(2) on a missing required argument.
    with pytest.raises(SystemExit) as exc:
        main(["--output", str(tmp_path / "out.jsonl")])
    assert exc.value.code != 0


def test_cli_rejects_both_sources(tmp_path: Path) -> None:
    with pytest.raises(SystemExit) as exc:
        main([
            "--ulg", str(ULG_FIXTURE),
            "--srt", str(SRT_FIXTURE),
            "--output", str(tmp_path / "out.jsonl"),
        ])
    assert exc.value.code != 0


def test_cli_default_output_path(tmp_path: Path) -> None:
    # Copy the fixture so the default-output writes into tmp_path, not the
    # committed fixtures directory.
    local_srt = tmp_path / "tiny.srt"
    shutil.copy(SRT_FIXTURE, local_srt)
    rc = main(["--srt", str(local_srt)])
    assert rc == 0
    expected = tmp_path / "tiny.srt.jsonl"
    assert expected.exists(), f"default output {expected} not written"
    rows = _read_jsonl(expected)
    assert len(rows) >= 1


def test_cli_strip_geo_removes_sidecar(tmp_path: Path) -> None:
    out = tmp_path / "out.jsonl"
    rc = main(["--srt", str(SRT_FIXTURE), "--output", str(out), "--strip-geo"])
    assert rc == 0
    rows = _read_jsonl(out)
    assert rows, "expected at least one row"
    assert all("_geo" not in r for r in rows), (
        "--strip-geo should drop the _geo key from every row"
    )
