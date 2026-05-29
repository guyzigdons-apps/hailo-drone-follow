"""Plan 7 Task 9 — determinism + ``--version`` regression tests.

These cover the hardening requirements from the plan:

- ``parse_ulg`` / ``parse_srt`` plus the JSONL writer in
  ``hailo-tiling-import-telemetry`` produce **byte-identical** output across
  two runs of the same input (spec §13 reproducibility requirement).
- Both CLIs (``hailo-tiling-import-telemetry`` and
  ``hailo-tiling-visualize``) expose a ``--version`` flag that exits 0 and
  prints the ``hailo_tiling`` package version.

The tests are intentionally cheap — they reuse the tiny committed fixtures
from Plan 7 Tasks 2 / 3 — so the full pytest run stays well under a second.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from hailo_tiling import __version__ as PKG_VERSION
from hailo_tiling.cli.import_telemetry import main as import_main
from hailo_tiling.cli.visualize import main as visualize_main


FIXTURES = Path(__file__).parent / "fixtures" / "telemetry"
ULG_FIXTURE = FIXTURES / "tiny.ulg"
SRT_FIXTURE = FIXTURES / "tiny.srt"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_import_ulg_is_deterministic(tmp_path: Path) -> None:
    """Two ULG imports of the same fixture produce byte-identical JSONL."""
    out_a = tmp_path / "a.jsonl"
    out_b = tmp_path / "b.jsonl"
    rc_a = import_main(["--ulg", str(ULG_FIXTURE), "--output", str(out_a)])
    rc_b = import_main(["--ulg", str(ULG_FIXTURE), "--output", str(out_b)])
    assert rc_a == 0
    assert rc_b == 0
    assert out_a.read_bytes() == out_b.read_bytes(), (
        "ULG -> JSONL output is not byte-identical across two runs"
    )
    # Hash check is redundant given the bytes equality above but serves
    # as the explicit Plan 7 Task 9 acceptance criterion ("hash both
    # outputs, assert SHA-256 equality").
    assert _sha256(out_a) == _sha256(out_b)


def test_import_srt_is_deterministic(tmp_path: Path) -> None:
    """Two SRT imports of the same fixture produce byte-identical JSONL."""
    out_a = tmp_path / "a.jsonl"
    out_b = tmp_path / "b.jsonl"
    rc_a = import_main(["--srt", str(SRT_FIXTURE), "--output", str(out_a)])
    rc_b = import_main(["--srt", str(SRT_FIXTURE), "--output", str(out_b)])
    assert rc_a == 0
    assert rc_b == 0
    assert out_a.read_bytes() == out_b.read_bytes(), (
        "SRT -> JSONL output is not byte-identical across two runs"
    )
    assert _sha256(out_a) == _sha256(out_b)


def test_cli_import_telemetry_version_flag(capsys: pytest.CaptureFixture) -> None:
    """``--version`` exits 0 and prints the package version."""
    with pytest.raises(SystemExit) as exc:
        import_main(["--version"])
    assert exc.value.code == 0
    captured = capsys.readouterr()
    # argparse's version action writes to stdout by default.
    combined = captured.out + captured.err
    assert PKG_VERSION in combined, (
        f"expected {PKG_VERSION!r} in CLI --version output, got "
        f"stdout={captured.out!r} stderr={captured.err!r}"
    )


def test_cli_visualize_version_flag(capsys: pytest.CaptureFixture) -> None:
    """``--version`` exits 0 and prints the package version."""
    with pytest.raises(SystemExit) as exc:
        visualize_main(["--version"])
    assert exc.value.code == 0
    captured = capsys.readouterr()
    combined = captured.out + captured.err
    assert PKG_VERSION in combined, (
        f"expected {PKG_VERSION!r} in CLI --version output, got "
        f"stdout={captured.out!r} stderr={captured.err!r}"
    )
