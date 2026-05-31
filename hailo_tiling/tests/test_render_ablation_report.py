"""Tests for scripts/render_ablation_into_report.py (Night-2 C2).

The renderer must splice committed ablation tables into the report between the
markers, invent nothing, and be idempotent. Tested via importlib (scripts/ is
not a package).
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
_RENDERER = _REPO_ROOT / "scripts" / "render_ablation_into_report.py"


def _load():
    spec = importlib.util.spec_from_file_location("render_ablation", _RENDERER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_splice_replaces_between_markers_only():
    r = _load()
    report = "HEAD\n<!-- BEGIN:ablation -->\nOLD\n<!-- END:ablation -->\nTAIL\n"
    out = r.splice(report, "NEWBLOCK")
    assert out.startswith("HEAD\n")
    assert out.endswith("TAIL\n")
    assert "OLD" not in out
    assert "NEWBLOCK" in out
    assert out.count(r._BEGIN) == 1 and out.count(r._END) == 1


def test_splice_is_idempotent():
    r = _load()
    report = "H\n<!-- BEGIN:ablation -->\nx\n<!-- END:ablation -->\nT\n"
    once = r.splice(report, "BLOCK")
    twice = r.splice(once, "BLOCK")
    assert once == twice


def test_splice_requires_markers():
    r = _load()
    with pytest.raises(SystemExit):
        r.splice("no markers here", "BLOCK")


def test_strip_table_header_drops_h1():
    r = _load()
    md = "# Ablation table\n\n- cache: x\n\n| a |\n|---|\n| 1 |\n"
    out = r._strip_table_header(md)
    assert "# Ablation table" not in out
    assert out.startswith("- cache: x")
    assert "| a |" in out


def test_render_block_uses_only_committed_tables():
    """The block reflects the real committed tables (the headline fov50 numbers
    must appear verbatim) and never fabricates a missing FOV."""
    r = _load()
    block = r.render_block()
    fov50 = r._table_path("fov50")
    if fov50.exists():
        # A known committed fov50 figure must round-trip verbatim.
        assert "12x9" in block
        assert "1.0000" in block  # reference recall
        assert "Clip 0026 — fov50" in block
    else:  # pragma: no cover
        assert "not yet generated" in block


def test_report_check_mode_passes_when_current():
    """After a render, --check mode must report up-to-date (exit 0)."""
    r = _load()
    if not r._REPORT.exists():
        pytest.skip("report not present")
    # render in place, then check.
    assert r.main([]) == 0
    assert r.main(["--check"]) == 0
