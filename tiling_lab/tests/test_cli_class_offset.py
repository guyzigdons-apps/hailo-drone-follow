"""Regression guard: every HEF backend constructed in a `tiling_lab.cli`
module must pass `class_offset=` explicitly.

The yolov8n_4_classes_vga HEF emits person=1/vehicle=2 ONLY when the backend is
built with `class_offset=1` (unified convention, single source of truth in
`hailo_tiling/classes.py`). Two weekend bugs (591f557, eae88c1) came from CLI
construction sites silently defaulting to offset 0, shifting person -> 0. This
test ast-walks every module under `tiling_lab/cli/` and fails if any call whose
callee name ends with "HefBackend" omits the `class_offset` keyword.
"""
import ast
from pathlib import Path

import pytest

CLI_DIR = Path(__file__).resolve().parents[1] / "cli"


def _callee_name(func: ast.expr) -> str | None:
    """Last name segment of a Call's func (`HefBackend`, `mod.HefBackend`)."""
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return None


def _hef_backend_calls():
    """Yield (path, lineno, has_class_offset) for every *HefBackend(...) call
    constructed in a tiling_lab.cli module."""
    for py in sorted(CLI_DIR.glob("*.py")):
        tree = ast.parse(py.read_text(), filename=str(py))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            name = _callee_name(node.func)
            if name and name.endswith("HefBackend"):
                has_offset = any(kw.arg == "class_offset" for kw in node.keywords)
                yield py, node.lineno, has_offset


def test_cli_has_hef_backend_construction_sites():
    """Sanity: the scan finds at least one construction site (else the guard is
    silently vacuous)."""
    calls = list(_hef_backend_calls())
    assert calls, "no *HefBackend(...) calls found under tiling_lab/cli/"


def test_every_cli_hef_backend_passes_class_offset():
    missing = [
        f"{py.name}:{lineno}"
        for py, lineno, has_offset in _hef_backend_calls()
        if not has_offset
    ]
    assert not missing, (
        "tiling_lab.cli HEF backend construction sites missing class_offset= : "
        + ", ".join(missing)
    )
