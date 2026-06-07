"""Dependency rules between top-level packages (spec 2026-06-07).

hailo_tiling  -> nothing internal
drone_follow  -> hailo_tiling only
tiling_lab    -> drone_follow + hailo_tiling
nothing       -> tiling_lab / tiling_benchmark
"""
import ast
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
INTERNAL = {"drone_follow", "hailo_tiling", "tiling_lab", "tiling_benchmark", "dynamic_tiling"}
ALLOWED = {
    "hailo_tiling": set(),
    "drone_follow": {"hailo_tiling"},
    "tiling_lab": {"drone_follow", "hailo_tiling"},
}


def _imports(pkg: str):
    for py in (REPO / pkg).rglob("*.py"):
        tree = ast.parse(py.read_text(), filename=str(py))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for a in node.names:
                    yield py, a.name.split(".")[0]
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                yield py, node.module.split(".")[0]


@pytest.mark.parametrize("pkg", sorted(ALLOWED))
def test_package_respects_dependency_rules(pkg):
    bad = [(str(f.relative_to(REPO)), m) for f, m in _imports(pkg)
           if m in INTERNAL and m != pkg and m not in ALLOWED[pkg]]
    assert not bad, f"{pkg} has forbidden internal imports: {bad}"


def test_dynamic_tiling_is_gone():
    assert not (REPO / "dynamic_tiling").exists()
