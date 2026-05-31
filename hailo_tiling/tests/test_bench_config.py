"""Tests for the ablation bench config matrix (Plan 6 Task B1)."""
from __future__ import annotations

from hailo_tiling.bench.config import BenchConfig, default_matrix


def test_default_matrix_has_expected_rows():
    rows = default_matrix()
    by_name = {r.name: r for r in rows}

    # Static grid rows from the canonical run_pxt_bench static set.
    for name in ("1x1", "3x2", "6x4"):
        assert name in by_name, f"missing static row {name!r}; have {sorted(by_name)}"
        assert by_name[name].kind == "static"

    # Dynamic + lever rows.
    for name in ("dynamic", "dynamic+asahi", "dynamic+altitude_zoom"):
        assert name in by_name, f"missing dynamic row {name!r}; have {sorted(by_name)}"
        assert by_name[name].kind == "dynamic"

    # Exactly one reference row (the dense 12x9 GT).
    refs = [r for r in rows if r.is_reference]
    assert len(refs) == 1, f"expected exactly one is_reference row, got {len(refs)}"
    assert refs[0].kind == "static"
    assert refs[0].tiles_x == 12 and refs[0].tiles_y == 9


def test_static_rows_carry_grid_spec():
    rows = {r.name: r for r in default_matrix()}
    r = rows["3x2"]
    assert r.tiles_x == 3 and r.tiles_y == 2
    assert 0.0 <= r.overlap < 1.0
    # Dynamic rows carry no grid; static rows carry no scheduler kwargs.
    assert r.scheduler_kwargs == {}


def test_dynamic_rows_carry_scheduler_kwargs_and_lever_flags():
    rows = {r.name: r for r in default_matrix()}
    base = rows["dynamic"]
    assert base.kind == "dynamic"
    assert isinstance(base.scheduler_kwargs, dict)
    assert base.asahi is False and base.altitude_zoom is False

    asahi = rows["dynamic+asahi"]
    assert asahi.asahi is True and asahi.altitude_zoom is False

    az = rows["dynamic+altitude_zoom"]
    assert az.altitude_zoom is True and az.asahi is False


def test_benchconfig_names_are_unique():
    rows = default_matrix()
    names = [r.name for r in rows]
    assert len(names) == len(set(names)), f"duplicate config names: {names}"


def test_benchconfig_is_frozen_dataclass():
    cfg = BenchConfig(name="x", kind="static", tiles_x=2, tiles_y=2, overlap=0.25)
    import dataclasses

    assert dataclasses.is_dataclass(cfg)
    # frozen => mutating raises
    import pytest

    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.name = "y"  # type: ignore[misc]
