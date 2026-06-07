"""Ablation bench config matrix (Plan 6 Task B1).

A :class:`BenchConfig` describes one ablation row: a tiling strategy (static
grid or dynamic scheduler), its budget, and the lever flags. :func:`default_matrix`
returns the v1 rows used for the tiling-paper ablation table.

Static rows mirror the canonical grid set in
``tiling_benchmark/run_pxt_bench.py`` (1x1, 2x2, 3x2, 3x3, 4x3, 6x4, 8x6, all
at 0.25 overlap) plus the dense ``12x9`` ground-truth reference. Dynamic rows
exercise the existing ``hailo_tiling.dynamic.scheduler.TileScheduler`` with the two
levers (ASAHI adaptive slice sizing, altitude-gated zoom) toggled on
individually.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

Kind = Literal["static", "dynamic"]


@dataclass(frozen=True)
class BenchConfig:
    """One ablation row.

    Attributes:
        name: unique row label (e.g. ``"3x2"``, ``"dynamic+asahi"``).
        kind: ``"static"`` (fixed grid) or ``"dynamic"`` (scheduler-driven).
        tiles_x / tiles_y / overlap: grid spec for ``static`` rows (ignored for
            ``dynamic``). ``overlap`` is the symmetric x/y overlap fraction.
        scheduler_kwargs: kwargs for ``TileScheduler`` for ``dynamic`` rows
            (empty for ``static``). ``src_w``/``src_h`` are supplied at run time.
        asahi: enable the ASAHI adaptive-slice-sizing modifier (dynamic only).
        altitude_zoom: enable the altitude-gated zoom modifier (dynamic only).
        budget: soft target tiles/frame for reporting (None = unbounded).
        is_reference: True for the single dense-GT row that defines the
            recall/precision denominators.
    """

    name: str
    kind: Kind
    tiles_x: int = 0
    tiles_y: int = 0
    overlap: float = 0.0
    scheduler_kwargs: dict = field(default_factory=dict)
    asahi: bool = False
    altitude_zoom: bool = False
    budget: float | None = None
    is_reference: bool = False


# Canonical static grid set (matches run_pxt_bench.py CONFIGS, minus the
# +vga3x / +full rescue variants which are a v2 concern). Overlap 0.25 for all
# grids except the 1x1 no-tiling baseline.
_STATIC_GRIDS: list[tuple[str, int, int, float]] = [
    ("1x1", 1, 1, 0.0),
    ("2x2", 2, 2, 0.25),
    ("3x2", 3, 2, 0.25),
    ("3x3", 3, 3, 0.25),
    ("4x3", 4, 3, 0.25),
    ("6x4", 6, 4, 0.25),
    ("8x6", 8, 6, 0.25),
]

# Dense ground-truth reference grid (12x9 @ 0.25 overlap).
_REFERENCE_GRID = ("12x9", 12, 9, 0.25)

# Default scheduler kwargs for dynamic rows (mirrors the scheduler defaults the
# library ships: discovery_period=15, discovery_grid=(3,2), recovery_grid=(3,3),
# max_zoom=2.0). src_w/src_h are injected at run time by the runner.
_DYNAMIC_DEFAULTS: dict = {
    "discovery_period": 15,
    "discovery_grid": (3, 2),
    "recovery_grid": (3, 3),
    "max_zoom": 2.0,
}


def default_matrix() -> list[BenchConfig]:
    """Return the v1 ablation rows: static grid set + dynamic + levers + GT ref."""
    rows: list[BenchConfig] = []

    # Static grid rows.
    for name, tx, ty, ov in _STATIC_GRIDS:
        rows.append(
            BenchConfig(
                name=name,
                kind="static",
                tiles_x=tx,
                tiles_y=ty,
                overlap=ov,
                budget=float(tx * ty),
            )
        )

    # Dynamic rows: base + each lever toggled on individually.
    rows.append(
        BenchConfig(
            name="dynamic",
            kind="dynamic",
            scheduler_kwargs=dict(_DYNAMIC_DEFAULTS),
        )
    )
    rows.append(
        BenchConfig(
            name="dynamic+asahi",
            kind="dynamic",
            scheduler_kwargs=dict(_DYNAMIC_DEFAULTS),
            asahi=True,
        )
    )
    rows.append(
        BenchConfig(
            name="dynamic+altitude_zoom",
            kind="dynamic",
            scheduler_kwargs=dict(_DYNAMIC_DEFAULTS),
            altitude_zoom=True,
        )
    )

    # Dense-GT reference row (exactly one).
    rname, rtx, rty, rov = _REFERENCE_GRID
    rows.append(
        BenchConfig(
            name=rname,
            kind="static",
            tiles_x=rtx,
            tiles_y=rty,
            overlap=rov,
            budget=float(rtx * rty),
            is_reference=True,
        )
    )

    return rows
