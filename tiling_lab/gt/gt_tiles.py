"""Reconstruct the static GT dense-pass tile set for viewer overlays.

The GT dense pass (e.g. ``GT-12x9-25-multi``) runs a fixed tile layout:
the main 12x9 grid plus extra grids and an extra rect (``center_vga_3x``).
For debugging the GT overlays in ``overlay_viewer`` it's useful to draw
that exact tile layout on top of every frame — e.g. to investigate the
vga3-rect-boundary bbox-jump issue.

This module reconstructs the tile list from a dense run's ``config`` block
(the SAME config the bench wrote into ``pxt_*.json``) and emits dicts that
match the ``tiles`` schema produced by
``tiling_lab.harness.replay.emit_frames_json``::

    {"x": float, "y": float, "w": float, "h": float, "category": str}

so the viewer's existing tile display/toggles work without any change.

Grid enumeration uses ``hailo_tiling.geometry.grid_to_static_tiles`` (the
single source of truth for the tile math) rather than re-deriving it.
"""
from __future__ import annotations

import json
from pathlib import Path

from hailo_tiling.geometry import grid_to_static_tiles


def _parse_rect(s: str) -> tuple[float, float, float, float]:
    """Parse 'x,y,w,h[,mode]' (grid_to_static_tiles output) -> floats."""
    parts = s.split(",")
    return (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))


def tiles_from_dense_config(cfg: dict) -> list[dict]:
    """Reconstruct the full static tile set from a dense run ``config`` block.

    Returns a list of ``{"x","y","w","h","category"}`` dicts in normalized
    [0,1] frame coordinates. Categories mirror the viewer convention:

      * ``"multi-scale"`` — the main dense grid (boundary-strip layer).
      * ``"single-scale"`` — rescue tiles (full-frame, center-tile,
        ``extra_grids``, ``extra_rects``).

    An empty / falsy config yields an empty list.
    """
    if not cfg:
        return []
    out: list[dict] = []

    tiles_x = int(cfg.get("tiles_x", 0) or 0)
    tiles_y = int(cfg.get("tiles_y", 0) or 0)
    ox = float(cfg.get("overlap_x", 0.0) or 0.0)
    oy = float(cfg.get("overlap_y", 0.0) or 0.0)
    for rect in grid_to_static_tiles(tiles_x, tiles_y, ox, oy):
        x, y, w, h = _parse_rect(rect)
        out.append({"x": x, "y": y, "w": w, "h": h, "category": "multi-scale"})

    if cfg.get("include_full_frame"):
        out.append({"x": 0.0, "y": 0.0, "w": 1.0, "h": 1.0,
                    "category": "single-scale"})

    if cfg.get("include_center_tile"):
        s = float(cfg.get("center_tile_size", 0.0) or 0.0)
        if s > 0.0:
            s = min(s, 1.0)
            off = (1.0 - s) / 2.0
            out.append({"x": off, "y": off, "w": s, "h": s,
                        "category": "single-scale"})

    for entry in (cfg.get("extra_grids") or []):
        tx, ty = int(entry[0]), int(entry[1])
        gox, goy = float(entry[2]), float(entry[3])
        for rect in grid_to_static_tiles(tx, ty, gox, goy):
            x, y, w, h = _parse_rect(rect)
            out.append({"x": x, "y": y, "w": w, "h": h,
                        "category": "single-scale"})

    for entry in (cfg.get("extra_rects") or []):
        rx, ry, rw, rh = (float(entry[0]), float(entry[1]),
                          float(entry[2]), float(entry[3]))
        out.append({"x": rx, "y": ry, "w": rw, "h": rh,
                    "category": "single-scale"})

    return out


def load_dense_tiles(dense_path: Path) -> list[dict]:
    """Read a dense run ``.json`` and reconstruct its static tile set.

    Accepts either a dense summary ``.json`` (with a top-level ``config``
    block, e.g. ``pxt_GT-12x9-25-multi.json``) or a raw config dict file.
    """
    doc = json.loads(Path(dense_path).read_text())
    cfg = doc.get("config") if isinstance(doc, dict) else None
    if cfg is None and isinstance(doc, dict):
        cfg = doc  # allow a bare config dict
    return tiles_from_dense_config(cfg or {})
