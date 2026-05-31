"""Tests for the hailo-tiling-bench CLI + ablation table (Plan 6 Task B3)."""
from __future__ import annotations

import json

import pytest

from hailo_tiling.cache.hashing import tile_norm_to_source_px
from hailo_tiling.cache.store import SqliteCacheStore
from hailo_tiling.cli import bench
from hailo_tiling.types import Det
from tiling_benchmark.tiling_record import _grid_to_static_tiles

_W, _H = 3840, 2160


def _crops_for_grid(tx, ty, ov):
    out = []
    for r in _grid_to_static_tiles(tx, ty, ov, ov):
        x, y, w, h = (float(v) for v in r.split(",")[:4])
        out.append(tile_norm_to_source_px(x, y, w, h, _W, _H))
    return out


def _seed(store, frames):
    """Seed the 1x1 grid AND the 12x9 reference grid for the given frames,
    placing one detection in the 1x1 tile (so recall/precision are numeric)."""
    rows = []
    one = _crops_for_grid(1, 1, 0.0)
    ref = _crops_for_grid(12, 9, 0.25)
    for fi in frames:
        # 1x1 tile carries one detection (centred).
        rows.append({
            "frame_idx": fi, "crop_rect": one[0], "ppv": 1,
            "dets": [Det(cls=2, score=0.8, x=0.45, y=0.45, w=0.1, h=0.1)],
        })
        # Reference grid tiles: seed empty (enough for a 0-miss replay; the
        # reference simply has no dets here -> recall denominator 0 -> recall 0).
        for c in ref:
            rows.append({"frame_idx": fi, "crop_rect": c, "ppv": 1, "dets": []})
    store.meta_put("video_w", str(_W))
    store.meta_put("video_h", str(_H))
    store.put_many(rows)


def test_bench_cli_emits_table(tmp_path):
    cache = tmp_path / "seed.sqlite3"
    store = SqliteCacheStore.open(cache)
    try:
        _seed(store, [0, 1])
    finally:
        store.close()

    out_dir = tmp_path / "ablation"
    rc = bench.main([
        "--cache", str(cache),
        "--video", "/fake/0026__fov50.mp4",
        "--configs", "1x1",
        "--out-dir", str(out_dir),
    ])
    assert rc == 0

    table = out_dir / "ablation_table.md"
    assert table.exists(), "ablation_table.md not written"
    text = table.read_text()
    # A row per config: 1x1 + the reference (12x9, always included).
    assert "| 1x1 |" in text
    assert "| 12x9 |" in text
    # frames.json per config.
    assert (out_dir / "1x1.frames.json").exists()
    assert (out_dir / "12x9.frames.json").exists()

    # The 1x1 frames.json carries the seeded detection.
    doc = json.loads((out_dir / "1x1.frames.json").read_text())
    assert doc["config"] == "1x1"
    assert doc["n_misses_total"] == 0
    total_dets = sum(len(f["dets"]) for f in doc["frames"])
    assert total_dets == 2  # one det in each of 2 frames


def test_bench_cli_recall_is_numeric(tmp_path):
    """The recall_vs_reference column must be a parseable number for non-ref
    rows."""
    cache = tmp_path / "seed2.sqlite3"
    store = SqliteCacheStore.open(cache)
    try:
        _seed(store, [0])
    finally:
        store.close()

    out_dir = tmp_path / "ablation2"
    result = bench.run(
        cache=str(cache), video="/fake/v.mp4", out_dir=out_dir,
        configs_spec="1x1",
    )
    rows = {r["name"]: r for r in result["rows"]}
    assert "1x1" in rows and "12x9" in rows
    # recall/precision are floats (numeric), not None, for the candidate row.
    assert isinstance(rows["1x1"]["recall"], float)
    assert isinstance(rows["1x1"]["precision"], float)
    # Reference vs itself is 1.0 by definition.
    assert rows["12x9"]["recall"] == 1.0


def test_bench_cli_rejects_unknown_config(tmp_path):
    cache = tmp_path / "seed3.sqlite3"
    store = SqliteCacheStore.open(cache)
    try:
        _seed(store, [0])
    finally:
        store.close()
    with pytest.raises(SystemExit):
        bench.run(cache=str(cache), video="/v.mp4", out_dir=tmp_path / "o",
                  configs_spec="nonesuch")
