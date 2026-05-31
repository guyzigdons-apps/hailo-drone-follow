"""Tests for the ablation bench runner (Plan 6 Task B2)."""
from __future__ import annotations

import pytest

from hailo_tiling.bench.config import BenchConfig
from hailo_tiling.bench.runner import run_config
from hailo_tiling.backends.replay import CacheMissError
from hailo_tiling.cache.hashing import tile_norm_to_source_px
from hailo_tiling.cache.store import SqliteCacheStore
from hailo_tiling.types import CropRect, Det

_W, _H = 3840, 2160


def _seed_1x1(store: SqliteCacheStore, frames, dets_per_frame):
    """Seed the store with the single 1x1 source-pixel crop key for each frame."""
    crop = tile_norm_to_source_px(0.0, 0.0, 1.0, 1.0, _W, _H)
    rows = []
    for fi in frames:
        rows.append(
            {"frame_idx": fi, "crop_rect": crop, "ppv": 1, "dets": dets_per_frame.get(fi, [])}
        )
    store.put_many(rows)
    return crop


def _gate_px(xmin, ymin, w, h, W, H):
    """Inline copy of scripts/cache_gst_replay_gate.py:_tile_crop_px math
    (without importing gi/hailo) for cross-checking tile_norm_to_source_px."""
    cx, cy = int(xmin * W), int(ymin * H)
    cw, ch = int(w * W), int(h * H)
    cw = max(0, min(cw, W - cx))
    ch = max(0, min(ch, H - cy))
    return (cx, cy, cw, ch)


def test_tile_norm_to_source_px_matches_cropper_rule_across_grids():
    """The Python replay key must equal the cropper/warmer source-pixel rule
    EXACTLY across the canonical grid set — the foundation of chip-free replay
    (Plan-5b proved 0 deviations)."""
    from tiling_benchmark.tiling_record import _grid_to_static_tiles

    for (nx, ny) in [(1, 1), (2, 2), (3, 2), (3, 3), (4, 3), (6, 4), (8, 6), (12, 9)]:
        for ov in (0.0, 0.25):
            for r in _grid_to_static_tiles(nx, ny, ov, ov):
                x, y, w, h = (float(v) for v in r.split(",")[:4])
                cr = tile_norm_to_source_px(x, y, w, h, _W, _H)
                assert (cr.x, cr.y, cr.w, cr.h) == _gate_px(x, y, w, h, _W, _H), (
                    f"grid {nx}x{ny}:{ov} tile {r}"
                )


def test_static_config_replays_from_cache(tmp_path):
    """A 1x1 static config replays a seeded cache with 0 misses and returns
    aggregated source-frame detections."""
    store = SqliteCacheStore.open(tmp_path / "seed.sqlite3")
    try:
        frames = [0, 1, 2]
        # One tile-local-normalized detection centred in frame 1.
        dets = {1: [Det(cls=2, score=0.8, x=0.4, y=0.4, w=0.2, h=0.2)]}
        _seed_1x1(store, frames, dets)

        cfg = BenchConfig(name="1x1", kind="static", tiles_x=1, tiles_y=1, overlap=0.0)
        res = run_config(cfg, store, {"src_w": _W, "src_h": _H}, frames)

        assert res.name == "1x1"
        assert len(res.frames) == 3
        # Exactly one tile per frame for 1x1.
        assert all(f.n_tiles == 1 for f in res.frames)
        # No misses against a fully-seeded grid.
        assert all(f.n_misses == 0 for f in res.frames)
        assert res.mean_tiles_per_frame == 1.0
        # Frame 1 carries the one detection, mapped to source-frame coords.
        f1 = next(f for f in res.frames if f.frame_idx == 1)
        assert len(f1.dets) == 1
        d = f1.dets[0]
        assert d.cls == 2
        # 1x1 crop is the whole frame, so source-frame coords == tile-local.
        assert d.x == pytest.approx(0.4, abs=1e-6)
        assert d.w == pytest.approx(0.2, abs=1e-6)
        # Frames 0 and 2 have no detections.
        assert res.n_dets_total == 1
    finally:
        store.close()


def test_static_config_raises_on_miss(tmp_path):
    """A static row that asks for a frame not in the cache must raise — the
    crop-key consistency / full-warming guarantee is load-bearing."""
    store = SqliteCacheStore.open(tmp_path / "partial.sqlite3")
    try:
        _seed_1x1(store, [0], {})  # only frame 0 seeded
        cfg = BenchConfig(name="1x1", kind="static", tiles_x=1, tiles_y=1, overlap=0.0)
        with pytest.raises(CacheMissError):
            run_config(cfg, store, {"src_w": _W, "src_h": _H}, [0, 1])
    finally:
        store.close()


def test_dynamic_config_counts_misses_without_raising(tmp_path):
    """A dynamic row counts misses into n_misses instead of raising. With an
    empty cache, the discovery grid on the cadence frame all-misses."""
    store = SqliteCacheStore.open(tmp_path / "empty.sqlite3")
    try:
        cfg = BenchConfig(
            name="dynamic",
            kind="dynamic",
            scheduler_kwargs={"discovery_period": 15, "discovery_grid": (3, 2),
                              "recovery_grid": (3, 3), "max_zoom": 2.0},
        )
        # Frame 0 is on the discovery cadence (0 % 15 == 0) -> 6 tiles, all miss.
        res = run_config(cfg, store, {"src_w": _W, "src_h": _H}, [0, 1])
        f0 = next(f for f in res.frames if f.frame_idx == 0)
        assert f0.n_tiles == 6
        assert f0.n_misses == 6  # nothing warmed
        # Off-cadence frame emits no tiles.
        f1 = next(f for f in res.frames if f.frame_idx == 1)
        assert f1.n_tiles == 0
        assert res.n_misses_total == 6
    finally:
        store.close()


def test_dynamic_discovery_grid_hits_when_warmed(tmp_path):
    """When the discovery grid IS warmed, a dynamic row replays it with 0
    misses (chip-free)."""
    store = SqliteCacheStore.open(tmp_path / "warmed_disc.sqlite3")
    try:
        # Seed the exact 3x2 discovery-grid crops the scheduler emits at frame 0.
        from dynamic_tiling.scheduler import TileScheduler

        sched = TileScheduler(_W, _H, discovery_period=15, discovery_grid=(3, 2))
        crops = sched._grid(3, 2, 0, 0, _W, _H, "m")
        rows = [
            {"frame_idx": 0, "crop_rect": c, "ppv": 1, "dets": []} for c in crops
        ]
        store.put_many(rows)

        cfg = BenchConfig(
            name="dynamic",
            kind="dynamic",
            scheduler_kwargs={"discovery_period": 15, "discovery_grid": (3, 2),
                              "recovery_grid": (3, 3), "max_zoom": 2.0},
        )
        res = run_config(cfg, store, {"src_w": _W, "src_h": _H}, [0])
        f0 = res.frames[0]
        assert f0.n_tiles == 6
        assert f0.n_misses == 0
    finally:
        store.close()
