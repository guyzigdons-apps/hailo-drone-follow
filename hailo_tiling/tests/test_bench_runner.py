"""Tests for the ablation bench runner (Plan 6 Task B2 + Night-2 B2)."""
from __future__ import annotations

import pytest

from hailo_tiling.bench.config import BenchConfig
from hailo_tiling.bench.runner import (
    run_config,
    run_dynamic_config,
    run_static_config_crop_ordered,
)
from hailo_tiling.backends.backend import MockBackend
from hailo_tiling.backends.replay import CacheMissError
from hailo_tiling.cache.hashing import tile_norm_to_source_px
from hailo_tiling.cache.store import SqliteCacheStore
from hailo_tiling.classes import PERSON
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
    from hailo_tiling.geometry import grid_to_static_tiles

    for (nx, ny) in [(1, 1), (2, 2), (3, 2), (3, 3), (4, 3), (6, 4), (8, 6), (12, 9)]:
        for ov in (0.0, 0.25):
            for r in grid_to_static_tiles(nx, ny, ov, ov):
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


def test_static_config_records_tile_rectangles(tmp_path):
    """run_config must persist the normalized tile rectangles per frame so the
    visualizer can draw the tile layout (not just the n_tiles count)."""
    store = SqliteCacheStore.open(tmp_path / "seed.sqlite3")
    try:
        frames = [0, 1]
        _seed_1x1(store, frames, {})
        cfg = BenchConfig(name="1x1", kind="static", tiles_x=1, tiles_y=1, overlap=0.0)
        res = run_config(cfg, store, {"src_w": _W, "src_h": _H}, frames)

        for f in res.frames:
            assert len(f.tiles) == f.n_tiles == 1
            x, y, w, h, cat = f.tiles[0]
            # 1x1 tile is the whole normalized frame.
            assert (x, y) == pytest.approx((0.0, 0.0), abs=1e-6)
            assert (w, h) == pytest.approx((1.0, 1.0), abs=1e-6)
            assert isinstance(cat, str)
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


def test_crop_ordered_replay_reconstructs_source_frames(tmp_path):
    """A GST-style cache whose frame_idx is a per-tile-buffer counter (each crop
    appears once per source frame under a distinct frame_idx) replays via the
    crop-ordered path: the k-th occurrence of each crop = source frame k."""
    store = SqliteCacheStore.open(tmp_path / "ptb.sqlite3")
    try:
        # 2x2 grid, 3 source frames, per-tile-buffer frame_idx (monotonic):
        # frame_idx 0..11 = 3 frames x 4 tiles, in source order.
        crops = _crops_for_grid_2x2()
        store.meta_put("video_w", str(_W))
        store.meta_put("video_h", str(_H))
        rows = []
        fidx = 0
        for src_frame in range(3):
            for ci, c in enumerate(crops):
                # Put one detection in tile 0 of source frame 1 to check ordering.
                dets = []
                if src_frame == 1 and ci == 0:
                    dets = [Det(cls=1, score=0.7, x=0.5, y=0.5, w=0.2, h=0.2)]
                rows.append({"frame_idx": fidx, "crop_rect": c, "ppv": 1, "dets": dets})
                fidx += 1
        store.put_many(rows)

        cfg = BenchConfig(name="2x2", kind="static", tiles_x=2, tiles_y=2, overlap=0.0)
        res = run_static_config_crop_ordered(cfg, store, {"src_w": _W, "src_h": _H})
        assert len(res.frames) == 3
        assert all(f.n_tiles == 4 and f.n_misses == 0 for f in res.frames)
        # Tile rectangles persisted for the visualizer (parity with run_config).
        assert all(len(f.tiles) == 4 for f in res.frames)
        # The detection lands in reconstructed source frame 1.
        assert len(res.frames[0].dets) == 0
        assert len(res.frames[1].dets) == 1
        assert len(res.frames[2].dets) == 0
    finally:
        store.close()


def _crops_for_grid_2x2():
    from hailo_tiling.geometry import grid_to_static_tiles
    out = []
    for r in grid_to_static_tiles(2, 2, 0.0, 0.0):
        x, y, w, h = (float(v) for v in r.split(",")[:4])
        out.append(tile_norm_to_source_px(x, y, w, h, _W, _H))
    return out


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


def test_dynamic_runner_produces_per_frame_tiles():
    """Night-2 B2: the stateful dynamic runner drives TileScheduler + the
    production ByteTracker per frame, fed by the per-frame backend detections,
    and the emitted tile set VARIES across frames (discovery on cadence, then a
    tracker ROI once a target is locked). Aggregated dets are returned per
    frame. Backend-agnostic (MockBackend here)."""
    # GT-consistent backend (batched ABC): for every crop covering the GT
    # centre, emit a single person whose tile-local coords map back to GT in
    # source space — so the tracker locks onto the GT-seeded target and an ROI
    # tile appears on subsequent frames. Mirrors the lab harness's own
    # _AnyCropBackend, adapted to the batched infer signature.
    gt_at = lambda fi: (0.40 + 0.01 * fi, 0.40, 0.08, 0.20)

    class _GtConsistentBackend(MockBackend):
        def infer(self, frame, crops, frame_idx):
            self.calls.append({"frame_idx": frame_idx, "crops": list(crops)})
            gx, gy, gw, gh = gt_at(frame_idx)
            gcx = (gx + gw / 2) * _W
            gcy = (gy + gh / 2) * _H
            out = []
            for c in crops:
                if c.x <= gcx <= c.x + c.w and c.y <= gcy <= c.y + c.h:
                    lx = (gcx - c.x) / c.w
                    ly = (gcy - c.y) / c.h
                    lw = gw * _W / c.w
                    lh = gh * _H / c.h
                    out.append([Det(cls=PERSON, score=0.9, x=lx - lw / 2,
                                    y=ly - lh / 2, w=lw, h=lh)])
                else:
                    out.append([])
            return out

    backend = _GtConsistentBackend()
    # discovery_period=1 so the tracker is fed every frame and reaches TRACKING
    # within a couple of frames; once TRACKING the scheduler prepends an ROI
    # tile, so the per-frame tile set differs from the pure discovery grid.
    cfg = BenchConfig(
        name="dynamic",
        kind="dynamic",
        scheduler_kwargs={"discovery_period": 1, "discovery_grid": (3, 2),
                          "recovery_grid": (3, 3), "max_zoom": 2.0},
    )
    # GT trajectory: target drifting slightly each frame, present every frame,
    # so the lock seeds as soon as ByteTracker activates a track.
    gt_traj = {fi: gt_at(fi) for fi in range(6)}
    res = run_dynamic_config(
        cfg, backend, {"src_w": _W, "src_h": _H}, list(range(6)),
        gt_traj=gt_traj, fps=30.0,
    )

    assert res.kind == "dynamic"
    assert len(res.frames) == 6
    # The scheduler was driven (and emitted tiles) once per frame.
    assert [c["frame_idx"] for c in backend.calls] == list(range(6))
    # Frame 0: discovery grid only (lock not yet TRACKING) -> 6 tiles.
    assert res.frames[0].n_tiles == 6
    # Once locked, later frames emit ROI + discovery grid -> MORE than 6 tiles.
    later = [f.n_tiles for f in res.frames[1:]]
    assert any(n > 6 for n in later), f"expected an ROI tile to appear: {later}"
    # Crops genuinely vary frame-to-frame (not a constant grid).
    crop_sets = {tuple((c.x, c.y, c.w, c.h) for c in call["crops"]) for call in backend.calls}
    assert len(crop_sets) > 1, "dynamic tiles must vary across frames"


def test_dynamic_runner_counts_replay_misses(tmp_path):
    """The dynamic runner over a ReplayBackend counts per-crop cache misses into
    n_misses (does not raise), so a partially-warmed dynamic cache is surfaced
    rather than crashing the table."""
    from hailo_tiling.backends.replay import ReplayBackend

    store = SqliteCacheStore.open(tmp_path / "empty_dyn.sqlite3")
    try:
        backend = ReplayBackend(store, ppv=1)
        cfg = BenchConfig(
            name="dynamic", kind="dynamic",
            scheduler_kwargs={"discovery_period": 15, "discovery_grid": (3, 2),
                              "recovery_grid": (3, 3), "max_zoom": 2.0},
        )
        gt_traj = {0: (0.4, 0.4, 0.2, 0.2)}
        res = run_dynamic_config(
            cfg, backend, {"src_w": _W, "src_h": _H}, [0],
            gt_traj=gt_traj, fps=30.0,
        )
        # Frame 0 on cadence -> 6 discovery tiles, none warmed -> 6 misses.
        assert res.frames[0].n_tiles == 6
        assert res.frames[0].n_misses == 6
        assert res.n_misses_total == 6
    finally:
        store.close()


def test_dynamic_discovery_grid_hits_when_warmed(tmp_path):
    """When the discovery grid IS warmed, a dynamic row replays it with 0
    misses (chip-free)."""
    store = SqliteCacheStore.open(tmp_path / "warmed_disc.sqlite3")
    try:
        # Seed the exact 3x2 discovery-grid crops the scheduler emits at frame 0.
        from hailo_tiling.dynamic.scheduler import TileScheduler

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
