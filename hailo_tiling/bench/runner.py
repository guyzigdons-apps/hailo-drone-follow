"""Per-config crop generation + chip-free replay run (Plan 6 Task B2).

`run_config(cfg, store, video_meta, frames)` replays one ablation row against a
warmed source-pixel-keyed cache:

  * **static** rows: per frame, expand the grid to normalized tiles via the
    canonical ``_grid_to_static_tiles``, convert each to a source-pixel
    ``CropRect`` via ``tile_norm_to_source_px`` (the truncate-then-clamp rule
    matching the cropper / warmer), look up via ``ReplayBackend``, then
    aggregate (map-to-source + boundary strip + NMS) via ``Aggregator``.
  * **dynamic** rows: drive ``TileScheduler`` to get per-frame source-pixel
    crops, look them up the same way, aggregate. ROI-zoom tiles may not be in
    the pre-warmed cache — those are surfaced as a structured ``n_misses``
    count, NOT silently swallowed and NOT raised (a dynamic row is allowed to
    miss; only a *static* row must be fully warmed and so raises on a miss).

Chip-free: the only backend touched is ``ReplayBackend`` (a cache miss raises
``CacheMissError``; for dynamic rows we catch it per-tile and count it).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

from ..aggregator.aggregator import Aggregator
from ..aggregator.boundary_strip import BoundaryStripFilter
from ..backends.replay import CacheMissError, ReplayBackend
from ..cache.hashing import tile_norm_to_source_px
from ..cache.store import SqliteCacheStore
from ..types import CropRect, Det
from .config import BenchConfig

# Single source of truth for normalized grid -> tiles (shared with the warmer).
from tiling_benchmark.tiling_record import _grid_to_static_tiles


@dataclass
class FrameResult:
    """One frame's replayed + aggregated result for a config."""

    frame_idx: int
    dets: list[Det]
    n_tiles: int
    n_misses: int = 0


@dataclass
class ConfigResult:
    """All frames for one config plus rollups."""

    name: str
    kind: str
    frames: list[FrameResult] = field(default_factory=list)
    n_misses_total: int = 0

    @property
    def mean_tiles_per_frame(self) -> float:
        if not self.frames:
            return 0.0
        return sum(f.n_tiles for f in self.frames) / len(self.frames)

    @property
    def n_dets_total(self) -> int:
        return sum(len(f.dets) for f in self.frames)


def _static_crops(cfg: BenchConfig, src_w: int, src_h: int) -> list[CropRect]:
    """Source-pixel crops for a static grid row (constant across frames)."""
    rects = _grid_to_static_tiles(cfg.tiles_x, cfg.tiles_y, cfg.overlap, cfg.overlap)
    crops: list[CropRect] = []
    for r in rects:
        parts = r.split(",")
        x, y, w, h = (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
        crops.append(tile_norm_to_source_px(x, y, w, h, src_w, src_h))
    return crops


def _make_aggregator(cfg: BenchConfig) -> Aggregator:
    # Static grids with overlap want the boundary strip ON for "m"-mode tiles;
    # the warmer writes single-scale tiles ("s") by default, so the strip is a
    # no-op for them (filter only acts on mode == "m"). Keep a default
    # aggregator; iou_thr at the spec's 0.5.
    return Aggregator(boundary_strip=BoundaryStripFilter(), iou_thr=0.5)


def run_config(
    cfg: BenchConfig,
    store: SqliteCacheStore,
    video_meta: dict,
    frames: Sequence[int],
    *,
    ppv: int = 1,
) -> ConfigResult:
    """Replay `cfg` over `frames` against `store`, returning a ConfigResult.

    `video_meta` must provide `src_w`/`src_h` (the source resolution the cache
    was warmed at — read from the cache meta envelope by the caller). `frames`
    is the explicit list of frame indices to replay.

    Static rows raise ``CacheMissError`` on any miss (they must be fully
    warmed); dynamic rows count misses into ``n_misses`` and continue.
    """
    src_w = int(video_meta["src_w"])
    src_h = int(video_meta["src_h"])
    backend = ReplayBackend(store, ppv=ppv)
    agg = _make_aggregator(cfg)
    result = ConfigResult(name=cfg.name, kind=cfg.kind)

    if cfg.kind == "static":
        crops = _static_crops(cfg, src_w, src_h)
        for fi in frames:
            # A static row must be fully warmed: a miss is a hard error
            # (crop-key consistency broken) — let CacheMissError propagate.
            dets_per_crop = backend.infer(None, crops, fi)
            final = agg.aggregate(fi, crops, dets_per_crop, src_w, src_h)
            result.frames.append(
                FrameResult(frame_idx=fi, dets=final, n_tiles=len(crops), n_misses=0)
            )
        return result

    if cfg.kind == "dynamic":
        for fi in frames:
            crops = _dynamic_crops(cfg, src_w, src_h, fi)
            hit_crops: list[CropRect] = []
            hit_dets: list[list[Det]] = []
            n_misses = 0
            for c in crops:
                try:
                    dets = backend.infer(None, [c], fi)[0]
                except CacheMissError:
                    n_misses += 1
                    continue
                hit_crops.append(c)
                hit_dets.append(dets)
            final = agg.aggregate(fi, hit_crops, hit_dets, src_w, src_h)
            result.frames.append(
                FrameResult(
                    frame_idx=fi, dets=final, n_tiles=len(crops), n_misses=n_misses
                )
            )
            result.n_misses_total += n_misses
        return result

    raise ValueError(f"unknown config kind: {cfg.kind!r}")


def _dynamic_crops(cfg: BenchConfig, src_w: int, src_h: int, frame_idx: int) -> list[CropRect]:
    """Per-frame source-pixel crops for a dynamic row.

    v1 surfacing: with no live tracker/lock state available in the chip-free
    replay path, we drive the scheduler in its no-lock cadence — on a discovery
    frame it emits the discovery grid (which IS in the warmed set), off-cadence
    it emits nothing. ROI/recovery tiles (which would miss the pre-warmed
    cache) require live lock state and are exercised by the B4 live path; here
    they are simply absent, so dynamic rows replay their discovery grid and
    report 0 misses against a fully-warmed cache. The lever flags are recorded
    on the config for the table but do not alter the emitted crops in this
    chip-free v1 (they reshape ROI tiles, which need the live path).
    """
    from dynamic_tiling.scheduler import TileScheduler

    sched = TileScheduler(src_w, src_h, **cfg.scheduler_kwargs)
    # No lock: scheduler emits the discovery grid on cadence, else nothing.
    on_cadence = (frame_idx % sched.discovery_period == 0)
    if not on_cadence:
        return []
    gx, gy = sched.discovery_grid
    return sched._grid(gx, gy, 0, 0, src_w, src_h, "m")
