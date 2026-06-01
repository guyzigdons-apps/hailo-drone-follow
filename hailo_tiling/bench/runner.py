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
from ..classes import PERSON
from ..backends.replay import CacheMissError, ReplayBackend
from ..cache.hashing import tile_norm_to_source_px
from ..cache.store import SqliteCacheStore
from ..types import CropRect, Det
from .config import BenchConfig
from .grid import grid_to_static_tiles


@dataclass
class FrameResult:
    """One frame's replayed + aggregated result for a config."""

    frame_idx: int
    dets: list[Det]
    n_tiles: int
    n_misses: int = 0
    # Normalized tile rectangles requested this frame, as
    # ``(x, y, w, h, category)`` tuples. Lets the visualizer draw the tile
    # layout (esp. dynamic ROI tiles, whose positions are otherwise lost).
    tiles: list = field(default_factory=list)


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
    rects = grid_to_static_tiles(cfg.tiles_x, cfg.tiles_y, cfg.overlap, cfg.overlap)
    crops: list[CropRect] = []
    for r in rects:
        parts = r.split(",")
        x, y, w, h = (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
        crops.append(tile_norm_to_source_px(x, y, w, h, src_w, src_h))
    return crops


def _crops_to_norm_tiles(crops: Sequence[CropRect], src_w: int, src_h: int) -> list:
    """Convert source-pixel crops to normalized ``(x, y, w, h, category)`` tile
    tuples for frames.json / the visualizer. ``category`` is taken from the
    crop's ``mode`` tag (``s``/``m``) or any ``category`` attribute it carries."""
    out = []
    for c in crops:
        cat = getattr(c, "category", None) or getattr(c, "mode", "s")
        out.append((c.x / src_w, c.y / src_h, c.w / src_w, c.h / src_h, str(cat)))
    return out


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
        norm_tiles = _crops_to_norm_tiles(crops, src_w, src_h)
        for fi in frames:
            # A static row must be fully warmed: a miss is a hard error
            # (crop-key consistency broken) — let CacheMissError propagate.
            dets_per_crop = backend.infer(None, crops, fi)
            final = agg.aggregate(fi, crops, dets_per_crop, src_w, src_h)
            result.frames.append(
                FrameResult(frame_idx=fi, dets=final, n_tiles=len(crops),
                            n_misses=0, tiles=list(norm_tiles))
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
                    frame_idx=fi, dets=final, n_tiles=len(crops), n_misses=n_misses,
                    tiles=_crops_to_norm_tiles(crops, src_w, src_h),
                )
            )
            result.n_misses_total += n_misses
        return result

    raise ValueError(f"unknown config kind: {cfg.kind!r}")


def run_static_config_crop_ordered(
    cfg: BenchConfig,
    store: SqliteCacheStore,
    video_meta: dict,
    *,
    ppv: int = 1,
) -> ConfigResult:
    """Replay a STATIC grid against a GST-warmed cache whose ``frame_idx`` is a
    per-tile-buffer monotonic counter (the GStreamer ``hailocachewriter``
    convention) rather than a per-source-frame index.

    In such a cache each grid crop appears exactly once per source frame; the
    k-th occurrence of a crop (ordered by ``frame_idx``) belongs to source
    frame k. We therefore reconstruct source frames by zipping the per-crop
    occurrence streams. This sidesteps the writer's frame_idx semantics while
    relying only on the (correct) source-pixel crop keys.

    Returns a ConfigResult with one FrameResult per reconstructed source frame.
    Raises ``CacheMissError`` if any grid crop has zero occurrences (the grid
    was not warmed) — preserving the static-must-be-fully-warmed guarantee.
    """
    if cfg.kind != "static":
        raise ValueError("crop-ordered replay is for static configs only")
    src_w = int(video_meta["src_w"])
    src_h = int(video_meta["src_h"])
    crops = _static_crops(cfg, src_w, src_h)
    agg = _make_aggregator(cfg)

    # Per-crop ordered dets-json streams.
    from ..cache.store import _json_to_dets

    streams: list[list] = []
    n_frames = None
    for c in crops:
        rows = store._con.execute(  # noqa: SLF001 — same-package read
            "SELECT dets_json FROM detections WHERE "
            "crop_x=? AND crop_y=? AND crop_w=? AND crop_h=? AND ppv=? "
            "ORDER BY frame_idx",
            (c.x, c.y, c.w, c.h, int(ppv)),
        ).fetchall()
        if not rows:
            raise CacheMissError(
                f"static grid {cfg.name}: crop ({c.x},{c.y},{c.w},{c.h}) "
                "has no occurrences in the cache — grid not warmed."
            )
        streams.append([_json_to_dets(r[0]) for r in rows])
        n = len(rows)
        n_frames = n if n_frames is None else min(n_frames, n)

    result = ConfigResult(name=cfg.name, kind=cfg.kind)
    norm_tiles = _crops_to_norm_tiles(crops, src_w, src_h)
    for k in range(n_frames or 0):
        dets_per_crop = [streams[ci][k] for ci in range(len(crops))]
        final = agg.aggregate(k, crops, dets_per_crop, src_w, src_h)
        result.frames.append(
            FrameResult(frame_idx=k, dets=final, n_tiles=len(crops), n_misses=0,
                        tiles=list(norm_tiles))
        )
    return result


def run_dynamic_config(
    cfg: BenchConfig,
    backend,
    video_meta: dict,
    frames: Sequence[int],
    *,
    gt_traj: dict | None = None,
    fps: float = 30.0,
    budget: float | None = None,
    person_cls: int = PERSON,
    ppv: int = 1,
) -> ConfigResult:
    """Stateful dynamic-config runner (Night-2 B2).

    Drives ``dynamic_tiling.scheduler.TileScheduler`` fed by a per-frame
    ``TargetLock`` (the production ByteTracker) over ``frames``, producing the
    real per-frame ROI/discovery/recovery tiles — NOT the no-lock placeholder
    of :func:`run_config`. Each frame:

      1. ``crops = scheduler.decide(lock.state, fi, meter)`` (uses the lock
         state carried from the previous frame).
      2. ``dets_per_crop = backend.infer(frame, crops, fi)`` — one batched call.
         The backend is ``CachingBackend(GstCropperBackend)`` for the on-chip
         warm pass (A3) or ``ReplayBackend`` for the chip-free replay (B3); a
         per-crop ``CacheMissError`` is counted into ``n_misses`` (replay path)
         rather than raised, so a partially-warmed dynamic cache is surfaced.
      3. Aggregate to source-frame dets, charge the budget meter, and step the
         lock with the person detections (GT-seeded while unlocked).

    Determinism: ByteTracker is deterministic given the same per-frame detection
    sequence, so the warm pass and the chip-free replay request the SAME crops
    in the SAME order — the warmed cache therefore contains exactly the crops
    the replay looks up (the paper's matched-warm guarantee).

    ``frame`` passed to the backend is the source frame index ``fi`` (the
    GstCropperBackend decodes the video itself and selects ``frame_idx``; the
    ReplayBackend ignores the frame argument).
    """
    if cfg.kind != "dynamic":
        raise ValueError("run_dynamic_config is for dynamic configs only")

    from dynamic_tiling.scheduler import TileScheduler
    from dynamic_tiling.target_lock import TargetLock

    from ..budget import BudgetMeter

    src_w = int(video_meta["src_w"])
    src_h = int(video_meta["src_h"])
    gt_traj = gt_traj or {}

    sched = TileScheduler(src_w, src_h, **cfg.scheduler_kwargs)
    lock = TargetLock()
    # budget None / cfg.budget None => effectively unbounded (huge cap).
    eff_budget = budget if budget is not None else (cfg.budget if cfg.budget is not None else 1e9)
    meter = BudgetMeter(budget_inf_per_s=float(eff_budget) * fps, fps=fps)
    agg = _make_aggregator(cfg)
    result = ConfigResult(name=cfg.name, kind=cfg.kind)

    for fi in frames:
        crops = sched.decide(lock.state, fi, meter)
        meter.charge(len(crops), fi)

        hit_crops: list[CropRect] = []
        hit_dets: list[list[Det]] = []
        n_misses = 0
        if crops:
            # One batched backend call; on a ReplayBackend a missing crop raises
            # CacheMissError, so fall back to per-crop lookups to count misses
            # without losing the hits.
            try:
                dets_per_crop = backend.infer(fi, crops, fi)
                hit_crops = list(crops)
                hit_dets = list(dets_per_crop)
            except CacheMissError:
                for c in crops:
                    try:
                        dets = backend.infer(fi, [c], fi)[0]
                    except CacheMissError:
                        n_misses += 1
                        continue
                    hit_crops.append(c)
                    hit_dets.append(dets)

        final = agg.aggregate(fi, hit_crops, hit_dets, src_w, src_h)

        # Step the lock with this frame's person detections; GT-seed while the
        # lock has not yet acquired a track (mirrors dynamic_tiling.replay.run).
        persons = [d for d in final if d.cls == person_cls]
        gt_box = gt_traj.get(fi)
        if lock.track_id is None and gt_box is not None:
            lock.step(persons, gt_bbox_norm=gt_box)
        else:
            lock.step(persons)

        result.frames.append(
            FrameResult(frame_idx=fi, dets=final, n_tiles=len(crops),
                        n_misses=n_misses,
                        tiles=_crops_to_norm_tiles(crops, src_w, src_h))
        )
        result.n_misses_total += n_misses

    return result


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
