"""GStreamer-golden cache warmer (Plan 6 Task A2).

Runs the canonical tiled GStreamer pipeline
(``hailotilecropper_dynamic -> hailonet -> hailofilter -> hailocachewriter
mode=tile_cache``) over a video for a list of fixed normalized grids,
recording per-tile detections into ONE source-pixel-keyed SQLite cache file
(one cache per ``(clip, FOV)``).

Because Plan-5b proved the cropper's source-pixel crop equals
``tile_crop_to_source_px(normalized_tile, src_w, src_h)`` exactly (0
deviations), warming by feeding normalized tiles to the cropper produces
exactly the source-pixel keys that the chip-free ablation harness
(``hailo-tiling-bench``) reproduces.

Idempotency: the writer uses ``INSERT OR IGNORE`` (Plan 6 A1), so overlapping
grids / re-runs that re-record the same ``(frame_idx, crop, ppv)`` key are a
no-op. Warming is therefore re-runnable and grids may be appended to an
existing cache file.

The pipeline construction + the source-pixel crop-key math are REUSED from
``scripts/cache_gst_replay_gate.py`` (its pass-1 "live" inner branch) so the
warmer and the bit-exact replay gate share one definition.

Usage:
    source setup_env.sh
    GST_PLUGIN_PATH=gst-hailo-cache/build/src \\
      python scripts/warm_gst_cache.py \\
        --video /path/0026__fov50.mp4 \\
        --hef   /usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef \\
        --out-cache .tile_cache/0026__fov50__<hefsha16>.sqlite3 \\
        --grids "3x2:0.25;6x4:0.25" \\
        --source-width 3840 --source-height 2160 [--max-frames N]
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Sequence

# Reuse the canonical pipeline construction + run loop from the replay gate.
# (Same directory; import by path so this works whether or not scripts/ is a
# package.) Also put the repo root on sys.path so the top-level
# `tiling_benchmark` / `hailo_tiling` packages resolve when this script is run
# directly (not just under pytest, which injects the rootdir).
_THIS_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _THIS_DIR.parent
for _p in (str(_THIS_DIR), str(_REPO_ROOT)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import cache_gst_replay_gate as gate  # noqa: E402

from hailo_tiling.cache.hashing import file_sha256  # noqa: E402
from hailo_tiling.cache.store import SqliteCacheStore  # noqa: E402

# Single source of truth for the normalized-grid -> tiles definition (also
# used by the ablation harness's replay-key computation, per the plan).
from tiling_benchmark.tiling_record import _grid_to_static_tiles  # noqa: E402


# --------------------------------------------------------------------------
# grid-spec parsing
# --------------------------------------------------------------------------

def parse_grid_spec(spec: str) -> list[tuple[int, int, float]]:
    """Parse a grid-spec string like ``"3x2:0.25;6x4:0.25;1x1:0.0"`` into a
    list of ``(tiles_x, tiles_y, overlap)`` tuples.

    Each item is ``NxM`` with an optional ``:OVERLAP`` suffix (default 0.0).
    Whitespace and trailing separators are tolerated.
    """
    out: list[tuple[int, int, float]] = []
    for item in spec.split(";"):
        item = item.strip()
        if not item:
            continue
        grid, _, ov = item.partition(":")
        nx_s, _, ny_s = grid.strip().lower().partition("x")
        if not nx_s or not ny_s:
            raise ValueError(f"bad grid spec item {item!r}: expected 'NxM[:overlap]'")
        nx, ny = int(nx_s), int(ny_s)
        overlap = float(ov) if ov.strip() else 0.0
        if not (0.0 <= overlap < 1.0):
            raise ValueError(f"overlap must be in [0,1), got {overlap} in {item!r}")
        out.append((nx, ny, overlap))
    if not out:
        raise ValueError(f"no grids parsed from {spec!r}")
    return out


def grid_to_tiles_static(nx: int, ny: int, overlap: float) -> str:
    """Expand one (nx, ny, overlap) grid to a ``tiles-static`` string using the
    canonical :func:`_grid_to_static_tiles` (single grid definition)."""
    rects = _grid_to_static_tiles(nx, ny, overlap, overlap)
    return ";".join(rects)


def default_cache_filename(video: str | Path, fov: str, hef: str | Path) -> str:
    """Default cache name ``<clipstem>__<fov>__<hefsha16>.sqlite3``.

    Mirrors the plan's naming convention. ``fov`` is a short tag like
    ``fov50``. The clip stem has any trailing ``__fovNN`` removed so the name
    is stable regardless of whether the source filename already carries it.
    """
    stem = Path(video).stem
    # Drop a trailing __fovNN if present so we don't double-stamp.
    for tag in ("__fov50", "__fov60", "__fov70"):
        if stem.endswith(tag):
            stem = stem[: -len(tag)]
            break
    hef_sha = file_sha256(hef)
    return f"{stem}__{fov}__{hef_sha[:16]}.sqlite3"


# --------------------------------------------------------------------------
# warming
# --------------------------------------------------------------------------

def _warm_one_grid_in_process(
    video: str,
    hef: str,
    out_cache: str,
    grid: tuple[int, int, float],
    source_width: int,
    source_height: int,
    max_frames: int = 0,
    post_so: str | None = None,
    post_fn: str | None = None,
) -> dict:
    """Warm a SINGLE grid into ``out_cache`` in THIS process.

    This is the in-process unit of work. Running more than one grid per
    process wedges GStreamer/HailoRT (state leak across repeated cropper +
    hailonet pipeline teardown/relaunch — Plan 6 A3 finding), so :func:`warm`
    spawns a fresh subprocess per grid and each child invokes exactly this.

    Returns a per-grid summary dict (no meta stamp — meta is stamped by the
    parent once all grids are done).
    """
    import gi

    gi.require_version("Gst", "1.0")
    from gi.repository import Gst  # noqa: E402

    Gst.init(None)

    post_so = post_so or gate._DEFAULT_POST_SO
    post_fn = post_fn or gate._DEFAULT_POST_FN
    frame_cap = max_frames if max_frames and max_frames > 0 else 10_000_000

    out_path = Path(out_cache)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    def _row_count() -> int:
        store = SqliteCacheStore.open(out_path)
        try:
            return int(store.stats()["n_rows"])
        finally:
            store.close()

    nx, ny, overlap = grid
    tiles_static = grid_to_tiles_static(nx, ny, overlap)
    n_tiles = len([r for r in tiles_static.split(";") if r.strip()])
    rows_before = _row_count() if out_path.exists() else 0

    inner = gate._inner_live(
        hef, post_so, post_fn, str(out_path), source_width, source_height
    )
    pipe = gate._build_pipeline(video, gate._cropper_subgraph(inner, tiles_static))
    label = f"{nx}x{ny}:{overlap}"
    print(f"[warm] grid={label} tiles={n_tiles} ...", flush=True)
    result = gate._run_pass(pipe, source_width, source_height, frame_cap, n_tiles)
    if result.error:
        raise RuntimeError(f"grid {label} pipeline error: {result.error}")

    rows_after = _row_count()
    added = rows_after - rows_before
    print(
        f"[warm] grid={label} rows_added={added} rows_total={rows_after}",
        flush=True,
    )
    return {
        "grid": label,
        "tiles_per_frame": n_tiles,
        "tiles_buffers_seen": len(result.records),
        "rows_added": added,
        "rows_after": rows_after,
    }


def _default_subprocess_runner(
    grid: tuple[int, int, float],
    *,
    video: str,
    hef: str,
    out_cache: str,
    source_width: int,
    source_height: int,
    max_frames: int,
    post_so: str | None,
    post_fn: str | None,
) -> None:
    """Spawn a fresh ``python warm_gst_cache.py --_single-grid-child`` for ONE
    grid. A fresh process per grid is the fix for the in-process multi-grid
    GStreamer/HailoRT teardown deadlock (Plan 6 A3). Raises on non-zero exit."""
    nx, ny, overlap = grid
    cmd = [
        sys.executable,
        str(_THIS_DIR / "warm_gst_cache.py"),
        "--_single-grid-child",
        "--video", str(video),
        "--hef", str(hef),
        "--out-cache", str(out_cache),
        "--grids", f"{nx}x{ny}:{overlap}",
        "--source-width", str(source_width),
        "--source-height", str(source_height),
        "--max-frames", str(max_frames),
    ]
    if post_so:
        cmd += ["--post-so", str(post_so)]
    if post_fn:
        cmd += ["--post-function", str(post_fn)]
    import subprocess

    subprocess.run(cmd, check=True)


def warm(
    video: str,
    hef: str,
    out_cache: str,
    grids: list[tuple[int, int, float]],
    source_width: int,
    source_height: int,
    max_frames: int = 0,
    post_so: str | None = None,
    post_fn: str | None = None,
    subprocess_runner=None,
) -> dict:
    """Warm ``out_cache`` by warming each grid in a FRESH SUBPROCESS.

    Running N grids in one long-lived process wedges GStreamer/HailoRT (state
    leak across repeated cropper + hailonet pipeline teardown/relaunch — Plan 6
    A3 finding). So the parent spawns one child per grid (each child does
    exactly one :func:`_warm_one_grid_in_process`), all appending to the SAME
    cache file (idempotent via A1 INSERT OR IGNORE). Meta is stamped once by the
    parent after all grids complete.

    ``subprocess_runner`` is the per-grid spawn callable (one positional ``grid``
    arg + keyword args); defaults to :func:`_default_subprocess_runner`. Tests
    inject a recording stub. Row counts are read from the cache after each child
    so the summary is accurate regardless of how the child ran.
    """
    runner = subprocess_runner or _default_subprocess_runner

    out_path = Path(out_cache)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    print(
        f"[warm] video={video}\n[warm] out-cache={out_cache}\n"
        f"[warm] src={source_width}x{source_height} grids={len(grids)} "
        f"max_frames={max_frames or 'ALL'} (subprocess-per-grid)",
        flush=True,
    )

    def _row_count() -> int:
        if not out_path.exists():
            return 0
        store = SqliteCacheStore.open(out_path)
        try:
            return int(store.stats()["n_rows"])
        finally:
            store.close()

    rows_before_total = _row_count()

    per_grid: list[dict] = []
    for grid in grids:
        nx, ny, overlap = grid
        label = f"{nx}x{ny}:{overlap}"
        rows_before = _row_count()
        print(f"[warm] grid={label} (fresh subprocess) ...", flush=True)
        runner(
            grid,
            video=video,
            hef=hef,
            out_cache=str(out_path),
            source_width=source_width,
            source_height=source_height,
            max_frames=max_frames,
            post_so=post_so,
            post_fn=post_fn,
        )
        rows_after = _row_count()
        tiles_static = grid_to_tiles_static(nx, ny, overlap)
        n_tiles = len([r for r in tiles_static.split(";") if r.strip()])
        per_grid.append(
            {
                "grid": label,
                "tiles_per_frame": n_tiles,
                "rows_added": rows_after - rows_before,
                "rows_after": rows_after,
            }
        )

    # Stamp meta once, after all grid children have written (idempotent upsert).
    store = SqliteCacheStore.open(out_path)
    try:
        store.meta_put("video_path", str(video))
        store.meta_put("video_sha256", file_sha256(video))
        store.meta_put("video_w", str(source_width))
        store.meta_put("video_h", str(source_height))
        store.meta_put("hef_path", str(hef))
        store.meta_put("hef_sha256", file_sha256(hef))
        store.meta_put("grid_spec", ";".join(f"{a}x{b}:{c}" for a, b, c in grids))
        store.meta_put("warmed_by", "warm_gst_cache.py")
        store.meta_put("created_at", str(time.time()))
        total_rows = int(store.stats()["n_rows"])
    finally:
        store.close()

    summary = {
        "out_cache": str(out_path),
        "grids": per_grid,
        "rows_before_total": rows_before_total,
        "total_rows": total_rows,
        "total_rows_added": total_rows - rows_before_total,
    }
    print(
        f"[warm] DONE total_rows={total_rows} "
        f"(added {summary['total_rows_added']})",
        flush=True,
    )
    return summary


# --------------------------------------------------------------------------
# cli
# --------------------------------------------------------------------------

def _build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="warm_gst_cache")
    ap.add_argument("--video", required=True)
    ap.add_argument(
        "--hef",
        default="/usr/local/hailo/resources/models/hailo10h/"
        "hailo_yolov8n_4_classes_vga.hef",
    )
    ap.add_argument("--out-cache", required=True)
    ap.add_argument(
        "--grids",
        required=True,
        help='Grid spec like "3x2:0.25;6x4:0.25;1x1:0.0" '
        "(NxM with optional :overlap, default overlap 0.0).",
    )
    ap.add_argument("--source-width", type=int, required=True)
    ap.add_argument("--source-height", type=int, required=True)
    ap.add_argument("--max-frames", type=int, default=0,
                    help="Cap frames (0 = all).")
    ap.add_argument("--post-so", default=None)
    ap.add_argument("--post-function", default=None)
    ap.add_argument(
        "--_single-grid-child",
        action="store_true",
        help="INTERNAL: run exactly one grid in-process (spawned by the "
        "subprocess-per-grid parent). Not for direct use.",
    )
    return ap


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_argparser().parse_args(argv)
    grids = parse_grid_spec(args.grids)
    if not os.environ.get("GST_PLUGIN_PATH"):
        print(
            "[warm] WARNING: GST_PLUGIN_PATH is unset — the hailocache* "
            "plugins live in gst-hailo-cache/build/src. Set it before running.",
            file=sys.stderr,
        )
    if getattr(args, "_single_grid_child", False):
        # Child invocation: warm exactly one grid in this fresh process.
        if len(grids) != 1:
            raise SystemExit(
                "--_single-grid-child requires exactly one grid, got "
                f"{len(grids)}"
            )
        _warm_one_grid_in_process(
            video=args.video,
            hef=args.hef,
            out_cache=args.out_cache,
            grid=grids[0],
            source_width=args.source_width,
            source_height=args.source_height,
            max_frames=args.max_frames,
            post_so=args.post_so,
            post_fn=args.post_function,
        )
        return 0
    warm(
        video=args.video,
        hef=args.hef,
        out_cache=args.out_cache,
        grids=grids,
        source_width=args.source_width,
        source_height=args.source_height,
        max_frames=args.max_frames,
        post_so=args.post_so,
        post_fn=args.post_function,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
