"""hailo-tiling-warm-cache — walk a video and pre-compute crops to the cache.

Walks `--video` frame by frame, builds a fixed crop set from `--grid` flags
(each `--grid NxM` adds an N×M discovery grid), runs inference through the
selected backend, and writes the results into a `(video_sha, hef_sha)`-keyed
SQLite file in `--cache-dir`.

The backend defaults to `mock` (a `MockBackend` that returns empty det-lists
unless canned) so the CLI is unit-testable without HailoRT. Plan 5 will add
`--backend hef` (live HefBackend) and `--backend gst` (GstCropperBackend)
options.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Iterable, Sequence

from ..backends import CachingBackend, MockBackend
from ..backends.backend import InferenceBackend
from ..budget import BudgetMeter
from ..cache.hashing import file_sha256
from ..cache.store import SqliteCacheStore
from ..emitters import DiscoveryGridEmitter
from ..types import CropRect, LockState


# ----------------------------------------------------------- argument parse


def _build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="hailo-tiling-warm-cache",
        description="Pre-compute per-crop detections to an SQLite tile cache.",
    )
    p.add_argument("--video", required=True, type=Path,
                   help="Input video (mp4 / mkv / any cv2.VideoCapture-readable).")
    p.add_argument("--hef", required=True, type=Path,
                   help="HEF file; SHA-256 is part of the cache filename.")
    p.add_argument("--src-w", type=int, default=None,
                   help="Override video width; default: probed from cv2.")
    p.add_argument("--src-h", type=int, default=None,
                   help="Override video height; default: probed from cv2.")
    p.add_argument("--grid", action="append", default=[],
                   help="N×M discovery grid; repeat to warm multiple grids.")
    p.add_argument("--cache-dir", type=Path, default=Path(".tile_cache"),
                   help="Directory holding cache files.")
    p.add_argument("--ppv", type=int, default=1,
                   help="post-process version (spec §7.2). Default 1.")
    p.add_argument("--score-floor", type=float, default=0.01,
                   help="Score floor cached at (spec §7.2). For metadata only.")
    p.add_argument("--backend", choices=["mock"], default="mock",
                   help="Inference backend. Plan 4 ships 'mock' only; "
                        "Plan 5 adds 'hef' and 'gst'.")
    p.add_argument("--batch-rows", type=int, default=128,
                   help="Flush put_many() every N rows.")
    p.add_argument("--max-frames", type=int, default=None,
                   help="Limit number of frames warmed (debugging).")
    return p


def parse_grids(args: Sequence[str]) -> list[tuple[int, int]]:
    """Parse '--grid 3x2' tokens into a list of (gx, gy) tuples."""
    out: list[tuple[int, int]] = []
    for tok in args:
        if "x" not in tok:
            sys.exit(f"--grid: expected NxM, got {tok!r}")
        try:
            a, b = tok.split("x", 1)
            out.append((int(a), int(b)))
        except (ValueError, TypeError):
            sys.exit(f"--grid: expected NxM, got {tok!r}")
    return out


# -------------------------------------------------------------- crop layout


def crops_for_grids(
    src_w: int,
    src_h: int,
    grids: Sequence[tuple[int, int]],
) -> list[CropRect]:
    """Build a deduped crop list by running each grid through DiscoveryGridEmitter.

    Deterministic: identical args → identical output (so the cache key set is
    reproducible across runs).
    """
    out: list[CropRect] = []
    seen: set[tuple[int, int, int, int]] = set()
    meter = BudgetMeter(budget_inf_per_s=1e9, fps=30.0)  # effectively unlimited
    for gx, gy in grids:
        e = DiscoveryGridEmitter(grid=(gx, gy), period=1, mode="m")
        for c in e.emit(src_w, src_h, LockState(), frame_idx=0, meter=meter):
            key = (c.x, c.y, c.w, c.h)
            if key not in seen:
                seen.add(key)
                out.append(c)
    return out


# ------------------------------------------------------------- video walker


def iter_video_frames(path: Path) -> Iterable[tuple[int, "any"]]:
    """Yield `(frame_idx, frame_bgr)` pairs from `path` using cv2.VideoCapture.

    Lazy-imports cv2 so test stubs can monkeypatch this function without
    triggering the cv2 import.
    """
    import cv2  # noqa: WPS433 — lazy
    cap = cv2.VideoCapture(str(path))
    try:
        i = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                return
            yield i, frame
            i += 1
    finally:
        cap.release()


def probe_video_dims(path: Path) -> tuple[int, int]:
    import cv2
    cap = cv2.VideoCapture(str(path))
    try:
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return w, h
    finally:
        cap.release()


# -------------------------------------------------------- backend factories


def make_backend(kind: str, **kwargs) -> InferenceBackend:
    """Build the inference backend selected by --backend.

    Plan 4 supports only `mock` (returns empty lists). Plan 5 will register
    'hef' and 'gst'.
    """
    if kind == "mock":
        return MockBackend()
    raise ValueError(f"Unknown backend: {kind}")  # pragma: no cover


# ------------------------------------------------------------- main loop


def warm_video(
    frames: Iterable[tuple[int, "any"]],
    crops: Sequence[CropRect],
    backend: InferenceBackend,
    store: SqliteCacheStore,
    *,
    ppv: int,
    batch_rows: int = 128,
) -> None:
    """Run `backend.infer(frame, crops, frame_idx)` over `frames`, cache misses.

    Each frame's results land in the store via a `CachingBackend` decorator
    (so we get the cache-skip-on-hit behaviour for free).
    """
    cb = CachingBackend(wrapped=backend, store=store, ppv=ppv)
    for frame_idx, frame in frames:
        cb.infer(frame, crops, frame_idx)
        # `batch_rows` is currently advisory — `put_many` already does one
        # transaction per frame. Larger batches (multi-frame) are a v2
        # optimisation if profiling shows it matters.


def _cache_filename(video_sha: str, hef_sha: str) -> str:
    return f"{video_sha[:16]}__{hef_sha[:16]}.sqlite3"


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_argparser().parse_args(argv)
    grids = parse_grids(args.grid)
    if not grids:
        sys.exit("--grid: at least one grid required")

    video_sha = file_sha256(args.video)
    hef_sha = file_sha256(args.hef)
    args.cache_dir.mkdir(parents=True, exist_ok=True)
    cache_path = args.cache_dir / _cache_filename(video_sha, hef_sha)

    if args.src_w is None or args.src_h is None:
        src_w, src_h = probe_video_dims(args.video)
        src_w = args.src_w or src_w
        src_h = args.src_h or src_h
    else:
        src_w, src_h = args.src_w, args.src_h

    crops = crops_for_grids(src_w, src_h, grids)

    store = SqliteCacheStore.open(cache_path)
    try:
        store.meta_put("video_sha256", video_sha)
        store.meta_put("video_path", str(args.video))
        store.meta_put("video_w", str(src_w))
        store.meta_put("video_h", str(src_h))
        store.meta_put("hef_sha256", hef_sha)
        store.meta_put("hef_path", str(args.hef))
        store.meta_put("ppv", str(args.ppv))
        store.meta_put("score_floor", str(args.score_floor))
        store.meta_put("created_at", str(time.time()))

        backend = make_backend(args.backend)
        frames = iter_video_frames(args.video)
        if args.max_frames is not None:
            def _capped(it, n):
                for i, x in enumerate(it):
                    if i >= n:
                        return
                    yield x
            frames = _capped(frames, args.max_frames)

        warm_video(frames=frames, crops=crops, backend=backend,
                   store=store, ppv=args.ppv, batch_rows=args.batch_rows)
    finally:
        store.close()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
