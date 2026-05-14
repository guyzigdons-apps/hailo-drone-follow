"""Microbenchmark: cost of ReID gallery update on the streaming thread.

Compares two paths the detection callback can take when a target is locked
and `should_update()` fires:

  sync  — call ReIDManager.update_gallery(frame, bbox, w, h) directly
  async — call ReIDWorker.submit_gallery_update(...) (just queue.put_nowait)

The delta (sync - async) is the wall-clock time the streaming thread is
freed of per gallery-update call. At default cadence (update_interval=30,
30 fps), that's once per second.

This script does NOT exercise the full GStreamer callback — we only care
about the cost the callback would pay. The frame and bbox are synthesized
because the NPU runtime is content-independent (the model resizes the
crop to a fixed input shape).
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time
from pathlib import Path

import numpy as np

# Repo-rooted imports
HERE = Path(__file__).resolve().parent
APP_ROOT = HERE.parent
sys.path.insert(0, str(APP_ROOT))

import hailo
from robot_follow.pipeline_adapter.reid_manager import ReIDManager
from robot_follow.pipeline_adapter.reid_worker import ReIDWorker


def synth_frame(width: int, height: int) -> np.ndarray:
    """Produce a deterministic BGR frame with some gradient. Content is
    irrelevant to NPU runtime — we just need a real numpy array.
    """
    rng = np.random.default_rng(42)
    return rng.integers(0, 255, size=(height, width, 3), dtype=np.uint8)


def percentile(samples_ms: list[float], q: float) -> float:
    if not samples_ms:
        return float("nan")
    arr = np.asarray(samples_ms)
    return float(np.percentile(arr, q))


def stats(label: str, samples_ms: list[float]) -> None:
    if not samples_ms:
        print(f"{label}: no samples")
        return
    n = len(samples_ms)
    mean = statistics.fmean(samples_ms)
    p50 = percentile(samples_ms, 50)
    p95 = percentile(samples_ms, 95)
    p99 = percentile(samples_ms, 99)
    pmax = max(samples_ms)
    print(f"{label:36s} n={n:5d}  mean={mean:8.3f} ms  P50={p50:7.3f}  "
          f"P95={p95:7.3f}  P99={p99:7.3f}  max={pmax:7.3f}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hef", default="/usr/local/hailo/resources/models/hailo8/repvgg_a0_person_reid_512.hef",
                    help="Path to ReID HEF")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--bbox-w", type=float, default=0.15,
                    help="Bbox width in normalized coords (0..1)")
    ap.add_argument("--bbox-h", type=float, default=0.45,
                    help="Bbox height in normalized coords (0..1)")
    ap.add_argument("--sync-iters", type=int, default=200)
    ap.add_argument("--async-iters", type=int, default=2000,
                    help="Worker dispatch is cheap — use more iters")
    ap.add_argument("--warmup", type=int, default=10)
    args = ap.parse_args()

    if not os.path.isfile(args.hef):
        print(f"ERROR: HEF not found: {args.hef}", file=sys.stderr)
        return 1

    print(f"hef:    {args.hef}")
    print(f"frame:  {args.width}x{args.height} BGR")
    print(f"bbox:   {args.bbox_w:.2f} x {args.bbox_h:.2f} (normalized)")
    print()

    frame = synth_frame(args.width, args.height)
    bbox = hailo.HailoBBox(0.1, 0.1, args.bbox_w, args.bbox_h)

    # --- Build ReIDManager ----------------------------------------------------
    # Set update_interval=1 so should_update() is irrelevant — we call
    # update_gallery directly. Use a dummy original_id so the gallery has
    # somewhere to put embeddings.
    mgr = ReIDManager(
        hef_path=args.hef,
        update_interval=1,
        max_gallery_size=10,
        reid_match_threshold=0.75,
    )
    mgr.on_target_selected(track_id=1)  # initialize gallery for id=1

    # --- Warmup ---------------------------------------------------------------
    print(f"Warmup: {args.warmup} sync update_gallery calls...", flush=True)
    for _ in range(args.warmup):
        mgr.update_gallery(frame, bbox, args.width, args.height)

    # --- Sync benchmark -------------------------------------------------------
    print(f"Bench:  {args.sync_iters} sync update_gallery calls...", flush=True)
    sync_samples: list[float] = []
    for _ in range(args.sync_iters):
        t0 = time.perf_counter()
        mgr.update_gallery(frame, bbox, args.width, args.height)
        sync_samples.append((time.perf_counter() - t0) * 1000.0)

    # --- Async dispatch benchmark --------------------------------------------
    # Worker pulls from queue and calls update_gallery itself. The cost
    # paid by the streaming thread is just submit_gallery_update.
    print(f"Bench:  {args.async_iters} submit_gallery_update calls (worker drains in background)...",
          flush=True)
    worker = ReIDWorker(mgr, max_queue=2)
    worker.start()

    async_samples: list[float] = []
    for _ in range(args.async_iters):
        t0 = time.perf_counter()
        worker.submit_gallery_update(frame, bbox, args.width, args.height)
        async_samples.append((time.perf_counter() - t0) * 1000.0)

    drops = worker.dropped_count()

    # --- Report (BEFORE worker stop, in case shutdown races NPU cleanup) -----
    print()
    print("=" * 100)
    stats("sync update_gallery (streaming thread)", sync_samples)
    stats("async submit (streaming thread)", async_samples)
    print("=" * 100)
    sync_mean = statistics.fmean(sync_samples)
    async_mean = statistics.fmean(async_samples)
    delta = sync_mean - async_mean
    print(f"\nMean per-call delta (sync − async): {delta:.3f} ms")
    print(f"Worker dropped (queue full overruns): {drops} / {args.async_iters}")

    # --- Pipeline projection --------------------------------------------------
    # update_interval=30 frames @ 30 fps → 1 gallery call/sec
    # update_interval=30 frames @ 60 fps → 2 gallery calls/sec
    print()
    print("Streaming-thread time saved per second by going async:")
    for fps in (15, 30, 60):
        for interval in (30,):
            calls_per_sec = fps / interval
            saved_ms = calls_per_sec * delta
            print(f"  {fps:2d} fps, update_interval={interval}: "
                  f"{calls_per_sec:.2f} calls/sec → ~{saved_ms:.2f} ms/sec freed "
                  f"({saved_ms / 1000.0 * 100:.2f}% of one-CPU-second)")

    # Per-frame impact: how much would a single sync call delay one frame?
    frame_budget_ms_30 = 1000.0 / 30.0
    frame_budget_ms_60 = 1000.0 / 60.0
    print()
    print(f"Per-frame budget @ 30fps = {frame_budget_ms_30:.2f} ms; "
          f"sync call uses {sync_mean / frame_budget_ms_30 * 100:.1f}% of one frame")
    print(f"Per-frame budget @ 60fps = {frame_budget_ms_60:.2f} ms; "
          f"sync call uses {sync_mean / frame_budget_ms_60 * 100:.1f}% of one frame")

    # Now stop the worker. If this hangs/crashes the process exit is fine —
    # we already printed everything that matters.
    sys.stdout.flush()
    sys.stderr.flush()
    try:
        worker.stop(timeout=10.0)
    except Exception as e:
        print(f"(worker.stop raised: {e})")
    # Drop the manager's NPU handle BEFORE process exit so HailoRT can clean
    # up gracefully. The atexit segfault came from racing dtors.
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
