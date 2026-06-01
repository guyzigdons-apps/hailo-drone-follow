#!/usr/bin/env python3
"""Bounded dynamic-vs-static eval for the 8x6-discovery demo (chip-free replay).

Replays a dynamic cache (warmed by warm_dynamic_cache.py with a discovery-grid
override) and a static cache over the SAME frame prefix, scoring person+vehicle
recall against the 12x9 reference. Writes the dynamic config's frames.json (with
ROI tile rectangles) for the overlay viewer and prints a matched-compute table.

This is a one-shot experiment harness (not the full bench CLI): it pins the same
discovery-grid override on the dynamic replay so the requested crops match the
warmed cache (else every ROI tile would be a miss).
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hailo_tiling.backends.replay import ReplayBackend
from hailo_tiling.bench.config import default_matrix
from hailo_tiling.bench.metrics import (
    matched_compute_delta,
    recall_precision_vs_reference,
)
from hailo_tiling.bench.runner import (
    run_dynamic_config,
    run_static_config_crop_ordered,
)
from hailo_tiling.cache.store import SqliteCacheStore
from hailo_tiling.classes import PERSON, TRACKED_CLASSES
from warm_dynamic_cache import gt_traj_from_reference


def _ref_map(ref_path: Path, n: int) -> dict:
    """{frame_idx: [Det,...]} for the first n reference frames (person+vehicle)."""
    from hailo_tiling.types import Det
    doc = json.loads(ref_path.read_text())
    out = {}
    for f in doc.get("frames", [])[:n]:
        out[f["frame_idx"]] = [
            Det(cls=int(d["cls"]), score=d.get("score", 1.0),
                x=d["x"], y=d["y"], w=d["w"], h=d["h"])
            for d in f.get("dets", [])
        ]
    return out


def _det_map(res, n):
    return {f.frame_idx: f.dets for f in res.frames if f.frame_idx < n}


def main() -> int:
    ap = argparse.ArgumentParser(prog="dynamic_demo_eval")
    ap.add_argument("--dynamic-cache", required=True)
    ap.add_argument("--static-cache", required=True)
    ap.add_argument("--ref", required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--max-frames", type=int, required=True)
    ap.add_argument("--discovery-grid", default="8x6")
    ap.add_argument("--static-configs", default="1x1,3x2,8x6,12x9")
    ap.add_argument("--iou-thr", type=float, default=0.5)
    args = ap.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    n = args.max_frames
    frames = list(range(n))
    ref_map = _ref_map(Path(args.ref), n)
    gx, gy = (int(v) for v in args.discovery_grid.lower().split("x"))

    dyn_store = SqliteCacheStore.open(args.dynamic_cache)
    src_w = int(dyn_store.meta_get("video_w"))
    src_h = int(dyn_store.meta_get("video_h"))
    meta = {"src_w": src_w, "src_h": src_h}
    gt = gt_traj_from_reference(Path(args.ref), PERSON)

    rows = []
    # --- dynamic 8x6 ---
    dyn_cfg = next(c for c in default_matrix() if c.name == "dynamic")
    kw = dict(dyn_cfg.scheduler_kwargs); kw["discovery_grid"] = (gx, gy)
    dyn_cfg = dataclasses.replace(dyn_cfg, scheduler_kwargs=kw)
    backend = ReplayBackend(dyn_store, ppv=1)
    dyn_res = run_dynamic_config(dyn_cfg, backend, meta, frames,
                                 gt_traj=gt, fps=30.0, person_cls=PERSON, ppv=1)
    dr, dp, _ = recall_precision_vs_reference(
        _det_map(dyn_res, n), ref_map, iou_thr=args.iou_thr,
        keep_classes=TRACKED_CLASSES)
    _write_frames(args.out_dir, dyn_res, f"dynamic_{gx}x{gy}disc")

    # --- static configs over the same prefix ---
    static_store = SqliteCacheStore.open(args.static_cache)
    matrix = {c.name: c for c in default_matrix()}
    static_rows = []
    for name in args.static_configs.split(","):
        cfg = matrix[name]
        res = run_static_config_crop_ordered(cfg, static_store, meta, ppv=1)
        rr, rp, _ = recall_precision_vs_reference(
            _det_map(res, n), ref_map, iou_thr=args.iou_thr,
            keep_classes=TRACKED_CLASSES)
        mt = sum(f.n_tiles for f in res.frames[:n]) / n
        static_rows.append({"name": name, "mean_tiles": mt, "recall": rr})
        rows.append((name, "static", mt, rr, rp))
        _write_frames(args.out_dir, res, name, limit=n)

    dyn_mt = dyn_res.mean_tiles_per_frame
    ms, md = matched_compute_delta(dyn_mt, dr, static_rows)
    rows.append((f"dynamic({gx}x{gy}disc)", "dynamic", dyn_mt, dr, dp))

    print(f"\n=== dynamic-vs-static, first {n} frames, person+vehicle recall ===")
    print(f"{'config':<22}{'kind':<9}{'tiles/frame':>12}{'recall':>9}{'prec':>8}")
    for name, kind, mt, rr, rp in rows:
        print(f"{name:<22}{kind:<9}{mt:>12.2f}{rr:>9.4f}{rp:>8.4f}")
    print(f"\nmatched-compute: dynamic ({dyn_mt:.2f} tiles) vs closest static "
          f"'{ms}' -> recall delta {md:+.4f}")
    dyn_store.close(); static_store.close()
    return 0


def _write_frames(out_dir: Path, res, name: str, limit: int | None = None):
    fr = res.frames if limit is None else [f for f in res.frames if f.frame_idx < limit]
    doc = {"config": name, "kind": res.kind, "n_misses_total": res.n_misses_total,
           "frames": [{
               "frame_idx": f.frame_idx, "n_tiles": f.n_tiles, "n_misses": f.n_misses,
               "dets": [{"cls": d.cls, "score": d.score, "x": d.x, "y": d.y,
                         "w": d.w, "h": d.h} for d in f.dets],
               "tiles": [{"x": t[0], "y": t[1], "w": t[2], "h": t[3], "category": t[4]}
                         for t in f.tiles],
           } for f in fr]}
    (out_dir / f"{name}.frames.json").write_text(json.dumps(doc))


if __name__ == "__main__":
    raise SystemExit(main())
