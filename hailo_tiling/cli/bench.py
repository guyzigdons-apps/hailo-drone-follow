"""hailo-tiling-bench — chip-free ablation harness CLI (Plan 6 Task B3).

Runs the ablation config matrix against a warmed source-pixel-keyed cache,
writing one ``<config>.frames.json`` per config and an ``ablation_table.md``
summarising each config's tiles/frame, detection count, recall/precision vs the
dense-GT reference row, and cache misses.

The whole thing is chip-free: every config replays the warmed cache via
``ReplayBackend`` (no ``hailonet`` anywhere). The reference row (the dense 12x9
GT) defines the recall/precision denominators (IoU-matched at 0.5).

Usage:
    hailo-tiling-bench --cache .tile_cache/0026__fov50__<sha>.sqlite3 \\
        --video /path/0026__fov50.mp4 [--configs default|name1,name2] \\
        [--max-frames N] [--out-dir runs/ablation_0026_fov50]
"""
from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path
from typing import Sequence

from ..bench.config import BenchConfig, default_matrix
from ..bench.metrics import recall_precision_vs_reference
from ..bench.runner import ConfigResult, run_config, run_static_config_crop_ordered
from ..cache.store import SqliteCacheStore
from ..types import Det


def _frames_in_cache(store: SqliteCacheStore, ppv: int = 1) -> list[int]:
    rows = store._con.execute(  # noqa: SLF001 — same-package read
        "SELECT DISTINCT frame_idx FROM detections WHERE ppv=? ORDER BY frame_idx",
        (int(ppv),),
    ).fetchall()
    return [int(r[0]) for r in rows]


def _is_per_tile_buffer_keyed(store: SqliteCacheStore, ppv: int = 1) -> tuple[bool, int]:
    """Detect whether a cache's ``frame_idx`` is a per-tile-buffer monotonic
    counter (GStreamer hailocachewriter) rather than a per-source-frame index.

    In a per-tile-buffer cache each distinct crop appears once per source
    frame, so the number of source frames == the max per-crop occurrence count,
    which is far smaller than the number of distinct frame_idx values. Returns
    ``(is_per_tile_buffer, n_source_frames)``.
    """
    n_distinct_frame_idx = store._con.execute(  # noqa: SLF001
        "SELECT COUNT(DISTINCT frame_idx) FROM detections WHERE ppv=?", (int(ppv),)
    ).fetchone()[0]
    max_occ = store._con.execute(  # noqa: SLF001
        "SELECT MAX(c) FROM (SELECT COUNT(*) c FROM detections WHERE ppv=? "
        "GROUP BY crop_x, crop_y, crop_w, crop_h)",
        (int(ppv),),
    ).fetchone()[0] or 0
    # Heuristic: if distinct frame_idx greatly exceeds the per-crop occurrence
    # count, frame_idx is a per-tile-buffer counter.
    is_ptb = n_distinct_frame_idx > 1.5 * max(1, max_occ)
    return bool(is_ptb), int(max_occ)


def _select_configs(spec: str) -> list[BenchConfig]:
    """Return the matrix rows to run. ``default`` = all. Otherwise a
    comma-separated name list; the reference row is always included (it defines
    the recall/precision denominators)."""
    matrix = default_matrix()
    if spec.strip().lower() in ("default", "all", ""):
        return matrix
    wanted = {s.strip() for s in spec.split(",") if s.strip()}
    by_name = {r.name: r for r in matrix}
    out: list[BenchConfig] = []
    for name in wanted:
        if name not in by_name:
            raise SystemExit(
                f"unknown config {name!r}; available: {sorted(by_name)}"
            )
        out.append(by_name[name])
    # Always include the reference row.
    ref = next(r for r in matrix if r.is_reference)
    if ref.name not in {c.name for c in out}:
        out.append(ref)
    return out


def _frame_dets_map(res: ConfigResult) -> dict[int, list[Det]]:
    return {f.frame_idx: f.dets for f in res.frames}


def _det_to_dict(d: Det) -> dict:
    return {"cls": d.cls, "score": d.score, "x": d.x, "y": d.y, "w": d.w, "h": d.h}


def _write_frames_json(out_dir: Path, res: ConfigResult) -> Path:
    path = out_dir / f"{res.name}.frames.json"
    doc = {
        "config": res.name,
        "kind": res.kind,
        "n_misses_total": res.n_misses_total,
        "frames": [
            {
                "frame_idx": f.frame_idx,
                "n_tiles": f.n_tiles,
                "n_misses": f.n_misses,
                "dets": [_det_to_dict(d) for d in f.dets],
            }
            for f in res.frames
        ],
    }
    path.write_text(json.dumps(doc, indent=2))
    return path


def _format_table(rows: list[dict]) -> str:
    header = (
        "| config | kind | mean_tiles_per_frame | n_dets | "
        "recall_vs_reference | precision_vs_reference | n_misses |\n"
        "|---|---|---:|---:|---:|---:|---:|\n"
    )
    lines = []
    for r in rows:
        rec = "—" if r["recall"] is None else f"{r['recall']:.4f}"
        prec = "—" if r["precision"] is None else f"{r['precision']:.4f}"
        lines.append(
            f"| {r['name']} | {r['kind']} | {r['mean_tiles']:.2f} | "
            f"{r['n_dets']} | {rec} | {prec} | {r['n_misses']} |"
        )
    return header + "\n".join(lines) + "\n"


def run(
    cache: str,
    video: str,
    out_dir: Path,
    configs_spec: str = "default",
    max_frames: int = 0,
    ppv: int = 1,
    iou_thr: float = 0.5,
) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    store = SqliteCacheStore.open(cache)
    try:
        vw = store.meta_get("video_w")
        vh = store.meta_get("video_h")
        if vw is None or vh is None:
            raise SystemExit(
                f"{cache}: cache meta lacks video_w/video_h — re-warm with "
                "source-width/source-height set."
            )
        video_meta = {"src_w": int(vw), "src_h": int(vh)}

        per_tile_buffer, n_src_frames = _is_per_tile_buffer_keyed(store, ppv=ppv)
        if per_tile_buffer:
            # GST writer keys frame_idx per tile-buffer; reconstruct source
            # frames by crop-occurrence order. Static rows replay via the
            # crop-ordered path; dynamic rows are not supported on such a cache
            # in chip-free v1 (need live frame indexing) and are skipped.
            frames = list(range(n_src_frames))
            if max_frames and max_frames > 0:
                frames = frames[:max_frames]
            max_k = (max_frames if max_frames and max_frames > 0 else None)

            def _run(cfg: BenchConfig) -> ConfigResult:
                res = run_static_config_crop_ordered(cfg, store, video_meta, ppv=ppv)
                if max_k is not None:
                    res.frames = res.frames[:max_k]
                return res
        else:
            frames = _frames_in_cache(store, ppv=ppv)
            if max_frames and max_frames > 0:
                frames = frames[:max_frames]
            if not frames:
                raise SystemExit(f"{cache}: no frames in cache")

            def _run(cfg: BenchConfig) -> ConfigResult:
                return run_config(cfg, store, video_meta, frames, ppv=ppv)

        configs = _select_configs(configs_spec)
        if per_tile_buffer:
            skipped = [c.name for c in configs if c.kind == "dynamic"]
            configs = [c for c in configs if c.kind == "static"]
            if skipped:
                print(
                    f"[bench] per-tile-buffer cache: skipping dynamic rows "
                    f"{skipped} (need live frame indexing; static-only table)."
                )

        # Run the reference row first so its frame-dets map is available as the
        # recall/precision denominator for every other config.
        ref_cfg = next(c for c in configs if c.is_reference)
        results: dict[str, ConfigResult] = {}
        results[ref_cfg.name] = _run(ref_cfg)
        ref_map = _frame_dets_map(results[ref_cfg.name])

        for cfg in configs:
            if cfg.name in results:
                continue
            results[cfg.name] = _run(cfg)

        # Build the table (configs in matrix order, reference last for clarity).
        ordered = [c for c in configs if not c.is_reference] + [ref_cfg]
        table_rows: list[dict] = []
        for cfg in ordered:
            res = results[cfg.name]
            _write_frames_json(out_dir, res)
            if cfg.is_reference:
                # Reference vs itself = 1.0 by definition; record as such.
                recall, precision = 1.0, 1.0
            else:
                recall, precision, _ = recall_precision_vs_reference(
                    _frame_dets_map(res), ref_map, iou_thr=iou_thr
                )
            table_rows.append(
                {
                    "name": cfg.name,
                    "kind": cfg.kind,
                    "mean_tiles": res.mean_tiles_per_frame,
                    "n_dets": res.n_dets_total,
                    "recall": recall,
                    "precision": precision,
                    "n_misses": res.n_misses_total,
                }
            )

        table_md = (
            f"# Ablation table\n\n"
            f"- cache: `{cache}`\n- video: `{video}`\n"
            f"- source: {video_meta['src_w']}x{video_meta['src_h']}\n"
            f"- frames: {len(frames)}\n"
            f"- reference: `{ref_cfg.name}` ({ref_cfg.tiles_x}x{ref_cfg.tiles_y}), "
            f"IoU>={iou_thr}\n\n"
            + _format_table(table_rows)
        )
        table_path = out_dir / "ablation_table.md"
        table_path.write_text(table_md)
        print(table_md)
        print(f"[bench] wrote {table_path}")
        return {
            "out_dir": str(out_dir),
            "table": str(table_path),
            "n_frames": len(frames),
            "configs": [r["name"] for r in table_rows],
            "rows": table_rows,
        }
    finally:
        store.close()


def _build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="hailo-tiling-bench")
    ap.add_argument("--cache", required=True, help="Warmed source-pixel-keyed SQLite cache.")
    ap.add_argument("--video", required=True, help="Source video (recorded in the table).")
    ap.add_argument("--configs", default="default",
                    help='"default" (all rows) or comma-separated config names.')
    ap.add_argument("--max-frames", type=int, default=0, help="Cap frames (0 = all).")
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--ppv", type=int, default=1)
    ap.add_argument("--iou-thr", type=float, default=0.5)
    return ap


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_argparser().parse_args(argv)
    run(
        cache=args.cache,
        video=args.video,
        out_dir=args.out_dir,
        configs_spec=args.configs,
        max_frames=args.max_frames,
        ppv=args.ppv,
        iou_thr=args.iou_thr,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
