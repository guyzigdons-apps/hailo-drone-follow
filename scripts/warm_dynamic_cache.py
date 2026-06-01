"""Warm the dynamic-config ROI tiles on-chip (Night-2 Task A3) into a
per-source-frame SQLite cache via CachingBackend(GstCropperBackend).

Unlike static grids, the dynamic scheduler's track-guided ROI tiles are
run-specific and cannot be pre-enumerated. We run the dynamic configs once
on-chip through the live GstCropperBackend wrapped in CachingBackend (which
writes rows keyed by the TRUE source frame index via the Python store), driven
by the stateful bench runner (scheduler + production ByteTracker). The tracker
is deterministic given the per-frame detection sequence, so a later chip-free
ReplayBackend run requests exactly the crops warmed here -> 0 misses.

Target class: the single-target tracker locks the PERSON class (cls 1 in the
hailo_4_classes HEF — see hailo_tiling.classes; id 0 is the json's "unlabeled"
slot). The committed 12x9 reference is the GT trajectory source.

Usage:
    source setup_env.sh
    GST_PLUGIN_PATH=gst-hailo-cache/build/src HAILO_CHIP=1 \
      python scripts/warm_dynamic_cache.py \
        --video /path/0026__fov50.mp4 \
        --ref dynamic_tiling/runs/ablation_0026_fov50/12x9.frames.json \
        --out-cache .tile_cache/0026__fov50__dynamic.sqlite3 \
        --source-width 3840 --source-height 2160 \
        --target-class 1 [--max-frames N] [--configs dynamic,dynamic+asahi]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _THIS_DIR.parent
for _p in (str(_THIS_DIR), str(_REPO_ROOT)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from hailo_tiling.classes import PERSON, label as _class_label  # noqa: E402

_DEFAULT_HEF = ("/usr/local/hailo/resources/models/hailo10h/"
                "hailo_yolov8n_4_classes_vga.hef")
_DEFAULT_POST = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"


def gt_traj_from_reference(ref_path: Path, target_cls: int) -> dict:
    """Stitch a single-target trajectory for ``target_cls`` from a committed
    bench ``<ref>.frames.json`` (schema: frame_idx / dets / cls), via
    dynamic_tiling.gt_track.build_target_trajectory (schema: frame / detections
    / label)."""
    from dynamic_tiling.gt_track import build_target_trajectory

    doc = json.loads(ref_path.read_text())
    label = _class_label(target_cls)
    adapted = {
        "frames": [
            {
                "frame": f["frame_idx"],
                "detections": [
                    {
                        "label": _class_label(d["cls"]),
                        "bbox": [d["x"], d["y"], d["w"], d["h"]],
                        "confidence": d.get("score", 1.0),
                    }
                    for d in f.get("dets", [])
                ],
            }
            for f in doc.get("frames", [])
        ]
    }
    return build_target_trajectory(adapted, label=label, anchor="largest")


def warm_dynamic(
    video: str,
    ref: str,
    out_cache: str,
    source_width: int,
    source_height: int,
    target_cls: int = 2,
    hef: str | None = None,
    post_so: str | None = None,
    configs: list[str] | None = None,
    max_frames: int = 0,
    ppv: int = 1,
) -> dict:
    from hailo_tiling.backends.caching import CachingBackend
    from hailo_tiling.backends.gst_cropper import GstCropperBackend
    from hailo_tiling.bench.config import default_matrix
    from hailo_tiling.bench.runner import run_dynamic_config
    from hailo_tiling.cache.hashing import file_sha256
    from hailo_tiling.cache.store import SqliteCacheStore

    hef = hef or _DEFAULT_HEF
    post_so = post_so or _DEFAULT_POST
    gt = gt_traj_from_reference(Path(ref), target_cls)
    n_ref_frames = len(json.loads(Path(ref).read_text()).get("frames", []))
    n_frames = max_frames if (max_frames and max_frames > 0) else n_ref_frames
    frames = list(range(n_frames))

    matrix = {c.name: c for c in default_matrix() if c.kind == "dynamic"}
    want = configs or list(matrix)
    cfgs = [matrix[n] for n in want if n in matrix]

    out_path = Path(out_cache)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    store = SqliteCacheStore.open(out_path)
    summary = {"out_cache": str(out_path), "target_cls": target_cls,
               "n_frames": n_frames, "configs": {}}
    try:
        store.meta_put("video_path", str(video))
        store.meta_put("video_w", str(source_width))
        store.meta_put("video_h", str(source_height))
        store.meta_put("hef_path", str(hef))
        store.meta_put("hef_sha256", file_sha256(hef))
        store.meta_put("target_cls", str(target_cls))
        store.meta_put("warmed_by", "warm_dynamic_cache.py")
        store.meta_put("created_at", str(time.time()))

        backend = GstCropperBackend(hef=hef, post_so=post_so,
                                    source_w=source_width, source_h=source_height)
        caching = CachingBackend(backend, store, ppv=ppv)
        for cfg in cfgs:
            rows_before = int(store.stats()["n_rows"])
            print(f"[warm-dyn] config={cfg.name} frames={n_frames} ...", flush=True)
            res = run_dynamic_config(
                cfg, caching, {"src_w": source_width, "src_h": source_height},
                frames, gt_traj=gt, fps=30.0, person_cls=target_cls, ppv=ppv,
            )
            rows_after = int(store.stats()["n_rows"])
            summary["configs"][cfg.name] = {
                "mean_tiles_per_frame": res.mean_tiles_per_frame,
                "n_dets_total": res.n_dets_total,
                "n_misses_total": res.n_misses_total,
                "rows_added": rows_after - rows_before,
                "rows_after": rows_after,
            }
            print(f"[warm-dyn] config={cfg.name} mean_tiles={res.mean_tiles_per_frame:.2f} "
                  f"rows_added={rows_after - rows_before} misses(warm)={res.n_misses_total}",
                  flush=True)
    finally:
        store.close()
    print(f"[warm-dyn] DONE {json.dumps(summary)}", flush=True)
    return summary


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="warm_dynamic_cache")
    ap.add_argument("--video", required=True)
    ap.add_argument("--ref", required=True, help="Committed 12x9 reference frames.json.")
    ap.add_argument("--out-cache", required=True)
    ap.add_argument("--source-width", type=int, required=True)
    ap.add_argument("--source-height", type=int, required=True)
    ap.add_argument("--target-class", type=int, default=PERSON,
                    help="Single-target class (default 1 = person; "
                    "see hailo_tiling.classes).")
    ap.add_argument("--hef", default=None)
    ap.add_argument("--post-so", default=None)
    ap.add_argument("--configs", default=None,
                    help="Comma-separated dynamic config names (default: all dynamic rows).")
    ap.add_argument("--max-frames", type=int, default=0)
    args = ap.parse_args(argv)
    configs = [c.strip() for c in args.configs.split(",")] if args.configs else None
    warm_dynamic(
        video=args.video, ref=args.ref, out_cache=args.out_cache,
        source_width=args.source_width, source_height=args.source_height,
        target_cls=args.target_class, hef=args.hef, post_so=args.post_so,
        configs=configs, max_frames=args.max_frames,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
