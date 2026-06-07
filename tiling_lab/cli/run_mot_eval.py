"""CLI: score a --dump-mot predictions file against verified GT tracks,
OR build a static-cache MOT baseline by replaying cached dense tile dets
through ByteTracker (MultiTargetLock) — no chip required.

Score an existing predictions file::

    source setup_env.sh
    python -m tiling_lab.cli.run_mot_eval \
        --gt-tracks tiling_lab/runs/gt_verify_0026_fov50/gt_tracks.verified.json \
        --pred tiling_lab/runs/mot/dyn_0026_fov50.json \
        --classes 1 --iou-thr 0.5 \
        --out tiling_lab/runs/mot/dyn_0026_fov50.report.json

Build a static-cache baseline (cached dense dets -> ByteTracker -> preds)::

    python -m tiling_lab.cli.run_mot_eval \
        --from-static-cache tiling_lab/runs/cache/dense_0026.db \
        --classes 1 --src-wh 3840x2160 --fps 30 \
        --pred-out tiling_lab/runs/mot/static_0026_fov50.json \
        --gt-tracks tiling_lab/runs/gt_verify_0026_fov50/gt_tracks.verified.json

GT format (same as run_trials._load_tracks):
    {"tracks": [{"cls": int, "track_id": int, "frames": {frame_str: [x,y,w,h]}}]}
Pred format (run_dynamic --dump-mot):
    {"tracks": {tid_str: {frame_str: [x,y,w,h]}}}

Both are loaded into MOT dicts {id: {frame_int: (x,y,w,h)}} and scored via
mot_metrics.score_mot (greedy IoU matching).

Static-cache caveat
-------------------
The replay enumerates EVERY distinct crop rect stored per frame_idx in the
cache and runs all their detections through the tracker.  If more than one
tile grid was ever warmed into the DB (e.g. an 8x6 sweep plus extra zoom
tiles), the extra tiles only ADD detections and therefore only HELP recall.
This is therefore an OPTIMISTIC static baseline — possibly denser than a pure
8x6 sweep.  The mean number of distinct crops actually used per frame is
reported (and stored in the report) so the tiles/frame asymmetry is explicit."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from tiling_lab.harness.aggregator import map_to_source, nms
from tiling_lab.harness.mot_metrics import score_mot

# Order metrics print in.
_METRIC_ORDER = [
    "MOTA", "IDF1", "IDsw", "FP", "FN", "Frag", "MT", "ML",
    "n_gt", "n_pred", "n_frames",
]
_FLOAT_KEYS = {"MOTA", "IDF1"}


def load_gt_as_mot(path: Path, classes: set[int]) -> dict:
    """Load verified GT tracks, keep only tracks whose `cls` is in `classes`,
    return {track_id: {frame_int: (x,y,w,h)}}."""
    doc = json.loads(Path(path).read_text())
    out: dict = {}
    for t in doc["tracks"]:
        if t["cls"] not in classes:
            continue
        out[t["track_id"]] = {int(f): tuple(b) for f, b in t["frames"].items()}
    return out


def load_pred(path: Path) -> dict:
    """Load a --dump-mot predictions file -> {track_id_int: {frame_int: (x,y,w,h)}}."""
    doc = json.loads(Path(path).read_text())
    return {int(tid): {int(f): tuple(b) for f, b in traj.items()}
            for tid, traj in doc["tracks"].items()}


def _parse_wh(s: str) -> tuple[int, int]:
    """Parse a 'WxH' (or 'W,H') source-dimensions string -> (w, h)."""
    sep = "x" if "x" in s.lower() else ","
    parts = s.lower().split(sep)
    if len(parts) != 2:
        raise ValueError(f"--src-wh must be WxH (e.g. 3840x2160), got {s!r}")
    return int(parts[0]), int(parts[1])


def _cache_src_wh(store, fallback: tuple[int, int] | None) -> tuple[int, int]:
    """Resolve source dims from cache meta (video_w/video_h), else fallback."""
    vw, vh = store.meta_get("video_w"), store.meta_get("video_h")
    if vw is not None and vh is not None:
        return int(vw), int(vh)
    if fallback is None:
        raise ValueError(
            "cache has no video_w/video_h meta; pass --src-wh WxH explicitly")
    return fallback


def replay_static_cache(db_path, *, classes, src_wh=None, nms_thr=0.5,
                        fps=30, ppv=0):
    """Replay cached dense tile detections through a MultiTargetLock.

    Enumerates every DISTINCT crop rect per frame_idx in the cache, reads each
    crop's local dets, maps them to source coords, NMS-merges, filters to
    `classes`, and steps a fresh MultiTargetLock (target_classes=classes,
    frame_rate=fps) frame-by-frame.  Collects each frame's non-LOST targets
    keyed (cls, track_id), mirroring run_multi.

    Returns (by_track, mean_tiles_per_frame, n_frames) where
    by_track = {track_id: {frame_idx: (x,y,w,h)}} restricted to `classes`."""
    from hailo_tiling.cache.store import SqliteCacheStore
    from hailo_tiling.types import CropRect

    from tiling_lab.harness.target_lock import MultiTargetLock

    class_set = set(classes)
    store = SqliteCacheStore.open(db_path)
    try:
        src_w, src_h = _cache_src_wh(store, src_wh)
        # All distinct (frame_idx, crop_rect) rows for the requested ppv, in
        # frame order.  DISTINCT collapses identical rows warmed across re-runs.
        rows = store._con.execute(
            "SELECT DISTINCT frame_idx, crop_x, crop_y, crop_w, crop_h "
            "FROM detections WHERE ppv=? ORDER BY frame_idx",
            (ppv,),
        ).fetchall()
        crops_by_frame: dict[int, list] = {}
        for fi, cx, cy, cw, ch in rows:
            crops_by_frame.setdefault(int(fi), []).append(
                CropRect(x=int(cx), y=int(cy), w=int(cw), h=int(ch)))

        lock = MultiTargetLock(target_classes=class_set,
                               track_buffer=int(fps), frame_rate=int(fps))

        by_track: dict[int, dict] = {}
        total_tiles = 0
        frame_indices = sorted(crops_by_frame)
        for fi in frame_indices:
            crops = crops_by_frame[fi]
            total_tiles += len(crops)
            dets: list = []
            for crop in crops:
                local = store.get(fi, crop, ppv=ppv)
                if not local:
                    continue
                dets += map_to_source(local, crop, src_w, src_h)
            dets = nms(dets, iou_thr=nms_thr)
            dets = [d for d in dets if d.cls in class_set]
            lock.step(dets)
            # Mirror run_multi: every non-LOST target with a real bbox.
            for (cls, tid), t in lock.targets.items():
                if t.status == "LOST" or t.bbox_norm[2] <= 0:
                    continue
                if cls not in class_set:
                    continue
                by_track.setdefault(tid, {})[fi] = tuple(t.bbox_norm)
    finally:
        store.close()

    n_frames = len(frame_indices)
    mean_tiles = total_tiles / n_frames if n_frames else 0.0
    return by_track, mean_tiles, n_frames


def by_track_to_pred_doc(by_track: dict) -> dict:
    """Serialise {tid: {frame: bbox}} to the --dump-mot JSON shape."""
    return {"tracks": {str(tid): {str(f): list(b) for f, b in traj.items()}
                       for tid, traj in by_track.items()}}


def format_metrics(metrics: dict) -> str:
    """Aligned two-column metric table."""
    keys = [k for k in _METRIC_ORDER if k in metrics]
    keys += [k for k in metrics if k not in keys]
    width = max(len(k) for k in keys)
    lines = []
    for k in keys:
        v = metrics[k]
        sv = f"{v:.3f}" if k in _FLOAT_KEYS else str(v)
        lines.append(f"{k:<{width}} : {sv}")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gt-tracks", type=Path, default=None,
                    help="verified GT tracks JSON (run_trials format). "
                         "Required unless --from-static-cache + --pred-out only.")
    ap.add_argument("--pred", type=Path, default=None,
                    help="predictions JSON from run_dynamic --dump-mot "
                         "(score mode). Mutually exclusive with --from-static-cache.")
    ap.add_argument("--from-static-cache", type=Path, default=None, dest="static_cache",
                    help="SQLite tile cache to replay into MOT predictions "
                         "(cached dense dets -> ByteTracker). No chip needed.")
    ap.add_argument("--pred-out", type=Path, default=None,
                    help="(static-cache mode) write the replayed predictions JSON here")
    ap.add_argument("--src-wh", default=None,
                    help="(static-cache mode) source frame WxH (e.g. 3840x2160) "
                         "fallback when the cache lacks video_w/video_h meta")
    ap.add_argument("--nms-thr", type=float, default=0.5,
                    help="(static-cache mode) IoU threshold for det NMS-merge (default 0.5)")
    ap.add_argument("--fps", type=int, default=30,
                    help="(static-cache mode) tracker frame rate / track buffer (default 30)")
    ap.add_argument("--classes", default="1",
                    help="comma-separated class ids to score/track (default 1 = person)")
    ap.add_argument("--iou-thr", type=float, default=0.5,
                    help="IoU threshold for per-frame matching (default 0.5)")
    ap.add_argument("--out", type=Path, default=None,
                    help="write metrics + input paths + params as JSON")
    args = ap.parse_args()

    classes = [int(c) for c in args.classes.split(",") if c.strip()]

    if args.static_cache is not None:
        if args.pred is not None:
            ap.error("--from-static-cache and --pred are mutually exclusive")
        if args.gt_tracks is None and args.pred_out is None:
            ap.error("--from-static-cache needs --pred-out and/or --gt-tracks")
        src_wh = _parse_wh(args.src_wh) if args.src_wh else None
        by_track, mean_tiles, n_frames = replay_static_cache(
            args.static_cache, classes=classes, src_wh=src_wh,
            nms_thr=args.nms_thr, fps=args.fps)
        pred = {tid: dict(traj) for tid, traj in by_track.items()}
        print(f"static-cache : {args.static_cache}")
        print(f"classes      : {classes}   ppv : 0")
        print(f"frames       : {n_frames}   tracks : {len(pred)}")
        print(f"mean tiles/frame (distinct crops) : {mean_tiles:.2f}   "
              "[OPTIMISTIC: extra warmed tiles only help recall]")

        if args.pred_out is not None:
            args.pred_out.parent.mkdir(parents=True, exist_ok=True)
            args.pred_out.write_text(json.dumps(by_track_to_pred_doc(by_track)))
            print(f"pred-out     : {args.pred_out}")

        if args.gt_tracks is None:
            return  # dump-only

        gt = load_gt_as_mot(args.gt_tracks, set(classes))
        metrics = score_mot(gt, pred, iou_thr=args.iou_thr)
        print(f"gt-tracks    : {args.gt_tracks}")
        print(f"iou-thr      : {args.iou_thr}")
        print(format_metrics(metrics))
        if args.out is not None:
            doc = {
                "metrics": metrics,
                "gt_tracks": str(args.gt_tracks),
                "static_cache": str(args.static_cache),
                "params": {"classes": classes, "iou_thr": args.iou_thr,
                           "nms_thr": args.nms_thr, "fps": args.fps,
                           "src_wh": args.src_wh},
                "mean_tiles_per_frame": mean_tiles,
                "n_frames": n_frames,
            }
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(json.dumps(doc, indent=2))
            print(f"report       : {args.out}")
        return

    # --- score-existing-predictions mode ---
    if args.gt_tracks is None or args.pred is None:
        ap.error("score mode requires both --gt-tracks and --pred")

    gt = load_gt_as_mot(args.gt_tracks, set(classes))
    pred = load_pred(args.pred)

    metrics = score_mot(gt, pred, iou_thr=args.iou_thr)

    print(f"gt-tracks : {args.gt_tracks}")
    print(f"pred      : {args.pred}")
    print(f"classes   : {classes}   iou-thr : {args.iou_thr}")
    print(format_metrics(metrics))

    if args.out is not None:
        doc = {
            "metrics": metrics,
            "gt_tracks": str(args.gt_tracks),
            "pred": str(args.pred),
            "params": {"classes": classes, "iou_thr": args.iou_thr},
        }
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(doc, indent=2))
        print(f"report    : {args.out}")


if __name__ == "__main__":
    main()
