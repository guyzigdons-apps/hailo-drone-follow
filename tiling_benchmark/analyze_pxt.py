"""Pixels-per-target analyzer for the tiling sweep.

Like analyze_bench.py (greedy IoU per-frame, all-points AP) but adds a
*size-binned recall* table: bin each GT object by its height in source
pixels, then per candidate config, count how many of those GT objects were
recovered (IoU >= --iou with a same-class prediction).

The output is the practical answer the user is asking for: "for a 30-px
person, do I need a 3x2 grid or a 6x4 grid?". Headline classes are
'person' and 'vehicle' (the two classes the hailo_yolov8n_4_classes_vga
HEF emits that the user cares about — 'car' is folded into 'vehicle').

Usage:
  python bench/analyze_pxt.py \\
      --gt /home/giladn/Videos/Drone/Training/pxt_runs/pxt_GT-12x9-25.frames.json \\
      --pred /home/giladn/Videos/Drone/Training/pxt_runs/pxt_1x1-native.frames.json \\
      --pred /home/giladn/Videos/Drone/Training/pxt_runs/pxt_3x2-native.frames.json \\
      --video-w 6016 --video-h 3384 \\
      --out /home/giladn/Videos/Drone/Training/pxt_runs/pxt_analysis.json
"""

import argparse
import json
from pathlib import Path

# Reuse the iou + label-key helpers from analyze_bench.py — same package dir,
# no __init__.py, so add the parent to sys.path.
import sys
HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
from analyze_bench import iou_xywh, det_class_key, index_by_frame, load_frames  # noqa: E402


# NOTE: duplicated from tiling_record._grid_to_static_tiles to avoid pulling
# the GStreamer import (tiling_record imports gst). Keep the math identical.
def _grid_to_static_tiles(tiles_x: int, tiles_y: int,
                          overlap_x: float, overlap_y: float) -> list[str]:
    """Convert a regular tiles_x*tiles_y grid into a list of normalized
    'x,y,w,h' rectangles. Math: T = 1/(N - (N-1)*o); step = T*(1-o).
    For N==1 returns a single full-axis tile."""
    if tiles_x < 1 or tiles_y < 1:
        return []
    def axis(n: int, o: float):
        if n == 1:
            return [(0.0, 1.0)]
        T = 1.0 / (n - (n - 1) * o)
        S = T * (1.0 - o)
        return [(i * S, T) for i in range(n)]
    rects = []
    for (y, h) in axis(tiles_y, overlap_y):
        for (x, w) in axis(tiles_x, overlap_x):
            rects.append(f"{x:.6f},{y:.6f},{w:.6f},{h:.6f}")
    return rects


def _center_tile_rect(size: float) -> tuple[float, float, float, float]:
    """Centered square tile of side `size` (fraction of frame)."""
    x = (1.0 - size) / 2.0
    return (x, x, size, size)


def _compute_tile_rects(config: dict) -> list[tuple[float, float, float, float]]:
    """Reconstruct the originating run's tile rectangles in normalized coords.

    Mirrors the driver: main grid + optional include_full_frame + optional
    include_center_tile + optional extra_grids list.
    """
    rects: list[tuple[float, float, float, float]] = []
    tiles_x = int(config.get("tiles_x", 0) or 0)
    tiles_y = int(config.get("tiles_y", 0) or 0)
    overlap_x = float(config.get("overlap_x", 0.0) or 0.0)
    overlap_y = float(config.get("overlap_y", 0.0) or 0.0)
    for s in _grid_to_static_tiles(tiles_x, tiles_y, overlap_x, overlap_y):
        parts = s.split(",")
        rects.append((float(parts[0]), float(parts[1]),
                      float(parts[2]), float(parts[3])))
    # The driver only prepends a full-frame extra when the main grid is
    # non-trivial (not 1x1), but for filtering purposes the full-frame rect
    # (0,0,1,1) is equivalent to the 1x1 main grid we already added above,
    # so we always cover (0,0,1,1) when include_full_frame is set.
    if config.get("include_full_frame"):
        rects.append((0.0, 0.0, 1.0, 1.0))
    if config.get("include_center_tile"):
        size = float(config.get("center_tile_size", 0.4) or 0.4)
        rects.append(_center_tile_rect(size))
    for entry in (config.get("extra_grids") or []):
        # entry is tuple-like / list-like (tx, ty, ox, oy)
        tx, ty, ox, oy = entry[0], entry[1], entry[2], entry[3]
        for s in _grid_to_static_tiles(int(tx), int(ty), float(ox), float(oy)):
            parts = s.split(",")
            rects.append((float(parts[0]), float(parts[1]),
                          float(parts[2]), float(parts[3])))
    return rects


def is_phantom(det: dict, tile_rects: list[tuple[float, float, float, float]],
               tol: float = 0.01) -> bool:
    """Return True if det is a class-0 phantom against the given tile grid.

    A detection is treated as a phantom iff BOTH of these hold:
      1. The detection is class-0 (person). We check ``label == 'person'``
         (case-insensitive) OR ``class_id == 1`` for HEFs where the label
         string is missing. ``vehicle``/``face``/``license_plate`` are
         NEVER filtered — the artefact is class-0-specific.
      2. The bbox matches a tile rectangle in one of two ways:
         (a) Exact tile shape — ``(x, y, w, h)`` agrees with some tile rect
             within ``tol`` (normalized).
         (b) Large-person fallback — the bbox area is >=75% of a tile's
             area AND its centre lies within ``0.5 * tile_w`` (and tile_h)
             of that tile's centre. Catches "almost-tile" variants like
             ``bbox=(0.006, 0.007, 0.989, 0.991)`` that don't satisfy (a)
             but are clearly the same artefact.

    ``tile_rects`` is the list of normalized (x, y, w, h) rectangles from
    the originating run's tiling config; reconstruct it with
    :func:`_compute_tile_rects`.
    """
    if not tile_rects:
        return False
    label = (det.get("label") or "").lower()
    if label != "person" and det.get("class_id") != 1:
        return False
    bbox = det.get("bbox") or []
    if len(bbox) < 4:
        return False
    bx, by, bw, bh = (float(bbox[0]), float(bbox[1]),
                      float(bbox[2]), float(bbox[3]))
    det_area = bw * bh
    det_cx, det_cy = bx + bw / 2.0, by + bh / 2.0
    for (tx, ty, tw, th) in tile_rects:
        # (a) exact-tile match
        if (abs(bx - tx) < tol and abs(by - ty) < tol
                and abs(bw - tw) < tol and abs(bh - th) < tol):
            return True
        # (b) large-person fallback
        tile_area = tw * th
        if tile_area > 0 and det_area >= 0.75 * tile_area:
            t_cx, t_cy = tx + tw / 2.0, ty + th / 2.0
            if (abs(det_cx - t_cx) < 0.5 * tw
                    and abs(det_cy - t_cy) < 0.5 * th):
                return True
    return False


# Back-compat alias — the previous filter is now subsumed by is_phantom.
def _is_tile_shaped(bbox: list,
                    tile_rects: list[tuple[float, float, float, float]],
                    tol: float) -> bool:
    return is_phantom({"label": "person", "bbox": bbox}, tile_rects, tol)


def filter_tile_shaped(frames_doc: dict, tol: float) -> tuple[int, int]:
    """Mutate frames_doc in place, removing class-0 phantom detections
    (per :func:`is_phantom`) using the doc's own config's tile grid.
    Returns ``(dropped, original_total)``."""
    cfg = frames_doc.get("config", {}) or {}
    tile_rects = _compute_tile_rects(cfg)
    if not tile_rects:
        return 0, sum(len(fr["detections"]) for fr in frames_doc["frames"])
    dropped = 0
    total = 0
    for fr in frames_doc["frames"]:
        keep = []
        for det in fr["detections"]:
            total += 1
            if is_phantom(det, tile_rects, tol):
                dropped += 1
                continue
            keep.append(det)
        fr["detections"] = keep
    return dropped, total


def filter_classes(frames_doc: dict,
                   keep_labels: list[str] | None,
                   keep_class_ids: list[int] | None) -> tuple[int, int]:
    """Mutate frames_doc in place, dropping detections whose label is not
    in ``keep_labels`` AND whose class_id is not in ``keep_class_ids``.

    Either or both filter inputs may be None/empty. If both are empty,
    nothing is filtered. Returns ``(dropped, original_total)``.
    """
    keep_label_set: set[str] | None = (
        {s.lower() for s in keep_labels} if keep_labels else None
    )
    keep_cid_set: set[int] | None = (
        {int(c) for c in keep_class_ids} if keep_class_ids else None
    )
    if keep_label_set is None and keep_cid_set is None:
        return 0, sum(len(fr["detections"]) for fr in frames_doc["frames"])
    dropped = 0
    total = 0
    for fr in frames_doc["frames"]:
        keep = []
        for det in fr["detections"]:
            total += 1
            label = (det.get("label") or "").lower()
            cid = det.get("class_id")
            label_ok = keep_label_set is not None and label in keep_label_set
            cid_ok = (keep_cid_set is not None and cid is not None
                      and int(cid) in keep_cid_set)
            if label_ok or cid_ok:
                keep.append(det)
            else:
                dropped += 1
        fr["detections"] = keep
    return dropped, total

# Object-height bins in *source pixels*. Each (lo, hi] in px height. The bins
# straddle the small/medium/large boundaries that matter for tiling decisions.
DEFAULT_BINS_PX = [(0, 16), (16, 32), (32, 64), (64, 128), (128, 256), (256, 4096)]
# Classes the user explicitly cares about. Everything else is still scored,
# just not headlined. (hailo_yolov8n_4_classes_vga emits person / vehicle /
# face / license_plate — face & license_plate are too small for this dataset
# but kept in the JSON for completeness.)
HEADLINE_CLASSES = ("person", "vehicle")


def match_frame_size_aware(
    gt_dets: list[dict],
    pred_dets: list[dict],
    iou_thresh: float,
    video_h_px: int,
):
    """Per-frame greedy matching identical to analyze_bench.match_frame, BUT
    we also return, for each GT, (matched_bool, height_px) so the caller can
    bin GTs by size before aggregating recall.
    """
    pred_sorted = sorted(
        enumerate(pred_dets),
        key=lambda kv: -float(kv[1].get("confidence", 0.0)),
    )
    used_gt: set[int] = set()
    # also remember which pred is a TP — irrelevant for size-binned recall but
    # useful for keeping the AP path identical.
    for _, p in pred_sorted:
        p_class = det_class_key(p)
        best_iou = 0.0
        best_gi = -1
        for gi, g in enumerate(gt_dets):
            if gi in used_gt:
                continue
            if det_class_key(g) != p_class:
                continue
            i = iou_xywh(p["bbox"], g["bbox"])
            if i > best_iou:
                best_iou = i
                best_gi = gi
        if best_gi >= 0 and best_iou >= iou_thresh:
            used_gt.add(best_gi)

    gt_results = []
    for gi, g in enumerate(gt_dets):
        # bbox is [x, y, w, h] normalized 0-1
        h_norm = float(g["bbox"][3])
        h_px = h_norm * video_h_px
        gt_results.append((det_class_key(g), h_px, gi in used_gt))
    return gt_results


def bin_for_height(h_px: float, bins) -> int:
    for i, (lo, hi) in enumerate(bins):
        if lo < h_px <= hi:
            return i
    return len(bins) - 1


def analyze_one(gt_doc: dict, pred_doc: dict, iou_thresh: float,
                video_h_px: int, bins) -> dict:
    gt_idx = index_by_frame(gt_doc)
    pred_idx = index_by_frame(pred_doc)
    all_frames = sorted(set(gt_idx.keys()) | set(pred_idx.keys()))

    # bin_recall[cls][bin_idx] = (matched, total)
    bin_recall: dict[str, list[list[int]]] = {}
    overall_totals: dict[str, list[int]] = {}  # (matched, total) — all sizes
    for fr in all_frames:
        gts = gt_idx.get(fr, [])
        preds = pred_idx.get(fr, [])
        # Bucket by class so matching is per-class (mirrors analyze_bench).
        gts_by_class: dict[str, list[dict]] = {}
        preds_by_class: dict[str, list[dict]] = {}
        for g in gts:
            gts_by_class.setdefault(det_class_key(g), []).append(g)
        for p in preds:
            preds_by_class.setdefault(det_class_key(p), []).append(p)
        seen_classes = set(gts_by_class.keys()) | set(preds_by_class.keys())
        for cls in seen_classes:
            cls_gts = gts_by_class.get(cls, [])
            cls_preds = preds_by_class.get(cls, [])
            results = match_frame_size_aware(cls_gts, cls_preds, iou_thresh, video_h_px)
            if cls not in bin_recall:
                bin_recall[cls] = [[0, 0] for _ in bins]
                overall_totals[cls] = [0, 0]
            for cls_key, h_px, matched in results:
                # cls_key == cls by construction here; left for clarity.
                bi = bin_for_height(h_px, bins)
                bin_recall[cls][bi][1] += 1
                overall_totals[cls][1] += 1
                if matched:
                    bin_recall[cls][bi][0] += 1
                    overall_totals[cls][0] += 1

    return {
        "bins_px": [list(b) for b in bins],
        "per_class_bin": {
            cls: [
                {
                    "bin_lo": bins[i][0],
                    "bin_hi": bins[i][1],
                    "matched": bin_recall[cls][i][0],
                    "n_gt": bin_recall[cls][i][1],
                    "recall": (bin_recall[cls][i][0] / bin_recall[cls][i][1])
                              if bin_recall[cls][i][1] else None,
                }
                for i in range(len(bins))
            ]
            for cls in bin_recall
        },
        "per_class_overall": {
            cls: {
                "matched": overall_totals[cls][0],
                "n_gt": overall_totals[cls][1],
                "recall": (overall_totals[cls][0] / overall_totals[cls][1])
                          if overall_totals[cls][1] else None,
            }
            for cls in overall_totals
        },
    }


def print_size_recall_table(results: list[tuple[str, dict]], bins) -> None:
    """One table per headline class. Rows = bins; columns = configs."""
    all_classes = set()
    for _, r in results:
        all_classes.update(r["per_class_bin"].keys())
    classes_sorted = sorted(all_classes, key=lambda c:
                            (0, HEADLINE_CLASSES.index(c)) if c in HEADLINE_CLASSES
                            else (1, c))
    for cls in classes_sorted:
        # n_gt is the same per-bin across configs (same GT input).
        n_gt_per_bin = None
        for _, r in results:
            pc = r["per_class_bin"].get(cls)
            if pc:
                n_gt_per_bin = [b["n_gt"] for b in pc]
                break
        if not n_gt_per_bin:
            continue
        print()
        print("=" * (32 + 11 * len(results)))
        print(f"Size-binned recall — class: {cls}")
        header = f"{'bin_px':<14} {'n_gt':>8}"
        for lbl, _ in results:
            header += f" {lbl[:9]:>10}"
        print(header)
        print("-" * (32 + 11 * len(results)))
        for i, (lo, hi) in enumerate(bins):
            row = f"({lo:>4},{hi:>4}]    {n_gt_per_bin[i]:>8}"
            for _, r in results:
                pc = r["per_class_bin"].get(cls)
                if not pc or pc[i]["recall"] is None:
                    row += f" {'-':>10}"
                else:
                    row += f" {pc[i]['recall']*100:>9.1f}%"
            print(row)
        # overall row
        total_gt = sum(n_gt_per_bin)
        row = f"{'TOTAL':<14} {total_gt:>8}"
        for _, r in results:
            ov = r["per_class_overall"].get(cls)
            if not ov or ov["recall"] is None:
                row += f" {'-':>10}"
            else:
                row += f" {ov['recall']*100:>9.1f}%"
        print(row)
        print("=" * (32 + 11 * len(results)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gt", type=Path, required=True)
    ap.add_argument("--pred", type=Path, action="append", required=True)
    ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--video-w", type=int, required=True,
                    help="GT source width in pixels (for binning).")
    ap.add_argument("--video-h", type=int, required=True,
                    help="GT source height in pixels (for binning).")
    ap.add_argument("--out", type=Path, default=None,
                    help="Optional path to dump the full analysis as JSON.")
    ap.add_argument("--drop-tile-shaped", action="store_true",
                    help="Drop class-0 'person' phantom detections caused by the "
                         "yolov8n_4_classes_vga HEF on low-contrast tiles. Uses the "
                         "combined filter: (a) exact tile-rect match OR (b) area >= "
                         "75%% of a tile centered within 0.5*tile_w of the tile centre. "
                         "vehicle/face/license_plate are never filtered.")
    ap.add_argument("--tile-shape-tol", type=float, default=0.01,
                    help="Normalized tolerance for the exact-tile-shape arm of the "
                         "phantom filter (default 0.01).")
    ap.add_argument("--filter-classes", nargs="+", default=None,
                    metavar="LABEL",
                    help="Keep only detections whose label is in this list (case-"
                         "insensitive). Use to restrict to e.g. 'person vehicle' "
                         "for downstream yolov8m runs. Default: keep all classes.")
    ap.add_argument("--keep-classes-from-class-ids", nargs="+", type=int,
                    default=None, metavar="CID",
                    help="Same as --filter-classes but matches on integer class_id "
                         "(useful for HEFs without label strings). May be combined "
                         "with --filter-classes; a detection is kept if it matches "
                         "either.")
    args = ap.parse_args(argv)

    def _apply_filters(doc: dict, file_label: str) -> None:
        if args.drop_tile_shaped:
            dropped, total = filter_tile_shaped(doc, args.tile_shape_tol)
            pct = (100.0 * dropped / total) if total else 0.0
            print(f"[phantom] {file_label}: dropped {dropped} phantoms "
                  f"({pct:.1f}%)")
        if args.filter_classes or args.keep_classes_from_class_ids:
            dropped, total = filter_classes(
                doc,
                args.filter_classes,
                args.keep_classes_from_class_ids,
            )
            pct = (100.0 * dropped / total) if total else 0.0
            keep_desc_parts: list[str] = []
            if args.filter_classes:
                keep_desc_parts.append(",".join(args.filter_classes))
            if args.keep_classes_from_class_ids:
                keep_desc_parts.append(
                    "cid={" + ",".join(str(c) for c in
                                       args.keep_classes_from_class_ids) + "}"
                )
            keep_desc = " | ".join(keep_desc_parts) or "<none>"
            print(f"[class] {file_label}: dropped {dropped} not-in[{keep_desc}] "
                  f"({pct:.1f}%)")

    gt_doc = load_frames(args.gt)
    gt_total_before = sum(len(f["detections"]) for f in gt_doc["frames"])
    print(f"GT: {args.gt} ({len(gt_doc['frames'])} frames, {gt_total_before} dets)")
    _apply_filters(gt_doc, args.gt.name)

    bins = DEFAULT_BINS_PX
    results: list[tuple[str, dict]] = []
    full_dump = {"gt": str(args.gt),
                 "video_w": args.video_w, "video_h": args.video_h,
                 "iou": args.iou,
                 "bins_px": [list(b) for b in bins],
                 "drop_tile_shaped": bool(args.drop_tile_shaped),
                 "tile_shape_tol": float(args.tile_shape_tol),
                 "filter_classes": list(args.filter_classes) if args.filter_classes else None,
                 "keep_classes_from_class_ids": list(args.keep_classes_from_class_ids)
                                                 if args.keep_classes_from_class_ids else None,
                 "configs": {}}
    for p in args.pred:
        pred_doc = load_frames(p)
        label = pred_doc.get("label") or p.stem
        n_dets = sum(len(f["detections"]) for f in pred_doc["frames"])
        print(f"Pred: {p} (label={label!r}, {n_dets} dets)")
        _apply_filters(pred_doc, p.name)
        r = analyze_one(gt_doc, pred_doc, args.iou, args.video_h, bins)
        results.append((label, r))
        full_dump["configs"][label] = r

    print_size_recall_table(results, bins)

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("w") as f:
            json.dump(full_dump, f, indent=2)
        print(f"\nFull analysis dumped to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
