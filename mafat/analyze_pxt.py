"""Pixels-per-target analyzer for MAFAT tiling sweep.

Like analyze_bench.py (greedy IoU per-frame, all-points AP) but adds a
*size-binned recall* table: bin each GT object by its height in source
pixels, then per candidate config, count how many of those GT objects were
recovered (IoU >= --iou with a same-class prediction).

The output is the practical answer the user is asking for: "for a 30-px
person, do I need a 3x2 grid or a 6x4 grid?". Headline classes are
'person' and 'vehicle' (the two classes the hailo_yolov8n_4_classes_vga
HEF emits that the user cares about — 'car' is folded into 'vehicle').

Usage:
  python mafat/analyze_pxt.py \\
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
    args = ap.parse_args(argv)

    gt_doc = load_frames(args.gt)
    print(f"GT: {args.gt} ({len(gt_doc['frames'])} frames, "
          f"{sum(len(f['detections']) for f in gt_doc['frames'])} dets)")

    bins = DEFAULT_BINS_PX
    results: list[tuple[str, dict]] = []
    full_dump = {"gt": str(args.gt),
                 "video_w": args.video_w, "video_h": args.video_h,
                 "iou": args.iou,
                 "bins_px": [list(b) for b in bins],
                 "configs": {}}
    for p in args.pred:
        pred_doc = load_frames(p)
        label = pred_doc.get("label") or p.stem
        n_dets = sum(len(f["detections"]) for f in pred_doc["frames"])
        print(f"Pred: {p} (label={label!r}, {n_dets} dets)")
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
