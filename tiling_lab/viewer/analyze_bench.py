"""mAP-style analyzer for tiling-bench per-frame JSONs.

Treats one bench run as pseudo-ground-truth (typically the highest-recall
multi-scale config) and scores one or more other runs against it.

Usage:
  python -m tiling_lab.viewer.analyze_bench \
      --gt /tmp/tiling_bench_GT-multi-scale-3.frames.json \
      --pred /tmp/tiling_bench_1x1.frames.json \
      --pred /tmp/tiling_bench_3x2.frames.json \
      [--iou 0.5]

Outputs:
  - Per-config summary table (TP / FP / FN / P / R / F1 / mAP@0.5)
  - Per-class AP@0.5 table for each config

All matching is greedy by descending confidence with IoU >= --iou. AP is
computed via all-points integration of the precision-recall curve.

No sklearn / torchvision — small transparent code.
"""

import argparse
import json
from pathlib import Path
from typing import Iterable


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

def iou_xywh(a: list[float], b: list[float]) -> float:
    """IoU of two [x, y, w, h] boxes (any consistent units; coords need not be
    normalized so long as both boxes use the same convention).
    """
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ax2, ay2 = ax + aw, ay + ah
    bx2, by2 = bx + bw, by + bh

    ix1 = max(ax, bx)
    iy1 = max(ay, by)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0

    area_a = max(0.0, aw) * max(0.0, ah)
    area_b = max(0.0, bw) * max(0.0, bh)
    union = area_a + area_b - inter
    if union <= 0.0:
        return 0.0
    return inter / union


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def load_frames(path: Path) -> dict:
    with path.open() as f:
        return json.load(f)


def index_by_frame(frames_doc: dict) -> dict[int, list[dict]]:
    """Convert the `frames` list into {frame_idx: [detections...]} for O(1) join."""
    out: dict[int, list[dict]] = {}
    for fr in frames_doc["frames"]:
        out[int(fr["frame"])] = fr["detections"]
    return out


def det_class_key(det: dict) -> str:
    """Stable per-class key. Prefer label string; fall back to class_id."""
    label = det.get("label") or ""
    if label:
        return label
    cid = det.get("class_id")
    return f"cid={cid}" if cid is not None else "<unknown>"


# ---------------------------------------------------------------------------
# Per-frame, per-class greedy matching
# ---------------------------------------------------------------------------

def match_frame(
    gt_dets: list[dict],
    pred_dets: list[dict],
    iou_thresh: float,
) -> tuple[list[tuple[float, bool]], int]:
    """Return (per-pred (confidence, is_tp) list, num_unmatched_gt).

    Greedy match: sort preds by confidence desc; for each, pick the highest-IoU
    unmatched GT of the same class above iou_thresh. Predictions that don't
    match are FPs; GTs that never get matched count toward FNs.
    """
    # Sort preds desc by confidence; stable so ties keep input order.
    pred_sorted = sorted(
        enumerate(pred_dets),
        key=lambda kv: -float(kv[1].get("confidence", 0.0)),
    )

    used_gt: set[int] = set()
    pr_results: list[tuple[float, bool]] = []

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
            pr_results.append((float(p["confidence"]), True))
        else:
            pr_results.append((float(p["confidence"]), False))

    fn = len(gt_dets) - len(used_gt)
    return pr_results, fn


# ---------------------------------------------------------------------------
# AP via all-points PR-curve integration
# ---------------------------------------------------------------------------

def compute_ap(
    matches: list[tuple[float, bool]],
    n_gt: int,
) -> float:
    """All-points AP@<implicit IoU>: integrate over the PR curve.

    matches: list of (confidence, is_tp) for ALL predictions across ALL frames
             for one class.
    n_gt:    total number of GTs of that class across ALL frames.
    """
    if n_gt == 0:
        return 0.0
    if not matches:
        return 0.0

    # Sort all preds globally by confidence desc.
    matches.sort(key=lambda x: -x[0])

    tp_cum = 0
    fp_cum = 0
    precisions: list[float] = []
    recalls: list[float] = []
    for _, is_tp in matches:
        if is_tp:
            tp_cum += 1
        else:
            fp_cum += 1
        precisions.append(tp_cum / (tp_cum + fp_cum))
        recalls.append(tp_cum / n_gt)

    # All-points (PASCAL VOC 2010+): make precision monotonically non-increasing
    # going right-to-left, then sum precision * recall delta.
    # Prepend (recall=0, prec=1) to handle the leftmost segment.
    rec = [0.0] + recalls + [recalls[-1]]
    pre = [1.0] + precisions + [0.0]
    for i in range(len(pre) - 2, -1, -1):
        if pre[i] < pre[i + 1]:
            pre[i] = pre[i + 1]
    ap = 0.0
    for i in range(1, len(rec)):
        ap += (rec[i] - rec[i - 1]) * pre[i]
    return ap


# ---------------------------------------------------------------------------
# Whole-config evaluation
# ---------------------------------------------------------------------------

def evaluate_against_gt(
    gt_doc: dict,
    pred_doc: dict,
    iou_thresh: float,
) -> dict:
    """Compute aggregate TP/FP/FN + per-class AP for one (gt, pred) pair."""
    gt_idx = index_by_frame(gt_doc)
    pred_idx = index_by_frame(pred_doc)

    # All frames in either side. (In practice they should match 1:1 — both
    # runs decoded the same video — but be defensive.)
    all_frames = sorted(set(gt_idx.keys()) | set(pred_idx.keys()))

    # Aggregate counters at conf=0 (accept-all).
    tp = 0
    fp = 0
    fn = 0

    # Per-class aggregation for AP.
    per_class_matches: dict[str, list[tuple[float, bool]]] = {}
    per_class_n_gt: dict[str, int] = {}

    for fr in all_frames:
        gts = gt_idx.get(fr, [])
        preds = pred_idx.get(fr, [])

        # Update per-class GT counts.
        for g in gts:
            k = det_class_key(g)
            per_class_n_gt[k] = per_class_n_gt.get(k, 0) + 1

        # Bucket detections by class so per-class matching is independent.
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
            pr, frame_fn = match_frame(cls_gts, cls_preds, iou_thresh)
            for conf, is_tp in pr:
                per_class_matches.setdefault(cls, []).append((conf, is_tp))
                if is_tp:
                    tp += 1
                else:
                    fp += 1
            fn += frame_fn

    # Per-class AP.
    per_class_ap: dict[str, float] = {}
    for cls, n_gt in per_class_n_gt.items():
        if n_gt == 0:
            continue
        per_class_ap[cls] = compute_ap(per_class_matches.get(cls, []), n_gt)

    # mAP: mean over classes that have at least one GT.
    if per_class_ap:
        mAP = sum(per_class_ap.values()) / len(per_class_ap)
    else:
        mAP = 0.0

    # P / R / F1 at conf=0 (i.e. accept all preds).
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0

    return {
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "mAP": mAP,
        "per_class_ap": per_class_ap,
        "per_class_n_gt": per_class_n_gt,
    }


# ---------------------------------------------------------------------------
# Pretty printing
# ---------------------------------------------------------------------------

def print_summary_table(results: list[tuple[str, dict]]) -> None:
    print()
    print("=" * 92)
    print(f"mAP-style comparison vs GT (IoU threshold per row)")
    print("=" * 92)
    print(f"{'config':<22} | {'TP':>6} | {'FP':>6} | {'FN':>6} | "
          f"{'P':>6} {'R':>6} {'F1':>6} | {'mAP@.5':>8}")
    print("-" * 92)
    for label, r in results:
        print(f"{label:<22} | {r['tp']:>6} | {r['fp']:>6} | {r['fn']:>6} | "
              f"{r['precision']:>6.3f} {r['recall']:>6.3f} {r['f1']:>6.3f} | "
              f"{r['mAP']:>8.4f}")
    print("=" * 92)


def print_per_class_table(results: list[tuple[str, dict]]) -> None:
    # Union of class names across all configs, sorted.
    all_classes: set[str] = set()
    for _, r in results:
        all_classes |= set(r["per_class_ap"].keys())
        all_classes |= set(r["per_class_n_gt"].keys())
    if not all_classes:
        return
    classes_sorted = sorted(all_classes)

    # n_gt: take the max across configs (they should agree, but be safe).
    n_gt_for_class: dict[str, int] = {}
    for cls in classes_sorted:
        ng = 0
        for _, r in results:
            ng = max(ng, r["per_class_n_gt"].get(cls, 0))
        n_gt_for_class[cls] = ng

    labels = [lbl for lbl, _ in results]
    print()
    print("=" * (24 + 12 + 12 * len(labels)))
    print(f"Per-class AP@0.5")
    header = f"{'class':<24} {'n_gt':>10}"
    for lbl in labels:
        header += f" {lbl[:11]:>12}"
    print(header)
    print("-" * (24 + 12 + 12 * len(labels)))
    for cls in classes_sorted:
        row = f"{cls[:24]:<24} {n_gt_for_class[cls]:>10}"
        for _, r in results:
            ap = r["per_class_ap"].get(cls)
            if ap is None:
                row += f" {'-':>12}"
            else:
                row += f" {ap:>12.4f}"
        print(row)
    print("=" * (24 + 12 + 12 * len(labels)))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv: Iterable[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gt", type=Path, required=True,
                    help="Path to <bench>.frames.json for the pseudo-ground-truth run")
    ap.add_argument("--pred", type=Path, action="append", required=True,
                    help="Path to a <bench>.frames.json for a candidate config "
                         "(repeat for multiple)")
    ap.add_argument("--iou", type=float, default=0.5,
                    help="IoU threshold for matching (default: 0.5)")
    args = ap.parse_args(argv)

    if not args.gt.is_file():
        ap.error(f"GT file not found: {args.gt}")
    for p in args.pred:
        if not p.is_file():
            ap.error(f"pred file not found: {p}")

    gt_doc = load_frames(args.gt)
    gt_label = gt_doc.get("label") or args.gt.stem
    print(f"GT:    {args.gt}  (label={gt_label!r}, "
          f"{len(gt_doc['frames'])} frames, "
          f"{sum(len(f['detections']) for f in gt_doc['frames'])} dets)")

    results: list[tuple[str, dict]] = []
    for p in args.pred:
        pred_doc = load_frames(p)
        pred_label = pred_doc.get("label") or p.stem
        n_dets = sum(len(f["detections"]) for f in pred_doc["frames"])
        print(f"Pred:  {p}  (label={pred_label!r}, "
              f"{len(pred_doc['frames'])} frames, {n_dets} dets)")
        r = evaluate_against_gt(gt_doc, pred_doc, args.iou)
        results.append((pred_label, r))

    print_summary_table(results)
    print_per_class_table(results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
