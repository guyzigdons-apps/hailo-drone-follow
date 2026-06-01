"""Ablation metrics — IoU matching + recall/precision vs a reference (Plan 6 B3).

Small self-contained per-frame greedy IoU matcher over normalized full-frame
``Det``s. The reference config (the dense 12x9 GT row) supplies the
ground-truth boxes; each candidate config's detections are matched against it
to yield recall (fraction of reference boxes matched) and precision (fraction
of candidate boxes that matched a reference box), IoU-thresholded at 0.5 and
matched per class.
"""
from __future__ import annotations

from typing import Collection, Sequence

from ..types import Det


def iou(a: Det, b: Det) -> float:
    """IoU of two normalized (x, y, w, h) boxes."""
    ax2, ay2 = a.x + a.w, a.y + a.h
    bx2, by2 = b.x + b.w, b.y + b.h
    ix1, iy1 = max(a.x, b.x), max(a.y, b.y)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    ua = a.w * a.h + b.w * b.h - inter
    return inter / ua if ua > 0 else 0.0


def match_frame(
    preds: Sequence[Det],
    refs: Sequence[Det],
    iou_thr: float = 0.5,
) -> tuple[int, int, int]:
    """Greedy per-class IoU matching for one frame.

    Returns ``(tp, n_pred, n_ref)`` where ``tp`` is the number of reference
    boxes matched (each ref / pred used at most once, same class, IoU >= thr).
    Matching is greedy by descending IoU — adequate for recall/precision
    reporting against a dense reference.
    """
    n_pred = len(preds)
    n_ref = len(refs)
    if n_pred == 0 or n_ref == 0:
        return 0, n_pred, n_ref

    pairs: list[tuple[float, int, int]] = []
    for pi, p in enumerate(preds):
        for ri, r in enumerate(refs):
            if p.cls != r.cls:
                continue
            v = iou(p, r)
            if v >= iou_thr:
                pairs.append((v, pi, ri))
    pairs.sort(reverse=True)

    used_pred: set[int] = set()
    used_ref: set[int] = set()
    tp = 0
    for _v, pi, ri in pairs:
        if pi in used_pred or ri in used_ref:
            continue
        used_pred.add(pi)
        used_ref.add(ri)
        tp += 1
    return tp, n_pred, n_ref


def _filter_classes(
    dets: Sequence[Det], keep_classes: Collection[int] | None
) -> Sequence[Det]:
    """Keep only ``dets`` whose ``cls`` is in ``keep_classes`` (all if None)."""
    if keep_classes is None:
        return dets
    keep = set(keep_classes)
    return [d for d in dets if d.cls in keep]


def recall_precision_vs_reference(
    pred_frames: dict[int, Sequence[Det]],
    ref_frames: dict[int, Sequence[Det]],
    iou_thr: float = 0.5,
    keep_classes: Collection[int] | None = None,
) -> tuple[float, float, int]:
    """Aggregate recall/precision of ``pred_frames`` against ``ref_frames``.

    Both are ``{frame_idx: [Det, ...]}``. Returns
    ``(recall, precision, n_ref_total)``:

      recall    = sum(tp) / sum(n_ref)    over all frames
      precision = sum(tp) / sum(n_pred)   over all frames

    ``keep_classes`` (e.g. ``classes.TRACKED_CLASSES``) restricts BOTH
    predictions and reference to those class ids before matching, so recall is
    reported only over the classes we care about (persons/vehicles, not
    faces/plates). ``None`` keeps every class.

    A frame present in one dict but not the other contributes its boxes as
    pure false-negatives / false-positives. Recall/precision are 0.0 when the
    respective denominator is 0 (no reference boxes / no predictions).
    """
    tp_total = 0
    n_pred_total = 0
    n_ref_total = 0
    for fi in set(pred_frames) | set(ref_frames):
        preds = _filter_classes(pred_frames.get(fi, []), keep_classes)
        refs = _filter_classes(ref_frames.get(fi, []), keep_classes)
        tp, n_pred, n_ref = match_frame(preds, refs, iou_thr=iou_thr)
        tp_total += tp
        n_pred_total += n_pred
        n_ref_total += n_ref
    recall = tp_total / n_ref_total if n_ref_total else 0.0
    precision = tp_total / n_pred_total if n_pred_total else 0.0
    return recall, precision, n_ref_total


def matched_compute_delta(
    dynamic_mean_tiles: float,
    dynamic_recall: float,
    static_rows: Sequence[dict],
) -> tuple[str | None, float | None]:
    """Pair a dynamic row with the static grid of the closest mean tiles/frame
    and return ``(static_name, recall_delta)`` where
    ``recall_delta = dynamic_recall - static_recall`` at matched compute.

    ``static_rows`` is a list of ``{"name", "mean_tiles", "recall"}`` dicts (the
    static + reference rows). The match is the static row minimising
    ``|mean_tiles - dynamic_mean_tiles|``. Returns ``(None, None)`` if there are
    no static rows. The delta is the paper's headline: recall gained/lost vs a
    uniform grid of equal budget.
    """
    candidates = [r for r in static_rows if r.get("recall") is not None]
    if not candidates:
        return None, None
    best = min(candidates, key=lambda r: abs(r["mean_tiles"] - dynamic_mean_tiles))
    return best["name"], dynamic_recall - best["recall"]
