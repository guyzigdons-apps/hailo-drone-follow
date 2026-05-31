"""Analyze resize_quality_probe.py output: stretch vs letterbox vs reference.

Computes per-config:
  - total detections, mean dets/frame
  - mean / median confidence
  - per-class counts
  - recall + precision of the dense-tile reference (IoU-matched, greedy)

The dense-tile run (dense_ref) is the pseudo-GT. We also report against the
whole-frame run for context (expected weak: small targets vanish on downscale).
"""
from __future__ import annotations

import argparse
import json
import statistics as st
from pathlib import Path


def _iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = a["x"], a["y"], a["x"] + a["w"], a["y"] + a["h"]
    bx1, by1, bx2, by2 = b["x"], b["y"], b["x"] + b["w"], b["y"] + b["h"]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = a["w"] * a["h"] + b["w"] * b["h"] - inter
    return inter / ua if ua > 0 else 0.0


def load(path: Path) -> list[dict]:
    return json.loads(path.read_text())


def summarize(records: list[dict]) -> dict:
    all_dets = [d for r in records for d in r["dets"]]
    scores = [d["score"] for d in all_dets]
    per_class: dict[int, int] = {}
    for d in all_dets:
        per_class[d["cls"]] = per_class.get(d["cls"], 0) + 1
    n_frames = len(records)
    return {
        "frames": n_frames,
        "total_dets": len(all_dets),
        "dets_per_frame": round(len(all_dets) / n_frames, 3) if n_frames else 0,
        "mean_conf": round(st.mean(scores), 4) if scores else None,
        "median_conf": round(st.median(scores), 4) if scores else None,
        "per_class": {str(k): per_class[k] for k in sorted(per_class)},
    }


def match_recall(ref: list[dict], cand: list[dict], iou_thr: float) -> dict:
    """Greedy per-frame IoU match. Recall = fraction of ref dets matched by a
    cand det (same class, IoU>=thr). Precision = fraction of cand dets that
    match some ref det."""
    ref_by_f = {r["frame"]: r["dets"] for r in ref}
    cand_by_f = {r["frame"]: r["dets"] for r in cand}
    frames = sorted(set(ref_by_f) & set(cand_by_f))
    tp = fn = fp = 0
    for f in frames:
        rds = list(ref_by_f[f])
        cds = list(cand_by_f[f])
        used = [False] * len(cds)
        for rd in rds:
            best_i, best_iou = -1, iou_thr
            for i, cd in enumerate(cds):
                if used[i] or cd["cls"] != rd["cls"]:
                    continue
                iou = _iou(rd, cd)
                if iou >= best_iou:
                    best_i, best_iou = i, iou
            if best_i >= 0:
                used[best_i] = True
                tp += 1
            else:
                fn += 1
        fp += used.count(False)
    n_ref = tp + fn
    n_cand = tp + fp
    return {
        "ref_dets": n_ref, "cand_dets": n_cand,
        "tp": tp, "fn": fn, "fp": fp,
        "recall": round(tp / n_ref, 4) if n_ref else None,
        "precision": round(tp / n_cand, 4) if n_cand else None,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-dir", required=True, type=Path)
    ap.add_argument("--tags", nargs="+", required=True)
    ap.add_argument("--iou-thr", type=float, default=0.3)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    report: dict = {"iou_thr": args.iou_thr, "by_tag": {}}
    for tag in args.tags:
        runs = {}
        for name in ("whole", "dense_ref", "stretch", "letterbox"):
            p = args.in_dir / f"{tag}__{name}.json"
            if p.exists():
                runs[name] = load(p)
        summaries = {n: summarize(r) for n, r in runs.items()}
        recalls = {}
        if "dense_ref" in runs:
            for cand in ("stretch", "letterbox", "whole"):
                if cand in runs:
                    recalls[f"{cand}_vs_dense"] = match_recall(
                        runs["dense_ref"], runs[cand], args.iou_thr)
        report["by_tag"][tag] = {"summary": summaries, "recall": recalls}

    txt = json.dumps(report, indent=2)
    print(txt)
    if args.out:
        args.out.write_text(txt)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
