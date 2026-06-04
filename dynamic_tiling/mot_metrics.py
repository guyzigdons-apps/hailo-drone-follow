"""Minimal in-repo MOT metrics: MOTA (FP/FN/IDsw), IDF1, Frag, MT/ML.

Inputs are {gt_id: {frame: (x,y,w,h)}} and {pred_id: {frame: (x,y,w,h)}}
(normalized). Per-frame association: GREEDY best-IoU matching above iou_thr
(deterministic: sort candidate pairs by IoU desc, then ids). IDF1 identity
assignment: greedy on global per-(gt,pred) match counts (Hungarian-free; exact
enough for our small id counts — documented limitation)."""
from __future__ import annotations


def _iou(a, b):
    ax2, ay2 = a[0] + a[2], a[1] + a[3]
    bx2, by2 = b[0] + b[2], b[1] + b[3]
    iw = max(0.0, min(ax2, bx2) - max(a[0], b[0]))
    ih = max(0.0, min(ay2, by2) - max(a[1], b[1]))
    i = iw * ih
    return i / (a[2] * a[3] + b[2] * b[3] - i) if i > 0 else 0.0


def _frames_union(*track_dicts):
    fs = set()
    for tracks in track_dicts:
        for traj in tracks.values():
            fs.update(traj)
    return sorted(fs)


def score_mot(gt: dict, pred: dict, *, iou_thr: float = 0.5) -> dict:
    frames = _frames_union(gt, pred)
    n_gt = sum(len(t) for t in gt.values())
    fp = fn = idsw = 0
    matches_prev: dict = {}            # gt_id -> pred_id matched on previous frame
    pair_counts: dict = {}             # (gt_id, pred_id) -> matched frames
    gt_matched_frames = {g: set() for g in gt}

    for f in frames:
        g_boxes = {g: traj[f] for g, traj in gt.items() if f in traj}
        p_boxes = {p: traj[f] for p, traj in pred.items() if f in traj}
        cand = sorted(
            ((_iou(gb, pb), g, p) for g, gb in g_boxes.items()
             for p, pb in p_boxes.items()),
            key=lambda t: (-t[0], t[1], t[2]))
        used_g, used_p, matches = set(), set(), {}
        for iou, g, p in cand:
            if iou < iou_thr or g in used_g or p in used_p:
                continue
            used_g.add(g); used_p.add(p); matches[g] = p
        fn += len(g_boxes) - len(matches)
        fp += len(p_boxes) - len(matches)
        for g, p in matches.items():
            pair_counts[(g, p)] = pair_counts.get((g, p), 0) + 1
            gt_matched_frames[g].add(f)
            if g in matches_prev and matches_prev[g] != p:
                idsw += 1
        # carry forward last known assignment for IDsw across gaps (MOT convention)
        matches_prev.update(matches)

    # Frag: gaps inside each GT track's matched-frame set
    frag = 0
    for g, traj in gt.items():
        fs = sorted(set(traj) & gt_matched_frames[g])
        track_fs = sorted(traj)
        in_run = False
        seen_first = False
        for f in track_fs:
            m = f in gt_matched_frames[g]
            if m and seen_first and not in_run:
                frag += 1
            if m:
                seen_first = True
            in_run = m
        _ = fs

    # MT/ML on matched-coverage ratio
    mt = ml = 0
    for g, traj in gt.items():
        r = len(gt_matched_frames[g]) / len(traj) if traj else 0.0
        mt += r >= 0.8
        ml += r <= 0.2

    # IDF1 — greedy identity assignment by global overlap counts
    idtp = 0
    pairs = sorted(pair_counts.items(), key=lambda kv: (-kv[1], kv[0]))
    used_g, used_p = set(), set()
    for (g, p), c in pairs:
        if g in used_g or p in used_p:
            continue
        used_g.add(g); used_p.add(p); idtp += c
    n_pred = sum(len(t) for t in pred.values())
    idf1 = (2 * idtp / (n_gt + n_pred)) if (n_gt + n_pred) else 0.0

    mota = 1.0 - (fn + fp + idsw) / n_gt if n_gt else 0.0
    return {"MOTA": mota, "IDF1": idf1, "IDsw": idsw, "FP": fp, "FN": fn,
            "Frag": frag, "MT": mt, "ML": ml,
            "n_gt": n_gt, "n_pred": n_pred, "n_frames": len(frames)}
