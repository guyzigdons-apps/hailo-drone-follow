from __future__ import annotations


def _iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = a[0], a[1], a[0] + a[2], a[1] + a[3]
    bx1, by1, bx2, by2 = b[0], b[1], b[0] + b[2], b[1] + b[3]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = a[2] * a[3] + b[2] * b[3] - inter
    return inter / ua if ua > 0 else 0.0


def _centre_dist(a, b) -> float:
    acx, acy = a[0] + a[2] / 2, a[1] + a[3] / 2
    bcx, bcy = b[0] + b[2] / 2, b[1] + b[3] / 2
    return ((acx - bcx) ** 2 + (acy - bcy) ** 2) ** 0.5


def build_target_trajectory(doc: dict, label: str = "person",
                            anchor: str = "largest",
                            min_iou: float = 0.1,
                            max_centre_jump: float = 0.15) -> dict:
    """Return {frame_idx: (x,y,w,h)} for ONE target stitched by greedy
    association. Picks the `anchor` ("largest") box on the first frame that has
    one, then follows it; on a gap, re-associates by nearest centre."""
    frames = sorted(doc.get("frames", []), key=lambda fr: fr["frame"])

    def boxes(fr):
        return [tuple(d["bbox"]) for d in fr.get("detections", [])
                if d.get("label") == label]

    cur = None
    out: dict = {}
    for fr in frames:
        bs = boxes(fr)
        if not bs:
            continue
        if cur is None:
            cur = max(bs, key=lambda b: b[2] * b[3]) if anchor == "largest" else bs[0]
            out[fr["frame"]] = cur
            continue
        best = max(bs, key=lambda b: _iou(cur, b))
        if _iou(cur, best) < min_iou:
            best = min(bs, key=lambda b: _centre_dist(cur, b))
            if _centre_dist(cur, best) > max_centre_jump:
                continue
        out[fr["frame"]] = best
        cur = best
    return out
