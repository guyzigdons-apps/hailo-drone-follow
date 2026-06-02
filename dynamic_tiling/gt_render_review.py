"""Render annotated review images for human GT verification."""
from __future__ import annotations

from pathlib import Path

from .gt_review import ReviewCase


def norm_to_px(bbox, w: int, h: int):
    x, y, bw, bh = bbox
    return (int(round(x * w)), int(round(y * h)),
            int(round(bw * w)), int(round(bh * h)))


def case_caption(case: ReviewCase, *, index: int) -> str:
    ids = "+".join(str(i) for i in case.track_ids)
    if case.kind == "merge":
        return (f"[{index}] PROPOSED MERGE  ids {ids}  frame {case.frame}  "
                f"({case.reason}) -> verdict: merge / keep-separate")
    if case.kind == "keep_short":
        return (f"[{index}] REAL OBJECT? short track id {ids}  frame {case.frame}  "
                f"({case.reason}) -> verdict: keep / drop")
    return (f"[{index}] {case.kind.upper()} ids {ids} frame {case.frame} ({case.reason})")


def crop_region(boxes, frame_w: int, frame_h: int, *,
                pad_frac: float = 3.0, min_frac: float = 0.18):
    """Return a pixel crop rect (x, y, w, h) covering the union of candidate boxes.

    boxes: list of (cls, track_id, bbox) where bbox is normalized (x, y, w, h).
    The crop includes generous padding (pad_frac * max(union_w, union_h) on each
    side), enforces a minimum normalized size of min_frac on each dimension, and is
    clamped to the frame.  Returns the whole frame when boxes is empty.
    """
    if not boxes:
        return (0, 0, frame_w, frame_h)

    # Compute normalized union
    x1 = min(bbox[0] for _, _, bbox in boxes)
    y1 = min(bbox[1] for _, _, bbox in boxes)
    x2 = max(bbox[0] + bbox[2] for _, _, bbox in boxes)
    y2 = max(bbox[1] + bbox[3] for _, _, bbox in boxes)

    union_w = x2 - x1
    union_h = y2 - y1

    # Add symmetric padding
    pad = pad_frac * max(union_w, union_h)
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    half_w = (union_w / 2.0) + pad
    half_h = (union_h / 2.0) + pad

    # Enforce minimum crop size (centered on union center)
    half_w = max(half_w, min_frac / 2.0)
    half_h = max(half_h, min_frac / 2.0)

    # Normalized crop bounds before clamping
    nx1 = cx - half_w
    ny1 = cy - half_h
    nx2 = cx + half_w
    ny2 = cy + half_h

    # Clamp to [0, 1]
    nx1 = max(0.0, nx1)
    ny1 = max(0.0, ny1)
    nx2 = min(1.0, nx2)
    ny2 = min(1.0, ny2)

    # Convert to pixels
    px = int(round(nx1 * frame_w))
    py = int(round(ny1 * frame_h))
    pw = int(round((nx2 - nx1) * frame_w))
    ph = int(round((ny2 - ny1) * frame_h))

    # Ensure we don't exceed frame bounds (rounding edge cases)
    pw = min(pw, frame_w - px)
    ph = min(ph, frame_h - py)

    return (px, py, pw, ph)


_COLORS = [(0, 0, 255), (0, 200, 0), (255, 128, 0), (200, 0, 200)]


def render_case(frame_bgr, case: ReviewCase, *, index: int, target_w: int = 900):
    """Crop to candidate boxes, upscale to target_w, draw boxes + caption. Returns the image."""
    import cv2
    H, W = frame_bgr.shape[:2]

    cx, cy, cw, ch = crop_region(case.boxes, W, H)
    crop = frame_bgr[cy:cy + ch, cx:cx + cw].copy()

    scale = target_w / cw
    out_h = int(ch * scale)
    img = cv2.resize(crop, (target_w, out_h), interpolation=cv2.INTER_LINEAR)

    for k, (cls, tid, bbox) in enumerate(case.boxes):
        bx, by, bw, bh = norm_to_px(bbox, W, H)
        sx = int((bx - cx) * scale)
        sy = int((by - cy) * scale)
        sw = int(bw * scale)
        sh = int(bh * scale)
        col = _COLORS[k % len(_COLORS)]
        cv2.rectangle(img, (sx, sy), (sx + sw, sy + sh), col, 2)
        cv2.putText(img, f"id{tid}", (sx, max(0, sy - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)

    cap = case_caption(case, index=index)
    cv2.rectangle(img, (0, 0), (target_w, 26), (0, 0, 0), -1)
    cv2.putText(img, cap[:140], (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    return img


def render_queue(frames_by_idx, cases, out_dir: Path):
    """frames_by_idx: {frame_idx: bgr image}. Writes case_NNNN.png per case.
    Returns list of written paths."""
    import cv2
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for i, case in enumerate(cases):
        frame = frames_by_idx.get(case.frame)
        if frame is None:
            continue
        img = render_case(frame, case, index=i)
        p = out_dir / f"case_{i:04d}.png"
        cv2.imwrite(str(p), img)
        paths.append(p)
    return paths
