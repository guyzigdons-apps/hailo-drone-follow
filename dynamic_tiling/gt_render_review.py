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


_COLORS = [(0, 0, 255), (0, 200, 0), (255, 128, 0), (200, 0, 200)]


def render_case(frame_bgr, case: ReviewCase, *, index: int):
    """Draw the candidate boxes + caption onto a copy of frame_bgr. Returns the image."""
    import cv2
    img = frame_bgr.copy()
    h, w = img.shape[:2]
    for k, (cls, tid, bbox) in enumerate(case.boxes):
        x, y, bw, bh = norm_to_px(bbox, w, h)
        col = _COLORS[k % len(_COLORS)]
        cv2.rectangle(img, (x, y), (x + bw, y + bh), col, 2)
        cv2.putText(img, f"id{tid}", (x, max(0, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    cap = case_caption(case, index=index)
    cv2.rectangle(img, (0, 0), (w, 26), (0, 0, 0), -1)
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
