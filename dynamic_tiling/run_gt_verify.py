"""CLI: human-in-the-loop GT verification.

Build mode (default, runs in .venv_gt — needs boxmot + video):
    .venv_gt/bin/python -m dynamic_tiling.run_gt_verify \
        --dense /tmp/gt_legacy/12x9.frames.json --video <clip.mp4> \
        --outdir dynamic_tiling/runs/gt_verify_0026_fov50

Finalize mode (no boxmot needed):
    python -m dynamic_tiling.run_gt_verify --finalize \
        --outdir dynamic_tiling/runs/gt_verify_0026_fov50 \
        --decisions dynamic_tiling/runs/gt_verify_0026_fov50/review_decisions.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .gt_clean import clean_tracks
from .gt_review import merge_and_flag, apply_decisions, ReviewCase
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id


def cases_to_doc(cases) -> dict:
    return {"cases": [{"kind": c.kind, "frame": c.frame,
                       "track_ids": list(c.track_ids), "reason": c.reason,
                       "score": c.score} for c in cases]}


def doc_to_cases(doc) -> list:
    return [ReviewCase(kind=c["kind"], frame=c["frame"],
                       track_ids=tuple(c["track_ids"]), boxes=[],
                       reason=c.get("reason", ""), score=c.get("score", 0.0))
            for c in doc["cases"]]


def _read_frames(video, frame_idxs):
    import cv2
    want = set(frame_idxs)
    out = {}
    cap = cv2.VideoCapture(str(video))
    fi = -1
    try:
        while want:
            ok, fr = cap.read()
            if not ok:
                break
            fi += 1
            if fi in want:
                out[fi] = fr
                want.discard(fi)
    finally:
        cap.release()
    return out


def _build(args):
    from .gt_mot import build_raw_tracks_from_video, make_botsort
    from .gt_render_review import render_queue
    doc = json.loads(args.dense.read_text())
    raw = build_raw_tracks_from_video(doc, str(args.video),
                                      tracker_factory=make_botsort,
                                      dedup_iou=args.dedup_iou)
    clean = clean_tracks(raw, max_gap=args.max_gap, min_len=1)  # keep short; flag later
    merged, cases = merge_and_flag(clean, auto_iou=args.auto_iou,
                                   flag_iou=args.flag_iou, min_len=args.min_len)
    out = Path(args.outdir); out.mkdir(parents=True, exist_ok=True)
    (out / "gt_tracks.json").write_text(json.dumps(tracks_to_doc(merged, clip=Path(args.video).stem)))
    (out / "review_queue.json").write_text(json.dumps(cases_to_doc(cases), indent=2))
    (out / "overlay_by_id.frames.json").write_text(json.dumps(overlay_doc_by_id(merged)))
    frames = _read_frames(args.video, [c.frame for c in cases])
    paths = render_queue(frames, cases, out / "review")
    print(f"tracks: {len(merged)} | review cases: {len(cases)} | images: {len(paths)}")
    print(f"queue : {out/'review_queue.json'}")
    print(f"images: {out/'review'}")


def _finalize(args):
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / "gt_tracks.json").read_text()))
    cases = doc_to_cases(json.loads((out / "review_queue.json").read_text()))
    decisions = {int(k): v for k, v in json.loads(Path(args.decisions).read_text()).items()}
    final = apply_decisions(tracks, cases, decisions)
    final_doc = tracks_to_doc(final, clip=out.name)
    (out / "gt_tracks.verified.json").write_text(json.dumps(final_doc))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(final)))
    print(f"verified tracks: {len(final)}")
    print(f"verified GT : {out/'gt_tracks.verified.json'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--dense", type=Path)
    ap.add_argument("--video", type=Path)
    ap.add_argument("--dedup-iou", type=float, default=0.5)
    ap.add_argument("--max-gap", type=int, default=5)
    ap.add_argument("--min-len", type=int, default=30)
    ap.add_argument("--auto-iou", type=float, default=0.7)
    ap.add_argument("--flag-iou", type=float, default=0.3)
    ap.add_argument("--finalize", action="store_true")
    ap.add_argument("--decisions", type=Path)
    args = ap.parse_args()
    if args.finalize:
        _finalize(args)
    else:
        _build(args)


if __name__ == "__main__":
    main()
