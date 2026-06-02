"""CLI: dense frames.json + video -> gt_tracks.json (+ spot-check overlay).

    source setup_env.sh
    python -m dynamic_tiling.run_gt_tracks \
        --dense /tmp/gt_legacy/12x9.frames.json \
        --video /home/giladn/Videos/Drone/Training/DJI_..._0026_..._fov50.mp4 \
        --out dynamic_tiling/runs/gt_tracks_0026_fov50.json \
        --overlay dynamic_tiling/runs/gt_tracks_0026_fov50.frames.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from hailo_tiling.classes import TRACKED_CLASSES, label as cls_label
from .gt_clean import GtTrack, clean_tracks


def tracks_to_doc(tracks, *, clip: str) -> dict:
    return {"clip": clip, "tracks": [
        {"cls": t.cls, "track_id": t.track_id,
         "frames": {str(f): list(b) for f, b in sorted(t.frames.items())}}
        for t in tracks]}


def doc_to_tracks(doc) -> list[GtTrack]:
    return [GtTrack(cls=t["cls"], track_id=t["track_id"],
                    frames={int(f): tuple(b) for f, b in t["frames"].items()})
            for t in doc["tracks"]]


def _overlay_doc(tracks, label: str) -> dict:
    """One frames.json with every GT track's bbox, coloured by class label."""
    per_frame: dict = {}
    for t in tracks:
        for f, b in t.frames.items():
            per_frame.setdefault(f, []).append(
                {"label": cls_label(t.cls),
                 "confidence": 1.0, "bbox": list(b),
                 "track_id": t.track_id})
    frames = [{"frame": f, "detections": per_frame[f], "tiles": []}
              for f in sorted(per_frame)]
    return {"label": label, "frames": frames}


def overlay_doc_by_id(tracks, label: str = "gt-by-id") -> dict:
    """Overlay where each box's label is '<class>#<track_id>' so the viewer shows
    track identity (not just class)."""
    per_frame: dict = {}
    for t in tracks:
        name = f"{cls_label(t.cls)}#{t.track_id}"
        for f, b in t.frames.items():
            per_frame.setdefault(f, []).append(
                {"label": name, "confidence": 1.0, "bbox": list(b),
                 "track_id": t.track_id})
    frames = [{"frame": f, "detections": per_frame[f], "tiles": []}
              for f in sorted(per_frame)]
    return {"label": label, "frames": frames}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dense", required=True, type=Path)
    ap.add_argument("--video", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--overlay", type=Path, default=None)
    ap.add_argument("--max-gap", type=int, default=5)
    ap.add_argument("--min-len", type=int, default=30)
    args = ap.parse_args()

    from .gt_mot import build_raw_tracks_from_video, make_botsort
    doc = json.loads(args.dense.read_text())
    raw = build_raw_tracks_from_video(doc, str(args.video),
                                      tracker_factory=make_botsort,
                                      classes=TRACKED_CLASSES)
    tracks = clean_tracks(raw, max_gap=args.max_gap, min_len=args.min_len)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(tracks_to_doc(tracks, clip=args.video.stem)))
    print(f"GT tracks written: {args.out}  ({len(tracks)} tracks)")
    if args.overlay:
        args.overlay.parent.mkdir(parents=True, exist_ok=True)
        args.overlay.write_text(json.dumps(_overlay_doc(tracks, label=args.out.stem)))
        print(f"overlay written : {args.overlay}")


if __name__ == "__main__":
    main()
