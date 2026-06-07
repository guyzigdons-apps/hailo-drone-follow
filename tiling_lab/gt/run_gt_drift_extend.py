"""CLI: extend a parked object's track via camera drift from a reference track.

    python -m tiling_lab.gt.run_gt_drift_extend --outdir <gt_verify dir> \
        --track-id 5 --ref-track-id 1 --frame-range 0,1018

Reads <outdir>/gt_tracks.verified.json (by default), and for frames in
--frame-range that the target --track-id lacks, translates its nearest-end bbox
by the --ref-track-id's per-frame drift (a dense static object that tracks the
camera). Use when a parked object is present from earlier than it was detected
and a frozen box would mis-track the drift. Rewrites the verified GT + overlay +
corrections.json (accumulated as a list).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .gt_edit import drift_extend_track
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--track-id", required=True, type=int, help="target track to extend")
    ap.add_argument("--ref-track-id", required=True, type=int,
                    help="dense static reference track providing the drift")
    ap.add_argument("--frame-range", required=True, help="lo,hi inclusive to extend over")
    ap.add_argument("--source", default="gt_tracks.verified.json")
    args = ap.parse_args()
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / args.source).read_text()))
    ref = next((t for t in tracks if t.track_id == args.ref_track_id), None)
    if ref is None:
        raise SystemExit(f"reference track {args.ref_track_id} not found")
    lo, hi = (int(x) for x in args.frame_range.split(","))
    extended = drift_extend_track(tracks, track_id=args.track_id,
                                  ref_frames=ref.frames, frame_range=(lo, hi))
    (out / "gt_tracks.verified.json").write_text(json.dumps(tracks_to_doc(extended, clip=out.name)))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(extended)))

    corr_path = out / "corrections.json"
    corrections = json.loads(corr_path.read_text()) if corr_path.exists() else {}
    recs = corrections.get("drift_extend_track", [])
    recs.append({"track_id": args.track_id, "ref_track_id": args.ref_track_id,
                 "frame_range": [lo, hi], "source": args.source})
    corrections["drift_extend_track"] = recs
    corr_path.write_text(json.dumps(corrections, indent=2))
    tgt = next(t for t in extended if t.track_id == args.track_id)
    print(f"drift-extended track {args.track_id} via ref {args.ref_track_id} over {lo}..{hi}; "
          f"now n={len(tgt.frames)} span={min(tgt.frames)}..{max(tgt.frames)}")


if __name__ == "__main__":
    main()
