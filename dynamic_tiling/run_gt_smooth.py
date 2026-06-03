"""CLI: temporally smooth selected bbox components of a track (remove jitter).

    python -m dynamic_tiling.run_gt_smooth --outdir <gt_verify dir> \
        --track-ids 2 --window 21 --dims x,w

Reads <outdir>/gt_tracks.verified.json (by default), applies a centered
moving-average over --window frames to the --dims components (default x,w =
horizontal only) of the given tracks, and rewrites the verified GT + overlay +
corrections.json (accumulated as a list). Use after width/position fixes to
remove per-frame detector jitter on a (near-)static object.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .gt_edit import smooth_track_bbox
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--track-ids", required=True, help="comma-separated track ids, e.g. 2")
    ap.add_argument("--window", type=int, default=21, help="moving-average window (default 21)")
    ap.add_argument("--dims", default="x,w",
                    help="bbox components to smooth, comma-separated of x,y,w,h (default x,w)")
    ap.add_argument("--source", default="gt_tracks.verified.json")
    args = ap.parse_args()
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / args.source).read_text()))
    ids = [int(x) for x in args.track_ids.split(",")]
    dims = tuple(d.strip() for d in args.dims.split(","))
    smoothed = smooth_track_bbox(tracks, track_ids=ids, window=args.window, dims=dims)
    (out / "gt_tracks.verified.json").write_text(json.dumps(tracks_to_doc(smoothed, clip=out.name)))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(smoothed)))

    corr_path = out / "corrections.json"
    corrections = json.loads(corr_path.read_text()) if corr_path.exists() else {}
    recs = corrections.get("smooth_track_bbox", [])
    recs.append({"track_ids": ids, "window": args.window, "dims": list(dims),
                 "source": args.source})
    corrections["smooth_track_bbox"] = recs
    corr_path.write_text(json.dumps(corrections, indent=2))
    print(f"smoothed tracks {ids} dims={dims} window={args.window}")


if __name__ == "__main__":
    main()
