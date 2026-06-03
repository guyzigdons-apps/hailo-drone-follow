"""CLI: restore the width of a track occluded on one side (e.g. a car behind another).

    python -m dynamic_tiling.run_gt_restore_width --outdir <gt_verify dir> \
        --track-ids 2 --anchor left --percentile 90 --min-ratio 0.85 --window 151

Reads <outdir>/gt_tracks.verified.json (by default), and for the given tracks
resets any frame whose width is below --min-ratio * the local windowed
--percentile width (the unoccluded width) back up to that target, holding the
stable --anchor edge (left=xmin / right=xmax / center). Rewrites the verified GT
+ overlay + corrections.json (accumulated as a list).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .gt_edit import restore_track_width
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--track-ids", required=True, help="comma-separated track ids, e.g. 2")
    ap.add_argument("--anchor", default="left", choices=["left", "right", "center"],
                    help="edge to hold fixed (default left = xmin)")
    ap.add_argument("--percentile", type=float, default=90,
                    help="local width percentile treated as the unoccluded width (default 90)")
    ap.add_argument("--min-ratio", type=float, default=0.85,
                    help="restore frames with width < min-ratio * target (default 0.85)")
    ap.add_argument("--window", type=int, default=151,
                    help="centered window for the local percentile (default 151)")
    ap.add_argument("--source", default="gt_tracks.verified.json")
    args = ap.parse_args()
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / args.source).read_text()))
    ids = [int(x) for x in args.track_ids.split(",")]
    fixed = restore_track_width(tracks, track_ids=ids, anchor=args.anchor,
                                percentile=args.percentile, min_ratio=args.min_ratio,
                                window=args.window)
    (out / "gt_tracks.verified.json").write_text(json.dumps(tracks_to_doc(fixed, clip=out.name)))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(fixed)))

    corr_path = out / "corrections.json"
    corrections = json.loads(corr_path.read_text()) if corr_path.exists() else {}
    recs = corrections.get("restore_track_width", [])
    recs.append({"track_ids": ids, "anchor": args.anchor, "percentile": args.percentile,
                 "min_ratio": args.min_ratio, "window": args.window, "source": args.source})
    corrections["restore_track_width"] = recs
    corr_path.write_text(json.dumps(corrections, indent=2))
    print(f"restored width of tracks {ids} (anchor={args.anchor}, p{args.percentile}, "
          f"min_ratio={args.min_ratio}, window={args.window})")


if __name__ == "__main__":
    main()
