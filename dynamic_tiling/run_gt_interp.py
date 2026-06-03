"""CLI: interpolate in-span gaps on selected tracks to kill flicker.

    python -m dynamic_tiling.run_gt_interp --outdir <gt_verify dir> \
        --track-ids 3,4 --max-gap 10

Reads <outdir>/gt_tracks.verified.json (the pinned output, by default), fills
detection-dropout gaps (<= --max-gap) for the given track ids by linear bbox
interpolation, and rewrites:
  <outdir>/gt_tracks.verified.json, <outdir>/overlay_verified.frames.json,
  <outdir>/corrections.json   (merges the interp record alongside any pin).

Out-of-span absence (a track genuinely off-screen before its first / after its
last frame) is never filled -- only gaps inside the existing span.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .gt_edit import interp_track_gaps
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--track-ids", required=True, help="comma-separated track ids, e.g. 3,4")
    ap.add_argument("--max-gap", type=int, default=10,
                    help="max in-span gap length to interpolate (default 10)")
    ap.add_argument("--source", default="gt_tracks.verified.json",
                    help="source tracks file in outdir (default gt_tracks.verified.json)")
    args = ap.parse_args()
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / args.source).read_text()))
    ids = [int(x) for x in args.track_ids.split(",")]
    fixed = interp_track_gaps(tracks, track_ids=ids, max_gap=args.max_gap)
    (out / "gt_tracks.verified.json").write_text(json.dumps(tracks_to_doc(fixed, clip=out.name)))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(fixed)))

    corr_path = out / "corrections.json"
    corrections = json.loads(corr_path.read_text()) if corr_path.exists() else {}
    # interp can run more than once per chain (different tracks / max_gaps) -> list
    recs = corrections.get("interp_track_gaps", [])
    if isinstance(recs, dict):  # back-compat with the single-dict format
        recs = [recs]
    recs.append({"track_ids": ids, "max_gap": args.max_gap, "source": args.source})
    corrections["interp_track_gaps"] = recs
    corr_path.write_text(json.dumps(corrections, indent=2))
    print(f"interpolated in-span gaps (<= {args.max_gap}) for tracks {ids}")
    print(f"wrote {out/'gt_tracks.verified.json'} + overlay + corrections.json")


if __name__ == "__main__":
    main()
