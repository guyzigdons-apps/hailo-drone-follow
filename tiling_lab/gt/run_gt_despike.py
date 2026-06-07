"""CLI: restore detector-truncated bbox heights ("only top of person") glitches.

    python -m tiling_lab.gt.run_gt_despike --outdir <gt_verify dir> \
        --track-ids 3,4 --min-ratio 0.7 --window 31

Reads <outdir>/gt_tracks.verified.json (by default), and for the given track
ids resets any frame whose bbox height is below --min-ratio * the local
windowed-median height back up to that median. --anchor auto (default) decides
per frame which vertical edge is stable (un-truncated) and holds it; --anchor
top holds the top edge so the box grows downward to the feet (legs cut);
--anchor bottom holds the bottom edge so the box grows upward over the head
(head cut). Rewrites:
  <outdir>/gt_tracks.verified.json, <outdir>/overlay_verified.frames.json,
  <outdir>/corrections.json   (merges the despike record alongside pin/interp).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .gt_edit import despike_track_heights
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--track-ids", required=True, help="comma-separated track ids, e.g. 3,4")
    ap.add_argument("--min-ratio", type=float, default=0.7,
                    help="flag frames with h < min-ratio * local median height (default 0.7)")
    ap.add_argument("--window", type=int, default=31,
                    help="odd centered window for the rolling median height (default 31)")
    ap.add_argument("--anchor", choices=["auto", "top", "bottom"], default="auto",
                    help="held vertical edge: 'auto' (default) picks the stable "
                         "edge per frame; 'top' grows downward (feet cut); "
                         "'bottom' grows upward (head cut)")
    ap.add_argument("--source", default="gt_tracks.verified.json",
                    help="source tracks file in outdir (default gt_tracks.verified.json)")
    args = ap.parse_args()
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / args.source).read_text()))
    ids = [int(x) for x in args.track_ids.split(",")]
    choice_counts = {} if args.anchor == "auto" else None
    fixed = despike_track_heights(tracks, track_ids=ids,
                                  min_ratio=args.min_ratio, window=args.window,
                                  anchor=args.anchor, choice_counts=choice_counts)
    (out / "gt_tracks.verified.json").write_text(json.dumps(tracks_to_doc(fixed, clip=out.name)))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(fixed)))

    corr_path = out / "corrections.json"
    corrections = json.loads(corr_path.read_text()) if corr_path.exists() else {}
    record = {"track_ids": ids, "min_ratio": args.min_ratio,
              "window": args.window, "anchor": args.anchor,
              "source": args.source}
    if choice_counts is not None:
        record["auto_choices"] = {"auto_top": choice_counts.get("auto_top", 0),
                                  "auto_bottom": choice_counts.get("auto_bottom", 0)}
    corrections["despike_track_heights"] = record
    corr_path.write_text(json.dumps(corrections, indent=2))
    print(f"despiked heights (min_ratio={args.min_ratio}, window={args.window}, "
          f"anchor={args.anchor}) for tracks {ids}")
    if choice_counts is not None:
        print(f"auto anchor holds: top={choice_counts.get('auto_top', 0)} "
              f"bottom={choice_counts.get('auto_bottom', 0)}")
    print(f"wrote {out/'gt_tracks.verified.json'} + overlay + corrections.json")


if __name__ == "__main__":
    main()
