"""CLI: repeat a track's last bbox forward to the clip end (detector dropout).

    python -m tiling_lab.gt.run_gt_hold_tail --outdir <gt_verify dir> \
        --track-ids 4 [--until-frame 1313]

Reads <outdir>/gt_tracks.verified.json (by default), and for each given track
id holds its last-frame bbox constant through --until-frame inclusive (default:
the clip's max frame across all tracks). Rewrites:
  <outdir>/gt_tracks.verified.json, <outdir>/overlay_verified.frames.json,
  <outdir>/corrections.json   (merges the hold record alongside pin/interp/despike).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .gt_edit import clip_frame_range, hold_track_tail
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--track-ids", required=True, help="comma-separated track ids, e.g. 4")
    ap.add_argument("--until-frame", type=int, default=None,
                    help="hold last bbox through this frame (default: clip max frame)")
    ap.add_argument("--source", default="gt_tracks.verified.json",
                    help="source tracks file in outdir (default gt_tracks.verified.json)")
    args = ap.parse_args()
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / args.source).read_text()))
    ids = [int(x) for x in args.track_ids.split(",")]
    until = args.until_frame if args.until_frame is not None else clip_frame_range(tracks)[1]
    fixed = hold_track_tail(tracks, track_ids=ids, until_frame=until)
    (out / "gt_tracks.verified.json").write_text(json.dumps(tracks_to_doc(fixed, clip=out.name)))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(fixed)))

    corr_path = out / "corrections.json"
    corrections = json.loads(corr_path.read_text()) if corr_path.exists() else {}
    corrections["hold_track_tail"] = {"track_ids": ids, "until_frame": until,
                                      "source": args.source}
    corr_path.write_text(json.dumps(corrections, indent=2))
    print(f"held tail bbox of tracks {ids} through frame {until}")
    print(f"wrote {out/'gt_tracks.verified.json'} + overlay + corrections.json")


if __name__ == "__main__":
    main()
