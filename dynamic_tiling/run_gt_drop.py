"""CLI: drop spurious / rejected tracks (human-review verdict).

    python -m dynamic_tiling.run_gt_drop --outdir <gt_verify dir> --track-ids 6

Reads <outdir>/gt_tracks.json (by default), removes the given track ids, writes:
  <outdir>/gt_tracks.verified.json, <outdir>/overlay_verified.frames.json,
  <outdir>/corrections.json   (records the drop; first link of the chain).

Use as the FIRST correction (source gt_tracks.json) so later pin/interp/despike
steps (source gt_tracks.verified.json) build on the pruned track set.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .gt_edit import drop_tracks
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--track-ids", required=True, help="comma-separated track ids to drop, e.g. 6")
    ap.add_argument("--source", default="gt_tracks.json",
                    help="source tracks file in outdir (default gt_tracks.json)")
    args = ap.parse_args()
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / args.source).read_text()))
    ids = [int(x) for x in args.track_ids.split(",")]
    kept = drop_tracks(tracks, track_ids=ids)
    (out / "gt_tracks.verified.json").write_text(json.dumps(tracks_to_doc(kept, clip=out.name)))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(kept)))

    corr_path = out / "corrections.json"
    corrections = json.loads(corr_path.read_text()) if corr_path.exists() else {}
    corrections["drop_tracks"] = {"track_ids": ids, "source": args.source}
    corr_path.write_text(json.dumps(corrections, indent=2))
    print(f"dropped tracks {ids}; kept {len(kept)} tracks")
    print(f"wrote {out/'gt_tracks.verified.json'} + overlay + corrections.json")


if __name__ == "__main__":
    main()
