"""CLI: pin static tracks to a reference-frame bbox across the whole clip.

    python -m dynamic_tiling.run_gt_pin --outdir <gt_verify dir> \
        --track-ids 1,2 --ref-frame 330

Reads <outdir>/gt_tracks.json, pins the given track ids to their bbox at
--ref-frame across the full clip frame range, writes:
  <outdir>/gt_tracks.verified.json, <outdir>/overlay_verified.frames.json,
  <outdir>/corrections.json   (records the pin so it is replayable).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .gt_edit import clip_frame_range, pin_static_tracks
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--track-ids", required=True, help="comma-separated track ids, e.g. 1,2")
    ap.add_argument("--ref-frame", required=True, type=int)
    ap.add_argument("--source", default="gt_tracks.json",
                    help="source tracks file in outdir (default gt_tracks.json)")
    args = ap.parse_args()
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / args.source).read_text()))
    ids = [int(x) for x in args.track_ids.split(",")]
    rng = clip_frame_range(tracks)
    pinned = pin_static_tracks(tracks, track_ids=ids, ref_frame=args.ref_frame, frame_range=rng)
    (out / "gt_tracks.verified.json").write_text(json.dumps(tracks_to_doc(pinned, clip=out.name)))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(pinned)))
    (out / "corrections.json").write_text(json.dumps(
        {"pin_static_tracks": {"track_ids": ids, "ref_frame": args.ref_frame,
                               "frame_range": list(rng), "source": args.source}}, indent=2))
    print(f"pinned tracks {ids} to frame {args.ref_frame} over {rng}")
    print(f"wrote {out/'gt_tracks.verified.json'} + overlay + corrections.json")


if __name__ == "__main__":
    main()
