"""CLI: pin static tracks to a reference-frame bbox across the whole clip.

    python -m tiling_lab.gt.run_gt_pin --outdir <gt_verify dir> \
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
    ap.add_argument("--frame-range", default=None,
                    help="lo,hi inclusive to pin over (default: full clip range). "
                         "Use for an object present only part of the clip.")
    ap.add_argument("--source", default="gt_tracks.json",
                    help="source tracks file in outdir (default gt_tracks.json)")
    args = ap.parse_args()
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / args.source).read_text()))
    ids = [int(x) for x in args.track_ids.split(",")]
    if args.frame_range:
        lo, hi = (int(x) for x in args.frame_range.split(","))
        rng = (lo, hi)
    else:
        rng = clip_frame_range(tracks)
    pinned = pin_static_tracks(tracks, track_ids=ids, ref_frame=args.ref_frame, frame_range=rng)
    (out / "gt_tracks.verified.json").write_text(json.dumps(tracks_to_doc(pinned, clip=out.name)))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(pinned)))
    corr_path = out / "corrections.json"
    corrections = json.loads(corr_path.read_text()) if corr_path.exists() else {}
    # pin can run more than once per chain (different tracks / ranges) -> list
    pins = corrections.get("pin_static_tracks", [])
    if isinstance(pins, dict):  # back-compat with single-dict format
        pins = [pins]
    pins.append({"track_ids": ids, "ref_frame": args.ref_frame,
                 "frame_range": list(rng), "source": args.source})
    corrections["pin_static_tracks"] = pins
    corr_path.write_text(json.dumps(corrections, indent=2))
    print(f"pinned tracks {ids} to frame {args.ref_frame} over {rng}")
    print(f"wrote {out/'gt_tracks.verified.json'} + overlay + corrections.json")


if __name__ == "__main__":
    main()
