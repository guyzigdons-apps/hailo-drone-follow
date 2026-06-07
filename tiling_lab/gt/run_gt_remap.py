"""CLI: remap track ids to a canonical cross-fov/clip scheme.

    python -m tiling_lab.gt.run_gt_remap --outdir <gt_verify dir> \
        --mapping 1:1,3:2,2:3,5:4

Reads <outdir>/gt_tracks.verified.json (by default), renames track ids per the
mapping (old:new, comma-separated; unlisted ids keep their value), writes:
  <outdir>/gt_tracks.verified.json, <outdir>/overlay_verified.frames.json,
  <outdir>/corrections.json   (merges the remap record into the chain).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .gt_edit import remap_track_ids
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id


def _parse_mapping(s: str) -> dict:
    out = {}
    for pair in s.split(","):
        old, new = pair.split(":")
        out[int(old)] = int(new)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--mapping", required=True, help="old:new pairs, e.g. 1:1,3:2,2:3,5:4")
    ap.add_argument("--source", default="gt_tracks.verified.json",
                    help="source tracks file in outdir (default gt_tracks.verified.json)")
    args = ap.parse_args()
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / args.source).read_text()))
    mapping = _parse_mapping(args.mapping)
    remapped = remap_track_ids(tracks, mapping=mapping)
    (out / "gt_tracks.verified.json").write_text(json.dumps(tracks_to_doc(remapped, clip=out.name)))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(remapped)))

    corr_path = out / "corrections.json"
    corrections = json.loads(corr_path.read_text()) if corr_path.exists() else {}
    corrections["remap_track_ids"] = {"mapping": {str(k): v for k, v in mapping.items()},
                                      "source": args.source}
    corr_path.write_text(json.dumps(corrections, indent=2))
    print(f"remapped track ids {mapping}")
    print(f"wrote {out/'gt_tracks.verified.json'} + overlay + corrections.json")


if __name__ == "__main__":
    main()
