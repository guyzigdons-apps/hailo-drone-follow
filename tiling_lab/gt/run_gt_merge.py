"""CLI: merge ID-split track fragments of the same object into one track.

    python -m tiling_lab.gt.run_gt_merge --outdir <gt_verify dir> \
        --groups "4,10,15;3,7,11,12,16;6,9,13"

Reads <outdir>/gt_tracks.json (by default), merges each ';'-separated group of
track ids (comma-separated) into a single track keeping the group's min id, and
writes the verified GT + overlay + corrections.json. Use as an early chain step
(before remap), the source defaults to gt_tracks.json.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .gt_edit import merge_tracks
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--groups", required=True,
                    help="';'-separated groups, each comma-separated ids, e.g. 4,10,15;3,7,11")
    ap.add_argument("--source", default="gt_tracks.json")
    args = ap.parse_args()
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / args.source).read_text()))
    groups = [[int(x) for x in g.split(",")] for g in args.groups.split(";") if g.strip()]
    merged = merge_tracks(tracks, groups=groups)
    (out / "gt_tracks.verified.json").write_text(json.dumps(tracks_to_doc(merged, clip=out.name)))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(merged)))

    corr_path = out / "corrections.json"
    corrections = json.loads(corr_path.read_text()) if corr_path.exists() else {}
    corrections["merge_tracks"] = {"groups": groups, "source": args.source}
    corr_path.write_text(json.dumps(corrections, indent=2))
    print(f"merged groups {groups}; kept {len(merged)} tracks")


if __name__ == "__main__":
    main()
