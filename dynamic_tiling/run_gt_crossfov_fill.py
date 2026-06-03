"""CLI: fill an unstable track from the same object's clean track in another FOV.

    python -m dynamic_tiling.run_gt_crossfov_fill \
        --outdir dynamic_tiling/runs/gt_verify_0025_fov60 --track-id 4 \
        --src-dir dynamic_tiling/runs/gt_verify_0025_fov50 --src-track-id 4 \
        --src-fov 50 --dst-fov 60

Both FOVs are centered crops of the SAME source clip scaled to the SAME 4K
output, so a normalized bbox maps affinely about the centre. This reads the
source FOV's gt_tracks.verified.json, takes --src-track-id, projects every frame
into the destination FOV (scale = crop(src)/crop(dst) from prepare_video's
geometry), and REPLACES the destination --track-id's frames with the projection.
Rewrites <outdir>/gt_tracks.verified.json + overlay + corrections.json.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from tiling_benchmark.prepare_video import fov_to_crop_dims
from .gt_edit import crossfov_fill_track
from .run_gt_tracks import tracks_to_doc, doc_to_tracks, overlay_doc_by_id


def _track_frames(doc_path: Path, track_id: int) -> dict:
    doc = json.loads(doc_path.read_text())
    for t in doc["tracks"]:
        if t["track_id"] == track_id:
            return {int(f): tuple(b) for f, b in t["frames"].items()}
    raise SystemExit(f"source track {track_id} not found in {doc_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path)
    ap.add_argument("--track-id", required=True, type=int, help="target track id in THIS fov")
    ap.add_argument("--src-dir", required=True, type=Path, help="source fov gt_verify dir")
    ap.add_argument("--src-track-id", required=True, type=int)
    ap.add_argument("--src-fov", required=True, type=int)
    ap.add_argument("--dst-fov", required=True, type=int)
    ap.add_argument("--src-file", default="gt_tracks.verified.json")
    ap.add_argument("--source", default="gt_tracks.verified.json",
                    help="destination tracks file in outdir to edit")
    args = ap.parse_args()

    cw_s, ch_s = fov_to_crop_dims(args.src_fov)
    cw_d, ch_d = fov_to_crop_dims(args.dst_fov)
    sx, sy = cw_s / cw_d, ch_s / ch_d

    src_frames = _track_frames(Path(args.src_dir) / args.src_file, args.src_track_id)
    out = Path(args.outdir)
    tracks = doc_to_tracks(json.loads((out / args.source).read_text()))
    filled = crossfov_fill_track(tracks, track_id=args.track_id,
                                 source_frames=src_frames, sx=sx, sy=sy)
    (out / "gt_tracks.verified.json").write_text(json.dumps(tracks_to_doc(filled, clip=out.name)))
    (out / "overlay_verified.frames.json").write_text(json.dumps(overlay_doc_by_id(filled)))

    corr_path = out / "corrections.json"
    corrections = json.loads(corr_path.read_text()) if corr_path.exists() else {}
    # this correction can run once per filled track, so accumulate a list
    fills = corrections.get("crossfov_fill_track", [])
    fills.append({
        "track_id": args.track_id, "src_dir": str(args.src_dir),
        "src_track_id": args.src_track_id, "src_fov": args.src_fov,
        "dst_fov": args.dst_fov, "sx": sx, "sy": sy, "source": args.source})
    corrections["crossfov_fill_track"] = fills
    corr_path.write_text(json.dumps(corrections, indent=2))
    print(f"filled track {args.track_id} from {args.src_dir} track {args.src_track_id} "
          f"(fov{args.src_fov}->fov{args.dst_fov}, sx={sx:.4f}, sy={sy:.4f}); "
          f"{len(src_frames)} frames projected")
    print(f"wrote {out/'gt_tracks.verified.json'} + overlay + corrections.json")


if __name__ == "__main__":
    main()
