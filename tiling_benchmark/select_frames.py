"""Select N frames from a GT frames.json by small-object detection density.

For each frame in the input GT frames.json, count detections whose object
height (in source pixels) is <= 64 and whose label is in {'person',
'vehicle'}. Pick the top N frames by that count, then re-sort the chosen
indices in ascending order so a subset video extracted from them is
chronological.

Output JSON shape::

    {"src_h": 3384, "n_selected": N, "frame_indices": [...]}

Usage::

  python tiling_benchmark/select_frames.py \\
      --input tiling_benchmark/pxt_runs/pxt_GT-12x9-25.frames.json \\
      --n 50 \\
      --out tiling_benchmark/pxt_runs/upscale_subset/selected_frames.json
"""

import argparse
import json
import sys
from pathlib import Path

SMALL_PX_THRESHOLD = 64
SRC_H_ASSUMED = 3384
HEADLINE_LABELS = {"person", "vehicle"}


def count_small(detections, src_h):
    n = 0
    for d in detections:
        label = d.get("label")
        if label not in HEADLINE_LABELS:
            continue
        bbox = d.get("bbox")
        if not bbox or len(bbox) < 4:
            continue
        h_px = float(bbox[3]) * src_h
        if h_px <= SMALL_PX_THRESHOLD:
            n += 1
    return n


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", type=Path, required=True,
                    help="GT frames.json with per-frame normalized detections")
    ap.add_argument("--n", type=int, default=50,
                    help="Number of frames to select (top-N by small-object count)")
    ap.add_argument("--out", type=Path, required=True,
                    help="Output JSON file")
    ap.add_argument("--src-h", type=int, default=SRC_H_ASSUMED,
                    help="Assumed source height in pixels for converting "
                         "normalized bbox height to pixels")
    args = ap.parse_args()

    if not args.input.is_file():
        print(f"ERROR: input not found: {args.input}", file=sys.stderr)
        return 1

    with args.input.open() as f:
        doc = json.load(f)
    frames = doc.get("frames")
    if not isinstance(frames, list):
        print("ERROR: input JSON has no 'frames' list", file=sys.stderr)
        return 1

    # (frame_idx, small_count)
    scored = []
    for fr in frames:
        idx = fr.get("frame")
        if idx is None:
            continue
        n_small = count_small(fr.get("detections") or [], args.src_h)
        scored.append((int(idx), n_small))

    # Frames that have at least one small-object detection.
    nonzero = [(idx, c) for (idx, c) in scored if c > 0]
    n_nonzero = len(nonzero)
    print(f"frames total:     {len(scored)}")
    print(f"frames w/ small:  {n_nonzero}")
    if n_nonzero == 0:
        print("ERROR: no frames have any small-object GT detections", file=sys.stderr)
        return 1

    target_n = args.n
    if n_nonzero < target_n:
        print(f"WARNING: only {n_nonzero} frames have a small-object det "
              f"(< requested {target_n}); reducing N to {n_nonzero}",
              flush=True)
        target_n = n_nonzero

    # Sort descending by count, then ascending by frame index for stability.
    nonzero.sort(key=lambda t: (-t[1], t[0]))
    chosen = nonzero[:target_n]
    counts = [c for _, c in chosen]
    print(f"selected:         {len(chosen)} frames")
    print(f"small-det count min/median/max: {min(counts)} / "
          f"{counts[len(counts) // 2]} / {max(counts)}")

    # Re-sort by frame index ascending so downstream video extraction is
    # chronological.
    chosen.sort(key=lambda t: t[0])
    chosen_idxs = [idx for idx, _ in chosen]
    print(f"frame index range: {chosen_idxs[0]} - {chosen_idxs[-1]}")

    out = {
        "src_h": args.src_h,
        "n_selected": len(chosen_idxs),
        "frame_indices": chosen_idxs,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w") as f:
        json.dump(out, f, indent=2)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
