"""Standalone scrubber: drop class-0 phantom detections from a frames.json.

Reads a frames.json produced by the tiling benchmark driver, applies the
combined phantom filter (exact tile-shape match OR area>=75% of a tile
centred on the tile centre), and writes a cleaned copy to
``<input>.clean.frames.json`` (or ``--output PATH``). Preserves all
other top-level fields (``label``, ``config``, ``video``, ``summary``).

Usage::

    python tiling_benchmark/clean_frames.py \\
        --input  tiling_benchmark/pxt_runs/pxt_GT-12x9-25-multi.frames.json
    # -> writes pxt_GT-12x9-25-multi.clean.frames.json next to the input

Use ``--in-place`` to overwrite the source file (destructive — keep a
backup if you might want the raw phantoms back).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# Reuse the phantom logic from analyze_pxt — same package dir, no
# __init__.py, so add the parent to sys.path for script-mode invocation.
HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
from analyze_pxt import (  # noqa: E402
    _compute_tile_rects,
    containment_merge,
    is_phantom,
)


def clean_frames_doc(doc: dict, tol: float) -> tuple[int, int]:
    """Mutate ``doc`` in place, dropping phantom detections per
    :func:`analyze_pxt.is_phantom` against the doc's own config's tile grid.
    Returns ``(dropped, original_total)``.
    """
    cfg = doc.get("config", {}) or {}
    tile_rects = _compute_tile_rects(cfg)
    if not tile_rects:
        total = sum(len(fr.get("detections") or []) for fr in doc.get("frames", []))
        return 0, total
    dropped = 0
    total = 0
    for fr in doc.get("frames", []):
        keep = []
        for det in fr.get("detections") or []:
            total += 1
            if is_phantom(det, tile_rects, tol):
                dropped += 1
                continue
            keep.append(det)
        fr["detections"] = keep
    return dropped, total


def derive_output_path(input_path: Path) -> Path:
    """Return ``<stem>.clean.frames.json`` next to ``input_path``.

    Handles the conventional ``.frames.json`` double-suffix used by the
    benchmark driver (stem is ``pxt_X.frames``); strips the trailing
    ``.frames`` component before adding ``.clean.frames.json``.
    """
    stem = input_path.stem
    if stem.endswith(".frames"):
        stem = stem[: -len(".frames")]
    return input_path.with_name(f"{stem}.clean.frames.json")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--input", type=Path, required=True,
                    help="Path to the source frames.json.")
    ap.add_argument("--output", type=Path, default=None,
                    help="Output path. Default: <input-stem>.clean.frames.json "
                         "next to the input.")
    ap.add_argument("--tile-shape-tol", type=float, default=0.01,
                    help="Normalized tolerance for the exact-tile-shape arm of "
                         "the phantom filter (default 0.01).")
    ap.add_argument("--containment-merge", action="store_true",
                    help="Also apply the containment-merge post-NMS filter: drop "
                         "any det whose centre is inside a larger same-class det "
                         "and whose area is < ratio * big_area. Opt-in to preserve "
                         "backwards compat with raw-NMS pipelines.")
    ap.add_argument("--containment-area-ratio", type=float, default=0.5,
                    help="Area-ratio ceiling for containment-merge (default 0.5).")
    ap.add_argument("--containment-center-slack", type=float, default=0.0,
                    help="Centre-containment slack in normalized coords for "
                         "containment-merge (default 0.0 = strict).")
    ap.add_argument("--in-place", action="store_true",
                    help="Overwrite the input file. Destructive — destroys "
                         "the raw detections.")
    args = ap.parse_args(argv)

    if not args.input.is_file():
        print(f"ERROR: input file not found: {args.input}", file=sys.stderr)
        return 1

    if args.in_place:
        if args.output is not None:
            print("ERROR: --in-place and --output are mutually exclusive",
                  file=sys.stderr)
            return 1
        out_path = args.input
    else:
        out_path = args.output if args.output is not None \
            else derive_output_path(args.input)

    with args.input.open() as f:
        doc = json.load(f)

    dropped, total = clean_frames_doc(doc, args.tile_shape_tol)
    pct = (100.0 * dropped / total) if total else 0.0
    print(f"[phantom] dropped {dropped} of {total} detections ({pct:.1f}%)")

    if args.containment_merge:
        cm_suppressed = 0
        cm_total = 0
        for fr in doc.get("frames", []):
            dets = fr.get("detections") or []
            cm_total += len(dets)
            kept = containment_merge(
                dets,
                area_ratio_max=args.containment_area_ratio,
                center_slack=args.containment_center_slack,
            )
            cm_suppressed += len(dets) - len(kept)
            fr["detections"] = kept
        cm_pct = (100.0 * cm_suppressed / cm_total) if cm_total else 0.0
        print(f"[containment-merge] suppressed {cm_suppressed} of "
              f"{cm_total} detections ({cm_pct:.1f}%)")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w") as f:
        json.dump(doc, f, indent=2)
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
