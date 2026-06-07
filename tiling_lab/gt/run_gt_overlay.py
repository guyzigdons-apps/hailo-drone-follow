"""CLI: re-render GT overlay frames.json for an EXISTING gt_verify dir.

Render-only. Does NO tracking, NO finalize, NO GT mutation. It reads the
existing track files and re-emits the viewer overlay frames.json files,
optionally drawing the static GT dense-pass tile set on every frame
(``--tiles``) as a viewer debugging aid (e.g. the vga3-rect-boundary
bbox-jump issue).

It only ever writes ``overlay_by_id.frames.json`` and
``overlay_verified.frames.json`` (regenerable render artifacts). It will
refuse to touch the locked / verified GT files (``gt_tracks*.json``,
``corrections.json``, ``GT_STATUS.json``) — and asserts those stay intact.

    python -m tiling_lab.gt.run_gt_overlay \
        --outdir tiling_lab/runs/gt_verify_0013_default \
        --dense tiling_lab/runs/dense_0013/pxt_GT-12x9-25-multi.json \
        --tiles
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .run_gt_tracks import doc_to_tracks, overlay_doc_by_id
from .gt_tiles import load_dense_tiles

# Files this CLI must never write (locked / verified GT provenance).
PROTECTED = ("gt_tracks.json", "gt_tracks.verified.json",
             "corrections.json", "GT_STATUS.json", "review_queue.json")

# The only outputs this CLI is allowed to (re-)write.
OVERLAY_OUTPUTS = ("overlay_by_id.frames.json", "overlay_verified.frames.json")


def render_overlays(outdir: Path, tiles: list[dict] | None) -> list[Path]:
    """Re-emit overlay frames.json from the existing track files in ``outdir``.

    Renders ``overlay_by_id.frames.json`` from ``gt_tracks.json`` and, when
    present, ``overlay_verified.frames.json`` from ``gt_tracks.verified.json``.
    Returns the list of files written.
    """
    written: list[Path] = []

    by_id_src = outdir / "gt_tracks.json"
    if by_id_src.exists():
        tracks = doc_to_tracks(json.loads(by_id_src.read_text()))
        out = outdir / "overlay_by_id.frames.json"
        out.write_text(json.dumps(overlay_doc_by_id(tracks, tiles=tiles)))
        written.append(out)

    verified_src = outdir / "gt_tracks.verified.json"
    if verified_src.exists():
        tracks = doc_to_tracks(json.loads(verified_src.read_text()))
        out = outdir / "overlay_verified.frames.json"
        out.write_text(json.dumps(
            overlay_doc_by_id(tracks, label=outdir.name, tiles=tiles)))
        written.append(out)

    return written


def _protected_snapshot(outdir: Path) -> dict[str, float]:
    """mtime snapshot of protected files, for a post-write integrity assert."""
    return {name: (outdir / name).stat().st_mtime
            for name in PROTECTED if (outdir / name).exists()}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--outdir", required=True, type=Path,
                    help="existing gt_verify_* dir")
    ap.add_argument("--dense", type=Path, default=None,
                    help="dense run .json (config block) for the tile set; "
                         "required with --tiles")
    ap.add_argument("--tiles", action="store_true",
                    help="draw the static GT dense-pass tile set on every frame")
    args = ap.parse_args()

    outdir: Path = args.outdir
    if not outdir.is_dir():
        ap.error(f"--outdir not a directory: {outdir}")

    tiles = None
    if args.tiles:
        if args.dense is None:
            ap.error("--tiles requires --dense <dense .json>")
        tiles = load_dense_tiles(args.dense)
        print(f"tiles from {args.dense}: {len(tiles)}")

    before = _protected_snapshot(outdir)
    written = render_overlays(outdir, tiles)
    after = _protected_snapshot(outdir)

    # Integrity guard: protected files must be byte/time-identical.
    assert before == after, (
        f"protected GT file changed during overlay render! "
        f"before={before} after={after}")

    if not written:
        print(f"no track files found in {outdir}; nothing rendered")
    for w in written:
        print(f"wrote {w} ({len(tiles) if tiles else 0} tiles/frame)")


if __name__ == "__main__":
    main()
