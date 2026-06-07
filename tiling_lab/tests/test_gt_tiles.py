"""Tests for GT dense-tile reconstruction into overlay frames.json.

The viewer-debug feature renders the static GT dense-pass tile set (the
12x9 grid + extra grids + the center_vga_3x rect) onto every frame of an
overlay frames.json, using the SAME ``tiles`` schema that
``replay.emit_frames_json`` produces, so the viewer's existing tile
display/toggles work unchanged.
"""
import json

from tiling_lab.gt.gt_tiles import tiles_from_dense_config, load_dense_tiles
from tiling_lab.gt.run_gt_tracks import overlay_doc_by_id
from tiling_lab.gt.gt_clean import GtTrack


# The GT-12x9-25-multi dense config (mirrors dense_0013/pxt_GT-12x9-25-multi.json).
GT_12x9_25_MULTI = {
    "tiles_x": 12,
    "tiles_y": 9,
    "overlap_x": 0.25,
    "overlap_y": 0.25,
    "include_full_frame": False,
    "include_center_tile": False,
    "center_tile_size": 0.0,
    "extra_grids": [[1, 1, 0.0, 0.0], [3, 2, 0.25, 0.25]],
    "extra_rects": [[0.340426, 0.287234, 0.319149, 0.425532]],
}


def test_tile_count_116():
    """12x9 grid (108) + 1x1 (1) + 3x2 (6) + center_vga_3x rect (1) = 116.

    Matches the bench log line 'tiles count 116'.
    """
    tiles = tiles_from_dense_config(GT_12x9_25_MULTI)
    assert len(tiles) == 116


def test_tile_category_breakdown():
    tiles = tiles_from_dense_config(GT_12x9_25_MULTI)
    from collections import Counter
    cats = Counter(t["category"] for t in tiles)
    # 108 in the main dense grid; the rest (1 + 6 + 1) are rescue layers.
    assert cats["multi-scale"] == 108
    assert cats["single-scale"] == 8


def test_tile_dict_schema_matches_emit_frames_json():
    """Each tile must carry exactly the keys emit_frames_json writes:
    x, y, w, h, category — all floats except category (str)."""
    tiles = tiles_from_dense_config(GT_12x9_25_MULTI)
    for t in tiles:
        assert set(t.keys()) == {"x", "y", "w", "h", "category"}
        assert isinstance(t["x"], float)
        assert isinstance(t["y"], float)
        assert isinstance(t["w"], float)
        assert isinstance(t["h"], float)
        assert isinstance(t["category"], str)


def test_tiles_normalized_range():
    tiles = tiles_from_dense_config(GT_12x9_25_MULTI)
    for t in tiles:
        assert 0.0 <= t["x"] <= 1.0
        assert 0.0 <= t["y"] <= 1.0
        assert 0.0 < t["w"] <= 1.0
        assert 0.0 < t["h"] <= 1.0


def test_full_frame_extra_grid_present():
    """extra_grid (1,1,0,0) yields the full-frame rect."""
    tiles = tiles_from_dense_config(GT_12x9_25_MULTI)
    full = [t for t in tiles
            if t["x"] == 0.0 and t["y"] == 0.0 and t["w"] == 1.0 and t["h"] == 1.0]
    assert len(full) == 1


def test_center_vga_3x_rect_present():
    """The center_vga_3x extra_rect must appear verbatim."""
    tiles = tiles_from_dense_config(GT_12x9_25_MULTI)
    rect = [t for t in tiles
            if abs(t["x"] - 0.340426) < 1e-6 and abs(t["w"] - 0.319149) < 1e-6]
    assert len(rect) == 1


def test_empty_config_yields_no_tiles():
    assert tiles_from_dense_config({}) == []


def test_load_dense_tiles_from_json(tmp_path):
    dense = tmp_path / "pxt_GT-12x9-25-multi.json"
    dense.write_text(json.dumps({"label": "x", "config": GT_12x9_25_MULTI}))
    tiles = load_dense_tiles(dense)
    assert len(tiles) == 116


def _write_tracks_doc(path, tracks):
    from tiling_lab.gt.run_gt_tracks import tracks_to_doc
    path.write_text(json.dumps(tracks_to_doc(tracks, clip="clip")))


def test_run_gt_overlay_renders_only_overlays_and_keeps_protected(tmp_path):
    """render_overlays must write only overlay_*.frames.json and leave the
    protected (locked) GT files untouched."""
    import os
    from tiling_lab.gt.run_gt_overlay import render_overlays, PROTECTED

    tracks = [GtTrack(cls=1, track_id=7, frames={0: (0.1, 0.1, 0.2, 0.2)})]
    _write_tracks_doc(tmp_path / "gt_tracks.json", tracks)
    _write_tracks_doc(tmp_path / "gt_tracks.verified.json", tracks)

    # Locked provenance files (read-only).
    for name in ("corrections.json", "GT_STATUS.json"):
        p = tmp_path / name
        p.write_text('{"locked": true}')
        os.chmod(p, 0o444)
    protected_before = {n: (tmp_path / n).read_text()
                        for n in PROTECTED if (tmp_path / n).exists()}

    tiles = tiles_from_dense_config(GT_12x9_25_MULTI)
    written = render_overlays(tmp_path, tiles)

    names = {w.name for w in written}
    assert names == {"overlay_by_id.frames.json", "overlay_verified.frames.json"}
    # Protected files unchanged.
    for n, content in protected_before.items():
        assert (tmp_path / n).read_text() == content
    # Overlays carry the tiles.
    doc = json.loads((tmp_path / "overlay_verified.frames.json").read_text())
    assert all(len(fr["tiles"]) == 116 for fr in doc["frames"])


def test_run_gt_overlay_no_verified_only_by_id(tmp_path):
    from tiling_lab.gt.run_gt_overlay import render_overlays
    tracks = [GtTrack(cls=1, track_id=1, frames={0: (0.1, 0.1, 0.2, 0.2)})]
    _write_tracks_doc(tmp_path / "gt_tracks.json", tracks)
    written = render_overlays(tmp_path, None)
    assert {w.name for w in written} == {"overlay_by_id.frames.json"}


def test_overlay_doc_by_id_injects_tiles_every_frame():
    """overlay_doc_by_id with tiles= must put the same tile list on EVERY
    emitted frame, and frames with no tiles arg stay empty (backward compat)."""
    tracks = [GtTrack(cls=1, track_id=7,
                      frames={0: (0.1, 0.1, 0.2, 0.2),
                              3: (0.15, 0.1, 0.2, 0.2)})]
    tiles = tiles_from_dense_config(GT_12x9_25_MULTI)

    doc = overlay_doc_by_id(tracks, tiles=tiles)
    for fr in doc["frames"]:
        assert len(fr["tiles"]) == 116
        assert set(fr["tiles"][0].keys()) == {"x", "y", "w", "h", "category"}

    # Backward compat: default (no tiles) keeps empty list.
    doc0 = overlay_doc_by_id(tracks)
    for fr in doc0["frames"]:
        assert fr["tiles"] == []
