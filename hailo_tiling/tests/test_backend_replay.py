"""ReplayBackend — chip-free cache reader. Misses are loud."""
from __future__ import annotations

import struct
from pathlib import Path

import pytest

from hailo_tiling.backends import (
    ReplayBackend,
    map_dets_to_source,
    read_source_coord_detections,
)
from hailo_tiling.cache import CacheMissError
from hailo_tiling.cache.store import SqliteCacheStore
from hailo_tiling.types import CropRect, Det

# A GST-produced tile cache (Task 4/6 writer, source-width=3840
# source-height=2160) over a few frames of the fov50 drone clip. Real
# detections + the source-pixel crop keys + the meta envelope.
_GST_FIXTURE = (
    Path(__file__).resolve().parent / "fixtures" / "gst_tile_cache_fov50_small.sqlite3"
)


def _f32(v: float) -> float:
    """Round-trip a Python float through float32, as the cache effectively does
    (the HailoROI bbox/score are float32; the writer's %.9g and Python's
    shortest-repr both recover identical float32 bits)."""
    return struct.unpack("f", struct.pack("f", v))[0]


def _det(s: float = 0.9) -> Det:
    return Det(cls=0, score=s, x=0.1, y=0.1, w=0.1, h=0.1)


def _store(tmp_path: Path) -> SqliteCacheStore:
    return SqliteCacheStore.open(tmp_path / "r.sqlite3")


def test_all_hit_returns_dets_in_order(tmp_path):
    store = _store(tmp_path)
    crops = [CropRect(x=i * 100, y=0, w=640, h=480, mode="m") for i in range(3)]
    store.put_many([
        {"frame_idx": 0, "crop_rect": c, "ppv": 1, "dets": [_det(0.5 + 0.1 * i)]}
        for i, c in enumerate(crops)
    ])
    be = ReplayBackend(store=store, ppv=1)
    out = be.infer(frame=None, crops=crops, frame_idx=0)
    assert [d[0].score for d in out] == pytest.approx([0.5, 0.6, 0.7])
    store.close()


def test_miss_raises_cache_miss_error(tmp_path):
    store = _store(tmp_path)
    c = CropRect(x=0, y=0, w=640, h=480, mode="m")
    be = ReplayBackend(store=store, ppv=1)
    with pytest.raises(CacheMissError) as exc:
        be.infer(frame=None, crops=[c], frame_idx=42)
    msg = str(exc.value)
    assert "frame_idx=42" in msg
    assert "0,0,640,480" in msg or "(0, 0, 640, 480)" in msg
    store.close()


def test_partial_miss_still_raises_loudly(tmp_path):
    store = _store(tmp_path)
    crops = [CropRect(x=i * 100, y=0, w=640, h=480, mode="m") for i in range(3)]
    store.put_many([
        {"frame_idx": 0, "crop_rect": crops[0], "ppv": 1, "dets": [_det()]},
    ])
    be = ReplayBackend(store=store, ppv=1)
    with pytest.raises(CacheMissError):
        be.infer(frame=None, crops=crops, frame_idx=0)
    store.close()


def test_empty_crops_returns_empty(tmp_path):
    store = _store(tmp_path)
    be = ReplayBackend(store=store, ppv=1)
    assert be.infer(frame=None, crops=[], frame_idx=0) == []
    store.close()


def test_ppv_isolation(tmp_path):
    store = _store(tmp_path)
    c = CropRect(x=0, y=0, w=640, h=480, mode="m")
    store.put_many([{"frame_idx": 0, "crop_rect": c, "ppv": 1, "dets": [_det()]}])
    be = ReplayBackend(store=store, ppv=2)
    with pytest.raises(CacheMissError):
        be.infer(frame=None, crops=[c], frame_idx=0)
    store.close()


# ---------------------------------------------------------------------------
# Source-coord read path (offline postprocessing — R5)
# ---------------------------------------------------------------------------

def test_map_dets_to_source_pure():
    """The pure mapping reproduces the GST aggregator's de-tiling exactly."""
    # crop = right-half tile of a 3840x2160 frame: x=1920,y=0,w=1920,h=1080
    crop = CropRect(x=1920, y=0, w=1920, h=1080)
    # a tile-local-norm det at the tile centre, quarter-size
    local = [Det(cls=0, score=0.9, x=0.5, y=0.5, w=0.25, h=0.25)]
    out = map_dets_to_source(local, crop, 3840, 2160)
    assert len(out) == 1
    d = out[0]
    # src_x = (1920 + 0.5*1920)/3840 = 0.75 ; src_y = (0 + 0.5*1080)/2160 = 0.25
    # src_w = 0.25*1920/3840 = 0.125 ; src_h = 0.25*1080/2160 = 0.125
    assert (d.x, d.y, d.w, d.h) == pytest.approx((0.75, 0.25, 0.125, 0.125))
    assert d.cls == 0 and d.score == pytest.approx(0.9)


@pytest.mark.skipif(not _GST_FIXTURE.exists(), reason="GST fixture not committed")
def test_gst_cache_source_coords_value_exact():
    """Open a real GST-produced cache, pick a row with detections, map it by
    hand from the stored tile-local-norm dets + crop key + meta video_w/h, and
    assert the util reproduces it at float32 tolerance.

    Value-based comparison (NOT text equality): the C++ writer serializes with
    %.9g and Python uses shortest-repr, so the JSON text differs even though
    both recover identical float32 values. Hence the float32 tolerance.
    """
    store = SqliteCacheStore.open(_GST_FIXTURE)
    try:
        # The fixture must carry the source-resolution envelope and real dets.
        vw = int(store.meta_get("video_w"))
        vh = int(store.meta_get("video_h"))
        assert (vw, vh) == (3840, 2160)
        assert store.meta_get("resize_mode") == "stretch"

        # Find the first (frame_idx, crop) row that has at least one detection,
        # reading the raw stored fields so we can map by hand independently.
        raw = store._con.execute(
            "SELECT frame_idx, crop_x, crop_y, crop_w, crop_h, dets_json "
            "FROM detections WHERE dets_json != '[]' "
            "ORDER BY frame_idx, crop_x, crop_y LIMIT 1"
        ).fetchone()
        assert raw is not None, "fixture has no non-empty detection rows"
        frame_idx, cx, cy, cw, ch, dets_json = raw
        crop = CropRect(x=cx, y=cy, w=cw, h=ch)

        # Hand-map the stored tile-local-norm dets -> source-frame-norm.
        import json

        local = json.loads(dets_json)
        assert len(local) >= 1
        expected = []
        for o in local:
            ex = _f32((cx + float(o["x"]) * cw) / vw)
            ey = _f32((cy + float(o["y"]) * ch) / vh)
            ew = _f32(float(o["w"]) * cw / vw)
            eh = _f32(float(o["h"]) * ch / vh)
            expected.append((int(o["cls"]), _f32(float(o["score"])), ex, ey, ew, eh))

        # Pull the same (frame_idx, crop) tile out of the util's stream.
        got_tile = next(
            t
            for t in read_source_coord_detections(store, ppv=1)
            if t.frame_idx == frame_idx and t.crop == crop
        )
        assert len(got_tile.dets) == len(expected)
        f32eps = 1.2e-7  # ~ float32 machine epsilon
        for d, (ecls, escore, ex, ey, ew, eh) in zip(got_tile.dets, expected):
            assert d.cls == ecls
            assert d.score == pytest.approx(escore, abs=f32eps, rel=f32eps)
            assert d.x == pytest.approx(ex, abs=f32eps, rel=f32eps)
            assert d.y == pytest.approx(ey, abs=f32eps, rel=f32eps)
            assert d.w == pytest.approx(ew, abs=f32eps, rel=f32eps)
            assert d.h == pytest.approx(eh, abs=f32eps, rel=f32eps)

        # All mapped detections must lie within the source-normalized frame.
        for t in read_source_coord_detections(store, ppv=1):
            for d in t.dets:
                assert -1e-6 <= d.x <= 1.0 + 1e-6
                assert -1e-6 <= d.y <= 1.0 + 1e-6
    finally:
        store.close()


@pytest.mark.skipif(not _GST_FIXTURE.exists(), reason="GST fixture not committed")
def test_gst_cache_requires_video_dims(tmp_path):
    """A cache without the video_w/h envelope cannot be mapped to source coords."""
    store = _store(tmp_path)
    c = CropRect(x=0, y=0, w=1536, h=1080)
    store.put_many([{"frame_idx": 0, "crop_rect": c, "ppv": 1, "dets": [_det()]}])
    with pytest.raises(ValueError, match="video_w/video_h"):
        list(read_source_coord_detections(store, ppv=1))
    store.close()
