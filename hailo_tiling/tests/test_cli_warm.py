"""hailo-tiling-warm-cache CLI — end-to-end on a tiny canned video."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from hailo_tiling.backends import MockBackend
from hailo_tiling.cache.store import SqliteCacheStore
from hailo_tiling.cli import warm
from hailo_tiling.types import CropRect, Det


def _det(score: float = 0.9) -> Det:
    return Det(cls=0, score=score, x=0.2, y=0.3, w=0.1, h=0.2)


def test_parse_grids_accepts_NxM():
    assert warm.parse_grids(["1x1", "2x2", "3x2"]) == [(1, 1), (2, 2), (3, 2)]


def test_parse_grids_rejects_garbage():
    with pytest.raises(SystemExit):
        warm.parse_grids(["1"])
    with pytest.raises(SystemExit):
        warm.parse_grids(["axb"])


def test_grid_crops_for_dims_matches_discovery_emitter():
    from hailo_tiling.emitters import DiscoveryGridEmitter
    from hailo_tiling.budget import BudgetMeter
    from hailo_tiling.types import LockState

    crops = warm.crops_for_grids(src_w=3840, src_h=2160, grids=[(3, 2)])
    e = DiscoveryGridEmitter(grid=(3, 2), period=1, mode="m")
    expected = e.emit(3840, 2160, LockState(), frame_idx=0,
                       meter=BudgetMeter(budget_inf_per_s=1e9, fps=30))
    assert crops == expected


def test_warm_video_writes_one_row_per_crop_per_frame(tmp_path):
    db = tmp_path / "w.sqlite3"
    store = SqliteCacheStore.open(db)
    crops = warm.crops_for_grids(src_w=3840, src_h=2160, grids=[(2, 1)])
    frames = [(i, None) for i in range(3)]
    backend = MockBackend(canned={
        (i, (c.x, c.y, c.w, c.h)): [_det()] for i in range(3) for c in crops
    })
    warm.warm_video(frames=frames, crops=crops, backend=backend,
                     store=store, ppv=1, batch_rows=8)
    assert store.stats()["n_rows"] == 3 * len(crops)
    store.close()


def test_warm_video_skips_already_cached(tmp_path):
    db = tmp_path / "w2.sqlite3"
    store = SqliteCacheStore.open(db)
    crops = warm.crops_for_grids(src_w=3840, src_h=2160, grids=[(2, 1)])
    canned = {(i, (c.x, c.y, c.w, c.h)): [_det()] for i in range(2) for c in crops}
    backend = MockBackend(canned=canned)
    frames = [(i, None) for i in range(2)]
    warm.warm_video(frames=frames, crops=crops, backend=backend,
                     store=store, ppv=1, batch_rows=8)
    n_calls_first = backend.call_count
    warm.warm_video(frames=frames, crops=crops, backend=backend,
                     store=store, ppv=1, batch_rows=8)
    assert backend.call_count == n_calls_first
    store.close()


def test_main_argparse_round_trip(tmp_path, monkeypatch):
    with pytest.raises(SystemExit) as exc:
        warm.main(["--help"])
    assert exc.value.code == 0


def test_main_writes_meta_table(tmp_path, monkeypatch):
    vid = tmp_path / "v.mp4"
    vid.write_bytes(b"\x00fake-mp4\x00" * 1024)
    hef = tmp_path / "y.hef"
    hef.write_bytes(b"\x00fake-hef\x00" * 1024)
    cache_dir = tmp_path / ".cache"

    def fake_frames(path):
        return [(0, None), (1, None)]

    monkeypatch.setattr(warm, "iter_video_frames", fake_frames)
    monkeypatch.setattr(
        warm, "make_backend",
        lambda kind, **kw: MockBackend(canned={
            (i, (c.x, c.y, c.w, c.h)): [_det()]
            for i in range(2)
            for c in warm.crops_for_grids(3840, 2160, [(1, 1)])
        }),
    )

    rc = warm.main([
        "--video", str(vid),
        "--hef", str(hef),
        "--src-w", "3840", "--src-h", "2160",
        "--grid", "1x1",
        "--cache-dir", str(cache_dir),
        "--ppv", "1",
        "--backend", "mock",
    ])
    assert rc == 0
    matches = list(cache_dir.glob("*.sqlite3"))
    assert len(matches) == 1
    store = SqliteCacheStore.open(matches[0])
    try:
        assert store.meta_get("video_sha256") is not None
        assert store.meta_get("hef_sha256") is not None
        assert store.meta_get("ppv") == "1"
        assert int(store.meta_get("video_w") or 0) == 3840
        assert int(store.meta_get("video_h") or 0) == 2160
        assert store.stats()["n_rows"] == 2 * 1
    finally:
        store.close()
