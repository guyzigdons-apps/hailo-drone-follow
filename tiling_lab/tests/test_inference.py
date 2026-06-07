import numpy as np
from hailo_tiling.types import CropRect
from tiling_lab.harness.inference import ReplayBackend


class _CropLocalDet:
    def __init__(self, cls, x, y, w, h, score):
        self.cls, self.x, self.y, self.w, self.h, self.score = cls, x, y, w, h, score


def test_replay_backend_returns_canned_crop_local_dets():
    canned = {(0, (1000, 500, 640, 480)): [
        _CropLocalDet(cls=0, x=0.25, y=0.25, w=0.5, h=0.5, score=0.9)]}
    be = ReplayBackend(canned)
    crop = CropRect(x=1000, y=500, w=640, h=480)
    dets = be.infer(frame=np.zeros((3000, 4000, 3), np.uint8), crop=crop, frame_idx=0)
    assert len(dets) == 1 and dets[0].cls == 0


def test_replay_backend_empty_for_unknown_crop():
    be = ReplayBackend({})
    crop = CropRect(x=0, y=0, w=640, h=480)
    assert be.infer(frame=np.zeros((480, 640, 3), np.uint8), crop=crop, frame_idx=5) == []


class _FakeBatched:
    """Counts constructions/calls; returns one fixed det per crop."""
    constructed = 0

    def __init__(self):
        type(self).constructed += 1
        self.calls = 0

    def infer(self, frame, crops, frame_idx):
        from hailo_tiling.types import Det
        self.calls += 1
        return [[Det(cls=1, score=0.9, x=0.1, y=0.1, w=0.2, h=0.2)] for _ in crops]

    def close(self):
        pass


def _cached(tmp_path, name="c.sqlite3"):
    from tiling_lab.harness.inference import CachedHefBackend
    return CachedHefBackend(cache_path=tmp_path / name,
                            meta={"hef": "fake.hef", "nms": "0.25"},
                            make_backend=_FakeBatched)


def test_cached_backend_serves_repeat_from_cache(tmp_path):
    _FakeBatched.constructed = 0
    b = _cached(tmp_path)
    crop = CropRect(x=0, y=0, w=480, h=360)
    d1 = b.infer(None, crop, 7)
    d2 = b.infer(None, crop, 7)          # identical (frame, crop) -> cache hit
    b.close()
    assert [(d.cls, d.x) for d in d1] == [(1, 0.1)] == [(d.cls, d.x) for d in d2]
    assert _FakeBatched.constructed == 1  # chip backend built once, on first miss


def test_cached_backend_is_lazy_when_fully_warm(tmp_path):
    _FakeBatched.constructed = 0
    crop = CropRect(x=0, y=0, w=480, h=360)
    b = _cached(tmp_path)
    b.infer(None, crop, 7)
    b.close()
    b2 = _cached(tmp_path)               # reopen same cache
    d = b2.infer(None, crop, 7)
    b2.close()
    assert _FakeBatched.constructed == 1  # second session never touched the chip
    assert d[0].score == 0.9


def test_cached_backend_rejects_meta_mismatch(tmp_path):
    import pytest
    from tiling_lab.harness.inference import CachedHefBackend
    b = _cached(tmp_path)
    b.infer(None, CropRect(x=0, y=0, w=480, h=360), 0)
    b.close()
    with pytest.raises(ValueError, match="cache meta mismatch"):
        CachedHefBackend(cache_path=tmp_path / "c.sqlite3",
                         meta={"hef": "OTHER.hef", "nms": "0.25"},
                         make_backend=_FakeBatched)


def test_cached_backend_stats_delegates_to_caching_backend(tmp_path):
    _FakeBatched.constructed = 0
    b = _cached(tmp_path)
    crop = CropRect(x=0, y=0, w=480, h=360)
    b.infer(None, crop, 7)          # miss
    b.infer(None, crop, 7)          # hit
    s = b.stats
    assert s["misses"] == 1
    assert s["hits"] == 1
    assert s["chip_seconds"] >= 0.0
    assert "saved_seconds_estimate" in s
    b.close()


def test_cached_backend_saved_estimate_uses_per_miss_chip_time(tmp_path):
    _FakeBatched.constructed = 0
    b = _cached(tmp_path)
    crop = CropRect(x=0, y=0, w=480, h=360)
    b.infer(None, crop, 7)          # 1 miss
    b.infer(None, crop, 7)          # 1 hit
    s = b.stats
    # misses>0 -> saved ~= hits * (chip_seconds / misses)
    expected = s["hits"] * (s["chip_seconds"] / s["misses"])
    assert s["saved_seconds_estimate"] == expected
    b.close()


def test_cached_backend_saved_estimate_fallback_when_no_miss(tmp_path):
    """A fully-warm reopen forwards zero misses; the estimate falls back to the
    0.022 s/tile default so it is still non-zero."""
    _FakeBatched.constructed = 0
    crop = CropRect(x=0, y=0, w=480, h=360)
    b = _cached(tmp_path)
    b.infer(None, crop, 7)          # warm the cache
    b.close()
    b2 = _cached(tmp_path)
    b2.infer(None, crop, 7)         # hit only, never touches chip
    s = b2.stats
    assert s["misses"] == 0
    assert s["hits"] == 1
    assert s["chip_seconds"] == 0.0
    assert s["saved_seconds_estimate"] == 1 * 0.022
    b2.close()
