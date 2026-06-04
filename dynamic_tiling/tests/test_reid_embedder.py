import numpy as np
import pytest
from hailo_tiling.types import Det


class _FakeExtractor:
    model_name = "fake"

    def __init__(self):
        self.calls = 0

    def extract_embedding(self, crop_bgr):
        self.calls += 1
        v = np.ones(4, dtype=np.float32) * crop_bgr.mean()
        return v / np.linalg.norm(v)

    def close(self):
        pass


def _frame():
    return np.random.default_rng(0).integers(0, 255, (1080, 1920, 3), dtype=np.uint8)


def test_embed_person_crop_only():
    from dynamic_tiling.reid_embedder import ReidEmbedder
    e = ReidEmbedder(extractor=_FakeExtractor())
    person = Det(cls=1, score=0.9, x=0.5, y=0.5, w=0.05, h=0.12)
    v = e.embed(_frame(), person, frame_idx=0)
    assert v.shape == (4,) and abs(np.linalg.norm(v) - 1.0) < 1e-5


def test_vehicle_and_fullframe_rejected():
    from dynamic_tiling.reid_embedder import ReidEmbedder
    e = ReidEmbedder(extractor=_FakeExtractor())
    with pytest.raises(ValueError, match="person"):
        e.embed(_frame(), Det(cls=2, score=0.9, x=0.5, y=0.5, w=0.05, h=0.12), frame_idx=0)
    with pytest.raises(ValueError, match="full frame"):
        e.embed(_frame(), Det(cls=1, score=0.9, x=0.0, y=0.0, w=0.95, h=0.95), frame_idx=0)


def test_cached_embedder_serves_repeats_without_extractor(tmp_path):
    from dynamic_tiling.reid_embedder import ReidEmbedder
    fx = _FakeExtractor()
    e = ReidEmbedder(extractor=fx, cache_path=tmp_path / "c.sqlite3")
    person = Det(cls=1, score=0.9, x=0.5, y=0.5, w=0.05, h=0.12)
    f = _frame()
    v1 = e.embed(f, person, frame_idx=3)
    v2 = e.embed(f, person, frame_idx=3)
    assert fx.calls == 1                      # second call: cache hit
    assert np.allclose(v1, v2)
    assert e.stats["embeds"] == 2 and e.stats["chip_embeds"] == 1
