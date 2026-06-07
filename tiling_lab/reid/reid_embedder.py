"""Person-crop ReID embedding for the offline harness.

HARD RULE (user): ReID NEVER runs on the full frame and NEVER on non-person
classes — enforced here, the only embedding entry point.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from hailo_tiling.classes import PERSON
from hailo_tiling.types import CropRect

_FULL_FRAME_AREA = 0.9   # bbox covering >=90% of the frame is "full frame"
_MARGIN = 0.10           # crop margin around the bbox, fraction of bbox size


class ReidEmbedder:
    """embed(frame, person_det, frame_idx) -> L2-normalized float32 vector.

    `extractor` is a reid_analysis-style object (extract_embedding(crop_bgr),
    model_name, close()). Pass `cache_path` to memoize embeddings in the SQLite
    `embeddings` table (chip-free re-runs). The extractor may also be a zero-arg
    factory (lazy: never constructed when every lookup is a cache hit).
    """

    def __init__(self, *, extractor, cache_path: str | Path | None = None):
        self._make = extractor if callable(extractor) and not hasattr(extractor, "extract_embedding") else None
        self._extractor = None if self._make else extractor
        self._store = None
        if cache_path is not None:
            from hailo_tiling.cache.store import SqliteCacheStore
            Path(cache_path).parent.mkdir(parents=True, exist_ok=True)
            self._store = SqliteCacheStore.open(cache_path)
        self.stats = {"embeds": 0, "chip_embeds": 0}

    @property
    def model_name(self) -> str:
        if self._extractor is not None:
            return self._extractor.model_name
        return getattr(self._make, "model_name", "lazy")

    def _crop_rect(self, det, src_w: int, src_h: int) -> CropRect:
        mx, my = det.w * _MARGIN, det.h * _MARGIN
        x = max(0.0, det.x - mx)
        y = max(0.0, det.y - my)
        w = min(1.0 - x, det.w + 2 * mx)
        h = min(1.0 - y, det.h + 2 * my)
        return CropRect(x=int(round(x * src_w)), y=int(round(y * src_h)),
                        w=max(2, int(round(w * src_w))), h=max(2, int(round(h * src_h))))

    def embed(self, frame_bgr: np.ndarray, det, frame_idx: int) -> np.ndarray:
        if det.cls != PERSON:
            raise ValueError(f"ReID is person-only; got cls={det.cls}")
        if det.w * det.h >= _FULL_FRAME_AREA:
            raise ValueError("ReID must not run on the full frame")
        src_h, src_w = frame_bgr.shape[:2]
        rect = self._crop_rect(det, src_w, src_h)
        self.stats["embeds"] += 1
        if self._store is not None:
            hit = self._store.get_embedding(frame_idx, rect, model=self.model_name)
            if hit is not None:
                return hit
        if self._extractor is None:
            self._extractor = self._make()
        crop = frame_bgr[rect.y:rect.y + rect.h, rect.x:rect.x + rect.w]
        vec = self._extractor.extract_embedding(crop)
        self.stats["chip_embeds"] += 1
        if self._store is not None:
            self._store.put_embedding(frame_idx, rect, model=self.model_name, vec=vec)
        return vec

    def close(self) -> None:
        if self._extractor is not None:
            # The real HailoReIDExtractor exposes release(); fakes may expose
            # close(). Call whichever exists so we never AttributeError on teardown.
            closer = getattr(self._extractor, "release", None) or \
                getattr(self._extractor, "close", None)
            if closer is not None:
                closer()
        if self._store is not None:
            self._store.close()


def make_hef_embedder(hef_path: str, cache_path=None) -> ReidEmbedder:
    """Production-shaped embedder (lazy chip; import deferred so tests run chipless).

    Uses the genuine ``HailoReIDExtractor`` base class from
    ``reid_analysis.reid_embedding_extractor`` — it takes any reid ``hef_path``
    directly (repvgg_a0_person_reid_512 by default). The convenience subclasses
    (OSNetExtractor / RepVGG512Extractor) only add default HEF paths, which we
    supply explicitly here.
    """
    def factory():
        from reid_analysis.reid_embedding_extractor import HailoReIDExtractor
        return HailoReIDExtractor(hef_path=hef_path)
    factory.model_name = Path(hef_path).stem
    return ReidEmbedder(extractor=factory, cache_path=cache_path)
