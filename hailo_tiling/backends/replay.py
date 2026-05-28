"""ReplayBackend — chip-free cache reader.

For chip-free reruns over a published cache (CI, external reviewers). A
cache miss is a loud error — the spec (§7.6) is explicit: "Mixing live
inference into a replay path would muddy paper results; if the cache is
incomplete, the user re-runs warming."
"""
from __future__ import annotations

from typing import Any, Sequence

from ..cache.store import SqliteCacheStore
from ..types import CropRect, Det
from .backend import InferenceBackend


class CacheMissError(LookupError):
    """Raised by ReplayBackend when a requested crop is not in the cache."""


class ReplayBackend(InferenceBackend):
    def __init__(self, store: SqliteCacheStore, ppv: int = 1):
        self._store = store
        self._ppv = int(ppv)

    def infer(
        self,
        frame: Any,
        crops: Sequence[CropRect],
        frame_idx: int,
    ) -> list[list[Det]]:
        if not crops:
            return []
        results = self._store.get_many(frame_idx, list(crops), ppv=self._ppv)
        for c, dets in zip(crops, results):
            if dets is None:
                raise CacheMissError(
                    f"cache miss: frame_idx={frame_idx} "
                    f"crop=({c.x},{c.y},{c.w},{c.h}) ppv={self._ppv}. "
                    f"Re-warm the cache or use a live backend."
                )
        return results  # type: ignore[return-value]

    def close(self) -> None:
        return None
