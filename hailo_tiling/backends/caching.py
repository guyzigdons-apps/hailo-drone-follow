"""CachingBackend — decorator that serves repeated `(frame_idx, crop)` lookups
from an SQLite cache.

Wrap any `InferenceBackend`; the cache is invisible to the scheduler and the
aggregator. On `infer(frame, crops, frame_idx)`:

  1. Canonicalise each crop (optional quantise).
  2. Split `crops` into hits (served from `store`) and misses (forwarded).
  3. Forward misses to the wrapped backend as ONE batched call.
  4. Persist new (crop, dets) rows in ONE transaction.
  5. Merge hits + new results back into the original input order.

Duplicate crops in the input collapse to one wrapped call; the result is
broadcast back to every input position.
"""
from __future__ import annotations

import time
from typing import Any, Sequence

from ..cache.hashing import canonicalize_crop
from ..cache.store import SqliteCacheStore
from ..types import CropRect, Det
from .backend import InferenceBackend


class CachingBackend(InferenceBackend):
    def __init__(
        self,
        wrapped: InferenceBackend,
        store: SqliteCacheStore,
        ppv: int,
        quantise: int | None = None,
    ):
        self._wrapped = wrapped
        self._store = store
        self._ppv = int(ppv)
        self._quantise = quantise
        # Per-instance cache-savings accounting. Counters are per-tile:
        #   hits   — tile positions served from the store (incl. dedup-collapsed
        #            duplicates, which consumed no chip)
        #   misses — unique tiles forwarded to the wrapped backend
        #   chip_seconds   — wall time inside wrapped.infer (perf_counter)
        #   lookup_seconds — wall time in store.get_many + put_many
        self.stats: dict[str, float] = {
            "hits": 0,
            "misses": 0,
            "chip_seconds": 0.0,
            "lookup_seconds": 0.0,
        }

    def _key(self, c: CropRect) -> tuple[int, int, int, int]:
        return canonicalize_crop(c, quantise=self._quantise)

    def _canon_crop(self, c: CropRect) -> CropRect:
        if self._quantise is None or self._quantise <= 1:
            return c
        x, y, w, h = self._key(c)
        return CropRect(x=x, y=y, w=w, h=h, mode=c.mode)

    def infer(
        self,
        frame: Any,
        crops: Sequence[CropRect],
        frame_idx: int,
    ) -> list[list[Det]]:
        if not crops:
            return []

        canon = [self._canon_crop(c) for c in crops]
        keys = [self._key(c) for c in canon]

        unique_keys: list[tuple] = []
        unique_crops: list[CropRect] = []
        key_to_unique_idx: dict[tuple, int] = {}
        for k, c in zip(keys, canon):
            if k not in key_to_unique_idx:
                key_to_unique_idx[k] = len(unique_keys)
                unique_keys.append(k)
                unique_crops.append(c)

        _t0 = time.perf_counter()
        cached = self._store.get_many(frame_idx, unique_crops, ppv=self._ppv)
        self.stats["lookup_seconds"] += time.perf_counter() - _t0
        miss_indices = [i for i, d in enumerate(cached) if d is None]
        miss_crops = [unique_crops[i] for i in miss_indices]

        # Per-tile accounting: misses are the UNIQUE forwarded tiles; hits are
        # every other input position (unique cache hits + dedup-collapsed
        # duplicates, which consumed no chip).
        n_misses = len(miss_crops)
        self.stats["misses"] += n_misses
        self.stats["hits"] += len(crops) - n_misses

        if miss_crops:
            _c0 = time.perf_counter()
            new_results = self._wrapped.infer(frame, miss_crops, frame_idx)
            self.stats["chip_seconds"] += time.perf_counter() - _c0
            assert len(new_results) == len(miss_crops), (
                "wrapped backend returned wrong number of det-lists"
            )
            _p0 = time.perf_counter()
            self._store.put_many([
                {
                    "frame_idx": frame_idx,
                    "crop_rect": miss_crops[i],
                    "ppv": self._ppv,
                    "dets": new_results[i],
                }
                for i in range(len(miss_crops))
            ])
            self.stats["lookup_seconds"] += time.perf_counter() - _p0
            for slot, dets in zip(miss_indices, new_results):
                cached[slot] = dets

        return [cached[key_to_unique_idx[k]] for k in keys]

    def close(self) -> None:
        self._wrapped.close()
