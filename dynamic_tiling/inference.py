"""Compatibility shim: backends live in hailo_tiling.backends.

`HefBackend` here is a **thin wrapper** around `hailo_tiling.backends.hef.HefBackend`
that exposes the legacy single-crop `infer(frame, crop, frame_idx) -> list` API
used by `dynamic_tiling.run_dynamic` and `dynamic_tiling.replay`. The wrapped
class uses the new batched `infer(frame, crops, frame_idx) -> list[list[Det]]`
API internally.

`ReplayBackend` is kept here as a legacy single-crop helper used by
`dynamic_tiling.tests.test_inference`. The Plan-1-style per-crop API
(infer(frame, crop, frame_idx)) is preserved exactly.

This shim disappears in Plan 8 (drone-follow migration) once all callers
move to hailo_tiling.
"""
from __future__ import annotations

from pathlib import Path
from typing import Protocol

from hailo_tiling.backends.hef import HefBackend as _BatchedHefBackend

from .types import CropRect


class InferenceBackend(Protocol):
    """Legacy single-crop protocol; preserved for dynamic_tiling callers.

    The batched ABC lives in `hailo_tiling.backends.backend.InferenceBackend`.
    """

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:  # noqa: D401
        ...


class HefBackend:
    """Single-crop adapter over the batched `hailo_tiling.backends.hef.HefBackend`.

    Forwards `infer(frame, crop, frame_idx)` to the underlying backend's
    `infer(frame, [crop], frame_idx)` and unwraps the single result list.
    Constructor args are passed through verbatim.
    """

    def __init__(self, *args, **kwargs):
        self._inner = _BatchedHefBackend(*args, **kwargs)

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:
        return self._inner.infer(frame, [crop], frame_idx)[0]

    def close(self) -> None:
        self._inner.close()


class _LazyBatched:
    """Constructs the wrapped batched backend on FIRST infer call only.

    With a fully-warmed cache, `CachingBackend` never forwards a miss, so the
    chip backend (and the Hailo device) is never opened at all."""

    def __init__(self, make_backend):
        self._make = make_backend
        self._inner = None

    def infer(self, frame, crops, frame_idx):
        if self._inner is None:
            self._inner = self._make()
        return self._inner.infer(frame, crops, frame_idx)

    def close(self) -> None:
        if self._inner is not None:
            self._inner.close()


class CachedHefBackend:
    """Single-crop HEF backend with an SQLite tile cache (Plan-4 cache layer).

    Composition: `CachingBackend(SqliteCacheStore)` over a lazily-constructed
    batched HEF backend. Repeated `(frame_idx, crop)` pairs are served from the
    cache; a fully-cached run needs no chip. `meta` (e.g. hef/nms/class_offset/
    video identity) is stamped into the cache on creation and verified on
    reopen — a mismatch raises rather than silently mixing conventions.
    Note: dets are cached CROP-LOCAL (this backend's convention) — do not share
    a DB with GstCropperBackend-warmed source-coord caches.
    """

    def __init__(self, *args, cache_path, meta=None, make_backend=None, ppv=1, **kwargs):
        from hailo_tiling.backends.caching import CachingBackend
        from hailo_tiling.cache.store import SqliteCacheStore

        Path(cache_path).parent.mkdir(parents=True, exist_ok=True)
        self._store = SqliteCacheStore.open(cache_path)
        for k, v in (meta or {}).items():
            cur = self._store.meta_get(k)
            if cur is None:
                self._store.meta_put(k, str(v))
            elif cur != str(v):
                self._store.close()
                raise ValueError(
                    f"cache meta mismatch for {k!r}: cache has {cur!r}, run has {v!r} "
                    f"({cache_path})")
        if make_backend is None:
            def make_backend():
                return _BatchedHefBackend(*args, **kwargs)
        self._inner = CachingBackend(wrapped=_LazyBatched(make_backend),
                                     store=self._store, ppv=ppv)

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:
        return self._inner.infer(frame, [crop], frame_idx)[0]

    @property
    def stats(self) -> dict:
        """Cache-savings accounting, delegated to the inner CachingBackend plus
        a `saved_seconds_estimate`. The estimate is `hits * (chip_seconds /
        misses)` when any tile actually hit the chip this run, else a 0.022 s/
        tile fallback (a fully-warm reopen forwards zero misses but still saved
        roughly that per served tile). Survives `close()` — the CachingBackend
        instance (and its stats dict) outlives the store handle."""
        s = dict(self._inner.stats)
        hits = s["hits"]
        misses = s["misses"]
        per_miss = (s["chip_seconds"] / misses) if misses > 0 else 0.022
        s["saved_seconds_estimate"] = hits * per_miss
        return s

    def close(self) -> None:
        self._inner.close()
        self._store.close()


class ReplayBackend:
    """Legacy deterministic backend keyed on (frame_idx, (x, y, w, h)).

    Single-crop API. The batched ReplayBackend (`hailo_tiling.backends`) lands
    in Plan 4 alongside the SQLite cache layer.
    """

    def __init__(self, canned: dict):
        self._canned = canned

    def infer(self, frame, crop: CropRect, frame_idx: int) -> list:
        return list(self._canned.get((frame_idx, (crop.x, crop.y, crop.w, crop.h)), []))
