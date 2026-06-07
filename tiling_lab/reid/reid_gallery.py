"""Offline ReID gallery — ports drone_follow reid_manager semantics (read that file
for reference; DO NOT modify it). Bands on cosine sim vs current gallery max:
  sim < drift_threshold      -> "drift"     (not stored)
  sim > duplicate_threshold  -> "duplicate" (skipped; every refresh_every-th
                                             consecutive dup replaces the oldest -> "refreshed")
  otherwise                  -> "added"     (FIFO at capacity)
First min_gallery_for_drift_check samples bypass the drift gate. Optional EMA
anchor (StrongSORT-style): exponential moving average vector matched alongside
the gallery (match() returns the best of both)."""
from __future__ import annotations

import numpy as np


class ReidGallery:
    def __init__(self, *, size: int = 10, reid_threshold: float = 0.75,
                 drift_threshold: float = 0.6, duplicate_threshold: float = 0.9,
                 refresh_every: int = 5, min_gallery_for_drift_check: int = 6,
                 ema_alpha: float | None = None):
        self.size = size
        self.reid_threshold = reid_threshold
        self.drift_threshold = drift_threshold
        self.duplicate_threshold = duplicate_threshold
        self.refresh_every = refresh_every
        self.min_gallery_for_drift_check = min_gallery_for_drift_check
        self.ema_alpha = ema_alpha
        self._vecs: list[np.ndarray] = []
        self._ema: np.ndarray | None = None
        self._dup_streak = 0
        self._n_samples = 0

    def __len__(self) -> int:
        return len(self._vecs)

    def _sim(self, v: np.ndarray) -> float:
        best = max((float(np.dot(g, v)) for g in self._vecs), default=-1.0)
        if self._ema is not None:
            e = self._ema / np.linalg.norm(self._ema)
            best = max(best, float(np.dot(e, v)))
        return best

    def match(self, v: np.ndarray) -> float:
        """Best cosine similarity vs gallery (and EMA anchor if enabled); -1 if empty."""
        return self._sim(v)

    def _store(self, v: np.ndarray) -> None:
        if len(self._vecs) >= self.size:
            self._vecs.pop(0)
        self._vecs.append(v.astype(np.float32))
        if self.ema_alpha is not None:
            self._ema = v if self._ema is None else \
                (1 - self.ema_alpha) * self._ema + self.ema_alpha * v

    def add(self, v: np.ndarray) -> str:
        self._n_samples += 1
        if not self._vecs:
            self._store(v); self._dup_streak = 0; return "added"
        sim = self._sim(v)
        if sim > self.duplicate_threshold:
            self._dup_streak += 1
            if self._dup_streak % self.refresh_every == 0:
                self._vecs.pop(0); self._store(v); return "refreshed"
            return "duplicate"
        self._dup_streak = 0
        if sim < self.drift_threshold and self._n_samples > self.min_gallery_for_drift_check:
            return "drift"
        self._store(v)
        return "added"

    def clear(self) -> None:
        self._vecs.clear(); self._ema = None; self._dup_streak = 0; self._n_samples = 0
