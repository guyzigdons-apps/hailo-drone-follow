# Block R — ReID Recovery + Inference-Budget Ablations Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add ReID-based recovery to the offline harness (person crops ONLY), establish the generous upper bound, then implement+ablate inference-reduction policies (incl. a no-ReID arm), producing a quality-vs-embedding-fraction Pareto table.

**Architecture:** `ReidEmbedder` wraps the existing `reid_analysis` HailoRT extractor; an SQLite `embeddings` table makes re-runs chip-free; `ReidGallery` ports prod gallery semantics; a `ReidAssist` object plugs into `replay.run` (keeps frames OUT of `TargetLock`); policies are small strategy classes; `run_reid_ablation.py` drives arms × clips and writes `REID_ABLATION.md`.

**Tech Stack:** Python, pytest, HailoRT (`reid_analysis.reid_embedding_extractor`), SQLite cache layer (`hailo_tiling.cache`). Spec: `docs/superpowers/specs/2026-06-04-weekend-recovery-reid-sweep-mot-design.md`. Research: `docs/research/2026-06-04-reid-inference-reduction-survey.md`.

**Hard constraints (user-mandated, enforce in code):**
- ReID embeds **person bbox crops only** — the API takes an explicit bbox + class and `raise`s on non-person class or on a bbox covering ≥ 90% of the frame. No full-frame path exists.
- Vehicles never reach the embedder (filter by class at every call site).
- Every ablation includes P0 (no ReID at all).

**Context for the implementer:**
- Extractor: `reid_analysis/reid_embedding_extractor.py` — build with
  `hef_path`; `extract_embedding(crop_bgr) -> 1-D L2-normalized float32`. Uses a SHARED
  VDevice group (ROUND_ROBIN) so it coexists with the detection HEF. HEFs on this
  machine: `/usr/local/hailo/resources/models/hailo10h/repvgg_a0_person_reid_512.hef`
  (prod default) and `osnet_x1_0.hef`. Use repvgg_a0_512.
- Prod gallery semantics to port: `drone_follow/pipeline_adapter/reid_manager.py`
  (READ-ONLY reference — never modify it; it has pre-existing uncommitted changes).
  Defaults: reid_threshold 0.75, drift 0.6, duplicate 0.9, refresh_every 5,
  update_interval 30, min_gallery_for_drift_check 6.
- Block 1 must be DONE first (`reacq_motion` / `reacq_radius_growth` exist; best
  config from `BASELINE.md` validation section is the motion base for all arms).
- Tests: `source hailo-apps/venv_hailo_apps/bin/activate && python -m pytest dynamic_tiling/tests/ -q`
- Git: branch `tiling-benchmark`, never push; commit per task; result JSONs untracked.

---

### Task 1: Embedding cache table in the SQLite store

**Files:**
- Modify: `hailo_tiling/cache/schema.sql`, `hailo_tiling/cache/store.py`
- Test: `hailo_tiling/tests/test_cache_store.py`

- [ ] **Step 1: Failing test** (append to `hailo_tiling/tests/test_cache_store.py`,
follow its existing `SqliteCacheStore.open(tmp_path / ...)` pattern):

```python
def test_embedding_roundtrip(tmp_path):
    import numpy as np
    from hailo_tiling.cache.store import SqliteCacheStore
    from hailo_tiling.types import CropRect
    s = SqliteCacheStore.open(tmp_path / "c.sqlite3")
    crop = CropRect(x=10, y=20, w=64, h=128)
    vec = np.arange(8, dtype=np.float32) / 10.0
    assert s.get_embedding(5, crop, model="repvgg") is None
    s.put_embedding(5, crop, model="repvgg", vec=vec)
    got = s.get_embedding(5, crop, model="repvgg")
    assert got is not None and got.dtype == np.float32
    assert np.allclose(got, vec)
    assert s.get_embedding(5, crop, model="osnet") is None   # model-keyed
    s.close()
```

- [ ] **Step 2: Run, expect FAIL** (`AttributeError: get_embedding`).

- [ ] **Step 3: Implement** — append to `schema.sql`:

```sql
CREATE TABLE IF NOT EXISTS embeddings (
    frame_idx INTEGER NOT NULL,
    crop_x    INTEGER NOT NULL,
    crop_y    INTEGER NOT NULL,
    crop_w    INTEGER NOT NULL,
    crop_h    INTEGER NOT NULL,
    model     TEXT    NOT NULL,
    vec       BLOB    NOT NULL,
    ts_epoch  REAL    NOT NULL,
    PRIMARY KEY (frame_idx, crop_x, crop_y, crop_w, crop_h, model)
) WITHOUT ROWID;
```

Add to `SqliteCacheStore` (numpy import goes inside the methods or as a module import —
store.py already operates on plain types; keep `import numpy as np` at module level):

```python
def put_embedding(self, frame_idx: int, crop_rect, *, model: str, vec) -> None:
    import time as _t
    self._con.execute(
        "INSERT OR IGNORE INTO embeddings "
        "(frame_idx, crop_x, crop_y, crop_w, crop_h, model, vec, ts_epoch) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
        (int(frame_idx), int(crop_rect.x), int(crop_rect.y),
         int(crop_rect.w), int(crop_rect.h), model,
         np.asarray(vec, dtype=np.float32).tobytes(), _t.time()))
    self._con.commit()

def get_embedding(self, frame_idx: int, crop_rect, *, model: str):
    row = self._con.execute(
        "SELECT vec FROM embeddings WHERE frame_idx=? AND crop_x=? AND crop_y=? "
        "AND crop_w=? AND crop_h=? AND model=?",
        (int(frame_idx), int(crop_rect.x), int(crop_rect.y),
         int(crop_rect.w), int(crop_rect.h), model)).fetchone()
    return None if row is None else np.frombuffer(row[0], dtype=np.float32).copy()
```

(`CREATE TABLE IF NOT EXISTS` makes existing tile-cache DBs upgrade in place on open.)

- [ ] **Step 4: Run hailo_tiling + dynamic_tiling suites, expect PASS.**
- [ ] **Step 5: Commit** — `feat(cache): embeddings table + get/put_embedding`

---

### Task 2: ReidEmbedder (person-crop-only) + cached variant

**Files:**
- Create: `dynamic_tiling/reid_embedder.py`
- Test: `dynamic_tiling/tests/test_reid_embedder.py`

- [ ] **Step 1: Failing tests**

```python
import numpy as np
import pytest
from hailo_tiling.types import Det


class _FakeExtractor:
    model_name = "fake"
    def __init__(self): self.calls = 0
    def extract_embedding(self, crop_bgr):
        self.calls += 1
        v = np.ones(4, dtype=np.float32) * crop_bgr.mean()
        return v / np.linalg.norm(v)
    def close(self): pass


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
```

- [ ] **Step 2: Run, expect FAIL** (module missing).

- [ ] **Step 3: Implement** `dynamic_tiling/reid_embedder.py`:

```python
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
        x = max(0.0, det.x - mx); y = max(0.0, det.y - my)
        w = min(1.0 - x, det.w + 2 * mx); h = min(1.0 - y, det.h + 2 * my)
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
            self._extractor.close()
        if self._store is not None:
            self._store.close()


def make_hef_embedder(hef_path: str, cache_path=None) -> ReidEmbedder:
    """Production-shaped embedder (lazy chip; import deferred so tests run chipless)."""
    def factory():
        from reid_analysis.reid_embedding_extractor import OSNetExtractor
        return OSNetExtractor(hef_path=hef_path)
    factory.model_name = Path(hef_path).stem
    return ReidEmbedder(extractor=factory, cache_path=cache_path)
```

(Check the actual extractor class name exported by
`reid_analysis/reid_embedding_extractor.py` — module docstring shows `OSNetExtractor`;
it loads ANY reid HEF including repvgg. If a more general base class exists, use it.)

- [ ] **Step 4: Run tests, expect PASS.** (note `model_name` mismatch trap: cached
lookups before construction use `factory.model_name` — the test's fake covers the
eager path; add nothing more.)
- [ ] **Step 5: Commit** — `feat(reid): person-crop-only ReidEmbedder with embedding cache`

---

### Task 3: ReidGallery (prod-semantics port + EMA option)

**Files:**
- Create: `dynamic_tiling/reid_gallery.py`
- Test: `dynamic_tiling/tests/test_reid_gallery.py`

- [ ] **Step 1: Failing tests**

```python
import numpy as np


def _v(*xs):
    a = np.array(xs, dtype=np.float32)
    return a / np.linalg.norm(a)


def test_gallery_add_bands_and_match():
    from dynamic_tiling.reid_gallery import ReidGallery
    g = ReidGallery(size=3, drift_threshold=0.6, duplicate_threshold=0.9,
                    refresh_every=2, min_gallery_for_drift_check=1)
    assert g.add(_v(1, 0, 0)) == "added"            # seed
    assert g.add(_v(0.8, 0.6, 0)) == "added"        # sim ~0.8: enrichment band
    assert g.add(_v(1, 0.01, 0)) == "duplicate"     # sim ~1.0: dup band, skipped
    assert g.add(_v(1, 0, 0.01)) == "refreshed"     # 2nd consecutive dup -> refresh
    assert g.add(_v(0, 0, 1)) == "drift"            # sim ~0 vs gallery: NOT stored
    assert g.match(_v(1, 0, 0)) > 0.9
    assert g.match(_v(0, 1, 0)) < 0.9


def test_fifo_replacement_at_capacity():
    from dynamic_tiling.reid_gallery import ReidGallery
    g = ReidGallery(size=2, drift_threshold=0.0, duplicate_threshold=0.99,
                    min_gallery_for_drift_check=99)
    g.add(_v(1, 0, 0)); g.add(_v(0.7, 0.7, 0)); g.add(_v(0, 1, 0))
    assert len(g) == 2                              # oldest evicted


def test_ema_anchor():
    from dynamic_tiling.reid_gallery import ReidGallery
    g = ReidGallery(size=4, ema_alpha=0.5, min_gallery_for_drift_check=99,
                    drift_threshold=0.0, duplicate_threshold=0.999)
    g.add(_v(1, 0, 0)); g.add(_v(0, 1, 0))
    m = g.match(_v(1, 1, 0))
    assert m > 0.9                                   # EMA anchor sits between the two
```

- [ ] **Step 2: Run, expect FAIL.**

- [ ] **Step 3: Implement** `dynamic_tiling/reid_gallery.py`:

```python
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
```

- [ ] **Step 4: Run, expect PASS.** Tune test vectors if a band boundary is off-by-epsilon — fix the TEST values, not by weakening the semantics.
- [ ] **Step 5: Commit** — `feat(reid): offline ReidGallery (prod band semantics + EMA anchor option)`

---

### Task 4: Policies + ReidAssist integration into replay

**Files:**
- Create: `dynamic_tiling/reid_policy.py`
- Modify: `dynamic_tiling/replay.py` (run() takes optional `reid_assist`), `dynamic_tiling/target_lock.py` (public `adopt()`)
- Test: `dynamic_tiling/tests/test_reid_policy.py`

- [ ] **Step 1: Failing tests** (policy selection logic is pure — test without chip):

```python
import numpy as np
from hailo_tiling.types import Det


class _Trk:
    def __init__(self, tid, tlwh, activated=True):
        self.track_id = tid; self.filtered_tlwh = tlwh; self.is_activated = activated


def _d(x, y=0.5, w=0.04, h=0.10, score=0.9, cls=1):
    return Det(cls=cls, score=score, x=x, y=y, w=w, h=h)


def test_generous_embeds_everything_lost():
    from dynamic_tiling.reid_policy import GenerousPolicy
    p = GenerousPolicy()
    dets = [_d(0.1), _d(0.5), _d(0.9, cls=2)]      # vehicle must be excluded
    out = p.candidates(frame_idx=10, person_dets=dets[:2], tracks=[], frames_lost=5)
    assert out == dets[:2]
    assert p.should_sample_tracked(frame_idx=7)     # every frame


def test_ambiguity_policy_skips_clean_continuations():
    from dynamic_tiling.reid_policy import AmbiguityPolicy
    p = AmbiguityPolicy(iou_thr=0.5, min_score=0.4)
    clean = _d(0.10)                                # exactly one track overlaps -> skip
    contested = _d(0.50)                            # two tracks overlap -> embed
    orphan = _d(0.80)                               # zero tracks overlap -> embed
    lowconf = _d(0.30, score=0.2)                   # low score -> never embed
    tracks = [_Trk(1, (0.10, 0.5, 0.04, 0.10)),
              _Trk(2, (0.50, 0.5, 0.04, 0.10)), _Trk(3, (0.505, 0.5, 0.04, 0.10))]
    out = p.candidates(frame_idx=0, person_dets=[clean, contested, orphan, lowconf],
                       tracks=tracks, frames_lost=3)
    assert clean not in out and lowconf not in out
    assert contested in out and orphan in out


def test_motion_gated_policy_filters_by_radius_and_decays():
    from dynamic_tiling.reid_policy import MotionGatedPolicy
    p = MotionGatedPolicy(radius_growth=0.002)
    near, far = _d(0.32), _d(0.90)
    out = p.candidates(frame_idx=0, person_dets=[near, far], tracks=[],
                       frames_lost=10, anchor=(0.30, 0.5, 0.04, 0.10))
    assert near in out and far not in out
    # cadence decay: after 30 lost frames only every 5th frame embeds
    assert p.candidates(frame_idx=0, person_dets=[near], tracks=[],
                        frames_lost=31, anchor=(0.30, 0.5, 0.04, 0.10)) == []
    assert p.candidates(frame_idx=0, person_dets=[near], tracks=[],
                        frames_lost=35, anchor=(0.30, 0.5, 0.04, 0.10)) == [near]
```

- [ ] **Step 2: Run, expect FAIL.**

- [ ] **Step 3: Implement** `dynamic_tiling/reid_policy.py`:

```python
"""ReID inference policies — decide WHICH person dets get embedded and WHEN the
tracked target is sampled into the gallery. Pure logic, no chip access.
Arms (spec): P1 generous, P2 prod-style, P3 ambiguity-gated, P4 motion-gated+decay,
P5 histogram pre-filter (separate task)."""
from __future__ import annotations


def _iou(a, b):
    ax1, ay1, ax2, ay2 = a[0], a[1], a[0] + a[2], a[1] + a[3]
    bx1, by1, bx2, by2 = b[0], b[1], b[0] + b[2], b[1] + b[3]
    iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0.0, min(ay2, by2) - max(ay1, by1))
    i = iw * ih
    return i / (a[2] * a[3] + b[2] * b[3] - i) if i > 0 else 0.0


class GenerousPolicy:
    """P1 — upper bound: sample every frame, embed every visible person when lost."""
    name = "generous"

    def should_sample_tracked(self, frame_idx: int) -> bool:
        return True

    def candidates(self, *, frame_idx, person_dets, tracks, frames_lost, anchor=None):
        return list(person_dets)


class ProdPolicy:
    """P2 — prod-style: gallery sampling every update_interval frames; embed all
    visible persons while lost (drift/dup gating happens inside the gallery)."""
    name = "prod"

    def __init__(self, update_interval: int = 30):
        self.update_interval = update_interval

    def should_sample_tracked(self, frame_idx: int) -> bool:
        return frame_idx % self.update_interval == 0

    def candidates(self, *, frame_idx, person_dets, tracks, frames_lost, anchor=None):
        return list(person_dets)


class AmbiguityPolicy(ProdPolicy):
    """P3 — risk test (arXiv 2409.06617): skip dets with exactly ONE overlapping
    activated track (clean continuation); embed contested (>=2) or orphan (0) dets;
    never embed low-confidence boxes (their embeddings are noise)."""
    name = "ambiguity"

    def __init__(self, update_interval: int = 30, iou_thr: float = 0.5, min_score: float = 0.4):
        super().__init__(update_interval)
        self.iou_thr = iou_thr
        self.min_score = min_score

    def candidates(self, *, frame_idx, person_dets, tracks, frames_lost, anchor=None):
        act = [t.filtered_tlwh for t in tracks if t.is_activated and t.filtered_tlwh]
        out = []
        for d in person_dets:
            if d.score < self.min_score:
                continue
            n = sum(1 for tl in act if _iou((d.x, d.y, d.w, d.h), tl) > self.iou_thr)
            if n != 1:
                out.append(d)
        return out


class MotionGatedPolicy(ProdPolicy):
    """P4 — embed only candidates inside the (growing) motion gate; cadence decay:
    every frame for the first 30 lost frames, every 5th until 150, every 15th after."""
    name = "motion"

    def __init__(self, update_interval: int = 30, radius_growth: float = 0.002, r0: float = 0.12):
        super().__init__(update_interval)
        self.radius_growth = radius_growth
        self.r0 = r0

    def _cadence_ok(self, frames_lost: int) -> bool:
        if frames_lost <= 30:
            return True
        if frames_lost <= 150:
            return frames_lost % 5 == 0
        return frames_lost % 15 == 0

    def candidates(self, *, frame_idx, person_dets, tracks, frames_lost, anchor=None):
        if anchor is None or not self._cadence_ok(frames_lost):
            return []
        acx, acy = anchor[0] + anchor[2] / 2, anchor[1] + anchor[3] / 2
        r = max(self.r0, max(anchor[2], anchor[3])) + self.radius_growth * frames_lost
        return [d for d in person_dets
                if ((d.x + d.w / 2 - acx) ** 2 + (d.y + d.h / 2 - acy) ** 2) ** 0.5 <= r]


POLICIES = {p.name: p for p in (GenerousPolicy, ProdPolicy, AmbiguityPolicy, MotionGatedPolicy)}
```

(Adjust the `MotionGatedPolicy` test/impl boundary so `frames_lost=10` with anchor at
0.30 and det at 0.32 passes: r >= r0=0.12 — it does.)

- [ ] **Step 4: ReidAssist + replay integration** — add to `dynamic_tiling/reid_policy.py`:

```python
class ReidAssist:
    """Owns embedder+gallery+policy; called from replay.run (frames stay out of
    TargetLock). While TRACKING: sample the locked bbox into the gallery per policy.
    While SEARCHING/LOST: embed candidate person dets per policy; if the best gallery
    match crosses reid_threshold, adopt the ByteTracker track overlapping that det."""

    def __init__(self, embedder, gallery, policy):
        self.embedder, self.gallery, self.policy = embedder, gallery, policy
        self.person_dets_seen = 0

    def after_step(self, frame, frame_idx, persons, lock, state):
        from hailo_tiling.types import Det
        self.person_dets_seen += len(persons)
        if state.status == "TRACKING" and lock.track_id is not None:
            if self.policy.should_sample_tracked(frame_idx) and state.bbox_norm[2] > 0:
                x, y, w, h = state.bbox_norm
                self.gallery.add(self.embedder.embed(
                    frame, Det(cls=1, score=1.0, x=x, y=y, w=w, h=h), frame_idx))
            return
        if lock.track_id is None or len(self.gallery) == 0:
            return
        cands = self.policy.candidates(
            frame_idx=frame_idx, person_dets=persons,
            tracks=getattr(lock, "last_tracks", []),
            frames_lost=state.frames_since_seen,
            anchor=getattr(lock, "reacq_anchor", None))
        best_sim, best_det = self.gallery.reid_threshold, None
        for d in cands:
            sim = self.gallery.match(self.embedder.embed(frame, d, frame_idx))
            if sim > best_sim:
                best_sim, best_det = sim, d
        if best_det is not None:
            lock.adopt_overlapping(best_det)

    @property
    def stats(self):
        s = dict(self.embedder.stats)
        s["person_dets_seen"] = self.person_dets_seen
        return s
```

In `dynamic_tiling/target_lock.py` add the public adoption API:

```python
def adopt_overlapping(self, det) -> bool:
    """Re-point the lock at the activated track best overlapping `det`
    (ReID-confirmed recovery). Returns True if adopted."""
    best_iou, best = 0.1, None
    for t in self.last_tracks:
        if t.is_activated and t.filtered_tlwh:
            tl = t.filtered_tlwh
            # reuse module-level _iou_tlwh
            iou = _iou_tlwh((det.x, det.y, det.w, det.h), tl)
            if iou > best_iou:
                best_iou, best = iou, t
    if best is None:
        return False
    self._bt_track_id = best.track_id
    s = self.state
    s.bbox_norm = tuple(best.filtered_tlwh)
    s.status = "TRACKING"
    s.frames_since_seen = 0
    self._anchor = tuple(best.filtered_tlwh)
    return True
```

In `dynamic_tiling/replay.py` `run()`: add `reid_assist=None` keyword; after the
tracker-debug block (so dumps reflect pre-ReID state) call:

```python
if reid_assist is not None:
    reid_assist.after_step(frame, frame_idx, persons, lock, state)
    if lock.state.status == "TRACKING" and state.status != "TRACKING":
        state = lock.state                      # ReID recovered this frame
```

and record `res.frame_lock[frame_idx]["reid_recovered"] = True` when that branch fires.
Counters end-to-end test: write
`dynamic_tiling/tests/test_reid_assist.py` driving `ReidAssist` with the fake
extractor from Task 2 + a real `TargetLock` over a synthetic loss→reappear sequence
(GenerousPolicy): assert the lock re-acquires via `adopt_overlapping` and
`assist.stats["embeds"] > 0`.

- [ ] **Step 5: Full suite green; commit** — `feat(reid): policies + ReidAssist replay integration + lock adoption API`

---

### Task 5: Trials wiring + histogram pre-filter (P5)

**Files:**
- Modify: `dynamic_tiling/run_trials.py`, `dynamic_tiling/trials.py`
- Create: P5 in `dynamic_tiling/reid_policy.py`
- Test: extend `dynamic_tiling/tests/test_reid_policy.py`

- [ ] **Step 1:** `run_trials` flags: `--reid-policy {none,generous,prod,ambiguity,motion,histogram}`
(default none), `--reid-hef` (default
`/usr/local/hailo/resources/models/hailo10h/repvgg_a0_person_reid_512.hef`),
`--reid-cache PATH` (embedding cache; may be the same DB as `--cache`),
`--reid-threshold/--reid-gallery-ema` knobs. `trials.py`: build a fresh
`ReidAssist` per trial (gallery must reset between trials!) when policy != none, pass
`reid_assist` through `run()`. Results JSON `per_trial` rows gain
`reid_embeds`, `reid_chip_embeds`, `person_dets_seen`, `frac_dets_embedded`;
aggregate gains means. TDD with the FakeLock/fake_run monkeypatch pattern from Block 1
Task 3.

- [ ] **Step 2:** P5 `HistogramPolicy(MotionGatedPolicy)`: among gate survivors, compute
HSV histogram correlation (`cv2.calcHist` 32x32 H-S bins, `cv2.compareHist`
HISTCMP_CORREL) of each candidate crop vs the last tracked crop's histogram (capture it
in `should_sample_tracked` path — store `self._ref_hist` whenever the target is
sampled); keep only the top-2 candidates by correlation. Unit test with two synthetic
crops (red square vs blue square frames) asserting the color-matched candidate
survives. NOTE: needs the frame, so `candidates()` for P5 also accepts `frame=None`
kwarg — `ReidAssist.after_step` passes it; other policies ignore it.

- [ ] **Step 3:** Full suite green; smoke ONE chip run (fov50, generous, budget 3000)
end-to-end:

```bash
python -m dynamic_tiling.run_trials \
  --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov50.mp4 \
  --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov50/gt_tracks.verified.json \
  --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 \
  --reacq-motion <best-from-block1> --reacq-radius-growth <best> \
  --reid-policy generous \
  --cache dynamic_tiling/runs/cache/0025_fov50__yolov8n4c_vga.sqlite3 \
  --out /tmp/reid_smoke.json
```

Expected: runs to completion; `frac_dets_embedded` ≈ 1.0; recovery metrics ≥ Block-1-only run.

- [ ] **Step 4: Commit** — `feat(reid): trials CLI wiring + histogram pre-filter policy`

---

### Task 6: Ablation driver + REID_ABLATION.md

**Files:**
- Create: `dynamic_tiling/run_reid_ablation.py`
- Output: `dynamic_tiling/runs/REID_ABLATION.md` (committed), per-run JSONs in `dynamic_tiling/runs/reid_ablation/` (untracked)

- [ ] **Step 1:** Driver CLI: `python -m dynamic_tiling.run_reid_ablation --clips 0025:fov50,0025:fov60,0025:fov70,0026:fov50`.
For each clip × arm in [none(P0), generous(P1), prod(P2), ambiguity(P3), motion(P4), histogram(P5)]:
invoke `run_trials`-equivalent in-process (import `run_all_trials`) with the
Block-1 best motion config, budget 3000, `--cache`/`--reid-cache` per clip; write one
JSON per (clip, arm). 0026 fov50 GT path:
`dynamic_tiling/runs/gt_verify_0026_fov50/gt_tracks.verified.json`; cache
`dynamic_tiling/runs/cache/0026_fov50__yolov8n4c_vga.sqlite3` (cold — first runs warm it).

- [ ] **Step 2:** Sub-ablation on the winning arm: gallery FIFO vs `ema_alpha=0.1` vs both → 3 more runs on 0025-fov50 + 0026-fov50.

- [ ] **Step 3:** Render `REID_ABLATION.md`: one table per clip
(rows = arms; cols = coverage, mean IoU, losses, time-to-recover, recovery success,
reid embeds/frame, frac dets embedded) + a combined "quality vs fraction embedded"
section listing (arm, frac, coverage) tuples ordered by frac — the Pareto read.
Include the exact reproduction command per row. State the chosen default policy with
1-paragraph justification.

- [ ] **Step 4: Commit** — `docs(reid): ReID inference ablation results (P0–P5 + gallery sub-ablation)` (MD only; JSONs untracked).
