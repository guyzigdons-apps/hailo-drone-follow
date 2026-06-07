"""End-to-end ReidAssist integration: a real TargetLock over a synthetic
loss -> reappear sequence. The reappear position is FAR outside any motion gate
(frozen anchor + reacq_radius_growth=0 + distant reappear) so the lock's own
re-acquisition CANNOT bridge the gap -- only the ReID path can explain the
recovery, which proves the integration (not the Block-1 motion gates)."""
import numpy as np
from hailo_tiling.types import Det

from tiling_lab.reid.reid_gallery import ReidGallery
from tiling_lab.reid.reid_policy import GenerousPolicy, ReidAssist
from tiling_lab.reid.reid_embedder import ReidEmbedder
from tiling_lab.harness.target_lock import TargetLock


class _FakeExtractor:
    """Mirrors test_reid_embedder._FakeExtractor: L2-normalized, deterministic.
    With a constant frame every crop has the same mean, so every embedding is
    identical -> any visible person matches the seeded gallery (cosine ~1.0)."""
    model_name = "fake"

    def __init__(self):
        self.calls = 0

    def extract_embedding(self, crop_bgr):
        self.calls += 1
        v = np.ones(4, dtype=np.float32) * (crop_bgr.mean() + 1.0)
        return v / np.linalg.norm(v)

    def close(self):
        pass


def _frame():
    return np.full((1080, 1920, 3), 120, dtype=np.uint8)


def _person(x, y=0.5, w=0.05, h=0.12, score=0.9):
    return Det(cls=1, score=score, x=x, y=y, w=w, h=h)


def test_reid_recovers_lock_after_distant_reappear():
    # Frozen anchor + no radius growth => the lock's own reacq is IoU-only and
    # cannot bridge a distant reappear; ReID is the ONLY recovery path.
    lock = TargetLock(track_buffer=90, reacq_motion="frozen", reacq_radius_growth=0.0)
    gallery = ReidGallery(size=10, reid_threshold=0.75, drift_threshold=0.6,
                          duplicate_threshold=0.99, min_gallery_for_drift_check=99)
    assist = ReidAssist(ReidEmbedder(extractor=_FakeExtractor()), gallery, GenerousPolicy())
    frame = _frame()

    # Phase 1: target tracked at the left edge; gallery is seeded each frame.
    near = 0.10
    state = None
    for fi in range(12):
        persons = [_person(near)]
        if lock.track_id is None:
            state = lock.step(persons, gt_bbox_norm=(near, 0.5, 0.05, 0.12))
        else:
            state = lock.step(persons)
        assist.after_step(frame, fi, persons, lock, state)

    assert lock.track_id is not None
    assert state.status == "TRACKING"
    assert len(gallery) > 0                       # gallery seeded while tracking

    # Phase 2: target disappears entirely (no person dets) -> lock goes SEARCHING.
    for fi in range(12, 20):
        persons = []
        state = lock.step(persons)
        assist.after_step(frame, fi, persons, lock, state)
    assert state.status != "TRACKING"             # lost while no one is visible

    # Phase 3: a person reappears FAR away (right edge). IoU vs the frozen
    # left-edge anchor is 0, so the lock's reacq does nothing -- but ReID embeds
    # the candidate, matches the gallery, and adopts the overlapping track.
    far = 0.90
    recovered = False
    for fi in range(20, 35):
        persons = [_person(far)]
        state = lock.step(persons)
        assert state.status != "TRACKING"         # ByteTracker/reacq alone never recovers
        assist.after_step(frame, fi, persons, lock, state)
        if lock.state.status == "TRACKING":
            recovered = True
            break

    assert recovered, "ReID failed to re-acquire the distant reappearing target"
    assert lock.state.status == "TRACKING"
    # Recovered onto the far person, not the stale left-edge anchor.
    assert lock.state.bbox_norm[0] > 0.5
    assert assist.stats["embeds"] > 0


def test_reid_stats_are_per_assist_delta_not_cumulative():
    """Trials share ONE embedder (run_trials builds it once, run_all_trials makes a
    fresh ReidAssist per trial). embedder.stats is cumulative across trials, so each
    ReidAssist must snapshot the embedder counters at construction and report the
    DELTA. Two sequential assists each doing exactly one embed must BOTH report
    embeds==1 -- not 1 then 2."""
    embedder = ReidEmbedder(extractor=_FakeExtractor())
    frame = _frame()
    person = _person(0.10)

    a1 = ReidAssist(embedder, ReidGallery(), GenerousPolicy())
    embedder.embed(frame, person, frame_idx=0)
    assert a1.stats["embeds"] == 1
    assert a1.stats["chip_embeds"] == 1

    a2 = ReidAssist(embedder, ReidGallery(), GenerousPolicy())
    embedder.embed(frame, person, frame_idx=1)
    # a2 sees only ITS OWN embed; a1's delta is frozen at construction time of a2.
    assert a2.stats["embeds"] == 1
    assert a2.stats["chip_embeds"] == 1
