"""ReID inference policies — decide WHICH person dets get embedded and WHEN the
tracked target is sampled into the gallery. Pure logic, no chip access.
Arms (spec): P1 generous, P2 prod-style, P3 ambiguity-gated, P4 motion-gated+decay,
P5 histogram pre-filter (separate task)."""
from __future__ import annotations

from hailo_tiling.classes import PERSON
from hailo_tiling.types import Det


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


class ReidAssist:
    """Owns embedder+gallery+policy; called from replay.run (frames stay out of
    TargetLock). While TRACKING: sample the locked bbox into the gallery per policy.
    While SEARCHING/LOST: embed candidate person dets per policy; if the best gallery
    match crosses reid_threshold, adopt the ByteTracker track overlapping that det."""

    def __init__(self, embedder, gallery, policy):
        self.embedder, self.gallery, self.policy = embedder, gallery, policy
        self.person_dets_seen = 0

    def after_step(self, frame, frame_idx, persons, lock, state):
        self.person_dets_seen += len(persons)
        if state.status == "TRACKING" and lock.track_id is not None:
            if self.policy.should_sample_tracked(frame_idx) and state.bbox_norm[2] > 0:
                x, y, w, h = state.bbox_norm
                self.gallery.add(self.embedder.embed(
                    frame, Det(cls=PERSON, score=1.0, x=x, y=y, w=w, h=h), frame_idx))
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
