import numpy as np
from hailo_tiling.types import Det


class _Trk:
    def __init__(self, tid, tlwh, activated=True):
        self.track_id = tid
        self.filtered_tlwh = tlwh
        self.is_activated = activated


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
