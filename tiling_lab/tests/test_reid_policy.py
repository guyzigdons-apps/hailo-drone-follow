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
    from tiling_lab.reid.reid_policy import GenerousPolicy
    p = GenerousPolicy()
    dets = [_d(0.1), _d(0.5), _d(0.9, cls=2)]      # vehicle must be excluded
    out = p.candidates(frame_idx=10, person_dets=dets[:2], tracks=[], frames_lost=5)
    assert out == dets[:2]
    assert p.should_sample_tracked(frame_idx=7)     # every frame


def test_ambiguity_policy_skips_clean_continuations():
    from tiling_lab.reid.reid_policy import AmbiguityPolicy
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
    from tiling_lab.reid.reid_policy import MotionGatedPolicy
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


def _color_frame(color_bgr):
    f = np.zeros((1080, 1920, 3), dtype=np.uint8)
    f[:, :] = color_bgr
    return f


def test_histogram_policy_keeps_color_matched_candidate():
    """P5: among motion-gate survivors, keep the candidate whose HSV histogram
    correlates with the reference (tracked) crop. The color-matched candidate
    must survive; the off-color one is dropped (top-2 keep, only 2 candidates,
    but the color match must rank first and the reference must drive it)."""
    from tiling_lab.reid.reid_policy import HistogramPolicy
    p = HistogramPolicy(radius_growth=1.0, r0=2.0)  # huge gate: never filter on motion

    red = (0, 0, 255)   # BGR red
    blue = (255, 0, 0)   # BGR blue
    red_frame = _color_frame(red)

    # Reference: target was last sampled on a RED frame.
    p.note_reference(red_frame, _d(0.30))

    # Two candidates at the same position but evaluated on different frames:
    # a red-matched candidate and a blue mismatched candidate.
    red_cand = _d(0.30)
    blue_cand = _d(0.70)

    # Frame for scoring is RED everywhere; recolor the blue candidate region blue
    # so its crop histogram differs from the red reference.
    frame = _color_frame(red)
    # paint the blue candidate's pixel region blue
    bx = int(0.70 * 1920)
    frame[:, bx - 40:bx + 40] = blue

    out = p.candidates(frame_idx=0, person_dets=[red_cand, blue_cand], tracks=[],
                       frames_lost=5, anchor=(0.30, 0.5, 0.04, 0.10), frame=frame)
    assert red_cand in out
    # With top-2 and only 2 candidates both could survive; assert the color match
    # ranks strictly ahead by checking the blue one is dropped when we add a 3rd.
    blue_cand2 = _d(0.50)
    bx2 = int(0.50 * 1920)
    frame[:, bx2 - 40:bx2 + 40] = blue
    out3 = p.candidates(frame_idx=0,
                        person_dets=[red_cand, blue_cand, blue_cand2], tracks=[],
                        frames_lost=5, anchor=(0.30, 0.5, 0.04, 0.10), frame=frame)
    assert red_cand in out3 and len(out3) == 2


def test_histogram_policy_passthrough_without_reference():
    """No reference hist captured yet -> behave like the motion gate (no color cut)."""
    from tiling_lab.reid.reid_policy import HistogramPolicy
    p = HistogramPolicy(radius_growth=1.0, r0=2.0)
    a, b, c = _d(0.30), _d(0.31), _d(0.32)
    frame = _color_frame((0, 0, 255))
    out = p.candidates(frame_idx=0, person_dets=[a, b, c], tracks=[],
                       frames_lost=5, anchor=(0.30, 0.5, 0.04, 0.10), frame=frame)
    assert set(out) == {a, b, c}


def test_policies_registry_includes_histogram():
    from tiling_lab.reid.reid_policy import POLICIES, HistogramPolicy
    assert POLICIES["histogram"] is HistogramPolicy
