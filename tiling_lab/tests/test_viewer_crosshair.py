"""smooth_target_track parity with the web visualizer's smoothTargetTrack."""
from types import SimpleNamespace

from tiling_lab.viewer.overlay_viewer import smooth_target_track


def _run(centres):
    """Build a Run-like stub: frame -> a single 'target' det at (cx,cy)."""
    idx = {}
    for n, (cx, cy) in centres.items():
        idx[n] = [{"label": "target", "confidence": 0.9,
                   "bbox": [cx - 0.05, cy - 0.05, 0.1, 0.1]}]
    return SimpleNamespace(idx=idx)


def test_first_sample_seeds_ema_at_raw_centre():
    m = smooth_target_track([_run({0: (0.2, 0.2)})], alpha=0.35)
    assert m[0] == (0.2, 0.2)


def test_ema_lags_raw_motion():
    m = smooth_target_track([_run({0: (0.0, 0.0), 1: (1.0, 1.0)})], alpha=0.5)
    assert m[0] == (0.0, 0.0)
    assert m[1] == (0.5, 0.5)  # 0.5*1 + 0.5*0


def test_frames_without_target_get_no_entry():
    r = SimpleNamespace(idx={
        0: [{"label": "target", "bbox": [0.25, 0.25, 0.1, 0.1]}],
        1: [{"label": "vehicle", "bbox": [0.0, 0.0, 0.1, 0.1]}],
        2: [{"label": "target", "bbox": [0.25, 0.25, 0.1, 0.1]}],
    })
    m = smooth_target_track([r], alpha=0.5)
    assert 0 in m and 1 not in m and 2 in m


def test_long_gap_resets_filter():
    m = smooth_target_track([_run({0: (0.1, 0.1), 100: (0.9, 0.9)})],
                            alpha=0.5, reset_gap=30)
    assert m[100] == (0.9, 0.9)  # reset → snaps exactly, no blend


def test_lower_alpha_is_smoother_more_lag():
    centres = {0: (0.0, 0.0), 1: (1.0, 1.0)}
    snappy = smooth_target_track([_run(centres)], alpha=0.5)[1][0]
    smooth = smooth_target_track([_run(centres)], alpha=0.18)[1][0]
    assert smooth < snappy  # 0.18 lags further behind the new position
