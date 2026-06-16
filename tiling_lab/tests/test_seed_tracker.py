"""SeedTracker: direct seed->nearest-detection associator for the showcase SOT.

The seed is a POSITION to look at, never a box to display. The tracker places an
ROI there (status TRACKING) and, once a real detection lands in the gate, adopts
that detection's REAL bbox and follows it — never jumping to a far object.
"""
from hailo_tiling.types import Det
from tiling_lab.harness.target_lock import SeedTracker


def _det(x, y, w=0.03, h=0.02, conf=0.9):
    return Det(cls=0, score=conf, x=x, y=y, w=w, h=h)


def test_seed_reports_tracking_at_seed_but_not_detected():
    st = SeedTracker()
    st.seed((0.50, 0.30, 0.04, 0.04))
    # No detections yet: ROI must be placed at the seed (TRACKING) so the
    # scheduler samples there, but nothing is "detected" so no box is drawn.
    detected = st.step([])
    assert detected is False
    assert st.state.status == "TRACKING"          # ROI fires at the seed
    # bbox center sits at the seed center so the ROI is centred there.
    cx = st.state.bbox_norm[0] + st.state.bbox_norm[2] / 2
    cy = st.state.bbox_norm[1] + st.state.bbox_norm[3] / 2
    assert abs(cx - 0.52) < 1e-6 and abs(cy - 0.32) < 1e-6


def test_binds_to_real_detection_near_seed_and_shows_real_bbox():
    st = SeedTracker()
    st.seed((0.50, 0.30, 0.04, 0.04))            # seed centre (0.52, 0.32)
    real = _det(0.521, 0.318, 0.05, 0.035)        # a real car right by the seed
    detected = st.step([real])
    assert detected is True
    assert st.state.status == "TRACKING"
    # The reported bbox is the REAL detection, not the synthetic seed box.
    assert st.state.bbox_norm == (0.521, 0.318, 0.05, 0.035)


def test_does_not_adopt_far_detection_no_car_switch():
    st = SeedTracker()
    st.seed((0.50, 0.30, 0.04, 0.04))            # centre (0.52, 0.32)
    far = _det(0.10, 0.85, 0.06, 0.05)            # a different car, far away
    detected = st.step([far])
    assert detected is False                      # never jumps to the far car
    assert st.state.status == "TRACKING"          # still searching at the seed


def test_picks_nearest_to_prediction_when_two_cars_in_view():
    st = SeedTracker()
    st.seed((0.50, 0.30, 0.04, 0.04))            # centre (0.52, 0.32)
    near = _det(0.515, 0.315)                      # ours
    other = _det(0.555, 0.355)                     # a second car also in gate
    assert st.step([near, other]) is True
    # Adopted the nearer-to-seed detection.
    assert abs(st.state.bbox_norm[0] - 0.515) < 1e-9


def test_tracks_moving_car_frame_to_frame_without_switching():
    st = SeedTracker()
    st.seed((0.50, 0.50, 0.04, 0.04))            # centre (0.52, 0.52)
    # Our car drifts right a little each frame; a decoy sits far away the whole time.
    decoy = _det(0.05, 0.05, 0.06, 0.05)
    cx = 0.52
    for _ in range(20):
        ours = _det(cx - 0.015, 0.50, 0.03, 0.02)   # centre ~ (cx, 0.51)
        assert st.step([ours, decoy]) is True
        # Stays on our car (near cx), never the decoy at 0.08.
        adopted_cx = st.state.bbox_norm[0] + st.state.bbox_norm[2] / 2
        assert abs(adopted_cx - cx) < 0.02
        cx += 0.01


def test_goes_lost_after_track_buffer_misses():
    st = SeedTracker(track_buffer=5)
    st.seed((0.50, 0.50, 0.04, 0.04))
    st.step([_det(0.515, 0.49)])                  # bind once
    assert st.state.status == "TRACKING"
    for _ in range(6):                            # then sustained misses
        st.step([])
    assert st.state.status == "LOST"


def test_reseed_resets_to_fresh_acquisition():
    st = SeedTracker()
    st.seed((0.50, 0.50, 0.04, 0.04))
    st.step([_det(0.515, 0.49)])
    assert st.state.status == "TRACKING"
    # Re-seed somewhere else (second trail): the old binding must not linger.
    st.seed((0.10, 0.80, 0.04, 0.04))            # centre (0.12, 0.82)
    detected = st.step([_det(0.515, 0.49)])       # old car still visible
    assert detected is False                      # it's outside the new seed gate
    new_cx = st.state.bbox_norm[0] + st.state.bbox_norm[2] / 2
    assert abs(new_cx - 0.12) < 1e-6              # ROI now at the NEW seed


def test_seed_binds_immediately_to_detection_at_selection():
    st = SeedTracker()
    st.seed((0.50, 0.30, 0.04, 0.04))             # selection centre (0.52, 0.32)
    real = _det(0.518, 0.317, 0.03, 0.02)         # dense detection right there
    assert st.step([real]) is True                # binds on the selection frame
    assert st.state.bbox_norm == (0.518, 0.317, 0.03, 0.02)


def test_seed_tight_gate_never_grabs_far_neighbour_even_after_waiting():
    st = SeedTracker()
    st.seed((0.50, 0.30, 0.04, 0.04))             # centre (0.52, 0.32)
    far = _det(0.64, 0.27, 0.03, 0.02)            # a different car ~0.15 away
    for _ in range(40):                            # wait many frames
        assert st.step([far]) is False             # gate must NOT grow to reach it
