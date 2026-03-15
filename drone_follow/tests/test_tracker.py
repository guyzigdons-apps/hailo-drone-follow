"""Tests for tracker protocol, adapters, factory, and metrics wrapper."""

import argparse
import time

import numpy as np
import pytest

from drone_follow.pipeline_adapter.tracker import (
    MetricsTracker,
    TrackedObject,
    TrackerMetrics,
)
from drone_follow.pipeline_adapter.tracker_factory import (
    DEFAULT_TRACKER,
    TRACKER_CHOICES,
    add_tracker_args,
    create_tracker,
)
from drone_follow.pipeline_adapter.byte_tracker import (
    ByteTracker,
    ByteTrackerAdapter,
    KalmanFilter,
    STrack,
    iou_batch,
    center_distance_batch,
    combined_cost_batch,
    linear_assignment,
    joint_stracks,
    sub_stracks,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_det(x1, y1, x2, y2, conf=0.9):
    """Return a single detection row [x1, y1, x2, y2, conf]."""
    return [x1, y1, x2, y2, conf]


def _dets(*boxes):
    """Build Nx5 detection array from (x1, y1, x2, y2, conf) tuples."""
    if not boxes:
        return np.empty((0, 5), dtype=np.float32)
    return np.array(boxes, dtype=np.float32)


def _reset_strack_counter():
    """Reset global STrack ID counter between tests."""
    STrack._count = 0


# ---------------------------------------------------------------------------
# TrackedObject dataclass
# ---------------------------------------------------------------------------

class TestTrackedObject:
    def test_fields(self):
        t = TrackedObject(track_id=1, input_index=0, is_activated=True, score=0.9)
        assert t.track_id == 1
        assert t.input_index == 0
        assert t.is_activated is True
        assert t.score == 0.9


# ---------------------------------------------------------------------------
# TrackerMetrics
# ---------------------------------------------------------------------------

class TestTrackerMetrics:
    def test_defaults(self):
        m = TrackerMetrics()
        assert m.total_frames == 0
        assert m.fps == 0.0
        assert m.id_switches == 0

    def test_snapshot(self):
        m = TrackerMetrics(total_frames=10, fps=29.97, update_ms=1.234,
                           match_ratio=0.8765, active_tracks=3, id_switches=1)
        s = m.snapshot()
        assert s["total_frames"] == 10
        assert s["fps"] == 30.0  # rounded to 1 decimal
        assert s["update_ms"] == 1.23
        assert s["match_ratio"] == 0.876
        assert s["active_tracks"] == 3
        assert s["id_switches"] == 1


# ---------------------------------------------------------------------------
# KalmanFilter
# ---------------------------------------------------------------------------

class TestKalmanFilter:
    def test_initiate(self):
        kf = KalmanFilter()
        measurement = np.array([100, 200, 0.5, 300])
        mean, cov = kf.initiate(measurement)
        assert mean.shape == (10,)
        assert cov.shape == (10, 10)
        np.testing.assert_array_equal(mean[:4], measurement)
        np.testing.assert_array_equal(mean[4:8], 0)

    def test_predict_changes_state(self):
        kf = KalmanFilter()
        mean, cov = kf.initiate(np.array([100, 200, 0.5, 300]))
        mean2, cov2 = kf.predict(mean, cov)
        assert mean2.shape == mean.shape
        # Covariance should grow after prediction (more uncertainty)
        assert np.trace(cov2) > np.trace(cov)

    def test_update_reduces_uncertainty(self):
        kf = KalmanFilter()
        measurement = np.array([100, 200, 0.5, 300])
        mean, cov = kf.initiate(measurement)
        mean, cov = kf.predict(mean, cov)
        cov_before = np.trace(cov)
        mean, cov = kf.update(mean, cov, measurement)
        # Update with same measurement should reduce uncertainty
        assert np.trace(cov) < cov_before

    def test_gating_distance(self):
        kf = KalmanFilter()
        measurement = np.array([100, 200, 0.5, 300])
        mean, cov = kf.initiate(measurement)
        # Distance to itself should be near zero
        dist = kf.gating_distance(mean, cov, measurement.reshape(1, -1))
        assert dist[0] < 1.0

        # Distance to a far-away measurement should be large
        far = np.array([[500, 600, 1.0, 100]])
        dist_far = kf.gating_distance(mean, cov, far)
        assert dist_far[0] > dist[0]


# ---------------------------------------------------------------------------
# iou_batch / center_distance_batch / combined_cost_batch
# ---------------------------------------------------------------------------

class TestIoUBatch:
    def test_identical_boxes(self):
        boxes = np.array([[10, 20, 50, 80]])
        iou = iou_batch(boxes, boxes)
        assert iou.shape == (1, 1)
        np.testing.assert_almost_equal(iou[0, 0], 1.0)

    def test_non_overlapping(self):
        a = np.array([[0, 0, 10, 10]])
        b = np.array([[20, 20, 30, 30]])
        iou = iou_batch(a, b)
        assert iou[0, 0] == 0.0

    def test_partial_overlap(self):
        a = np.array([[0, 0, 10, 10]])
        b = np.array([[5, 5, 15, 15]])
        iou = iou_batch(a, b)
        # Intersection = 5*5=25, union = 100+100-25=175
        np.testing.assert_almost_equal(iou[0, 0], 25.0 / 175.0)

    def test_empty_inputs(self):
        a = np.array([]).reshape(0, 4)
        b = np.array([[0, 0, 10, 10]])
        assert iou_batch(a, b).shape == (0, 1)
        assert iou_batch(b, a).shape == (1, 0)

    def test_multiple_boxes(self):
        a = np.array([[0, 0, 10, 10], [20, 20, 30, 30]])
        b = np.array([[0, 0, 10, 10]])
        iou = iou_batch(a, b)
        assert iou.shape == (2, 1)
        np.testing.assert_almost_equal(iou[0, 0], 1.0)
        assert iou[1, 0] == 0.0


class TestCenterDistanceBatch:
    def test_identical(self):
        boxes = np.array([[0, 0, 10, 10]])
        d = center_distance_batch(boxes, boxes)
        assert d[0, 0] == 0.0

    def test_far_apart(self):
        a = np.array([[0, 0, 10, 10]])
        b = np.array([[100, 100, 110, 110]])
        d = center_distance_batch(a, b)
        assert d[0, 0] > 0


class TestCombinedCostBatch:
    def test_identical_is_zero(self):
        boxes = np.array([[0, 0, 100, 100]])
        c = combined_cost_batch(boxes, boxes)
        np.testing.assert_almost_equal(c[0, 0], 0.0)

    def test_empty(self):
        a = np.array([]).reshape(0, 4)
        b = np.array([[0, 0, 10, 10]])
        assert combined_cost_batch(a, b).shape == (0, 1)


# ---------------------------------------------------------------------------
# linear_assignment
# ---------------------------------------------------------------------------

class TestLinearAssignment:
    def test_perfect_assignment(self):
        cost = np.array([[0.1, 0.9], [0.9, 0.1]])
        matches, ua, ub = linear_assignment(cost, thresh=0.5)
        assert len(matches) == 2
        assert len(ua) == 0
        assert len(ub) == 0

    def test_above_threshold(self):
        cost = np.array([[0.8]])
        matches, ua, ub = linear_assignment(cost, thresh=0.5)
        assert len(matches) == 0
        assert 0 in ua
        assert 0 in ub

    def test_empty_cost(self):
        cost = np.empty((0, 0))
        matches, ua, ub = linear_assignment(cost, thresh=0.5)
        assert len(matches) == 0


# ---------------------------------------------------------------------------
# STrack helpers
# ---------------------------------------------------------------------------

class TestSTrack:
    def setup_method(self):
        _reset_strack_counter()

    def test_tlwh_to_xyah(self):
        # tlwh = [10, 20, 40, 80] -> center_x=30, center_y=60, aspect=0.5, h=80
        xyah = STrack.tlwh_to_xyah(np.array([10.0, 20.0, 40.0, 80.0]))
        np.testing.assert_almost_equal(xyah, [30, 60, 0.5, 80])

    def test_tlbr_to_tlwh(self):
        # tlbr = [10, 20, 50, 100] -> tlwh = [10, 20, 40, 80]
        tlwh = STrack.tlbr_to_tlwh(np.array([10, 20, 50, 100]))
        np.testing.assert_array_equal(tlwh, [10, 20, 40, 80])

    def test_next_id_increments(self):
        _reset_strack_counter()
        assert STrack.next_id() == 1
        assert STrack.next_id() == 2
        assert STrack.next_id() == 3

    def test_activate(self):
        _reset_strack_counter()
        st = STrack(np.array([10, 20, 40, 80]), score=0.9)
        kf = KalmanFilter()
        st.activate(kf, frame_id=1)
        assert st.is_activated
        assert st.track_id == 1
        assert st.mean is not None

    def test_tlbr_property(self):
        st = STrack(np.array([10, 20, 40, 80]), score=0.9)
        # Before activation: uses _tlwh directly
        tlbr = st.tlbr
        np.testing.assert_almost_equal(tlbr, [10, 20, 50, 100])


class TestJointSubStracks:
    def setup_method(self):
        _reset_strack_counter()

    def _make_tracks(self, n):
        kf = KalmanFilter()
        tracks = []
        for i in range(n):
            st = STrack(np.array([i * 100, 0, 50, 50]), score=0.9)
            st.activate(kf, frame_id=1)
            tracks.append(st)
        return tracks

    def test_joint_no_duplicates(self):
        a = self._make_tracks(2)
        b = self._make_tracks(1)  # Different IDs since counter increments
        result = joint_stracks(a, b)
        assert len(result) == 3

    def test_sub_stracks(self):
        tracks = self._make_tracks(3)
        to_remove = [tracks[1]]
        result = sub_stracks(tracks, to_remove)
        assert len(result) == 2
        ids = {t.track_id for t in result}
        assert tracks[1].track_id not in ids


# ---------------------------------------------------------------------------
# ByteTracker (core)
# ---------------------------------------------------------------------------

class TestByteTracker:
    def setup_method(self):
        _reset_strack_counter()

    def test_empty_frame(self):
        bt = ByteTracker(track_thresh=0.4, track_buffer=30)
        result = bt.update(np.empty((0, 5), dtype=np.float32))
        assert result == []

    def test_single_detection_creates_track(self):
        bt = ByteTracker(track_thresh=0.4, track_buffer=30)
        dets = _dets(_make_det(100, 200, 200, 400, 0.9))
        result = bt.update(dets)
        # First frame may or may not activate immediately depending on thresh
        # Run a second frame with same detection to confirm track
        result = bt.update(dets)
        assert len(result) >= 1
        assert result[0].is_activated

    def test_consistent_track_id(self):
        bt = ByteTracker(track_thresh=0.4, track_buffer=30)
        det = _dets(_make_det(100, 200, 200, 400, 0.9))
        bt.update(det)
        r2 = bt.update(det)
        r3 = bt.update(det)
        if r2 and r3:
            assert r2[0].track_id == r3[0].track_id

    def test_multiple_detections(self):
        bt = ByteTracker(track_thresh=0.4, track_buffer=30)
        dets = _dets(
            _make_det(100, 100, 200, 300, 0.9),
            _make_det(400, 100, 500, 300, 0.9),
        )
        bt.update(dets)
        result = bt.update(dets)
        assert len(result) == 2
        ids = {t.track_id for t in result}
        assert len(ids) == 2  # Each detection gets a unique track

    def test_lost_track_after_buffer(self):
        bt = ByteTracker(track_thresh=0.4, track_buffer=5)
        det = _dets(_make_det(100, 200, 200, 400, 0.9))
        # Create and confirm track
        for _ in range(3):
            bt.update(det)

        # Send empty frames beyond buffer
        empty = _dets()
        for _ in range(10):
            bt.update(empty)

        result = bt.update(empty)
        assert len(result) == 0

    def test_input_index_mapping(self):
        bt = ByteTracker(track_thresh=0.4, track_buffer=30)
        dets = _dets(
            _make_det(100, 100, 200, 300, 0.9),
            _make_det(400, 100, 500, 300, 0.9),
        )
        bt.update(dets)
        result = bt.update(dets)
        indices = {t.input_index for t in result}
        # Each track should map to a different input detection
        assert len(indices) == 2
        assert all(idx >= 0 for idx in indices)

    def test_moving_detection(self):
        """Track should follow a slowly moving detection."""
        bt = ByteTracker(track_thresh=0.4, track_buffer=30)
        for i in range(10):
            x = 100 + i * 5  # Slow movement
            dets = _dets(_make_det(x, 200, x + 100, 400, 0.9))
            result = bt.update(dets)

        assert len(result) == 1


# ---------------------------------------------------------------------------
# ByteTrackerAdapter
# ---------------------------------------------------------------------------

class TestByteTrackerAdapter:
    def setup_method(self):
        _reset_strack_counter()

    def test_protocol_compliance(self):
        adapter = ByteTrackerAdapter(track_thresh=0.4, track_buffer=30,
                                     match_thresh=0.5, frame_rate=30)
        dets = _dets(_make_det(100, 200, 200, 400, 0.9))
        result = adapter.update(dets)
        assert isinstance(result, list)
        for obj in result:
            assert isinstance(obj, TrackedObject)

    def test_empty_detections(self):
        adapter = ByteTrackerAdapter(track_thresh=0.4)
        result = adapter.update(np.empty((0, 5), dtype=np.float32))
        assert result == []


# ---------------------------------------------------------------------------
# FastTrackerAdapter
# ---------------------------------------------------------------------------

class TestFastTrackerAdapter:
    def setup_method(self):
        _reset_strack_counter()

    def test_protocol_compliance(self):
        from drone_follow.pipeline_adapter.fast_tracker import FastTrackerAdapter
        adapter = FastTrackerAdapter(track_thresh=0.4, track_buffer=30,
                                     match_thresh=0.5, frame_rate=30)
        dets = _dets(_make_det(100, 200, 200, 400, 0.9))
        result = adapter.update(dets)
        assert isinstance(result, list)
        for obj in result:
            assert isinstance(obj, TrackedObject)

    def test_empty_detections(self):
        from drone_follow.pipeline_adapter.fast_tracker import FastTrackerAdapter
        adapter = FastTrackerAdapter()
        result = adapter.update(np.empty((0, 5), dtype=np.float32))
        assert result == []

    def test_tracking_across_frames(self):
        from drone_follow.pipeline_adapter.fast_tracker import FastTrackerAdapter
        adapter = FastTrackerAdapter(track_thresh=0.4)
        det = _dets(_make_det(100, 200, 200, 400, 0.9))
        for _ in range(5):
            result = adapter.update(det)
        assert len(result) >= 1
        assert all(isinstance(r, TrackedObject) for r in result)


# ---------------------------------------------------------------------------
# MetricsTracker wrapper
# ---------------------------------------------------------------------------

class TestMetricsTracker:
    def setup_method(self):
        _reset_strack_counter()

    def test_wraps_inner_tracker(self):
        inner = ByteTrackerAdapter(track_thresh=0.4)
        mt = MetricsTracker(inner)
        dets = _dets(_make_det(100, 200, 200, 400, 0.9))
        result = mt.update(dets)
        assert isinstance(result, list)
        assert mt.metrics.total_frames == 1

    def test_frame_count_increments(self):
        inner = ByteTrackerAdapter(track_thresh=0.4)
        mt = MetricsTracker(inner)
        det = _dets(_make_det(100, 200, 200, 400, 0.9))
        for _ in range(5):
            mt.update(det)
        assert mt.metrics.total_frames == 5

    def test_match_ratio(self):
        inner = ByteTrackerAdapter(track_thresh=0.4)
        mt = MetricsTracker(inner)
        det = _dets(_make_det(100, 200, 200, 400, 0.9))
        # After a few frames, detection should be matched
        for _ in range(5):
            mt.update(det)
        assert mt.metrics.match_ratio > 0

    def test_id_switches_counted(self):
        inner = ByteTrackerAdapter(track_thresh=0.4, track_buffer=2)
        mt = MetricsTracker(inner)
        det = _dets(_make_det(100, 200, 200, 400, 0.9))
        # Establish a track
        for _ in range(3):
            mt.update(det)
        # Lose it
        for _ in range(5):
            mt.update(_dets())
        assert mt.metrics.id_switches >= 1

    def test_active_tracks(self):
        inner = ByteTrackerAdapter(track_thresh=0.4)
        mt = MetricsTracker(inner)
        dets = _dets(
            _make_det(100, 100, 200, 300, 0.9),
            _make_det(400, 100, 500, 300, 0.9),
        )
        for _ in range(3):
            mt.update(dets)
        assert mt.metrics.active_tracks == 2

    def test_update_ms_positive(self):
        inner = ByteTrackerAdapter(track_thresh=0.4)
        mt = MetricsTracker(inner)
        mt.update(_dets(_make_det(100, 200, 200, 400, 0.9)))
        assert mt.metrics.update_ms >= 0


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

class TestTrackerFactory:
    def test_create_byte(self):
        t = create_tracker("byte")
        assert hasattr(t, "update")

    def test_create_fast(self):
        t = create_tracker("fast")
        assert hasattr(t, "update")

    def test_unknown_raises(self):
        with pytest.raises(ValueError, match="Unknown tracker"):
            create_tracker("nonexistent")

    def test_default_tracker(self):
        assert DEFAULT_TRACKER == "byte"

    def test_choices(self):
        assert "byte" in TRACKER_CHOICES
        assert "fast" in TRACKER_CHOICES


class TestAddTrackerArgs:
    def test_adds_args(self):
        parser = argparse.ArgumentParser()
        add_tracker_args(parser)
        args = parser.parse_args([])
        assert args.tracker == DEFAULT_TRACKER

    def test_custom_tracker(self):
        parser = argparse.ArgumentParser()
        add_tracker_args(parser)
        args = parser.parse_args(["--tracker", "fast"])
        assert args.tracker == "fast"


# ---------------------------------------------------------------------------
# Cross-tracker integration: all adapters behave the same on a simple sequence
# ---------------------------------------------------------------------------

class TestAllTrackersIntegration:
    """Run the same detection sequence through all tracker adapters."""

    @pytest.fixture(params=["byte", "fast"])
    def tracker(self, request):
        _reset_strack_counter()
        return create_tracker(request.param, track_thresh=0.4, track_buffer=30)

    def test_single_stationary_target(self, tracker):
        det = _dets(_make_det(100, 200, 200, 400, 0.9))
        results = []
        for _ in range(5):
            results = tracker.update(det)
        assert len(results) >= 1
        assert all(r.is_activated for r in results)

    def test_empty_then_detection(self, tracker):
        tracker.update(_dets())
        tracker.update(_dets())
        det = _dets(_make_det(100, 200, 200, 400, 0.9))
        for _ in range(5):
            result = tracker.update(det)
        assert len(result) >= 1

    def test_returns_tracked_objects(self, tracker):
        det = _dets(_make_det(100, 200, 200, 400, 0.9))
        for _ in range(3):
            result = tracker.update(det)
        for obj in result:
            assert isinstance(obj, TrackedObject)
            assert isinstance(obj.track_id, (int, np.integer))
            assert isinstance(obj.score, (float, np.floating))
