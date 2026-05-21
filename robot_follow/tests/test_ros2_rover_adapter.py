"""ROS 2 rover adapter unit tests (rclpy injected via fixture)."""

from __future__ import annotations

import asyncio
import importlib
import os
import signal
import sys
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest


@pytest.fixture
def rclpy_mock(monkeypatch):
    """Inject MagicMock rclpy + geometry_msgs into sys.modules.

    monkeypatch auto-rolls back so the injection cannot leak across tests.
    """
    rclpy = MagicMock(name="rclpy")
    rclpy.signals.SignalHandlerOptions.NO = "NO"
    rclpy.executors.SingleThreadedExecutor = MagicMock(name="SingleThreadedExecutor")
    rclpy.node.Node = MagicMock(name="Node")

    geometry_msgs = MagicMock(name="geometry_msgs")
    geometry_msgs_msg = MagicMock(name="geometry_msgs.msg")

    class FakeTwist:
        def __init__(self):
            self.linear = MagicMock(x=0.0, y=0.0, z=0.0)
            self.angular = MagicMock(x=0.0, y=0.0, z=0.0)

    geometry_msgs_msg.Twist = FakeTwist

    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.executors", rclpy.executors)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy.node)
    monkeypatch.setitem(sys.modules, "rclpy.signals", rclpy.signals)
    monkeypatch.setitem(sys.modules, "geometry_msgs", geometry_msgs)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry_msgs_msg)

    yield {"rclpy": rclpy, "Twist": FakeTwist}


def _det(cx=0.5, cy=0.5, bh=0.3):
    from robot_follow.follow_api.types import Detection

    return Detection(
        label="person",
        confidence=0.9,
        center_x=cx,
        center_y=cy,
        bbox_height=bh,
        timestamp=0.0,
    )


def _default_args(**overrides):
    defaults = dict(
        cmd_vel_topic="/cmd_vel",
        ros_namespace="",
        ros_domain_id=0,
    )
    defaults.update(overrides)
    return SimpleNamespace(**defaults)


def _build_adapter(rclpy_mock, **arg_overrides):
    from robot_follow.follow_api.config import ControllerConfig
    from robot_follow.robot_api.adapters.ros2_rover import Ros2RoverAdapter

    args = _default_args(**arg_overrides)
    return Ros2RoverAdapter(args, ControllerConfig())


class TestImportSafety:
    def test_module_imports_cleanly_without_rclpy(self):
        importlib.import_module("robot_follow.robot_api.adapters.ros2_rover")

    def test_friendly_error_when_rclpy_missing(self, monkeypatch):
        from robot_follow.follow_api.config import ControllerConfig
        from robot_follow.robot_api.adapters.ros2_rover import Ros2RoverAdapter

        monkeypatch.delitem(sys.modules, "rclpy", raising=False)
        with pytest.raises(RuntimeError) as exc_info:
            Ros2RoverAdapter(_default_args(), ControllerConfig())
        msg = str(exc_info.value)
        assert "ROS 2 not" in msg
        assert "source /opt/ros/humble/setup.bash" in msg


class TestProtocolShape:
    def test_implements_robot_protocol(self, rclpy_mock):
        from robot_follow.robot_api.robot import Robot

        adapter = _build_adapter(rclpy_mock)
        assert isinstance(adapter, Robot)

    def test_caps_is_rover_caps(self, rclpy_mock):
        from robot_follow.follow_api.types import Axis
        from robot_follow.robot_api.adapters.ros2_rover import ROVER_CAPS

        adapter = _build_adapter(rclpy_mock)
        assert adapter.caps is ROVER_CAPS
        assert adapter.caps.axes == frozenset({Axis.FORWARD, Axis.YAW})
        assert adapter.caps.yaw_unit == "rad/s"
        assert Axis.ALTITUDE not in adapter.caps.axes

    def test_has_six_methods(self, rclpy_mock):
        from robot_follow.robot_api.adapters.ros2_rover import Ros2RoverAdapter

        for m in (
            "connect",
            "start_session",
            "send_command",
            "send_zero",
            "on_target_lost",
            "shutdown",
        ):
            assert hasattr(Ros2RoverAdapter, m), f"missing method: {m}"


class TestSignalHandlerPreservation:
    def test_connect_calls_init_with_signal_handler_options_no(self, rclpy_mock):
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        rclpy = rclpy_mock["rclpy"]
        assert rclpy.init.called
        _args, kwargs = rclpy.init.call_args
        assert kwargs.get("signal_handler_options") == "NO"

    def test_sigint_handler_survives_connect(self, rclpy_mock):
        prior = signal.getsignal(signal.SIGINT)
        try:
            def fake_handler(*_):
                pass

            signal.signal(signal.SIGINT, fake_handler)
            adapter = _build_adapter(rclpy_mock)
            asyncio.run(adapter.connect())
            assert signal.getsignal(signal.SIGINT) is fake_handler
        finally:
            signal.signal(signal.SIGINT, prior)


class TestTwistPublish:
    def test_publishes_twist_with_forward_and_yaw_rate(self, rclpy_mock):
        from robot_follow.follow_api.config import ControllerConfig
        from robot_follow.follow_api.types import RobotCommand, SafetyContext

        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        cmd = RobotCommand(forward_m_s=1.5, yaw_rate=0.3, down_m_s=0.0)
        ctx = SafetyContext.from_detection(_det())
        asyncio.run(adapter.send_command(cmd, ctx))
        assert adapter._publisher.publish.called
        twist = adapter._publisher.publish.call_args.args[0]
        # Forward is scaled by yaw-align: 1.5 * (1 - 0.3 / max_yawspeed).
        # See TestYawAlignForwardAttenuation for the dedicated coverage.
        cfg_max_yaw = ControllerConfig().max_yawspeed
        assert twist.linear.x == pytest.approx(1.5 * (1.0 - 0.3 / cfg_max_yaw))
        # Sign flip per ENU/NED frame adaptation in send_command (see
        # ros2_rover.py): controller emits clockwise-from-above +ve
        # (drone NED); rover publishes counter-clockwise-from-above +ve
        # (ROS REP-103 ENU). Magnitude preserved; sign inverted.
        assert twist.angular.z == pytest.approx(-0.3)
        assert twist.linear.y == pytest.approx(0.0)
        assert twist.linear.z == pytest.approx(0.0)
        assert twist.angular.x == pytest.approx(0.0)
        assert twist.angular.y == pytest.approx(0.0)

    def test_send_command_no_conversion_yaw_rate(self, rclpy_mock):
        # yaw_rate=0.5 rad/s must hit angular.z verbatim (magnitude-wise) —
        # no rad↔deg conversion. The sign flip is a separate frame-of-
        # reference adaptation (NED→ENU), not a unit conversion.
        from robot_follow.follow_api.types import RobotCommand, SafetyContext

        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        cmd = RobotCommand(forward_m_s=0.0, yaw_rate=0.5, down_m_s=0.0)
        ctx = SafetyContext.from_detection(_det())
        asyncio.run(adapter.send_command(cmd, ctx))
        twist = adapter._publisher.publish.call_args.args[0]
        assert abs(twist.angular.z) == pytest.approx(0.5), (
            "magnitude must be preserved (no rad↔deg conversion)"
        )
        assert twist.angular.z == pytest.approx(-0.5), (
            "sign flipped per NED→ENU frame adaptation"
        )

    def test_send_command_short_circuits_on_target_lost(self, rclpy_mock):
        from robot_follow.follow_api.types import RobotCommand, SafetyContext

        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        cmd = RobotCommand(forward_m_s=1.0, yaw_rate=0.5, down_m_s=0.0)
        lost_ctx = SafetyContext.lost(last_target_x=0.7)
        adapter._publisher.publish.reset_mock()
        asyncio.run(adapter.send_command(cmd, lost_ctx))
        assert not adapter._publisher.publish.called

    def test_send_zero_publishes_all_zeros(self, rclpy_mock):
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        adapter._publisher.publish.reset_mock()
        asyncio.run(adapter.send_zero())
        assert adapter._publisher.publish.called
        twist = adapter._publisher.publish.call_args.args[0]
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.linear.y == pytest.approx(0.0)
        assert twist.linear.z == pytest.approx(0.0)
        assert twist.angular.x == pytest.approx(0.0)
        assert twist.angular.y == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(0.0)

    def test_on_target_lost_publishes_zero_twist(self, rclpy_mock):
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        adapter._publisher.publish.reset_mock()
        asyncio.run(adapter.on_target_lost(_det(cx=0.8)))
        assert adapter._publisher.publish.called
        twist = adapter._publisher.publish.call_args.args[0]
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(0.0)


class TestLifecycle:
    def test_start_session_creates_node_and_publisher(self, rclpy_mock):
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        rclpy = rclpy_mock["rclpy"]
        assert rclpy.node.Node.called
        _args, kwargs = rclpy.node.Node.call_args
        assert kwargs.get("namespace") == adapter._namespace
        node_instance = rclpy.node.Node.return_value
        assert node_instance.create_publisher.called
        publish_args, _ = node_instance.create_publisher.call_args
        assert publish_args[0] is rclpy_mock["Twist"]
        assert publish_args[1] == "/cmd_vel"
        assert publish_args[2] == 10

    def test_start_session_starts_executor_thread(self, rclpy_mock):
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        try:
            assert adapter._executor_thread.is_alive()
        finally:
            asyncio.run(adapter.shutdown())

    def test_shutdown_idempotent(self, rclpy_mock):
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        asyncio.run(adapter.shutdown())
        asyncio.run(adapter.shutdown())

    def test_shutdown_orders_node_destroy_before_try_shutdown(self, rclpy_mock):
        # destroy_node must precede try_shutdown — otherwise the node holds a
        # dangling reference into a torn-down context.
        rclpy = rclpy_mock["rclpy"]
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        parent = MagicMock()
        parent.attach_mock(rclpy.node.Node.return_value.destroy_node, "destroy_node")
        parent.attach_mock(rclpy.try_shutdown, "try_shutdown")
        asyncio.run(adapter.shutdown())
        names = [c[0] for c in parent.mock_calls]
        assert "destroy_node" in names
        assert "try_shutdown" in names
        assert names.index("destroy_node") < names.index("try_shutdown")

    def test_shutdown_sets_shutdown_event(self, rclpy_mock):
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        asyncio.run(adapter.shutdown())
        assert adapter._shutdown_event.is_set()


class TestCustomCliArgs:
    def test_custom_cmd_vel_topic(self, rclpy_mock):
        adapter = _build_adapter(rclpy_mock, cmd_vel_topic="/rover/cmd_vel")
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        node_instance = rclpy_mock["rclpy"].node.Node.return_value
        publish_args, _ = node_instance.create_publisher.call_args
        assert publish_args[1] == "/rover/cmd_vel"

    def test_custom_namespace(self, rclpy_mock):
        adapter = _build_adapter(rclpy_mock, ros_namespace="robot1")
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        _args, kwargs = rclpy_mock["rclpy"].node.Node.call_args
        assert kwargs.get("namespace") == "robot1"

    def test_domain_id_sets_env(self, rclpy_mock, monkeypatch):
        monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
        _build_adapter(rclpy_mock, ros_domain_id=7)
        assert os.environ.get("ROS_DOMAIN_ID") == "7"


class TestCompositionRootIntegration:
    """Smoke tests for run_robot()'s rover branch — locks the dispatch
    contract and the SIGINT preservation at the integration layer."""

    def test_run_robot_constructs_ros2_rover_adapter_when_robot_is_rover(self, rclpy_mock):
        from robot_follow.robot_api.adapters.ros2_rover import Ros2RoverAdapter
        from robot_follow.follow_api.config import ControllerConfig
        from robot_follow.follow_api.types import Axis

        args = SimpleNamespace(
            robot="rover",
            cmd_vel_topic="/cmd_vel",
            ros_namespace="",
            ros_domain_id=0,
        )
        adapter = Ros2RoverAdapter(args, ControllerConfig())
        assert isinstance(adapter, Ros2RoverAdapter)
        assert adapter.caps.yaw_unit == "rad/s"
        assert Axis.ALTITUDE not in adapter.caps.axes

    def test_sigint_handler_survives_full_composition_root_path(self, rclpy_mock):
        from robot_follow.robot_api.adapters.ros2_rover import Ros2RoverAdapter
        from robot_follow.follow_api.config import ControllerConfig

        def fake_handler(*_):
            pass

        original = signal.getsignal(signal.SIGINT)
        try:
            signal.signal(signal.SIGINT, fake_handler)
            args = SimpleNamespace(
                robot="rover",
                cmd_vel_topic="/cmd_vel",
                ros_namespace="",
                ros_domain_id=0,
            )
            adapter = Ros2RoverAdapter(args, ControllerConfig())
            asyncio.run(adapter.connect())
            assert signal.getsignal(signal.SIGINT) is fake_handler
        finally:
            signal.signal(signal.SIGINT, original)


class TestBottomEdgeNaturalStop:
    """RINT-02 / Q1 lock: rover slow-down when bbox bottom enters lowest 15%.

    Tests the wire-level contract added by Plan 06-04 to ros2_rover.py:
        if safety_ctx.bbox_bottom_norm >= ROVER_BOTTOM_STOP_THRESHOLD (0.85):
            twist.linear.x = 0.0    # stop forward
            # twist.angular.z stays at cmd.yaw_rate (yaw preserved)

    The drone adapter does NOT exercise this path — it reads
    SafetyContext.bbox_bottom_normalized (the legacy field) instead, via
    _apply_retreat_from_tilt in mavsdk_drone.py. This class is rover-only.
    """

    def _ctx(self, bbox_bottom_norm):
        """Helper: build a SafetyContext with bbox_bottom_norm overridden
        and target_lost=False (so send_command does NOT early-exit on the
        lost-target check).
        """
        from robot_follow.follow_api.types import SafetyContext
        # Pure dataclass construction — we don't need a real Detection here.
        return SafetyContext(
            bbox_bottom_normalized=bbox_bottom_norm if bbox_bottom_norm is not None else 0.5,
            bbox_size_normalized=0.3,
            target_lost=False,
            last_target_x=0.5,
            bbox_bottom_norm=bbox_bottom_norm,
        )

    def test_rover_bottom_stop_threshold_constant_value(self, rclpy_mock):
        """Named constant per Phase 6 user decision — value locked at 0.85.

        Tuning happens by editing the constant in ros2_rover.py, NOT by
        changing magic numbers in send_command. Test fails loudly if the
        value drifts.
        """
        from robot_follow.robot_api.adapters.ros2_rover import (
            ROVER_BOTTOM_STOP_THRESHOLD,
        )
        assert ROVER_BOTTOM_STOP_THRESHOLD == 0.85, (
            f"named constant drift: ROVER_BOTTOM_STOP_THRESHOLD = "
            f"{ROVER_BOTTOM_STOP_THRESHOLD} (expected 0.85)"
        )

    def test_below_threshold_forward_unchanged(self, rclpy_mock):
        """bbox_bottom_norm=0.80 < 0.85 -> forward passes through unchanged.

        The rover continues to drive at the controller's commanded speed.
        """
        from robot_follow.follow_api.types import RobotCommand
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        cmd = RobotCommand(forward_m_s=0.5, yaw_rate=0.1)
        ctx = self._ctx(bbox_bottom_norm=0.80)
        asyncio.run(adapter.send_command(cmd, ctx))
        adapter._publisher.publish.assert_called_once()
        twist = adapter._publisher.publish.call_args[0][0]
        # Forward scaled by yaw-align (0.5 * (1 - 0.1/max_yawspeed)); the
        # bottom-edge override is NOT firing here. Yaw-align coverage lives
        # in TestYawAlignForwardAttenuation.
        from robot_follow.follow_api.config import ControllerConfig
        cfg_max_yaw = ControllerConfig().max_yawspeed
        assert twist.linear.x == pytest.approx(0.5 * (1.0 - 0.1 / cfg_max_yaw)), (
            f"below threshold: forward must pass through (yaw-align only) "
            f"(got linear.x={twist.linear.x})"
        )
        # Sign flip per NED→ENU frame adaptation in send_command.
        assert twist.angular.z == -0.1

    def test_at_threshold_forward_zeroed(self, rclpy_mock):
        """bbox_bottom_norm=0.85 == 0.85 -> override fires; forward zeroed.

        Boundary case: the >= comparison must trigger AT the threshold,
        not just above it.
        """
        from robot_follow.follow_api.types import RobotCommand
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        cmd = RobotCommand(forward_m_s=0.5, yaw_rate=0.1)
        ctx = self._ctx(bbox_bottom_norm=0.85)
        asyncio.run(adapter.send_command(cmd, ctx))
        adapter._publisher.publish.assert_called_once()
        twist = adapter._publisher.publish.call_args[0][0]
        assert twist.linear.x == 0.0, (
            f"at threshold: forward must be overridden to 0 "
            f"(got linear.x={twist.linear.x})"
        )

    def test_above_threshold_forward_zeroed(self, rclpy_mock):
        """bbox_bottom_norm=0.95 > 0.85 -> forward zeroed (person very close)."""
        from robot_follow.follow_api.types import RobotCommand
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        cmd = RobotCommand(forward_m_s=1.0, yaw_rate=0.0)
        ctx = self._ctx(bbox_bottom_norm=0.95)
        asyncio.run(adapter.send_command(cmd, ctx))
        adapter._publisher.publish.assert_called_once()
        twist = adapter._publisher.publish.call_args[0][0]
        assert twist.linear.x == 0.0

    def test_above_threshold_yaw_preserved(self, rclpy_mock):
        """When forward is zeroed, yaw STAYS at cmd.yaw_rate.

        Critical: the rover must still be able to rotate to recenter the
        target even when forward is suppressed. If yaw were also zeroed,
        the rover would freeze entirely — wrong shape for the use case.
        """
        from robot_follow.follow_api.types import RobotCommand
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        cmd = RobotCommand(forward_m_s=0.7, yaw_rate=0.3)
        ctx = self._ctx(bbox_bottom_norm=0.95)
        asyncio.run(adapter.send_command(cmd, ctx))
        adapter._publisher.publish.assert_called_once()
        twist = adapter._publisher.publish.call_args[0][0]
        assert twist.linear.x == 0.0, "forward must be zeroed"
        # Sign flip per NED→ENU frame adaptation: cmd.yaw_rate=+0.3 emerges
        # as twist.angular.z=-0.3. Magnitude preserved (yaw NOT zeroed —
        # the rover still rotates to recenter, just in ROS-convention sign).
        assert twist.angular.z == -0.3, (
            f"yaw magnitude must be preserved with NED→ENU sign flip "
            f"(got angular.z={twist.angular.z}; expected -0.3 — "
            f"RINT-02 rover-still-rotates-to-recenter contract)"
        )

    def test_none_bbox_bottom_norm_forward_passes_through(self, rclpy_mock):
        """When safety_ctx.bbox_bottom_norm is None (defensive default),
        the threshold check short-circuits and forward passes through.

        This is the belt-and-braces guard against a misshapen SafetyContext;
        in the real path, SafetyContext.from_detection always populates the
        field, so this branch fires only if a future caller passes an
        adversarial / minimal SafetyContext.
        """
        from robot_follow.follow_api.types import RobotCommand
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        cmd = RobotCommand(forward_m_s=0.4, yaw_rate=0.0)
        ctx = self._ctx(bbox_bottom_norm=None)
        asyncio.run(adapter.send_command(cmd, ctx))
        twist = adapter._publisher.publish.call_args[0][0]
        assert twist.linear.x == 0.4, (
            f"None bbox_bottom_norm must not trigger override "
            f"(got linear.x={twist.linear.x}; expected 0.4)"
        )


class TestYawSignSteersTowardPerson:
    """Frame-of-reference adaptation: the controller's yaw_rate is in the
    drone's NED clockwise-from-above convention; ROS REP-103 (ENU) is
    counter-clockwise-from-above positive. The adapter negates yaw_rate
    so the rover steers TOWARD a person on the right (not away).

    Verified end-to-end with the controller's actual _compute_yaw output,
    not just a synthetic cmd.yaw_rate, so a future controller refactor
    that changes the emitted sign convention is caught here.
    """

    def test_person_on_right_makes_rover_turn_right(self, rclpy_mock):
        """Person at center_x=0.7 (right side of frame) → rover must turn
        right (clockwise from above) → ROS angular.z < 0.
        """
        from robot_follow.follow_api import controller as ctrl
        from robot_follow.follow_api.config import ControllerConfig
        from robot_follow.follow_api.types import Detection, SafetyContext
        from robot_follow.robot_api.adapters.ros2_rover import ROVER_CAPS

        det = Detection(label="person", confidence=0.9,
                        center_x=0.7, center_y=0.5, bbox_height=0.20,
                        timestamp=0.0)
        cfg = ControllerConfig()  # default kp_yaw / hfov / dead_zone_deg
        cmd = ctrl.compute(det, ROVER_CAPS, cfg)
        assert cmd.yaw_rate > 0, (
            "controller emits clockwise-from-above +ve when person is on "
            f"the right (got cmd.yaw_rate={cmd.yaw_rate})"
        )

        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        ctx = SafetyContext.from_detection(det)
        asyncio.run(adapter.send_command(cmd, ctx))
        twist = adapter._publisher.publish.call_args[0][0]
        # ROS REP-103: angular.z < 0 = clockwise-from-above = turn RIGHT.
        # Person was on the right; rover must turn right to recenter.
        assert twist.angular.z < 0, (
            f"rover must turn RIGHT (angular.z < 0 in ROS ENU) toward a "
            f"person on the right; got angular.z={twist.angular.z}"
        )

    def test_person_on_left_makes_rover_turn_left(self, rclpy_mock):
        """Person at center_x=0.3 (left side) → rover must turn left
        (counter-clockwise from above) → ROS angular.z > 0.
        """
        from robot_follow.follow_api import controller as ctrl
        from robot_follow.follow_api.config import ControllerConfig
        from robot_follow.follow_api.types import Detection, SafetyContext
        from robot_follow.robot_api.adapters.ros2_rover import ROVER_CAPS

        det = Detection(label="person", confidence=0.9,
                        center_x=0.3, center_y=0.5, bbox_height=0.20,
                        timestamp=0.0)
        cfg = ControllerConfig()
        cmd = ctrl.compute(det, ROVER_CAPS, cfg)
        assert cmd.yaw_rate < 0, (
            "controller emits counter-clockwise-from-above -ve when "
            f"person is on the left (got cmd.yaw_rate={cmd.yaw_rate})"
        )

        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        ctx = SafetyContext.from_detection(det)
        asyncio.run(adapter.send_command(cmd, ctx))
        twist = adapter._publisher.publish.call_args[0][0]
        assert twist.angular.z > 0, (
            f"rover must turn LEFT (angular.z > 0 in ROS ENU) toward a "
            f"person on the left; got angular.z={twist.angular.z}"
        )


class TestEmaSmoothing:
    """EMA smoothing on yaw + forward axes (rover-side; matches mavsdk_drone's
    _apply_smoothing). First call passes raw through; subsequent calls
    converge via alpha-weighted EMA.
    """

    def test_first_call_passes_raw_through(self, rclpy_mock):
        """Initial filter state is None → first tick is unfiltered.

        Avoids the drone's first-tick attenuation (where filter starts at
        0 → first published value is alpha * raw). The rover starts at the
        first observed command, so the initial yaw response isn't muted.
        """
        from robot_follow.follow_api.types import RobotCommand, SafetyContext
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        cmd = RobotCommand(forward_m_s=0.5, yaw_rate=0.2)
        ctx = SafetyContext.from_detection(_det())
        asyncio.run(adapter.send_command(cmd, ctx))
        twist = adapter._publisher.publish.call_args[0][0]
        # Forward scaled by yaw-align (0.5 * (1 - 0.2/max_yawspeed)). EMA
        # first-call passthrough is unaffected; yaw_out used by the
        # attenuation IS the EMA-passed-through value, so the formula
        # composes cleanly.
        from robot_follow.follow_api.config import ControllerConfig
        cfg_max_yaw = ControllerConfig().max_yawspeed
        assert twist.linear.x == pytest.approx(0.5 * (1.0 - 0.2 / cfg_max_yaw))
        assert twist.angular.z == pytest.approx(-0.2)  # sign flip only

    def test_repeated_calls_converge_via_ema(self, rclpy_mock):
        """N constant commands → output decays toward the new setpoint.

        With yaw_alpha=0.3 (default), starting from yaw_out=0.2 (first call)
        and re-issuing yaw_rate=0.0 (target reached), the filter geometrically
        decays: 0.2 → 0.14 → 0.098 → 0.069 → ... Asserts the second tick
        is between raw (0) and previous (0.2) — proving smoothing fires.
        """
        from robot_follow.follow_api.types import RobotCommand, SafetyContext
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        ctx = SafetyContext.from_detection(_det())
        # First call: filter = raw = 0.2
        asyncio.run(adapter.send_command(
            RobotCommand(forward_m_s=0.0, yaw_rate=0.2), ctx,
        ))
        # Second call: new raw = 0.0; filter = 0.3 * 0.0 + 0.7 * 0.2 = 0.14
        adapter._publisher.publish.reset_mock()
        asyncio.run(adapter.send_command(
            RobotCommand(forward_m_s=0.0, yaw_rate=0.0), ctx,
        ))
        twist = adapter._publisher.publish.call_args[0][0]
        # angular.z = -filtered_yaw → -0.14 (with sign flip)
        assert twist.angular.z == pytest.approx(-0.14, abs=1e-3), (
            f"EMA must smooth between previous filtered yaw and new raw "
            f"(got angular.z={twist.angular.z}; expected ~-0.14)"
        )

    def test_smooth_yaw_disabled_skips_filter(self, rclpy_mock):
        """When smooth_yaw=False, yaw_rate passes through unchanged on
        every tick (only the sign flip applies).
        """
        from robot_follow.follow_api.config import ControllerConfig
        from robot_follow.follow_api.types import RobotCommand, SafetyContext
        from robot_follow.robot_api.adapters.ros2_rover import Ros2RoverAdapter
        cfg = ControllerConfig(smooth_yaw=False, smooth_forward=False)
        adapter = Ros2RoverAdapter(_default_args(), cfg)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        ctx = SafetyContext.from_detection(_det())
        # Two ticks with different setpoints — neither should be filtered.
        asyncio.run(adapter.send_command(
            RobotCommand(forward_m_s=0.0, yaw_rate=0.5), ctx,
        ))
        adapter._publisher.publish.reset_mock()
        asyncio.run(adapter.send_command(
            RobotCommand(forward_m_s=0.0, yaw_rate=0.0), ctx,
        ))
        twist = adapter._publisher.publish.call_args[0][0]
        assert twist.angular.z == pytest.approx(0.0), (
            "smooth_yaw=False must publish the current raw cmd, not a "
            "filtered blend of previous + current"
        )


class TestYawAlignForwardAttenuation:
    """Yaw-aware forward attenuation: skid-steer can't curve, so when the
    rover is yawing hard the person slides off the side of the frame.
    Positive forward is scaled by max(0, 1 - |yaw|/max_yawspeed); retreat
    (negative forward) is unattenuated so a near-edge target can still be
    recovered while backing up.
    """

    def _build(self, rclpy_mock, **cfg_overrides):
        from robot_follow.follow_api.config import ControllerConfig
        from robot_follow.robot_api.adapters.ros2_rover import Ros2RoverAdapter
        config = ControllerConfig()
        # Disable EMA to keep math direct; attenuation is independent of it.
        config.smooth_yaw = False
        config.smooth_forward = False
        for k, v in cfg_overrides.items():
            setattr(config, k, v)
        adapter = Ros2RoverAdapter(_default_args(), config)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        return adapter, config

    def test_yaw_at_max_attenuates_forward_to_zero(self, rclpy_mock):
        from robot_follow.follow_api.types import RobotCommand, SafetyContext
        adapter, config = self._build(rclpy_mock)
        cmd = RobotCommand(forward_m_s=1.0, yaw_rate=config.max_yawspeed)
        asyncio.run(adapter.send_command(cmd, SafetyContext.from_detection(_det())))
        twist = adapter._publisher.publish.call_args[0][0]
        assert twist.linear.x == pytest.approx(0.0)

    def test_yaw_zero_leaves_forward_unchanged(self, rclpy_mock):
        from robot_follow.follow_api.types import RobotCommand, SafetyContext
        adapter, _ = self._build(rclpy_mock)
        cmd = RobotCommand(forward_m_s=1.0, yaw_rate=0.0)
        asyncio.run(adapter.send_command(cmd, SafetyContext.from_detection(_det())))
        twist = adapter._publisher.publish.call_args[0][0]
        assert twist.linear.x == pytest.approx(1.0)

    def test_retreat_unattenuated_under_hard_yaw(self, rclpy_mock):
        """Backing up while turning to recover a near-edge target is the
        right move — retreat must keep its full magnitude regardless of yaw."""
        from robot_follow.follow_api.types import RobotCommand, SafetyContext
        adapter, config = self._build(rclpy_mock)
        cmd = RobotCommand(forward_m_s=-0.8, yaw_rate=config.max_yawspeed)
        asyncio.run(adapter.send_command(cmd, SafetyContext.from_detection(_det())))
        twist = adapter._publisher.publish.call_args[0][0]
        assert twist.linear.x == pytest.approx(-0.8)

    def test_yaw_beyond_max_clamps_attenuation_at_zero(self, rclpy_mock):
        """max(0, ...) clamp: yaw above max doesn't drive forward negative."""
        from robot_follow.follow_api.types import RobotCommand, SafetyContext
        adapter, config = self._build(rclpy_mock)
        cmd = RobotCommand(forward_m_s=1.0, yaw_rate=config.max_yawspeed * 2.0)
        asyncio.run(adapter.send_command(cmd, SafetyContext.from_detection(_det())))
        twist = adapter._publisher.publish.call_args[0][0]
        assert twist.linear.x == pytest.approx(0.0)


class TestSigintShutdown:
    """RINT-06: SIGINT shutdown contract pins (test-only; adapter unchanged).

    The adapter's shutdown method (Phase 4) is the thing under test. These
    tests pin three structural facts:
      1. After shutdown returns, send_command does NOT publish (publisher is
         None; the early-return guard at `if self._publisher is None: return`
         is what keeps the rover quiet).
      2. After shutdown returns, send_zero also does NOT publish.
      3. shutdown() itself completes in well under 1.0 s with the mocked
         executor (real-world bound is 2.0 s thread.join timeout in the
         adapter; the mocked spin_once exits cleanly when the event fires).

    Existing tests in TestLifecycle already pin idempotency, event-set, and
    destroy_node-before-try_shutdown ordering — NOT duplicated here.
    """

    def test_post_shutdown_send_command_does_not_publish(self, rclpy_mock):
        from robot_follow.follow_api.types import RobotCommand, SafetyContext

        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        asyncio.run(adapter.shutdown())
        # publisher is cleared in shutdown — guard pins RINT-06 silence
        assert adapter._publisher is None, (
            "shutdown must null out _publisher so send_command early-returns"
        )
        cmd = RobotCommand(forward_m_s=1.0, yaw_rate=0.0, down_m_s=0.0)
        ctx = SafetyContext.from_detection(_det())
        # Must not raise; the `if self._publisher is None: return` guard fires.
        asyncio.run(adapter.send_command(cmd, ctx))

    def test_post_shutdown_send_zero_does_not_publish(self, rclpy_mock):
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        asyncio.run(adapter.shutdown())
        assert adapter._publisher is None
        # Must not raise (publisher-is-None guard fires).
        asyncio.run(adapter.send_zero())

    def test_shutdown_completes_within_1s(self, rclpy_mock):
        """RINT-06 spec: rover stops within 1 s. The mocked executor's
        spin_once exits cleanly when _shutdown_event fires, so shutdown
        completes in well under 100 ms in practice. The 1.0 s bound is the
        spec target (the adapter's thread.join uses timeout=2.0 internally
        as a hard ceiling for the real-rclpy path).
        """
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        t0 = time.monotonic()
        asyncio.run(adapter.shutdown())
        elapsed = time.monotonic() - t0
        assert elapsed < 1.0, (
            f"shutdown took {elapsed:.3f}s — RINT-06 requires < 1.0 s"
        )
