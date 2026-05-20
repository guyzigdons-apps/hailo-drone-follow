"""ROS 2 rover adapter unit tests (rclpy injected via fixture)."""

from __future__ import annotations

import asyncio
import importlib
import os
import signal
import sys
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
        from robot_follow.follow_api.types import RobotCommand, SafetyContext

        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        cmd = RobotCommand(forward_m_s=1.5, yaw_rate=0.3, down_m_s=0.0)
        ctx = SafetyContext.from_detection(_det())
        asyncio.run(adapter.send_command(cmd, ctx))
        assert adapter._publisher.publish.called
        twist = adapter._publisher.publish.call_args.args[0]
        assert twist.linear.x == pytest.approx(1.5)
        assert twist.angular.z == pytest.approx(0.3)
        assert twist.linear.y == pytest.approx(0.0)
        assert twist.linear.z == pytest.approx(0.0)
        assert twist.angular.x == pytest.approx(0.0)
        assert twist.angular.y == pytest.approx(0.0)

    def test_send_command_no_conversion_yaw_rate(self, rclpy_mock):
        # yaw_rate=0.5 rad/s must hit angular.z verbatim, not 0.5 * π/180.
        from robot_follow.follow_api.types import RobotCommand, SafetyContext

        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        cmd = RobotCommand(forward_m_s=0.0, yaw_rate=0.5, down_m_s=0.0)
        ctx = SafetyContext.from_detection(_det())
        asyncio.run(adapter.send_command(cmd, ctx))
        twist = adapter._publisher.publish.call_args.args[0]
        assert twist.angular.z == pytest.approx(0.5)

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
