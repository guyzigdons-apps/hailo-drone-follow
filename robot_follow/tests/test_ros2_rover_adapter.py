"""Phase 4 Wave 0 test scaffold for the ROS 2 rover adapter.

This file is the failing contract for Plan 04-03 (Wave 3): every test
in this module is marked ``@pytest.mark.xfail(strict=False, …)`` until
``robot_follow/robot_api/adapters/ros2_rover.py`` lands. When 04-03
adds the adapter and these xfails start passing, the markers are
stripped en bloc.

Why this scaffold lands before the adapter:
    - GSD test-first contract — establish the failing contract before
      any production code lands (see ``.planning/phases/04-rover-adapter/RESEARCH.md``
      § "Migration commit shape Wave 0").
    - Locks the wire shape (Twist semantics, rclpy lifecycle order,
      protocol fit, signal-handler preservation) so 04-03 has a single
      target to satisfy.
    - Documents the rclpy mock strategy now — Plan 04-03 reuses the
      same ``rclpy_mock`` fixture without rewriting it.

Reference: ``.planning/phases/04-rover-adapter/RESEARCH.md`` §§
"rclpy mock fixture (the foundation)", "Test classes", and
"Test count summary" — this file mirrors that map exactly:

    | Class                          | # tests |
    | TestImportSafety               | 2       |
    | TestProtocolShape              | 3       |
    | TestSignalHandlerPreservation  | 2       |
    | TestTwistPublish               | 5       |
    | TestLifecycle                  | 5       |
    | TestCustomCliArgs              | 3       |
    | **Total**                      | **20**  |

The ``rclpy_mock`` fixture uses ``monkeypatch.setitem(sys.modules, …)``
(per RESEARCH § 7 / threat T-04-01-01) so the injections roll back at
teardown and cannot leak into other tests in the suite.
"""

from __future__ import annotations

import asyncio
import importlib
import os
import signal
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

# Reason strings — Plan 04-03 strips the xfail markers wholesale.
XFAIL_REASON_ADAPTER = (
    "ros2_rover.py + Ros2RoverAdapter land in Phase 4 Plan 04-03 "
    "(adapter implementation). This Wave 0 scaffold makes the failing "
    "contract explicit; 04-03 strips these xfail markers."
)


# ---------------------------------------------------------------------------
# rclpy_mock fixture
# ---------------------------------------------------------------------------


@pytest.fixture
def rclpy_mock(monkeypatch):
    """Inject MagicMock stand-ins for rclpy + geometry_msgs into ``sys.modules``.

    Must be applied BEFORE the adapter is constructed — the adapter's
    ``__init__`` does ``import rclpy`` and ``from geometry_msgs.msg
    import Twist`` lazily, so with these mocks in ``sys.modules`` the
    lazy imports succeed and the adapter constructs cleanly on this
    no-rclpy dev box.

    Uses ``monkeypatch.setitem`` so each entry is auto-removed at
    teardown (threat T-04-01-01: prevent mock leakage between tests).
    The friendly-error test deliberately does NOT use this fixture; it
    deletes ``rclpy`` from ``sys.modules`` to exercise the import-failure
    path.
    """
    rclpy = MagicMock(name="rclpy")
    rclpy.signals.SignalHandlerOptions.NO = "NO"  # sentinel
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


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _det(cx=0.5, cy=0.5, bh=0.3):
    """Build a Detection (mirrors test_mavsdk_drone_adapter.py:49-57)."""
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
    """Default argparse Namespace for adapter construction.

    Plan 04-02 adds these flags via ``add_rover_args``; until then the
    namespace just carries the defaults the adapter would read.
    """
    defaults = dict(
        cmd_vel_topic="/cmd_vel",
        ros_namespace="",
        ros_domain_id=0,
    )
    defaults.update(overrides)
    return SimpleNamespace(**defaults)


def _build_adapter(rclpy_mock, **arg_overrides):
    """Construct ``Ros2RoverAdapter`` against the mocked rclpy.

    Imports the adapter lazily so collection succeeds without the
    module existing yet (collection-time imports would error and break
    the whole file). Plan 04-03 ships ``ros2_rover.py``; until then
    every test that calls this helper xfails on the ``ImportError``.
    """
    from robot_follow.follow_api.config import ControllerConfig
    from robot_follow.robot_api.adapters.ros2_rover import Ros2RoverAdapter

    args = _default_args(**arg_overrides)
    return Ros2RoverAdapter(args, ControllerConfig())


# ---------------------------------------------------------------------------
# TestImportSafety — 2 tests
# ---------------------------------------------------------------------------


class TestImportSafety:
    """Module import safety and friendly-error behavior."""

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_module_imports_cleanly_without_rclpy(self):
        """``import robot_follow.robot_api.adapters.ros2_rover`` succeeds on a
        no-ROS box (the lazy import is INSIDE ``__init__``, not at module
        top). Does NOT use the ``rclpy_mock`` fixture — that's the
        whole point of this test."""
        importlib.import_module("robot_follow.robot_api.adapters.ros2_rover")

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_friendly_error_when_rclpy_missing(self, monkeypatch):
        """Constructing the adapter without rclpy in ``sys.modules``
        raises RuntimeError whose message names ROS 2 and the source
        command (PITFALLS Pitfall 2). Explicitly REMOVES the mocks —
        does NOT use the ``rclpy_mock`` fixture."""
        from robot_follow.follow_api.config import ControllerConfig
        from robot_follow.robot_api.adapters.ros2_rover import Ros2RoverAdapter

        monkeypatch.delitem(sys.modules, "rclpy", raising=False)
        with pytest.raises(RuntimeError) as exc_info:
            Ros2RoverAdapter(_default_args(), ControllerConfig())
        msg = str(exc_info.value)
        assert "ROS 2 not" in msg, f"missing 'ROS 2 not' in: {msg!r}"
        assert "source /opt/ros/humble/setup.bash" in msg, (
            f"missing setup-bash hint in: {msg!r}"
        )


# ---------------------------------------------------------------------------
# TestProtocolShape — 3 tests
# ---------------------------------------------------------------------------


class TestProtocolShape:
    """``Ros2RoverAdapter`` must structurally implement the Robot protocol."""

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_implements_robot_protocol(self, rclpy_mock):
        """``isinstance(adapter, Robot)`` works via @runtime_checkable."""
        from robot_follow.robot_api.robot import Robot

        adapter = _build_adapter(rclpy_mock)
        assert isinstance(adapter, Robot)

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_caps_is_rover_caps(self, rclpy_mock):
        """caps.axes == {FORWARD, YAW}; yaw_unit == 'rad/s'; ALTITUDE absent."""
        from robot_follow.follow_api.types import Axis
        from robot_follow.robot_api.adapters.ros2_rover import ROVER_CAPS

        adapter = _build_adapter(rclpy_mock)
        assert adapter.caps is ROVER_CAPS
        assert adapter.caps.axes == frozenset({Axis.FORWARD, Axis.YAW})
        assert adapter.caps.yaw_unit == "rad/s"
        assert Axis.ALTITUDE not in adapter.caps.axes

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_has_six_methods(self, rclpy_mock):
        """Mirrors test_robot_protocol_shape.py:39-48 — hasattr loop."""
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


# ---------------------------------------------------------------------------
# TestSignalHandlerPreservation — 2 tests (ROVER-02, ROVER-08)
# ---------------------------------------------------------------------------


class TestSignalHandlerPreservation:
    """ROVER-02 / ROVER-08 — rclpy.init must NOT clobber the SIGINT handler."""

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_connect_calls_init_with_signal_handler_options_no(self, rclpy_mock):
        """After ``await adapter.connect()``, ``rclpy.init`` was called
        with the ``signal_handler_options="NO"`` sentinel."""
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        rclpy = rclpy_mock["rclpy"]
        # Inspect kwargs of the most recent init call.
        assert rclpy.init.called, "rclpy.init was never called"
        _args, kwargs = rclpy.init.call_args
        assert kwargs.get("signal_handler_options") == "NO", (
            f"expected signal_handler_options='NO', got {kwargs!r}"
        )

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_sigint_handler_survives_connect(self, rclpy_mock):
        """ROVER-08 smoke: set a fake SIGINT handler, await connect(),
        assert the handler is still bound. Mocked rclpy never touches
        signal state, but this test locks the contract so the production
        adapter's own ``assert before is after`` is exercised."""
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


# ---------------------------------------------------------------------------
# TestTwistPublish — 5 tests (ROVER-06)
# ---------------------------------------------------------------------------


class TestTwistPublish:
    """ROVER-06 — ``send_command`` writes a Twist with no unit conversion."""

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_publishes_twist_with_forward_and_yaw_rate(self, rclpy_mock):
        """forward_m_s=1.5, yaw_rate=0.3 → linear.x=1.5, angular.z=0.3,
        all other Twist components = 0.0."""
        from robot_follow.follow_api.types import RobotCommand, SafetyContext

        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        cmd = RobotCommand(forward_m_s=1.5, yaw_rate=0.3, down_m_s=0.0)
        ctx = SafetyContext.from_detection(_det())
        asyncio.run(adapter.send_command(cmd, ctx))
        assert adapter._publisher.publish.called, "publisher.publish was not called"
        twist = adapter._publisher.publish.call_args.args[0]
        assert twist.linear.x == pytest.approx(1.5)
        assert twist.angular.z == pytest.approx(0.3)
        assert twist.linear.y == pytest.approx(0.0)
        assert twist.linear.z == pytest.approx(0.0)
        assert twist.angular.x == pytest.approx(0.0)
        assert twist.angular.y == pytest.approx(0.0)

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_send_command_no_conversion_yaw_rate(self, rclpy_mock):
        """yaw_rate=0.5 rad/s → angular.z==0.5 (NOT 0.5 * π/180).
        Locks ROVER-06: rover adapter publishes yaw_rate verbatim
        because RobotCommand.yaw_rate is already in rad/s — no deg→rad
        conversion (unlike the drone adapter which does deg/s)."""
        from robot_follow.follow_api.types import RobotCommand, SafetyContext

        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        cmd = RobotCommand(forward_m_s=0.0, yaw_rate=0.5, down_m_s=0.0)
        ctx = SafetyContext.from_detection(_det())
        asyncio.run(adapter.send_command(cmd, ctx))
        twist = adapter._publisher.publish.call_args.args[0]
        assert twist.angular.z == pytest.approx(0.5)

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_send_command_short_circuits_on_target_lost(self, rclpy_mock):
        """Q6 lock: target_lost=True → publisher.publish is NOT called."""
        from robot_follow.follow_api.types import RobotCommand, SafetyContext

        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        cmd = RobotCommand(forward_m_s=1.0, yaw_rate=0.5, down_m_s=0.0)
        lost_ctx = SafetyContext.lost(last_target_x=0.7)
        # Reset publish mock so we measure only this call.
        adapter._publisher.publish.reset_mock()
        asyncio.run(adapter.send_command(cmd, lost_ctx))
        assert not adapter._publisher.publish.called, (
            "publish should not be called when target is lost"
        )

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_send_zero_publishes_all_zeros(self, rclpy_mock):
        """``send_zero`` publishes a Twist with all 6 fields = 0.0."""
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

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_on_target_lost_publishes_zero_twist(self, rclpy_mock):
        """Rover does NOT yaw-spin on target loss — ``on_target_lost``
        publishes an all-zero Twist (unlike the drone, which spins
        toward last-seen side)."""
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        adapter._publisher.publish.reset_mock()
        asyncio.run(adapter.on_target_lost(_det(cx=0.8)))
        assert adapter._publisher.publish.called
        twist = adapter._publisher.publish.call_args.args[0]
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# TestLifecycle — 5 tests
# ---------------------------------------------------------------------------


class TestLifecycle:
    """connect / start_session / shutdown ordering and idempotency."""

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_start_session_creates_node_and_publisher(self, rclpy_mock):
        """Node called with namespace=self._namespace; create_publisher
        called with (Twist, '/cmd_vel', 10)."""
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        rclpy = rclpy_mock["rclpy"]
        # The adapter constructs Node(namespace=...); inspect call kwargs.
        assert rclpy.node.Node.called, "Node constructor was never called"
        _args, kwargs = rclpy.node.Node.call_args
        assert kwargs.get("namespace") == adapter._namespace
        # The Node instance must have had create_publisher called on it
        # with (Twist, "/cmd_vel", 10). Probe via the node mock.
        node_instance = rclpy.node.Node.return_value
        assert node_instance.create_publisher.called
        publish_args, _pub_kwargs = node_instance.create_publisher.call_args
        # Args: (Twist, topic, qos_depth)
        assert publish_args[0] is rclpy_mock["Twist"]
        assert publish_args[1] == "/cmd_vel"
        assert publish_args[2] == 10

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_start_session_starts_executor_thread(self, rclpy_mock):
        """After start_session(), adapter._executor_thread.is_alive()."""
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        try:
            assert adapter._executor_thread.is_alive()
        finally:
            asyncio.run(adapter.shutdown())

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_shutdown_idempotent(self, rclpy_mock):
        """Calling shutdown() twice in a row must not raise."""
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        asyncio.run(adapter.shutdown())
        # Second call must be a no-op (not raise).
        asyncio.run(adapter.shutdown())

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_shutdown_orders_node_destroy_before_try_shutdown(self, rclpy_mock):
        """PITFALLS 'Looks Done But Isn't' line 197 — destroy_node must
        be called BEFORE rclpy.try_shutdown(); record mock_calls and
        compare indices."""
        rclpy = rclpy_mock["rclpy"]
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        # Track call order across both mocks via a parent mock.
        parent = MagicMock()
        parent.attach_mock(rclpy.node.Node.return_value.destroy_node, "destroy_node")
        parent.attach_mock(rclpy.try_shutdown, "try_shutdown")
        asyncio.run(adapter.shutdown())
        names = [c[0] for c in parent.mock_calls]
        assert "destroy_node" in names, f"destroy_node never called: {names!r}"
        assert "try_shutdown" in names, f"try_shutdown never called: {names!r}"
        assert names.index("destroy_node") < names.index("try_shutdown"), (
            f"destroy_node must precede try_shutdown; saw: {names!r}"
        )

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_shutdown_sets_shutdown_event(self, rclpy_mock):
        """After shutdown(), adapter._shutdown_event.is_set() is True."""
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        asyncio.run(adapter.shutdown())
        assert adapter._shutdown_event.is_set()


# ---------------------------------------------------------------------------
# TestCustomCliArgs — 3 tests (ROVER-05)
# ---------------------------------------------------------------------------


class TestCustomCliArgs:
    """ROVER-05 — --cmd-vel-topic / --ros-namespace / --ros-domain-id."""

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_custom_cmd_vel_topic(self, rclpy_mock):
        """args.cmd_vel_topic='/rover/cmd_vel' → create_publisher called
        with that topic."""
        adapter = _build_adapter(rclpy_mock, cmd_vel_topic="/rover/cmd_vel")
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        rclpy = rclpy_mock["rclpy"]
        node_instance = rclpy.node.Node.return_value
        publish_args, _ = node_instance.create_publisher.call_args
        assert publish_args[1] == "/rover/cmd_vel"

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_custom_namespace(self, rclpy_mock):
        """args.ros_namespace='robot1' → Node called with namespace='robot1'."""
        adapter = _build_adapter(rclpy_mock, ros_namespace="robot1")
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        rclpy = rclpy_mock["rclpy"]
        _args, kwargs = rclpy.node.Node.call_args
        assert kwargs.get("namespace") == "robot1"

    @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
    def test_domain_id_sets_env(self, rclpy_mock, monkeypatch):
        """args.ros_domain_id=7 → os.environ['ROS_DOMAIN_ID']=='7'."""
        monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
        adapter = _build_adapter(rclpy_mock, ros_domain_id=7)
        # Construction (or connect) must have set the env var.
        # We accept either timing; check after connect just in case.
        asyncio.run(adapter.connect())
        assert os.environ.get("ROS_DOMAIN_ID") == "7"
