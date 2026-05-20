"""ROS 2 rover adapter publishing geometry_msgs/Twist on /cmd_vel."""

from __future__ import annotations

import logging
import os
import signal
import threading
from typing import TYPE_CHECKING, Optional

from robot_follow.follow_api.types import (
    Axis,
    Capabilities,
    Detection,
    RobotCommand,
    SafetyContext,
)

if TYPE_CHECKING:
    from robot_follow.follow_api.config import ControllerConfig

LOGGER = logging.getLogger(__name__)


ROVER_CAPS: Capabilities = Capabilities(
    axes=frozenset({Axis.FORWARD, Axis.YAW}),
    yaw_unit="rad/s",
)


class Ros2RoverAdapter:
    """Robot-protocol adapter publishing geometry_msgs/Twist to ROS 2."""

    caps: Capabilities = ROVER_CAPS

    def __init__(self, args, config: "ControllerConfig"):
        # Lazy so the module is importable on no-ROS boxes (tests inject a
        # MagicMock rclpy into sys.modules before construction).
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.signals import SignalHandlerOptions
            from geometry_msgs.msg import Twist
        except ImportError as e:
            raise RuntimeError(
                "ROS 2 not available — the rover adapter requires "
                "rclpy + geometry_msgs.\n"
                "Fix:\n"
                "  1) sudo apt install ros-humble-ros-base "
                "ros-humble-geometry-msgs\n"
                "  2) source /opt/ros/humble/setup.bash\n"
                "  3) re-source setup_env.sh\n"
                f"Underlying error: {e}"
            ) from e

        self._rclpy = rclpy
        self._Twist = Twist
        self._Node = Node
        self._SignalHandlerOptions = SignalHandlerOptions
        self._SingleThreadedExecutor = SingleThreadedExecutor

        self._topic: str = getattr(args, "cmd_vel_topic", "/cmd_vel")
        self._namespace: str = getattr(args, "ros_namespace", "")
        self._domain_id: int = getattr(args, "ros_domain_id", 0)

        # ROS_DOMAIN_ID is read at rclpy.init; later writes are no-ops.
        if self._domain_id != 0:
            os.environ["ROS_DOMAIN_ID"] = str(self._domain_id)

        self._node = None
        self._publisher = None
        self._executor = None
        self._executor_thread: Optional[threading.Thread] = None
        # threading.Event (not asyncio.Event) so shutdown() can signal from any thread.
        self._shutdown_event = threading.Event()
        self._config = config

    async def connect(self) -> None:
        """Initialize rclpy without taking the SIGINT handler."""
        before_sigint = signal.getsignal(signal.SIGINT)
        self._rclpy.init(
            args=None,
            signal_handler_options=self._SignalHandlerOptions.NO,
        )
        after_sigint = signal.getsignal(signal.SIGINT)
        if after_sigint is not before_sigint:
            raise ConnectionError(
                "rclpy.init clobbered the SIGINT handler despite "
                f"SignalHandlerOptions.NO. before={before_sigint!r} "
                f"after={after_sigint!r}"
            )

    async def start_session(self) -> None:
        self._node = self._Node("robot_follow_rover", namespace=self._namespace)
        self._publisher = self._node.create_publisher(self._Twist, self._topic, 10)
        self._executor = self._SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._spin_loop,
            name="rover-ros-spin",
            daemon=True,
        )
        self._executor_thread.start()

    def _spin_loop(self) -> None:
        # spin_once with timeout (not rclpy.spin / executor.spin) so the
        # loop can observe the shutdown event without blocking forever.
        while not self._shutdown_event.is_set():
            self._executor.spin_once(timeout_sec=0.05)

    async def send_command(
        self,
        cmd: RobotCommand,
        safety_ctx: SafetyContext,
    ) -> None:
        if safety_ctx.target_lost:
            return
        if self._publisher is None:
            return
        twist = self._Twist()
        twist.linear.x = cmd.forward_m_s
        twist.angular.z = cmd.yaw_rate
        self._publisher.publish(twist)

    async def send_zero(self) -> None:
        if self._publisher is None:
            return
        self._publisher.publish(self._Twist())

    async def on_target_lost(self, last_detection: Optional[Detection]) -> None:
        if self._publisher is None:
            return
        self._publisher.publish(self._Twist())

    async def shutdown(self) -> None:
        """Idempotent; safe after a partially-failed connect/start_session."""
        self._shutdown_event.set()
        if self._executor_thread is not None:
            self._executor_thread.join(timeout=2.0)
            if self._executor_thread.is_alive():
                LOGGER.warning("[rover] spin thread did not exit within 2s")
        if self._node is not None:
            self._node.destroy_node()
        if self._rclpy is not None:
            self._rclpy.try_shutdown()
        self._node = None
        self._publisher = None
        self._executor = None
        self._executor_thread = None
