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


# RINT-02 / Q1 lock (Phase 6 CONTEXT, 2026-05-20): rover-specific slow-down
# when the person's bbox bottom is in the lowest 15% of the frame (the actor
# is too close to the rover; stop driving forward to avoid a collision).
# YAW IS PRESERVED — the rover can still rotate to recenter, just not drive
# forward. The drone adapter does NOT read this threshold; its existing
# retreat-from-tilt behavior operates on SafetyContext.bbox_bottom_normalized
# (the legacy field) via mavsdk_drone.py::_apply_retreat_from_tilt.
#
# Named constant per Phase 6 user decision: this is NOT a magic number
# embedded in send_command. Tuning happens here, ONE site.
ROVER_BOTTOM_STOP_THRESHOLD: float = 0.85


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

        # EMA smoothing state (rover-side; drone has its own SmoothingState
        # in mavsdk_drone.py). Initialised lazily on the first send_command
        # call so the first tick passes raw through — same gentle warmup as
        # the drone's filter, but starting from the first observed command
        # rather than zero (avoids attenuating the initial yaw response
        # below cmd.yaw_rate when the rover first sees the target).
        self._filtered_yaw: Optional[float] = None
        self._filtered_forward: Optional[float] = None

    async def connect(self) -> None:
        """Initialize rclpy without taking the SIGINT handler.

        Idempotent: returns immediately if rclpy is already initialized
        (e.g. by main-thread pre-init that avoids the Hailo-plugin race
        described in init_on_main_thread).
        """
        # `is True` (not just truthy) so MagicMock-rclpy from the test
        # fixture, which returns a truthy MagicMock from .ok(), does not
        # accidentally short-circuit and skip the rclpy.init call the
        # test asserts on.
        if self._rclpy.ok() is True:
            return
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
        """Create the Node + publisher + spin thread.

        Idempotent: returns immediately if the Node was already created
        (e.g. by main-thread pre-init).
        """
        if self._node is not None:
            return
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

    def init_on_main_thread(self) -> None:
        """Initialize rclpy + Node + publisher synchronously on the main thread.

        WORKAROUND: rclpy.Node creation from a worker thread races with the
        Hailo gstreamer pipeline's plugin loading and SIGABRT/SIGSEGV's the
        process at the C-level rcl_node_init call. Reproduced reliably with
        `robot-follow --robot rover --input udp://... --webui`; the abort
        signal even changes (SIGABRT ↔ SIGSEGV) depending on whether
        `--webui` is set, which is a tell-tale sign of heap-state corruption
        whose symptom depends on memory layout. None of the obvious
        single-cause reproducers (Gst MainLoop, MAVSDK/gRPC import, Hailo
        plugin load, 3 servers+threads) trigger it in isolation — only the
        full robot-follow startup combo reproduces.

        Call this on the main thread BEFORE any Hailo work begins. The
        worker thread's subsequent `connect()` / `start_session()` calls
        will detect the pre-initialized state and short-circuit.
        """
        # Calling the async methods directly here would require an event
        # loop; both methods are non-blocking in this implementation (just
        # rclpy.init + Node construction + thread.start), so we run them
        # via asyncio.run on a fresh loop.
        import asyncio
        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(self.connect())
            loop.run_until_complete(self.start_session())
        finally:
            loop.close()

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
        # EMA smoothing (low-pass filter) on both axes. The controller emits
        # a fresh P-controller setpoint every tick; with the rover's modest
        # dynamics + ~100 ms control-loop latency, even a well-tuned gain
        # can ring. Smoothing here matches what mavsdk_drone.py does in
        # _apply_smoothing — but it lives in this adapter, not in a shared
        # helper, because the smoothing is conceptually robot-specific
        # (the drone smooths POST-altitude-P, before MAVSDK; the rover
        # smooths POST-controller, before Twist publish). Honors
        # smooth_yaw / yaw_alpha / smooth_forward / forward_alpha from
        # ControllerConfig (set in configs/rover_simulation.json).
        yaw_raw = cmd.yaw_rate
        if self._config.smooth_yaw and self._filtered_yaw is not None:
            self._filtered_yaw = (
                self._config.yaw_alpha * yaw_raw
                + (1.0 - self._config.yaw_alpha) * self._filtered_yaw
            )
            yaw_out = self._filtered_yaw
        else:
            self._filtered_yaw = yaw_raw
            yaw_out = yaw_raw

        forward_raw = cmd.forward_m_s
        if self._config.smooth_forward and self._filtered_forward is not None:
            self._filtered_forward = (
                self._config.forward_alpha * forward_raw
                + (1.0 - self._config.forward_alpha) * self._filtered_forward
            )
            forward_out = self._filtered_forward
        else:
            self._filtered_forward = forward_raw
            forward_out = forward_raw

        twist = self._Twist()
        twist.linear.x = forward_out
        # YAW SIGN FLIP: the controller emits yaw_rate using the drone's NED /
        # MAVSDK convention where +z body axis points DOWN and clockwise-from-
        # above is positive yawspeed. ROS REP-103 uses ENU with +z body axis
        # pointing UP, so counter-clockwise-from-above is positive angular.z
        # (right-hand rule). Same numeric sign → OPPOSITE rotation direction:
        # without the flip the rover steers AWAY from the person.
        # This is a frame-of-reference adaptation, NOT a unit conversion —
        # the Q5 lock about deg/s vs rad/s magnitudes is preserved (the
        # controller emits in caps.yaw_unit; the adapter only flips sign).
        twist.angular.z = -yaw_out
        # RINT-02 / Q1 lock: when the person's bbox bottom enters the lowest
        # 15% of the frame, override forward to 0.0 (stop driving). Yaw is
        # PRESERVED — the rover can still rotate to recenter. cmd is not
        # mutated (RobotCommand is a value type emitted by the controller);
        # only the about-to-publish twist is overridden. The drone adapter
        # never reads bbox_bottom_norm — it stays on bbox_bottom_normalized
        # (the legacy field used by _apply_retreat_from_tilt).
        if (safety_ctx.bbox_bottom_norm is not None
                and safety_ctx.bbox_bottom_norm >= ROVER_BOTTOM_STOP_THRESHOLD):
            twist.linear.x = 0.0
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
