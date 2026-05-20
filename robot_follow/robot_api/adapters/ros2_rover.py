"""ROS 2 rover adapter — Ros2RoverAdapter + ROVER_CAPS.

Phase 4 plan 04-03. Closes ROVER-01 (adapter class + Node + Publisher),
ROVER-02 (SignalHandlerOptions.NO preserves drone-follow's SIGINT
handler), ROVER-03 (background spin loop with shutdown event), ROVER-04
(friendly RuntimeError on missing rclpy), ROVER-06 (Twist mapping with
NO unit conversion — yaw_rate already in rad/s per Q5 lock), ROVER-07
(axes-only Capabilities, no ALTITUDE), ROVER-08 (SIGINT-survives smoke
assertion).

Why all rclpy / geometry_msgs imports are LAZY (inside ``__init__``,
NOT at module top): on a no-ROS box (the dev machines + CI), ``import
rclpy`` raises ImportError. Module-level imports would then make THIS
file un-importable, which would in turn break the drone path's
``adapters/__init__.py`` discovery AND break every unit test that
tries to construct the rover adapter with a mocked rclpy. The lazy
pattern lets the test suite inject MagicMock stand-ins into
``sys.modules`` BEFORE construction; the production path raises the
ROVER-04 friendly RuntimeError when the real rclpy is genuinely
missing. See RESEARCH § "Defensive import (ROVER-04)" for the locked
message string and PITFALLS Pitfall 2 for the original incident.

Architectural locks honored here:
- Lives as a SIBLING to mavsdk_drone.py under robot_api/adapters/.
  No shared code, no cross-import; both implement the SAME Robot
  protocol.
- Capabilities stays axes + units only (per
  ``feedback_robot_abstraction_axes_only.md``). Behaviors (Twist
  mapping, signal-handler preservation, threading model) live in
  THIS adapter, never gated by a Capabilities flag.
- yaw_rate is published verbatim to ``angular.z`` — adapter does NO
  unit conversion. ``Capabilities.yaw_unit`` declares the unit the
  controller emits; both sides agree on rad/s for the rover.
"""

from __future__ import annotations

import asyncio  # noqa: F401  (kept for symmetry with mavsdk_drone.py; async methods below)
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


# Rover capability constant — per Q8 lock (Phase 3 CONTEXT): per-adapter
# constants live with their adapter; follow_api/types.py stays types-only.
# No ALTITUDE axis (ROVER-07): rovers operate on FORWARD + YAW only.
# yaw_unit="rad/s" — controller emits yaw_rate already in this unit
# (Q5 lock per Phase 3 RESEARCH); adapter does NO conversion.
ROVER_CAPS: Capabilities = Capabilities(
    axes=frozenset({Axis.FORWARD, Axis.YAW}),
    yaw_unit="rad/s",
)


class Ros2RoverAdapter:
    """Robot-protocol adapter that publishes geometry_msgs/Twist to ROS 2.

    Sibling to ``MavsdkDroneAdapter``. Both implement the same Robot
    protocol but with NO shared code. The drone goes out through MAVSDK
    gRPC; the rover goes out through rclpy + a Twist publisher on
    ``/cmd_vel`` (configurable via ``--cmd-vel-topic`` per ROVER-05,
    landed in Plan 04-02).

    Lifecycle (per RESEARCH § "rclpy lifecycle ordering — single
    checklist"):
        connect()
            - capture SIGINT handler
            - rclpy.init(signal_handler_options=NO)
            - assert SIGINT handler unchanged (ROVER-02)
        start_session()
            - create Node(namespace=…)
            - create_publisher(Twist, topic, qos=10)
            - start SingleThreadedExecutor + spin thread (daemon)
        send_command()
            - short-circuit if target_lost (Q6 lock)
            - publish Twist(linear.x=forward_m_s, angular.z=yaw_rate)
        send_zero() / on_target_lost()
            - publish Twist() (all zeros — rover does NOT yaw-spin)
        shutdown()
            - idempotent: shutdown_event.set() →
              thread.join(timeout=2.0) → node.destroy_node() →
              rclpy.try_shutdown()
    """

    caps: Capabilities = ROVER_CAPS  # class-level for type-checker

    def __init__(self, args, config: "ControllerConfig"):
        # Defensive lazy import — keeps the module file importable on
        # no-ROS boxes (so unit tests can inject MagicMock stand-ins
        # into sys.modules BEFORE construction). The real ImportError
        # surfaces as a ROVER-04 friendly RuntimeError that names the
        # apt package + the source command.
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

        # Bind imported symbols on self so the rest of the methods can
        # use them without re-importing (and so tests can swap them via
        # the rclpy_mock fixture transparently).
        self._rclpy = rclpy
        self._Twist = Twist
        self._Node = Node
        self._SignalHandlerOptions = SignalHandlerOptions
        self._SingleThreadedExecutor = SingleThreadedExecutor

        # Per CONTEXT instance-attr rule (mirrors mavsdk_drone.py:582):
        # class-level caps is for the type-checker; we still set the
        # instance attribute for Protocol-fit checks at runtime.
        self.caps = ROVER_CAPS

        # CLI args — getattr fallbacks let unit tests pass a minimal
        # SimpleNamespace (Plan 04-02 wires the argparse registration).
        self._topic: str = getattr(args, "cmd_vel_topic", "/cmd_vel")
        self._namespace: str = getattr(args, "ros_namespace", "")
        self._domain_id: int = getattr(args, "ros_domain_id", 0)

        # DDS domain isolation — only relevant when non-zero. Setting
        # before rclpy.init lets the runtime pick up the override.
        # PITFALLS Pitfall: ROS_DOMAIN_ID is read AT rclpy.init; later
        # changes are no-ops.
        if self._domain_id != 0:
            os.environ["ROS_DOMAIN_ID"] = str(self._domain_id)

        # State — populated by connect/start_session, torn down by
        # shutdown. Tests assert on every one of these via the public
        # attribute name (see test_ros2_rover_adapter.py lines 138-141).
        self._node = None
        self._publisher = None
        self._executor = None
        self._executor_thread: Optional[threading.Thread] = None

        # PITFALLS Pitfall 4: cross-thread shutdown signaling MUST use
        # threading.Event (NOT asyncio.Event — those are loop-bound and
        # not safe to set from outside the loop).
        self._shutdown_event = threading.Event()

        # config is accepted for signature symmetry with
        # MavsdkDroneAdapter (mavsdk_drone.py:581) — rover doesn't need
        # ControllerConfig fields today, but accepting it keeps the
        # composition root in robot_follow_app.py uniform.
        self._config = config

    async def connect(self) -> None:
        """Initialize rclpy without clobbering drone-follow's SIGINT handler.

        Per ROVER-02 (PITFALLS Pitfall 1): rclpy.init defaults to
        ``SignalHandlerOptions.ALL`` which replaces the existing SIGINT
        handler with rclpy's. We pass ``SignalHandlerOptions.NO`` so
        the orchestrator's on_signal stays bound, then assert the
        handler is still the pre-init one (ROVER-08 smoke test locks
        this contract from the test side).

        Raises ConnectionError on signal-handler clobber (per
        ``Robot.connect`` docstring at robot.py:48 — orchestrator
        catches ConnectionError and exits cleanly).
        """
        before_sigint = signal.getsignal(signal.SIGINT)
        self._rclpy.init(
            args=None,
            signal_handler_options=self._SignalHandlerOptions.NO,
        )
        after_sigint = signal.getsignal(signal.SIGINT)
        if after_sigint is not before_sigint:
            raise ConnectionError(
                "ROVER-02 violation: rclpy.init clobbered the SIGINT "
                "handler despite SignalHandlerOptions.NO. "
                f"before={before_sigint!r} after={after_sigint!r}"
            )

    async def start_session(self) -> None:
        """Create Node + publisher; start the background spin thread.

        Per ROVER-03: spin loop runs in a daemon thread calling
        ``spin_once(timeout_sec=0.05)`` in a loop that checks
        ``self._shutdown_event`` each iteration. NEVER ``rclpy.spin``
        or ``executor.spin()`` (both block indefinitely and hang
        shutdown — PITFALLS Pitfall 3).
        """
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
        """Background spin loop. Runs in self._executor_thread.

        PITFALLS Pitfall 3: NEVER call rclpy.spin(node) or
        executor.spin() — both block forever and hang shutdown.
        Use spin_once(timeout_sec=0.05) in a while-loop checking
        self._shutdown_event (ROVER-03).
        """
        while not self._shutdown_event.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.05)
            except Exception:
                LOGGER.warning(
                    "[rover] spin_once raised; continuing",
                    exc_info=True,
                )

    async def send_command(
        self,
        cmd: RobotCommand,
        safety_ctx: SafetyContext,
    ) -> None:
        """Per-tick actuator call. Maps RobotCommand → Twist with NO
        unit conversion (ROVER-06 / Q5 lock).

        - Short-circuit on safety_ctx.target_lost (Q6 lock).
        - linear.x  = cmd.forward_m_s   (m/s direct — both sides m/s)
        - linear.y  = 0.0               (non-holonomic rover)
        - linear.z  = 0.0               (no ALTITUDE axis — ROVER-07)
        - angular.x = 0.0               (no ROLL)
        - angular.y = 0.0               (no PITCH)
        - angular.z = cmd.yaw_rate      (rad/s direct — controller
                                          emits rad/s per ROVER_CAPS,
                                          NO deg→rad conversion)
        """
        if safety_ctx.target_lost:
            return  # Q6 lock — adapter MUST early-return on target_lost
        if self._publisher is None:
            return  # connect/start_session failed or not yet called
        twist = self._Twist()
        twist.linear.x = cmd.forward_m_s
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = cmd.yaw_rate
        self._publisher.publish(twist)

    async def send_zero(self) -> None:
        """Publish an all-zero Twist (quiescent setpoint).

        Default ``Twist()`` ctor fills every field with 0.0; we publish
        it verbatim. Called once in the orchestrator's finally block
        on shutdown (per ``Robot.send_zero`` docstring at robot.py:81).
        """
        if self._publisher is None:
            return
        twist = self._Twist()  # all-zero by default ctor
        self._publisher.publish(twist)

    async def on_target_lost(self, last_detection: Optional[Detection]) -> None:
        """Rover's per-tick lost-target reaction: stop.

        Unlike the drone (which yaw-spins to scan toward the
        last-seen side), the rover STOPS when the target is lost.
        ``last_detection`` is intentionally unused (no scan behavior
        in v1.1).

        # TODO(Phase 6 / RINT-02): bottom-edge slow-near-edge for
        # rover lands here. Phase 4 ships the wire path; Phase 6
        # ships the tuning + safety overlay.
        """
        if self._publisher is None:
            return
        twist = self._Twist()  # all-zero by default ctor
        self._publisher.publish(twist)

    async def shutdown(self) -> None:
        """Idempotent. Safe to call multiple times (and after failed
        connect / partial start_session).

        Order per PITFALLS "Looks Done But Isn't" checklist:
            1. shutdown_event.set()  → spin loop exits at next iter
            2. thread.join(timeout=2.0) → bounded wait (daemon=True
               ensures the thread cannot outlive the process if it
               wedges)
            3. node.destroy_node()
            4. rclpy.try_shutdown()  (NOT shutdown() — try_shutdown
               is the idempotent variant; shutdown() raises on
               already-shutdown state)

        Each step is wrapped in try/except so a partial-init followed
        by shutdown() never raises (per ``Robot.shutdown`` docstring
        at robot.py:104).
        """
        self._shutdown_event.set()
        if self._executor_thread is not None:
            try:
                self._executor_thread.join(timeout=2.0)
                if self._executor_thread.is_alive():
                    LOGGER.warning(
                        "[rover] spin thread did not exit within 2s; "
                        "continuing cleanup (daemon=True ensures thread "
                        "dies with process)"
                    )
            except Exception:
                LOGGER.warning("[rover] thread.join failed", exc_info=True)
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                LOGGER.warning("[rover] node.destroy_node failed", exc_info=True)
        if self._rclpy is not None:
            try:
                self._rclpy.try_shutdown()
            except Exception:
                LOGGER.warning("[rover] rclpy.try_shutdown failed", exc_info=True)
        # NULL-OUT so subsequent send_command / send_zero /
        # on_target_lost early-return on self._publisher is None.
        self._node = None
        self._publisher = None
        self._executor = None
        self._executor_thread = None
