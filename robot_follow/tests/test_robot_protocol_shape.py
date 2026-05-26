"""ABS-01 Robot protocol-shape gate.

Both halves now pass (xfail markers stripped):
  1. test_robot_protocol_has_six_methods_plus_caps — Robot protocol
     landed in 03-04.
  2. test_mavsdk_drone_adapter_implements_robot — MavsdkDroneAdapter
     implementing Robot landed in 03-06.
"""

from types import SimpleNamespace

import pytest

from robot_follow.follow_api.config import ControllerConfig
from robot_follow.robot_api.adapters.mavsdk_drone import MavsdkDroneAdapter
from robot_follow.robot_api.robot import Robot


def test_robot_protocol_has_six_methods_plus_caps():
    """Robot protocol must declare 6 methods + a caps attribute."""
    expected_methods = {
        "connect",
        "start_session",
        "send_command",
        "send_zero",
        "on_target_lost",
        "shutdown",
    }
    actual_methods = {
        m for m in dir(Robot) if not m.startswith("_") and callable(getattr(Robot, m, None))
    }
    assert expected_methods <= actual_methods
    from typing import get_type_hints

    hints = get_type_hints(Robot)
    assert "caps" in hints


def test_mavsdk_drone_adapter_implements_robot():
    """MavsdkDroneAdapter must structurally implement the Robot protocol."""
    for m in ("connect", "start_session", "send_command", "send_zero", "on_target_lost", "shutdown"):
        assert hasattr(MavsdkDroneAdapter, m), f"missing method: {m}"
    adapter = MavsdkDroneAdapter(
        SimpleNamespace(connection="udp://0:0", takeoff_landing=False),
        ControllerConfig(),
    )
    assert isinstance(adapter, Robot)
    assert adapter.caps.yaw_unit == "deg/s"
