"""ABS-01 Robot protocol-shape gate.

Two xfail tests:
  1. test_robot_protocol_has_six_methods_plus_caps (xfail until 03-04 lands the
     Robot Protocol in robot_follow/robot_api/robot.py)
  2. test_mavsdk_drone_adapter_implements_robot (xfail until 03-06 makes
     MavsdkDroneAdapter implement the Robot Protocol)

The xfail markers are stripped in plans 03-04 and 03-06 respectively. Grep
XFAIL_REASON_PROTOCOL / XFAIL_REASON_ADAPTER to find the strip sites.
"""

import pytest

XFAIL_REASON_PROTOCOL = "Robot protocol lands in 03-04-PLAN"
XFAIL_REASON_ADAPTER = "MavsdkDroneAdapter implements Robot in 03-06-PLAN"


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON_PROTOCOL)
def test_robot_protocol_has_six_methods_plus_caps():
    """Robot protocol must declare 6 methods + a caps attribute."""
    try:
        from robot_follow.robot_api.robot import Robot
    except ImportError:
        pytest.skip("robot_api.robot.Robot not yet defined (lands in 03-04)")
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
    # caps is a Protocol attribute, so check the Protocol's annotations
    from typing import get_type_hints

    hints = get_type_hints(Robot)
    assert "caps" in hints


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
def test_mavsdk_drone_adapter_implements_robot():
    """MavsdkDroneAdapter must structurally implement the Robot protocol."""
    try:
        from robot_follow.robot_api.adapters.mavsdk_drone import MavsdkDroneAdapter
    except ImportError:
        pytest.skip("adapter not yet present (lands in 03-06)")
    for m in ("connect", "start_session", "send_command", "send_zero", "on_target_lost", "shutdown"):
        assert hasattr(MavsdkDroneAdapter, m), f"missing method: {m}"
    assert hasattr(MavsdkDroneAdapter, "caps") or "caps" in getattr(
        MavsdkDroneAdapter, "__annotations__", {}
    )
