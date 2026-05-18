"""ABS-02 RobotCommand field-shape gate.

Renamed from test_velocity_command_shape.py via `git mv` so git log --follow
shows the rename history (03-02-PLAN).

Two tests:
  1. test_velocity_command_shape_legacy (xfail until 03-07-PLAN deletes VelocityCommand)
     — keeps the old shape check alive until VelocityCommand is removed.
  2. test_robot_command_shape (xfail until 03-03-PLAN lands RobotCommand)
     — asserts the new RobotCommand dataclass shape: forward_m_s / yaw_rate / down_m_s.
"""

import dataclasses

import pytest

XFAIL_REASON_LEGACY = "VelocityCommand deleted in 03-07-PLAN"
XFAIL_REASON_NEW = "RobotCommand lands in 03-03-PLAN"


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON_LEGACY)
def test_velocity_command_shape_legacy():
    """Original VelocityCommand 3-field shape; xfails after 03-07 deletes the class."""
    from robot_follow.follow_api import VelocityCommand

    names = {f.name for f in dataclasses.fields(VelocityCommand)}
    assert names == {"forward_m_s", "down_m_s", "yawspeed_deg_s"}
    cmd = VelocityCommand(1.0, -0.5, 30.0)
    assert cmd.forward_m_s == 1.0
    assert cmd.down_m_s == -0.5
    assert cmd.yawspeed_deg_s == 30.0


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON_NEW)
def test_robot_command_shape():
    """New RobotCommand 3-field shape; passes once 03-03 lands the dataclass."""
    try:
        from robot_follow.follow_api.types import RobotCommand
    except ImportError:
        pytest.skip("RobotCommand not yet defined (lands in 03-03)")
    field_names = {f.name for f in dataclasses.fields(RobotCommand)}
    assert field_names == {"forward_m_s", "yaw_rate", "down_m_s"}
    rc = RobotCommand()
    assert rc.forward_m_s == 0.0
    assert rc.yaw_rate == 0.0
    assert rc.down_m_s == 0.0
