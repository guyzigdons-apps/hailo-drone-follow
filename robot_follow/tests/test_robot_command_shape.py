"""ABS-02 RobotCommand field-shape gate.

Phase 3 plan 03-07 deleted the legacy ``VelocityCommand`` class; the
legacy shape test has been removed. The xfail marker from 03-02 was
stripped here once 03-03 landed ``RobotCommand`` and 03-07 removed
``VelocityCommand`` from ``follow_api/types.py``.
"""

import dataclasses


def test_robot_command_shape():
    """New RobotCommand 3-field shape."""
    from robot_follow.follow_api.types import RobotCommand

    field_names = {f.name for f in dataclasses.fields(RobotCommand)}
    assert field_names == {"forward_m_s", "yaw_rate", "down_m_s"}
    rc = RobotCommand()
    assert rc.forward_m_s == 0.0
    assert rc.yaw_rate == 0.0
    assert rc.down_m_s == 0.0


def test_velocity_command_is_deleted():
    """Phase 3 plan 03-07 deleted VelocityCommand. ImportError is the gate."""
    import pytest
    with pytest.raises(ImportError):
        from robot_follow.follow_api.types import VelocityCommand  # noqa: F401
