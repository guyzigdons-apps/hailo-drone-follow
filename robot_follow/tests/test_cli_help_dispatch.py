"""ABS-09 two-pass argparse: --robot drone --help shows drone flags;
--robot rover --help does not.

Locks the dispatch implemented in 03-08 (robot_follow_app._build_parser):
a pre-parser extracts --robot {drone,rover}, then the full parser is
assembled via add_common_args + (add_drone_args | add_rover_args). The
tests invoke the console-script entry point as ``python -m
robot_follow.robot_follow_app`` so they exercise the same path the
``robot-follow`` / ``drone-follow`` aliases use, regardless of PATH
state.

Phase 4 Plan 04-02 added the rover-flag assertions; they now PASS
because ``add_rover_args`` registers ``--cmd-vel-topic`` /
``--ros-namespace`` / ``--ros-domain-id`` (ROVER-05).
"""

import subprocess
import sys

import pytest

DRONE_ONLY_FLAGS = ["--takeoff-landing", "--target-altitude", "--serial"]
ROVER_ONLY_FLAGS = ["--cmd-vel-topic", "--ros-namespace", "--ros-domain-id"]


def _help_output(robot: str) -> str:
    # Invoke via `python -m robot_follow.robot_follow_app` so the test
    # works regardless of console-script PATH state.
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "robot_follow.robot_follow_app",
            "--robot",
            robot,
            "--help",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )
    # --help exits 0; combine stdout + stderr just in case.
    return (result.stdout or "") + (result.stderr or "")


@pytest.mark.parametrize("flag", DRONE_ONLY_FLAGS)
def test_drone_help_includes_drone_flag(flag: str):
    out = _help_output("drone")
    assert flag in out, f"expected {flag} in --robot drone --help output"


@pytest.mark.parametrize("flag", DRONE_ONLY_FLAGS)
def test_rover_help_excludes_drone_flag(flag: str):
    out = _help_output("rover")
    assert flag not in out, f"unexpected {flag} in --robot rover --help output"


def test_common_flags_visible_to_both_robots():
    drone_out = _help_output("drone")
    rover_out = _help_output("rover")
    # --webui and --display are common; should appear under both.
    for flag in ("--webui", "--display"):
        assert flag in drone_out, f"{flag} missing from drone help"
        assert flag in rover_out, f"{flag} missing from rover help"


# ----------------------------------------------------------------------------
# Phase 4 rover-flag tests — active since Plan 04-02
# ----------------------------------------------------------------------------
# Six standalone functions (3 includes + 3 excludes) cover the disjoint
# rover-flag contract per DESIGN-NOTES line 128. Originally landed by
# Plan 04-01 as failing scaffolds; Plan 04-02 landed the real
# ``add_rover_args`` body in ``robot_follow_app.py`` and dropped the
# markers — every case now PASSes.


def test_rover_help_includes_cmd_vel_topic():
    """ROVER-05: --robot rover --help shows --cmd-vel-topic."""
    out = _help_output("rover")
    assert "--cmd-vel-topic" in out, "expected --cmd-vel-topic in --robot rover --help output"


def test_rover_help_includes_ros_namespace():
    """ROVER-05: --robot rover --help shows --ros-namespace."""
    out = _help_output("rover")
    assert "--ros-namespace" in out, "expected --ros-namespace in --robot rover --help output"


def test_rover_help_includes_ros_domain_id():
    """ROVER-05: --robot rover --help shows --ros-domain-id."""
    out = _help_output("rover")
    assert "--ros-domain-id" in out, "expected --ros-domain-id in --robot rover --help output"


def test_drone_help_excludes_cmd_vel_topic():
    """Plan-checker invariant (DESIGN-NOTES line 128): --cmd-vel-topic is rover-only."""
    out = _help_output("drone")
    assert "--cmd-vel-topic" not in out, "unexpected --cmd-vel-topic in --robot drone --help output"


def test_drone_help_excludes_ros_namespace():
    """Plan-checker invariant (DESIGN-NOTES line 128): --ros-namespace is rover-only."""
    out = _help_output("drone")
    assert "--ros-namespace" not in out, "unexpected --ros-namespace in --robot drone --help output"


def test_drone_help_excludes_ros_domain_id():
    """Plan-checker invariant (DESIGN-NOTES line 128): --ros-domain-id is rover-only."""
    out = _help_output("drone")
    assert "--ros-domain-id" not in out, "unexpected --ros-domain-id in --robot drone --help output"
