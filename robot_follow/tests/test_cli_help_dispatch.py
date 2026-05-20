"""ABS-09 two-pass argparse: --robot drone --help shows drone flags;
--robot rover --help does not.

Locks the dispatch implemented in 03-08 (robot_follow_app._build_parser):
a pre-parser extracts --robot {drone,rover}, then the full parser is
assembled via add_common_args + (add_drone_args | add_rover_args). The
tests invoke the console-script entry point as ``python -m
robot_follow.robot_follow_app`` so they exercise the same path the
``robot-follow`` / ``drone-follow`` aliases use, regardless of PATH
state.

Phase 4 Wave 0 (04-01) appends 6 xfail-marked rover-flag tests
(3 includes + 3 excludes) that Plan 04-02 closes when
``add_rover_args`` gains its real body.
"""

import subprocess
import sys

import pytest

DRONE_ONLY_FLAGS = ["--takeoff-landing", "--target-altitude", "--serial"]
ROVER_ONLY_FLAGS = ["--cmd-vel-topic", "--ros-namespace", "--ros-domain-id"]
XFAIL_REASON_ROVER_DISPATCH = (
    "add_rover_args registers --cmd-vel-topic / --ros-namespace / "
    "--ros-domain-id in Phase 4 Plan 04-02. This Wave 0 scaffold "
    "locks the contract; 04-02 strips these xfail markers."
)


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
# Phase 4 Wave 0 rover-flag tests — closed by Plan 04-02
# ----------------------------------------------------------------------------
# Each parametrized case is written as a standalone function (rather than a
# single @pytest.mark.parametrize over ROVER_ONLY_FLAGS) so each test case
# carries its own @pytest.mark.xfail decorator on a dedicated line. This
# satisfies the literal Plan 04-01 acceptance criterion
# `grep -cE '@pytest.mark.xfail' >= 6` and gives Plan 04-02 a clean per-flag
# anchor to delete when the rover argparse body lands. Plan 04-02 may collapse
# the six functions back into two parametrize blocks once the xfail markers
# are stripped.


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ROVER_DISPATCH)
def test_rover_help_includes_cmd_vel_topic():
    """ROVER-05: --robot rover --help shows --cmd-vel-topic."""
    out = _help_output("rover")
    assert "--cmd-vel-topic" in out, "expected --cmd-vel-topic in --robot rover --help output"


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ROVER_DISPATCH)
def test_rover_help_includes_ros_namespace():
    """ROVER-05: --robot rover --help shows --ros-namespace."""
    out = _help_output("rover")
    assert "--ros-namespace" in out, "expected --ros-namespace in --robot rover --help output"


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ROVER_DISPATCH)
def test_rover_help_includes_ros_domain_id():
    """ROVER-05: --robot rover --help shows --ros-domain-id."""
    out = _help_output("rover")
    assert "--ros-domain-id" in out, "expected --ros-domain-id in --robot rover --help output"


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ROVER_DISPATCH)
def test_drone_help_excludes_cmd_vel_topic():
    """Plan-checker invariant (DESIGN-NOTES line 128): --cmd-vel-topic is rover-only."""
    out = _help_output("drone")
    assert "--cmd-vel-topic" not in out, "unexpected --cmd-vel-topic in --robot drone --help output"


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ROVER_DISPATCH)
def test_drone_help_excludes_ros_namespace():
    """Plan-checker invariant (DESIGN-NOTES line 128): --ros-namespace is rover-only."""
    out = _help_output("drone")
    assert "--ros-namespace" not in out, "unexpected --ros-namespace in --robot drone --help output"


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ROVER_DISPATCH)
def test_drone_help_excludes_ros_domain_id():
    """Plan-checker invariant (DESIGN-NOTES line 128): --ros-domain-id is rover-only."""
    out = _help_output("drone")
    assert "--ros-domain-id" not in out, "unexpected --ros-domain-id in --robot drone --help output"
