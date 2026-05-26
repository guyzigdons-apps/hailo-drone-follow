"""--robot drone --help shows drone flags; --robot rover --help shows rover flags; the two sets are disjoint."""

import subprocess
import sys

import pytest

DRONE_ONLY_FLAGS = ["--takeoff-landing", "--target-altitude", "--serial"]
ROVER_ONLY_FLAGS = ["--cmd-vel-topic", "--ros-namespace", "--ros-domain-id"]


def _help_output(robot: str) -> str:
    # Use `python -m ...` so the test doesn't depend on console-script PATH.
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
    for flag in ("--webui", "--display"):
        assert flag in drone_out, f"{flag} missing from drone help"
        assert flag in rover_out, f"{flag} missing from rover help"


@pytest.mark.parametrize("flag", ROVER_ONLY_FLAGS)
def test_rover_help_includes_rover_flag(flag: str):
    assert flag in _help_output("rover"), f"expected {flag} in --robot rover --help"


@pytest.mark.parametrize("flag", ROVER_ONLY_FLAGS)
def test_drone_help_excludes_rover_flag(flag: str):
    assert flag not in _help_output("drone"), f"unexpected {flag} in --robot drone --help"
