"""ABS-09 two-pass argparse: --robot drone --help shows drone flags;
--robot rover --help does not.

xfail until 03-08-PLAN lands the --robot dispatch + add_common/drone/rover_args split.
"""

import subprocess
import sys

import pytest

XFAIL_REASON = "--robot dispatch lands in 03-08-PLAN"
DRONE_ONLY_FLAGS = ["--takeoff-landing", "--target-altitude", "--serial"]


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


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON)
@pytest.mark.parametrize("flag", DRONE_ONLY_FLAGS)
def test_drone_help_includes_drone_flag(flag: str):
    out = _help_output("drone")
    assert flag in out, f"expected {flag} in --robot drone --help output"


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON)
@pytest.mark.parametrize("flag", DRONE_ONLY_FLAGS)
def test_rover_help_excludes_drone_flag(flag: str):
    out = _help_output("rover")
    assert flag not in out, f"unexpected {flag} in --robot rover --help output"


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON)
def test_common_flags_visible_to_both_robots():
    drone_out = _help_output("drone")
    rover_out = _help_output("rover")
    # --webui and --display are common; should appear under both.
    for flag in ("--webui", "--display"):
        assert flag in drone_out, f"{flag} missing from drone help"
        assert flag in rover_out, f"{flag} missing from rover help"
