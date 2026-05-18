"""ABS-10 setup_env.sh conditional ROS sourcing.

Three assertions:
1. Sub-shell sourcing produces ROS_DISTRO=humble IFF /opt/ros/humble/setup.bash exists.
2. sys.path after sourcing has venv site-packages BEFORE /opt/ros/humble/...
   site-packages (venv-first ordering per PITFALLS.md Pitfall 2).
3. (Documentation): the ROS-source block goes AFTER the venv activation in setup_env.sh.

Assertions 1 + 2 + 3 are xfail until 03-09-PLAN lands the conditional ROS source block.
The file-existence half of assertion 1 (the `not exists -> no ROS_DISTRO` branch)
can pass today on this dev box where ROS is not installed.
"""

import subprocess
from pathlib import Path

import pytest

XFAIL_REASON = "ROS-source block lands in 03-09-PLAN"
REPO_ROOT = Path(__file__).resolve().parent.parent.parent
SETUP_ENV = REPO_ROOT / "setup_env.sh"
ROS_SETUP = Path("/opt/ros/humble/setup.bash")


def _source_and_capture_env() -> dict[str, str]:
    """Source setup_env.sh in a sub-shell and return the resulting env as a dict."""
    result = subprocess.run(
        ["bash", "-c", f"set -e; source {SETUP_ENV} >/dev/null 2>&1; env"],
        capture_output=True,
        text=True,
        timeout=60,
    )
    env: dict[str, str] = {}
    for line in result.stdout.splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            env[k] = v
    return env


def test_setup_env_sh_exists():
    """Sanity: setup_env.sh is at the repo root."""
    assert SETUP_ENV.exists(), f"setup_env.sh not at {SETUP_ENV}"


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON)
def test_ros_distro_iff_ros_installed():
    env = _source_and_capture_env()
    if ROS_SETUP.exists():
        assert env.get("ROS_DISTRO") == "humble", (
            "ROS installed but ROS_DISTRO not set after sourcing"
        )
    else:
        # On a box without ROS, sourcing must NOT introduce ROS_*.
        assert "ROS_DISTRO" not in env, (
            "ROS not installed but ROS_DISTRO leaked into env"
        )


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON)
def test_venv_first_in_pythonpath():
    if not ROS_SETUP.exists():
        pytest.skip("ROS not installed on this box; venv-first ordering test deferred")
    env = _source_and_capture_env()
    pythonpath = env.get("PYTHONPATH", "")
    venv_path = str(REPO_ROOT / "hailo-apps" / "venv_hailo_apps" / "lib")
    ros_path = "/opt/ros/humble/lib"
    if venv_path in pythonpath and ros_path in pythonpath:
        venv_idx = pythonpath.find(venv_path)
        ros_idx = pythonpath.find(ros_path)
        assert venv_idx < ros_idx, "venv must appear before ROS in PYTHONPATH"


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON)
def test_setup_env_sh_contains_conditional_ros_block():
    """Static check: setup_env.sh has the conditional ROS-source guard."""
    text = SETUP_ENV.read_text()
    assert "/opt/ros/humble/setup.bash" in text, (
        "setup_env.sh missing ROS conditional source block"
    )
