"""Tier-1 install smoke test for the robot-follow package.

Post-rename canonical verification gate for Phase 1. Asserts the full Phase 1
success-criteria contract from ROADMAP:

  1. `pip show robot-follow` succeeds; `pip show drone-follow` returns nothing.
  2. `drone-follow --help` and `robot-follow --help` produce byte-identical output.
  3. The legacy `drone_follow` import path raises ModuleNotFoundError at runtime.

The `drone-follow` console script is preserved permanently as an alias entry
point pointing at `robot_follow.robot_follow_app:main` — same script, two
names. It is the stable invocation contract used by `scripts/start_air.sh`,
the boot service unit, and user muscle memory; both names MUST work.

Tests:
  - test_robot_follow_package_imports
  - test_robot_follow_follow_api_imports
  - test_drone_follow_import_raises
  - test_robot_follow_console_script_on_path
  - test_drone_follow_console_script_alias_on_path
  - test_robot_follow_help_exits_zero
  - test_drone_follow_help_exits_zero
  - test_help_outputs_byte_identical
  - test_pip_show_robot_follow_succeeds
  - test_pip_show_drone_follow_returns_nothing
"""
import difflib
import os
import shutil
import subprocess
import sys
from importlib import import_module

import pytest


# --- Package import tests ---------------------------------------------------


def test_robot_follow_package_imports():
    mod = import_module("robot_follow")
    assert mod is not None


def test_robot_follow_follow_api_imports():
    for name in (
        "robot_follow.follow_api.types",
        "robot_follow.follow_api.config",
        "robot_follow.follow_api.controller",
        "robot_follow.follow_api.state",
    ):
        m = import_module(name)
        assert m is not None, name


def test_drone_follow_import_raises():
    """Negative assertion: the legacy import path must be gone post-rename."""
    with pytest.raises(ModuleNotFoundError):
        import_module("drone_follow")


# --- Console script tests ---------------------------------------------------


def test_robot_follow_console_script_on_path():
    assert shutil.which("robot-follow"), (
        "robot-follow console script not on PATH. Activate the venv "
        "(`source setup_env.sh`) and re-run."
    )


def test_drone_follow_console_script_alias_on_path():
    """The `drone-follow` console-script alias must remain on PATH (permanent
    alias for the boot service + user muscle memory)."""
    assert shutil.which("drone-follow"), (
        "drone-follow console script alias not on PATH. Activate the venv "
        "(`source setup_env.sh`) and re-run."
    )


# --- --help exit tests ------------------------------------------------------


def _run_help(bin_path: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [bin_path, "--help"],
        capture_output=True,
        timeout=30,
        check=False,
        env={**os.environ},
    )


def test_robot_follow_help_exits_zero():
    bin_path = shutil.which("robot-follow")
    assert bin_path, "robot-follow console script not on PATH"
    proc = _run_help(bin_path)
    assert proc.returncode == 0, (
        f"`{bin_path} --help` exited {proc.returncode}\n"
        f"stdout:\n{proc.stdout.decode(errors='replace')}\n"
        f"stderr:\n{proc.stderr.decode(errors='replace')}"
    )
    assert b"--input" in proc.stdout or b"--input" in proc.stderr


def test_drone_follow_help_exits_zero():
    """The `drone-follow` alias must produce a working --help (same code path
    as `robot-follow --help` via the shared `main()` entry point)."""
    bin_path = shutil.which("drone-follow")
    assert bin_path, (
        "drone-follow console script not on PATH. Activate the venv "
        "(`source setup_env.sh`) and re-run."
    )
    proc = _run_help(bin_path)
    assert proc.returncode == 0, (
        f"`{bin_path} --help` exited {proc.returncode}\n"
        f"stdout:\n{proc.stdout.decode(errors='replace')}\n"
        f"stderr:\n{proc.stderr.decode(errors='replace')}"
    )
    assert b"--input" in proc.stdout or b"--input" in proc.stderr


def test_help_outputs_byte_identical():
    """`robot-follow --help` and `drone-follow --help` must be byte-identical
    — the alias is a pure entry-point rename, no separate code path."""
    rf_bin = shutil.which("robot-follow")
    df_bin = shutil.which("drone-follow")
    assert rf_bin and df_bin, (
        f"Both console scripts must be on PATH "
        f"(robot-follow={rf_bin!r}, drone-follow={df_bin!r})"
    )
    rf = _run_help(rf_bin)
    df = _run_help(df_bin)
    rf_out = rf.stdout + rf.stderr
    df_out = df.stdout + df.stderr
    if rf_out != df_out:
        diff = "".join(
            difflib.unified_diff(
                df_out.decode(errors="replace").splitlines(keepends=True),
                rf_out.decode(errors="replace").splitlines(keepends=True),
                fromfile="drone-follow --help",
                tofile="robot-follow --help",
            )
        )
        pytest.fail(
            "robot-follow --help and drone-follow --help diverged:\n" + diff
        )


# --- pip metadata tests -----------------------------------------------------


def _pip_show(distribution: str) -> subprocess.CompletedProcess:
    """Bind `pip show` to the test interpreter's venv via `sys.executable -m pip`."""
    return subprocess.run(
        [sys.executable, "-m", "pip", "show", distribution],
        capture_output=True,
        timeout=15,
        check=False,
        env={**os.environ},
    )


def test_pip_show_robot_follow_succeeds():
    proc = _pip_show("robot-follow")
    assert proc.returncode == 0, (
        f"`pip show robot-follow` exited {proc.returncode}\n"
        f"stdout:\n{proc.stdout.decode(errors='replace')}\n"
        f"stderr:\n{proc.stderr.decode(errors='replace')}"
    )


def test_pip_show_drone_follow_returns_nothing():
    """The legacy `drone-follow` PyPI distribution must not exist post-rename.
    Only the `drone-follow` console-script alias (provided by `robot-follow`)
    remains."""
    proc = _pip_show("drone-follow")
    assert proc.returncode != 0, (
        "`pip show drone-follow` unexpectedly succeeded post-rename. "
        "The legacy distribution should have been removed; only the "
        "`drone-follow` console-script alias (provided by robot-follow) "
        "should remain.\n"
        f"stdout:\n{proc.stdout.decode(errors='replace')}"
    )
