"""Tier-1 install smoke test for the robot-follow package (formerly drone-follow).

Forward-compatible design: this file lands BEFORE the rename commit. The rename
itself (Phase 1 Wave 1) makes `robot_follow` the importable package and adds
the `robot-follow` console script alongside the legacy `drone-follow` alias.
This test asserts the FULL post-rename contract once `robot_follow` is
installed, and SKIPS cleanly on the pre-rename tree so it does not break the
current state.

The `drone-follow` console script is asserted to ALWAYS work (legacy
pre-rename, alias post-rename) — it is the stable invocation contract used by
`scripts/start_air.sh`, the boot service, and user muscle memory.

Detection strategy:
  `_robot_follow_installed()` uses `importlib.util.find_spec` (no side-effect
  import) to detect whether the renamed package is on `sys.path`. Tests gated
  on the post-rename tree call `pytest.skip(...)` when this returns False.

Tests that hold pre- and post-rename (no skip):
  - test_drone_follow_console_script_alias_on_path
  - test_drone_follow_help_exits_zero

Tests that skip pre-rename and assert post-rename:
  - test_robot_follow_package_imports
  - test_robot_follow_follow_api_imports
  - test_drone_follow_import_raises_after_rename
  - test_robot_follow_console_script_on_path
  - test_robot_follow_help_exits_zero
  - test_help_outputs_byte_identical
  - test_pip_show_robot_follow_succeeds
  - test_pip_show_drone_follow_returns_nothing

Phase 1 success-criteria mapping (from ROADMAP):
  1. `pip show robot-follow` succeeds; `pip show drone-follow` returns nothing
     → test_pip_show_robot_follow_succeeds + test_pip_show_drone_follow_returns_nothing
  2. `drone-follow --help` and `robot-follow --help` produce identical output
     → test_help_outputs_byte_identical
  3. No `from drone_follow` / `import drone_follow` anywhere in source
     → test_drone_follow_import_raises_after_rename
"""
import difflib
import importlib.util
import os
import shutil
import subprocess
import sys
from importlib import import_module

import pytest

_SKIP_REASON = "pre-rename tree; robot_follow not yet installed"


def _robot_follow_installed() -> bool:
    """Return True iff `robot_follow` is importable (post-rename tree)."""
    return importlib.util.find_spec("robot_follow") is not None


def _skip_if_pre_rename() -> None:
    if not _robot_follow_installed():
        pytest.skip(_SKIP_REASON)


# --- Package import tests (post-rename) -------------------------------------


def test_robot_follow_package_imports():
    _skip_if_pre_rename()
    mod = import_module("robot_follow")
    assert mod is not None


def test_robot_follow_follow_api_imports():
    _skip_if_pre_rename()
    for name in (
        "robot_follow.follow_api.types",
        "robot_follow.follow_api.config",
        "robot_follow.follow_api.controller",
        "robot_follow.follow_api.state",
    ):
        m = import_module(name)
        assert m is not None, name


def test_drone_follow_import_raises_after_rename():
    """Negative assertion: post-rename, the legacy import path must be gone."""
    _skip_if_pre_rename()
    with pytest.raises(ModuleNotFoundError):
        import_module("drone_follow")


# --- Console script tests ---------------------------------------------------


def test_robot_follow_console_script_on_path():
    _skip_if_pre_rename()
    assert shutil.which("robot-follow"), (
        "robot-follow console script not on PATH. Activate the venv "
        "(`source setup_env.sh`) and re-run."
    )


def test_drone_follow_console_script_alias_on_path():
    """Legacy invocation must always work: pre-rename (real script) and
    post-rename (alias entry point pointing at the same main())."""
    assert shutil.which("drone-follow"), (
        "drone-follow console script not on PATH. Activate the venv "
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
    _skip_if_pre_rename()
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
    """Holds pre- and post-rename: pre = real script, post = alias entry point."""
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
    """Post-rename: `robot-follow --help` and `drone-follow --help` must be
    byte-identical — the alias is a pure entry-point rename, no separate code
    path."""
    _skip_if_pre_rename()
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
    _skip_if_pre_rename()
    proc = _pip_show("robot-follow")
    assert proc.returncode == 0, (
        f"`pip show robot-follow` exited {proc.returncode}\n"
        f"stdout:\n{proc.stdout.decode(errors='replace')}\n"
        f"stderr:\n{proc.stderr.decode(errors='replace')}"
    )


def test_pip_show_drone_follow_returns_nothing():
    """Post-rename: the `drone-follow` PyPI distribution must not exist.
    Only the `drone-follow` console script (provided by `robot-follow`) remains."""
    _skip_if_pre_rename()
    proc = _pip_show("drone-follow")
    assert proc.returncode != 0, (
        "`pip show drone-follow` unexpectedly succeeded post-rename. "
        "The legacy distribution should have been removed; only the "
        "`drone-follow` console-script alias (provided by robot-follow) "
        "should remain.\n"
        f"stdout:\n{proc.stdout.decode(errors='replace')}"
    )
