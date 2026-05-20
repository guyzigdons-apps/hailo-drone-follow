"""Phase 5 rover-sim smoke tests.

Parse-only structural assertions covering RSIM-01..07.  All gz / shellcheck /
git invocations skip cleanly when the tool is unavailable.

This file is INTENTIONALLY weak by design.  The Phase 5 milestone scope is
sim-infrastructure (SDF + bash + apt + docs); the "rover actually follows a
walking actor end-to-end in Gazebo" verification is Phase 6 RINT-04 under
operator gate.  See `.planning/phases/05-rover-sim/RESEARCH.md` for the test
strategy rationale.
"""

import re
import shutil
import subprocess
from pathlib import Path

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ROVER_DIR = PROJECT_ROOT / "sim" / "rover"
ROVER_SDF = ROVER_DIR / "rover.sdf"
ROVER_WORLDS_DIR = ROVER_DIR / "worlds"
ROVER_README = ROVER_DIR / "README.md"
START_ROVER_SH = ROVER_DIR / "start_rover_sim.sh"
INSTALL_SH = PROJECT_ROOT / "install.sh"
BRIDGE_SCRIPT = PROJECT_ROOT / "sim" / "bridge" / "video_bridge.py"


# ---------- RSIM-01 + RSIM-03: no ignition:: prefix anywhere in sim/rover/ ----------

def test_sdf_no_ignition_prefix() -> None:
    """RSIM-01 + RSIM-03 + PITFALLS Pitfall 5.

    Every SDF under sim/rover/ (rover.sdf + worlds/*.sdf) must use the
    Garden-era ``gz::`` prefix and never the ``ignition::`` prefix.
    """
    assert ROVER_DIR.is_dir(), f"{ROVER_DIR} missing — Plan 05-01 not landed?"
    sdfs = list(ROVER_DIR.rglob("*.sdf"))
    assert len(sdfs) >= 4, (
        f"expected >=4 SDFs (rover.sdf + 3 worlds), found {len(sdfs)}: {sdfs}"
    )
    for sdf in sdfs:
        text = sdf.read_text()
        assert "ignition::" not in text, (
            f"{sdf} contains 'ignition::' (Pitfall 5: silent load failure on Garden)"
        )


# ---------- RSIM-01: rover.sdf uses Garden DiffDrive ----------

def test_rover_sdf_uses_gz_diff_drive() -> None:
    """RSIM-01: rover.sdf must use ``gz::sim::systems::DiffDrive`` with
    filename ``gz-sim-diff-drive-system`` (Garden+ naming)."""
    assert ROVER_SDF.is_file(), f"{ROVER_SDF} missing — Plan 05-01 not landed?"
    text = ROVER_SDF.read_text()
    assert "gz::sim::systems::DiffDrive" in text, (
        f"{ROVER_SDF}: Garden DiffDrive class name absent (RSIM-01)"
    )
    assert "gz-sim-diff-drive-system" in text, (
        f"{ROVER_SDF}: Garden DiffDrive filename absent (RSIM-01)"
    )


# ---------- RSIM-02: rover.sdf overrides DiffDrive default topic ----------

def test_rover_sdf_cmd_vel_topic_override() -> None:
    """RSIM-02 + PITFALLS Pitfall 6: ``<topic>cmd_vel</topic>`` must appear
    EXACTLY ONCE inside the DiffDrive plugin block.  Without it the plugin
    subscribes on ``/model/rover/cmd_vel`` and the ros_gz_bridge mapping
    silently fails."""
    assert ROVER_SDF.is_file(), f"{ROVER_SDF} missing — Plan 05-01 not landed?"
    text = ROVER_SDF.read_text()
    count = text.count("<topic>cmd_vel</topic>")
    assert count == 1, (
        f"{ROVER_SDF}: expected exactly 1 '<topic>cmd_vel</topic>' override "
        f"in the DiffDrive plugin block (got {count}); RSIM-02 / Pitfall 6"
    )


# ---------- RSIM-03: 3 world SDFs include the rover model ----------

def test_world_sdfs_include_rover_model() -> None:
    """RSIM-03: each rover-adapted world must spawn the rover via
    ``<include><uri>model://rover</uri></include>``."""
    assert ROVER_WORLDS_DIR.is_dir(), (
        f"{ROVER_WORLDS_DIR} missing — Plan 05-02 not landed?"
    )
    worlds = sorted(ROVER_WORLDS_DIR.glob("*.sdf"))
    required = {
        "walk_across_then_approach.sdf",
        "random_walk.sdf",
        "circle_around.sdf",
    }
    names = {w.name for w in worlds}
    assert required.issubset(names), (
        f"missing required worlds: {required - names}; got {names} (RSIM-03)"
    )
    for w in worlds:
        text = w.read_text()
        assert "<uri>model://rover</uri>" in text, (
            f"{w}: missing <uri>model://rover</uri> (Plan 05-02 contract)"
        )


# ---------- RSIM-05: install.sh --rover apt list correct ----------

def test_install_sh_rover_lists_garden_bridge() -> None:
    """RSIM-05: ``install.sh --rover`` lists the correct Garden apt package
    and includes the apt-cache preflight check."""
    assert INSTALL_SH.is_file(), f"{INSTALL_SH} missing"
    text = INSTALL_SH.read_text()
    assert "ros-humble-ros-gzgarden-bridge" in text, (
        f"{INSTALL_SH}: ros-humble-ros-gzgarden-bridge absent (RSIM-05)"
    )
    assert "ros-humble-ros-base" in text, (
        f"{INSTALL_SH}: ros-humble-ros-base absent (RSIM-05)"
    )
    assert "ros-humble-geometry-msgs" in text, (
        f"{INSTALL_SH}: ros-humble-geometry-msgs absent (RSIM-05)"
    )
    assert "apt-cache show ros-humble-ros-base" in text, (
        f"{INSTALL_SH}: ros.org repo preflight absent (RSIM-05)"
    )
    assert "apt-cache show ros-humble-ros-gzgarden-bridge" in text, (
        f"{INSTALL_SH}: osrfoundation repo preflight absent (RSIM-05)"
    )


def test_install_sh_rover_excludes_fortress_bridge() -> None:
    """PITFALLS Pitfall 5 + RSIM-05: the Fortress-form package name
    ``ros-humble-ros-gz-bridge`` (without ``garden`` / ``harmonic`` suffix)
    must NOT appear in install.sh — it's the wrong binding and breaks
    DiffDrive silently on Garden.

    Implementation note: we check the install.sh text after stripping both
    valid Garden/Harmonic forms, so any leftover ``ros-humble-ros-gz-bridge``
    occurrence is by definition the Fortress form.
    """
    assert INSTALL_SH.is_file(), f"{INSTALL_SH} missing"
    text = INSTALL_SH.read_text()
    stripped = (
        text
        .replace("ros-humble-ros-gzgarden-bridge", "")
        .replace("ros-humble-ros-gzharmonic-bridge", "")
    )
    assert "ros-humble-ros-gz-bridge" not in stripped, (
        f"{INSTALL_SH}: Fortress-form 'ros-humble-ros-gz-bridge' present "
        f"(Pitfall 5; use 'ros-humble-ros-gzgarden-bridge' instead)"
    )


# ---------- RSIM-04: start_rover_sim.sh passes shellcheck ----------

def test_start_rover_sim_passes_shellcheck() -> None:
    """RSIM-04: the rover-sim launcher must pass shellcheck.  Skips when
    shellcheck is not installed (Phase 5 ethos: tests skip cleanly rather
    than hard-fail on missing dev tooling)."""
    assert START_ROVER_SH.is_file(), (
        f"{START_ROVER_SH} missing — Plan 05-05 not landed?"
    )
    if not shutil.which("shellcheck"):
        pytest.skip("shellcheck not installed")
    result = subprocess.run(
        ["shellcheck", str(START_ROVER_SH)],
        capture_output=True,
        text=True,
        cwd=str(PROJECT_ROOT),
    )
    assert result.returncode == 0, (
        f"shellcheck failed on {START_ROVER_SH}:\n"
        f"--- stdout ---\n{result.stdout}\n--- stderr ---\n{result.stderr}"
    )


# ---------- RSIM-06: video_bridge.py is byte-identical to HEAD ----------

def test_video_bridge_byte_identical_to_head() -> None:
    """RSIM-06 architectural lock: ``sim/bridge/video_bridge.py`` must NOT
    be edited as part of any rover-sim work — the rover sim reuses it
    VERBATIM via CLI flags (--topic, --host, --port).  Skips when git is
    unavailable or the file does not exist at HEAD (defensive guard for
    oddball CI envs)."""
    assert BRIDGE_SCRIPT.is_file(), f"{BRIDGE_SCRIPT} missing"
    if not shutil.which("git"):
        pytest.skip("git not installed")
    # Fetch HEAD version of the file.
    head_result = subprocess.run(
        ["git", "show", "HEAD:sim/bridge/video_bridge.py"],
        capture_output=True,
        cwd=str(PROJECT_ROOT),
    )
    if head_result.returncode != 0:
        pytest.skip(
            f"git show HEAD:sim/bridge/video_bridge.py failed: "
            f"{head_result.stderr.decode(errors='replace')}"
        )
    head_bytes = head_result.stdout
    wt_bytes = BRIDGE_SCRIPT.read_bytes()
    assert head_bytes == wt_bytes, (
        f"{BRIDGE_SCRIPT} differs from HEAD ({len(head_bytes)} bytes at HEAD vs "
        f"{len(wt_bytes)} bytes in working tree) — RSIM-06 architectural lock "
        f"violated; the rover sim must reuse video_bridge.py verbatim via CLI flags"
    )


# ---------- RSIM-07: README documents Garden EOL + Harmonic migration + smoke ----------

def test_readme_documents_eol_and_smoke() -> None:
    """RSIM-07: README must document Gazebo Garden EOL (November 2024),
    the Harmonic migration path (``ros-humble-ros-gzharmonic-bridge`` apt
    name substitution), and the ``gz topic -l`` smoke-test step."""
    assert ROVER_README.is_file(), (
        f"{ROVER_README} missing — Plan 05-03 not landed?"
    )
    text = ROVER_README.read_text()
    assert "November 2024" in text, (
        f"{ROVER_README}: Garden EOL date 'November 2024' absent (RSIM-07)"
    )
    assert "gz topic -l" in text, (
        f"{ROVER_README}: 'gz topic -l' smoke step absent (RSIM-07)"
    )
    assert "ros-humble-ros-gzharmonic-bridge" in text, (
        f"{ROVER_README}: Harmonic migration apt-name absent (RSIM-07)"
    )


# ---------- Optional: gz sdf -k lint, skip-on-no-gz ----------

def test_sdfs_lint_clean_with_gz() -> None:
    """RSIM-01..03: every standalone SDF under sim/rover/ parses cleanly
    with ``gz sdf -k``.  SDFs that ``<include><uri>model://...</uri></include>``
    a sibling model are skipped because ``gz sdf -k`` has no find-callback
    registered (that hookup is `gz sim` runtime only), so include resolution
    fails with ``Tried to use callback in sdf::findFile()`` regardless of
    GZ_SIM_RESOURCE_PATH.  Include validity is covered by
    ``test_world_sdfs_include_rover_model`` instead.  Skips when gz is not
    installed (Garden / Harmonic contributor boxes both supply ``gz``; CI
    without Gazebo skips)."""
    if not shutil.which("gz"):
        pytest.skip("gz not installed")
    sdfs = sorted(ROVER_DIR.rglob("*.sdf"))
    assert sdfs, f"{ROVER_DIR}: no SDFs found"
    # An SDF is "standalone" for `gz sdf -k` if its XML body (comments
    # stripped) has no <uri>model://...</uri> to resolve.  XML comments
    # mentioning the pattern are fine; the parser only chokes on real
    # <uri> include resolution.
    comment_re = re.compile(r"<!--.*?-->", re.DOTALL)
    standalone = [
        s for s in sdfs
        if "<uri>model://" not in comment_re.sub("", s.read_text())
    ]
    assert standalone, (
        f"{ROVER_DIR}: no standalone SDFs to lint (all use model:// includes; "
        f"unexpected on a Phase 5 tree that must include rover.sdf)"
    )
    for sdf in standalone:
        result = subprocess.run(
            ["gz", "sdf", "-k", str(sdf)],
            capture_output=True,
            text=True,
            cwd=str(PROJECT_ROOT),
        )
        assert result.returncode == 0, (
            f"gz sdf -k failed on {sdf}:\n"
            f"--- stdout ---\n{result.stdout}\n--- stderr ---\n{result.stderr}"
        )
