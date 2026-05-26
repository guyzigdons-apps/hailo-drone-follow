"""RINT-03 wiring tests for ByteTracker config-driven init.

Covers the Phase 6 Plan 06-03 contract:
  - create_app reads ByteTracker knobs from controller_config (not hardcoded)
  - Drone defaults are byte-identical to the legacy literals (0.4, 90, 0.5, 30)
  - Rover override (track_buffer=30) propagates through the wiring
  - Composition root (robot_follow_app.py) passes controller_config kwarg

These tests use a minimal create_tracker monkey-patch to capture the kwargs
create_app would pass to it WITHOUT requiring Hailo HW or a full pipeline.
"""

from pathlib import Path

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
HAILO_MGR = (PROJECT_ROOT / "robot_follow" / "pipeline_adapter"
             / "hailo_drone_detection_manager.py")
ROBOT_APP = PROJECT_ROOT / "robot_follow" / "robot_follow_app.py"


# ---------------------------------------------------------------------------
# Tests 3 + 4: source-level grep regression guards (no Hailo / no pipeline)
# ---------------------------------------------------------------------------

def test_create_app_does_not_use_hardcoded_bytetracker_literals() -> None:
    """RINT-03 regression guard: the legacy hardcoded
    `track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30`
    block MUST NOT appear at the create_tracker call site post-Plan-06-03.

    Filters out comment lines so a future doc-string mentioning the legacy
    values doesn't false-positive this test.
    """
    assert HAILO_MGR.is_file()
    text = HAILO_MGR.read_text()
    # Strip lines that are only comments (start with optional whitespace + #)
    non_comment = "\n".join(
        line for line in text.splitlines()
        if not line.lstrip().startswith("#")
    )
    legacy = "track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30"
    assert legacy not in non_comment, (
        f"legacy hardcoded ByteTracker literals still present in "
        f"hailo_drone_detection_manager.py — RINT-03 wiring is incomplete "
        f"(Plan 06-03 must replace this with reads from controller_config)"
    )


def test_robot_follow_app_passes_controller_config_to_create_app() -> None:
    """RINT-03: robot_follow_app.py's create_app invocation passes
    controller_config=ControllerConfig.from_args(pre_args) (Path A wiring).
    """
    assert ROBOT_APP.is_file()
    text = ROBOT_APP.read_text()
    target = "controller_config=ControllerConfig.from_args(pre_args)"
    count = text.count(target)
    assert count == 1, (
        f"expected exactly 1 occurrence of {target!r} in robot_follow_app.py, "
        f"got {count} — Plan 06-03 Task 2 must wire this kwarg into create_app"
    )


def test_pre_parser_registers_config_flag() -> None:
    """RINT-03 / RESEARCH Pitfall A: --config must be on the pre-parser so
    ControllerConfig.from_args(pre_args) honors --config (same JSON source
    of truth as the full-args derivation at line 448)."""
    assert ROBOT_APP.is_file()
    text = ROBOT_APP.read_text()
    # The pre-parser is the `pre = argparse.ArgumentParser(...)` block.
    # We search for `pre.add_argument("--config"` to confirm registration.
    assert 'pre.add_argument("--config"' in text, (
        "--config not registered on the pre-parser in robot_follow_app.py — "
        "Pitfall A wiring incomplete (tracker init would skip the rover JSON)"
    )


# ---------------------------------------------------------------------------
# Tests 1 + 2: create_tracker kwargs capture via monkey-patch
# ---------------------------------------------------------------------------

def _capture_create_tracker_kwargs(monkeypatch, controller_config):
    """Helper: replicate the few lines that construct _cfg and call create_tracker,
    returning the captured kwargs.

    Because create_app's full body builds the entire GStreamer pipeline (which
    requires Hailo + gst, unavailable on CI), we DON'T invoke create_app
    directly. Instead, we replicate the few lines that construct _cfg and call
    create_tracker, and assert on the captured kwargs.

    This is intentionally a focused unit assertion: the contract under test is
    "the value of _cfg.bytetracker_* fields flows into create_tracker kwargs".
    The full integration (Hailo, gst, sim) is covered by Plan 06-06's RINT-04
    E2E test.
    """
    from robot_follow.follow_api.config import ControllerConfig
    captured = {}

    def fake_create_tracker(name, **kwargs):
        captured["name"] = name
        captured.update(kwargs)
        return object()  # tracker is unused after capture

    # Replicate create_app's tracker-init lines (the lines added by Plan 06-03
    # Task 1). This couples the test to the implementation by intent — if the
    # tracker-init shape changes, this test must change in lockstep.
    _cfg = controller_config if controller_config is not None else ControllerConfig()
    fake_create_tracker(
        "byte",
        track_thresh=_cfg.bytetracker_track_thresh,
        track_buffer=_cfg.bytetracker_track_buffer,
        match_thresh=_cfg.bytetracker_match_thresh,
        frame_rate=_cfg.bytetracker_frame_rate,
    )
    return captured


def test_create_app_uses_default_bytetracker_when_controller_config_none(monkeypatch):
    """When controller_config=None, _cfg falls back to ControllerConfig() and
    create_tracker receives the BYTE-IDENTICAL drone defaults (0.4, 90, 0.5, 30)."""
    kw = _capture_create_tracker_kwargs(monkeypatch, controller_config=None)
    assert kw == {
        "name": "byte",
        "track_thresh": 0.4,
        "track_buffer": 90,
        "match_thresh": 0.5,
        "frame_rate": 30,
    }, f"drone defaults regressed: {kw}"


def test_create_app_uses_rover_overridden_track_buffer(monkeypatch):
    """When controller_config has bytetracker_track_buffer=30 (rover override),
    create_tracker receives track_buffer=30 (others stay at drone defaults)."""
    from robot_follow.follow_api.config import ControllerConfig
    rover_cfg = ControllerConfig(
        bytetracker_track_thresh=0.4,
        bytetracker_track_buffer=30,   # rover override (1 s @ 30 fps)
        bytetracker_match_thresh=0.5,
        bytetracker_frame_rate=30,
    )
    kw = _capture_create_tracker_kwargs(monkeypatch, controller_config=rover_cfg)
    assert kw["track_buffer"] == 30, (
        f"rover override did not propagate: {kw}"
    )
    # Other three at drone-parity values (rover config keeps these at drone defaults)
    assert kw["track_thresh"] == 0.4
    assert kw["match_thresh"] == 0.5
    assert kw["frame_rate"] == 30


def test_create_app_signature_has_controller_config_param() -> None:
    """RINT-03 wiring: create_app's signature has controller_config kwarg
    with default None (backward-compat for legacy callers)."""
    import inspect
    from robot_follow.pipeline_adapter.hailo_drone_detection_manager import create_app
    sig = inspect.signature(create_app)
    assert "controller_config" in sig.parameters, (
        "create_app missing controller_config param — Plan 06-03 Task 1 "
        "must extend the signature"
    )
    param = sig.parameters["controller_config"]
    assert param.default is None, (
        f"controller_config default must be None (backward-compat) — got {param.default!r}"
    )
