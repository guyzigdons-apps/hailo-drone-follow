"""CLEAN-15 acceptance — decide_branches() single source of truth.

Pre-fix: implicit-display rule duplicated at 3 sites (robot_follow_app.py:226,
:330; hailo_drone_detection_manager.py:1166); record-branch-enabled rule and
--openhd/--webui mutex live elsewhere.
Post-fix (plan 02-05): vision_branches.decide_branches(...) returns
BranchDecision; called from one place; the 16-combo input matrix produces the
expected output.

Marked xfail until plan 02-05 lands the helper.
"""
import itertools

import pytest


XFAIL_REASON = "CLEAN-15 decide_branches not yet landed; closes in plan 02-05"


def _decide():
    # Lazy import so collection doesn't fail before 02-05 lands the symbol.
    from robot_follow.pipeline_adapter.vision_branches import decide_branches
    return decide_branches


@pytest.mark.xfail(reason=XFAIL_REASON, strict=False)
def test_no_flags_defaults_display_true():
    d = _decide()(openhd=False, webui=False, display=False, record=False)
    assert d.display is True
    assert d.record_branch_enabled is False
    assert d.webui is False
    assert d.openhd is False


@pytest.mark.xfail(reason=XFAIL_REASON, strict=False)
def test_explicit_display_only():
    d = _decide()(openhd=False, webui=False, display=True, record=False)
    assert d.display is True
    assert d.record_branch_enabled is False


@pytest.mark.xfail(reason=XFAIL_REASON, strict=False)
def test_webui_implies_display_false():
    d = _decide()(openhd=False, webui=True, display=False, record=False)
    assert d.display is False   # implicit-display rule: webui consumes the branch
    assert d.webui is True
    assert d.record_branch_enabled is True  # record auto-on under webui


@pytest.mark.xfail(reason=XFAIL_REASON, strict=False)
def test_openhd_implies_display_false():
    d = _decide()(openhd=True, webui=False, display=False, record=False)
    assert d.display is False
    assert d.openhd is True
    assert d.record_branch_enabled is True  # record auto-on under openhd


@pytest.mark.xfail(reason=XFAIL_REASON, strict=False)
def test_openhd_and_webui_raises():
    with pytest.raises(ValueError, match="mutually exclusive"):
        _decide()(openhd=True, webui=True, display=False, record=False)


@pytest.mark.xfail(reason=XFAIL_REASON, strict=False)
def test_record_alone_does_not_force_display_off():
    d = _decide()(openhd=False, webui=False, display=False, record=True)
    assert d.display is True   # neither openhd nor webui → display still on
    assert d.record_branch_enabled is True


@pytest.mark.xfail(reason=XFAIL_REASON, strict=False)
@pytest.mark.parametrize(
    "openhd,webui,display,record",
    [combo for combo in itertools.product([False, True], repeat=4)
     if not (combo[0] and combo[1])],  # exclude the mutex-raising combo
)
def test_full_matrix_returns_branch_decision(openhd, webui, display, record):
    d = _decide()(openhd=openhd, webui=webui, display=display, record=record)
    # Invariants:
    # 1. record_branch_enabled = record OR webui OR openhd
    assert d.record_branch_enabled == (record or webui or openhd)
    # 2. webui/openhd pass through
    assert d.webui is webui
    assert d.openhd is openhd
    # 3. Implicit-display rule: when neither openhd nor webui is set, display is forced True
    if not openhd and not webui:
        assert d.display is True
    else:
        assert d.display is display  # explicit flag respected
