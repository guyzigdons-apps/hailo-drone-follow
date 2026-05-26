"""CLEAN-15 acceptance — decide_branches() single source of truth.

Post-fix (plan 02-05): vision_branches.decide_branches(...) returns
BranchDecision; called from one place; the 16-combo input matrix produces
the expected output. The implicit-display rule and --openhd/--webui mutex
live exclusively inside the helper.
"""
import itertools

import pytest

from robot_follow.pipeline_adapter.vision_branches import decide_branches


def test_no_flags_defaults_display_true():
    d = decide_branches(openhd=False, webui=False, display=False, record=False)
    assert d.display is True
    assert d.record_branch_enabled is False
    assert d.webui is False
    assert d.openhd is False


def test_explicit_display_only():
    d = decide_branches(openhd=False, webui=False, display=True, record=False)
    assert d.display is True
    assert d.record_branch_enabled is False


def test_webui_implies_display_false():
    d = decide_branches(openhd=False, webui=True, display=False, record=False)
    assert d.display is False   # implicit-display rule: webui consumes the branch
    assert d.webui is True
    assert d.record_branch_enabled is True  # record auto-on under webui


def test_openhd_implies_display_false():
    d = decide_branches(openhd=True, webui=False, display=False, record=False)
    assert d.display is False
    assert d.openhd is True
    assert d.record_branch_enabled is True  # record auto-on under openhd


def test_openhd_and_webui_raises():
    with pytest.raises(ValueError, match="mutually exclusive"):
        decide_branches(openhd=True, webui=True, display=False, record=False)


def test_record_alone_does_not_force_display_off():
    d = decide_branches(openhd=False, webui=False, display=False, record=True)
    assert d.display is True   # neither openhd nor webui → display still on
    assert d.record_branch_enabled is True


@pytest.mark.parametrize(
    "openhd,webui,display,record",
    [combo for combo in itertools.product([False, True], repeat=4)
     if not (combo[0] and combo[1])],  # exclude the mutex-raising combo
)
def test_full_matrix_returns_branch_decision(openhd, webui, display, record):
    d = decide_branches(openhd=openhd, webui=webui, display=display, record=record)
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
