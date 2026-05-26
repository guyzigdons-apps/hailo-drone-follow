"""Sanity tests for follow_api.event_log.

Pins three contracts:
1. ``emit`` is a no-op when no log file is open (so call sites don't
   need to guard).
2. ``open`` then ``emit`` writes a JSON line with ``t`` and ``kind``
   auto-added.
3. The convenience wrappers (``log_click`` / ``log_follow_change`` /
   ``log_reacquire`` / ``log_record``) emit the expected field shapes.
"""

import json
from pathlib import Path

import pytest

from robot_follow.follow_api.event_log import (
    EventLog, log_click, log_follow_change, log_reacquire, log_record,
)


@pytest.fixture(autouse=True)
def _isolate_singleton():
    """Reset the EventLog singleton between tests so each test gets a
    clean writer.  We use the same module-level singleton in production,
    but tests must not leak file handles to each other.
    """
    EventLog._instance = None
    yield
    EventLog.get().close()
    EventLog._instance = None


def _read_rows(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def test_emit_is_noop_when_closed():
    # No log open — emit should silently no-op (not raise).
    log_click(source="webui", det_id=5)
    log_follow_change(None, 5, cause="USER")
    log_reacquire(5)
    # If we got here without an exception, the contract holds.


def test_emit_writes_t_and_kind(tmp_path):
    path = tmp_path / "events.jsonl"
    EventLog.get().open(str(path))
    EventLog.get().emit("custom", extra="value")
    EventLog.get().close()
    rows = _read_rows(path)
    assert len(rows) == 1
    assert rows[0]["kind"] == "custom"
    assert "t" in rows[0] and isinstance(rows[0]["t"], float)
    assert rows[0]["extra"] == "value"


def test_log_click_shape(tmp_path):
    path = tmp_path / "events.jsonl"
    EventLog.get().open(str(path))
    log_click(source="openhd", det_id=7)
    rows = _read_rows(path)
    assert rows == [{**rows[0],
                     "kind": "click", "source": "openhd", "id": 7}]


def test_log_follow_change_shape(tmp_path):
    path = tmp_path / "events.jsonl"
    EventLog.get().open(str(path))
    log_follow_change(3, 5, cause="USER")
    log_follow_change(5, None, cause="CLEAR")
    rows = _read_rows(path)
    assert rows[0]["from"] == 3
    assert rows[0]["to"] == 5
    assert rows[0]["cause"] == "USER"
    assert rows[1]["from"] == 5
    assert rows[1]["to"] is None
    assert rows[1]["cause"] == "CLEAR"


def test_log_reacquire_shape(tmp_path):
    path = tmp_path / "events.jsonl"
    EventLog.get().open(str(path))
    log_reacquire(7)
    log_reacquire(8, similarity=0.835)
    rows = _read_rows(path)
    assert rows[0]["kind"] == "reacquire"
    assert rows[0]["track_id"] == 7
    assert "similarity" not in rows[0]
    assert rows[1]["track_id"] == 8
    assert rows[1]["similarity"] == 0.835


def test_log_record_bookends(tmp_path):
    path = tmp_path / "events.jsonl"
    EventLog.get().open(str(path))
    log_record("start", bundle="/tmp/rec/xyz")
    log_record("stop")
    rows = _read_rows(path)
    assert [r["kind"] for r in rows] == ["record", "record"]
    assert rows[0]["action"] == "start"
    assert rows[0]["bundle"] == "/tmp/rec/xyz"
    assert rows[1]["action"] == "stop"
    assert "bundle" not in rows[1]


def test_reopen_truncates(tmp_path):
    path = tmp_path / "events.jsonl"
    EventLog.get().open(str(path))
    log_record("start")
    EventLog.get().open(str(path))  # re-open same path — truncate semantics
    log_record("stop")
    rows = _read_rows(path)
    assert len(rows) == 1
    assert rows[0]["action"] == "stop"
