"""Sparse JSONL writer for operator-action and follow-state events.

Lives next to the per-frame recording log (``frames.jsonl``) so the
offline overlay renderer can attribute followee transitions ("USER
LOCKED ID 5" vs "REID re-acquired") and so flight reviews have a
single source of truth for who did what when.

Lifecycle: a process-global singleton, opened by ``start_recording``
and closed by ``stop_recording``. Emits are thread-safe and silently
no-op when no log file is open, so call sites don't need to know
whether a recording is in progress.

Record schema (line-delimited JSON)::

    {"t": 1701234567.123, "kind": "click",          "source": "webui",   "id": 5}
    {"t": ..., "kind": "follow_change",  "from": 3, "to": 5, "cause": "USER"}
    {"t": ..., "kind": "reacquire",      "track_id": 5}
    {"t": ..., "kind": "record",         "action": "start"}

``cause`` strings: ``"USER"`` / ``"REID"`` / ``"REID-DRIFT"`` /
``"AUTO"`` / ``"TIMEOUT"`` / ``"CLEAR"``. Sites without rich
attribution may omit the field or pass ``""``.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from typing import Any, Optional

LOGGER = logging.getLogger(__name__)


class EventLog:
    """Thread-safe sparse JSONL writer with a process-global singleton."""

    _instance: Optional["EventLog"] = None
    _instance_lock = threading.Lock()

    def __init__(self) -> None:
        self._file_lock = threading.Lock()
        self._file = None
        self._path: Optional[str] = None

    @classmethod
    def get(cls) -> "EventLog":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = EventLog()
            return cls._instance

    @property
    def path(self) -> Optional[str]:
        return self._path

    def open(self, path: str) -> None:
        """Open the log file (line-buffered). Closes any existing handle."""
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with self._file_lock:
            self._close_unlocked()
            self._file = open(path, "w", buffering=1)
            self._path = path
            LOGGER.info("[events] writing event log to %s", path)

    def close(self) -> None:
        with self._file_lock:
            self._close_unlocked()

    def _close_unlocked(self) -> None:
        if self._file is None:
            return
        try:
            self._file.close()
        except OSError:
            pass
        self._file = None
        self._path = None

    def emit(self, kind: str, **fields: Any) -> None:
        """Write a single JSON line. ``t`` (wall-clock) and ``kind`` are
        auto-added; the caller supplies any other fields. No-op when
        no log is open — sites can call this unconditionally.
        """
        with self._file_lock:
            if self._file is None:
                return
            row = {"t": time.time(), "kind": kind, **fields}
            try:
                self._file.write(json.dumps(row, default=str) + "\n")
            except (OSError, ValueError, AttributeError):
                # Mirrors the resilience pattern in app_callback's
                # frame-log writer — never crash the live path because
                # a log handle vanished.
                pass


# -- Convenience helpers ------------------------------------------------------
# Site-friendly wrappers so callers don't repeat the kind string. All are
# silent no-ops when no log is open (see EventLog.emit), so they're safe
# to call unconditionally.

def log_click(source: str, det_id: int) -> None:
    """Operator clicked a bbox in the UI (``source`` is ``"webui"`` /
    ``"openhd"``).
    """
    EventLog.get().emit("click", source=source, id=int(det_id))


def log_follow_change(from_id, to_id, cause: str) -> None:
    """The followee just transitioned. ``cause`` ∈ ``USER`` / ``REID`` /
    ``REID-DRIFT`` / ``AUTO`` / ``TIMEOUT`` / ``CLEAR``.
    """
    EventLog.get().emit(
        "follow_change",
        **{"from": int(from_id) if from_id is not None else None},
        to=int(to_id) if to_id is not None else None,
        cause=cause,
    )


def log_reacquire(track_id: int, similarity: Optional[float] = None) -> None:
    """ReID found the locked person again. ``similarity`` optional."""
    fields = {"track_id": int(track_id)}
    if similarity is not None:
        fields["similarity"] = round(float(similarity), 4)
    EventLog.get().emit("reacquire", **fields)


def log_record(action: str, bundle: Optional[str] = None) -> None:
    """``action`` ∈ ``"start"`` / ``"stop"``. Bookends the session."""
    fields = {"action": action}
    if bundle:
        fields["bundle"] = bundle
    EventLog.get().emit("record", **fields)
