"""Sparse JSONL writer for operator-action and follow-state events.

Lives next to the per-frame recording log (``frames.jsonl``) so the
offline overlay renderer can attribute followee transitions ("USER
LOCKED ID 5" vs "REID re-acquired") and so flight reviews have a
single source of truth for who did what when.

Lifecycle
---------
A process-global singleton, opened by ``start_recording`` and closed
by ``stop_recording``. Emits are thread-safe and silently no-op when
no log file is open, so call sites don't need to know whether a
recording is in progress. Also usable as a context manager::

    with EventLog.get().session(path) as log:
        log.emit(EventKind.RECORD, action="start")
        # ...
        log.emit(EventKind.RECORD, action="stop")

Schema
------
Each kind has a fixed set of fields. See :data:`EVENT_LOG_SCHEMA` for
the JSON Schema definition. :class:`EventKind` and :class:`FollowCause`
provide the typed enums used by writers and the renderer's toast
resolver.

Wire example (line-delimited JSON)::

    {"t": 1701234567.123, "kind": "click",         "source": "webui",   "id": 5}
    {"t": ...,             "kind": "follow_change", "from": 3, "to": 5, "cause": "USER"}
    {"t": ...,             "kind": "reacquire",     "track_id": 5}
    {"t": ...,             "kind": "record",        "action": "start"}
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from enum import Enum
from typing import Any, Iterator, Optional, Union

LOGGER = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Typed enums
# ---------------------------------------------------------------------------

class EventKind(str, Enum):
    """Top-level event types written to ``events.jsonl``.

    ``(str, Enum)`` so ``EventKind.CLICK == "click"`` works and the
    enum can be passed directly to ``json.dumps`` without conversion.
    """
    CLICK         = "click"
    FOLLOW_CHANGE = "follow_change"
    REACQUIRE     = "reacquire"
    RECORD        = "record"


class FollowCause(str, Enum):
    """Why the followee transitioned. Drives the offline renderer's
    toast text — "USER LOCKED ID 5" vs "REID RE-ACQUIRED ID 5", etc.
    """
    USER       = "USER"          # operator click in webui / OpenHD
    CLEAR      = "CLEAR"         # operator pressed Clear / AUTO
    REID       = "REID"          # ReID gallery re-identified
    REID_DRIFT = "REID-DRIFT"    # drift detected, switched tracks
    AUTO       = "AUTO"          # auto-acquired biggest person
    TIMEOUT    = "TIMEOUT"       # ReID search exceeded budget


# ---------------------------------------------------------------------------
# JSON Schema (Draft 2020-12)
# ---------------------------------------------------------------------------
#
# Shipped as a plain dict so consumers without jsonschema installed can
# still inspect it and use the enum lists for ad-hoc validation. The
# offline renderer's EventIndex relies on ``kind == "follow_change"``
# and the field names below; treat them as a wire contract.

EVENT_LOG_SCHEMA: dict = {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "title": "robot-follow event log row",
    "type": "object",
    "required": ["t", "kind"],
    "properties": {
        "t":    {"type": "number", "description": "wall-clock timestamp (seconds)"},
        "kind": {"type": "string", "enum": [e.value for e in EventKind]},
    },
    "oneOf": [
        {
            "properties": {
                "kind":   {"const": EventKind.CLICK.value},
                "source": {"type": "string", "enum": ["webui", "openhd"]},
                "id":     {"type": "integer", "minimum": 0},
            },
            "required": ["source", "id"],
        },
        {
            "properties": {
                "kind":  {"const": EventKind.FOLLOW_CHANGE.value},
                "from":  {"type": ["integer", "null"]},
                "to":    {"type": ["integer", "null"]},
                "cause": {"type": "string", "enum": [c.value for c in FollowCause]},
            },
            "required": ["from", "to", "cause"],
        },
        {
            "properties": {
                "kind":       {"const": EventKind.REACQUIRE.value},
                "track_id":   {"type": "integer", "minimum": 0},
                "similarity": {"type": "number"},
            },
            "required": ["track_id"],
        },
        {
            "properties": {
                "kind":   {"const": EventKind.RECORD.value},
                "action": {"type": "string", "enum": ["start", "stop"]},
                "bundle": {"type": "string"},
            },
            "required": ["action"],
        },
    ],
}


# ---------------------------------------------------------------------------
# Writer
# ---------------------------------------------------------------------------

class EventLog:
    """Thread-safe sparse JSONL writer with a process-global singleton.

    Two ways to drive it:

      * **Singleton** — call ``EventLog.get().open(path)`` once at
        record-start, ``EventLog.get().close()`` at stop. All site-level
        emit helpers (:func:`log_click`, :func:`log_follow_change`, …)
        target the singleton.

      * **Context manager** — for tests or one-off captures::

            with EventLog.get().session(path):
                log_click("webui", 5)
                log_follow_change(None, 5, FollowCause.USER)
    """

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

    @property
    def is_open(self) -> bool:
        return self._file is not None

    def open(self, path: str) -> "EventLog":
        """Open the log file (line-buffered). Closes any existing handle.
        Returns self so callers can chain ``.open(path).emit(...)``.
        """
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with self._file_lock:
            self._close_unlocked()
            self._file = open(path, "w", buffering=1)
            self._path = path
            LOGGER.info("[events] writing event log to %s", path)
        return self

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

    # -- Context-manager support (Sagigamil review feedback) ------------------

    def __enter__(self) -> "EventLog":
        # No-op enter; the caller is expected to have already called
        # ``open()`` (or used :meth:`session` to bundle the two).
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def session(self, path: str) -> "EventLog":
        """Open ``path`` and return self for use in a ``with`` block::

            with EventLog.get().session("events.jsonl") as log:
                log.emit(EventKind.CLICK, source="webui", id=5)

        Closes on exit even if an exception is raised inside the block.
        """
        return self.open(path)

    # -- Emit -----------------------------------------------------------------

    def emit(self, kind: Union[EventKind, str], **fields: Any) -> None:
        """Write a single JSON line. ``t`` (wall-clock) and ``kind`` are
        auto-added; the caller supplies any other fields. ``kind`` may be
        either an :class:`EventKind` enum or its string value — both
        serialise identically.

        No-op when no log is open — sites can call this unconditionally.
        """
        kind_str = kind.value if isinstance(kind, EventKind) else str(kind)
        with self._file_lock:
            if self._file is None:
                return
            row = {"t": time.time(), "kind": kind_str, **_jsonify(fields)}
            try:
                self._file.write(json.dumps(row) + "\n")
            except (OSError, ValueError, AttributeError):
                # Mirrors the resilience pattern in app_callback's
                # frame-log writer — never crash the live path because
                # a log handle vanished.
                pass

    # -- Reader (used by the offline renderer + tests) ------------------------

    @staticmethod
    def iter_rows(path: str) -> Iterator[dict]:
        """Yield parsed rows from a previously-written log file.

        Drops malformed lines silently — operationally, a single corrupt
        line shouldn't take out a flight review.
        """
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    yield json.loads(line)
                except json.JSONDecodeError:
                    continue


def _jsonify(d: dict) -> dict:
    """Coerce enum values to their underlying str so json.dumps emits
    the plain string (not ``"EventKind.CLICK"``). Recursive only one
    level deep — sufficient for the flat schema we use.
    """
    out = {}
    for k, v in d.items():
        if isinstance(v, Enum):
            out[k] = v.value
        else:
            out[k] = v
    return out


# ---------------------------------------------------------------------------
# Convenience helpers
# ---------------------------------------------------------------------------
# Site-friendly wrappers so callers don't repeat the kind string. All are
# silent no-ops when no log is open (see EventLog.emit), so they're safe
# to call unconditionally.

def log_click(source: str, det_id: int) -> None:
    """Operator clicked a bbox in the UI (``source`` is ``"webui"`` /
    ``"openhd"``).
    """
    EventLog.get().emit(EventKind.CLICK, source=source, id=int(det_id))


def log_follow_change(from_id, to_id, cause: Union[FollowCause, str]) -> None:
    """The followee just transitioned. ``cause`` may be a
    :class:`FollowCause` or its string value.
    """
    EventLog.get().emit(
        EventKind.FOLLOW_CHANGE,
        **{"from": int(from_id) if from_id is not None else None},
        to=int(to_id) if to_id is not None else None,
        cause=cause,
    )


def log_reacquire(track_id: int, similarity: Optional[float] = None) -> None:
    """ReID found the locked person again. ``similarity`` optional."""
    fields = {"track_id": int(track_id)}
    if similarity is not None:
        fields["similarity"] = round(float(similarity), 4)
    EventLog.get().emit(EventKind.REACQUIRE, **fields)


def log_record(action: str, bundle: Optional[str] = None) -> None:
    """``action`` ∈ ``"start"`` / ``"stop"``. Bookends the session."""
    fields: dict = {"action": action}
    if bundle:
        fields["bundle"] = bundle
    EventLog.get().emit(EventKind.RECORD, **fields)
