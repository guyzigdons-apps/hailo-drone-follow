"""Follow-state enum shared across the overlay, OpenHD bridge, and the
offline renderer.

Each value is also a string (``class FollowMode(str, Enum)``), so
existing string comparisons (``mode == "LOCKED"``) keep working unchanged
while new code can use the typed enum.

Wire-format mapping for the OpenHD v4 binary detection payload's mode
byte is exposed via :meth:`FollowMode.byte`; mirrors the OpenHD C++
encoder in ``ohd_video/src/hailo_follow_bridge.cpp``.
"""

from __future__ import annotations

from enum import Enum
from typing import Optional


class FollowMode(str, Enum):
    """Operator-visible follow state.

    Drawn as the top-left badge on every overlay (live cairo + offline
    renderer + QOpenHD ground station) and serialised into the OpenHD
    v4 binary payload's mode byte.
    """

    AUTO   = "AUTO"
    LOCKED = "LOCKED"
    SEARCH = "SEARCH"
    IDLE   = "IDLE"

    # Wire byte for the OpenHD v4 detection payload. Stable contract —
    # changing this breaks ground-station rendering until the OpenHD
    # C++ + QOpenHD QML are updated in lockstep.
    @property
    def byte(self) -> int:
        return _MODE_TO_BYTE[self]

    @classmethod
    def from_byte(cls, b: int) -> "FollowMode":
        return _BYTE_TO_MODE.get(b, cls.AUTO)

    @classmethod
    def from_str(cls, s: Optional[str], default: "FollowMode" = None) -> "FollowMode":
        """Forgiving parser — accepts the enum value or any equivalent
        string. Falls back to ``default`` (or AUTO) on unknown input.
        """
        if s is None:
            return default if default is not None else cls.AUTO
        try:
            return cls(s)
        except ValueError:
            return default if default is not None else cls.AUTO


_MODE_TO_BYTE = {
    FollowMode.AUTO:   0,
    FollowMode.LOCKED: 1,
    FollowMode.SEARCH: 2,
    FollowMode.IDLE:   3,
}
_BYTE_TO_MODE = {v: k for k, v in _MODE_TO_BYTE.items()}
