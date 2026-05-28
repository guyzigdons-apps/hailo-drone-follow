"""Tests for follow_api.follow_mode.

Pins the contract used by:
* the OpenHD v4 binary detection payload encoder (live + the loopback)
* the live cairo overlay badge
* the offline overlay renderer's mode-to-text mapping
"""

from robot_follow.follow_api.follow_mode import FollowMode


def test_enum_values_are_strings():
    """(str, Enum) inheritance means instances compare equal to the
    wire string; existing string comparisons in the codebase keep
    working unchanged."""
    assert FollowMode.AUTO == "AUTO"
    assert FollowMode.LOCKED == "LOCKED"
    assert FollowMode.SEARCH == "SEARCH"
    assert FollowMode.IDLE == "IDLE"


def test_byte_mapping_is_stable():
    """Wire contract — the OpenHD C++ encoder
    (ohd_video/src/hailo_follow_bridge.cpp) reads these exact byte
    values. Changing them silently breaks every ground station.
    """
    assert FollowMode.AUTO.byte == 0
    assert FollowMode.LOCKED.byte == 1
    assert FollowMode.SEARCH.byte == 2
    assert FollowMode.IDLE.byte == 3


def test_byte_round_trip():
    for m in FollowMode:
        assert FollowMode.from_byte(m.byte) is m


def test_from_byte_unknown_falls_back_to_auto():
    """An unrecognised byte (e.g. forward-compat from a future
    drone-follow against an older ground station) shouldn't crash."""
    assert FollowMode.from_byte(99) is FollowMode.AUTO
    assert FollowMode.from_byte(-1) is FollowMode.AUTO


def test_from_str_accepts_known_strings():
    assert FollowMode.from_str("LOCKED") is FollowMode.LOCKED
    assert FollowMode.from_str("SEARCH") is FollowMode.SEARCH


def test_from_str_none_returns_default():
    assert FollowMode.from_str(None) is FollowMode.AUTO
    assert FollowMode.from_str(None, FollowMode.IDLE) is FollowMode.IDLE


def test_from_str_unknown_returns_default():
    assert FollowMode.from_str("PARTY_MODE") is FollowMode.AUTO


def test_enum_serialises_as_string_in_json():
    """str inheritance means json.dumps emits the plain wire string,
    not "FollowMode.LOCKED". This is what the OpenHD JSON payload
    expects on the other end."""
    import json
    assert json.dumps({"mode": FollowMode.LOCKED}) == '{"mode": "LOCKED"}'
