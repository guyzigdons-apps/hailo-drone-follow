"""Tests for `hailo_tiling.telemetry.srt.parse_srt`.

The committed fixture `tiny.srt` is a 30-block (~1 s @ 30 fps) trimmed
copy of a real DJI sidecar with anonymised GPS; see `tiny.srt.README.md`
for provenance.
"""
from __future__ import annotations

from pathlib import Path

import pytest

from hailo_tiling.telemetry.srt import (
    _bracket_pairs,
    _parse_block,
    _split_blocks,
    parse_srt,
)


FIXTURE = Path(__file__).parent / "fixtures" / "telemetry" / "tiny.srt"


def test_parse_srt_smoke():
    rows = parse_srt(FIXTURE)
    assert len(rows) >= 5
    assert rows[0]["timestamp"] == 0.0
    for i in range(len(rows) - 1):
        assert rows[i]["timestamp"] <= rows[i + 1]["timestamp"], (
            f"row {i} ts {rows[i]['timestamp']} > row {i+1} ts {rows[i+1]['timestamp']}"
        )


def test_parse_srt_geo_extraction():
    rows = parse_srt(FIXTURE)
    # The fixture was anonymised by multiplying lat / lon by 0.001, so
    # we expect lat ≈ 0.031884 (= 0.001 * 31.883741).
    assert rows
    geo = rows[0]["_geo"]
    assert geo["lat"] is not None
    assert geo["lon"] is not None
    assert geo["lat"] == pytest.approx(0.031884, abs=1e-6)
    assert geo["lon"] == pytest.approx(0.035027, abs=1e-6)
    # focal_len comes through as a float (the fixture has [focal_len: 28.00]).
    assert geo["focal_len_mm"] == pytest.approx(28.0)
    # alt_msl comes from `abs_alt` inside the compound `[rel_alt: ... abs_alt: ...]` bracket.
    assert geo["alt_msl"] is not None
    assert 300.0 < geo["alt_msl"] < 305.0  # ≈ 302.6 in the fixture
    # The visualizer-facing AGL-source marker.
    assert geo["_agl_source"] == "rel_alt"
    # Pitch / roll are not derivable from SRT; ULG-shape compatibility.
    assert geo["pitch"] is None
    assert geo["roll"] is None


def test_parse_srt_no_velocity_or_attitude():
    rows = parse_srt(FIXTURE)
    assert rows
    for r in rows:
        assert r["velocity_world"] is None, f"row {r} has unexpected velocity_world"
        assert r["attitude_quat"] is None, f"row {r} has unexpected attitude_quat"
        assert r["yaw_rate_rad_s"] is None, f"row {r} has unexpected yaw_rate_rad_s"


def test_parse_srt_handles_missing_optional_keys():
    """A synthetic block missing the `[ev: ...]` bracket must not crash."""
    block = (
        "1\n"
        "00:00:00,000 --> 00:00:00,033\n"
        '<font size="28">FrameCnt: 1, DiffTime: 33ms\n'
        "2026-04-30 10:47:32.749\n"
        "[iso: 100] [shutter: 1/2500.0] [fnum: 2.8] [focal_len: 28.00] "
        "[latitude: 0.031884] [longitude: 0.035027] "
        "[rel_alt: 7.410 abs_alt: 302.607] </font>"
    )
    parsed = _parse_block(block)
    assert parsed["frame_cnt"] == 1
    assert parsed["iso_ts"] is not None
    # `ev` is absent — must not be in the kv dict and must not crash.
    assert "ev" not in parsed["kv"]
    # Present keys are still extracted.
    assert parsed["kv"]["iso"] == "100"
    assert parsed["kv"]["focal_len"] == "28.00"
    assert parsed["kv"]["latitude"] == "0.031884"
    assert parsed["kv"]["rel_alt"] == "7.410"
    assert parsed["kv"]["abs_alt"] == "302.607"


def test_parse_srt_handles_html_font_tags():
    """The real fixture wraps payload in `<font size="28">...</font>`; tags must be stripped."""
    rows = parse_srt(FIXTURE)
    assert rows
    # Parser must have stripped `<font ...>` and `</font>` — if not, the
    # bracketed lookups would fail and we'd see all-None geo fields.
    assert rows[0]["_geo"]["lat"] is not None
    assert rows[0]["_geo"]["focal_len_mm"] is not None
    # Direct check on the helper: an HTML-tagged block parses identically
    # to its untagged counterpart.
    tagged = (
        '<font size="28">FrameCnt: 1, DiffTime: 33ms\n'
        "2026-04-30 10:47:32.749\n"
        "[focal_len: 28.00] [latitude: 0.031884]</font>"
    )
    untagged = (
        "FrameCnt: 1, DiffTime: 33ms\n"
        "2026-04-30 10:47:32.749\n"
        "[focal_len: 28.00] [latitude: 0.031884]"
    )
    a = _parse_block(tagged)
    b = _parse_block(untagged)
    assert a["kv"] == b["kv"]
    assert a["frame_cnt"] == b["frame_cnt"]
    assert a["iso_ts"] == b["iso_ts"]


def test_parse_srt_empty_file(tmp_path):
    empty = tmp_path / "empty.srt"
    empty.write_text("", encoding="utf-8")
    assert parse_srt(empty) == []
    # Also: whitespace-only input.
    ws = tmp_path / "ws.srt"
    ws.write_text("\n\n   \n\n", encoding="utf-8")
    assert parse_srt(ws) == []


def test_split_blocks_basic():
    text = "1\nAAA\n\n2\nBBB\n\n3\nCCC\n"
    blocks = _split_blocks(text)
    assert [b.strip() for b in blocks] == ["1\nAAA", "2\nBBB", "3\nCCC"]
    # CRLF normalisation.
    assert _split_blocks("a\r\n\r\nb") == ["a", "b"]
    # Whitespace-only input yields no blocks.
    assert _split_blocks("") == []
    assert _split_blocks("\n\n  \n\n") == []


def test_bracket_pairs_compound():
    """`[rel_alt: 7.41 abs_alt: 302.6]` must split into two top-level keys."""
    out = _bracket_pairs("[rel_alt: 7.410 abs_alt: 302.607]")
    assert out == {"rel_alt": "7.410", "abs_alt": "302.607"}
    # `[ct: 4429, tint: 3]` similarly.
    out = _bracket_pairs("[ct: 4429, tint: 3]")
    assert out == {"ct": "4429", "tint": "3"}
