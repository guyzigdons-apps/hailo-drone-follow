"""Unit tests for the Plan 7 Task 7 ASS overlay generator (pure-Python)."""
from __future__ import annotations

from hailo_tiling.cli.visualize import (
    _format_overlay_text,
    build_ass,
)


def _dialogue_lines(ass: str) -> list[str]:
    return [ln for ln in ass.splitlines() if ln.startswith("Dialogue:")]


def _dialogue_text(line: str) -> str:
    # Dialogue: Layer,Start,End,Style,Name,MarginL,MarginR,MarginV,Effect,Text
    # Text may contain commas, so split with maxsplit=9.
    parts = line.split(",", 9)
    assert len(parts) == 10, f"unexpected dialogue line: {line!r}"
    return parts[9]


def test_ass_header_well_formed():
    ass = build_ass(rows=[], fps=30.0, duration_s=0.0)
    assert ass.startswith("[Script Info]")
    assert "[V4+ Styles]" in ass
    assert "[Events]" in ass
    # Single Default style, Consolas font, 24pt, BottomLeft anchor (Alignment=1).
    style_line_present = any(
        line.startswith("Style: Default,") and "Consolas" in line and ",24," in line
        for line in ass.splitlines()
    )
    assert style_line_present, "expected Default Consolas 24pt style line"


def test_ass_one_dialogue_per_frame():
    ass = build_ass(rows=[], fps=30.0, duration_s=2.0)
    dialogues = _dialogue_lines(ass)
    assert len(dialogues) == 60


def test_ass_cue_uses_latest_le_telemetry_row():
    rows = [
        {"timestamp": 0.0, "altitude_agl_m": 1.0},
        {"timestamp": 1.0, "altitude_agl_m": 9.0},
    ]
    ass = build_ass(rows=rows, fps=30.0, duration_s=2.0)
    dialogues = _dialogue_lines(ass)
    # Frame 45 = 1.5 s, must reflect the row at t=1.0 (alt=9.0).
    text_45 = _dialogue_text(dialogues[45])
    assert "ALT    9.0 m" in text_45
    # Frame 10 = 0.333 s -> still the row at t=0.0 (alt=1.0).
    text_10 = _dialogue_text(dialogues[10])
    assert "ALT    1.0 m" in text_10


def test_ass_renders_missing_fields_as_dashes():
    rows = [{"timestamp": 0.0, "altitude_agl_m": None}]
    ass = build_ass(rows=rows, fps=30.0, duration_s=1.0)
    dialogues = _dialogue_lines(ass)
    text_0 = _dialogue_text(dialogues[0])
    # Width-5 dashes for altitude.
    assert "ALT  ----- m" in text_0


def test_ass_format_overlay_text_is_fixed_width():
    sample_rows = [
        {},
        {"timestamp": 0.0, "altitude_agl_m": 12.3,
         "velocity_world": [3.0, 4.0, 0.0], "yaw_rate_rad_s": 0.12,
         "_geo": {"lat": 0.000123, "lon": 0.000456}, "_frame_idx": 1},
        {"timestamp": 1.0, "altitude_agl_m": -7.5,
         "yaw_rate_rad_s": -1.23, "_frame_idx": 9999},
        {"timestamp": 2.0, "altitude_agl_m": 0.0,
         "velocity_world": [0.0, 0.0, 0.0], "yaw_rate_rad_s": 0.0,
         "_geo": {"lat": -12.345678, "lon": 123.456789}, "_frame_idx": 0},
    ]
    widths = {len(_format_overlay_text(r)) for r in sample_rows}
    assert len(widths) == 1, f"non-uniform overlay widths: {widths}"


def test_ass_escapes_ass_specials():
    rows = [{
        "timestamp": 0.0,
        "_geo": {"lat": "{evil}", "lon": "back\\slash"},
    }]
    ass = build_ass(rows=rows, fps=30.0, duration_s=1.0 / 30.0)
    dialogues = _dialogue_lines(ass)
    text = _dialogue_text(dialogues[0])
    # The escape helper transforms `{` -> `\{`, `}` -> `\}`, `\` -> `\\`.
    # After escaping, the literal `{` or `}` must not appear unescaped.
    # Find each brace and confirm it is preceded by a backslash.
    for i, ch in enumerate(text):
        if ch in "{}":
            assert i > 0 and text[i - 1] == "\\", (
                f"unescaped brace at index {i} in: {text!r}"
            )
    # And the raw `back\slash` substring must have been doubled.
    assert "back\\\\slash" in text
