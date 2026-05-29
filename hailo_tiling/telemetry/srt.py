"""DJI SRT sidecar -> JSONL row adapter.

Reads a DJI-style SRT sidecar file (one block per video frame, bracketed
key-value payload) and emits a list of dicts shaped like a
`RecordedTelemetry` JSONL timeline.

The DJI SRT dialect this parser targets has one block per video frame:

    1
    00:00:00,000 --> 00:00:00,033
    <font size="28">FrameCnt: 1, DiffTime: 33ms
    2026-04-30 10:47:32.749
    [iso: 100] [shutter: 1/2500.0] [fnum: 2.8] [ev: 0] [color_md: default]
    [focal_len: 28.00] [latitude: 31.883741] [longitude: 35.026936]
    [rel_alt: 7.410 abs_alt: 302.607] [ct: 4429, tint: 3] </font>

Notes on dialect:

- HTML font wrappers (``<font size="28">...</font>``) appear around the
  payload; the parser strips any ``<...>`` tag transparently.
- Some bracketed groups contain multiple key:value pairs sharing one
  pair of brackets, e.g. ``[rel_alt: 7.410 abs_alt: 302.607]``. The
  parser splits these so both ``rel_alt`` and ``abs_alt`` show up as
  top-level keys.
- SRT carries no inertial state. ``velocity_world``, ``attitude_quat``,
  and ``yaw_rate_rad_s`` are always ``None`` in the emitted rows.

See `docs/superpowers/plans/2026-05-28-telemetry-import-visualizer.md`
Task 3 for the field-mapping contract.
"""
from __future__ import annotations

import re
from datetime import datetime, timedelta
from pathlib import Path
from typing import Optional


# Matches a single bracketed `[key: value]` pair. The value extends up to
# the next `]`. Compound brackets such as `[rel_alt: 7.41 abs_alt: 302.6]`
# are handled by post-processing the captured value (see `_bracket_pairs`).
_BRACKET_RE = re.compile(r"\[(\w+):\s*([^\]]+)\]")

# Matches an HTML-style tag, e.g. `<font size="28">` or `</font>`.
_HTML_TAG_RE = re.compile(r"<[^>]+>")

# Matches an embedded `key: value` pair inside a bracket's value, used to
# split compound brackets. Value is everything up to the next ` key:` or
# end-of-string.
_INNER_KV_RE = re.compile(r"(\w+):\s*([^\s,]+)")

# DJI's ISO timestamp inside the block looks like `2026-04-30 10:47:32.749`.
# `datetime.fromisoformat` accepts this in Python 3.11+ (space separator,
# fractional seconds). For 3.10 we replace the space with `T` first.
_ISO_TS_RE = re.compile(
    r"\b(\d{4}-\d{2}-\d{2}[T ]\d{2}:\d{2}:\d{2}(?:\.\d+)?)\b"
)

# SRT timing line: `HH:MM:SS,mmm --> HH:MM:SS,mmm`.
_SRT_TIMING_RE = re.compile(
    r"(\d{2}:\d{2}:\d{2}),(\d{3})\s*-->\s*(\d{2}:\d{2}:\d{2}),(\d{3})"
)

# FrameCnt: 123
_FRAME_CNT_RE = re.compile(r"FrameCnt:\s*(\d+)", re.IGNORECASE)


def _split_blocks(text: str) -> list[str]:
    """Split SRT text into per-frame blocks on blank lines.

    Normalises line endings to ``\n`` first, then splits on one or more
    blank lines (lines containing only whitespace are treated as blank).
    Empty blocks are dropped.
    """
    if not text:
        return []
    normalised = text.replace("\r\n", "\n").replace("\r", "\n")
    parts = re.split(r"\n\s*\n", normalised)
    return [p for p in parts if p.strip()]


def _bracket_pairs(payload: str) -> dict[str, str]:
    """Extract `[key: value]` pairs from a payload string.

    Handles compound brackets by re-scanning the captured value for
    additional `key: value` pairs:

        >>> _bracket_pairs("[rel_alt: 7.41 abs_alt: 302.6] [foo: bar]")
        {'rel_alt': '7.41', 'abs_alt': '302.6', 'foo': 'bar'}

        >>> _bracket_pairs("[ct: 4429, tint: 3]")
        {'ct': '4429', 'tint': '3'}

    Values are returned as raw strings; the caller is responsible for
    type-converting (e.g. ``float(...)``).
    """
    result: dict[str, str] = {}
    for m in _BRACKET_RE.finditer(payload):
        key = m.group(1)
        value = m.group(2).strip()
        # Look for `name:` markers embedded in `value` that indicate a
        # compound bracket like `[rel_alt: 7.41 abs_alt: 302.6]` or
        # `[ct: 4429, tint: 3]`.
        inner_markers = list(_INNER_KV_RE.finditer(value))
        if not inner_markers:
            # Simple `[key: value]`; strip trailing punctuation.
            result[key] = value.rstrip(", ").strip()
            continue
        # The outer `key` owns everything up to the first embedded
        # `name:` marker. Subsequent inner pairs each get their own
        # key; their value runs to the next inner pair or end of value.
        outer_end = inner_markers[0].start()
        outer_value = value[:outer_end].rstrip(", ").strip()
        result[key] = outer_value
        for j, inner in enumerate(inner_markers):
            if j + 1 < len(inner_markers):
                end = inner_markers[j + 1].start()
            else:
                end = len(value)
            iv = value[inner.start(2):end].rstrip(", ").strip()
            result[inner.group(1)] = iv
    return result


def _parse_iso(ts: str) -> Optional[datetime]:
    """Parse an ISO-ish timestamp; return None on failure.

    Accepts both `2026-04-30 10:47:32.749` and `2026-04-30T10:47:32.749`.
    """
    if not ts:
        return None
    try:
        return datetime.fromisoformat(ts)
    except ValueError:
        # Python <3.11 doesn't accept the space separator in fromisoformat;
        # try replacing it with `T` once.
        try:
            return datetime.fromisoformat(ts.replace(" ", "T"))
        except ValueError:
            return None


def _parse_srt_timing(line: str) -> Optional[timedelta]:
    """Parse an SRT `HH:MM:SS,mmm --> ...` line; return start as timedelta.

    Returns None if the line doesn't match. Used as a fallback when the
    block's ISO timestamp can't be parsed.
    """
    m = _SRT_TIMING_RE.search(line)
    if m is None:
        return None
    hh, mm, ss = (int(x) for x in m.group(1).split(":"))
    ms = int(m.group(2))
    return timedelta(
        hours=hh, minutes=mm, seconds=ss, milliseconds=ms
    )


def _parse_block(block: str) -> dict:
    """Extract `frame_cnt`, ISO timestamp, SRT-start, and kv-pairs.

    Returns a dict with these (any may be absent):

    - ``frame_cnt``: int (from `FrameCnt: <n>`)
    - ``iso_ts``: datetime (from the standalone ISO line) or None
    - ``srt_start``: timedelta (from the timing line) or None
    - ``kv``: dict[str, str] of bracketed key-value pairs

    HTML tags (`<font ...>`, `</font>`, etc.) are stripped before parsing
    so they don't interfere with the bracket / ISO matches.
    """
    cleaned = _HTML_TAG_RE.sub("", block)

    frame_cnt: Optional[int] = None
    fm = _FRAME_CNT_RE.search(cleaned)
    if fm is not None:
        try:
            frame_cnt = int(fm.group(1))
        except ValueError:
            frame_cnt = None

    iso_ts: Optional[datetime] = None
    im = _ISO_TS_RE.search(cleaned)
    if im is not None:
        iso_ts = _parse_iso(im.group(1))

    srt_start: Optional[timedelta] = _parse_srt_timing(cleaned)

    kv = _bracket_pairs(cleaned)

    return {
        "frame_cnt": frame_cnt,
        "iso_ts": iso_ts,
        "srt_start": srt_start,
        "kv": kv,
    }


def _to_float(s: Optional[str]) -> Optional[float]:
    """Best-effort float conversion; return None on failure or empty."""
    if s is None:
        return None
    s = s.strip()
    if not s:
        return None
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def parse_srt(path: Path) -> list[dict]:
    """Parse a DJI SRT sidecar file into a list of telemetry row dicts.

    Each row has keys: ``timestamp``, ``altitude_agl_m``, ``yaw_rate_rad_s``,
    ``velocity_world``, ``attitude_quat``, plus a ``_geo`` sidecar. SRT
    carries no inertial data, so ``velocity_world``, ``attitude_quat``,
    and ``yaw_rate_rad_s`` are always ``None``.

    Time base: monotonic seconds, with the first block's ISO timestamp
    mapped to ``timestamp=0.0``. If a block's ISO timestamp can't be
    parsed, the SRT timing line's start-time is used as a fallback.

    Geo fields are extracted into ``_geo`` (``lat``, ``lon``, ``alt_msl``,
    ``focal_len_mm``, plus ``pitch`` / ``roll`` set to ``None`` to match
    the ULG parser shape). The string ``"rel_alt"`` is written to
    ``_geo['_agl_source']`` so the visualizer can disclaim that the
    altitude is above takeoff, not strict AGL.

    Returns ``[]`` for empty input. Never raises on malformed blocks —
    they're silently skipped.
    """
    path = Path(path)
    text = path.read_text(encoding="utf-8-sig", errors="replace")

    blocks = _split_blocks(text)
    if not blocks:
        return []

    parsed_blocks = []
    for raw in blocks:
        b = _parse_block(raw)
        # A block needs *some* time signal to be useful. If both iso_ts
        # and srt_start are absent, skip silently.
        if b["iso_ts"] is None and b["srt_start"] is None:
            continue
        parsed_blocks.append(b)

    if not parsed_blocks:
        return []

    # Choose the time origin from the first block. Prefer ISO timestamp;
    # fall back to SRT timing.
    origin_iso = parsed_blocks[0]["iso_ts"]
    origin_srt = parsed_blocks[0]["srt_start"]

    rows: list[dict] = []
    for b in parsed_blocks:
        kv = b["kv"]

        # Compute monotonic seconds from origin. Prefer the per-block
        # ISO timestamp; fall back to SRT timing.
        timestamp: Optional[float] = None
        if b["iso_ts"] is not None and origin_iso is not None:
            timestamp = (b["iso_ts"] - origin_iso).total_seconds()
        elif b["srt_start"] is not None and origin_srt is not None:
            timestamp = (b["srt_start"] - origin_srt).total_seconds()
        elif b["iso_ts"] is not None and origin_srt is not None:
            # Origin came from SRT but this block has only ISO — anchor
            # by zero (treat as origin too).
            timestamp = 0.0
        elif b["srt_start"] is not None and origin_iso is not None:
            timestamp = 0.0
        if timestamp is None:
            continue

        # _geo sidecar.
        lat = _to_float(kv.get("latitude"))
        lon = _to_float(kv.get("longitude"))
        alt_msl = _to_float(kv.get("abs_alt"))
        focal_len_mm = _to_float(kv.get("focal_len"))
        rel_alt = _to_float(kv.get("rel_alt"))

        row = {
            "timestamp": timestamp,
            # Spec §8.8 marks rel_alt as the closest AGL signal SRT
            # provides ("height above takeoff"). The visualizer reads
            # `_geo['_agl_source']` to disclaim.
            "altitude_agl_m": rel_alt,
            "yaw_rate_rad_s": None,
            "velocity_world": None,
            "attitude_quat": None,
            "_geo": {
                "lat": lat,
                "lon": lon,
                "alt_msl": alt_msl,
                "pitch": None,
                "roll": None,
                "focal_len_mm": focal_len_mm,
                "_agl_source": "rel_alt",
            },
        }
        rows.append(row)

    rows.sort(key=lambda r: r["timestamp"])
    return rows
