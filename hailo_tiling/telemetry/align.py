"""Time-alignment helpers for imported telemetry timelines.

Converts native-time-base telemetry rows (produced by :func:`parse_ulg` /
:func:`parse_srt`) into monotonic seconds matching the source video's PTS-0.

Strategies (per Plan 7 Task 4):

- ``"video-start"`` (default) -- identity passthrough. The implicit
  assumption is "telemetry row 0 corresponds to video PTS 0"; logged at
  ``INFO``.
- ``"offset:<sec>"`` -- add the given float (positive or negative) to
  every row's ``timestamp``. Rows whose shifted ``timestamp`` falls outside
  ``[0, video_duration]`` are **kept** (callers can pre-clip if they want).
  ``RecordedTelemetry.snapshot()`` already handles out-of-range queries.
- ``"video-creation"`` -- read the video's ``creation_time`` MP4 metadata
  via ``ffprobe -show_format -of json``, parse as ISO, and compute the
  offset to align the SRT's first-block ISO timestamp to video PTS-0.
  **Only valid for SRT input** -- raises ``ValueError`` for ULG rows.

See `docs/superpowers/plans/2026-05-28-telemetry-import-visualizer.md`.
"""
from __future__ import annotations

import copy
import json
import logging
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Optional

# Lazy import to avoid creating a hard cycle between align.py and srt.py.
# Importing at module top-level is fine here -- srt.py has no align import.
from hailo_tiling.telemetry.srt import _parse_iso, parse_srt


log = logging.getLogger(__name__)

# Valid strategy identifiers. ``offset:<sec>`` is parametric, so the
# membership test handles it via prefix matching below.
_VALID_STRATEGIES = ("video-start", "offset:<seconds>", "video-creation")

# Marker that the SRT parser writes onto every row's ``_geo`` sidecar.
# We use it to distinguish SRT-sourced rows from ULG-sourced ones (the
# ULG parser does NOT set ``_agl_source``).
_SRT_GEO_MARKER_KEY = "_agl_source"
_SRT_GEO_MARKER_VALUE = "rel_alt"


def _is_srt_row(row: dict) -> bool:
    """Return True if ``row`` looks like it came from :func:`parse_srt`.

    The SRT parser writes ``_geo['_agl_source'] = 'rel_alt'`` on every row;
    the ULG parser does not set that key. This is the cheapest marker we
    have without modifying either parser (which the plan forbids).
    """
    geo = row.get("_geo")
    if not isinstance(geo, dict):
        return False
    return geo.get(_SRT_GEO_MARKER_KEY) == _SRT_GEO_MARKER_VALUE


def _ffprobe_creation_time(video_path: Path) -> datetime:
    """Run ``ffprobe`` on ``video_path`` and return parsed ``creation_time``.

    Invocation: ``ffprobe -v error -show_format -of json <video>``.
    The JSON has ``format.tags.creation_time`` as an ISO 8601 string
    (often UTC, with ``Z`` suffix).

    Raises ``ValueError`` if the metadata is absent or unparsable, and
    propagates ``subprocess.CalledProcessError`` / ``FileNotFoundError`` /
    ``TimeoutExpired`` if ffprobe itself fails.
    """
    result = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-show_format",
            "-of",
            "json",
            str(video_path),
        ],
        capture_output=True,
        text=True,
        check=True,
        timeout=10,
    )
    try:
        payload = json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"ffprobe returned non-JSON output for {video_path}: {exc}"
        ) from exc

    fmt = payload.get("format")
    if not isinstance(fmt, dict):
        raise ValueError(
            f"ffprobe JSON for {video_path} has no 'format' object"
        )
    tags = fmt.get("tags")
    if not isinstance(tags, dict):
        raise ValueError(
            f"ffprobe JSON for {video_path} has no format.tags"
        )
    creation_time = tags.get("creation_time")
    if not creation_time:
        raise ValueError("video has no creation_time metadata")

    # ffprobe emits e.g. "2026-04-30T10:47:30.000000Z". datetime.fromisoformat
    # before Python 3.11 doesn't accept the trailing 'Z'; strip it.
    iso = str(creation_time)
    if iso.endswith("Z"):
        iso = iso[:-1] + "+00:00"
    parsed = _parse_iso(iso)
    if parsed is None:
        raise ValueError(
            f"could not parse creation_time {creation_time!r} as ISO"
        )
    return parsed


def _find_sibling_srt(video_path: Path) -> Optional[Path]:
    """Locate an SRT sidecar next to ``video_path``.

    DJI's convention pairs ``DJI_0001.MP4`` with ``DJI_0001.SRT``. We
    look for the case-sensitive match first, then a lowercase variant.
    Returns ``None`` if neither exists.
    """
    for suffix in (".srt", ".SRT"):
        candidate = video_path.with_suffix(suffix)
        if candidate.exists():
            return candidate
    return None


def _srt_first_iso(rows: list[dict], video_path: Path) -> datetime:
    """Recover the SRT first-block ISO timestamp.

    The SRT parser does not expose absolute ISO timestamps on each row
    (only relative seconds since row 0), so to align to a wall-clock
    video ``creation_time`` we need the absolute time of row 0.

    Strategy: re-parse the SRT sidecar by sibling-lookup next to
    ``video_path``. This is "option (a)" from the Plan 7 Task 4 brief
    -- wasteful (we parse the file twice) but keeps the SRT parser
    untouched as the plan requires. A future enhancement could add an
    ``_iso_ts`` marker on each row in srt.py to skip this step.
    """
    srt_path = _find_sibling_srt(video_path)
    if srt_path is None:
        raise ValueError(
            "video-creation strategy needs an SRT sidecar next to "
            f"{video_path} (looked for .srt/.SRT)"
        )
    # Re-parse to recover the first block's absolute ISO. We don't use
    # the returned rows -- only the side effect of reading the file.
    # parse_srt() consumes the SRT but only emits relative timestamps;
    # we need the raw first ISO, so dip into the block parser instead.
    from hailo_tiling.telemetry.srt import (  # local import: avoid cycle
        _parse_block,
        _split_blocks,
    )
    text = srt_path.read_text(encoding="utf-8-sig", errors="replace")
    for raw in _split_blocks(text):
        block = _parse_block(raw)
        if block["iso_ts"] is not None:
            return block["iso_ts"]
    raise ValueError(
        f"SRT sidecar {srt_path} has no parseable ISO timestamp"
    )


def _shift(rows: list[dict], offset: float) -> list[dict]:
    """Return a deep-copied row list with ``offset`` added to each timestamp.

    Deep-copies so the caller's input list isn't mutated; the ``_geo``
    sub-dict in particular is shared by reference in the SRT parser
    output, and we don't want a stray shift to corrupt it.
    """
    shifted: list[dict] = []
    for row in rows:
        new_row = copy.deepcopy(row)
        ts = new_row.get("timestamp")
        if isinstance(ts, (int, float)):
            new_row["timestamp"] = float(ts) + offset
        shifted.append(new_row)
    return shifted


def _parse_offset_strategy(strategy: str) -> float:
    """Extract the float ``<seconds>`` from an ``offset:<sec>`` strategy."""
    _, _, tail = strategy.partition(":")
    if not tail:
        raise ValueError(
            f"offset strategy needs a value, got {strategy!r}. "
            f"Valid strategies: {_VALID_STRATEGIES}"
        )
    try:
        return float(tail)
    except ValueError as exc:
        raise ValueError(
            f"offset strategy value {tail!r} is not a float "
            f"(strategy={strategy!r})"
        ) from exc


def align_to_video(
    rows: list[dict],
    video_path: Path,
    strategy: str,
) -> list[dict]:
    """Align telemetry ``rows`` to the timebase of ``video_path``.

    ``strategy`` is one of:

    - ``"video-start"`` -- identity (returns ``rows`` unchanged).
    - ``"offset:<sec>"`` -- add ``<sec>`` to every row's ``timestamp``.
    - ``"video-creation"`` -- read ``creation_time`` from ``video_path``
      and compute the offset to align the SRT's first-block ISO time
      to video PTS-0. Only valid for SRT-sourced rows.

    Returns a (possibly new) list of row dicts. The input list is never
    mutated. Raises ``ValueError`` for unknown strategies or for
    ``"video-creation"`` against non-SRT input.
    """
    if strategy == "video-start":
        log.info(
            "align_to_video: strategy=video-start (assumes telemetry "
            "row 0 corresponds to video PTS 0; pass --align "
            "offset:<sec> or video-creation to override)"
        )
        return rows

    if strategy.startswith("offset:"):
        offset = _parse_offset_strategy(strategy)
        log.info(
            "align_to_video: strategy=offset shift=%+.3fs (%d rows)",
            offset,
            len(rows),
        )
        return _shift(rows, offset)

    if strategy == "video-creation":
        # Only meaningful for SRT input -- ULG rows are already
        # relativised and carry no absolute wall-clock anchor.
        if not rows or not _is_srt_row(rows[0]):
            raise ValueError(
                "video-creation strategy requires SRT rows with "
                "absolute timestamps"
            )
        first_iso = _srt_first_iso(rows, Path(video_path))
        creation = _ffprobe_creation_time(Path(video_path))
        # Make timezone handling tolerant: if exactly one of the two has
        # a tzinfo we drop it for the subtraction (we're aligning two
        # timestamps that both nominally describe wall-clock UTC anyway).
        if (first_iso.tzinfo is None) != (creation.tzinfo is None):
            first_iso = first_iso.replace(tzinfo=None)
            creation = creation.replace(tzinfo=None)
        offset = (first_iso - creation).total_seconds()
        log.info(
            "align_to_video: strategy=video-creation "
            "srt_first_iso=%s video_creation=%s shift=%+.3fs (%d rows)",
            first_iso.isoformat(),
            creation.isoformat(),
            offset,
            len(rows),
        )
        return _shift(rows, offset)

    raise ValueError(
        f"unknown strategy {strategy!r}; valid: {_VALID_STRATEGIES}"
    )
