"""Time-alignment helpers for imported telemetry timelines.

Converts native-time-base telemetry rows (produced by :func:`parse_ulg` /
:func:`parse_srt`) into monotonic seconds matching the source video's PTS-0.

Strategies (per Plan 7 Task 4):
- ``"video-start"`` (default) — identity passthrough.
- ``"offset:<sec>"`` — add the given float to every ``timestamp``.
- ``"video-creation"`` — use the MP4 ``creation_time`` metadata (SRT only).

Plan 7 Task 1 ships only the stub; Task 4 lands the implementation.
See `docs/superpowers/plans/2026-05-28-telemetry-import-visualizer.md`.
"""
from __future__ import annotations

from pathlib import Path


def align_to_video(
    rows: list[dict],
    video_path: Path,
    strategy: str,
) -> list[dict]:
    """Align telemetry ``rows`` to the timebase of ``video_path``.

    ``strategy`` is one of ``"video-start"``, ``"offset:<seconds>"``,
    ``"video-creation"``.

    Stub: implemented in Plan 7 Task 4.
    """
    raise NotImplementedError(__name__)
