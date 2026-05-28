"""BoundaryStripFilter — drops detections touching a tile's interior boundary.

Python port of the C++ `remove_exceeded_bboxes(border_threshold)` documented
in tiling_benchmark/PERF_REPORT.md §8. The default threshold (0.005) is the
same value used in the production drone-follow pipeline.

Semantics:
- For each 'm'-mode tile, compute its four edges in normalized source coords.
- For each edge that is NOT also the source-frame edge, mark it as "interior".
- A detection is stripped iff any of its four bbox edges lies within
  `border_threshold` of any interior edge AND that detection is not also
  touching a frame edge on that side.
- 's'-mode tiles (ROI / recovery) are exempt — they do not contribute
  interior edges to the strip set.

A `border_threshold` of 0.0 disables stripping entirely (matches the C++
`bypass-on-zero` rule documented in §8.7).
"""
from __future__ import annotations

from typing import Sequence

from ..types import CropRect, Det


def _interior_edges_norm(tile: CropRect, src_w: int, src_h: int
                         ) -> tuple[float | None, float | None, float | None, float | None]:
    """Return (left, right, top, bottom) interior edges in normalized coords.

    An edge is None if it coincides with the source-frame edge.
    """
    left = tile.x / src_w if tile.x > 0 else None
    right = (tile.x + tile.w) / src_w if (tile.x + tile.w) < src_w else None
    top = tile.y / src_h if tile.y > 0 else None
    bottom = (tile.y + tile.h) / src_h if (tile.y + tile.h) < src_h else None
    return left, right, top, bottom


class BoundaryStripFilter:
    """Filter detections whose bbox touches a tile's interior boundary."""

    name = "boundary_strip"

    def __init__(self, border_threshold: float = 0.005):
        self.border_threshold = border_threshold

    def filter(self, dets: Sequence[Det], tiles: Sequence[CropRect],
               src_w: int, src_h: int) -> list[Det]:
        if self.border_threshold <= 0.0:
            return list(dets)

        thr = self.border_threshold
        lefts: list[float] = []
        rights: list[float] = []
        tops: list[float] = []
        bottoms: list[float] = []
        for t in tiles:
            if t.mode != "m":
                continue
            l, r, top, bot = _interior_edges_norm(t, src_w, src_h)
            if l is not None: lefts.append(l)
            if r is not None: rights.append(r)
            if top is not None: tops.append(top)
            if bot is not None: bottoms.append(bot)

        out: list[Det] = []
        for d in dets:
            dx1, dy1, dx2, dy2 = d.xyxy
            touches_left_frame = dx1 <= thr
            touches_right_frame = dx2 >= 1.0 - thr
            touches_top_frame = dy1 <= thr
            touches_bottom_frame = dy2 >= 1.0 - thr

            stripped = False
            if not touches_left_frame:
                if any(abs(dx1 - r) < thr for r in rights):
                    stripped = True
                elif any(abs(dx1 - l) < thr for l in lefts):
                    stripped = True
            if not stripped and not touches_right_frame:
                if any(abs(dx2 - l) < thr for l in lefts):
                    stripped = True
                elif any(abs(dx2 - r) < thr for r in rights):
                    stripped = True
            if not stripped and not touches_top_frame:
                if any(abs(dy1 - b) < thr for b in bottoms):
                    stripped = True
                elif any(abs(dy1 - t) < thr for t in tops):
                    stripped = True
            if not stripped and not touches_bottom_frame:
                if any(abs(dy2 - t) < thr for t in tops):
                    stripped = True
                elif any(abs(dy2 - b) < thr for b in bottoms):
                    stripped = True

            if not stripped:
                out.append(d)
        return out
