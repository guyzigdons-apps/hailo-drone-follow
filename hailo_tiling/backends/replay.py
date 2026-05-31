"""ReplayBackend — chip-free cache reader.

For chip-free reruns over a published cache (CI, external reviewers). A
cache miss is a loud error — the spec (§7.6) is explicit: "Mixing live
inference into a replay path would muddy paper results; if the cache is
incomplete, the user re-runs warming."

This module also exposes a pure-Python *source-coord* read path
(`read_source_coord_detections` / `map_dets_to_source`) for offline
postprocessing (requirement R5) over a GST-produced cache. Python *follows*
the GST cache — it never re-resizes pixels; it only de-tiles the detections
the cropper+HEF already produced, reproducing the GST aggregator's
tile-local-normalized -> source-frame-normalized mapping.
"""
from __future__ import annotations

from typing import Any, Iterator, NamedTuple, Sequence

from ..cache.store import SqliteCacheStore
from ..types import CropRect, Det
from .backend import InferenceBackend


class CacheMissError(LookupError):
    """Raised by ReplayBackend when a requested crop is not in the cache."""


class ReplayBackend(InferenceBackend):
    def __init__(self, store: SqliteCacheStore, ppv: int = 1):
        self._store = store
        self._ppv = int(ppv)

    def infer(
        self,
        frame: Any,
        crops: Sequence[CropRect],
        frame_idx: int,
    ) -> list[list[Det]]:
        if not crops:
            return []
        results = self._store.get_many(frame_idx, list(crops), ppv=self._ppv)
        for c, dets in zip(crops, results):
            if dets is None:
                raise CacheMissError(
                    f"cache miss: frame_idx={frame_idx} "
                    f"crop=({c.x},{c.y},{c.w},{c.h}) ppv={self._ppv}. "
                    f"Re-warm the cache or use a live backend."
                )
        return results  # type: ignore[return-value]

    def close(self) -> None:
        return None


# ---------------------------------------------------------------------------
# Source-coord read path (offline postprocessing — R5)
# ---------------------------------------------------------------------------
#
# The GST writer (Task 4/6) serializes each tile's detections as
# [{cls,score,x,y,w,h}] in TILE-LOCAL NORMALIZED coords (0..1 within the
# crop), keyed by the source-pixel crop rect (crop_x,crop_y,crop_w,crop_h),
# with the `meta` table holding video_w / video_h (the source resolution).
#
# To get a detection's SOURCE-FRAME NORMALIZED coords we reproduce the
# de-tiling the GST aggregator does:
#
#   src_x = (crop_x + det.x * crop_w) / video_w
#   src_y = (crop_y + det.y * crop_h) / video_h
#   src_w =          det.w * crop_w  / video_w
#   src_h =          det.h * crop_h  / video_h
#
# NOTE on comparison: this is a *value-based* reproduction, NOT a
# byte/text-identical one. The C++ writer serializes floats with `%.9g`
# while Python (json/repr) uses shortest round-trip repr; the two differ as
# text but recover identical float32 values. Tests therefore compare with a
# float32 tolerance (math.ulp / float32 eps), never string equality.


class SourceTile(NamedTuple):
    """One cached tile, with its detections mapped to source-frame coords.

    `crop` is the source-pixel crop key the GST writer recorded (ints).
    `dets` are `Det`s whose (x,y,w,h) are SOURCE-FRAME normalized (0..1 over
    the full `video_w x video_h` frame), de-tiled from the tile-local-norm
    detections stored in `dets_json`.
    """
    frame_idx: int
    crop: CropRect
    dets: list[Det]


def map_dets_to_source(
    dets: Sequence[Det],
    crop: CropRect,
    video_w: int,
    video_h: int,
) -> list[Det]:
    """Map tile-local-normalized `dets` to source-frame-normalized `Det`s.

    `dets` are detections as stored in `dets_json` — normalized within the
    crop. `crop` is the source-pixel crop rect they belong to; `video_w` /
    `video_h` are the source resolution (from the cache `meta` envelope).
    Reproduces the GST aggregator's de-tiling (see module docstring).
    """
    if video_w <= 0 or video_h <= 0:
        raise ValueError(f"video_w/h must be positive (got {video_w}x{video_h})")
    sx = crop.w / video_w
    sy = crop.h / video_h
    ox = crop.x / video_w
    oy = crop.y / video_h
    return [
        Det(
            cls=d.cls,
            score=d.score,
            x=ox + d.x * sx,
            y=oy + d.y * sy,
            w=d.w * sx,
            h=d.h * sy,
        )
        for d in dets
    ]


def read_source_coord_detections(
    store: SqliteCacheStore,
    *,
    ppv: int = 1,
) -> Iterator[SourceTile]:
    """Yield every cached tile's detections mapped to source-frame coords.

    Reads `video_w` / `video_h` once from the cache `meta` envelope (the
    source resolution the GST writer recorded), then walks the `detections`
    table for the given `ppv`, deserializing each row's tile-local-norm
    `dets_json` (via the store) and de-tiling it into source-frame-normalized
    `Det`s. Rows are yielded ordered by (frame_idx, crop_x, crop_y).

    Raises `ValueError` if the cache lacks the `video_w` / `video_h` meta
    keys (it was not produced with `source-width`/`source-height` set).
    """
    vw = store.meta_get("video_w")
    vh = store.meta_get("video_h")
    if vw is None or vh is None:
        raise ValueError(
            f"{store.path}: cache meta is missing video_w/video_h; this cache "
            "was not produced with source-width/source-height set, so tile "
            "detections cannot be mapped to source-frame coords."
        )
    video_w, video_h = int(vw), int(vh)
    rows = store._con.execute(  # noqa: SLF001 — same-package read of the store
        "SELECT frame_idx, crop_x, crop_y, crop_w, crop_h, dets_json "
        "FROM detections WHERE ppv=? ORDER BY frame_idx, crop_x, crop_y",
        (int(ppv),),
    ).fetchall()
    from ..cache.store import _json_to_dets  # local import: avoid cycle at module load

    for frame_idx, cx, cy, cw, ch, dets_json in rows:
        crop = CropRect(x=int(cx), y=int(cy), w=int(cw), h=int(ch))
        local = _json_to_dets(dets_json)
        yield SourceTile(
            frame_idx=int(frame_idx),
            crop=crop,
            dets=map_dets_to_source(local, crop, video_w, video_h),
        )
