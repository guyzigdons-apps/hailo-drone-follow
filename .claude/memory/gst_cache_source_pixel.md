---
name: gst_cache_source_pixel
description: gst-hailo-cache provenance model — source-pixel crop keys, resize contract, and the per-tile live-vs-cached bit-exact gate. Read before touching the cache writer/reader, the cropper resize path, or the replay gate.
type: project
---

# gst-hailo-cache — source-pixel provenance, resize contract, bit-exact gate

Durable facts from Plan `docs/superpowers/plans/2026-05-31-gst-cache-source-pixel-provenance.md`
(branch `tiling-benchmark`).

## Provenance model
- The cache is **provenance-only**: it never decides tiling, it records whatever
  the `hailotilecropper_dynamic` cropper actually produced (the cropper is the
  golden standard).
- Crop keys are stored in **source-video pixels** — normalized tile ROI bbox ×
  source dims. Source dims come from the writer/reader `source-width` /
  `source-height` properties, read once from the input video (not the cropped
  caps space). Conversion uses TAPPAS's truncate-then-clamp rule, shared by
  writer and reader.

## Resize contract
- Tile resize is **OpenCV CPU** in the cropper. The DSP crop+resize path is
  `#ifdef HAILO15_TARGET`-only — **NOT** on Hailo-8/Hailo-10/x86.
- Cropper has a `resize-mode=stretch|letterbox` property. **`stretch` is the
  default** and the only fully validated mode.
- **`letterbox` has a back-mapping offset bug** (systematic positional offset on
  map-back; Task 7, see
  `docs/superpowers/research/2026-05-31-resize-envelope-vs-stretch.md`) — not
  production-ready.
- The resize envelope is recorded in the cache `meta` table:
  `video_w`, `video_h`, `resize_mode`, `dst_w`, `dst_h`, `interpolation`,
  `hef_sha`. (VGA HEF is 640×480, so `dst_h="480"`.)

## Bit-exact gate
- The gate is **per-tile, pre-aggregator, GST live vs GST cached** — same
  GStreamer pipeline run twice (live `hailonet` vs `hailocachereader` +
  `hailocachebypass`), diffed before the aggregator's NMS.
- Helper: `scripts/cache_gst_replay_gate.py`; test:
  `tests/integration/test_cache_gst_replay_gate.py` (`HAILO_CHIP=1`-gated).
  Passes with 0 deviations.
- **Cross-engine (Python) equality is value-lossless at float32, NOT
  text-identical** — C++ serializes with `%.9g`, Python uses shortest-repr; the
  JSON text differs but round-trips to identical float32 values.

## Detection serialization
- `tile_cache` mode serializes real per-tile detections:
  `[{cls,score,x,y,w,h}]`, tile-local normalized coords; field names shared via
  `gst-hailo-cache/src/cache_keys.hpp` constants.
- **`full_frame` (post-aggregator) `dets_json`/`tiles_json` payload is still
  TODO** (Phase-14 follow-up) — emits `"[]"` for now.

## Recorder placement
- `tile_cache` writer → **PRE-aggregator** (per-tile, for cache testing /
  bit-exact gate).
- `full_frame` writer → **POST-aggregator** (post-NMS, for solution review).
- See `gst-hailo-cache/README.md` → "Two recording scenarios".
