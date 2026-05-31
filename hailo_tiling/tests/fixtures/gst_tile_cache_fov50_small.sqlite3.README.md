# gst_tile_cache_fov50_small.sqlite3 — GST-produced tile-cache fixture

A ~28 KB SQLite tile cache produced by the **GStreamer** `gst-hailo-cache`
writer (NOT by the Python `SqliteCacheStore`), used by
`hailo_tiling.tests.test_backend_replay` to verify the pure-Python
cache → source-coord read path
(`hailo_tiling.backends.read_source_coord_detections`, Task 8 of plan
`2026-05-31-gst-cache-source-pixel-provenance.md`).

## Why a GST-produced fixture (not a Python-written one)

The point of the test is reproducibility *across* the C++ ↔ Python boundary:
the C++ writer serializes detection floats with `%.9g` while Python uses
shortest round-trip repr, so the `dets_json` **text** differs between the two
implementations even though both recover identical float32 values. Only a
real GST-written cache exercises that round-trip, so the comparison in the
test is **value-based at float32 tolerance**, never text-equality.

## Contents

- Schema: `hailo_tiling/cache/schema.sql` (`PRAGMA user_version = 1`).
- `detections`: 159 rows (per-tile, `ppv=1`), tile-local-normalized
  `dets_json` keyed by source-pixel crop `(crop_x, crop_y, crop_w, crop_h)`.
  Two rows carry real detections (frame_idx 3: 1 det; frame_idx 39: 2 dets,
  all `cls=2`); the rest are empty (`dets_json='[]'`) — `record-empty=true`.
- `meta`: the provenance envelope —
  `video_w=3840`, `video_h=2160`, `resize_mode=stretch`,
  `dst_w=640`, `dst_h=480`, `interpolation=linear`, `hef_sha=""`.
- Single file (WAL checkpointed + `journal_mode=DELETE`): no `-wal`/`-shm`.

## Provenance / reproduce

Generated on a HAILO10H (FW 5.3.0) from the first 40 source frames of
`/home/giladn/Videos/Drone/Training/DJI_20260528155741_0029_D_prepared__fov50.mp4`
(3840×2160), tiled 2×2, with:

```
hailotilecropper_dynamic tiles-static="0,0,0.5,0.5;0.5,0,0.5,0.5;0,0.5,0.5,0.5;0.5,0.5,0.5,0.5"
  ! videoconvert
  ! hailonet hef-path=.../hailo10h/hailo_yolov8n_4_classes_vga.hef batch-size=1 force-writable=true
  ! hailofilter so-path=.../libyolo_hailortpp_postprocess.so function-name=filter
  ! hailocachewriter mode=tile_cache source-width=3840 source-height=2160 record-empty=true
```

`identity eos-after=40` upstream of the cropper bounds the run so the writer
drains cleanly on EOS (`dropped-rows=0`). The frame_idx counter increments
per tile-buffer (per-tile, not per-source-frame), so ~4 rows correspond to
each source frame across distinct crop keys. The cache was then trimmed of
its WAL and copied here verbatim.
