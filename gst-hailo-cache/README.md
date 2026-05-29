# gst-hailo-cache — GStreamer cache plugins for the Hailo tile-cache layer

This directory ships **`libgsthailocache.so`**, a single GStreamer plugin that will eventually register two elements:

- **`hailocachewriter`** — records per-crop detections (mode `tile_cache`) or per-frame
  aggregated detections (mode `full_frame`) into a SQLite cache file. Drop-in
  passthrough; never blocks the streaming thread.
- **`hailocachereader`** — drop-in replacement for `hailonet` that serves cached
  detections instead of running inference. Research / replay tool.

The plugin lives at the repo root in `gst-hailo-cache/` (NOT in the `hailo-apps/`
submodule). It uses the same meson + ninja build pattern as
`hailo-apps/hailo_apps/postprocess/meson.build`, and installs `libgsthailocache.so`
into the system GStreamer plugin directory resolved via
`pkg-config --variable=pluginsdir gstreamer-1.0`.

> **Naming.** The design spec
> (`docs/superpowers/specs/2026-05-28-tiling-library-design.md`) calls these
> elements `hailodet_record` and `hailonet_cache`. To make it clear these are
> the in-repo plugins (not vendored from `hailo-apps`), they are renamed
> `hailocachewriter` and `hailocachereader`. Functionality, properties, and
> on-disk format are identical to the spec; only the element class names
> differ.

## Status

**Task 1 of Plan 5** — scaffold only. The plugin currently registers an empty
description (no elements yet). Elements land in later tasks:

- Task 4 — `hailocachewriter` element skeleton
- Task 5 — writer thread + `tile_cache` mode
- Task 6 — `full_frame` mode + `frame_results` schema
- Task 8 — `hailocachereader` element skeleton
- Task 9 — `hailocachereader` lookup + cache-hit semantics

See `docs/superpowers/plans/2026-05-28-gst-cache-plugins.md` for the full task
list.

## Build & install

Prerequisites (Ubuntu / Debian):

```bash
sudo apt install -y meson ninja-build libsqlite3-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-tools
```

Then:

```bash
bash gst-hailo-cache/install.sh
```

This runs `meson setup build`, `ninja -C build`, `sudo ninja -C build install`,
and clears the user's GStreamer registry cache
(`~/.cache/gstreamer-1.0/registry.*.bin`) so the new plugin is picked up on
next inspect.

Verify with:

```bash
gst-inspect-1.0 hailocache
```

(at Task 1, only the plugin description is shown; element classes appear in
later tasks).

## Build options

`meson_options.txt`:

- `with_bench=true` (default) — build the latency microbenches under
  `tests/`. Set to `false` to skip them.

## Canonical pipeline placement

### Writer — spec §7.8 (`hailodet_record`)

The writer is intended to be instantiated **twice in a single pipeline**:

```
... ! hailotilecropper_dynamic ! hailonet ! hailofilter ! \
    hailodet_record mode=tile_cache  output-file=cache.sqlite3        ! \
    hailodetiler ! \
    hailodet_record mode=full_frame  output-file=flight_record.sqlite3 ! ...
```

(Read `hailodet_record` as `hailocachewriter` in this repo.)

**Mode `tile_cache` (after `hailofilter`, before `hailodetiler`):**
Records **per-crop, tile-local-coords** detections in the Section 7.2 schema.
This is the file consumed by `hailonet_cache` for replay; on a cache hit, both
inference and postprocess are skipped. Data shape per row: `(frame_idx,
crop_x, crop_y, crop_w, crop_h, ppv, dets_json)`.

**Mode `full_frame` (after `hailodetiler`):**
Records **per-frame, source-frame-coord** aggregated detections plus the tile
layout that was used. This is the file consumed by the visualizer and the
overlay renderer.

### Reader — spec §7.9 (`hailonet_cache`)

A drop-in replacement for `hailonet` with identical sink/source caps and the
same property surface. Pipeline string change is `s/hailonet/hailonet_cache/`.

```
... ! hailotilecropper_dynamic ! hailonet_cache cache-file=flight.sqlite3 hef-path=...hef ! \
    hailofilter bypass-on-cache-hit=true ! hailodet_record record-cache-hits=false ! ...
```

(Read `hailonet_cache` as `hailocachereader` and `hailodet_record` as
`hailocachewriter` in this repo.)

**Cache-hit semantics (critical):**
1. Look up `(frame_idx, crop_rect, hef_sha)` in the cache.
2. **On hit:**
   - Attach the cached detection objects directly to the buffer's `HailoROI`
     (matching the format `hailofilter` would have produced)
   - Set buffer meta `hailo-cache-hit=true`
   - **Emit no raw tensors.** No `HailoTensor` meta is attached to the buffer;
     the network output payload does not exist on a cache hit.
   - Push the buffer downstream.
3. **On miss:** behaviour controlled by `on-miss` property. Strict-by-default;
   this is research-only, so misses are loud errors.

## Layout

```
gst-hailo-cache/
  README.md                 # this file
  meson.build               # top-level meson, mirrors hailo-apps/hailo_apps/postprocess/meson.build
  meson_options.txt         # with_bench (default true)
  install.sh                # meson setup + ninja install + registry flush
  src/
    meson.build             # builds libgsthailocache.so → gst_plugins_dir
    plugin.cpp              # GST_PLUGIN_DEFINE — element classes register here in later tasks
```

(Additional sources — `tile_cache_db.{hpp,cpp}`, `cache_keys.{hpp,cpp}`,
`gst_hailocachewriter.{hpp,cpp}`, `gst_hailocachereader.{hpp,cpp}` — and the
`tests/` directory land in later tasks.)
