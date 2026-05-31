# gst-hailo-cache — GStreamer cache plugins for the Hailo tile-cache layer

This directory ships **`libgsthailocache.so`**, a single GStreamer plugin that registers three elements:

- **`hailocachewriter`** — records per-crop detections (mode `tile_cache`) or per-frame
  aggregated detections (mode `full_frame`) into a SQLite cache file. Drop-in
  passthrough; never blocks the streaming thread.
- **`hailocachereader`** — drop-in replacement for `hailonet` that serves cached
  detections instead of running inference. Research / replay tool.
- **`hailocachebypass`** — wrapper-first stand-in for `hailofilter` on the
  cache-replay path (Plan 5 Task 12). Reads the `hailo-cache-hit` qdata that
  `hailocachereader` sets on each buffer and forwards the buffer without
  invoking the postprocess `.so`. Avoids patching the `hailo-apps/`
  submodule for the Phase 14 `bypass-on-cache-hit` contract.

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

### Two recording scenarios (where to put the writer, and why)

The aggregator (`hailotileaggregator` / `hailodetiler`) runs **NMS** and drops
detections. So **where** you place `hailocachewriter` decides *what* you are
recording — and there are exactly two scenarios:

| Scenario | Mode | Placement | Records | Why this placement |
| --- | --- | --- | --- | --- |
| **Validate the CACHE** | `tile_cache` | **PRE-aggregator** — on the cropped branch, after `hailofilter`, before `agg.sink_1` | per-tile detections keyed by **source-pixel crop rect** (`frame_idx, crop_x/y/w/h, ppv, dets_json`), tile-local coords | NMS hasn't run yet, so this is the raw per-tile inference stream. This is the layer the bit-exact gate compares (live vs cached). |
| **Review the SOLUTION** | `full_frame` | **POST-aggregator** — after `hailotileaggregator`/`hailodetiler` | per-frame **aggregated** detections (`frame_results`: `frame_idx, ppv, dets_json, tiles_json, ts_epoch`), source-frame coords | This is the final post-NMS detection set — the "whole solution" a visualizer/overlay would render. |

**Canonical cache-test** is the per-tile, pre-aggregator path, driven by
[`scripts/cache_gst_replay_gate.py`](../scripts/cache_gst_replay_gate.py)
(Plan `2026-05-31-gst-cache-source-pixel-provenance` Task 6): it replays the
**same** GStreamer pipeline twice — live `hailonet` vs
`hailocachereader`+`hailocachebypass` — and diffs per-tile detections **before**
the aggregator. Use `tile_cache` here.

**Solution-review recorder** is the post-aggregator `full_frame` writer: it
records the final aggregated, post-NMS detections per frame for offline review.

**Both placements in one pipeline:**

```
... ! hailotilecropper_dynamic name=tc ... \
    tc. ! queue ! agg.sink_0 \
    tc. ! video/x-raw,format=RGB ! queue ! videoconvert ! \
      hailonet hef-path=...hef ! \
      hailofilter so-path=...so ! \
      hailocachewriter mode=tile_cache  output-file=cache.sqlite3 \
                       source-width=3840 source-height=2160 \
                       resize-mode=stretch hef-sha=<sha> ! \   # PRE-aggregator (per-tile, pre-NMS)
      queue ! agg.sink_1 \
    hailotileaggregator name=agg ... ! \
    hailocachewriter mode=full_frame output-file=flight_record.sqlite3 ! \  # POST-aggregator (post-NMS)
    hailooverlay_community ! videoconvert ! ...
```

**Source-pixel + resize envelope (this plan).** In `tile_cache` mode set
`source-width`/`source-height` to the **source video** dimensions so crop keys
land in source pixels (normalized `HailoROI` bbox × source dims, TAPPAS
truncate-then-clamp rule), and set `resize-mode` (`stretch`|`letterbox`) +
`hef-sha`. The writer stamps the envelope into the cache `meta` table once at
start: `video_w`, `video_h`, `resize_mode`, `dst_w`, `dst_h`,
`interpolation=linear`, `hef_sha`. (`dst_w`/`dst_h` default to the
cropped-branch caps — i.e. the network input dims the cropper resized to.)

**Element ordering caveats (verified):**
- `hailocachewriter` is a **passthrough** `GstBaseTransform`, so it links
  **directly** after the aggregator src pad — no `videoconvert` needed in
  front of it. (Verified: aggregator → `hailocachewriter` → `fakesink` runs to
  EOS.)
- A **caps-querying sink** after the aggregator (a real overlay/encoder/display)
  still needs `hailooverlay_community ! videoconvert` first — documented in
  `.claude/memory/hailotilecropper_dynamic.md`. `hailocachewriter` is unaffected
  because it negotiates passthrough caps.

**Full-frame post-aggregator smoke (no chip, evidence).** `videotestsrc` through
`hailotilecropper_dynamic` + `hailotileaggregator`, then
`hailocachewriter mode=full_frame` directly after the aggregator (plugin loaded
from `gst-hailo-cache/build/src` via `GST_PLUGIN_PATH`):

```bash
gst-launch-1.0 -e videotestsrc num-buffers=10 ! video/x-raw,width=1280,height=720,format=RGB ! \
  hailotilecropper_dynamic name=tc tiles-static="0,0,0.5,1.0;0.5,0,0.5,1.0" \
  hailotileaggregator name=agg \
  tc. ! queue ! agg.sink_0 \
  tc. ! queue ! agg.sink_1 \
  agg. ! hailocachewriter mode=full_frame output-file=/tmp/ff.sqlite3 ! \
  fakesink sync=false
# → EOS, exit 0

sqlite3 /tmp/ff.sqlite3 "SELECT count(*) FROM frame_results;"
# → 10        (one row per buffer)
```

10 buffers in → **10 `frame_results` rows** (`record-empty` defaults TRUE, so a
row is written per frame even when `dets_json`/`tiles_json` are `[]`). Note:
with `videotestsrc` (and at this plan's stage) the `full_frame` writer emits
`dets_json='[]'` / `tiles_json='[]'` — the aggregated-detection and tile-list
**payload** channels are wired in a later (Phase 14) task; the `frame_results`
**schema and row-per-frame recording are confirmed working** here. To get
non-empty `dets_json` you need a real detection pipeline (chip `hailonet` +
`hailofilter`/`hailodetiler` upstream of the aggregator) once those payload
channels land.

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
... ! hailotilecropper_dynamic ! hailocachereader cache-file=flight.sqlite3 hef-path=...hef ! \
    hailocachebypass ! hailocachewriter record-cache-hits=false ! ...
```

(Spec calls these `hailonet_cache` / `hailodet_record` / Phase 14 `hailofilter
bypass-on-cache-hit=true`; this repo ships them as `hailocachereader` /
`hailocachewriter` / `hailocachebypass`. The bypass element STANDS IN for
`hailofilter` in the replay pipeline — see Task 12 below.)

### Bypass — Plan 5 Task 12 (`hailocachebypass`)

Stand-in for `hailofilter` in the cache-replay pipeline. Reads the
`hailo-cache-hit` qdata that `hailocachereader` attaches to each buffer
(`GST_HAILO_CACHE_HIT_VALUE_HIT` / `_MISS`) and forwards the buffer
without invoking any postprocess `.so`. The cached detection JSON (also
attached upstream under `hailo-cached-detections` qdata) survives the
bypass untouched so downstream visualisers / writers see exactly the
state a live `hailofilter` would have produced.

| Upstream qdata | Behaviour |
| --- | --- |
| `HIT`  (1) | Forward buffer unchanged. No postprocess invoked. |
| `MISS` (2) | Forward buffer unchanged. (Reader was in `on-miss=drop`.) |
| absent     | Forward buffer unchanged + emit `GST_WARNING` once. |

This is the **option 1** (wrapper-first) resolution of spec §7.9's
Phase 14 `hailofilter bypass-on-cache-hit=true` property — avoids
patching the `hailo-apps/` submodule. See
`docs/superpowers/plans/2026-05-28-gst-cache-plugins.md` Task 12 for the
decision matrix.

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
`gst_hailocachewriter.{hpp,cpp}`, `gst_hailocachereader.{hpp,cpp}`,
`gst_hailocachebypass.{hpp,cpp}` — and the `tests/` directory land in
later tasks.)
