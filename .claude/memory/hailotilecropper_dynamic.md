---
name: hailotilecropper_dynamic
description: Community GStreamer plugin that crops tiles from dynamic ROIs + automatic grid + static rectangles. Drop-in superset of upstream hailotilecropper.
type: reference
---

# `hailotilecropper_dynamic` — community plugin

Vendored from upstream `hailotilecropper` plus extensions. Source at
`hailo_apps/postprocess/cpp/hailotilecropper_dynamic/` (this repo).

## Tile sources (stack in this order every frame)
1. **Dynamic** — `HailoTileROI` objects pre-attached upstream (use `identity signal-handoffs=true` + `handoff` callback; pad probes don't work because Python sees a non-writable buffer).
2. **Grid** — `tiles-along-x-axis` × `tiles-along-y-axis` with `overlap-x-axis`/`overlap-y-axis`. Math matches upstream `prepare_tiles()` exactly. Default `0` on both axes ⇒ grid disabled.
3. **Static** — `tiles-static="x,y,w,h;..."` semicolon list, normalized 0..1.

## Properties (gst-inspect)
| Property | Type | Default |
|---|---|---|
| `tiles-along-x-axis` / `tiles-along-y-axis` | uint | 0 (disabled) |
| `overlap-x-axis` / `overlap-y-axis` | float 0..1 | 0.0 |
| `tiles-static` | string | `""` |
| `allow-empty` | bool | true |

Inherited from base: `internal-offset`, `cropping-period`, `drop-uncropped-buffers`, `filter-streams`.

## Why apps don't hand-build the grid string anymore
Earlier versions of `mafat/tiling_record.py` / `tiling_bench.py` had a Python `_grid_to_static_tiles()` helper. **Its math differed from upstream** (mine: `t = 1/(N - (N-1)*o)`; upstream: `col_step = 1/N` with overlap added on each side and clamped). Helper was deleted; pass grid params to the plugin instead via the `DYNAMIC_TILE_CROPPER_PIPELINE` helper's `tiles_x`/`tiles_y`/`overlap_x`/`overlap_y` kwargs.

## Build & install (after editing the .cpp/.hpp)
```bash
source setup_env.sh
hailo-compile-postprocess install
rm -f ~/.cache/gstreamer-1.0/registry.x86_64.bin
gst-inspect-1.0 hailotilecropper_dynamic   # verify
```
The `.so` lands at `/usr/lib/x86_64-linux-gnu/gstreamer-1.0/libgsthailotilecropper_dynamic.so` (system path, root-owned). Sudo is required for the install step.

## Pitfalls observed
- **Loading two versions side-by-side segfaults**. Don't try to `gst-launch-1.0 --gst-plugin-load=<built.so>` while the system version is also installed — `g_type_register_static` collides on first PLAYING.
- **`tests/unit/meson.build` exists but isn't wired into the parent meson tree**. The unit tests don't run with `meson test`; this is a known gap (preexisting before the dynamic-cropper work).
- **`<algorithm>` is required** in `gsthailotilecropper_dynamic.cpp` for `std::min`/`std::max` used in grid clamping.

## Pipeline composition (canonical)
```
... ! identity name=tile_setter signal-handoffs=true !
hailotilecropper_dynamic name=tc tiles-along-x-axis=3 tiles-along-y-axis=2 \
                              overlap-x-axis=0.5 overlap-y-axis=0.5 \
                              tiles-static="0.3,0.3,0.4,0.4" !
  tc.src_0 ! queue ! agg.sink_0
  tc.src_1 ! video/x-raw,format=RGB ! queue ! <inference> ! agg.sink_1
hailotileaggregator name=agg flatten-detections=true iou-threshold=0.3 !
agg.src ! hailooverlay_community ! videoconvert ! autovideosink
```

`hailooverlay_community` between aggregator and any caps-querying sink — aggregator's `src` pad doesn't advertise caps and a direct connection asserts.
