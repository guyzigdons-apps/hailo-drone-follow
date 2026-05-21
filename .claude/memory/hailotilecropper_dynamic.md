---
name: hailotilecropper_dynamic
description: Community GStreamer plugin that crops tiles from dynamic ROIs + automatic grid + static rectangles. Drop-in superset of upstream hailotilecropper.
type: reference
---

# `hailotilecropper_dynamic` — community plugin

Vendored from upstream `hailotilecropper` plus extensions. Source at
`hailo_apps/postprocess/cpp/hailotilecropper_dynamic/` (this repo).

## Tile sources (currently installed plugin)

⚠️ **As of 2026-05-21 the installed `hailotilecropper_dynamic` only supports `tiles-static`.** The grid/dynamic-ROI extensions described below were never landed in the C++ source in this repo (`hailo-apps/hailo_apps/postprocess/cpp/hailotilecropper_dynamic/gsthailotilecropper_dynamic.cpp`). Verified via `gst-inspect-1.0 hailotilecropper_dynamic` → only `tiles-static` and `allow-empty`.

1. **Static (only working source today)** — `tiles-static="x,y,w,h;..."` semicolon list, normalized 0..1.
2. **Dynamic** *(intended, NOT in current build)* — `HailoTileROI` objects pre-attached upstream.
3. **Grid** *(intended, NOT in current build)* — `tiles-along-x-axis` × `tiles-along-y-axis` with `overlap-x-axis`/`overlap-y-axis`.

## Working around the missing grid props

`bench/tiling_record.py` was patched (commit `7d9a8d9`) to **restore the Python-side `_grid_to_static_tiles()` helper** that was previously deleted. `DYNAMIC_TILE_CROPPER_PIPELINE` now enumerates the requested grid into normalized rectangles and concatenates them with any user-supplied `tiles_static` extras, all passed via the single `tiles-static` property. Math used: `T = 1 / (N - (N-1) * o)`, `step = T * (1 - o)`. Diverges slightly from upstream `prepare_tiles()` clamping but is internally consistent — fine for comparing GT vs candidate runs that all use this helper.

## Properties on the installed plugin (gst-inspect)
| Property | Type | Default |
|---|---|---|
| `tiles-static` | string | `""` |
| `allow-empty` | bool | true |
| (inherited from base) `internal-offset`, `cropping-period`, `drop-uncropped-buffers`, `filter-streams` | | |

## If you ever want the native grid props back
Extending the C++ to add `tiles-along-x-axis` / `tiles-along-y-axis` / `overlap-x-axis` / `overlap-y-axis` is "drop in the property declarations + call upstream `prepare_tiles()` in `chain`". Likely 50 lines. Don't forget to rebuild via `hailo-compile-postprocess install` and clear `~/.cache/gstreamer-1.0/registry.x86_64.bin`. Be aware: loading two versions of the plugin side-by-side segfaults — uninstall the system .so first.

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
