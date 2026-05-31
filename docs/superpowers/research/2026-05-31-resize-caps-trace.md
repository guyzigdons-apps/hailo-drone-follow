# Caps trace: tile resize happens exactly once (OpenCV cropper path)

**Date:** 2026-05-31
**Task:** Plan `2026-05-31-gst-cache-source-pixel-provenance.md` — Task 1
**Chip:** HAILO10H, FW 5.3.0 (`hailortcli fw-control identify` → `Device Architecture: HAILO10H`)
**Clip:** `/home/giladn/Videos/Drone/Training/DJI_20260528155741_0029_D_prepared__fov50.mp4` (3840×2160)
**HEF:** `/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef` (VGA = 640×480 RGB input)

## TL;DR

**Resize happens exactly once — in the `hailotilecropper_dynamic` cropper, via the OpenCV CPU path,
resizing each tile to the network input dims 640×480.** The `videoscale` inside the standard
`INFERENCE_PIPELINE` helper is a **no-op** (sink caps == src caps == 640×480): the cropper has already
delivered the correct network-input size on its `src_1` pad, so videoscale negotiates 640×480 → 640×480
and copies through. There is **no double resize**.

## Method (note on the plan's debug spec)

The plan's `GST_DEBUG="hailotilecropper_dynamic:6,..."` does **not** surface the resize line. The
`Opencv Crop + Resize` message is emitted by the *base* cropper, whose debug category is
**`hailobasecropper`** (registered at `gsthailobasecropper.cpp:61`), not the element category
`hailotilecropper_dynamic`. The trace below therefore used:

```
GST_DEBUG="hailobasecropper:5,GST_CAPS:4,videoscale:5"
```

Pipeline used (3×2 static grid, `agg. ! fakesink`, ~25 s on chip; named `videoscale name=vs`,
`hailonet name=hnet` to isolate their pads):

```bash
gst-launch-1.0 -v filesrc location="<clip>" ! \
  decodebin ! videoconvert ! video/x-raw,format=RGB ! \
  hailotilecropper_dynamic name=tc tiles-static="0,0,0.4,0.5;0.3,0,0.4,0.5;0.6,0,0.4,0.5;0,0.5,0.4,0.5;0.3,0.5,0.4,0.5;0.6,0.5,0.4,0.5" \
  hailotileaggregator name=agg flatten-detections=true iou-threshold=0.3 \
  tc. ! queue ! agg.sink_0 \
  tc. ! video/x-raw,format=RGB ! queue ! \
    videoscale name=vs ! videoconvert ! \
    hailonet name=hnet hef-path=<hef> batch-size=1 force-writable=true ! \
    queue ! agg.sink_1 \
  agg. ! fakesink sync=false
```

This is the gst-launch transcription of the known-good `DYNAMIC_TILE_CROPPER_PIPELINE`
(`tiling_benchmark/tiling_record.py:152`): bypass branch on `sink_0`, cropped tiles on `src_1`
with a `format=RGB` capsfilter, rejoined at `hailotileaggregator`. It linked and ran on the chip with
no negotiation error.

## Evidence

### 1. OpenCV path is active (DSP path is `HAILO15_TARGET`-only, not compiled here)

`opencv_crop_and_resize` (`gsthailobasecropper.cpp:708`) ran for every tile. The DSP path
(all `#ifdef HAILO15_TARGET`) was **never** taken — zero `dsp`/`DSP` log lines in the trace.

```
hailobasecropper gsthailobasecropper.cpp:716:opencv_crop_and_resize:<tc> Opencv Crop + Resize: Input Width: 3840, Height: 2160.  Target Crop shape X: 0.000000 Y: 0.000000 Width: 0.400000 Height: 0.500000.  Resize width 640 height 480
hailobasecropper gsthailobasecropper.cpp:716:opencv_crop_and_resize:<tc> Opencv Crop + Resize: Input Width: 3840, Height: 2160.  Target Crop shape X: 0.300000 Y: 0.000000 Width: 0.400000 Height: 0.500000.  Resize width 640 height 480
```

- Input frame: **3840×2160**.
- Per-tile crop expressed as normalized `bbox` (e.g. `X 0.0 Y 0.0 W 0.4 H 0.5`) — confirms the cropper
  crops in normalized source space.
- **Cropper destination dims (resize target): 640 × 480.**
- 3816 such lines over the run (6 tiles/frame × ~636 frames). The resize dispatcher used is
  `resize_normal` (bilinear **stretch**, `cv::INTER_LINEAR`) per `gsthailotilecropper_dynamic.cpp:71`.

### 2. Cropper output pad (`tc.src_1`) is already 640×480

```
/GstPipeline:pipeline0/GstHailoTileCropperDynamic:tc.GstPad:src_1: caps = video/x-raw, format=(string)RGB, width=(int)640, height=(int)480, framerate=(fraction)0/1
/GstPipeline:pipeline0/GstHailoTileCropperDynamic:tc.GstPad:src_0: caps = video/x-raw, width=(int)3840, height=(int)2160, ... format=(string)RGB    # bypass pad keeps full source res
```

### 3. `videoscale` is a no-op (no second resize)

```
/GstPipeline:pipeline0/GstVideoScale:vs.GstPad:sink: caps = video/x-raw, format=(string)RGB, width=(int)640, height=(int)480, framerate=(fraction)0/1
/GstPipeline:pipeline0/GstVideoScale:vs.GstPad:src:  caps = video/x-raw, format=(string)RGB, width=(int)640, height=(int)480, framerate=(fraction)0/1
```

videoscale **in WxH == out WxH == 640×480**. No other caps were ever seen on its pads.

### 4. `hailonet` required input == 640×480 (matches the cropper output)

```
/GstPipeline:pipeline0/GstHailoNet:hnet.GstPad:sink: caps = video/x-raw, format=(string)RGB, width=(int)640, height=(int)480, framerate=(fraction)0/1
```

## Conclusion

| Site | In WxH | Out WxH | Resizes? |
|------|--------|---------|----------|
| `hailotilecropper_dynamic` (OpenCV `resize_normal`, stretch) | 3840×2160 crop region | **640×480** | **YES — the one and only resize** |
| `videoscale` (in `INFERENCE_PIPELINE`) | 640×480 | 640×480 | no (no-op pass-through) |
| `hailonet` input requirement | — | 640×480 | n/a (consumes 640×480) |

**Resize is done once, in the cropper.** It negotiates its `src_1` caps directly to the network input
size (640×480), so any downstream `videoscale` has nothing to do. The cache provenance therefore only
needs to record a single resize envelope: source crop region (in 3840×2160 pixels) → 640×480, mode
`stretch`, interpolation `linear`.

## Canonical gate inner pipeline (verbatim — for Task 6)

Because the cropper already resizes to the network input, the gate's inner (per-tile) branch needs **no
extra `videoscale`** — just a format guarantee + `hailonet` + `hailofilter`. This is the inner pipeline
fed to `DYNAMIC_TILE_CROPPER_PIPELINE(inner_pipeline, ...)` (which itself prepends the
`video/x-raw,format=RGB` capsfilter on the cropped `src_1` branch):

**Live pass (Pass 1):**
```
video/x-raw,format=RGB ! videoconvert ! hailonet hef-path=<HEF> batch-size=1 force-writable=true ! hailofilter so-path=<POST_SO> qos=false
```

**Cached pass (Pass 2):** identical up to and including `hailofilter`, with the live inference swapped
for cache replay — `hailonet` replaced by `hailocachereader ! hailocachebypass`:
```
video/x-raw,format=RGB ! videoconvert ! hailocachereader <cache props> ! hailocachebypass ! hailofilter so-path=<POST_SO> qos=false
```

Both passes tap per-tile `HailoROI` detections with a pad probe **after `hailofilter`** (pre-aggregator,
pre-NMS), keyed by the source-pixel crop rectangle.

Notes:
- Drop the helper's `videoscale` — proven no-op here; keeping it is harmless but adds nothing and would
  mask any future cropper/HEF dim mismatch. Keep one `videoconvert` for format safety.
- The cropper's `src_1` capsfilter `video/x-raw,format=RGB` is supplied by
  `DYNAMIC_TILE_CROPPER_PIPELINE` itself, so the inner-pipeline string above begins with the same
  `video/x-raw,format=RGB` capsfilter for explicitness (idempotent).
- 640×480 is dictated by the **HEF** (`hailo_yolov8n_4_classes_vga.hef`); the cropper reads the network
  input dims from the downstream caps. If the HEF changes, the cropper's resize target follows
  automatically — no pipeline edit needed.
