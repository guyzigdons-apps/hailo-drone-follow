# Debug View: Real DSP Tile Crops (the exact NPU inputs) in the Web UI

**Status:** proposed / not yet implemented.
**Scope:** `--native-pipeline` mode only (the C++ `df_native_pipeline` + Python
orchestrator).

## Summary

Route the **actual 640×384 tile images the DSP produces** — the exact buffers
fed into the NPU — back to the Python/web UI, and show them in the existing
**"show tiles"** debug strip. Today that strip shows a *browser-side
reconstruction* cropped from the low-res display video; this task replaces it
(with fallback) by the literal chip inputs.

## Why it's needed (real debug purposes)

The whole point of tiling is that each region is cropped and resized to the
network input before inference. When detections look wrong on a region, the
first question is always *"what did the chip actually see?"* — and right now we
can only **approximate** the answer.

The current "show tiles" thumbnails are reconstructed in the browser from the
`/api/video` MJPEG (a 640×360 stream), so they are only **geometry-faithful**:

- ✅ correct crop rectangle and target aspect (good for "right region / no warp")
- ⚠️ **wrong source resolution** — 640×360 display frame vs the 1920×1080 AI
  frame the DSP actually crops from
- ⚠️ **wrong resize kernel** — browser `drawImage` (bilinear) vs the DSP resizer
- ⚠️ **wrong colors** — H264 + JPEG re-encode of the display stream vs raw NV12

That approximation cannot answer pixel-level questions that matter for real
debugging:

- Is the DSP **resize** introducing artifacts (ringing, chroma smear, wrong
  interpolation) at the network resolution?
- Are **colors / NV12 handling** correct end-to-end (YUV plane alignment, range)?
- Is a suspected **letterbox/padding** actually applied as configured?
- Does a specific region **at full AI resolution** contain enough detail for the
  detector, or is the person too small once resized?
- Ground-truth for **"the model saw exactly this"** when reproducing a missed or
  false detection.

Seeing the real chip input turns "the detection is wrong" into "the *input* is
wrong (bad crop/resize)" vs "the *model* is wrong (input fine, output bad)" —
which is the single most useful split when debugging a detection pipeline.

## Background: how the crop works today

Per frame, in the C++ binary:

1. **`DynamicTilingCropStage::prepare_crops`**
   (`robot_follow/native_pipeline/dynamic_tiling_stage.cpp`) converts each tile's
   normalized bbox into a pixel ROI (`dsp_crop_api_t = {start_x,start_y,end_x,end_y}`
   on the 1920×1080 AI frame, sink2).
2. **`DspBaseCropStage::process`** (framework) allocates a 640×384 NV12 DMABUF
   from a pool and builds a `dsp_crop_resize_params_t` (crop ROI + destination
   buffer + `scaling_mode` + pad color), then calls
   `dsp_utils::perform_dsp_telescopic_multi_resize` → `dsp_frontend_process`.
   The **DSP hardware does crop + resize in one DMA pass**, one 640×384 NV12
   buffer per tile.
3. Each output buffer is tagged with `set_scaling_bbox(tile_bbox)` (so the
   aggregator can map detections back to the full frame).
4. The buffer goes to the detection sub-pipeline: **`HailortAsyncStage`**
   (`ai_stage.hpp`) does `set_pix_buf(buffer)` + async `infer()` → NPU
   (YOLOv8s, `hailo_yolov8s_384_640.hef`).

**Key fact:** the 640×384 NV12 buffer from step 2 *is* the image the NPU sees.
It never leaves the C++ binary — it flows crop → NPU only. That buffer is what
we want to expose.

> Note: the NPU emits detection tensors, not the image, so nothing comes "back"
> *from* the NPU. We tap the image on its way *in* (the crop-stage output) and
> ship a copy to Python alongside the detections.

## Goal

Expose each per-frame 640×384 crop as a viewable image in the web UI's "show
tiles" strip, matched to its tile, **without regressing the hot path** (the ARM
cores are already the pipeline bottleneck — see Performance).

## Design / how to implement

### 1. Tap the crop-stage output (C++)

Insert a lightweight "debug tap" after the tiling crop stage (or a hook inside
our `DynamicTilingCropStage`) that, for each subframe NV12 buffer:

- JPEG-encodes it (prefer the H15 **hardware encoder** in the media library over
  a software encode to keep CPU off the critical path),
- attaches the tile index / `scaling_bbox` and the current frame id,
- publishes it (see Transport).

Only do this work when the debug view is **enabled** (see Gating).

### 2. Transport options (pick one)

| Option | How | Pros | Cons |
|---|---|---|---|
| **A. Separate ZMQ PUB topic** *(recommended)* | New PUB socket (e.g. `tcp://*:7002`) in our binary; Python subscribes alongside the existing 7000 detections | Fully in code we own; no framework/proto changes; decoupled from detection metadata | ⚠️ A **second ZMQ context** next to the framework's `analytic_metadata_zmq_sender` was a suspected cause of an earlier launch failure (see the bisection note in `main.cpp`) — must be tested carefully |
| **B. Piggyback on the framework metadata message** | Add `bytes crop_image` to `protos/analytics_metadata.proto`, fill it in the framework packager | One channel, perfectly frame-synced with detections | Requires modifying framework packaging / proto; heavier coupling |
| **C. Debug montage video stream** | DSP-composite tiles into one frame, encode with the existing H264 encoder to a new UDP port, decode in a second `video_bridge` | Reuses the proven sink1 → `video_bridge` → MJPEG path | Compositing a variable tile count is fiddly; extra encoder instance |

Recommendation: **Option A** for isolation, but validate against the
second-ZMQ-context concern first (bring the pipeline up with the extra socket
and confirm detections still flow — that's exactly the bisection worry).

### 3. Python side (`robot_follow/native_pipeline/subscriber.py`)

- Subscribe to the crop channel; decode/hold the latest JPEG per tile index in
  `SharedUIState` (e.g. `ui_state.set_tile_crops({idx: jpeg_bytes})`).
- Keep it best-effort and drop-old (like the existing frame handling) so a slow
  UI never backs up the pipeline.

### 4. Web server (`robot_follow/servers/web_server.py`)

- Add `GET /api/tile-crop/{i}` returning the latest JPEG for tile `i`
  (or include small base64 crops in the detections SSE — simpler but heavier per
  event; a per-tile endpoint is cleaner).
- Add `POST /api/tile-crops/debug {enabled: bool}` to toggle the C++ tap
  on/off (forwarded to the binary; see Gating).

### 5. UI (`robot_follow/ui/src/App.jsx`)

- In the "show tiles" strip, when real crops are available, render
  `GET /api/tile-crop/{i}` (poll at the debug rate) into each thumbnail.
- **Fall back** to the current browser reconstruction when a real crop isn't
  available (debug tap off, or not yet received), so the strip always shows
  something.
- Toggling "show tiles" should also flip the C++ tap on/off via the debug
  endpoint, so encoding only runs while someone is looking.

## Performance considerations (protect the hot path)

The ARM cores are the end-to-end bottleneck: the C++/NPU side publishes at the
full **15 fps**, but the Python subscriber processes ~**2.6 fps**. Naively
JPEG-encoding N × 640×384 every frame would add load to the exact resource
that's already saturated. Therefore the feature **must**:

- **Debug-gate** — encode/publish only while the debug view is open (no cost in
  normal operation).
- **Rate-limit** — ~2–4 fps is plenty to eyeball crops; do not run at 15 fps.
- **Prefer the HW JPEG encoder** over software encode.
- **Drop-old** everywhere (bounded queues), never block the crop → NPU path.

## Testing / verification

1. **Launch integrity** (esp. Option A): bring the pipeline up with the debug
   channel and confirm detections still flow on ZMQ 7000 (`msg/s ≈ 15`) — this
   directly checks the second-ZMQ-context concern.
2. **Correctness**: draw a region with a known landmark at a known spot; confirm
   the real crop shows that landmark, correctly framed and *not warped*.
3. **Compare** the real crop against the browser reconstruction — geometry
   should match; the real one should be sharper (sourced from 1920×1080) and
   truer in color.
4. **Hot-path cost**: with the debug view **off**, verify publish rate is still
   15 fps and subscriber fps is unchanged. With it **on**, confirm the pipeline
   FPS does not drop below the target.
5. **Edge cases**: single region (two identical crops — both should render);
   many regions; region flush to a frame edge (aspect clamp visible).

## File-by-file change checklist

- `robot_follow/native_pipeline/dynamic_tiling_stage.{hpp,cpp}` — debug tap hook
  after crop (gated).
- `robot_follow/native_pipeline/main.cpp` — wire the debug tap / crop PUB socket;
  respect the enable flag; **mind the bisection state** (crop stage vs framework
  static stage) and the second-ZMQ-context note.
- (Option B only) `robot_follow/native_pipeline/protos/analytics_metadata.proto`
  + regenerate `analytics_metadata_pb2.py` + framework packager.
- `robot_follow/native_pipeline/subscriber.py` — receive/hold latest per-tile
  crops.
- `robot_follow/native_pipeline/control_client.py` (or the tiles control path) —
  toggle the tap.
- `robot_follow/servers/web_server.py` — `GET /api/tile-crop/{i}` +
  `POST /api/tile-crops/debug`.
- `robot_follow/ui/src/App.jsx` — render real crops in the "show tiles" strip,
  fall back to reconstruction, toggle the tap with the strip.
- Build/deploy: cross-compile (`build.sh` + Yocto SDK), redeploy the binary,
  rsync the UI/Python, restart the orchestrator.

## Open questions / risks

- **Second ZMQ context** (Option A) vs the earlier launch regression — resolve
  by test, or fall back to Option B/C.
- **`main.cpp` bisection state**: it currently uses the framework's *static*
  `TilingCropStage`, not our `DynamicTilingCropStage`. The tap must attach to
  whichever stage is active (or this lands together with exiting bisection).
- **HW encoder availability/contention** with the existing sink0/sink1 encoders.
- Whether to reuse the tiles control socket or add a dedicated debug toggle.
