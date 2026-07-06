# Native detection pipeline

C++ application that runs YOLOv8s detection on the Hailo15 ISP frontend
and publishes raw detections over ZMQ for the Python orchestrator (which
runs ByteTracker + the follow control loop).

> **Note on naming:** the binary is `df_native_pipeline` (not
> `drone_follow_native_pipeline`). Running it under the longer name on
> the H15 reproducibly triggers SIGKILL during `MediaLibrary::initialize()`
> from a kill source we never positively identified; an empirical bisection
> (byte-identical binary, only the filename changed) confirmed the shorter
> name is required.

## Build prerequisites

A Hailo Yocto Poky SDK matching the device image. Install once:

```bash
sudo /local/users/sdk/poky-glibc-x86_64-core-image-hailo-dev-armv8a-hailo15l-sbc-toolchain-4.0.23.sh -y
# Installs to /opt/poky/4.0.23/ (the default the build script expects).
```

## Build

```bash
./build.sh           # default: $TOOLCHAIN_DIR=/opt/poky/4.0.23
./build.sh --clean   # wipe and rebuild
TOOLCHAIN_DIR=/path/to/other/sdk ./build.sh
```

Output binary: `build/df_native_pipeline`.

## Deploy to H15

```bash
ssh root@10.0.0.1 "mkdir -p /home/root/native_pipeline"
scp build/df_native_pipeline root@10.0.0.1:/home/root/native_pipeline/
```

## Manual smoke test on the device

```bash
ssh root@10.0.0.1 "/home/root/native_pipeline/df_native_pipeline --zmq-port 7000 --timeout 30"
```

While it's running, on the same device (or a host that can reach the
ZMQ port), subscribe and verify messages flow:

```python
import zmq
ctx = zmq.Context()
s = ctx.socket(zmq.SUB)
s.connect("tcp://10.0.0.1:7000")
s.setsockopt_string(zmq.SUBSCRIBE, "")
while True:
    print(len(s.recv()), "bytes")
```

Expected: one binary protobuf message per inference frame
(~15–30/sec depending on the model).

## Wire format

Protobuf — schema is `protos/analytics_metadata.proto`
(copied from `hailo-analytics/hailo_analytics_api/src/pipeline/codecs/protos/`
in the media-library repo). Top-level message: `hailo_analytics.Frame`.

## Lifecycle in production

This binary is spawned as a subprocess by `drone_follow_h15.py`. Manual
launch is for debugging only — in production the Python orchestrator
manages start / stop / restart, similar to how `MavsdkDroneAdapter`
manages `mavsdk_server`.

## Running via the orchestrator

The Python orchestrator (`drone_follow_h15`) owns the follow control loop,
ByteTracker, the follow/HTTP servers, and the web UI (port 5001). Pass
`--native-pipeline` to have it spawn `df_native_pipeline` for video +
inference instead of the in-process Python Hailo pipeline:

```bash
# On the H15 (root@10.0.0.1), from /home/root:
PYTHONPATH=/home/root python3 -m robot_follow.drone_follow_h15 \
    --native-pipeline --serial /dev/ttyACM0
```

Relevant flags: `--native-pipeline`, `--zmq-port` (default 7000),
`--tiles "x,y,w,h;..."` (startup tile geometry; normalized [0,1], ≥2 tiles).
The web UI is served at `http://10.0.0.1:5001`. The `scripts/h15_boot/`
init.d service launches this same entry point from `DRONE_FOLLOW_ARGS`.

## Tiling & inference regions

Detection runs on **tiles**: sub-rectangles of the frame, each cropped and
resized on the H15 **DSP** to the network input (**640×384**, YOLOv8s
`hailo_yolov8s_384_640.hef`), run as its own NPU inference, then merged back
to full-frame coordinates by the aggregator (cross-tile NMS). Tiling lets a
small/distant person occupy more pixels in its tile than in the whole frame,
so it's detected. The NPU `batch_size` is set to the tile count (capped at 8)
so a frame's crops batch into one scheduled inference.

### Choosing tiles from the web UI (draw-to-select regions)

- **`＋ Add region`** (button or the `+` key) arms selection; **click-drag**
  on the video to draw an inference region. Draw again to add more.
- Each drawn region is **expanded to the network pixel aspect (640×384)** so
  the DSP's resize scales it uniformly — **no warp**. The extra area is real
  neighbouring scene (not black padding).
- The **full frame is always included as a tile**, so the whole scene is
  always inferred in addition to the drawn regions. (This also means a single
  region needs no duplicate — `full + region` already satisfies the ≥2-tile
  rule.)
- **Clear regions** reverts to the whole-frame default (4 quadrants + full).
- **Edit tiles** edits the region list as text (`x,y,w,h;...`).
- **Show tiles** overlays the *actual* tiles sent to the chip (padded) and
  shows a debug strip with each tile's chip-input image (crop → 640×384,
  reconstructed from the display frame — see
  [`docs/tile-crop-debug-view.md`](../../docs/tile-crop-debug-view.md) for the
  plan to expose the *literal* DSP buffers).

Display convention: the solid region boxes are the **original shapes you
drew**; "show tiles" shows the **padded tiles** actually fed to the NPU.

### How tiles are applied

The web UI POSTs the tile set to `POST /api/tiles`
(`{"spec": "x,y,w,h;..."}` or `{"tiles": [...]}`). Applying a change
currently **restarts** the native pipeline (~5 s: teardown + HEF/ISP cold
start), because the tile geometry is baked at pipeline construction. The
runtime hot-swap path (`DynamicTilingCropStage` + `control_server.cpp`) exists
but is disabled in `main.cpp` (a bisection left it on the framework's static
tiling stage); re-enabling it would make same-count geometry edits ~ms while
count changes still restart. See the source comments in `main.cpp`.

Why the DSP (not the CPU) does the crop/resize: it's zero-copy DMA into the
NPU-consumed buffer, NV12-native, and keeps the (already CPU-bound) ARM cores
free — the resizes run in parallel with control logic and inference.
