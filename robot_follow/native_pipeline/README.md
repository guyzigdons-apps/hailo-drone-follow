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
