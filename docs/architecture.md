# Architecture & Data Flow

How a photon becomes a motor command. This document traces the full path from
the **camera sensor**, through the **Hailo NPU detection pipeline**, into the
**tracking / ReID / target-selection** logic, across the **shared-state
boundary**, into the **decision (controller) + robot adapter**, and out to the
**flight controller** — plus the parallel branch that feeds the **web UI /
OpenHD** and takes **operator input** back in.

Two production run-paths exist:

1. **Main path** — `robot_follow_app.py`: a single in-process GStreamer +
   Hailo pipeline. This is the default on RPi 5 + Hailo-8L and on x86 dev
   machines. **Most of this document covers this path.**
2. **Native H15 path** — `drone_follow_h15.py --native-pipeline`: a C++ binary
   (`df_native_pipeline`) owns the ISP + NPU and publishes detections over ZMQ;
   Python runs only tracking + control + UI. See
   [§ Native H15 pipeline](#native-h15-pipeline) below.

> The diagrams are [Mermaid](https://mermaid.js.org/) and render natively on
> GitHub. Colour = layer: 🔵 sensor/source · 🟣 NPU inference · 🟠 per-frame
> callback · 🟢 shared state · 🔴 decision/control · 🟥 robot/actuator ·
> 🩵 video-out / UI.

---

## Main path — end to end

```mermaid
flowchart TB
    classDef sensor  fill:#e3f2fd,stroke:#1565c0,color:#0d1b2a
    classDef npu     fill:#ede7f6,stroke:#5e35b1,color:#0d1b2a
    classDef cb      fill:#fff3e0,stroke:#ef6c00,color:#0d1b2a
    classDef state   fill:#e8f5e9,stroke:#2e7d32,color:#0d1b2a
    classDef control fill:#fce4ec,stroke:#c2185b,color:#0d1b2a
    classDef robot   fill:#ffebee,stroke:#c62828,color:#0d1b2a
    classDef ui      fill:#e0f7fa,stroke:#00838f,color:#0d1b2a
    classDef ext     fill:#eceff1,stroke:#455a64,color:#0d1b2a

    %% ---------- SENSOR / SOURCE ----------
    subgraph SRC_L["① Sensor & source — pipeline_adapter/hailo_drone_detection_manager.py"]
        CAM["📷 Camera / source<br/>--input rpi · usb · udp:// · shm://"]:::sensor
        SRC["GStreamer source stage<br/>SOURCE_PIPELINE / _udp_h264 / _shm<br/>NV12 / decoded frames"]:::sensor
        CAM --> SRC
    end

    %% ---------- NPU INFERENCE ----------
    subgraph NPU_L["② Detection — Hailo NPU (hailotilecropper + INFERENCE_PIPELINE)"]
        TILE["hailotilecropper<br/>3×2 tiles + full frame, multi-scale<br/>7 crops/frame → DSP resize 640×384"]:::npu
        NPU["Hailo NPU — YOLOv8 detector HEF<br/>batch = tile count"]:::npu
        POST["community NMS postprocess (.so)<br/>+ cross-tile merge → HailoDetection"]:::npu
        TILE --> NPU --> POST
    end

    %% ---------- PER-FRAME CALLBACK ----------
    subgraph CB_L["③ Per-frame callback — app_callback / _app_callback_inner"]
        CB["Filter label=='person'<br/>get_objects_typed(HAILO_DETECTION)"]:::cb
        BT["ByteTracker.update(Nx5)<br/>byte_tracker.py → persistent track IDs"]:::cb
        REID["ReIDManager<br/>gallery · drift-guard · reacquire · raw-fallback<br/>repvgg_a0 ReID HEF"]:::cb
        TGT["Target-selection state machine<br/>AUTO · LOCKED · SEARCH · IDLE"]:::cb
        CB -->|"Nx5 float32<br/>(xyxy, conf×1000)"| BT
        BT -->|"{track_id → bbox}<br/>Kalman-filtered tlwh"| TGT
        CB -.->|"BGR crop of target"| REID
        BT -.->|"person_by_id"| REID
        REID <-->|"cosine match ≥ threshold<br/>drift / reacquire verdict"| TGT
    end

    %% ---------- SHARED STATE ----------
    subgraph ST_L["④ Shared state — follow_api/state.py · servers/web_server.py"]
        SDS["SharedDetectionState<br/>Detection(center_x,center_y,bbox_height)<br/>+ available_ids + available_bboxes"]:::state
        FTS["FollowTargetState<br/>target id · mode · paused · last_seen"]:::state
        UIS["SharedUIState<br/>JPEG frame · detection dicts · velocity"]:::state
    end

    %% ---------- DECISION / CONTROL ----------
    subgraph CTRL_L["⑤ Decision — robot_api/orchestrator.py + follow_api/controller.py"]
        LOOP["run_robot_loop() @ control_loop_hz (10 Hz)<br/>get_latest() → SafetyContext"]:::control
        CTRL["controller.compute()<br/>signed-√ yaw P + distance P (bbox size)<br/>→ RobotCommand(fwd, yaw_rate, down)"]:::control
        LOOP -->|"Detection + Capabilities"| CTRL
        CTRL -->|"RobotCommand"| LOOP
    end

    %% ---------- ROBOT / ACTUATOR ----------
    subgraph ROB_L["⑥ Robot adapter — robot_api/adapters/"]
        ADPT["MavsdkDroneAdapter · Ros2RoverAdapter<br/>altitude-P · tilt-retreat · EMA smoothing · safety"]:::robot
        FC["🛩️ PX4 flight controller / 🚜 rover<br/>OFFBOARD body velocity · /cmd_vel Twist"]:::ext
        ADPT -->|"VelocityBodyYawspeed<br/>(fwd,0,down,yaw)"| FC
    end

    %% ---------- VIDEO OUT / UI ----------
    subgraph UI_L["⑦ Video-out branches & UI — vision_branches.py + servers/"]
        TEE["output_tee<br/>(post-callback video buffers)"]:::ui
        OVL["hailooverlay_community + target cross<br/>highlight_target pad probe"]:::ui
        DISP["🖥️ display sink / record .mkv<br/>clean.mkv + overlay.mkv"]:::ui
        OHD["x264 → RTP → udpsink :5500<br/>→ OpenHD WFB link"]:::ui
        MJPEG["jpegenc → appsink mjpeg_sink<br/>(clean, no overlay)"]:::ui
        WEB["WebServer :5001<br/>/api/video MJPEG · /api/detections SSE · /api/config"]:::ui
        FSRV["FollowServer :8080<br/>POST /follow/&lt;id&gt; · /follow/clear · /status"]:::ui
        BR["OpenHDBridge :5510 / :5511<br/>MAVLink params ⇄ ControllerConfig"]:::ui
        BROWSER["🌐 Browser — React Web UI"]:::ext
    end

    %% ===== cross-layer wiring =====
    SRC --> TILE
    POST --> CB

    TGT -->|"update(Detection, ids, bboxes)"| SDS
    TGT -->|"set/clear target · mode"| FTS
    CB  -->|"update_detections(dicts, following_id)"| UIS

    SDS -->|"get_latest() → (Detection, frame#)"| LOOP
    FTS -->|"mode / paused"| LOOP
    LOOP -->|"send_command(cmd, safety)"| ADPT
    LOOP -->|"update_velocity()"| UIS

    POST -. "video buffers" .-> TEE
    TEE --> MJPEG --> UIS
    TEE --> OVL --> DISP
    TEE --> OHD
    FTS -. "target id → recolour + cross" .-> OVL

    UIS -->|"MJPEG + SSE frames"| WEB
    WEB --> BROWSER
    BROWSER -->|"click-to-follow<br/>POST /follow/&lt;id&gt;"| FSRV
    FSRV -->|"set_target()"| FTS
    BR <-->|"follow_id (-1/0/N) · live tuning"| FTS
    BR <-->|"gain params"| CTRL
```

### Reading the diagram

- **The main loop is synchronous per frame** (layers ①–④ run inside the
  GStreamer streaming thread via `app_callback`). ByteTracker and ReID both run
  *in the callback*, not on a background thread.
- **The shared-state layer (④) is the only thread boundary.** The vision
  callback writes; the control loop (⑤, its own asyncio thread at 10 Hz) and the
  servers (⑦, their own threads) read. All access is lock-guarded.
- **Video and control diverge after the callback.** The same detections drive
  both the controller (⑤→⑥) and the on-screen overlay (⑦); the MJPEG branch
  taps *clean* frames (no overlay) so the browser draws its own boxes from SSE.
- **Operator input flows backwards** through ⑦: a click in the browser →
  `FollowServer` → `FollowTargetState`, which the callback reads next frame to
  switch AUTO→LOCKED. OpenHD ground-station input arrives via `OpenHDBridge`.

---

## Zoom-in: the per-frame decision (tracking → ReID → target)

Layer ③ is where "which pixels are the person we follow" is decided. This is
the most subtle part of the system. Full algorithm:
[`docs/tracking-reid-algorithm.md`](tracking-reid-algorithm.md).

```mermaid
flowchart TB
    classDef cb    fill:#fff3e0,stroke:#ef6c00,color:#0d1b2a
    classDef dec   fill:#fff8e1,stroke:#f9a825,color:#0d1b2a
    classDef out   fill:#e8f5e9,stroke:#2e7d32,color:#0d1b2a

    IN["Per-frame: list of person HailoDetection"]:::cb
    TRK["ByteTracker.update()<br/>→ active track IDs + Kalman bboxes"]:::cb

    MODE{"FollowTargetState<br/>mode?"}:::dec
    AUTO["AUTO: pick biggest person<br/>_find_biggest_person()"]:::cb
    LOCKED{"Locked track<br/>still active?"}:::dec

    DRIFT{"sample every N frames<br/>gallery sim vs target"}:::dec
    ADD["Enrich gallery<br/>(0.6 ≤ sim ≤ 0.9)"]:::cb
    SKIPD["Suspected drift (sim &lt; 0.6)<br/>→ inline _reacquire()"]:::cb
    DUP["Near-duplicate (sim &gt; 0.9)<br/>skip, periodic refresh"]:::cb

    LOST{"tracker lost target?"}:::dec
    RAWF["Raw-detection fallback<br/>score_visible_persons()<br/>(0 tracks but persons visible)"]:::cb
    REACQ["try_reidentify()<br/>match gallery ≥ reid_threshold<br/>over visible tracks"]:::cb
    SEARCH["SEARCH: yaw-spin toward last side<br/>until --reid-timeout, then AUTO"]:::cb

    EMIT["SharedDetectionState.update(Detection)<br/>+ available_ids + bboxes"]:::out

    IN --> TRK --> MODE
    MODE -->|"AUTO (no lock)"| AUTO --> EMIT
    MODE -->|"LOCKED / operator click"| LOCKED
    LOCKED -->|"yes"| DRIFT
    DRIFT --> ADD --> EMIT
    DRIFT --> DUP --> EMIT
    DRIFT --> SKIPD
    SKIPD -->|"other track matches"| REACQ
    SKIPD -->|"same track (false drift)"| EMIT
    LOCKED -->|"no active track"| LOST
    LOST -->|"raw persons visible"| RAWF
    RAWF -->|"best ≥ threshold"| EMIT
    LOST -->|"reacquire over tracks"| REACQ
    REACQ -->|"matched"| EMIT
    REACQ -->|"no match"| SEARCH --> EMIT
```

**Follow modes** (`follow_api/follow_mode.py`, wire values for OpenHD):

| Mode | `follow_id` | Meaning |
|------|:----------:|---------|
| **AUTO** | `0` | Follow the largest person; ReID gallery built for recovery. |
| **LOCKED** | `N` | Operator locked onto track/person `N` (click or REST). |
| **SEARCH** | — | Transient: target lost, ReID searching / yaw-spinning. |
| **IDLE** | `-1` | Hold position, ignore all detections. |

---

## Native H15 pipeline

On the **Hailo-15** target, inference is split out into a C++ binary
(`df_native_pipeline`, in [`robot_follow/native_pipeline/`](../robot_follow/native_pipeline/README.md))
that owns the ISP + NPU and publishes protobuf detections over ZMQ. The Python
orchestrator (`drone_follow_h15.py --native-pipeline`) runs **only** ByteTracker,
the follow loop, and the servers — layers ④–⑦ above are identical; layers ①–③
move into C++ and cross a ZMQ boundary. Enables per-tile inference regions
drawable from the web UI.

```mermaid
flowchart TB
    classDef cpp   fill:#ede7f6,stroke:#5e35b1,color:#0d1b2a
    classDef py    fill:#e8f5e9,stroke:#2e7d32,color:#0d1b2a
    classDef ext   fill:#eceff1,stroke:#455a64,color:#0d1b2a
    classDef net   fill:#fff4d6,stroke:#c98800,color:#0d1b2a

    subgraph CPP["C++ — df_native_pipeline (owns ISP + NPU)"]
        FE["ISP frontend (IMX678 / IMX675)"]:::cpp
        ENC4K["H264 encode (4K @ 30fps)"]:::cpp
        ENC720["H264 encode (720p @ 30fps)"]:::cpp
        DET["tiling (DSP crop→640×384) + YOLOv8s NPU<br/>batch = tile count, cross-tile NMS"]:::cpp
        ZMQ["ZMQ pub :7000<br/>protobuf hailo_analytics.Frame"]:::cpp
        FE --> ENC4K & ENC720 & DET
        DET --> ZMQ
    end

    U0(("udpsink :5000<br/>RTP H264")):::net
    U1(("udpsink :5002<br/>RTP H264")):::net
    ENC4K --> U0
    ENC720 --> U1

    subgraph PY["Python — drone_follow_h15 --native-pipeline"]
        REC["MulticastRecorderProcess<br/>udpsrc :5000 → mkv (no decode)"]:::py
        BRIDGE["NativeVideoBridge<br/>udpsrc :5002 → decode → 960×540 JPEG"]:::py
        SUB["NativePipelineSubscriber<br/>ZMQ sub :7000 + ByteTracker"]:::py
        UIS["SharedUIState"]:::py
        SDS["SharedDetectionState"]:::py
        CTRL["Control loop (10 Hz) → MAVSDK"]:::py
        BRIDGE -->|"JPEGs"| UIS
        SUB -->|"detections"| UIS
        SUB --> SDS --> CTRL
    end

    U0 -. "RTP" .-> REC
    U1 -. "RTP" .-> BRIDGE
    ZMQ -. "protobuf" .-> SUB

    UIS --> WEB["Web UI :5001<br/>MJPEG + SSE + draw inference regions"]:::py
    CTRL --> PIX["🛩️ Pixhawk (serial /dev/ttyACM0)"]:::ext
    WEB --> BROWSER["🌐 Browser"]:::ext
    REC --> STORE["Recording: /mnt/usb else /home/root/recordings"]:::ext
```

---

## Subsystem reference

| Layer | File | Key class / fn | Role & data produced |
|-------|------|----------------|----------------------|
| ① Source | `pipeline_adapter/hailo_drone_detection_manager.py` | `get_pipeline_string()` | Builds the GStreamer source per `--input` (rpi/usb/udp/shm). |
| ② Inference | *(same)* | `TILE_CROPPER_PIPELINE` + `INFERENCE_PIPELINE` | Tiling (3×2 + full, multi-scale) → YOLOv8 HEF on NPU → NMS `.so` → `HailoDetection`. |
| ③ Callback | *(same)* | `app_callback` → `_app_callback_inner`, `_run_tracker` | Filters persons, runs tracker, drives target state machine, emits `Detection`. |
| ③ Tracking | `pipeline_adapter/byte_tracker.py`, `tracker.py`, `tracker_factory.py` | `ByteTrackerAdapter`, `MetricsTracker` | Nx5 array → persistent IDs + Kalman bboxes. `--tracker {byte\|fast}`. |
| ③ ReID | `pipeline_adapter/reid_manager.py` | `ReIDManager` (`update_gallery`, `_reacquire`, `try_reidentify`, `score_visible_persons`) | Appearance gallery, drift-guard, re-acquisition, raw-detection fallback. `repvgg_a0` HEF. |
| ④ State | `follow_api/state.py` | `SharedDetectionState`, `FollowTargetState` | Thread-safe hand-off: detection→control, and who-to-follow. |
| ④ UI state | `servers/web_server.py` | `SharedUIState` | Latest JPEG frame + detection dicts + velocity + tiles. |
| ⑤ Loop | `robot_api/orchestrator.py` | `run_robot_loop()` | 10 Hz: read state → compute → send. Robot-agnostic. |
| ⑤ Controller | `follow_api/controller.py`, `config.py`, `types.py` | `compute()`, `ControllerConfig`, `RobotCommand` | Pure decision: yaw-P + distance-P → velocity command. `--yaw-only` zeroes forward. |
| ⑥ Adapter | `robot_api/adapters/mavsdk_drone.py`, `ros2_rover.py` | `MavsdkDroneAdapter`, `Ros2RoverAdapter` | Smoothing, altitude-P, safety, transport. `--robot {drone\|rover}`. |
| ⑦ Video out | `pipeline_adapter/vision_branches.py` | `assemble_output_stage`, `highlight_target` | `tee` → MJPEG / OpenHD H264 / display / record; target-cross overlay probe. |
| ⑦ Web | `servers/web_server.py` | `WebServer` :5001 | MJPEG (`/api/video`), SSE (`/api/detections/stream`), live config. |
| ⑦ Follow API | `servers/follow_server.py` | `FollowServer` :8080 | `POST /follow/<id>` click-to-follow, stale-id IoU recovery. |
| ⑦ OpenHD | `servers/openhd_bridge.py` | `OpenHDBridge` :5510/:5511 | MAVLink param ⇄ `ControllerConfig`; `follow_id` semantics. |
| Wiring | `robot_follow_app.py` | `main()`, `decide_branches` | Constructs and connects everything above. |

## CLI flags that reroute the flow

| Flag | Effect on the diagram |
|------|-----------------------|
| `--input {rpi\|usb\|udp://\|shm://}` | Selects the ① source branch. |
| `--webui` / `--openhd` (mutually exclusive) / `--display` / `--record` | Which ⑦ output branches are built. No UI flag → display defaults on. |
| `--yaw-only` / `--no-yaw-only` | Controller (⑤) zeroes forward+altitude, or full follow. |
| `--no-reid`, `--reid-*`, `--update-interval` | Disable / tune the ③ ReID gallery + drift logic. |
| `--auto-select` / `--no-auto-select` | On loss: re-acquire in AUTO vs hold IDLE. |
| `--tracker {byte\|fast}` | Which tracker in ③. |
| `--robot {drone\|rover}` | Which ⑥ adapter. |
| `--connection` / `--serial[-baud]`, `--takeoff-landing`, `--auto-offboard`, `--target-altitude` | ⑥ drone transport + offboard behaviour. |
| `--native-pipeline` (H15) | Switches to the [native pipeline](#native-h15-pipeline) (① –③ in C++ over ZMQ). |
| `--control-loop-hz` (10) | ⑤ tick rate. |

---

*Generated from the code as of branch `hailo15_tiling_optimize`. When the
pipeline shape, shared-state contract, or adapter surface changes, update the
Mermaid blocks above so this stays the source-of-truth architecture map.*
