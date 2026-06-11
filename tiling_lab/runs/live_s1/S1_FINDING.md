# Spike S1 — does `hailotilecropper_dynamic` honour per-frame tile changes?

**Date:** 2026-06-11 · **Branch:** `dynamic-tiling-live` · **Box:** H10 (hailo10h)

## Verdict

**MECHANISM: A — PASS.** Setting the cropper's `tiles-static` GObject property
mid-stream (`cropper.set_property("tiles-static", ...)` from an aggregator-src
buffer probe) **is re-read every cropping period at runtime**. The crop set
changes live, with a ~1-cropping-period latency after each change, and **no caps
renegotiation errors or pipeline stalls**. No C++ extension of the plugin is
needed. Phase 2 proceeds with the property-set path.

## How it was tested

`tiling_lab/live/spike_s1.py` builds:

```
filesrc(0025_fov50.mp4) ! decodebin ! ...
  ! hailotilecropper_dynamic name=tc tiling-mode=single-scale tiles-static="<left>"
  ! <hailonet yolov8n_4cls + hailofilter libyolo_hailortpp_postprocess.so filter>
  ! hailotileaggregator flatten-detections=true ! fakesink
```

A buffer probe on `agg.src` flips `tiles-static` between the **left half**
(`0,0,0.5,1,s`) and the **right half** (`0.5,0,0.5,1,s`) every 30 frames and
logs, per frame, the x-centroid of all aggregated detections.

Resources (all on disk, the project's TILING defaults):
- HEF `/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef`
- post `.so` `/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so`, fn `filter`
- labels `/usr/local/hailo/resources/json/hailo_4_classes.json`
  (`["unlabeled","person","vehicle","face","license_plate"]`, threshold 0.5)

Command:
```bash
source setup_env.sh
python -m tiling_lab.live.spike_s1 \
  --video tiling_visualizer_site/dist/data/videos/0025_fov50.mp4 \
  --frames 150 --left "0,0,0.5,1,s" --right "0.5,0,0.5,1,s"
```

## Evidence

The 0025_fov50 scene has a car on the right (cx≈0.78). Detection presence
tracks the active crop half exactly:

| block (frames) | tiles set | det-frames | cx>0.5 | cx<0.5 |
|---|---|---|---|---|
| 0–29   | LEFT half  | 0  | 0  | 0 |
| 30–59  | RIGHT half | 27 | 27 | 0 |
| 60–89  | LEFT half  | 3  | 3  | 0 |
| 90–119 | RIGHT half | 27 | 27 | 0 |
| 120–149| LEFT half  | 3  | 3  | 0 |

- Right-side car is detected **only** while the right half is cropped (27/30
  frames each RIGHT block) and **disappears** under the left half (0 in the
  initial LEFT block; 3 residual = the 1-period switch latency after each flip).
- The initial property value was the LEFT half, and frames 0–29 produced zero
  right-side detections — ruling out a frozen-at-startup property.
- No `[BUS-ERROR]` lines; pipeline ran to the requested frame count cleanly.

## Notes for the runner (Task 3)

- One-cropping-period latency means tiles set on frame N take effect ~frame N+1.
  That matches the controller design (decide tiles for the *next* frame).
- Inference, post-process and the `person`/`vehicle` labels all work with the
  resources above — earlier "0 persons" was just the small corner test tiles
  missing the centred walker, not a misconfig.
