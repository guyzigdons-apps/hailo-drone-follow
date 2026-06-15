# Dynamic-Tiling Realtime Showcase — Results (0013, 4K60)

Validation of the striped dynamic-tiling pipeline on a single-vehicle (SOT)
DJI drone clip. Spec: `docs/superpowers/specs/2026-06-15-tiling-showcase-realtime-design.md`.
Plan: `docs/superpowers/plans/2026-06-15-tiling-showcase-realtime.md`.

**Hardware:** laptop, HAILO10H. **Clip:** `DJI_20260614161040_0013_D.MP4`
(HEVC, 3840×2160, 59.94 fps, 2200 frames, ~37 s, no rotation metadata).
**HEF:** `hailo10h/hailo_yolov8n_4_classes_vga.hef`, target class `vehicle`.

## Headline: sustains 4K60 ✅

Full-clip `run_showcase` (decode → tile-crop → track → infer, **no draw, no
saved video**), `metrics.json`:

| metric | value |
|---|---|
| frames | 2200 |
| achieved_fps | **~69–80** (run-to-run) |
| sustains_60fps | **true** |
| mean_tiles_per_frame | 2.05 |
| max_tiles_per_frame | 3–4 |
| p95_tiles_per_frame | 3 |

Per-frame tile-count distribution `{1: 7, 2: 2070, 3: 123}` — **flat, no
discovery spike**: the 8×6 dense grid (2 fps full refresh) is striped one
sub-stripe per frame, so per-frame inference stays ~2 tiles instead of bursting
the whole grid on cadence frames.

### Why it's fast (and was nearly not)
Component baselines confirm head-room: HEVC 4K decode ~208 fps, raw HEF on device
~352 fps. The assembled pipeline initially ran at ~2 fps until three fixes
(documented in `.claude/memory/live_tiling_pipeline_perf.md`):
1. **Probe moved off `agg.src` to the downstream queue** — the controller step
   was serialising the aggregator's streaming thread (~2 fps → ~65 fps).
2. **`fakesink enable-last-sample=false`** — bare fakesink pinned a pool buffer
   and deadlocked after frame 0.
3. **Write outputs then `os._exit(0)`** — Hailo elements abort on the
   PLAYING→NULL transition in headless runs.

## Persistence + SOT
`frames.json` detections combine the **live SOT target** (tracked every frame
from its ROI tile — 72 TRACKING frames on this clip) with the **persisted dense
union** (background context that survives between ~0.5 s dense refreshes):
`{target: 72, vehicle: 126, person: 270}` detection records.

## Artifacts (this dir, not committed — regenerate as below)
- `frames.json` — visualizer schema (detections + tiles per frame).
- `metrics.json` — throughput + tile-count stats.
- `showcase_0013_h264.mp4` — 1920×1080 @60 fps visualizer render (tiles +
  persisted boxes + recoloured SOT target), H.264 crf21.

## Reproduce
```bash
source setup_env.sh
# 1. Ingest (rotation-only; this clip has none, passes through, no resize):
python -m tiling_lab.video.prepare_video \
  /home/giladn/Videos/Drone/Training/Car/DJI_20260614161040_0013_D.MP4 --verify
# 2. Realtime pipeline (no draw / no save) -> frames.json + metrics.json:
python -m tiling_lab.live.run_showcase \
  --video /home/giladn/Videos/Drone/Training/Car/DJI_20260614161040_0013_D.MP4 \
  --out tiling_lab/runs/showcase_0013 --fps 60 --target-class vehicle --label showcase
# 3. Render the visualizer showcase (offline, postprocess draw):
python -m tiling_lab.viewer.overlay_viewer \
  --video /home/giladn/Videos/Drone/Training/Car/DJI_20260614161040_0013_D.MP4 \
  --frames tiling_lab/runs/showcase_0013/frames.json:showcase \
  --export tiling_lab/runs/showcase_0013/showcase_overlay.mp4 \
  --export-tiles showcase --export-fps 60 --export-width 1920
# (then transcode showcase_overlay.mp4 -> compact H.264 with ffmpeg crf 21)
```

## Note on the RPi target
The laptop number *includes* HEVC decode (208 fps). On the RPi the same
controller runs from a **live camera** (no decode), so the crop+track+infer
budget — the part validated here — is what matters there.
