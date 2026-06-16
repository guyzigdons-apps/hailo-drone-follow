# Central-Acquisition Showcase — Results (0012, 4K60)

Validates **central-target auto-acquisition** + dynamic tiling on
`DJI_20260614160924_0012_D.MP4` (HEVC 3840×2160 @ 59.94, 3103 frames). The target
is a **single, small/distant car**. Chosen by the 7-clip scan as the clip where
the target gets most central (min centre-dist 0.025).

## Result
- **Sustains 4K60**: achieved ~70 fps, `sustains_60fps: true`, mean ~1.75
  tiles/frame, max 3 (no spike).
- **Locks the correct (central) target** and tracks it for a contiguous
  **~353 frames (frames 2091–2445, ~6 s)** — centre ≈ (0.58, 0.40), bbox ~1.4 %
  of frame width.

## How central acquisition works (and why earlier attempts failed)
A small distant target is sampled only ~2 fps by the striped dense grid, so it
**never forms an activated ByteTracker track before locking** — and the dense ROI
tile that would track it only turns on *after* a lock. Resolution:
1. Auto-acquire from **raw detections**, not tracks: accumulate central-detection
   observations of the same target, associated by **centre distance** (tiny
   few-px boxes don't overlap on IoU), tolerating sampling gaps.
2. On reaching `--central-frames` central observations, set a **pending seed**.
   While pending, the lock reports TRACKING at the seed so the ROI tile densely
   samples that spot until a real track activates there and is adopted.
3. `--init-bbox x,y,w,h` is the manual initial-location fallback (same seed path).

## Reproduce
```bash
source setup_env.sh
# Auto central acquisition (default):
python -m tiling_lab.live.run_showcase \
  --video /home/giladn/Videos/Drone/Training/Car/DJI_20260614160924_0012_D.MP4 \
  --out tiling_lab/runs/showcase_0012 --fps 60 --target-class vehicle --label showcase_0012
# Manual seed fallback (if a target is too small to auto-acquire):
#   ... --init-bbox 0.57,0.39,0.015,0.012
python -m tiling_lab.viewer.overlay_viewer \
  --video .../DJI_20260614160924_0012_D.MP4 \
  --frames tiling_lab/runs/showcase_0012/frames.json:showcase_0012 \
  --export tiling_lab/runs/showcase_0012/out.mp4 --export-tiles showcase_0012 \
  --export-fps 60 --export-width 1920
```

## Artifacts (gitignored — regenerate as above)
- `frames.json`, `metrics.json`
- `showcase_0012_h264.mp4` — full clip (1920×1080@60).
- `showcase_0012_tracked.mp4` — **focused ~7.5 s clip of the tracked span** (the
  money shot: small central car followed by the dynamic ROI tile).

## Known limitation
The target is at the detector's size limit (~23 px wide in 4K). Acquisition +
tracking work where it's central and detected, but a much smaller/faster target
or a clip where it's never centred will need the `--init-bbox` seed and/or a
clip with a larger target. Per-clip detection density (7-clip scan): 0012 & 0007
richest; 0013/0014 sparse.
