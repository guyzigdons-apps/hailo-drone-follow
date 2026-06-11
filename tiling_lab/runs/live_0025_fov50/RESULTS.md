# Live dynamic-tiling — end-to-end validation on 0025_fov50

**Date:** 2026-06-11 · **Branch:** `dynamic-tiling-live` · **Box:** H10 (hailo10h)
**Plan:** `docs/superpowers/plans/2026-06-11-dynamic-tiling-live-on-gt-video.md`

## What this demonstrates

The dynamic tile scheduler (`hailo_tiling.dynamic.TileScheduler`) runs **live in
a real GStreamer pipeline on the chip**, driving `hailotilecropper_dynamic` via
per-frame `tiles-static` updates. Discovery grid acquires the walker, then a
budget-trimmed ROI tile follows it frame-to-frame. S1 mechanism: runtime
property re-read (see `../live_s1/S1_FINDING.md`).

Pipeline (standalone runner `tiling_lab/live/run_live.py`):
```
filesrc(0025_fov50.mp4) ! decodebin
  ! hailotilecropper_dynamic (tiles-static set every frame by the controller)
  ! hailonet yolov8n_4cls VGA + hailofilter (libyolo_hailortpp_postprocess.so)
  ! hailotileaggregator ! hailooverlay_community ! x264enc ! matroskamux ! filesink
```
A buffer probe on `agg.src` feeds this frame's person detections to
`DynamicTilingController.update()` and pushes the returned tiles-static onto the
cropper for the next frame (1 cropping-period latency).

## Headline numbers (full clip, 1315 frames)

| budget (inf/s) | person-detected | TRACKING | mean tiles/frame |
|---|---|---|---|
| 60 (~2/frame)  | 7.8%  | 3.5%  | 1.00 |
| **300 (~10/frame cap)** | **64.7%** | **44.6%** | **3.46** |

Command (default budget is now 300):
```bash
source setup_env.sh
python -m tiling_lab.live.run_live \
  --video tiling_visualizer_site/dist/data/videos/0025_fov50.mp4 \
  --out tiling_lab/runs/live_0025_fov50
```

## Reading the result

- **End-to-end works.** No bus errors / caps stalls across 1315 frames; clean
  EOS; `overlay.mkv` + 1315-row `tiles.jsonl` written.
- **The ROI follows the target.** Sample overlay frames (in `../live_0025_fov50_b300/`):
  frame_03 the walker is lower-centre with the ROI tile on it; frame_05 the
  walker has moved left and the ROI tiles moved left with it. Visual proof the
  tiles relocate onto the target.
- **Budget matters for acquisition.** At budget 60 (~2 inf/frame) the 6-tile
  discovery grid is starved by `BudgetTrimModifier`, so the small aerial walker
  is acquired only 7.8% of frames (84.8% LOST). Raising to 300 lets discovery
  fire and acquisition jumps to 64.7% detected / 44.6% TRACKING at 3.46
  tiles/frame — the budget-reallocation thesis, live: ~65% follow of a tiny
  target at 3.46 tiles/frame vs 48 tiles/frame for dense 8×6.

## Caveats / not-yet

- 44.6% TRACKING (vs 64.7% detected) reflects ByteTracker drop/reacquire on the
  intermittently-detected small target — this is the AUTO-mode auto-lock with no
  GT seeding, unlike the lab's GT-seeded single-target runs (≈0.93). A live
  target-recall ablation vs the locked GT is future work.
- One-frame latency between detection and the tile that uses it (by design).
- This is the standalone runner, vision-only. Folding into the `drone-follow`
  app behind a `--dynamic-tiling` flag (and the prod-ByteTracker `e01fc4f`
  sim-sanity precondition for flying) are the follow-ups listed in the plan.
