# 0025 verified-GT tracking/recovery baseline

First tracking+recovery baseline scored against **locked, human-verified GT**
(`gt_verify_0025_fov{50,60,70}/gt_tracks.verified.json`, approved 2026-06-03).
Supersedes the 2026-06-02 Phase-0 number (coverage 0.254) which used the
unverified 0026 14-person GT.

Run: 2026-06-04, `python -m dynamic_tiling.run_trials` per FOV with identical knobs:

```
--budget 3000 --fps 30 --discovery-fps 2 --discovery-grid 8x6 --max-zoom 2.0
--hef hailo10h/hailo_yolov8n_4_classes_vga.hef  (class_offset=1, nms 0.25)
```

Full machine-readable results (params + aggregate + per-trial): `trials_fov{50,60,70}.json`.

## Aggregate (mean over the 2 person trials per FOV)

| FOV | tiles/frame | coverage | mean IoU | drift | loss events | time-to-recover | recovery success |
|-----|------------|----------|----------|-------|-------------|-----------------|------------------|
| 50  | 6.23       | 0.465    | 0.901    | 0.000 | 1.0         | 46.0            | 0.500            |
| 60  | 5.18       | 0.684    | 0.871    | 0.000 | 2.0         | 47.0            | 0.833            |
| 70  | 5.42       | 0.612    | 0.861    | 0.000 | 1.5         | 61.5            | 0.750            |

## Per-trial

| FOV | track       | GT frames | coverage | mean IoU | losses | recovery                  |
|-----|-------------|-----------|----------|----------|--------|---------------------------|
| 50  | 3 walker    | 1314      | 0.054    | 0.918    | 1      | FAILED (never reacquired) |
| 50  | 4 bg-person | 746       | 0.877    | 0.885    | 1      | recovered @ 92 frames     |
| 60  | 3 walker    | 1314      | 0.491    | 0.877    | 3      | 2/3 recovered, ~2 frames  |
| 60  | 4 bg-person | 746       | 0.877    | 0.865    | 1      | recovered @ 92 frames     |
| 70  | 3 walker    | 1314      | 0.388    | 0.895    | 2      | 1/2 recovered, ~1 frame   |
| 70  | 4 bg-person | 746       | 0.836    | 0.827    | 1      | recovered @ 122 frames    |

## Findings

1. **Hold quality is solved.** While locked: IoU 0.86–0.90, zero distractor
   drift, at every FOV. The follow loop itself is not the quality problem.
2. **Recovery is the whole game.** Every lost coverage point is time spent
   un-reacquired. fov50 walker is pathological: one loss, then ~1240 frames
   (41 s of 2 fps 8x6 discovery) with no re-lock.
3. **Recoveries are bimodal.** Either ~1–2 frames (ROI-local re-detection) or
   90+ frames / never (waiting on the discovery grid). The slow path is the
   Phase A target.
4. **bg-person is near-identical across FOVs** (0.877/0.877/0.836; recovery
   92/92/122) → a deterministic event at a fixed point in its track (likely its
   GT span start vs lock acquisition), not tracking noise.

## Root cause of failed recoveries (2026-06-04, tagged replay dumps)

The walker losses are NOT detection or ByteTracker failures. From the tagged
dumps (`frames_fov*_ov*/trial_3.frames.json` — tracker ids, activation, and the
re-acquisition anchor per frame):

- fov50 ov0: after the f71 loss (1243 lost frames) the detector saw the walker
  in 404 frames and ByteTracker held an ACTIVATED track on him in 135 (e.g.
  trk89, IoU 0.82 w/ GT, conf 0.99).
- fov60 ov0.25: after the f215 loss (1099 lost frames): detector 382, activated
  track on walker 210.

In both, `TargetLock.step()`'s only re-lock path — a new activated track whose
IoU with the FROZEN last-known bbox exceeds `_REACQ_IOU` — never fires:
the walker walks away from the anchor, IoU→0, loss becomes permanent.
Stationary targets (bg-person) recover; moving targets cannot survive any loss
longer than their anchor overlap. Candidate fixes (velocity-extrapolated
anchor, time-growing search radius, ReID as the model boundary) are tracked in
`docs/superpowers/plans/INDEX.md` for Phase A.

## Discovery overlap (`--discovery-overlap 0.25`) — mixed effect

`TileScheduler(grid_overlap)` landed 2026-06-04 (cell = span/(g−(g−1)·o),
stride = cell·(1−o)). At fixed 8x6 it grows tiles ~28% → boundary objects stay
whole (fov50 walker: dets fragment-free, losses 1–2 frames) but per-target
resolution drops (hurts wider FOVs where the walker is tiny):

| FOV | metric            | ov 0       | ov 0.25 |
|-----|-------------------|------------|---------|
| 50  | walker coverage   | 0.054      | **0.989** (9 losses, all rec ~1.6f) |
| 50  | aggregate cov     | 0.465      | **0.932**; tiles/frame 6.23→3.96 |
| 60  | walker coverage   | 0.491      | **0.164** (1 loss @f215, never rec) |
| 70  | walker coverage   | 0.388      | 0.278   |
| all | bg-person cov     | 0.84–0.88  | 0.78–0.88 (flat) |

Interpretation: overlap shifts WHEN losses happen; final coverage is then
decided by a binary event — does any loss outlive the anchor overlap (frozen-
anchor gate above). Once re-acquisition is fixed, overlap should be re-swept as
a tiling-quality knob (boundary wholeness vs per-target resolution), free of
recovery luck.
