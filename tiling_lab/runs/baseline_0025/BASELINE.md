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

## Block 1 validation (2026-06-05)

Validation of the Block-1 re-acquisition fixes (commits `33cbc69` velocity
anchor, `c79d8a2` time-growing centre-distance gate, `6396b2d` CLI plumb) **plus**
the duplicate-track fix `e01fc4f` (`remove_duplicate_stracks` was pruning
NON-overlapping tracks — raw IoU vs 1−IoU distance). Replayed the three known
failing walker cells against the warm `0025_fov{50,60,70}__yolov8n4c_vga`
caches; identical knobs to the baseline (`--budget 3000 --fps 30
--discovery-fps 2 --discovery-grid 8x6 --max-zoom 2.0`). Result JSONs:
`dynamic_tiling/runs/block1_validation/` (untracked).

### Grid — walker (track 3), 24 runs

3 failing cells × `--reacq-motion {frozen,velocity}` × `--reacq-radius-growth {0,
0.001, 0.002, 0.005}`:

| cell | motion | growth | coverage | losses | ttr | recovery |
|------|--------|--------|----------|--------|-----|----------|
| fov50_ov0 | frozen | 0 | 0.490 | 2 | 2.00 | 0.50 |
| fov50_ov0 | frozen | 0.001 | 0.989 | 9 | 1.67 | 1.00 |
| fov50_ov0 | frozen | 0.002 | 0.989 | 9 | 1.67 | 1.00 |
| fov50_ov0 | frozen | 0.005 | 0.989 | 9 | 1.67 | 1.00 |
| fov50_ov0 | velocity | 0 | 0.490 | 2 | 2.00 | 0.50 |
| fov50_ov0 | velocity | 0.001 | 0.989 | 9 | 1.67 | 1.00 |
| fov50_ov0 | velocity | 0.002 | 0.989 | 9 | 1.67 | 1.00 |
| fov50_ov0 | velocity | 0.005 | 0.989 | 9 | 1.67 | 1.00 |
| fov60_ov0.25 | frozen | 0 | 0.967 | 16 | 2.69 | 1.00 |
| fov60_ov0.25 | frozen | 0.001 | 0.967 | 16 | 2.69 | 1.00 |
| fov60_ov0.25 | frozen | 0.002 | 0.967 | 16 | 2.69 | 1.00 |
| fov60_ov0.25 | frozen | 0.005 | 0.967 | 16 | 2.69 | 1.00 |
| fov60_ov0.25 | velocity | 0 | 0.397 | 3 | 1.50 | 0.67 |
| fov60_ov0.25 | velocity | 0.001 | 0.967 | 16 | 2.69 | 1.00 |
| fov60_ov0.25 | velocity | 0.002 | 0.967 | 16 | 2.69 | 1.00 |
| fov60_ov0.25 | velocity | 0.005 | 0.967 | 16 | 2.69 | 1.00 |
| fov70_ov0.25 | frozen | 0 | 0.987 | 13 | 1.31 | 1.00 |
| fov70_ov0.25 | frozen | 0.001 | 0.987 | 13 | 1.31 | 1.00 |
| fov70_ov0.25 | frozen | 0.002 | 0.987 | 13 | 1.31 | 1.00 |
| fov70_ov0.25 | frozen | 0.005 | 0.987 | 13 | 1.31 | 1.00 |
| fov70_ov0.25 | velocity | 0 | 0.987 | 13 | 1.31 | 1.00 |
| fov70_ov0.25 | velocity | 0.001 | 0.987 | 13 | 1.31 | 1.00 |
| fov70_ov0.25 | velocity | 0.002 | 0.987 | 13 | 1.31 | 1.00 |
| fov70_ov0.25 | velocity | 0.005 | 0.987 | 13 | 1.31 | 1.00 |

**Reading of the grid:** the discriminator is `growth > 0`, not the motion mode —
`frozen` and `velocity` are byte-identical at every growth ≥ 0.001 (the IoU
re-lock now fires within `track_buffer` thanks to `e01fc4f`, so the
velocity-extrapolated anchor never gets a chance to matter on these clips). The
three growth values 0.001/0.002/0.005 are also identical. The only failures are
at `growth = 0`: fov50_ov0 stays at 0.490 (the original frozen-anchor pathology),
and `velocity`/`growth=0` on fov60_ov0.25 actually regresses to 0.397 (a
different, costlier tile schedule — 5.46 tiles/frame — caused by the moving
anchor reshaping discovery ROIs, with no compensating re-lock benefit).
fov60_ov0.25 and fov70_ov0.25 already pass at `growth=0`/`frozen` because the
`e01fc4f` dedup fix alone rescues them.

### Chosen best config

**`--reacq-motion frozen --reacq-radius-growth 0.001`**

Summed walker coverage over the three cells = 2.943 (tied with all six
growth ≥ 0.001 configs). Tie-break → lowest growth (0.001); motion → `frozen`
(the unchanged default — `velocity` is indistinguishable here, so the only knob
that needs to move from the legacy default is the growth). Per-cell walker:
fov50_ov0 **0.989**, fov60_ov0.25 **0.967**, fov70_ov0.25 **0.987** — all > 0.8.

### No-regression check (best config on the other baseline cells)

Re-ran the remaining (fov × ov) cells at `frozen`/`0.001`. Per-trial coverage vs
the corresponding baseline numbers (walker = track 3, bg-person = track 4):

| cell | track | baseline cov | new cov | Δ | verdict |
|------|-------|--------------|---------|------|---------|
| fov50_ov0.25 | 3 walker | 0.989 | 0.989 | +0.000 | ok |
| fov50_ov0.25 | 4 bg | ~0.78 | 0.996 | +0.216 | ok |
| fov60_ov0 | 3 walker | 0.491 | 0.969 | +0.478 | ok |
| fov60_ov0 | 4 bg | 0.877 | 0.997 | +0.120 | ok |
| fov70_ov0 | 3 walker | 0.388 | 0.983 | +0.595 | ok |
| fov70_ov0 | 4 bg | 0.836 | 0.937 | +0.101 | ok |

No coverage regression anywhere (worst Δ is +0.000); every track holds or
improves. **GATE: PASS** — walker > 0.8 on all three step-2 cells AND no
regression > 0.02 on the others.

### Time-to-recover vs `track_buffer` — the dedup fix collapsed the 90-frame floor

The baseline's headline finding was a **bimodal** recovery: ~1–2 frames
(ROI-local) or **90+ frames / never** (waiting on the 2 fps discovery grid). At
the chosen config the slow mode is gone. Per-trial ttr (frozen/0.001, all 6 cells
= 12 trials) sorted: `[1.3, 1.4, 1.5, 1.6, 1.7, 2.0, 2.0, 2.0, 2.6, 2.7, 3.0, 47.0]`
— **11 of 12 ≤ 3 frames, max 47, none ≥ `track_buffer` (90)**. The lone 47-frame
value is one bg-person recovery on fov70_ov0; still comfortably inside the
90-frame `track_buffer` window. The `e01fc4f` dedup fix is what collapses the
floor: the duplicate (non-overlapping) track that the buggy
`remove_duplicate_stracks` used to prune now survives, so the locked id is
re-acquired *within* the tracker's lost window (1–3 frames) instead of having to
wait for a fresh discovery-grid detection (~90 frames). The growth knob then
mops up the residual fov50_ov0 case where even that in-buffer re-lock needs the
distance gate to open past the frozen anchor's IoU=0.
