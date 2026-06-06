# Phase A — single-target tiling sweep (coordinate descent + budget frontier)

Recovery config fixed at the Block-1 best (`--reacq-motion frozen --reacq-radius-growth 0.001`,
ReID `none` per the Block-R ablation). Budget 3000 inf/s @30fps unless noted; clip 0025, 3 FOVs,
locked verified GT. Scoring: `mean(coverage) − 0.005·mean(tiles/frame)`. Run JSONs: `runs/phase_a/` (untracked).

## Winner

**`6x4 grid @ 1 fps discovery, overlap 0.15`** — coverage **0.989** at **1.59 tiles/frame**
(zoom and target_model_h are no-ops at the winner). The pre-sweep default (8x6 @ 2 fps, ov 0.25)
reaches the same coverage at 2.40 tiles/frame — with recovery fixed, HALVING the discovery
density and rate costs nothing. 4x3 is the coverage cliff (0.955).

## All configs (mean over 3 FOVs, sorted by score)

| config | coverage | tiles/frame | mean IoU | recovery |
|---|---|---|---|---|
| grid6x4_fps1_ov0.15_zoom1.5_h60 | 0.9893 | 1.590 | 0.860 | 1.000 |
| grid6x4_fps1_ov0.15_zoom1.5_h40 | 0.9893 | 1.590 | 0.861 | 1.000 |
| grid6x4_fps1_ov0.15_zoom2.0_h60 | 0.9893 | 1.590 | 0.860 | 1.000 |
| grid6x4_fps1_ov0.15_zoom3.0_h30 | 0.9893 | 1.590 | 0.860 | 1.000 |
| grid6x4_fps1_ov0.15_zoom3.0_h40 | 0.9893 | 1.590 | 0.861 | 1.000 |
| grid6x4_fps1_ov0.15_zoom2.0_h40 | 0.9893 | 1.590 | 0.861 | 1.000 |
| grid6x4_fps1_ov0.15_zoom3.0_h60 | 0.9893 | 1.590 | 0.860 | 1.000 |
| grid6x4_fps2_ov0.15_zoom3.0_h60 | 0.9892 | 2.395 | 0.860 | 1.000 |
| grid8x6_fps1_ov0.15_zoom3.0_h60 | 0.9892 | 2.395 | 0.860 | 1.000 |
| grid6x4_fps1_ov0.25_zoom2.0_h40 | 0.9823 | 1.587 | 0.861 | 1.000 |
| grid6x4_fps1_ov0.25_zoom3.0_h60 | 0.9823 | 1.587 | 0.859 | 1.000 |
| grid6x4_fps2_ov0.25_zoom2.0_h40 | 0.9825 | 2.391 | 0.860 | 1.000 |
| grid6x4_fps4_ov0.15_zoom3.0_h60 | 0.9890 | 3.798 | 0.859 | 0.833 |
| grid12x9_fps1_ov0.15_zoom3.0_h60 | 0.9890 | 4.087 | 0.858 | 1.000 |
| grid8x6_fps2_ov0.25_zoom2.0_h40 | 0.9882 | 4.000 | 0.861 | 1.000 |
| grid6x4_fps1_ov0_zoom2.0_h40 | 0.9758 | 1.583 | 0.861 | 1.000 |
| grid6x4_fps1_ov0_zoom3.0_h60 | 0.9758 | 1.583 | 0.859 | 1.000 |
| grid6x4_fps4_ov0.25_zoom2.0_h40 | 0.9853 | 3.796 | 0.861 | 0.833 |
| grid12x9_fps2_ov0.25_zoom2.0_h40 | 0.9889 | 7.106 | 0.861 | 1.000 |
| grid4x3_fps1_ov0.15_zoom3.0_h60 | 0.9554 | 1.171 | 0.860 | 1.000 |
| grid4x3_fps2_ov0.25_zoom2.0_h40 | 0.9354 | 1.562 | 0.862 | 1.000 |

## Budget frontier @ winner config

| budget (inf/s) | coverage (mean 3 fovs) | tiles/frame | recovery |
|---|---|---|---|
| 300 | 0.5081 | 0.609 | 0.833 |
| 600 | 0.9893 | 1.405 | 1.000 |
| 1000 | 0.9893 | 1.590 | 1.000 |
| 1500 | 0.9893 | 1.590 | 1.000 |
| 3000 | 0.9893 | 1.590 | 1.000 |

## Deltas vs the pre-fix baseline (BASELINE.md, 2026-06-04)

Aggregate coverage fov50/60/70 was 0.465/0.684/0.612 at 4-6 tiles/frame;
the winner config posts ~0.99 at 1.59 — recovery fixes (Block 1) plus the sweep
turn a recall problem into a budget problem.

## Reproduction

```bash
python -m dynamic_tiling.run_sweep --passes 2 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy none
python -m dynamic_tiling.run_sweep --budget-frontier --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy none
```
