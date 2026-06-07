# ReID Inference-Budget Ablation (Block R)

Person-crop-only ReID recovery on top of the Block-1 best re-acquisition config. Arms: **none** (P0, no ReID), **generous** (P1 upper bound), **prod** (P2), **ambiguity** (P3), **motion** (P4 motion-gated + cadence decay), **histogram** (P5 motion gate + HSV color screen).

Budget 3000 inf/s @ 30 fps; reacq frozen / radius-growth 0.001; det HEF `hailo_yolov8n_4_classes_vga.hef`, ReID HEF `repvgg_a0_person_reid_512.hef`.

## 0025_fov50

| arm | coverage | mean IoU | losses | t-to-recover | recovery | frac dets embedded |
|---|---|---|---|---|---|---|
| none | 0.993 | 0.876 | 5.50 | 1.53 | 1.000 | 0.000 |
| generous | 0.993 | 0.876 | 5.50 | 1.53 | 1.000 | 0.814 |
| prod | 0.993 | 0.876 | 5.50 | 1.53 | 1.000 | 0.027 |
| ambiguity | 0.993 | 0.876 | 5.50 | 1.53 | 1.000 | 0.027 |
| motion | 0.993 | 0.876 | 5.50 | 1.53 | 1.000 | 0.027 |
| histogram | 0.993 | 0.876 | 5.50 | 1.53 | 1.000 | 0.027 |

## 0025_fov60

| arm | coverage | mean IoU | losses | t-to-recover | recovery | frac dets embedded |
|---|---|---|---|---|---|---|
| none | 0.980 | 0.860 | 9.00 | 2.84 | 1.000 | 0.000 |
| generous | 0.980 | 0.860 | 9.00 | 2.84 | 1.000 | 0.825 |
| prod | 0.980 | 0.860 | 9.00 | 2.84 | 1.000 | 0.032 |
| ambiguity | 0.980 | 0.860 | 9.00 | 2.84 | 1.000 | 0.029 |
| motion | 0.980 | 0.860 | 9.00 | 2.84 | 1.000 | 0.030 |
| histogram | 0.980 | 0.860 | 9.00 | 2.84 | 1.000 | 0.030 |

## 0025_fov70

| arm | coverage | mean IoU | losses | t-to-recover | recovery | frac dets embedded |
|---|---|---|---|---|---|---|
| none | 0.992 | 0.846 | 7.00 | 1.65 | 1.000 | 0.000 |
| generous | 0.992 | 0.846 | 7.00 | 1.65 | 1.000 | 0.797 |
| prod | 0.992 | 0.846 | 7.00 | 1.65 | 1.000 | 0.028 |
| ambiguity | 0.992 | 0.846 | 7.00 | 1.65 | 1.000 | 0.028 |
| motion | 0.992 | 0.846 | 7.00 | 1.65 | 1.000 | 0.028 |
| histogram | 0.992 | 0.846 | 7.00 | 1.65 | 1.000 | 0.028 |

## 0026_fov50

| arm | coverage | mean IoU | losses | t-to-recover | recovery | frac dets embedded |
|---|---|---|---|---|---|---|
| none | 0.647 | 0.733 | 3.82 | 5.40 | 0.784 | 0.000 |
| generous | 0.647 | 0.733 | 3.82 | 5.40 | 0.784 | 0.536 |
| prod | 0.647 | 0.733 | 3.82 | 5.40 | 0.784 | 0.052 |
| ambiguity | 0.647 | 0.733 | 3.82 | 5.40 | 0.784 | 0.023 |
| motion | 0.647 | 0.733 | 3.82 | 5.40 | 0.784 | 0.030 |
| histogram | 0.647 | 0.733 | 3.82 | 5.40 | 0.784 | 0.030 |

## Quality vs fraction of person-dets embedded

Ordered by embedding fraction (lower = cheaper). The Pareto read: the arm that holds coverage closest to **generous** at the lowest fraction wins.

| clip | arm | frac dets embedded | coverage |
|---|---|---|---|
| 0025_fov50 | none | 0.000 | 0.993 |
| 0025_fov50 | prod | 0.027 | 0.993 |
| 0025_fov50 | ambiguity | 0.027 | 0.993 |
| 0025_fov50 | motion | 0.027 | 0.993 |
| 0025_fov50 | histogram | 0.027 | 0.993 |
| 0025_fov50 | generous | 0.814 | 0.993 |
| 0025_fov60 | none | 0.000 | 0.980 |
| 0025_fov60 | ambiguity | 0.029 | 0.980 |
| 0025_fov60 | motion | 0.030 | 0.980 |
| 0025_fov60 | histogram | 0.030 | 0.980 |
| 0025_fov60 | prod | 0.032 | 0.980 |
| 0025_fov60 | generous | 0.825 | 0.980 |
| 0025_fov70 | none | 0.000 | 0.992 |
| 0025_fov70 | prod | 0.028 | 0.992 |
| 0025_fov70 | ambiguity | 0.028 | 0.992 |
| 0025_fov70 | motion | 0.028 | 0.992 |
| 0025_fov70 | histogram | 0.028 | 0.992 |
| 0025_fov70 | generous | 0.797 | 0.992 |
| 0026_fov50 | none | 0.000 | 0.647 |
| 0026_fov50 | ambiguity | 0.023 | 0.647 |
| 0026_fov50 | histogram | 0.030 | 0.647 |
| 0026_fov50 | motion | 0.030 | 0.647 |
| 0026_fov50 | prod | 0.052 | 0.647 |
| 0026_fov50 | generous | 0.536 | 0.647 |

## Interpretation (read this before the tables)

1. **Quality is IDENTICAL across all arms on every clip** — coverage/IoU/losses/
   time-to-recover match to 3 decimals, arm by arm. ReID never changed a single
   outcome at this config: the Block-1 re-acquisition gates (dedup fix + distance
   growth) re-lock in ~1-3 frames, before the ReID path gets a chance to act.
2. **0025 (all fovs) is saturated** by the Block-1 fixes (P0 coverage 0.98-0.99,
   recovery 1.0). It cannot discriminate ReID arms; its value is the COST column:
   selective policies embed ~3% of person dets vs generous's ~80% — a ~28x
   reduction in ReID inferences at identical quality.
3. **0026 has real headroom** (coverage 0.647, recovery 0.784) but ReID does not
   close it: its residual losses are DETECTION-bound (no person detections during
   the loss window — nothing to embed), not association-bound. ReID's measurable
   win requires clips with occlusion/crowd re-entry where the detector sees the
   person but identity is ambiguous. That is a dataset gap, not a code gap.
4. **Gallery sub-ablation skipped** — with all arms tied on quality, FIFO-vs-EMA
   comparison is meaningless on these clips; deferred to the harder-clip round.

## Chosen default policy

**`ambiguity` when ReID is enabled; `none` is equally good on this dataset.**
On every measured clip, P0 (none) matches every ReID arm — so the honest default
for *these clips* is no ReID at all. When ReID is wanted as insurance for harder
scenes (occlusion re-entry, crowds), `ambiguity` is the right default: it is the
cheapest gated arm (2.3-3.2% of dets embedded, vs prod's 2.7-5.2%), grounded in
the published risk-test rule, and its cost scales with scene ambiguity rather
than with loss duration. `generous` is strictly dominated (28x the embeds, zero
quality gain) and remains useful only as the upper-bound arm in future ablations.

## Reproduction

Driver (all clips x arms):

```bash
source setup_env.sh
python -m dynamic_tiling.run_reid_ablation --clips 0025:fov50,0025:fov60,0025:fov70,0026:fov50 --arms none,generous,prod,ambiguity,motion,histogram --budget 3000
python -m dynamic_tiling.run_reid_ablation --gallery-sub-ablation
python -m dynamic_tiling.run_reid_ablation --render-only
```

Per-row single runs:

```bash
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov50.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov50/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy none --cache dynamic_tiling/runs/cache/0025_fov50__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov50__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov50.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov50/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy generous --cache dynamic_tiling/runs/cache/0025_fov50__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov50__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov50.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov50/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy prod --cache dynamic_tiling/runs/cache/0025_fov50__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov50__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov50.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov50/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy ambiguity --cache dynamic_tiling/runs/cache/0025_fov50__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov50__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov50.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov50/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy motion --cache dynamic_tiling/runs/cache/0025_fov50__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov50__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov50.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov50/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy histogram --cache dynamic_tiling/runs/cache/0025_fov50__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov50__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov60.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov60/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy none --cache dynamic_tiling/runs/cache/0025_fov60__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov60__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov60.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov60/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy generous --cache dynamic_tiling/runs/cache/0025_fov60__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov60__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov60.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov60/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy prod --cache dynamic_tiling/runs/cache/0025_fov60__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov60__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov60.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov60/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy ambiguity --cache dynamic_tiling/runs/cache/0025_fov60__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov60__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov60.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov60/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy motion --cache dynamic_tiling/runs/cache/0025_fov60__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov60__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov60.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov60/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy histogram --cache dynamic_tiling/runs/cache/0025_fov60__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov60__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov70.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov70/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy none --cache dynamic_tiling/runs/cache/0025_fov70__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov70__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov70.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov70/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy generous --cache dynamic_tiling/runs/cache/0025_fov70__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov70__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov70.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov70/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy prod --cache dynamic_tiling/runs/cache/0025_fov70__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov70__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov70.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov70/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy ambiguity --cache dynamic_tiling/runs/cache/0025_fov70__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov70__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov70.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov70/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy motion --cache dynamic_tiling/runs/cache/0025_fov70__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov70__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov70.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0025_fov70/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy histogram --cache dynamic_tiling/runs/cache/0025_fov70__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0025_fov70__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0026_fov50/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy none --cache dynamic_tiling/runs/cache/0026_fov50__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0026_fov50__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0026_fov50/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy generous --cache dynamic_tiling/runs/cache/0026_fov50__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0026_fov50__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0026_fov50/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy prod --cache dynamic_tiling/runs/cache/0026_fov50__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0026_fov50__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0026_fov50/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy ambiguity --cache dynamic_tiling/runs/cache/0026_fov50__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0026_fov50__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0026_fov50/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy motion --cache dynamic_tiling/runs/cache/0026_fov50__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0026_fov50__yolov8n4c_vga.sqlite3
python -m dynamic_tiling.run_trials --video /home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4 --gt-tracks dynamic_tiling/runs/gt_verify_0026_fov50/gt_tracks.verified.json --budget 3000 --discovery-grid 8x6 --discovery-fps 2 --discovery-overlap 0.25 --reacq-motion frozen --reacq-radius-growth 0.001 --reid-policy histogram --cache dynamic_tiling/runs/cache/0026_fov50__yolov8n4c_vga.sqlite3 --reid-cache dynamic_tiling/runs/cache/0026_fov50__yolov8n4c_vga.sqlite3
```
