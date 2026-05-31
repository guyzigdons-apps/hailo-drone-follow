# Ablation table

- cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov50__a2e9861507428064.sqlite3`
- video: `/home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4`
- source: 3840x2160
- frames: 879
- reference: `12x9` (12x9), IoU>=0.5
- dynamic cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov50__dynamic.sqlite3` (per-source-frame; target class 2)
- `recall_delta_at_matched_budget` = dynamic recall − the closest-mean-tiles static grid's recall.

| config | kind | mean_tiles_per_frame | n_dets | recall_vs_reference | precision_vs_reference | n_misses | matched_static | recall_delta_at_matched_budget |
|---|---|---:|---:|---:|---:|---:|---|---:|
| 1x1 | static | 1.00 | 2643 | 0.2802 | 0.9368 | 0 | — | — |
| 2x2 | static | 4.00 | 3247 | 0.3396 | 0.9242 | 0 | — | — |
| 3x2 | static | 6.00 | 3298 | 0.3403 | 0.9118 | 0 | — | — |
| 3x3 | static | 9.00 | 3476 | 0.3387 | 0.8610 | 0 | — | — |
| 4x3 | static | 12.00 | 3903 | 0.3578 | 0.8101 | 0 | — | — |
| 6x4 | static | 24.00 | 5747 | 0.4366 | 0.6713 | 0 | — | — |
| 8x6 | static | 48.00 | 7043 | 0.5266 | 0.6608 | 0 | — | — |
| dynamic | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.2802 |
| dynamic+asahi | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.2802 |
| dynamic+altitude_zoom | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.2802 |
| 12x9 | static | 108.00 | 8837 | 1.0000 | 1.0000 | 0 | — | — |
