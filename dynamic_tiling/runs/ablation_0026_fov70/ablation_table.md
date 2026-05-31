# Ablation table

- cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov70__a2e9861507428064.sqlite3`
- video: `/home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov70.mp4`
- source: 3840x2160
- frames: 879
- reference: `12x9` (12x9), IoU>=0.5
- dynamic cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov70__dynamic.sqlite3` (per-source-frame; target class 2)
- `recall_delta_at_matched_budget` = dynamic recall − the closest-mean-tiles static grid's recall.

| config | kind | mean_tiles_per_frame | n_dets | recall_vs_reference | precision_vs_reference | n_misses | matched_static | recall_delta_at_matched_budget |
|---|---|---:|---:|---:|---:|---:|---|---:|
| 1x1 | static | 1.00 | 2209 | 0.2636 | 0.8705 | 0 | — | — |
| 2x2 | static | 4.00 | 3750 | 0.4474 | 0.8704 | 0 | — | — |
| 3x2 | static | 6.00 | 4041 | 0.4728 | 0.8535 | 0 | — | — |
| 3x3 | static | 9.00 | 4385 | 0.5079 | 0.8449 | 0 | — | — |
| 4x3 | static | 12.00 | 4132 | 0.5175 | 0.9136 | 0 | — | — |
| 6x4 | static | 24.00 | 4849 | 0.5416 | 0.8148 | 0 | — | — |
| 8x6 | static | 48.00 | 6537 | 0.6092 | 0.6798 | 0 | — | — |
| dynamic | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.2636 |
| dynamic+asahi | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.2636 |
| dynamic+altitude_zoom | dynamic | 0.40 | 0 | 0.0000 | 0.0000 | 0 | 1x1 | -0.2636 |
| 12x9 | static | 108.00 | 7295 | 1.0000 | 1.0000 | 0 | — | — |
