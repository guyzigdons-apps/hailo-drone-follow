# Ablation table

- cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov70__a2e9861507428064.sqlite3`
- video: `/home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov70.mp4`
- source: 3840x2160
- frames: 879
- reference: `12x9` (12x9), IoU>=0.5

| config | kind | mean_tiles_per_frame | n_dets | recall_vs_reference | precision_vs_reference | n_misses |
|---|---|---:|---:|---:|---:|---:|
| 1x1 | static | 1.00 | 2209 | 0.2867 | 0.8705 | 0 |
| 2x2 | static | 4.00 | 3750 | 0.4867 | 0.8704 | 0 |
| 3x2 | static | 6.00 | 4041 | 0.5142 | 0.8535 | 0 |
| 3x3 | static | 9.00 | 4385 | 0.5524 | 0.8449 | 0 |
| 4x3 | static | 12.00 | 4132 | 0.5628 | 0.9136 | 0 |
| 6x4 | static | 24.00 | 4849 | 0.5867 | 0.8144 | 0 |
| 8x6 | static | 48.00 | 6537 | 0.6477 | 0.6760 | 0 |
| 12x9 | static | 108.00 | 7295 | 1.0000 | 1.0000 | 0 |
