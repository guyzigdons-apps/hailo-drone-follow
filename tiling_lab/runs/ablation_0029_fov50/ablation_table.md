# Ablation table

- cache: `.tile_cache/DJI_20260528155741_0029_D_prepared__fov50__a2e9861507428064.sqlite3`
- video: `/home/giladn/Videos/Drone/Training/DJI_20260528155741_0029_D_prepared__fov50.mp4`
- source: 3840x2160
- frames: 636
- reference: `12x9` (12x9), IoU>=0.5

| config | kind | mean_tiles_per_frame | n_dets | recall_vs_reference | precision_vs_reference | n_misses |
|---|---|---:|---:|---:|---:|---:|
| 1x1 | static | 1.00 | 158 | 0.0000 | 0.0000 | 0 |
| 2x2 | static | 4.00 | 38 | 0.0022 | 0.4211 | 0 |
| 3x2 | static | 6.00 | 140 | 0.0182 | 0.9357 | 0 |
| 3x3 | static | 9.00 | 521 | 0.0649 | 0.8983 | 0 |
| 4x3 | static | 12.00 | 1175 | 0.1488 | 0.9132 | 0 |
| 6x4 | static | 24.00 | 4857 | 0.5867 | 0.8713 | 0 |
| 8x6 | static | 48.00 | 6743 | 0.7851 | 0.8398 | 0 |
| 12x9 | static | 108.00 | 7224 | 1.0000 | 1.0000 | 0 |
