# Ablation table

- cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov60__a2e9861507428064.sqlite3`
- video: `/home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov60.mp4`
- source: 3840x2160
- frames: 879
- reference: `12x9` (12x9), IoU>=0.5

| config | kind | mean_tiles_per_frame | n_dets | recall_vs_reference | precision_vs_reference | n_misses |
|---|---|---:|---:|---:|---:|---:|
| 1x1 | static | 1.00 | 2436 | 0.3212 | 0.9532 | 0 |
| 2x2 | static | 4.00 | 3270 | 0.4037 | 0.8927 | 0 |
| 3x2 | static | 6.00 | 3294 | 0.4203 | 0.9226 | 0 |
| 3x3 | static | 9.00 | 3228 | 0.4102 | 0.9188 | 0 |
| 4x3 | static | 12.00 | 3345 | 0.4141 | 0.8951 | 0 |
| 6x4 | static | 24.00 | 4665 | 0.4548 | 0.7048 | 0 |
| 8x6 | static | 48.00 | 5801 | 0.5465 | 0.6811 | 0 |
| 12x9 | static | 108.00 | 7230 | 1.0000 | 1.0000 | 0 |
