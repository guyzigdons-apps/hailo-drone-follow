# Ablation table

- cache: `.tile_cache/DJI_20260528155239_0026_D_prepared__fov50__a2e9861507428064.sqlite3`
- video: `/home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4`
- source: 3840x2160
- frames: 879
- reference: `12x9` (12x9), IoU>=0.5

| config | kind | mean_tiles_per_frame | n_dets | recall_vs_reference | precision_vs_reference | n_misses |
|---|---|---:|---:|---:|---:|---:|
| 1x1 | static | 1.00 | 2643 | 0.3137 | 0.9368 | 0 |
| 2x2 | static | 4.00 | 3247 | 0.3802 | 0.9242 | 0 |
| 3x2 | static | 6.00 | 3298 | 0.3810 | 0.9118 | 0 |
| 3x3 | static | 9.00 | 3476 | 0.3792 | 0.8615 | 0 |
| 4x3 | static | 12.00 | 3903 | 0.3939 | 0.8096 | 0 |
| 6x4 | static | 24.00 | 5747 | 0.4585 | 0.6573 | 0 |
| 8x6 | static | 48.00 | 7043 | 0.5125 | 0.6335 | 0 |
| 12x9 | static | 108.00 | 8837 | 1.0000 | 1.0000 | 0 |
