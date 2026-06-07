# Ablation table

- cache: `.tile_cache/DJI_20260528155332_0027_D_prepared__fov50__a2e9861507428064.sqlite3`
- video: `/home/giladn/Videos/Drone/Training/DJI_20260528155332_0027_D_prepared__fov50.mp4`
- source: 3840x2160
- frames: 1914
- reference: `12x9` (12x9), IoU>=0.5

| config | kind | mean_tiles_per_frame | n_dets | recall_vs_reference | precision_vs_reference | n_misses |
|---|---|---:|---:|---:|---:|---:|
| 1x1 | static | 1.00 | 1892 | 0.1288 | 0.9482 | 0 |
| 2x2 | static | 4.00 | 5009 | 0.3118 | 0.8666 | 0 |
| 3x2 | static | 6.00 | 6496 | 0.4302 | 0.9221 | 0 |
| 3x3 | static | 9.00 | 6895 | 0.4530 | 0.9149 | 0 |
| 4x3 | static | 12.00 | 7945 | 0.5338 | 0.9356 | 0 |
| 6x4 | static | 24.00 | 9963 | 0.6155 | 0.8602 | 0 |
| 8x6 | static | 48.00 | 10194 | 0.6106 | 0.8424 | 0 |
| 12x9 | static | 108.00 | 15876 | 1.0000 | 1.0000 | 0 |
