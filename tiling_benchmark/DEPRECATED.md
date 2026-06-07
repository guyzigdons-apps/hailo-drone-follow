# tiling_benchmark — DEPRECATED (frozen 2026-06-07)

This package is **frozen**. Nothing in the live codebase imports it.

It was the original tiling-benchmark sandbox. The tiling-lab restructure
(2026-06-07) moved every still-used module out into proper packages and left
this directory behind only for reference by the legacy `pxt`/`zoom`/`upscale`
runners that nobody depends on anymore.

## Live homes (where the code went)

| Was here | Now lives in |
|---|---|
| `overlay_viewer.py` + analysis modules (`analyze_pxt.py`, …) | `tiling_lab/viewer/` |
| `prepare_video.py` | `tiling_lab/video/` |
| `_grid_to_static_tiles`, `fov_to_crop_dims` geometry helpers | `hailo_tiling/geometry.py` |
| benchmark harness | `tiling_lab/bench/` |
| dense / static-grid / yolov8m artifacts | `tiling_lab/runs/legacy_dense/` |

## Exception: `analyze_pxt.py`

`tiling_benchmark/analyze_pxt.py` is itself a thin re-export shim that imports
**from** `tiling_lab.viewer.analyze_pxt`. It is kept only so the frozen
`tiling_benchmark/clean_frames.py` (a bare same-dir `from analyze_pxt import …`)
keeps working. This is the single intentional `tiling_benchmark → tiling_lab`
edge; it does not make `tiling_benchmark` a live dependency of anything.

## Lifecycle

`run_pxt_bench.py` here can still regenerate the dense passes if ever needed.
Delete this whole directory once those legacy runners are no longer wanted —
nothing else will break.
