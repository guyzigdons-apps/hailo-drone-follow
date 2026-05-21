# tiling_benchmark

A pixels-per-target tiling benchmark suite for the Hailo detection pipeline.
Sweeps source resolution × tile grid through the dynamic cropper, scores
recall against a pseudo-ground-truth, and lets you visually compare runs.

## One-shot recipe for a new video

```bash
# 1. Strip rotation metadata from the source so decodebin doesn't squash it.
python tiling_benchmark/prepare_video.py /path/to/new.MP4

# 2. Sweep the experiment matrix. Source dims auto-detected via ffprobe.
python tiling_benchmark/run_pxt_bench.py --video /path/to/new_prepared.MP4

# 3. Inspect detections frame-by-frame against the pseudo-GT.
python tiling_benchmark/overlay_viewer.py --video /path/to/new_prepared.MP4 \
    --frames tiling_benchmark/pxt_runs/pxt_GT-12x9-25.frames.json:GT \
    --frames tiling_benchmark/pxt_runs/pxt_4x3-native.frames.json:4x3
```

If the input video has no rotation metadata, `prepare_video.py` exits cleanly
without re-encoding; pass `--copy` if you want a canonical "prepared" path
anyway.

## What each script does

### `prepare_video.py`
`python tiling_benchmark/prepare_video.py INPUT [--output PATH] [--force] [--copy] [--verify]`

Detects rotation metadata (`tags.rotate`, Display Matrix side-data) and, if
present, re-encodes via `ffmpeg -c:v libx265 -crf 22 -preset fast
-metadata:s:v rotate=0 -an`, baking rotation into pixels and dropping audio.
Idempotent — skips when cached output is newer than the input. **Use this
when** ingesting any new DJI / phone clip before the bench, or when you see
horizontally-stretched frames in `overlay_viewer`.

### `run_pxt_bench.py`
`python tiling_benchmark/run_pxt_bench.py [--video PATH] [--out-dir DIR] [--only LABEL...] [--skip-existing] [--skip-analyze]`

Driver that runs each `(source resolution × tile grid)` config in `CONFIGS`
through `tiling_bench.py`, then invokes `analyze_pxt.py` against the
`GT-12x9-25` pseudo-ground-truth. Probes the video at startup and
substitutes native dims into every "native" config; tile counts stay fixed.
**Use this when** you want a full sweep on a new video.

### `tiling_bench.py`
`python tiling_benchmark/tiling_bench.py --input file://... --tiles-x N --tiles-y M --width W --height H --bench-output PATH`

The single-run harness that `run_pxt_bench.py` shells out to. Builds the
dynamic-cropper GStreamer pipeline, runs detection, and emits a summary JSON
plus a per-frame normalized detections JSON. **Use this when** you want to
re-run one config in isolation or experiment with extra flags.

### `analyze_pxt.py`
`python tiling_benchmark/analyze_pxt.py --gt GT.frames.json --pred A.frames.json --pred B.frames.json --video-w W --video-h H --out analysis.json`

Computes size-binned recall and per-config metrics by matching pred boxes to
the pseudo-GT via IoU. **Use this when** you want headline numbers across
configs.

### `overlay_viewer.py`
`python tiling_benchmark/overlay_viewer.py --video PATH --frames RUN.frames.json:LABEL [--frames ...]`

Interactive frame-by-frame viewer that overlays detection boxes from one or
more runs onto the decoded source. Uses a preview cache under
`pxt_runs/.cache/` so repeated launches are fast. **Use this when**
visually comparing what each config saw.

### `overlay_dets.py`
`python tiling_benchmark/overlay_dets.py --video PATH --frames RUN.frames.json --out RUN.mp4`

Batch overlay — bakes detection boxes into an output video for offline
review or sharing. **Use this when** you want to scrub through a single
run's detections without running the viewer.

## Outputs

Everything lands in `tiling_benchmark/pxt_runs/`:

- `pxt_<label>.json` — headline summary: total frames, frames-with-detection
  percent, mean detections per frame, mean confidence.
- `pxt_<label>.frames.json` — per-frame normalized detection boxes
  (`[x0, y0, x1, y1]` in 0..1). Consumed by `analyze_pxt.py` and
  `overlay_viewer.py`.
- `pxt_analysis.json` — output of `analyze_pxt.py`: size-binned recall for
  each non-GT config against `pxt_GT-12x9-25.frames.json`.
- `pxt_runs/.cache/` — preview cache for `overlay_viewer.py`. Populated
  automatically on first viewer launch.

## Why GT runs at native source resolution

The pseudo-ground-truth uses 12×9 tiles with 25% overlap because that
produces source-pixel tile sizes (~650×484) that closely match the model's
native 640×480 input on a 6016×3384 source — minimizing scale distortion
inside each tile, maximizing detection sensitivity. For sources of other
dimensions, the tile *counts* stay fixed (12×9) so the matrix stays
comparable, but the source-pixel *tile size* scales with the source. See
the plan doc for the tradeoff.

## Knobs

Common `run_pxt_bench.py` flags:

| Flag | Description |
|---|---|
| `--video PATH` | Override the input video (default: hardcoded rotated clip). |
| `--out-dir DIR` | Where summary + per-frame JSONs land. |
| `--only LABEL [...]` | Run a subset of configs by label. |
| `--skip-existing` | Skip configs whose output JSON is already on disk. |
| `--skip-analyze` | Don't invoke `analyze_pxt.py` at the end. |

Unknown args are forwarded to `tiling_bench.py` so you can pass e.g.
`--max-frames 100` to all configs.

## Limitations

- `1x1-640x480` has crashed with SIGSEGV in past runs at the
  same-as-model source resolution. If it fails again, drop it from the
  matrix with `--only` and proceed.
- `4x3-native+full` has an open bug: the `--include-full-frame` extra
  rectangle isn't actually contributing detections (output byte-identical
  to `4x3-native`). The dynamic-cropper enumeration path needs a
  follow-up fix.
- Cross-network analysis (Task 8 in the plan) is deferred and not
  currently scripted.

## Where to read more

- Plan doc: `docs/tiling-benchmark-plan.md` (root).
- Cropper memo: `.claude/memory/hailotilecropper_dynamic.md` — community
  cropper plugin properties + build/install gotchas.
