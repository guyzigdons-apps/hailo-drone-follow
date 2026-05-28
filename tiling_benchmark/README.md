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

## FOV emulation (source-data prep for the paper-reported ablation table)

The paper-reported ablation rows are evaluated at three emulated horizontal FOVs — 70° (native), 60°, and 50° — all rendered to 4K (3840×2160). Each FOV variant is a separate physical MP4 file alongside the rotation-stripped source, so the inference pipeline sees only the claimed resolution at runtime.

### FOV → crop dimensions

Crop dimensions are derived from the source's native FOV (70° on the DJI Mavic 4 Pro main camera, 6016×3384) via:

```
crop_ratio = tan(fov_target / 2) / tan(70° / 2)
crop_w = round(6016 * crop_ratio)
crop_h = round(3384 * crop_ratio)
```

| FOV  | Crop from 6K        | Output resolution | Scale factor (downscale) |
|------|---------------------|-------------------|--------------------------|
| 70°  | full 6016 × 3384    | 3840 × 2160       | 1.57× down               |
| 60°  | center 4963 × 2792  | 3840 × 2160       | 1.29× down               |
| 50°  | center 4007 × 2254  | 3840 × 2160       | 1.04× down (no upscale)  |

### ffmpeg recipe (spec §8.3)

For each FOV, the canonical ffmpeg invocation is:

```bash
# FOV-70 (no crop):
ffmpeg -y -i clip_prepared.MP4 \
    -vf "scale=3840:2160:flags=lanczos" \
    -c:v libx265 -crf 18 -preset slow -an \
    clip_prepared__fov70.mp4

# FOV-60 / FOV-50 (center crop, then scale):
ffmpeg -y -i clip_prepared.MP4 \
    -vf "crop=W:H:(in_w-W)/2:(in_h-H)/2,scale=3840:2160:flags=lanczos" \
    -c:v libx265 -crf 18 -preset slow -an \
    clip_prepared__fov<N>.mp4
```

Where `W` and `H` come from the table above. `crf 18 -preset slow` is the spec's quality target — the emulation must not introduce codec artifacts that confound the FOV ablation.

### Reproducibility

To regenerate the FOV variants for any prepared clip:

```bash
python tiling_benchmark/prepare_video.py /path/to/clip.MP4 \
    --emit-fov 70,60,50
```

Outputs land alongside the prepared clip:
- `clip_prepared__fov70.mp4`
- `clip_prepared__fov60.mp4`
- `clip_prepared__fov50.mp4`
- `fov_variants_manifest.json` (appended atomically; one record per variant)

To match the overnight prep agent's environmental settings (nice + ionice for politeness):

```bash
python tiling_benchmark/prepare_video.py clip.MP4 \
    --emit-fov 70,60,50 \
    --nice 10 --ionice 3
```

### Manifest schema

`fov_variants_manifest.json` is a JSON array of records. Each record describes one variant:

```json
{
  "input":        "clip_prepared.MP4",
  "variant":      "fov70",
  "output":       "clip_prepared__fov70.mp4",
  "output_bytes": 97821909,
  "sha256":       "00c7f232…",
  "ffmpeg_cmd":   "ffmpeg -y -i clip_prepared.MP4 -vf scale=3840:2160:flags=lanczos -c:v libx265 -crf 18 -preset slow -an clip_prepared__fov70.mp4"
}
```

`input` and `output` are basenames (relative to the manifest's parent directory). `sha256` is the SHA-256 hex digest of the output file's bytes — used as a cache key by the inference cache (`hailo_tiling.cache`, Plan 4).

### Determinism caveat

H.265 (libx265) encoding is **not byte-deterministic across runs** in general (encoder threading, optimisation passes). Two runs of `--emit-fov 70` on the same source may produce MP4s with identical pixel content but different SHA-256s. The overnight agent's outputs are the canonical artifacts for paper-reported runs; in-repo regeneration is functionally equivalent (same pixels, same crop math) but not byte-equivalent. Plan 4's cache layer will use SHA-256 to key entries, so swapping in regenerated variants invalidates that variant's cache.

To verify the on-disk overnight artifacts match the schema, run:

```bash
python -c "
import importlib.util; from pathlib import Path
s = importlib.util.spec_from_file_location('pv','tiling_benchmark/prepare_video.py')
pv = importlib.util.module_from_spec(s); s.loader.exec_module(pv)
recs = pv.import_overnight_manifest(
    Path('/home/giladn/Videos/Drone/Training/fov_variants_manifest.json'),
    verify_sha=True,
)
print(f'{len(recs)} records validated; SHA-256s match on-disk files')
"
```

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
