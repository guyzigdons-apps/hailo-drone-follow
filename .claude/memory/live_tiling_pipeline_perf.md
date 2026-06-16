# Live tiling pipeline — performance & shutdown gotchas

Hard-won from building `tiling_lab/live/run_showcase.py` (2026-06-15). The live
GStreamer pipeline is `filesrc → decodebin → hailotilecropper_dynamic →
hailotileaggregator (sink_0 bypass / sink_1 infer) → sink`, with a Python pad
probe that steps a controller and pushes `tiles-static` back to the cropper.

On a laptop with HAILO10H, the components are all individually fast — verify
before blaming inference:
- HEVC 4K decode (`decodebin`): ~208 fps (`filesrc ! decodebin ! videoconvert ! fakesink num-buffers=N`).
- Raw HEF on device: ~352 fps (`hailortcli run2 set-net <hef>`).

So if the assembled pipeline runs at single-digit fps, it is **pipeline wiring**, not the device/decoder/HEF. Three distinct traps, each cost hours:

## 1. Probe on `agg.src` serialises the whole pipeline (~2 fps). THE BIG ONE.
Attaching the buffer probe to the aggregator's **src pad** runs the callback on
the aggregator's streaming thread. Stepping the controller there — and pushing
`tiles-static` back to the *upstream* cropper — stalls aggregation and serialises
the pipeline to ~2 fps, regardless of tile count or how trivial the callback is
(even a no-op probe on `agg.src` is slow; even `get_roi_from_buffer` measuring
0.0 ms still stalls). **Fix:** attach the probe to the src pad of the queue
*downstream* of the aggregator (`agg. ! queue name=out_q ! sink`; probe `out_q`).
The queue decouples the probe onto its own thread; the aggregator keeps
producing. Result: ~2 fps → 65 fps (sustains 4K60). `run_live.py` still probes
`agg.src` and is therefore self-limited (~17 fps with encode) — same fix applies.

## 2. Bare `fakesink` deadlocks after frame 0 (buffer-pool pinning).
`fakesink` defaults to `enable-last-sample=true`, keeping a ref to the last
buffer — which pins one from the aggregator/cropper buffer pool, so the cropper
blocks forever waiting to reuse it. The pipeline deadlocks right after frame 0.
**Fix:** `fakesink sync=false async=false enable-last-sample=false`. (Raw
`filesink` stalls the same way; `x264enc` accidentally avoids it because it
*copies* the buffer, freeing the pool — which is why `run_live`'s encode tail
never hit this.) `hailotileaggregator`'s src pad also advertises no caps, so
caps-querying sinks need care — see `hailotilecropper_dynamic.md`.

## 3. Hailo elements abort during PLAYING→NULL teardown in non-TTY runs.
Under subprocess capture / headless (no TTY), `pipeline.set_state(Gst.State.NULL)`
can abort with `terminate called ... std::logic_error: basic_string::_M_construct
null not valid` → SIGABRT, *after* all useful work, corrupting the exit code and
(if outputs are written in a `finally` after teardown) losing all results. A C++
`abort()` can't be caught in Python. **Fix for measurement tools:** write all
outputs (flushed/closed) BEFORE teardown, then `sys.stdout/err.flush()` +
`os._exit(0)`, bypassing the crashy NULL transition. The OS reclaims the device
handle on exit; back-to-back runs reuse it fine.

## Measuring throughput honestly
A short burst (e.g. `--frames 30`) reads fast because GStreamer pre-buffers
during startup — it reports ~60 fps even when sustained throughput is ~2 fps.
Always measure ≥300 frames for a real number. `run_showcase.py` reports
`achieved_fps` from the first-probe→last-probe window in `metrics.json`.

# Live tiling — DETECTION-CORRECTNESS gotchas (2026-06-16)

The pipeline can run at full fps yet silently drop most detections. Verify the
model first: crop a tile region offline (`hailo_tiling.backends.hef_runtime`
`HefHandle` → `infer` → `decode_nms_output`) and confirm it detects. On 0007 the
road cars detect at conf ~0.88 in a native crop — so a pipeline showing 0 was a
pipeline bug, not a model/scale limit. Two traps, each dropped ~all dense dets:

## 4. Dense tiles tagged multi-scale ("m") get dropped by the aggregator.
`hailotilecropper_dynamic` accepts a per-tile 5th mode field (`x,y,w,h,m|s`).
`m` flags the tile MULTI_SCALE; `hailotileaggregator` then applies cross-scale
suppression — but with no companion scale layer it **discards those detections**.
The single-scale ("s") ROI tile detected fine while the "m" dense grid recorded
0. **Fix:** dense grid tiles must be single-scale `"s"` — each native ~640×480
cell is already model-sized, there is nothing to pyramid. (`StripedDenseScheduler._dense`
now builds with `_grid(...,"s")`.)

## 5. Persistence must be DETECTION-DRIVEN, not schedule-driven (pipeline latency).
A tile's detection arrives at the probe **several frames after** the tile was
inferred (decode+crop+infer+aggregate+queue latency — observed ~10+ frames).
`DetectionPersistence.update(stripe_indices(current_frame), dets)` refilled only
the cells *scheduled this frame*, so a real detection landed in a cell no longer
being swept and was discarded — dropping nearly every dense detection (0007: 50 →
3348 vehicle records over 720 frames after the fix). **Fix:** `update(dets)` keys
off each detection's own `cell_of(det)` (latency-robust) with an age-based TTL
(~2× sweep period) for "saved until the tile's next iteration" + departed-object
expiry. Confirm with a RAW-`all_dets` probe log: the SUV WAS in the aggregator
output, just dropped before `frames.json`.

## 6. overlay_viewer src-px readout must invert the exact draw transform.
The image draws at `canvas = (src - x0)*scale_uniform + off` (clamped viewport
origin + letterbox offset). `_canvas_to_src` had used the *unclamped* viewport
over the *full* canvas with *no* letterbox offset → screen→src wasn't the inverse
of src→canvas, so the readout ran wildly off-range (e.g. -1920..5753) and overlays
drifted. **Fix:** cache `(x0,y0,scale_uniform,off_x,off_y)` in `_render` as
`self._draw_tf` and invert it in `_canvas_to_src`. The black letterbox bars are
cosmetic (16:9 image in a non-16:9 canvas) — correct the alignment, don't clamp.
