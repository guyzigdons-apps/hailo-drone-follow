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
