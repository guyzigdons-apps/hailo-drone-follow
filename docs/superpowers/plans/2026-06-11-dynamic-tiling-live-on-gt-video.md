# Dynamic Tiling — Live End-to-End on a GT Video — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **Chip/GStreamer tasks (Phase 0, Phase 2) are run by the MANAGER, not delegated to a backgrounding subagent** — per the weekend-run ops lesson "subagent executors die if they background+pause." Pure-Python TDD tasks (Phase 1) go to fresh subagents.

**Goal:** Run the dynamic tile scheduler live in a real GStreamer pipeline on a ground-truth DJI clip on the Hailo chip — discovery grid acquires the target, then budget-aware ROI tiles follow it frame-to-frame.

**Architecture:** A pure-Python `DynamicTilingController` (wrapping the proven `TargetLock` + `TileScheduler` + `BudgetMeter` from the lab) computes a per-frame list of source-pixel crops and serialises them to the normalized `tiles-static` string the community `hailotilecropper_dynamic` element consumes. A thin standalone GStreamer runner (`run_live.py`) feeds a GT `.mp4` through `filesrc → decodebin → hailotilecropper_dynamic → hailonet inference → hailotileaggregator`, and a buffer pad-probe on the aggregator output steps the controller each frame and pushes the next frame's tiles onto the cropper. Everything lives under `tiling_lab/live/` (the only package allowed to import both `drone_follow` and `hailo_tiling`); production app integration is an explicit follow-up.

**Tech Stack:** Python 3.11, GStreamer 1.0 (PyGObject `gi`), HailoRT + `hailotilecropper_dynamic`/`hailotileaggregator` community plugins, `hailo` python bindings, pytest.

---

## Why a standalone runner (not the drone-follow app) for v1

The full `drone-follow` app wires MAVSDK, follow controller and HTTP servers; running it vision-only is possible but entangled. The whole risk here is GStreamer + the dynamic cropper, not the app shell. A standalone runner isolates that risk, is headless-verifiable (saves an overlay `.mkv` + a per-frame tiles JSONL — no human needs to watch a live window), and keeps the new code in `tiling_lab/` where the dependency rules already permit importing `TargetLock`. Promoting the controller into `drone_follow.pipeline_adapter` (which can import only `hailo_tiling`, not `tiling_lab`) requires first lifting `TargetLock` into `hailo_tiling` — out of scope for "get it running on a GT video", filed as a follow-up at the end.

## File Structure

- Create `tiling_lab/live/__init__.py` — package marker.
- Create `tiling_lab/live/tiles_format.py` — pure crop→`tiles-static` serialiser. One responsibility: geometry/string formatting + clamping. (Task 1)
- Create `tiling_lab/live/controller.py` — `DynamicTilingController`: owns `TargetLock` + `TileScheduler` + `BudgetMeter`, exposes `update(persons) -> str` and read-only stats. (Task 2)
- Create `tiling_lab/live/spike_s1.py` — throwaway-but-kept chip probe that determines the working per-frame tile-injection mechanism (property re-read vs HailoROI attach). (Phase 0 / Task 0)
- Create `tiling_lab/live/run_live.py` — the GStreamer runner script + buffer pad-probe wiring the controller to the cropper. (Task 3)
- Create `tiling_lab/tests/test_live_tiles_format.py` — unit tests for the serialiser. (Task 1)
- Create `tiling_lab/tests/test_live_controller.py` — unit tests for the controller against a scripted detection stream. (Task 2)
- Create `tiling_lab/runs/live_s1/S1_FINDING.md` — recorded S1 verdict (Phase 0).
- Create `tiling_lab/runs/live_<clip>/RESULTS.md` — validation results (Task 4).

## Conventions to follow (verified against current code)

- `CropRect` (`hailo_tiling/types.py:10`) is **source pixels** `(x, y, w, h)` top-left corner, plus `mode: str` (`"s"`/`"m"`). The `tiles-static` property wants **normalized [0,1]** `"x,y,w,h[,mode]"` semicolon-joined (verified via `gst-inspect-1.0 hailotilecropper_dynamic`). So the serialiser divides by `src_w/src_h` and clamps.
- `Det` (`hailo_tiling/types.py:40`) is normalized `(cls, score, x, y, w, h)`.
- `TargetLock` (`tiling_lab/harness/target_lock.py:53`): `step(person_dets, *, lock_if_unlocked=False, gt_bbox_norm=None) -> LockState`. With `lock_if_unlocked=True` it auto-locks the **largest activated track** (`target_lock.py:120-124`) — this is our AUTO-mode seed, no GT needed.
- `TileScheduler` (`hailo_tiling/dynamic/scheduler.py:9`): `decide(lock: LockState, frame_idx: int, meter) -> list[CropRect]`.
- `BudgetMeter` (`hailo_tiling/budget.py:14`): `BudgetMeter(budget_inf_per_s, fps, window_s=1.0)`; per frame call `available(frame_idx)` is read by the scheduler internally, and we call `charge(n_tiles, frame_idx)` after `decide`.
- Reference per-frame flow: `tiling_lab/bench/runner.py:run_dynamic_config` (decide → charge → infer → aggregate → lock.step).
- Dynamic-cropper pipeline-string prior art (frozen, copy the shape — do NOT import it): `tiling_benchmark/tiling_record.py:DYNAMIC_TILE_CROPPER_PIPELINE`.
- Always `source setup_env.sh` (or `source ./hailo-apps/venv_hailo_apps/bin/activate`) in EVERY chained chip command — venv state does not persist across Bash calls.
- Commit with **explicit paths** (`git add <path> ...`), never `git add -A` — the working tree has unrelated dirty files (`reid_manager.py` is under a standing do-not-touch rule).

---

## Phase 0 — Spike S1 (MANAGER-RUN, on chip)

### Task 0: Determine the working per-frame tile-injection mechanism

**Files:**
- Create: `tiling_lab/live/__init__.py`
- Create: `tiling_lab/live/spike_s1.py`
- Create: `tiling_lab/runs/live_s1/S1_FINDING.md`

**Goal of the spike:** answer one question — *can we change the crop set every frame on the running pipeline, and via which mechanism?* Two candidate mechanisms, tested in order:
- **(A) Property re-read:** `cropper.set_property("tiles-static", "<new rects>")` mid-stream and confirm the crop set changes per `cropping-period`.
- **(B) Buffer ROI attach:** attach `HailoTileROI` objects to each buffer's `HailoROI` from an upstream `identity signal-handoffs=true` / pad-probe (the plugin doc states static tiles are *appended to dynamic tiles read from the buffer's HailoROI on every frame*).

- [ ] **Step 1: Create the package marker**

`tiling_lab/live/__init__.py`:
```python
"""Live GStreamer integration of the dynamic tile scheduler."""
```

- [ ] **Step 2: Write the spike script**

`tiling_lab/live/spike_s1.py` — builds a minimal pipeline with `hailotilecropper_dynamic`, and a buffer probe on the aggregator src that **alternates the crop set every 30 frames** between a top-left tile and a bottom-right tile, recording where aggregated detections land. If detections track the active tile region, the mechanism works.

```python
"""Spike S1: does hailotilecropper_dynamic honour per-frame tile changes?

Run on chip:  source setup_env.sh && python -m tiling_lab.live.spike_s1 \
                  --video <GT.mp4> --frames 240

Alternates tiles-static between two disjoint regions every 30 frames and logs,
per buffer, the cropping-period buffer count and the x-centroid of aggregated
person detections. If the centroid follows the active tile, mechanism (A)
works. The script also watches the bus for caps-renegotiation errors/stalls.
"""
import argparse
import sys
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib  # noqa: E402
import hailo  # noqa: E402

from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
    SOURCE_PIPELINE, INFERENCE_PIPELINE,
)
from hailo_apps.python.core.common.core import get_resource_path  # HEF/.so resolution

TILE_LEFT = "0.0,0.0,0.4,0.3,s"
TILE_RIGHT = "0.6,0.7,0.4,0.3,s"


def build_pipeline(video, hef, post_so):
    inner = INFERENCE_PIPELINE(hef_path=hef, post_process_so=post_so,
                               name="s1_infer")
    src = SOURCE_PIPELINE(video_source=video, video_width=1280,
                          video_height=720, frame_rate=30, sync=False)
    return (
        f"{src} ! "
        f"queue name=s1_in_q ! "
        f"hailotilecropper_dynamic name=tc internal-offset=true "
        f"tiling-mode=single-scale tiles-static=\"{TILE_LEFT}\" "
        f"hailotileaggregator name=agg flatten-detections=true iou-threshold=0.3 "
        f"tc. ! queue name=s1_bypass_q ! agg.sink_0 "
        f"tc. ! video/x-raw,format=RGB ! {inner} ! agg.sink_1 "
        f"agg. ! queue name=s1_out_q ! fakesink sync=false name=s1_sink"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--frames", type=int, default=240)
    ap.add_argument("--hef", default=None)
    ap.add_argument("--post-so", default=None)
    args = ap.parse_args()

    Gst.init(None)
    # Resolve HEF + post .so the same way the app does if not provided.
    hef = args.hef or get_resource_path("detection", "models")
    post_so = args.post_so or get_resource_path("detection", "so")
    pipeline = Gst.parse_launch(build_pipeline(args.video, hef, post_so))
    cropper = pipeline.get_by_name("tc")
    agg = pipeline.get_by_name("agg")

    state = {"frame": 0, "region": "L"}

    def probe(pad, info):
        buf = info.get_buffer()
        roi = hailo.get_roi_from_buffer(buf)
        dets = roi.get_objects_typed(hailo.HAILO_DETECTION)
        persons = [d for d in dets if d.get_label() == "person"]
        cxs = []
        for d in persons:
            b = d.get_bbox()
            cxs.append(b.xmin() + b.width() / 2.0)
        f = state["frame"]
        # flip region every 30 frames
        if f > 0 and f % 30 == 0:
            state["region"] = "R" if state["region"] == "L" else "L"
            tiles = TILE_RIGHT if state["region"] == "R" else TILE_LEFT
            cropper.set_property("tiles-static", tiles)  # mechanism (A)
            print(f"[S1] frame {f}: set tiles-static -> {state['region']} ({tiles})",
                  flush=True)
        mean_cx = sum(cxs) / len(cxs) if cxs else float("nan")
        print(f"[S1] frame {f} region={state['region']} "
              f"persons={len(persons)} mean_cx={mean_cx:.3f}", flush=True)
        state["frame"] += 1
        if state["frame"] >= args.frames:
            loop.quit()
        return Gst.PadProbeReturn.OK

    agg.get_static_pad("src").add_probe(Gst.PadProbeType.BUFFER, probe)

    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_msg(_bus, msg):
        t = msg.type
        if t == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print(f"[S1][BUS-ERROR] {err}: {dbg}", flush=True)
            loop.quit()
        elif t == Gst.MessageType.EOS:
            print("[S1] EOS", flush=True)
            loop.quit()
        return True

    bus.connect("message", on_msg)
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 3: (MANAGER) Run the spike on a GT clip on chip**

Run (manager, foreground — do NOT background+pause):
```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
source setup_env.sh
VID=$(ls tiling_visualizer_site/web/data/videos/0025_fov50.mp4 \
        tiling_visualizer_site/dist/data/videos/0025_fov50.mp4 2>/dev/null | head -1)
python -m tiling_lab.live.spike_s1 --video "$VID" --frames 240 2>&1 | tee /tmp/s1.log
```
Expected: log lines flip `region=L`→`region=R` every 30 frames; `mean_cx` should cluster low (~<0.4) while region=L and high (~>0.6) while region=R **if mechanism (A) works**. No `[BUS-ERROR]`.

- [ ] **Step 4: Interpret the result and decide the mechanism**

- **If `mean_cx` follows the active region with no bus errors → mechanism (A) works.** Record "A" as `MECHANISM` and proceed; Task 3 uses `cropper.set_property("tiles-static", ...)`.
- **If `mean_cx` does NOT shift (crop set frozen) → mechanism (A) fails.** Extend `spike_s1.py` with mechanism (B): add `identity name=tile_setter signal-handoffs=true` upstream of `tc`, in its `handoff` (or a pad probe) build `hailo.HailoTileROI(...)` objects and attach to the buffer ROI; re-run. If (B) works, record "B"; Task 3 uses the ROI-attach path.
- **If neither works → STOP** and write the C++-extension sub-plan per `.claude/memory/hailotilecropper_dynamic.md` ("property declarations + upstream `prepare_tiles()`, rebuild via `hailo-compile-postprocess install`, clear `~/.cache/gstreamer-1.0/registry.x86_64.bin`"). Do not proceed to Task 3 until a mechanism passes.

- [ ] **Step 5: Record the finding + commit**

Write `tiling_lab/runs/live_s1/S1_FINDING.md` with: the command run, the verdict (`MECHANISM: A|B|FAIL`), a 5-line excerpt of `/tmp/s1.log` showing the centroid shift, and any bus warnings.

```bash
git add tiling_lab/live/__init__.py tiling_lab/live/spike_s1.py \
        tiling_lab/runs/live_s1/S1_FINDING.md
git commit -m "spike(tiling): S1 — verify per-frame tile injection on hailotilecropper_dynamic"
```

---

## Phase 1 — Pure-Python controller (SUBAGENTS, TDD)

### Task 1: `tiles-static` serialiser

**Files:**
- Create: `tiling_lab/live/tiles_format.py`
- Test: `tiling_lab/tests/test_live_tiles_format.py`

- [ ] **Step 1: Write the failing test**

`tiling_lab/tests/test_live_tiles_format.py`:
```python
import math
from hailo_tiling.types import CropRect
from tiling_lab.live.tiles_format import crops_to_tiles_static


def test_empty_crops_returns_empty_string():
    assert crops_to_tiles_static([], 1280, 720) == ""


def test_single_crop_normalized_with_mode():
    # 256x144 px crop at (128,72) in a 1280x720 frame -> 0.1,0.1,0.2,0.2
    c = CropRect(x=128, y=72, w=256, h=144, mode="s")
    out = crops_to_tiles_static([c], 1280, 720)
    assert out == "0.100000,0.100000,0.200000,0.200000,s"


def test_multiple_crops_semicolon_joined():
    c1 = CropRect(x=0, y=0, w=640, h=360, mode="m")
    c2 = CropRect(x=640, y=360, w=640, h=360, mode="s")
    out = crops_to_tiles_static([c1, c2], 1280, 720)
    assert out == "0.000000,0.000000,0.500000,0.500000,m;0.500000,0.500000,0.500000,0.500000,s"


def test_crop_overflowing_frame_is_clamped_into_unit_square():
    # crop runs past the right/bottom edge -> clamp so x+w<=1, y+h<=1
    c = CropRect(x=1152, y=648, w=256, h=144, mode="s")  # x/w=0.9..1.1
    out = crops_to_tiles_static([c], 1280, 720)
    x, y, w, h, mode = out.split(",")
    assert math.isclose(float(x) + float(w), 1.0, abs_tol=1e-6)
    assert math.isclose(float(y) + float(h), 1.0, abs_tol=1e-6)
    assert float(x) >= 0.0 and float(y) >= 0.0


def test_negative_origin_is_clamped_to_zero():
    c = CropRect(x=-64, y=-36, w=256, h=144, mode="s")
    out = crops_to_tiles_static([c], 1280, 720)
    x, y, w, h, _ = out.split(",")
    assert float(x) == 0.0 and float(y) == 0.0
    assert float(w) > 0.0 and float(h) > 0.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && pytest tiling_lab/tests/test_live_tiles_format.py -v`
Expected: FAIL — `ModuleNotFoundError: tiling_lab.live.tiles_format`.

- [ ] **Step 3: Write minimal implementation**

`tiling_lab/live/tiles_format.py`:
```python
"""Serialise scheduler CropRects (source pixels) to the normalized
`tiles-static` string consumed by hailotilecropper_dynamic.

Format per rect: 'x,y,w,h,mode' with x,y,w,h in [0,1]; rects are ';'-joined.
Crops are clamped into the unit square (origin >= 0, origin+extent <= 1) so a
scheduler ROI that ran past a frame edge never produces an invalid tile.
"""
from collections.abc import Sequence

from hailo_tiling.types import CropRect


def _clamp_axis(origin: float, extent: float) -> tuple[float, float]:
    if extent <= 0.0:
        return 0.0, 0.0
    if origin < 0.0:
        origin = 0.0
    if extent > 1.0:
        extent = 1.0
    if origin + extent > 1.0:
        origin = 1.0 - extent
    return origin, extent


def crops_to_tiles_static(crops: Sequence[CropRect], src_w: int,
                          src_h: int) -> str:
    """Return the ';'-joined normalized tiles-static string for `crops`."""
    parts: list[str] = []
    for c in crops:
        nx, nw = _clamp_axis(c.x / src_w, c.w / src_w)
        ny, nh = _clamp_axis(c.y / src_h, c.h / src_h)
        if nw <= 0.0 or nh <= 0.0:
            continue
        mode = c.mode or "s"
        parts.append(f"{nx:.6f},{ny:.6f},{nw:.6f},{nh:.6f},{mode}")
    return ";".join(parts)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && pytest tiling_lab/tests/test_live_tiles_format.py -v`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
git add tiling_lab/live/tiles_format.py tiling_lab/tests/test_live_tiles_format.py
git commit -m "feat(tiling): crops_to_tiles_static serialiser for dynamic cropper"
```

### Task 2: `DynamicTilingController`

**Files:**
- Create: `tiling_lab/live/controller.py`
- Test: `tiling_lab/tests/test_live_controller.py`

- [ ] **Step 1: Write the failing test**

`tiling_lab/tests/test_live_controller.py`:
```python
from hailo_tiling.types import Det
from tiling_lab.live.controller import DynamicTilingController


def _person(cx, cy, w=0.05, h=0.12, score=0.9):
    # Det stores top-left x,y normalized
    return Det(cls=0, score=score, x=cx - w / 2, y=cy - h / 2, w=w, h=h)


def test_first_update_returns_a_string():
    ctrl = DynamicTilingController(src_w=1280, src_h=720, fps=30.0,
                                   budget_inf_per_s=40.0)
    out = ctrl.update([])
    assert isinstance(out, str)


def test_acquires_then_follows_target():
    # Feed a steadily-present person; after enough frames the controller should
    # be TRACKING and emit at least one tile (the ROI) on most frames.
    ctrl = DynamicTilingController(src_w=1280, src_h=720, fps=30.0,
                                   budget_inf_per_s=60.0)
    last = ""
    for f in range(60):
        last = ctrl.update([_person(0.5, 0.5)])
    assert ctrl.status == "TRACKING"
    assert last != ""  # tiles emitted while tracking
    # every emitted rect is a well-formed normalized 5-tuple
    for rect in last.split(";"):
        x, y, w, h, mode = rect.split(",")
        assert 0.0 <= float(x) <= 1.0 and 0.0 <= float(y) <= 1.0
        assert float(w) > 0.0 and float(h) > 0.0
        assert mode in ("s", "m")


def test_budget_is_charged_and_tiles_per_frame_tracked():
    ctrl = DynamicTilingController(src_w=1280, src_h=720, fps=30.0,
                                   budget_inf_per_s=60.0)
    for f in range(30):
        ctrl.update([_person(0.5, 0.5)])
    # mean tiles/frame for a single steady target should be modest (< grid size)
    assert 0.0 < ctrl.mean_tiles_per_frame < 10.0
    assert ctrl.frame_count == 30


def test_no_detections_eventually_reports_searching_or_lost():
    ctrl = DynamicTilingController(src_w=1280, src_h=720, fps=30.0,
                                   budget_inf_per_s=60.0)
    for f in range(10):
        ctrl.update([_person(0.5, 0.5)])  # acquire
    for f in range(5):
        ctrl.update([])  # target gone
    assert ctrl.status in ("SEARCHING", "LOST")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `source setup_env.sh && pytest tiling_lab/tests/test_live_controller.py -v`
Expected: FAIL — `ModuleNotFoundError: tiling_lab.live.controller`.

- [ ] **Step 3: Write minimal implementation**

`tiling_lab/live/controller.py`:
```python
"""Per-frame dynamic-tiling controller for the live GStreamer pipeline.

Wraps the proven lab components:
  - TargetLock        (auto-locks the largest person, reuses prod ByteTracker)
  - TileScheduler     (discovery grid + ROI follow + recovery, budget-trimmed)
  - BudgetMeter       (sliding-window inference-rate cap)

`update(persons)` is called once per frame with the person detections from the
aggregator (normalized Det). It steps the lock, asks the scheduler for the next
crop set, charges the budget, and returns the crops as a `tiles-static` string
ready to push onto hailotilecropper_dynamic.
"""
from collections.abc import Sequence

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.dynamic.scheduler import TileScheduler
from hailo_tiling.types import Det
from tiling_lab.harness.target_lock import TargetLock

from tiling_lab.live.tiles_format import crops_to_tiles_static


class DynamicTilingController:
    def __init__(self, src_w: int, src_h: int, *, fps: float = 30.0,
                 budget_inf_per_s: float = 60.0, track_buffer: int = 90,
                 scheduler_kwargs: dict | None = None):
        self.src_w = int(src_w)
        self.src_h = int(src_h)
        self._sched = TileScheduler(self.src_w, self.src_h,
                                    **(scheduler_kwargs or {}))
        self._lock = TargetLock(track_buffer=track_buffer)
        self._meter = BudgetMeter(budget_inf_per_s=float(budget_inf_per_s),
                                  fps=float(fps))
        self._frame = 0
        self._total_tiles = 0

    def update(self, persons: Sequence[Det]) -> str:
        """Step one frame; return the tiles-static string for the next frame."""
        self._lock.step(list(persons), lock_if_unlocked=True)
        crops = self._sched.decide(self._lock.state, self._frame, self._meter)
        self._meter.charge(len(crops), self._frame)
        self._total_tiles += len(crops)
        self._frame += 1
        return crops_to_tiles_static(crops, self.src_w, self.src_h)

    @property
    def status(self) -> str:
        return self._lock.state.status

    @property
    def frame_count(self) -> int:
        return self._frame

    @property
    def mean_tiles_per_frame(self) -> float:
        return self._total_tiles / self._frame if self._frame else 0.0
```

- [ ] **Step 4: Run test to verify it passes**

Run: `source setup_env.sh && pytest tiling_lab/tests/test_live_controller.py -v`
Expected: PASS (4 tests). If `test_acquires_then_follows_target` shows `status != TRACKING`, the default `discovery_period` may exceed the warm-up loop — raise the loop to 90 frames OR pass `scheduler_kwargs={"discovery_period": 5}` in the test; do NOT weaken the assertion.

- [ ] **Step 5: Run the full lab suite to confirm no regression**

Run: `source setup_env.sh && pytest tiling_lab/tests hailo_tiling/tests -q 2>&1 | tail -15`
Expected: previous green floor + the new tests, no failures.

- [ ] **Step 6: Commit**

```bash
git add tiling_lab/live/controller.py tiling_lab/tests/test_live_controller.py
git commit -m "feat(tiling): DynamicTilingController wrapping lock+scheduler+budget"
```

---

## Phase 2 — Live runner (MANAGER-RUN, on chip)

### Task 3: GStreamer runner wiring the controller to the cropper

**Files:**
- Create: `tiling_lab/live/run_live.py`

**Pre-req:** Task 0 recorded `MECHANISM: A` (property set) or `B` (ROI attach). The code below is the **mechanism-A** form (cropper.set_property each frame). If S1 recorded B, replace the `_apply_tiles` body with the ROI-attach path proven in `spike_s1.py` (build `hailo.HailoTileROI` from the normalized rects and attach to the buffer ROI in the probe) and drop the `set_property` call.

- [ ] **Step 1: Write the runner**

`tiling_lab/live/run_live.py`:
```python
"""Run dynamic tiling live on a video through the real GStreamer pipeline.

  source setup_env.sh
  python -m tiling_lab.live.run_live --video <GT.mp4> \
         --out tiling_lab/runs/live_0025_fov50

Pipeline: filesrc -> decodebin -> hailotilecropper_dynamic -> hailonet infer
-> hailotileaggregator. A buffer probe on the aggregator src steps the
DynamicTilingController with this frame's person detections and pushes the
returned tiles-static string onto the cropper for the NEXT frame (one-frame
latency, by design). Saves an overlay .mkv and a per-frame tiles JSONL so the
run is verifiable headless.
"""
import argparse
import json
import os
import sys
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib  # noqa: E402
import hailo  # noqa: E402

from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
    SOURCE_PIPELINE, INFERENCE_PIPELINE,
)
from hailo_apps.python.core.common.core import get_resource_path

from hailo_tiling.types import Det
from tiling_lab.live.controller import DynamicTilingController

# Discovery grid seeds frame 0 before any detection exists.
INITIAL_TILES = "0.0,0.0,0.5,0.6,s;0.5,0.0,0.5,0.6,s;0.0,0.4,0.5,0.6,s;0.5,0.4,0.5,0.6,s"


def build_pipeline(video, hef, post_so, w, h, fps, out_mkv):
    inner = INFERENCE_PIPELINE(hef_path=hef, post_process_so=post_so,
                               name="live_infer")
    src = SOURCE_PIPELINE(video_source=video, video_width=w, video_height=h,
                          frame_rate=fps, sync=False)
    return (
        f"{src} ! queue name=live_in_q ! "
        f"hailotilecropper_dynamic name=tc internal-offset=true "
        f"tiling-mode=single-scale tiles-static=\"{INITIAL_TILES}\" "
        f"hailotileaggregator name=agg flatten-detections=true iou-threshold=0.3 "
        f"tc. ! queue name=live_bypass_q ! agg.sink_0 "
        f"tc. ! video/x-raw,format=RGB ! {inner} ! agg.sink_1 "
        f"agg. ! queue name=live_out_q ! hailooverlay_community ! "
        f"videoconvert ! x264enc tune=zerolatency bitrate=5000 ! "
        f"matroskamux ! filesink location={out_mkv} sync=false"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--out", required=True, help="output run dir")
    ap.add_argument("--frames", type=int, default=0, help="0 = whole clip")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--budget", type=float, default=60.0,
                    help="inference budget, tiles/sec")
    ap.add_argument("--hef", default=None)
    ap.add_argument("--post-so", default=None)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    out_mkv = os.path.join(args.out, "overlay.mkv")
    tiles_jsonl = open(os.path.join(args.out, "tiles.jsonl"), "w")

    Gst.init(None)
    hef = args.hef or get_resource_path("detection", "models")
    post_so = args.post_so or get_resource_path("detection", "so")
    pipeline = Gst.parse_launch(
        build_pipeline(args.video, hef, post_so, args.width, args.height,
                       args.fps, out_mkv))
    cropper = pipeline.get_by_name("tc")
    agg = pipeline.get_by_name("agg")

    ctrl = DynamicTilingController(src_w=args.width, src_h=args.height,
                                   fps=args.fps, budget_inf_per_s=args.budget)
    state = {"frame": 0}

    def probe(pad, info):
        buf = info.get_buffer()
        roi = hailo.get_roi_from_buffer(buf)
        dets = roi.get_objects_typed(hailo.HAILO_DETECTION)
        persons = []
        for d in dets:
            if d.get_label() != "person":
                continue
            b = d.get_bbox()
            persons.append(Det(cls=0, score=d.get_confidence(),
                               x=b.xmin(), y=b.ymin(),
                               w=b.width(), h=b.height()))
        tiles = ctrl.update(persons)
        cropper.set_property("tiles-static", tiles or INITIAL_TILES)  # mechanism A
        f = state["frame"]
        tiles_jsonl.write(json.dumps({
            "frame": f, "status": ctrl.status, "n_persons": len(persons),
            "n_tiles": tiles.count(";") + 1 if tiles else 0, "tiles": tiles,
        }) + "\n")
        if f % 30 == 0:
            print(f"[live] frame {f} status={ctrl.status} "
                  f"persons={len(persons)} mean_t/f={ctrl.mean_tiles_per_frame:.2f}",
                  flush=True)
        state["frame"] += 1
        if args.frames and state["frame"] >= args.frames:
            loop.quit()
        return Gst.PadProbeReturn.OK

    agg.get_static_pad("src").add_probe(Gst.PadProbeType.BUFFER, probe)

    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_msg(_bus, msg):
        if msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print(f"[live][BUS-ERROR] {err}: {dbg}", flush=True)
            loop.quit()
        elif msg.type == Gst.MessageType.EOS:
            print("[live] EOS", flush=True)
            loop.quit()
        return True

    bus.connect("message", on_msg)
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)
        tiles_jsonl.close()
        print(f"[live] wrote {out_mkv} and tiles.jsonl in {args.out}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: (MANAGER) Smoke-run 240 frames on chip**

Run (manager, foreground):
```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow && source setup_env.sh
VID=$(ls tiling_visualizer_site/web/data/videos/0025_fov50.mp4 \
        tiling_visualizer_site/dist/data/videos/0025_fov50.mp4 2>/dev/null | head -1)
python -m tiling_lab.live.run_live --video "$VID" \
       --out tiling_lab/runs/live_0025_fov50 --frames 240 2>&1 | tee /tmp/live_smoke.log
```
Expected: no `[BUS-ERROR]`; status transitions `LOST/SEARCHING → TRACKING` within the first discovery cycles; `tiles.jsonl` has 240 rows; `overlay.mkv` is non-empty. If caps errors appear at the `video/x-raw,format=RGB` junction, that's the known dynamic-cropper caps issue — the capsfilter is already pinned per the prior-art note; if it still fails, insert `hailooverlay_community` only after agg (already done) and confirm `videoconvert` precedes the encoder.

- [ ] **Step 3: Commit the runner**

```bash
git add tiling_lab/live/run_live.py
git commit -m "feat(tiling): live GStreamer runner driving dynamic cropper from controller"
```

### Task 4: Full-clip validation + results

**Files:**
- Create: `tiling_lab/runs/live_0025_fov50/RESULTS.md`

- [ ] **Step 1: (MANAGER) Run the whole clip on chip**

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow && source setup_env.sh
VID=$(ls tiling_visualizer_site/web/data/videos/0025_fov50.mp4 \
        tiling_visualizer_site/dist/data/videos/0025_fov50.mp4 2>/dev/null | head -1)
python -m tiling_lab.live.run_live --video "$VID" \
       --out tiling_lab/runs/live_0025_fov50 2>&1 | tee /tmp/live_full.log
```

- [ ] **Step 2: (MANAGER) Compute headline stats from tiles.jsonl**

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow && source setup_env.sh
python - <<'PY'
import json
rows = [json.loads(l) for l in open("tiling_lab/runs/live_0025_fov50/tiles.jsonl")]
n = len(rows)
tracking = sum(r["status"] == "TRACKING" for r in rows)
mean_tiles = sum(r["n_tiles"] for r in rows) / n
print(f"frames={n} tracking_frac={tracking/n:.3f} mean_tiles/frame={mean_tiles:.2f}")
PY
```
Expected (sanity, not a hard gate — live ByteTracker + real detector differ from the cached lab replay): tracking fraction materially > 0 and well above a single static-grid-only run, mean tiles/frame in the low single digits (the lab single-target win was ~1.3 t/f @ ~0.93 recall). Large divergence → investigate (detector confidence, discovery cadence), do not paper over.

- [ ] **Step 3: Visual spot-check the overlay (headless-safe)**

```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow
ffmpeg -y -i tiling_lab/runs/live_0025_fov50/overlay.mkv \
       -vf "select='not(mod(n,60))'" -vsync vfr \
       tiling_lab/runs/live_0025_fov50/frame_%03d.png 2>/dev/null
ls tiling_lab/runs/live_0025_fov50/frame_*.png
```
Then Read a couple of the PNGs to confirm the overlay shows the target boxed and the ROI following it. (The manager inspects these images directly.)

- [ ] **Step 4: Write RESULTS.md + commit**

`tiling_lab/runs/live_0025_fov50/RESULTS.md`: command, S1 mechanism used, the headline stats line, 2-3 embedded frame observations, and any caveats (one-frame latency, detector-vs-GT differences). Then:
```bash
git add tiling_lab/runs/live_0025_fov50/RESULTS.md
git commit -m "docs(tiling): live dynamic-tiling validation results on 0025_fov50"
```

---

## Follow-ups (out of scope here — note, do not implement)

1. **Promote into the drone-follow app.** Move/duplicate `DynamicTilingController` into `drone_follow.pipeline_adapter` behind a `--dynamic-tiling` flag, swapping the static `TILE_CROPPER_PIPELINE` for the dynamic cropper and driving it from the existing `_app_callback_inner` (`hailo_drone_detection_manager.py:261`). Requires lifting `TargetLock` from `tiling_lab.harness` into `hailo_tiling` first (drone_follow may not import tiling_lab — architecture test).
2. **Multi-target.** Swap `TileScheduler`+`TargetLock` for `MultiTargetTileScheduler`+`MultiTargetLock` once single-target follow is proven live.
3. **Flight precondition.** The prod-shared ByteTracker dedup fix `e01fc4f` still needs a sim/flight sanity pass before any of this flies (not needed for the offline video run).

---

## Self-Review

- **Spec coverage:** The restructure spec's Phase-2 intent (scheduler in `hailo_tiling.dynamic` → `tiles-static` on `hailotilecropper_dynamic`, ReID out of scope) is covered by Tasks 2–3; the spec's gate **spike S1** is Task 0 with both candidate mechanisms and the C++ fallback. The user's explicit goal ("end-to-end pipeline on a GT video") is Tasks 3–4.
- **Placeholder scan:** No TBD/"handle edge cases"/"similar to" — every code step is complete. Mechanism-B branch is described as a concrete edit, not a placeholder, and is gated behind an S1 result.
- **Type consistency:** `crops_to_tiles_static(crops, src_w, src_h)` signature is identical in Task 1 def, its tests, and the Task 2/3 call sites. `DynamicTilingController.update(persons) -> str` and the `.status`/`.frame_count`/`.mean_tiles_per_frame` properties match between Task 2 def, its tests, and the Task 3 runner. `Det(cls, score, x, y, w, h)` and `CropRect(x, y, w, h, mode)` match `hailo_tiling/types.py`. `TargetLock.step(..., lock_if_unlocked=True)` and `TileScheduler.decide(lock, frame_idx, meter)` match the verified signatures.
