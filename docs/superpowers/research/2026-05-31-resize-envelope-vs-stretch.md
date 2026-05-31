# Resize envelope vs stretch — detection-quality comparison

**Date:** 2026-05-31
**Task:** Plan `2026-05-31-gst-cache-source-pixel-provenance.md` — Task 7
**Chip:** HAILO10H, FW 5.3.0 (`hailortcli fw-control identify` → `Device Architecture: HAILO10H`)
**Clips:** `/home/giladn/Videos/Drone/Training/DJI_20260528155741_0029_D_prepared__fov50.mp4` (120 frames) and `…__fov70.mp4` (60 frames), both 3840×2160
**HEF:** `/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef` (VGA input 640×480 RGB)
**Grid:** 3×2 equal static grid (6 tiles, no overlap)

## TL;DR — recommendation: **keep `stretch` (status quo)**

`stretch` detects **more** targets than `letterbox` (29 vs 15 dets at fov50; 38 vs 35 at fov70) **and** its
detections back-map to source coordinates **cleanly** — median center offset vs a dense-tile reference is
**0.0004–0.0007** (sub-pixel). `letterbox` back-maps with a **systematic positional error 20–30× larger**
(median center offset **0.0092–0.0136**), large enough on these tiny drone-distance boxes to drop IoU below
0.3 against the reference for **every** letterbox detection. That is the **Task-2 watch item realized**: the
`resize_letterbox` → `scaling_bbox` → aggregator back-mapping is **not** correct end-to-end. **Do not flip the
cropper default to `letterbox`** until that back-mapping bug is fixed.

## Method

The drone-follow app consumes **post-aggregator** (final source-frame) detections, so that is what we tapped —
an `identity` element on the aggregator output pad, reading `HailoROI` detections via the python `hailo`
bindings (same extraction approach as `scripts/cache_gst_replay_gate.py`). Pipeline construction reuses the
canonical cropper subgraph from `tiling_benchmark/tiling_record.py:DYNAMIC_TILE_CROPPER_PIPELINE`
(cropper bypass on `sink_0`, tiles through `videoconvert ! hailonet ! hailofilter` on `sink_1`, rejoined at
`hailotileaggregator flatten-detections=true iou-threshold=0.3`). Score threshold 0.3, `nms-score-threshold=0.3`.

Per FOV variant we ran four configs through that pipeline, all on the same HEF:

| config | what |
|---|---|
| `whole` | untiled — whole frame scaled 3840×2160 → 640×480, single inference |
| `dense_ref` | 6×4 static grid (24 tiles), `resize-mode=stretch` — **pseudo-reference** |
| `stretch` | 3×2 grid, `resize-mode=stretch` (today's default) |
| `letterbox` | 3×2 grid, `resize-mode=letterbox` (the candidate) |

**On the reference.** No labelled GT exists for these clips. The repo's tiling benchmark uses a dense tile grid
(12×9) as a pseudo-GT (`tiling_benchmark/run_pxt_yolov8m.py`); we use a lighter 6×4 grid here for runtime.
The `whole`-frame config is **not** usable as a reference — at 3840×2160 → 640×480 the people (≈0.7–2.5% of
frame width at drone distance) are obliterated by the downscale: it found **4 dets in 120 frames** (fov50) and
**0 in 60** (fov70). This is the whole reason tiling exists, and it is reported below only for context. **The
6×4 dense grid is the reference**; it is a proxy, not ground truth.

Scripts (committed under `tiling_benchmark/`): `resize_quality_probe.py` (runs the 4 configs, dumps per-frame
detections), `resize_quality_analyze.py` (counts / confidence / per-class / recall+precision). Raw outputs and
`analysis.json` under `tiling_benchmark/runs/task7/`.

## Per-mode numbers

### Detection counts & confidence

| FOV | config | frames | total dets | dets/frame | mean conf | median conf | per-class (cls:count) |
|---|---|---:|---:|---:|---:|---:|---|
| fov50 | whole | 120 | 4 | 0.033 | 0.389 | 0.387 | 1:4 |
| fov50 | dense_ref | 120 | 1064 | 8.867 | 0.508 | 0.494 | 1:50, 2:1014 |
| fov50 | **stretch** | 120 | **29** | 0.242 | 0.425 | 0.399 | 1:1, 2:28 |
| fov50 | **letterbox** | 120 | **15** | 0.125 | 0.436 | 0.407 | 2:15 |
| fov70 | whole | 60 | 0 | 0.0 | — | — | — |
| fov70 | dense_ref | 60 | 458 | 7.633 | 0.544 | 0.539 | 2:458 |
| fov70 | **stretch** | 60 | **38** | 0.633 | 0.339 | 0.327 | 1:29, 2:9 |
| fov70 | **letterbox** | 60 | **35** | 0.583 | 0.359 | 0.338 | 1:15, 2:20 |

`stretch` finds **more** detections than `letterbox` (≈2× at fov50, +9% at fov70). Confidence distributions are
comparable between the two (means within ±0.02); this is a count/placement difference, not a confidence
difference.

### Recall / precision vs the 6×4 dense reference (greedy per-frame IoU match, same-class)

| FOV | candidate | TP | FN | FP | recall | precision |
|---|---|---:|---:|---:|---:|---:|
| fov50 | stretch | 25 | 1039 | 4 | 0.0235 | **0.862** |
| fov50 | letterbox | 0 | 1064 | 15 | **0.000** | **0.000** |
| fov70 | stretch | 8 | 450 | 30 | 0.0175 | 0.211 |
| fov70 | letterbox | 0 | 458 | 35 | **0.000** | **0.000** |

The absolute recall is low for **both** modes because a 3×2 grid resolves far fewer small targets than the 24-tile
dense reference (≈0.2–0.6 dets/frame vs ≈8) — that gap is the sparse-vs-dense grid effect, **not** the resize-mode
question. The discriminating signal is the **precision** column: `stretch`'s detections land where the reference's
do (86% at fov50), while **none** of `letterbox`'s detections match any reference detection at IoU≥0.3.

## The letterbox back-mapping anomaly (Task-2 watch item)

The 0.0 letterbox precision is not "letterbox finds nothing useful" — its boxes land in the **right region** of the
frame (right side, y≈0.5–0.65, same neighborhood as stretch and the reference). The problem is a **systematic
positional offset** introduced by the letterbox→`scaling_bbox`→aggregator back-mapping.

Two independent pieces of evidence:

**1. Recall climbs as the IoU threshold loosens — but only for letterbox.**

| FOV | mode | recall @0.3 | @0.1 | @0.05 |
|---|---|---:|---:|---:|
| fov50 | stretch | 25 | 25 | 25 |
| fov50 | letterbox | 0 | 1 | 1 |
| fov70 | stretch | 8 | 8 | 8 |
| fov70 | **letterbox** | **0** | **3** | **14** |

`stretch` recall is **flat** across thresholds — when it hits, it hits cleanly (well-aligned), and the misses are
genuinely different targets. `letterbox` recall **climbs** (fov70: 0 → 3 → 14) as the bar drops, the classic
signature of detections that are near the right place but consistently shifted.

**2. Direct center-offset to the nearest same-class reference detection.**

| FOV | mode | n | min offset | median offset | mean offset |
|---|---|---:|---:|---:|---:|
| fov50 | stretch | 28 | 0.0001 | **0.0004** | 0.0044 |
| fov50 | letterbox | 15 | 0.0051 | **0.0136** | 0.0126 |
| fov70 | stretch | 9 | 0.0003 | **0.0007** | 0.0029 |
| fov70 | letterbox | 20 | 0.0083 | **0.0092** | 0.0103 |

`stretch` back-maps to **sub-pixel** accuracy (median 0.0004–0.0007 of frame). `letterbox` is off by a **systematic
~0.009–0.014** — note even its **minimum** offset (0.0051–0.0083) never approaches the reference, i.e. there is no
well-aligned letterbox detection at all. The offset is small in absolute terms (~1% of frame) but fatal to IoU on
these 0.7–1.1%-wide boxes.

This is exactly the risk flagged in the Task-2 review: enabling `resize_letterbox` changes the aggregation
back-mapping math end-to-end, and the `scaling_bbox` transform the letterbox path records does **not** invert the
aspect-preserve+pad operation correctly through the aggregator. The padding offset and/or the per-axis scale of the
letterbox envelope is not being fully removed when tile-local detections are mapped back to source coordinates.

## Decision

**Provisional production default: `stretch` (keep status quo). Do not flip the cropper or writer `resize_mode`
default.**

Reasoning:
1. `stretch` detects **more** targets at the production 3×2 grid in both FOV variants.
2. `stretch` back-maps to source coordinates with **sub-pixel** accuracy; `letterbox` has a **systematic
   back-mapping offset 20–30× larger** that makes its detections unusable for the follow controller (the bbox
   center drives yaw/distance — a 1%-of-frame systematic bias is a real aim error, and the boxes wouldn't survive
   ByteTracker/ReID association either).
3. The theoretical upside of letterbox (no aspect distortion of the non-4:3 tiles) does **not** materialize as more
   or better detections here — it produces **fewer**, and **misplaced**, detections.

### Follow-up required before letterbox could ever be the default

`letterbox` is **not production-ready**. The blocker is a back-mapping bug, not a quality trade-off. Before
reconsidering:
1. **Fix the letterbox `scaling_bbox` back-mapping** in the cropper/aggregator path
   (`hailotilecropper_dynamic` `resize_letterbox` + `hailotileaggregator` consumption of `scaling_bbox`). Acceptance:
   on this same probe, letterbox median center-offset vs the dense reference drops to `stretch`-class (≤0.001) and
   precision vs reference is comparable to stretch.
2. Only **after** that fix, re-run this experiment. If letterbox then matches or beats stretch on count **and**
   placement, flip the default — and at that point also flip the writer's `resize_mode` meta default (Task 4) and
   the cropper property default (Task 2) together.

### What would further sharpen the (already clear) decision

- A labelled GT (vs the dense-tile proxy) would let us report true recall instead of recall-vs-proxy; it would not
  change the placement finding, which is reference-relative-agnostic (the offset is systematic).
- More FOV variants / clips (fov60, other May-28 clips) for breadth — but the back-mapping offset is consistent
  across the two FOVs tested, so the conclusion is robust.

## Reproduce

```bash
source setup_env.sh
OUT=tiling_benchmark/runs/task7
for FOV in fov50 fov70; do
  python tiling_benchmark/resize_quality_probe.py \
    --video "/home/giladn/Videos/Drone/Training/DJI_20260528155741_0029_D_prepared__${FOV}.mp4" \
    --out-dir $OUT --tag $FOV --max-frames 120
done
python tiling_benchmark/resize_quality_analyze.py \
  --in-dir $OUT --tags fov50 fov70 --iou-thr 0.3 --out $OUT/analysis.json
```
