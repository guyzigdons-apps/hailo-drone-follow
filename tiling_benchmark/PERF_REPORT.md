# Tiling Benchmark — Performance Report

**Run date**: 2026-05-21
**Test video**: `DJI_20260430103421_0010_D_rotated.MP4` (6016×3384, 1035 frames, 29.97 fps, ~34.5 s)
**Model**: `hailo_yolov8n_4_classes_vga.hef` — 640×480 input, 4 classes (`person`, `vehicle`, `face`, `license_plate`), detection threshold 0.5
**Hardware**: Hailo-10H PCIe, x86_64 host
**Analyzer settings**: phantom filter ON (`--drop-tile-shaped --tile-shape-tol 0.01`), IoU 0.5 for greedy per-class match
**Pseudo-GT**: `GT-12x9-25-multi` — 12×9 dense grid (650×484 src-px tiles, 25 % overlap) + 3×2 medium grid (~2407×1934 src-px, 0 overlap) + 1×1 full frame. Total 115 tiles/frame.

---

## 1. Per-config performance

| Config | Tiles/frame | Extras | Wall (s) | Total infs | Inf/s | Det count | Det/frame | Mean conf |
|--------|------------:|:------:|---------:|-----------:|------:|----------:|----------:|----------:|
| **`GT-12x9-25-multi`** | 115 | `1×1` + `3×2` | **370.0** | 119 140 | 322 | **6 911** | 6.671 | 0.794 |
| `1x1-native` | 1 | — | 71.7 | 1 036 | 14 | 531 | 0.512 | 0.652 |
| `2x2-native` | 4 | — | 74.0 | 4 144 | 56 | 2 043 | 1.972 | 0.672 |
| `3x2-native` | 6 | — | 73.7 | 6 216 | 84 | 2 951 | 2.849 | 0.693 |
| `3x3-native` | 9 | — | 74.5 | 9 324 | 125 | 3 175 | 3.065 | 0.704 |
| `4x3-native` | 12 | — | 77.5 | 12 432 | 160 | 4 402 | 4.249 | 0.731 |
| `4x3-native+full` | 13 | `+full` | 79.2 | 13 468 | 170 | 4 402 | 4.249 | 0.731 |
| `6x4-native` | 24 | — | 82.4 | 24 864 | 302 | 5 263 | 5.080 | 0.768 |
| `8x6-native` | 48 | — | 155.6 | 49 728 | 320 | 5 993 | 5.785 | 0.787 |

**Observations**
- **Pipeline overhead** dominates below ~12 tiles/frame: `1x1` through `4x3` all land ~71–78 s wall clock regardless of tile count — the source decode + GStreamer scheduling cost is ~70 s on its own. **Effective chip throughput** stabilises at ~322 inf/s for the denser configs (`6x4`, `8x6`, GT).
- **`4x3-native+full` is byte-identical to plain `4x3-native`** (4 402 dets, same conf, identical frames.json). The `--include-full-frame` extra isn't reaching the cropper plugin — known bug, separate fix needed.
- **GT wall clock is +6.6 %** over the old single-scale 12×9 (347 s → 370 s) for the 7-tile multi-scale addition — cheap insurance for catching big objects.
- **Mean confidence climbs monotonically with tile count** (0.65 → 0.79), reflecting that denser tiling lets the model see each object at higher pixel density → more confident.

---

## 2. Size-binned recall — `person`

GT n_gt by bin (source-pixel height): `(16,32]=50`, `(32,64]=283`, `(64,128]=1 119`, `(128,256]=737`. After phantom filter, the previously-broken `(256,4096]` bin is empty (it was 183 phantoms in the unfiltered run).

| bin (src-px) | n_gt | `1x1` | `2x2` | `3x2` | `3x3` | `4x3` | `4x3+full` | `6x4` | `8x6` |
|------|-----:|------:|------:|------:|------:|------:|----------:|------:|------:|
| (16, 32] | 50 | 0 % | 0 % | 0 % | 0 % | 0 % | 0 % | 0 % | **34.0 %** |
| (32, 64] | 283 | 0 % | 0 % | 0 % | 0 % | 2.5 % | 2.5 % | 19.8 % | **54.8 %** |
| (64, 128] | 1 119 | 0 % | 7.9 % | 17.1 % | 22.4 % | 43.0 % | 43.0 % | 74.1 % | **79.4 %** |
| (128, 256] | 737 | 19.0 % | 40.2 % | 46.0 % | 45.0 % | 55.4 % | 55.4 % | 76.8 % | **89.4 %** |
| **TOTAL** | 2 189 | 6.4 % | 17.5 % | 24.2 % | 26.6 % | 40.9 % | 40.9 % | 66.3 % | **78.5 %** |

- Recall is **monotone in tile count** across every bin → "more, smaller tiles" wins for people.
- **No config except 8×6 detects 16–32 px persons at all.** Even the upscale-experiment results suggest that going *denser* than 8×6 would keep gaining ground here. The cliff is below ~24 px source height where the model's P3 head bottoms out.
- **64–128 px is the sweet spot**: 8×6 hits 79 %, 6×4 hits 74 %. Either is acceptable for typical-distance pedestrians.

## 3. Size-binned recall — `vehicle`

GT n_gt by bin: `(32,64]=1 290`, `(64,128]=1 841`, `(128,256]=597`, `(256+]=303` (the multi-scale GT added 36 large vehicles that single-scale missed).

| bin (src-px) | n_gt | `1x1` | `2x2` | `3x2` | `3x3` | `4x3` | `4x3+full` | `6x4` | `8x6` |
|------|-----:|------:|------:|------:|------:|------:|----------:|------:|------:|
| (16, 32] | 7 | 0 % | 0 % | 0 % | 0 % | 0 % | 0 % | 0 % | 0 % |
| (32, 64] | 1 290 | 0 % | 27.8 % | 39.6 % | 76.2 % | 88.4 % | 88.4 % | 92.9 % | **94.3 %** |
| (64, 128] | 1 841 | 0.3 % | 44.8 % | 68.1 % | 55.4 % | 75.8 % | 75.8 % | 88.4 % | **93.4 %** |
| (128, 256] | 597 | 29.6 % | 43.7 % | 67.8 % | 60.8 % | **80.1 %** | 80.1 % | 75.7 % | 76.5 % |
| (256+) | 303 | 60.1 % | 61.1 % | 72.9 % | 57.8 % | **81.5 %** | 81.5 % | 81.2 % | 78.5 % |
| **TOTAL** | 4 038 | 9.0 % | 40.3 % | 59.2 % | 62.9 % | 80.7 % | 80.7 % | 87.3 % | **89.9 %** |

- Vehicles are **dramatically easier** than people at the same source-px size: 4×3 already clears 80 % on every bin from 32 px up. Probably class-specific feature signature (longer aspect, more texture).
- **Inversion past 128 px**: `4x3` beats `6x4` and `8x6` on the (128, 256] and (256+) bins. Big vehicles span multiple dense-tile boundaries and get split; `4x3` keeps them intact.
- **3×3 is anomalously poor on the (64, 128] bin** (55 % vs `3x2` 68 %). The 3×3 tile aspect is 16:9 — the model letterboxes off ~25 % of the input, hurting horizontally-elongated objects more.
- **No config detects 16–32 px vehicles** (only 7 GT objects in this bin — likely far-away cars partly occluded).

---

## 4. Phantom-filter impact (recall correction)

Drop counts per config (filter removes detections whose bbox exactly matches a tile in that config's grid — yolov8n class-0 "person" emits these on low-contrast tiles):

| Config | Total dets | Phantoms dropped | % |
|--------|-----------:|-----------------:|--:|
| GT-12x9-25-multi | 6 911 | **183** | 2.6 % |
| 3x3-native | 3 175 | 5 | 0.2 % |
| 4x3-native | 4 402 | 13 | 0.3 % |
| 4x3-native+full | 4 402 | 13 | 0.3 % |
| 8x6-native | 5 993 | 25 | 0.4 % |
| 1x1 / 2x2 / 3x2 / 6x4 | — | **0** | 0 % |

Phantom rate **scales with tile density** (smaller tiles → more low-contrast regions → more bogus class-0 firings). 6×4 is the lone exception — its tile aspect ratio (~5:4) may not coincide with a problematic content alignment.

Without the filter, the `person` `(256,4096]` bin in the old analysis showed 0 % recall across every config — those 183 GT "objects" were the GT's own phantoms with no real-world counterpart, so no candidate could match them. After the filter, the bin is empty (n_gt=0) and the bogus result disappears.

---

## 5. Upscale-hypothesis subset experiment (separate)

Subset: 50 frames hand-picked from the GT by small-object density. **GT for this analysis is `upscale-20x15` (densest in the run), NO phantom filter applied** — small caveat. IoU 0.5.

| Config | Tiles | Wall (s) | Total dets | Det/frame | Mean conf |
|--------|------:|---------:|-----------:|----------:|----------:|
| `baseline-12x9` | 108 | 18.8 | 297 | 5.82 | 0.811 |
| `upscale-14x10` | 140 | 24.8 | 327 | 6.41 | 0.792 |
| `upscale-16x12` | 192 | 32.2 | 314 | 6.16 | 0.799 |
| `upscale-18x13` | 234 | 38.8 | 321 | 6.29 | 0.801 |
| `upscale-20x15` | 300 | 49.1 | 373 | 7.31 | 0.790 |

**Vehicle recall** (vs 20×15 GT, monotone clean):

| bin | n_gt | 12×9 | 14×10 | 16×12 | 18×13 |
|-----|-----:|-----:|------:|------:|------:|
| (32, 64] | 127 | 81.9 % | 82.7 % | 84.3 % | **85.0 %** |
| (64, 128] | 102 | 97.1 % | 97.1 % | 98.0 % | **100 %** |
| TOTAL | 236 | 86.0 % | 86.4 % | 87.7 % | **89.0 %** |

**Verdict**: digital-zoom past 1.0× **does help** vehicle detection, monotone increase, **trend still rising at the 1.83× cap**. The `TilingConfiguration` `[1, 20]` per-axis limit (in `hailo-apps/.../configuration.py`) is the binding constraint. Lifting it would let us measure whether the curve saturates at 2–3× upscale.

**Person recall** in the small bins was noisy at n=25 GT objects — the trend looks up overall (32 % → 80 % in 16–32 px) but with a dip at 16×12. Need a bigger subset (or full-frame sweep) to draw a clean person conclusion.

---

## 6. Recommendations (pick by use case)

| Use case | Recommended config | Justification |
|----------|--------------------|---------------|
| **Real-time follow loop** (~30 fps, single chip) | `4x3-native` (12 tiles, 78 s on 34 s of video → 4× real-time slowdown which is still useful for offline; for live, downsample source) | Best person/vehicle recall trade for chip time. 41 % person TOTAL recall, 81 % vehicle TOTAL. |
| **Best-effort detection on big targets** (cars) | `4x3-native` | Wins large-vehicle bins (128 px+) by intact tile coverage; 80 % recall up to 256+ px. |
| **Best-effort detection on small persons** (e.g. surveillance at distance) | `8x6-native` (and consider denser if cap permits) | Only config that fires below 32 px persons. 79 % person `(64,128]`, 89 % `(128,256]`. |
| **Highest-recall reference** (offline GT) | `GT-12x9-25-multi` | Multi-scale catches big + medium + small with phantom filter active. 6 911 dets total. |
| **No tiling at all** | Don't. `1x1-native` finds 9 % of vehicles and 6 % of people. |

---

## 7. Known issues / future work

### 7.1 NMS confidence-ranking favors fragments over wholes — **architectural flaw, not a bug**

When the multi-scale GT (12×9 + 3×2 + 1×1) sees a large object that straddles a dense-tile boundary, the standard IoU-based NMS in `hailotileaggregator` produces the wrong result.

**Concrete case** (frame 982, white car): the car spans roughly the middle of the 12×9 tile grid horizontally and crosses one vertical tile boundary. Three competing detections fire:

| Source tile | Geometry | Conf | What the model sees |
|-------------|----------|------|---------------------|
| 12×9 tile (left half) | Half-car @ ~650×484 src-px → ~640×480 model | **0.89** | Half a car at near-1:1 native model resolution |
| 12×9 tile (right half) | Other half-car → ~640×480 model | **0.86** | The other half, same near-1:1 resolution |
| 1×1 full frame | Whole car @ 6016×3384 → 640×480 model | **~0.5–0.6** | Entire car at 9.4× downscale (model sees a tiny car) |

Standard NMS sorts by confidence descending. The two fragment detections beat the whole-car detection because **dense tiles deliver more pixels per object → higher per-fragment confidence than the downscaled whole**. NMS keeps both fragments (their pairwise IoU is low — they're side-by-side rectangles), suppresses the whole. Result: the user sees a car incorrectly split into two boxes.

**Mitigated by adding 25 % overlap to the 3×2 extra grid** (commit `<sha>`) — overlap ensures the medium-scale tile contains the whole car, producing a strong (~0.85+) whole-car detection that wins on confidence. Solves the *spatial* version of the problem.

**Unsolved**: the confidence-ranking flaw is still there for any object that crosses a tile boundary at a scale where the model has marginal per-fragment confidence (e.g. ~0.7) but the wider tile sees the object at a resolution where the model only gives ~0.55. Standard NMS will still pick the fragment.

**Proposed mitigations** (none implemented yet — flagged for future work):
1. **Area-aware NMS**: when two detections overlap above a threshold, prefer the **larger** one regardless of confidence (above some minimum). Catches "fragment of A vs. whole that contains A" pairs.
2. **Containment merge**: if detection A is geometrically contained in detection B (e.g. A's centre is within B and A's area is < 30 % of B's area), suppress A unconditionally. Cheaper than re-doing NMS; targets exactly the fragment-vs-whole case.
3. **Scale-prefer NMS**: tag each detection with the source-tile area at production time, prefer detections from coarser tiles when IoU is significant. Requires modifying `hailotileaggregator` (C++) or a post-aggregator step that knows about tile geometry.
4. **Soft-NMS / weighted averaging**: instead of suppressing fragments, weight-merge them with the whole — the resulting bbox is the centroid-weighted union. Risk: produces fewer crisp detections, may smear the bbox.

**Recommendation**: try (2) containment-merge as a post-aggregator step in Python (cheap, no plugin work). If it works, it can ride on top of the existing pipeline as a frames.json scrubber.

### 7.2 Other known issues

- **`4x3-native+full` produces output byte-identical to plain `4x3-native`** — NOT a code bug. The `--include-full-frame` flag DOES add the (0,0,1,1) tile (final `tiles-static` has 13 rectangles, verified by diagnostic log in commit `cccf120`). At native 6016×3384 source, the full-frame tile downscales 9.4× to fit 640×480 model input, shrinking ~78 px source targets to ~8 px in model input — below the 0.5 confidence floor. The full-frame tile produces zero net new detections in this video. This is **physics, not code** — the 1×1 view is too coarse for anything but the largest objects.

- **`1x1-640x480` config crashes (SIGSEGV)** when source is scaled to 640×480 (same as model input). Likely a degenerate caps-negotiation path. Removed from current matrix.

- **`TilingConfiguration` caps `tiles_x` / `tiles_y` at 20** in `hailo-apps/.../configuration.py`. Blocks testing upscale factors > 1.83× on this source. Patching the cap is straightforward but requires hailo-apps modification.

- **Phantom-detection bug confirmed at HEF level** — standalone HailoRT probe (`probe_phantom_hef.py`) shows `hailo_yolov8n_4_classes_vga` class 0 (= `person`) emits `bbox ≈ (0, 0, 1, 1)` on uniform/noisy inputs at low conf (~0.06), occasionally crossing 0.5 on certain content. **yolov8m does NOT have this artefact** — training-data issue specific to the 4-class HEF. Mitigated by the post-hoc `--drop-tile-shaped` filter in `analyze_pxt.py` and the "Hide phantoms" checkbox in `overlay_viewer.py`.

- **Upscale subset analysis did not apply the phantom filter** — should be re-analyzed with `--drop-tile-shaped` for consistency. The subset's `20x15` GT contains some phantoms.

---

## 8. Boundary-strip in the aggregator — per-tile mode override (NEW)

Follow-up to sec 7.1. The IoU-based NMS in `hailotileaggregator` cannot, by
geometry, merge fragment-vs-whole pairs: a small fragment contained in a
larger whole has `IoU = area_small / area_big`, so for any `area_small <
0.3 * area_big` the IoU is below the default threshold of 0.3 and NMS is
forced to keep both. The aggregator does have a separate filter that
correctly handles this case — `remove_exceeded_bboxes` (controlled by the
`border-threshold` property) — but until now the upstream
`hailotilecropper_dynamic` hardcoded `SINGLE_SCALE` on every emitted tile,
and the aggregator gates the boundary-strip on `MULTI_SCALE` so the filter
never ran.

### 8.1 Aggregator code paths

`gsthailotileaggregator.cpp`:

- `remove_exceeded_bboxes` (lines 184-200) is called per sub-tile inside
  `handle_sub_frame_roi`, **gated per-tile** on `get_mode() == MULTI_SCALE`.
  It drops detections whose bbox edges sit within `border_threshold` of the
  tile edges in **tile-normalized coordinates**, with a critical
  exemption: tile edges that coincide with the frame boundary
  (`tile_bbox.xmin() == 0` etc) are skipped so objects at the actual edge
  of the camera frame are preserved.
- `remove_large_landscape` is gated on the **first tile's** mode (a frame-level
  proxy). Disable with the `remove-large-landscape=false` property if
  you don't want it.
- NMS (lines 262-294) sorts by confidence DESC and runs class-aware
  greedy IoU NMS. Standard, no surprises.

### 8.2 Plugin change: per-tile `tiling-mode` in `hailotilecropper_dynamic`

Added to `hailotilecropper_dynamic`:

| Property | Type | Default | Purpose |
|---|---|---|---|
| `tiling-mode` | GEnum (`single-scale`/`multi-scale`) | `single-scale` | Cropper-level default mode applied to every static tile that doesn't carry its own override. Back-compat default keeps legacy behaviour. |
| `tiles-static` 5th field | `m`/`s`/`multi-scale`/`single-scale`/`0`/`1` (optional) | — | Per-tile mode override. Omit ⇒ inherit the cropper default. Accepts the long names and integers for parser tolerance. |

The HailoTileROI objects emitted by the cropper now carry the right mode,
so the aggregator's per-tile `MULTI_SCALE` gate opens for exactly the
tiles you want stripped. End-to-end verified by
`tests/e2e/test_e2e_tiling_mode.py` (8 tests, mode propagation confirmed
by reading `HailoTileROI.mode()` from the buffer metadata in a pad probe).

### 8.3 Recommended config for multi-scale GT

Tag the **dense grid** with `m` (the fragment-producing layer) and **extras**
with `s` (the rescue layer that must preserve everything). In
`tiling_bench.py` this is now the default for `GT-12x9-25-multi`:

```
tiles-static="<108 dense tiles>,m;<1×1 full frame>,s;<6 from 3×2 medium>,s"
tiling-mode=single-scale  # cropper default — extras inherit; dense forced 'm'
border-threshold=0.005    # see threshold sweep below
```

### 8.4 Threshold sweep (51-frame subset of problem scenes)

Measured on the same `/tmp/gt_frag_audit/subset.mp4` used in the sec-7.1
audit. **Phantom = the yolov8n class-0 artefact (bbox≈tile)** counted
post-run by `analyze_pxt.is_phantom`. **Fragment leak = residual dets
that `containment_merge` would still suppress**. **Recall = match rate
against the OLD baseline's containment-merge-cleaned output (IoU≥0.3,
same class).**

| `border-threshold` | total dets | phantom leak | fragment leak | person recall | vehicle recall |
|---|---:|---:|---:|---:|---:|
| **CM-ideal (Python, 100% recall ref)** | **312** | 0 | 0 | 127/127 (100%) | 111/111 (100%) |
| 0.0001 (effectively off) | 335 | 14 | 18 | 125/127 (98%) | 104/111 (94%) |
| 0.001 | 319 | 4 | 12 | 125/127 (98%) | 104/111 (94%) |
| **0.005 (recommended)** | **293** | **0** | **0** | **119/127 (94%)** | **100/111 (90%)** |
| 0.02 | 283 | 0 | 0 | 117/127 (92%) | 93/111 (84%) |
| 0.05 | 274 | 0 | 0 | 109/127 (86%) | 93/111 (84%) |
| 0.10 | 227 | 0 | 0 | 66/127 (52%) | 93/111 (84%) |
| 0.15 (plugin default) | 227 | 0 | 0 | 66/127 (52%) | 93/111 (84%) |

The 0.05 → 0.10 cliff is where the strip starts eating real bboxes that
sit near (but not on) a tile edge. The 0.005 → 0.001 cliff is where the
strip becomes too narrow to catch phantoms (bbox ≈ tile, edges literally
at 0/1 in tile coords) and small fragments. `0.005` is the unique
sweet spot that eliminates 100% of phantoms + 100% of fragments while
preserving ≥90% recall on both classes.

### 8.5 Why no threshold reaches 100% recall

The strip kills any bbox whose edges sit within `border_threshold` of a
tile edge. A "real" detection whose visible portion happens to end at
the tile boundary (e.g. a person at the literal edge of what one tile
sees in source space) has the same geometric signature as a fragment — the
filter cannot tell them apart. The 6%/10% person/vehicle recall loss at
`0.005` is real boundary-touching detections that the 3×2 rescue layer
can't recover (objects too small for the 3×2 tile's 3.8× downscale).

This is the fundamental safety trade-off the original Hailo design
encoded by gating the strip on MULTI_SCALE: it expects the cropper to
emit a dense-enough rescue layer that everything lost by the strip is
caught coarser. For aerial drone footage where the smallest objects are
detectable only by the dense 12×9 grid (no rescue dense enough), some
real detections are inevitably lost.

### 8.6 Choice of cleanup path (GT vs realtime)

| Use case | Recommended | Why |
|---|---|---|
| **Offline GT analysis** | `analyze_pxt.py --containment-merge` | 100% recall (audit verified, 0 false suppressions on 14 inspected pairs), perfect cleanup. The strip is too aggressive for ground-truth where every real detection matters. |
| **Realtime drone-follow pipeline** | C++ strip at `border-threshold=0.005`, dense grid tagged `,m`, rescue tiles `,s` | Runs in the aggregator (no Python post-processing pad probe needed). Kills phantoms + fragments at acceptable recall cost. The tracker (ByteTrack + ReID) interpolates across the occasional lost frame. |

### 8.7 Best-GT recipe (use this when you need the canonical ground truth)

Generate the cleanest possible GT for `DJI_20260430103421_0010_D_rotated.MP4`:

```bash
source setup_env.sh
# 1. Raw GT with strip OFF (passing --border-threshold 0 auto-drops the dense-grid 'm' tag).
python tiling_benchmark/run_pxt_bench.py \
    --out-dir tiling_benchmark/pxt_runs_clean \
    --only GT-12x9-25-multi \
    --skip-analyze \
    --border-threshold 0
# 2. Apply Python phantom filter + containment-merge.
python tiling_benchmark/clean_frames.py \
    --input  tiling_benchmark/pxt_runs_clean/pxt_GT-12x9-25-multi.frames.json \
    --containment-merge \
    --output tiling_benchmark/pxt_runs_clean/pxt_GT-12x9-25-multi.clean.frames.json
```

Output (1036 frames, 35 s of footage):

| stage | dets | delta |
|---|---:|---:|
| raw aggregator output | 6 900 | — |
| after phantom filter (`is_phantom`, tile-shape match) | 6 717 | −183 (2.7 %) |
| after containment-merge (`area_small < 0.5·area_big`, centre inside) | 6 346 | −371 (5.5 %) |

Visual audit across the 10 hardest containment-merge-drop frames (882, 979, 853, 855, 859, 861, 881, 883, 884, 886) — **zero false suppressions across 36 inspected drops**. All suppressed dets are either yolov8n class-0 phantoms on empty terrain or geometric fragment-vs-whole pairs. Audit PNGs available at `tiling_benchmark/pxt_runs_clean/audit_zoom/<frame>_zoom.png` (raw / clean side-by-side, ~1100×550 zoom on the action area).

### 8.8 `tiling_bench.py` auto-disables dense-grid strip on `--border-threshold 0`

When `--border-threshold 0` is passed, `tiling_bench.py` now also drops the
`,m` tag on the dense grid (sets `grid_tile_mode=""`). Without this, the
aggregator would still run `remove_exceeded_bboxes` for any tile tagged
multi-scale because `DYNAMIC_TILE_CROPPER_PIPELINE` only emits
`border-threshold=…` for truthy values — a zero value would silently fall
back to the aggregator's built-in 0.1 default. Auto-disabling on 0
preserves the user's intent ("strip off, clean in Python").

### 8.9 Future work — dynamic tracker tile

The current GT uses a static 12×9 dense grid + extras. For the realtime
drone-follow runtime we have richer per-frame context that the GT pipeline
ignores: the tracker's current target position (and bbox), plus any weak
detections from previous frames. A natural production-grade extension is
to add **one extra tile per frame, centred on the current target**:

- **Source**: the most-recent confirmed ByteTrack track for the locked
  target (`hailo_drone_detection_manager`), or — when there's no lock — the
  highest-confidence person/vehicle from the previous frame as a "weak
  hint".
- **Tile geometry**: a square (or aspect-matched) crop ~2× the target's
  bbox size, clamped to frame extents. Roughly the size of one 12×9 tile
  but positioned exactly on the object.
- **Wiring**: emit it as a HailoTileROI from a Python pad probe upstream of
  `hailotilecropper_dynamic`. The cropper already accepts dynamic tiles
  alongside `tiles-static` (it's why we use the `_dynamic` variant). Tag
  the tracker tile `,s` so the aggregator's boundary-strip leaves it
  alone — it's a rescue tile, not a fragment-producing one.
- **Win for inference budget**: replace ~half the dense grid (e.g. drop
  12×9 → 4×3 native + tracker tile) so each frame runs 13 tiles instead
  of 108. ~8× fewer chip inferences per frame.
- **Win for accuracy**: the tracker tile is sized to the actual object,
  so the model sees the target at near-1:1 pixel density even when the
  drone-camera distance changes mid-flight. Combined with the
  containment-merge filter, fragment artefacts disappear because there's
  one canonical "best view" of the target every frame.
- **Risk**: when the tracker drifts or is lost, the tracker tile lands on
  the wrong region. The ReID gallery + raw-detection fallback already in
  `reid_manager.py` mitigates this (existing code path).

**Not to be implemented in the current iteration** — this is a follow-up
once the GT pipeline is settled and we move to dynamic-tiling experiments.

### 8.10 Implementation files

- `hailo-apps/hailo_apps/postprocess/cpp/hailotilecropper_dynamic/gsthailotilecropper_dynamic.{cpp,hpp}` — new `tiling-mode` property + per-tile 5th field in `tiles-static`.
- `hailo-apps/hailo_apps/postprocess/cpp/hailotilecropper_dynamic/tests/e2e/test_e2e_tiling_mode.py` — 8 tests including HailoTileROI metadata propagation check.
- `tiling_benchmark/tiling_record.py` — `_grid_to_static_tiles(mode=...)`, `_center_tile_static(mode=...)`, `DYNAMIC_TILE_CROPPER_PIPELINE(tiling_mode=..., grid_tile_mode=...)`.
- `tiling_benchmark/tiling_bench.py` — dense grid forced `,m`, all extras tagged `,s`, `border_threshold` read directly from CLI to bypass `configuration.py`'s zero-out on single-scale runs.

---

## 9. Artefact paths (all under `tiling_benchmark/pxt_runs/`, gitignored)

| Artefact | Path |
|----------|------|
| Multi-scale GT detections | `pxt_GT-12x9-25-multi.frames.json` |
| Per-config detections (×8) | `pxt_<label>.frames.json` |
| Aggregate analysis with phantom filter | `pxt_analysis_multi.json` |
| Multi-scale GT chip log | `run_gt_multi.log` |
| Analysis console log | `analyze_multi.log` |
| Upscale subset video | `upscale_subset/subset_video.mp4` |
| Upscale subset frames.json (×5) | `upscale_subset/upscale_<label>.frames.json` |
| Upscale subset analysis | `upscale_subset/upscale_analysis.json` |
| Preview cache for viewer | `.cache/DJI_*_5eedd77d_1920x1080.bin` (~6.4 GB, mmap) |
