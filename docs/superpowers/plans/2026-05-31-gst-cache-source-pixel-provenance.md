# GST Cache — Source-Pixel Provenance, Resize Contract & Bit-Exact Gate

> **Status: DONE** (Tasks 1–10 complete; branch `tiling-benchmark`). The per-tile
> GST-live-vs-GST-cached bit-exact gate passes with 0 deviations. Supersedes
> Plan 5 (`2026-05-28-gst-cache-plugins.md`) tasks T7/T11/T14. Known limitations
> recorded in `gst-hailo-cache/README.md` → "Known limitations" and the
> "Known limitations" section at the foot of this file.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the `gst-hailo-cache` plugins record per-tile detections keyed by **source-video-pixel** crop rectangles with a recorded **resize envelope**, and gate the cache byte-for-byte by replaying the **same GStreamer pipeline** (live inference vs cached) per-tile, pre-aggregator.

**Architecture:** The cache is **provenance-only** — it never decides tiling; it records whatever the `hailotilecropper_dynamic` cropper actually produced (the cropper is the golden standard). Crop keys are absolute source pixels (normalized `HailoROI` bbox × source resolution, supplied once via a writer property read from the input video). The cropper's tile resize (OpenCV CPU — DSP is `HAILO15_TARGET`-only) is made explicit via a new `resize-mode=stretch|letterbox` property and recorded in the cache `meta` table. The bit-exact gate runs the canonical pipeline twice — once with live `hailonet`, once with `hailocachereader`+`hailocachebypass` — and diffs per-tile detections **before** the aggregator (which drops detections via NMS).

**Tech Stack:** C++ GStreamer (`GstBaseTransform`), TAPPAS `HailoROI`/`gst_hailo_meta`, SQLite, meson/ninja, Python (`hailo_tiling`), pytest, Hailo HAILO10H (`hailo_yolov8n_4_classes_vga.hef`).

---

## Context for the implementer (read once)

- Repo: `/home/giladn/tappas_apps/repos/hailo-drone-follow`, branch `tiling-benchmark` (feature branch — commit freely; do NOT touch main).
- The cache plugin lives in `gst-hailo-cache/` (NOT the `hailo-apps/` submodule). Build: `ninja -C gst-hailo-cache/build`; test: `meson test -C gst-hailo-cache/build` (currently **5/5**). Python suite: `source setup_env.sh && ./hailo-apps/venv_hailo_apps/bin/python -m pytest -q` (currently **235 passed + 4 skipped**).
- Already landed (HEAD `3347518`): the writer reads each tile's `get_hailo_main_roi(buf,false)->get_bbox()` and records a per-tile crop, but in the **cropped-branch caps space** (e.g. 640×480) and with no resize metadata. This plan changes the key to **source-pixel** space and records the resize envelope.
- A HAILO10H is reachable (`hailortcli fw-control identify` → HAILO10H, FW 5.3.0). `[CHIP]` tasks must run serialised (one chip).
- Test clip: `/home/giladn/Videos/Drone/Training/DJI_20260528155741_0029_D_prepared__fov50.mp4` (**3840×2160**; fov60/fov70 variants exist in the same dir). HEF: `/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef`.
- Working tiled-pipeline builder: `tiling_benchmark/tiling_record.py:152 DYNAMIC_TILE_CROPPER_PIPELINE` (cropper → bypass on `sink_0`; tiles on `src_1` through the inner inference pipeline → `sink_1`). The cache writer (tile_cache mode) sits in that inner branch after `hailofilter`, before `agg.sink_1`.
- **Cropper facts (verified):** `hailotilecropper_dynamic_resize` (`gsthailotilecropper_dynamic.cpp:71`) hardcodes `resize_normal` (stretch, `cv::INTER_LINEAR`). `resize_letterbox` already exists in the base (`gsthailobasecropper.cpp:1021`) but is unwired. The DSP crop+resize path is entirely `#ifdef HAILO15_TARGET` — on Hailo-8/10/x86 the OpenCV path (`opencv_crop_and_resize`, `:708`) runs.
- **Submodule build/install (Task 2 only):** changes to `hailo-apps/hailo_apps/postprocess/cpp/...` need `source setup_env.sh && hailo-compile-postprocess install` then `rm -f ~/.cache/gstreamer-1.0/registry.x86_64.bin`; the `.so` installs to `/usr/lib/x86_64-linux-gnu/gstreamer-1.0/` (root-owned — sudo). **Pitfall:** loading two versions of the plugin side-by-side segfaults; never `--gst-plugin-load` a freshly-built `.so` while the system one is installed. Do NOT push the submodule.

---

## Task 1 — `[CHIP]` — Caps trace: prove resize happens exactly once; define the canonical gate pipeline

**Files:**
- Create: `docs/superpowers/research/2026-05-31-resize-caps-trace.md`

**Goal:** Empirically establish, on the live pipeline, **where** the tile gets resized (cropper vs the `INFERENCE_PIPELINE` `videoscale`) and confirm it is exactly once. Decide the canonical inner pipeline the gate (Task 6) will use so resize is unambiguous.

- [ ] **Step 1: Run the canonical tiled pipeline with caps + cropper debug logging.**

Run (3×2 grid, 64 buffers):
```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow && source setup_env.sh
GST_DEBUG="hailotilecropper_dynamic:6,GST_CAPS:4,videoscale:5" \
gst-launch-1.0 -v filesrc location="/home/giladn/Videos/Drone/Training/DJI_20260528155741_0029_D_prepared__fov50.mp4" ! \
  decodebin ! videoconvert ! video/x-raw,format=RGB ! \
  hailotilecropper_dynamic name=tc tiles-static="0,0,0.4,0.5;0.3,0,0.4,0.5;0.6,0,0.4,0.5;0,0.5,0.4,0.5;0.3,0.5,0.4,0.5;0.6,0.5,0.4,0.5" \
  hailotileaggregator name=agg flatten-detections=true iou-threshold=0.3 \
  tc. ! queue ! agg.sink_0 \
  tc. ! video/x-raw,format=RGB ! queue ! \
    videoscale ! videoconvert ! \
    hailonet hef-path=/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef batch-size=1 force-writable=true ! \
    queue ! agg.sink_1 \
  agg. ! fakesink sync=false 2>&1 | grep -iE "Opencv Crop|DSP|Resize width|caps|negotiat" | head -60
```
Expected: `"Opencv Crop + Resize: ... Resize width <W> height <H>"` lines from the cropper (confirming OpenCV, with the destination dims it resizes to), and the negotiated caps into `hailonet`.

- [ ] **Step 2: Determine whether `videoscale` also rescales.**

Inspect the caps in/out of the `videoscale` element in the log. Record: (a) the cropper's output (resized) WxH, (b) videoscale's in WxH and out WxH, (c) hailonet's required input WxH.
- If videoscale in==out (no scaling) → resize happens **once**, in the cropper. ✅
- If videoscale rescales a second time → **double resize**; the gate pipeline (Task 6) MUST drop `videoscale` and rely on the cropper's resize (which targets the network input via caps).

- [ ] **Step 3: Write the finding.**

In `docs/superpowers/research/2026-05-31-resize-caps-trace.md` record: the OpenCV path is active (quote the `Opencv Crop + Resize` line), the cropper's destination dims, whether videoscale double-resizes, and the **canonical gate inner pipeline string** to use in Task 6 (cropper resizes to network input; `videoconvert`-only, no extra `videoscale`, OR an explicit single `videoscale` with cropper resize disabled — but per this plan resize stays in the cropper, so the gate inner pipeline is `video/x-raw,format=RGB ! videoconvert ! hailonet ! hailofilter`).

- [ ] **Step 4: Commit.**
```bash
git add docs/superpowers/research/2026-05-31-resize-caps-trace.md
git commit -m "research: caps trace — tile resize happens once (OpenCV cropper path)"
```

**Acceptance:** The finding document states, with quoted log evidence, that the cropper uses OpenCV resize, the destination dims, and whether any second resize exists. The canonical gate inner pipeline is written out verbatim.

---

## Task 2 — `[no-chip]` — Cropper: add `resize-mode=stretch|letterbox` property; wire `resize_letterbox`

**Files:**
- Modify: `hailo-apps/hailo_apps/postprocess/cpp/hailotilecropper_dynamic/gsthailotilecropper_dynamic.cpp` (resize dispatcher + property)
- (submodule — local change only, do NOT push)

**Goal:** Make the tile resize selectable. Default `stretch` preserves today's behavior; `letterbox` calls the existing `resize_letterbox` (aspect-preserving with padding) so Task 7 can compare them.

- [ ] **Step 1: Read the current dispatcher and the base resize signatures.**

Read `gsthailotilecropper_dynamic.cpp:71-79` (`hailotilecropper_dynamic_resize`) and `gsthailobasecropper.hpp:59-60` (`resize_normal`, `resize_letterbox`). Note `resize_letterbox(method, cropped, resized, roi, fmt, no_scaling_bbox)` takes the ROI (the dispatcher currently ignores its ROI param) and a `no_scaling_bbox` bool.

- [ ] **Step 2: Add a `resize-mode` GEnum property.**

In `gsthailotilecropper_dynamic.cpp`, define an enum type and a property (`PROP_RESIZE_MODE`, default `0`=stretch). Register the GType:
```cpp
typedef enum {
    HAILO_TILECROP_RESIZE_STRETCH   = 0,  // resize_normal — distort to dst dims (today's default)
    HAILO_TILECROP_RESIZE_LETTERBOX = 1,  // resize_letterbox — aspect-preserve + pad, sets scaling_bbox
} HailoTileCropResizeMode;

#define GST_TYPE_HAILO_TILECROP_RESIZE_MODE (gst_hailo_tilecrop_resize_mode_get_type())
static GType gst_hailo_tilecrop_resize_mode_get_type(void) {
    static GType t = 0;
    if (!t) {
        static const GEnumValue vals[] = {
            { HAILO_TILECROP_RESIZE_STRETCH,   "Stretch crop to network dims (distort)", "stretch" },
            { HAILO_TILECROP_RESIZE_LETTERBOX, "Letterbox crop (aspect-preserve + pad)", "letterbox" },
            { 0, NULL, NULL },
        };
        t = g_enum_register_static("HailoTileCropResizeMode", vals);
    }
    return t;
}
```
Add a `resize_mode` field to the instance struct (default `HAILO_TILECROP_RESIZE_STRETCH`), install the property in `class_init`, and handle it in set/get_property (follow the existing `tiles-static`/property pattern in this file).

- [ ] **Step 3: Branch the resize dispatcher on the mode.**

Replace `hailotilecropper_dynamic_resize` so it reads the instance's `resize_mode` and dispatches:
```cpp
static void
hailotilecropper_dynamic_resize(GstHailoBaseCropperDyn *base,
                                std::vector<cv::Mat> &cropped,
                                std::vector<cv::Mat> &resized,
                                HailoROIPtr roi,
                                GstVideoFormat fmt)
{
    GstHailoTileCropperDynamic *self = GST_HAILO_TILECROPPER_DYNAMIC(base);
    if (self->resize_mode == HAILO_TILECROP_RESIZE_LETTERBOX) {
        // resize_letterbox records the letterbox transform on the ROI's
        // scaling_bbox so the aggregator maps detections back correctly.
        resize_letterbox(cv::INTER_LINEAR, cropped, resized, roi, fmt, /*no_scaling_bbox=*/false);
    } else {
        resize_normal(cv::INTER_LINEAR, cropped, resized, fmt);
    }
}
```
(Use the correct instance-cast macro already defined in the file for `GST_HAILO_TILECROPPER_DYNAMIC`.)

- [ ] **Step 4: Build + install the cropper, flush the registry, verify the property.**
```bash
cd /home/giladn/tappas_apps/repos/hailo-drone-follow && source setup_env.sh
hailo-compile-postprocess install 2>&1 | tail -5
rm -f ~/.cache/gstreamer-1.0/registry.x86_64.bin
gst-inspect-1.0 hailotilecropper_dynamic | grep -A3 -i "resize-mode"
```
Expected: `resize-mode` property listed with `stretch`/`letterbox` enum values, default `stretch`.

- [ ] **Step 5: Smoke both modes (no chip — fakesink, no hailonet needed).**
```bash
for M in stretch letterbox; do
  echo "== resize-mode=$M =="
  gst-launch-1.0 videotestsrc num-buffers=3 ! video/x-raw,width=1280,height=720,format=RGB ! \
    hailotilecropper_dynamic tiles-static="0,0,0.5,1.0;0.5,0,0.5,1.0" resize-mode=$M \
    hailotileaggregator name=agg \
    tc. ! queue ! agg.sink_0 tc. ! queue ! agg.sink_1 agg. ! fakesink sync=false 2>&1 | tail -2
done
```
Expected: both run to EOS without error (the element instantiates and crops in both modes).

- [ ] **Step 6: Commit.**
```bash
git -C hailo-apps add hailo_apps/postprocess/cpp/hailotilecropper_dynamic/gsthailotilecropper_dynamic.cpp
git -C hailo-apps commit -m "hailotilecropper_dynamic: add resize-mode=stretch|letterbox (wire resize_letterbox)"
```
(The submodule has its own git; commit there. Do NOT push.)

**Acceptance:** `gst-inspect-1.0 hailotilecropper_dynamic` shows the `resize-mode` property (default `stretch`); both modes run a smoke pipeline to EOS. Existing behavior is unchanged at the default.

---

## Task 3 — `[no-chip]` — Schema: record source resolution + resize envelope in `meta`

**Files:**
- Modify: `hailo_tiling/cache/schema.sql` (comment only — `meta` table already exists)
- Modify: `gst-hailo-cache/src/tile_cache_db.hpp`, `gst-hailo-cache/src/tile_cache_db.cpp` (typed meta helpers)
- Test: `gst-hailo-cache/tests/test_tile_cache_db.cpp`

**Goal:** Standardize the `meta` keys that record the provenance envelope so both GST and Python agree. No new tables — the `meta(k,v)` table already exists (`schema.sql:21`).

**Meta keys (string values):** `video_w`, `video_h` (source pixels), `resize_mode` (`stretch`|`letterbox`), `dst_w`, `dst_h` (network input dims the cropper resized to), `interpolation` (`linear`), `hef_sha` (HEF identifier).

- [ ] **Step 1: Write a failing test for typed meta round-trip.**

Add to `tests/test_tile_cache_db.cpp` (gtest):
```cpp
TEST(TileCacheDbMeta, EnvelopeRoundTrip) {
    auto tmp = std::filesystem::temp_directory_path() / "env_meta.sqlite3";
    std::filesystem::remove(tmp);
    hailo_cache::TileCacheDb db;
    db.open(tmp.string(), /*create_if_missing=*/true);
    db.meta_put("video_w", "3840");
    db.meta_put("video_h", "2160");
    db.meta_put("resize_mode", "stretch");
    EXPECT_EQ(db.meta_get("video_w").value(), "3840");
    EXPECT_EQ(db.meta_get("resize_mode").value(), "stretch");
    EXPECT_FALSE(db.meta_get("missing_key").has_value());
}
```

- [ ] **Step 2: Run it — verify it passes (meta_get/meta_put already exist).**

Run: `ninja -C gst-hailo-cache/build && meson test -C gst-hailo-cache/build tile_cache_db`
Expected: PASS — `meta_get`/`meta_put` already exist in `tile_cache_db.hpp:96-100`. (If the test reveals a gap, fix `tile_cache_db.cpp` accordingly.)

- [ ] **Step 3: Document the canonical keys.**

Add a comment block to `hailo_tiling/cache/schema.sql` under the `meta` table listing the seven envelope keys and their units/semantics (so the Python and C++ sides share one spec).

- [ ] **Step 4: Run the full meson suite.**

Run: `meson test -C gst-hailo-cache/build` — expected **6/6** (5 existing + this is part of `tile_cache_db`, so still 5 suites but more cases).

- [ ] **Step 5: Commit.**
```bash
git add gst-hailo-cache/tests/test_tile_cache_db.cpp hailo_tiling/cache/schema.sql
git commit -m "gst-hailo-cache: standardize cache meta envelope keys (source res + resize)"
```

**Acceptance:** meta round-trip test passes; schema.sql documents the seven keys.

---

## Task 4 — `[no-chip]` — Writer: source-pixel crops + record resize envelope

**Files:**
- Modify: `gst-hailo-cache/src/gst_hailocachewriter.hpp` (properties), `gst-hailo-cache/src/gst_hailocachewriter.cpp` (`read_tile_crop_rect_`, set_caps, meta write)
- Test: `tests/integration/test_writer_crop_provenance.py` (extend, `[CHIP]`-gated)

**Goal:** Record crop keys in **source-video pixels** (normalized bbox × source dims from new properties), and write the resize envelope to `meta` once at start.

**New properties:** `source-width` (uint, default 0), `source-height` (uint, default 0), `resize-mode` (string, default `stretch`), `dst-width`/`dst-height` (uint, default 0 → fall back to caps dims), `hef-sha` (string, default "").

- [ ] **Step 1: Add the properties.**

In `gst_hailocachewriter.hpp` add fields (`guint source_width; guint source_height; gchar* resize_mode; guint dst_width; guint dst_height; gchar* hef_sha;`). In `.cpp` install them in `class_init` and handle set/get (follow the existing `output_file`/`mode` property pattern). Default `source_width=source_height=0`.

- [ ] **Step 2: Change `read_tile_crop_rect_` to scale by source dims.**

Replace the dimension used for normalized→pixel conversion: when `source_width>0 && source_height>0`, use those; else fall back to the caps `frame_width/frame_height` (preserves current no-property behavior). Keep the exact TAPPAS truncate-then-clamp rule:
```cpp
const int W = (self->source_width  > 0) ? (int)self->source_width  : self->frame_width;
const int H = (self->source_height > 0) ? (int)self->source_height : self->frame_height;
// ... cx = (int)(bbox.xmin()*W); cw = clampi((double)bbox.width()*W, 0, W - cx); etc. (unchanged rule)
```

- [ ] **Step 3: Write the envelope to `meta` once (on first buffer / writer-thread open).**

When the DB is opened, `meta_put` the keys: `video_w`/`video_h` from source props (or caps fallback), `resize_mode` from the property, `dst_w`/`dst_h` from `dst-width`/`dst-height` (or caps), `interpolation`="linear", `hef_sha` from the property. Guard so it runs once.

- [ ] **Step 4: Extend the on-chip test to assert source-pixel keys.**

In `tests/integration/test_writer_crop_provenance.py`, run the canonical pipeline with `source-width=3840 source-height=2160` on the writer and assert the recorded crops are in 3840×2160 space (e.g. tile 0 ≈ `(0,0,1536,1080)` for `tiles-static` `0,0,0.4,0.5`) and that `meta` contains `video_w=3840`, `resize_mode=stretch`. Keep the existing assertion structure + `HAILO_CHIP=1` gate.

- [ ] **Step 5: Verify no-chip suites stay green.**

Run: `ninja -C gst-hailo-cache/build && meson test -C gst-hailo-cache/build` (5/5) and `./hailo-apps/venv_hailo_apps/bin/python -m pytest -q` (235 + 4 skipped).

- [ ] **Step 6: On-chip verify.**

Run: `source setup_env.sh && HAILO_CHIP=1 ./hailo-apps/venv_hailo_apps/bin/python -m pytest tests/integration/test_writer_crop_provenance.py -v` — expected PASS with source-pixel crops.

- [ ] **Step 7: Commit.**
```bash
git add gst-hailo-cache/src/gst_hailocachewriter.hpp gst-hailo-cache/src/gst_hailocachewriter.cpp tests/integration/test_writer_crop_provenance.py
git commit -m "gst-hailo-cache: writer records source-pixel crops + resize envelope meta"
```

**Acceptance:** With `source-width/height` set, recorded crops are in source-pixel space and `meta` holds the envelope; suites stay green; on-chip test passes.

---

## Task 5 — `[no-chip]` — Reader: source-pixel provenance lookup

**Files:**
- Modify: `gst-hailo-cache/src/gst_hailocachereader.cpp` (props + transform_ip crop derivation)
- Test: `gst-hailo-cache/tests/test_hailocachereader.cpp`

**Goal:** The reader must derive the **same source-pixel crop key** the writer used, so replay hits. Today the reader uses a full-frame fallback (`gst_hailocachereader.cpp:591-600`).

- [ ] **Step 1: Add `source-width`/`source-height` properties** to the reader (mirror Task 4 defaults/handling).

- [ ] **Step 2: Write a failing unit test** in `test_hailocachereader.cpp` that seeds a cache row at a source-pixel crop (e.g. `(0,0,1536,1080)` for frame 0) and asserts the reader, fed a buffer carrying a tile `HailoROI` of normalized `(0,0,0.4,0.5)` with `source-width=3840 source-height=2160`, looks up and HITs that row. (Follow the existing reader test harness for constructing buffers + ROIs.)

- [ ] **Step 3: Run it — verify it fails** (reader still uses full-frame fallback).
Run: `meson test -C gst-hailo-cache/build hailocachereader` → FAIL.

- [ ] **Step 4: Implement source-pixel crop derivation** in `transform_ip`: when a per-tile `HailoROI` is present and `source-width/height` set, compute the crop key as `bbox × source dims` with the same truncate-then-clamp rule as the writer (factor the conversion into a shared helper if clean). Keep the full-frame fallback when no ROI.

- [ ] **Step 5: Run it — verify it passes.**
Run: `meson test -C gst-hailo-cache/build hailocachereader` → PASS.

- [ ] **Step 6: Full suite + commit.**
Run: `meson test -C gst-hailo-cache/build` (5/5).
```bash
git add gst-hailo-cache/src/gst_hailocachereader.cpp gst-hailo-cache/tests/test_hailocachereader.cpp
git commit -m "gst-hailo-cache: reader derives source-pixel crop key from tile ROI"
```

**Acceptance:** Reader HITs writer-produced source-pixel rows in a unit test; 5/5 meson.

---

## Task 6 — `[CHIP]` — Bit-exact gate: live vs cached, per-tile, pre-aggregator

**Files:**
- Create: `scripts/cache_gst_replay_gate.py` (helper: runs the two passes, dumps per-tile detections)
- Create: `tests/integration/test_cache_gst_replay_gate.py` (`[CHIP]`-gated)

**Goal:** Prove that replaying the cache reproduces live inference **exactly**, at the per-tile point (before NMS), using the **same** GStreamer pipeline.

**Approach:** Pass 1 — canonical pipeline with live `hailonet ! hailofilter ! hailocachewriter(tile_cache, source-width/height set)`; tap per-tile detections via a pad probe after `hailofilter`, write the cache. Pass 2 — same pipeline but `hailocachereader(cache) ! hailocachebypass`; tap per-tile detections at the same point. Compare the two per-tile detection streams keyed by `(frame_idx, crop_rect)`.

- [ ] **Step 1: Write the helper `scripts/cache_gst_replay_gate.py`.**

It builds both pipelines (use the canonical inner pipeline from Task 1's finding + `DYNAMIC_TILE_CROPPER_PIPELINE`), runs each for N frames, and on each buffer after `hailofilter` extracts the `HailoROI` detections (via the python `hailo` bindings used elsewhere in the repo) keyed by the source-pixel crop. Writes `pass1.json` / `pass2.json` and a `diff_report.json` (`status: OK|DIFF`, list of mismatching keys). Accept `--video`, `--hef`, `--out-dir`, `--max-frames`, `--tiles-static`.

- [ ] **Step 2: Write the chip-gated test** `tests/integration/test_cache_gst_replay_gate.py` mirroring `test_cache_bit_exact_e2e.py` conventions (`HAILO_CHIP=1`/`--chip` gate, `_chip_available()`, artifact-existence skips). It invokes the helper (`max_frames=16`) and asserts `diff_report["status"] == "OK"` and `pass2` served every tile from cache (no chip access in pass 2 — assert via a marker the reader sets / chip-call count if available).

- [ ] **Step 3: Run on chip.**
Run: `source setup_env.sh && HAILO_CHIP=1 ./hailo-apps/venv_hailo_apps/bin/python -m pytest tests/integration/test_cache_gst_replay_gate.py -v`
Expected: PASS — per-tile detections byte-match between live and cached passes.

- [ ] **Step 4: Verify no-chip suites still green, then commit.**
```bash
git add scripts/cache_gst_replay_gate.py tests/integration/test_cache_gst_replay_gate.py
git commit -m "gst-hailo-cache: per-tile live-vs-cached bit-exact replay gate (chip)"
```

**Acceptance:** On chip, per-tile detections from the cached pass equal the live pass exactly (`status: OK`); pass 2 serves all tiles from cache.

---

## Task 7 — `[CHIP]` — Experiment: envelope vs stretch resize quality

**Files:**
- Create: `docs/superpowers/research/2026-05-31-resize-envelope-vs-stretch.md`

**Goal:** Decide the production default for `resize-mode` by comparing detection quality between `stretch` and `letterbox` on the test clip(s). This is an experiment, not a TDD task.

- [ ] **Step 1: Run the canonical detection pipeline twice** (same clip, same grid, same HEF), once `resize-mode=stretch`, once `resize-mode=letterbox`, recording post-aggregator detections (full_frame mode writer after `hailodetiler`, or `hailocachewriter mode=full_frame`). Use ≥2 FOV variants (fov50, fov70) for breadth.

- [ ] **Step 2: Compare** detection counts, confidence distributions, and (if a labelled/GT reference exists in `tiling_benchmark`) recall/precision per mode. Note any qualitative differences at tile boundaries.

- [ ] **Step 3: Write the decision** in the research doc: which mode detects better and why, and the recommended production default for `resize-mode`. If `letterbox` wins, note that the writer's `resize_mode` meta + the cropper default should flip (follow-up, not in this task).

- [ ] **Step 4: Commit.**
```bash
git add docs/superpowers/research/2026-05-31-resize-envelope-vs-stretch.md
git commit -m "research: resize envelope vs stretch — detection-quality comparison + decision"
```

**Acceptance:** A written, data-backed recommendation for the production `resize-mode` default.

---

## Task 8 — `[no-chip]` — Python reproducibility: cache → source-coord detections

**Files:**
- Modify: `hailo_tiling/backends/replay.py` (or a small new reader util) to expose per-tile detections mapped to source coords from a GST-produced cache
- Test: `hailo_tiling/tests/test_backend_replay.py` (add a case over a committed small GST-produced cache fixture)

**Goal:** A pure-Python path can read a GST-produced cache and yield per-tile detections in **source coords**, suitable for offline postprocessing (R5). Python follows the GST cache; it does not re-resize pixels.

- [ ] **Step 1: Add a tiny committed fixture** — a small GST-produced cache (a few frames, produced by the Task 4 writer with `source-width/height` set) under `hailo_tiling/tests/fixtures/`. (Generate on chip once, commit the `.sqlite3`.)

- [ ] **Step 2: Write a failing test** asserting `ReplayBackend` (or the new util) reads the fixture and returns, for a known `(frame_idx, source-pixel crop)`, the expected detections in source-frame normalized coords.

- [ ] **Step 3: Implement** the read/map path (the store already returns `dets_json`; map tile-local dets → source coords using the source-pixel crop key + recorded `video_w/h` from meta).

- [ ] **Step 4: Run test → PASS; full Python suite → green; commit.**
```bash
git add hailo_tiling/backends/replay.py hailo_tiling/tests/test_backend_replay.py hailo_tiling/tests/fixtures/
git commit -m "hailo_tiling: replay reads GST cache, maps tiles to source coords"
```

**Acceptance:** Python reads a GST-produced cache and yields source-coord detections; Python suite green.

---

## Task 9 — `[no-chip]` — Document/wire full-solution recorder placement (post-aggregator)

**Files:**
- Modify: `gst-hailo-cache/README.md` (placement section)
- Test: existing `full_frame` writer tests (no new behavior — placement guidance)

**Goal:** Make explicit the two scenarios: **cache testing → tile_cache writer pre-aggregator (per-tile)**; **full-solution review → full_frame writer post-aggregator (post-NMS)**. The writer already supports both modes; this task documents the placement and verifies the post-aggregator path runs.

- [ ] **Step 1: Add a "Two recording scenarios" section** to `gst-hailo-cache/README.md` with the exact pipeline placements for each (tile_cache after `hailofilter`; full_frame after `hailodetiler`/aggregator).

- [ ] **Step 2: Smoke the full_frame post-aggregator placement** (no chip ok with videotestsrc through the cropper+aggregator) to confirm rows land in `frame_results`.

- [ ] **Step 3: Commit.**
```bash
git add gst-hailo-cache/README.md
git commit -m "docs: cache recorder placement — per-tile (cache test) vs post-NMS (solution review)"
```

**Acceptance:** README documents both placements; full_frame post-aggregator smoke writes `frame_results` rows.

---

## Task 10 — `[no-chip]` — Close-out: docs + MEMORY + plan INDEX

**Files:**
- Modify: `.claude/memory/` (new note + MEMORY.md pointer), `docs/superpowers/plans/INDEX*` (mark this plan done), `docs/tracking-reid-algorithm.md`/cache docs if needed

- [x] **Step 1: Add a memory note** summarizing: cache is provenance-only + source-pixel keys; resize is OpenCV (DSP=HAILO15-only); `resize-mode` cropper property; the per-tile live-vs-cached gate; recorder placements. Add the MEMORY.md pointer line.
- [x] **Step 2: Flip this plan's INDEX entry to done** (and reconcile with the Plan 5 gst-cache-plugins tasks T7/T11/T14 — note they're superseded/satisfied by this plan's source-pixel approach).
- [x] **Step 3: Run both suites one final time** (meson 5/5; pytest green) and record counts in the commit.
- [x] **Step 4: Commit.**
```bash
git add .claude/memory docs/superpowers/plans
git commit -m "gst-hailo-cache: close-out — source-pixel provenance + bit-exact gate complete"
```

**Acceptance:** Memory + INDEX updated; both suites green.

---

## Dependency ordering

- Task 1 (caps trace) → informs Task 6's pipeline.
- Task 2 (cropper resize-mode) is independent `[no-chip]`; needed before Task 7.
- Task 3 (schema meta) → Task 4 (writer meta).
- Task 4 (writer source-pixel) → Task 5 (reader source-pixel) → Task 6 (gate).
- Task 6 → produces fixture for Task 8.
- Tasks 7, 9 independent after their deps; Task 10 last.
- `[CHIP]` tasks (1, 4-on-chip-verify, 6, 7) run serialised.

---

## Known limitations (recorded at close-out)

1. **`letterbox` resize-mode has a back-mapping offset bug.** There is a
   systematic positional offset when detections are mapped back from a
   letterboxed tile (Task 7; see
   `docs/superpowers/research/2026-05-31-resize-envelope-vs-stretch.md`).
   `stretch` is the production default and the only fully validated mode;
   `letterbox` is **not production-ready**.
2. **`full_frame` mode `dets_json`/`tiles_json` payloads are unwired.** The
   `full_frame` (post-aggregator) writer still emits `"[]"` for both columns
   pending the Phase-14 source-frame-coord payload wiring. Only `tile_cache`
   (pre-aggregator) mode serializes real per-tile detections.
3. **Cross-engine cache equality is value-exact, not byte-text-identical.** The
   per-tile GST-vs-GST gate is bit-exact, but the Python reader path is
   value-lossless at float32 only: serialized JSON differs textually (`%.9g` in
   C++ vs Python shortest-repr) while round-tripping to identical float32 values.

Mirrored in `gst-hailo-cache/README.md` → "Known limitations".
