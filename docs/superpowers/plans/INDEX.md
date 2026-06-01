# Implementation Plans Index

Tracks the decomposed plans derived from the master spec:
`docs/superpowers/specs/2026-05-28-tiling-library-design.md`.

| #   | Plan                                                  | Spec phases | Status     |
|-----|-------------------------------------------------------|-------------|------------|
| 1   | `2026-05-28-hailo-tiling-scaffold-and-scheduler-refactor.md` | 1, 2        | done       |
| 2   | `2026-05-28-telemetry-modifiers-backends.md`          | 3, 4, 5     | done       |
| 3   | `2026-05-28-fov-emulation-source-data-prep.md`        | 6           | done       |
| 4   | `2026-05-28-cache-schema-and-python-cache-layer.md`   | 7           | done       |
| 5   | `2026-05-28-gst-cache-plugins.md`                     | 8, 14       | done (via Plan 5b; T7/T11/T14 superseded) |
| 5b  | `2026-05-31-gst-cache-source-pixel-provenance.md`     | 8, 14       | done       |
| 6   | `2026-05-31-cache-warming-and-ablation-harness.md` (warmer + GstCropperBackend + `hailo-tiling-bench`) | 9, 10 | done (Night 1: harness + 0026 static tables, 0 misses) |
| 6b  | `2026-05-31-dynamic-ablation-and-scaling.md` (dynamic rows + scaling + paper scaffold) | 9, 10, 15 | done — but Night-2's "negative" dynamic result was **3 bugs, not science** (see 6c) |
| 6c  | Dynamic-path bug fixes + infra validation (2026-06-01 session, no separate plan doc) | 9, 10 | done — label off-by-one, GstCropperBackend video-path, discovery density all fixed; **full multi-target dynamic run validated** (8x6 disc @2fps + ROI follow, 6.84 tiles/frame, 879 frames via HefBackend). Open tuning in follow-ups below. |
| 7   | `2026-05-28-telemetry-import-visualizer.md`           | 11, 12      | done        |
| 8   | Drone-follow migration + RPI-GS data collection       | 13, 16      | not started (needs flights / ops) |
| 9   | Paper-with-code artifacts                             | 15          | not started |
| (10)| DJI optical-zoom maximum-range bonus shoot            | 17          | bonus / ops |

Update the **Status** column as plans land. When a plan finishes, set its
status to `done` and bump the next plan to `in flight`.

## Open follow-ups (tracked, not yet planned)

### Dynamic-path tuning + unification (next stage — feeds the experiment sweep)

- **Per-frame GStreamer relaunch DEADLOCKS after ~6 frames** (in-process pipeline
  teardown/relaunch hang). Full dynamic runs currently use the direct `HefBackend`
  (OpenCV crop → HailoRT) to sidestep it. To run the dynamic flow through the
  *GStreamer-golden* path at full length, implement the signal-handoff per-frame ROI
  injection in `hailotilecropper_dynamic` (C++, single long-lived pipeline) — or
  subprocess-per-frame as a stopgap.
- **Two inference paths not unified:** static caches = GStreamer cropper; dynamic full run =
  `HefBackend`. Slightly different resize/NMS → not byte-identical. Unify before paper-grade
  head-to-head numbers.
- **Two class conventions:** `HefBackend` emits 0-indexed (person=0, raw NMS decode);
  GStreamer `hailofilter` emits 1-indexed (person=1, label-file convention). Comparisons bridge
  by label today; unify (add a +1 offset option to `HefBackend`/decode).
- **Discovery grid has no overlap** (`scheduler._grid` lays tiles edge-to-edge); static ablation
  grids use 0.25. Add a discovery overlap fraction so boundary objects don't fragment.
- **Discovery cadence not perfectly regular** when targets are lost (recovery grid pre-empts the
  discovery slot); make discovery fire on cadence regardless of lock state.
- **ROI follow degrades to the recovery grid** on small aerial targets (ROI loses them) — tune
  ROI margin / max-zoom / re-detection. This is the core of the single-target follow sweep.
- **Metric:** for the dynamic claim, report *single-target* recall at low budget (dynamic's
  strength), not whole-frame detect-everything recall (where a uniform grid wins).

### Earlier follow-ups

- **Phase 14 — detiler `full_frame` payload — RESOLVED (Night-2 C3, commit pending).** Plan 5/5b
  satisfied the cache-hit bypass (wrapper element, no `hailofilter` patch) and per-tile crop
  provenance (reading the cropper's existing `HailoTileROI`). Night-2 C3 wired the remaining piece:
  `hailocachewriter mode=full_frame` now serializes the post-aggregator `HailoROI` detections
  (source-frame coords, via `read_tile_dets_json_`) into `dets_json` AND the per-frame tile layout
  (`HailoTileROI` sub-objects → `[{x,y,w,h,mode}]` via the new `read_tiles_json_`, `cache_keys`
  field constants) into `tiles_json` — both non-empty when the upstream ROI carries them (no-chip
  gtest `FullFramePayload.RecordsNonEmptyDetsAndTilesFromRoi` seeds an ROI + tile and verifies).
  CHIP smoke (full_frame post-aggregator over real frames) is the last remaining validation step.
  This unblocks the visualizer/overlay (Phase 12) on real `flight_record.sqlite3` data.
- **`resize-mode=letterbox` back-mapping bug.** Added to `hailotilecropper_dynamic` in Plan 5b but
  the letterbox→`scaling_bbox`→aggregator back-mapping is offset (see
  `docs/superpowers/research/2026-05-31-resize-envelope-vs-stretch.md`); `stretch` is the
  production default. Fix + re-test gate documented before letterbox could be used.
- **Cross-engine cache equality is value-exact (float32), not byte-text-identical** (`%.9g` vs
  Python shortest-repr) — fine for replay + Python postprocessing; relevant if a future test
  compares C++-written vs Python-written cache text.
