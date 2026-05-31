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
| 6b  | `2026-05-31-dynamic-ablation-and-scaling.md` (dynamic rows + scaling + paper scaffold) | 9, 10, 15 | **in flight — autonomous Night 2** |
| 7   | `2026-05-28-telemetry-import-visualizer.md`           | 11, 12      | done        |
| 8   | Drone-follow migration + RPI-GS data collection       | 13, 16      | not started (needs flights / ops) |
| 9   | Paper-with-code artifacts                             | 15          | not started |
| (10)| DJI optical-zoom maximum-range bonus shoot            | 17          | bonus / ops |

Update the **Status** column as plans land. When a plan finishes, set its
status to `done` and bump the next plan to `in flight`.

## Open follow-ups (tracked, not yet planned)

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
