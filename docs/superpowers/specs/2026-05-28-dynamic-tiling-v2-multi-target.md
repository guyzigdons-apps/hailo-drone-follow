# Dynamic Tiling v2 — Multi-Target Extension

**Date:** 2026-05-28
**Builds on:** [`2026-05-27-dynamic-tiling-design.md`](2026-05-27-dynamic-tiling-design.md) (v1 single-target)
**Branch:** `tiling-benchmark`

## 1. Goal (one sentence)

Extend the dynamic tile scheduler to follow **all activated ByteTracker tracks**
(persons + vehicles), with size-aware ROI merging when targets cluster, and an
aging-counter budget policy that always serves the user-selected target first
— so we can score dynamic tiling head-to-head against dense static tiling on
whole-frame mAP, not just single-target recall.

## 2. Why now

v1 (single-target) on the DJI clip (`analyze_pxt --iou 0.5`):

| class | n_gt | dynamic-single | 3x3+vga3x | 8x6 |
|---|---|---|---|---|
| person | 1972 | **86.8%** | 40.4% | 81.3% |
| vehicle | 3706 | 36.3% | 81.0% | **93.0%** |
| face | 125 | **85.6%** | 0.0% | 4.8% |
| license_plate | 363 | 42.7% | 0.0% | 43.0% |

Vehicles get no dedicated ROI in v1 → 36.3%. Multi-target should close that gap
without inflating budget, by reallocating the same per-frame headroom across all
visible targets.

## 3. Scope

### v2 includes
- All activated tracks across configurable classes (default: `{0,1}` =
  person + vehicle).
- One ROI per tracked target, sized by the same adaptive-zoom rule as v1.
- **Size-aware ROI merge** when two targets' padded detection bboxes overlap
  enough AND the union still fits in a reasonable single tile.
- **Aging-counter budget allocation:** the user-selected target (the GT
  trajectory's target, as before) is always served; other ROIs are served by
  aging priority (oldest deferred first).
- Whole-frame mAP scoring via the existing `tiling_benchmark/analyze_pxt.py`
  (already compatible with our `frames.json` schema).
- Viewer support: multiple lime-green ROIs per frame; merged ROIs get a
  distinct category (`"dynamic-merged"`, magenta) so they're visually
  distinguishable.

### v2 explicitly does NOT include
- Re-identification beyond what ByteTracker provides (ReID is the real
  pipeline's job — out of dynamic_tiling scope).
- Camera-motion compensation (CMC) — separate future work, see research memo.
- Altitude-gated zoom — separate future work.
- Skipped-tile carry-forward — separate future work.

## 4. Module changes

### 4.1 `dynamic_tiling/target_lock.py` → add `MultiTargetLock`
Keep `TargetLock` (v1) untouched for backwards compatibility. Add
`MultiTargetLock` that:
- Constructs one `create_tracker("byte", ...)` and feeds it **all class
  detections** (caller chooses which classes go in the array).
- Maintains a dict `targets: dict[int, TargetState]` keyed by stable
  `track_id`, holding `TargetState(track_id, cls, bbox_norm, status,
  frames_since_seen, last_velocity, age_counter, selected)`.
- `selected_track_id: int | None` — first track that IoU-matches the GT
  bbox on its lock frame (uses `lock_from_gt`-style logic). Selected target's
  ROI always gets priority.
- `step(dets_all_classes: Sequence[Det], *, gt_bbox_norm=None) -> list[TargetState]`:
  feeds the tracker, updates the per-track state, derives status from
  presence + buffer, increments `age_counter` for tracks the scheduler did
  NOT serve last frame (set externally), resets it for served tracks.

### 4.2 `dynamic_tiling/scheduler.py` → multi-target `decide`
New signature (single-target path stays available via a new class or a
config flag — TBD by implementer; cleanest is a `MultiTargetTileScheduler`
that reuses the existing helpers `_grid` and the ROI-sizing math).

Per frame:
1. **Discovery grid** on cadence (unchanged from v1).
2. **Per-target ROI candidates:** for every TRACKING target, build one
   `CropRect` using the same adaptive-zoom sizing as v1 (sized by the
   target's predicted height). Tag with the track_id.
3. **Merge pass (size-aware):** pair-wise greedily merge ROIs when both
   conditions hold:
   ```
   pad = merge_pad_frac (default 0.25)
   det_iou_padded(a, b, pad) >= merge_iou_threshold (default 0.5)
   AND union_crop_w(a, b) <= merge_union_inflate_max * max(crop_w_a, crop_w_b)
       (default 1.5)
   ```
   The first clause says "the underlying detections (padded) overlap a lot";
   the second clause is the automatic size-aware gate (a merged ROI must not
   blow up the tile beyond ~1.5× the largest contributor — otherwise the
   zoom benefit collapses). When merging, the new tile is the smallest
   `CropRect` containing both targets centred on the union midpoint.
4. **Recovery grid** when the SELECTED target is SEARCHING/LOST (other
   tracks' loss does NOT trigger recovery in v2 — keeps budget bounded).
5. **Budget allocation (aging-counter):**
   - Selected target's ROI is reserved (always included if it exists).
   - Remaining ROIs sorted by `age_counter` desc, then by bbox area desc
     (large first as tie-break).
   - Truncate to `int(meter.available(frame_idx)) - (1 if selected_present
     else 0)`.
   - Notify the lock: which targets were served (`age_counter ← 0`) vs
     deferred (`age_counter += 1`).
6. **Tagging for the viewer:**
   - merged tiles → `"dynamic-merged"` (new category — implement in
     `overlay_viewer.py`, suggested colour magenta).
   - per-target ROIs → `"dynamic"` (existing lime green).
   - discovery → `"multi-scale"`; recovery → `"single-scale"` (unchanged).

### 4.3 `dynamic_tiling/replay.py`
- `run(...)` now feeds **all** detections (not just persons) to the lock;
  records the served-vs-deferred set per frame so the lock can update aging
  on the next iteration.
- `emit_frames_json` already writes the per-frame `tiles` array — just
  carries the new `"dynamic-merged"` category through.

### 4.4 `dynamic_tiling/run_dynamic.py`
New CLI flags:
- `--multi-target / --no-multi-target` (default: `--multi-target` for v2).
- `--target-classes` (default: `0,1`).
- `--merge-iou-threshold` (default: `0.5`).
- `--merge-pad-frac` (default: `0.25`).
- `--merge-union-inflate-max` (default: `1.5`).

Scoring: also write `frames.json` is unchanged (overlay_viewer compatible);
the CLI prints a hint to run `analyze_pxt.py` for whole-frame mAP.

### 4.5 `tiling_benchmark/overlay_viewer.py`
Add `"dynamic-merged"` colour to `CAT_COLOURS` (e.g. magenta `(255, 0, 200)`
BGR).

## 5. Implementation phases

**Phase 1** — multi-target lock + per-target ROIs (no merge, no aging yet).
Should already close most of the vehicle gap.

**Phase 2** — size-aware merging + aging-counter budget + selected-target
priority + viewer category.

Each phase committed independently; mAP measured at the end of each phase
to attribute the gain.

## 6. Acceptance criteria

After phase 2:
- `analyze_pxt --pred dynamic_run_multi.frames.json` produces a row whose
  vehicle TOTAL recall is competitive with `pxt_3x3+vga3x` (≥ 80%) at an
  average tiles/frame still well below 10 (i.e. rt_factor ≥ 1.0).
- A new PERF_REPORT subsection documents the v2 row alongside the static
  rows.
- 35+ tests pass (29 existing + new per-target/merge/aging tests).
