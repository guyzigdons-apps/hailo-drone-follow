---
name: tracking_callback_risks
description: Known fragile spots in the detection callback / vision_branches `highlight_target` probe — start here when investigating tracking regressions.
type: project
---

# Tracking-callback risk map

When the user reports "tracking lost target", "wrong person locked", or "ReID misbehaving" — these are the first places to look.

## Where the per-frame logic lives
- `drone_follow/pipeline_adapter/hailo_drone_detection_manager.py` — the user_callback. Drives ByteTracker, ReID gallery decisions, raw-detection fallback, AUTO/LOCKED/IDLE state machine.
- `drone_follow/pipeline_adapter/vision_branches.py` — output-stage assembly (display / record / openhd / webui) AND a `highlight_target` pad probe that mutates the ROI on the local-display branch.
- `drone_follow/pipeline_adapter/reid_manager.py` — gallery, drift gate, reacquire.
- Algorithm doc: `docs/tracking-reid-algorithm.md` (full per-frame flow).

## Fragile patterns to audit on changes

### 1. Iterate-and-remove on `roi.get_objects_typed(...)`
**Why:** Hailo Python bindings may invalidate iterators when the underlying list is mutated mid-iteration; entries can be silently skipped. Adding `roi.remove_object(det)` inside the loop body is the failure pattern.

**Where:** seen near the top of `_app_callback_inner` filtering non-`person` detections (~`hailo_drone_detection_manager.py:237-249` as of `c48a65e1`).

**Safer:** collect targets in a list first, then remove in a second pass. Or operate on a copy via `list(roi.get_objects_typed(...))`.

### 2. ROI mutation on a tee'd buffer
**Why:** GStreamer `tee` shares buffer metadata across branches by reference (no automatic COW for `HailoROI`). If branch A removes an object the ROI list is mutated for branches B / C / D too.

**Where:** `vision_branches.py` `highlight_target` does `roi.remove_object(target_orig); roi.add_object(<copy with class_id=99>)`. Lives downstream of an `output_tee`. Webui (MJPEG) and OpenHD branches see the *modified* ROI.

**What can go wrong:**
- Anything that keys the locked target by `class_id` or by HailoUniqueID ordering on a non-display branch finds the target re-classified as `99` (sentinel).
- The child re-attach loop silently swallows exceptions, so `HailoUniqueID` and ReID-embedding sub-objects may be dropped on the way through.

### 3. `target_id > 0` strict positivity check
**Where:** `vision_branches.py` target match: `target_id is not None and target_id > 0`.

**Why fragile:** `follow_id == 0` is the AUTO-mode sentinel per `openhd_bridge.py`. AUTO-mode targets are never highlighted. Also any tracker that ever assigns `track_id == 0` will silently fail to match.

### 4. ByteTracker bypass via raw-detection fallback
**Where:** `score_visible_persons` in `reid_manager.py`.

**Why:** When ByteTracker activates 0 tracks but raw `persons` are visible, ReID drives the controller from a raw bbox with no `track_id`. Locked-follow can survive a tracker dropout — but check coordinate space (raw-detection bbox is post-tile-aggregation, full-frame normalized).

### 5. Coordinate spaces (post-tiling)
**Why:** Detections from cropped tile inferences must be mapped back to full-frame coords by `hailotileaggregator`. If a tile boundary clips a person, NMS dedup needs `iou-threshold` and `border-threshold` tuned right or you get phantom doubles.

**Audit when:** changing `tiles-x`/`tiles-y`/overlap, switching grids, or using the `_dynamic` cropper's mixed sources (dynamic + grid + static).

## Quick diff against a known-good revision
Tag the last-known-good tip before pulling, then compare just the tracking files:
```bash
git tag pre-pull-$(date +%Y%m%d)
git pull --ff-only
git diff pre-pull-<date>..HEAD -- \
  community/apps/hailo_drone_follow/drone_follow/pipeline_adapter/
```
See [safe-pull-and-rollback skill](../skills/safe-pull-and-rollback/SKILL.md) for the full pattern.
