# Independent review — Night-1 (Plan 6) commits `a5f2938..b15623d`

**Reviewer:** autonomous-project-manager (Night 2), manual review (no `Agent`
subagent tool available this session — self-review with the same rigor a
spec+quality reviewer pair would apply). **Date:** 2026-05-31.

**Scope:** the 8 substantive code commits in the range plus the 3 table commits
and the state/progress commits. Night 1 self-reviewed inline only; this is the
first independent pass.

| commit | area | verdict |
|---|---|---|
| `a5f2938` | C++/Python idempotent cache writes (INSERT OR IGNORE) | APPROVED (1 nit) |
| `c9e61f0` | warm_gst_cache.py (GST-golden warmer) | APPROVED |
| `d8c1949` | bench config matrix | APPROVED |
| `60533f9` | bench runner (static replay + tile_norm_to_source_px) | APPROVED |
| `b699660` | hailo-tiling-bench CLI + metrics + grid | APPROVED (1 nit) |
| `9a5f6ea` | GstCropperBackend | APPROVED |
| `7dbf2ad` | per-grid warm helper + .tile_cache gitignore | APPROVED |
| `899b1fc` | crop-ordered replay for per-tile-buffer caches | APPROVED (1 IMPORTANT, non-blocking) |

**Overall: no BLOCKING correctness issues found. All suites green at review
time (pytest 265 + 9 skipped, meson 5/5 — count grew from Night-1's 261 via
Night-2 A1/B1/B2). Findings below are non-blocking; logged, not all fixed.**

---

## Per-commit findings

### `a5f2938` — idempotent cache writes — APPROVED
- `INSERT OR IGNORE` on both `frame_results` and `detections` (C++) and Python
  `put_many`. Correct first-writer-wins idempotency for re-runnable / overlapping
  grid warming. Tests were correctly migrated from the old "duplicate-PK ->
  throw + rollback whole batch" contract to the idempotent no-op contract, and
  double-insert tests added in both languages. Semantics-preserving as claimed.
- **NIT (non-blocking):** the "content is identical for a given key" justification
  is an assumption. If a re-warm ever produced *different* dets for the same
  `(frame_idx, crop, ppv)` (e.g. non-deterministic chip output), the first write
  silently wins and the divergence is undetectable. Acceptable because the chip
  output is treated as deterministic (the whole dynamic-caching scheme relies on
  it), but worth a one-line code comment if a future "verify-on-conflict" debug
  mode is wanted. Not fixing tonight.

### `c9e61f0` — warm_gst_cache.py — APPROVED
- Reuses the replay-gate pipeline construction + run loop (single source of truth
  for the source-pixel crop-key math). Grid-spec parsing is well-tested and
  rejects bad input. `default_cache_filename` strips a trailing `__fovNN` so the
  name is stable. Meta stamped once. (Night-2 A1 has since folded the
  subprocess-per-grid fix into this file — see commit `442d86e`.)

### `d8c1949` — bench config matrix — APPROVED
- `BenchConfig` frozen dataclass; `default_matrix()` mirrors the canonical grid
  set + dynamic + per-lever rows + exactly one dense-GT reference. Clean.

### `60533f9` — bench runner — APPROVED
- `tile_norm_to_source_px` has a 0-deviation parity test against the cropper rule
  across all canonical grids (the load-bearing chip-free-replay foundation).
  `run_config` static branch raises on a miss (full-warming guarantee); dynamic
  branch counts misses. The `_dynamic_crops` placeholder (no-lock cadence) is
  explicitly documented as a v1 stand-in; Night-2 B2 (`94d04e4`) adds the real
  stateful `run_dynamic_config`.

### `b699660` — CLI + metrics + grid — APPROVED
- `recall_precision_vs_reference` aggregation is correct; empty-denominator
  handling sound. Reference row run first so its dets are the denominator.
- **NIT (non-blocking):** `match_frame` uses greedy-by-descending-IoU matching,
  not optimal (Hungarian) assignment. Documented in the docstring and adequate
  for recall/precision vs a *dense* reference (where greedy and optimal rarely
  diverge). No change.

### `9a5f6ea` — GstCropperBackend — APPROVED
- Pure pipeline-string construction is chip-free + tested; `infer` is the only
  chip path. The per-crop pad-probe extraction with `first_buf = frame_idx *
  n_tiles` correctly windows the requested frame's tile buffers. Defensive
  pad/truncate to `n_tiles`. (Night-2 B1 `108f0c0` documents this as the
  per-frame-relaunch fallback and pins the per-frame-ROI contract.)

### `7dbf2ad` — warm helper + gitignore — APPROVED
- `_warm_one_fov.sh` is a thin external per-grid loop; `.tile_cache/` gitignored
  (caches are regenerable). Fine. (Now superseded by A1's in-warmer fix but kept
  as a harmless runner.)

### `899b1fc` — crop-ordered replay — APPROVED (1 IMPORTANT, non-blocking)
- `run_static_config_crop_ordered` reconstructs source frames by zipping the
  per-crop dets streams ordered by `frame_idx`, taking `n_frames = min(per-crop
  occurrence counts)`. The CLI auto-detects per-tile-buffer caches via a
  heuristic and routes static rows here, skipping dynamic rows on such caches.
  Crop keys (source-pixel) are the correctness anchor; frame *indexing* is
  sidestepped. Sound approach.

- **IMPORTANT finding (non-blocking, verified against the live fov50 cache):**
  the per-crop occurrence counts are NOT uniform. In the warmed
  `0026 fov50` cache, 204 crops occur **879** times but **8 crops occur 878**
  times — including the **1x1 full-frame crop `(0,0,3840,2160)`**. Consequences:
  - The `min`-based `n_frames` makes a grid that contains a short crop replay
    one fewer source frame (e.g. the 1x1 static config replays 878 frames while
    the 12x9 reference replays 879). The Night-1 tables report "879 frames"
    (the reference count) even for configs that internally truncate to 878.
  - In `recall_precision_vs_reference` the reference's extra trailing frame
    (878) then contributes its boxes as pure false-negatives for the short
    config — *slightly understating* that config's recall (≈0.1%, one frame of
    879). The recall *trend* across grids (the paper's point) is unaffected.
  - **Verified NOT a misalignment bug:** the short crops' `frame_idx` are
    contiguous `0..877` (the missing occurrence is the *trailing* frame, not a
    middle one), so the crop-ordered zip stays aligned — each k-th occurrence is
    genuinely source frame k. The defect is a benign off-by-one in coverage, not
    a corrupted alignment.
  - **Why 878 vs 879:** the 1x1 grid was warmed in its own subprocess and that
    decode produced one fewer frame than the multi-tile grids (a decoder
    last-frame edge, or a one-frame cap drift between per-grid subprocess runs).
  - **Recommendation (logged, not fixed tonight — would change committed
    numbers / needs a re-warm or a CLI flag):** either (a) intersect frames
    across configs so every row is scored over the SAME frame set (the largest
    common prefix, 878), making the comparison apples-to-apples; or (b) re-warm
    so all grids cover the same frame count. This is a precision refinement, not
    a soundness fix; the static tables stand. Filed as a Monday cleanup item in
    the state file.

---

## Verification performed this review
- Re-read each substantive source file (`tile_cache_db.cpp` insert constants,
  `warm_gst_cache.py`, `config.py`, `runner.py`, `metrics.py`, `bench.py`,
  `gst_cropper.py`).
- Re-ran the live fov50 cache occurrence-count audit (the IMPORTANT finding
  above) via direct SQLite queries.
- Confirmed full suites green: `pytest` 265 passed + 9 skipped; `meson` 5/5.

No blocking fixes were required, so no fix commits were produced for C1.
