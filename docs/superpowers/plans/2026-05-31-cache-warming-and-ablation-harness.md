# Cache Warming + Ablation Harness — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the GStreamer-golden cache **warmer** and the chip-free **ablation harness** (`hailo-tiling-bench`) so that warmed caches (chip) feed a reproducible ablation table (no chip) — the results engine for the tiling paper (master spec Phases 9–10).

**Architecture:** Two tracks that compose. **Track A (chip):** `warm_gst_cache.py` runs the canonical GStreamer pipeline (`hailotilecropper_dynamic → hailonet → hailofilter → hailocachewriter mode=tile_cache source-width/height`) over a video for a list of fixed normalized grids, recording per-tile detections in source-pixel-keyed SQLite (one cache file per `(clip, FOV)`). **Track B (no-chip):** `hailo-tiling-bench` sweeps a config matrix; each row's per-frame crops are computed as normalized tiles → `cache_keys.tile_crop_to_source_px` → source-pixel `CropRect`, looked up via `ReplayBackend` against the warmed cache, aggregated (NMS + boundary strip), and scored into an ablation table. The two tracks are key-compatible because the Plan-5b gate **proved** the cropper's source-pixel crop equals `tile_crop_to_source_px(normalized_tile, src_w, src_h)` exactly (0 deviations) — so warming by feeding normalized tiles to the cropper produces keys that the harness reproduces.

**Tech Stack:** Python (`hailo_tiling`), SQLite, GStreamer (`gst-launch`/pad probes via the existing `scripts/cache_gst_replay_gate.py`), C++ (`gst-hailo-cache` writer — one idempotency fix), pytest, Hailo HAILO10H (`hailo_yolov8n_4_classes_vga.hef`).

---

## Context for the implementer (read once)

- Repo: `/home/giladn/tappas_apps/repos/hailo-drone-follow`, branch `tiling-benchmark` (commit freely; not main).
- `source setup_env.sh` before any Python/chip work. Python: `./hailo-apps/venv_hailo_apps/bin/python -m pytest -q` (currently **238 passed + 6 skipped**). C++: `meson test -C gst-hailo-cache/build` (**5/5**).
- A HAILO10H is reachable (`hailortcli fw-control identify`). **`[CHIP]` tasks serialise on the one chip.**
- **Verified interfaces (build on these, don't reinvent):**
  - `hailo_tiling/backends/backend.py` — `InferenceBackend.infer(frame, crops: Sequence[CropRect], frame_idx) -> list[list[Det]]` (crop-local normalized dets, one list per crop, in order).
  - `hailo_tiling/backends/replay.py` — `ReplayBackend(store, ppv=1)`; `infer` raises `CacheMissError` on miss. Also `read_source_coord_detections` / `map_dets_to_source` (Task 8).
  - `hailo_tiling/backends/caching.py` — `CachingBackend(wrapped, store, ppv, quantise=None)`.
  - `hailo_tiling/cache/store.py` — `SqliteCacheStore.open(path)`, `get_many(frame_idx, crops, ppv)`, `put_many(rows)`, `meta_get/meta_put`, `stats()`.
  - `gst-hailo-cache/src/cache_keys.hpp` (C++) and the Python equivalent: normalized→source-pixel is `x=int(xmin*W)`, `w=clamp(int(width*W), 0, W-x)` (truncate-then-clamp; the TAPPAS rule). The Python helper lives in `hailo_tiling/types.py`/`cache` — **find it; if absent, add `tile_norm_to_source_px(xmin,ymin,w,h,W,H)` to `hailo_tiling/cache/hashing.py`** mirroring the C++ rule exactly.
  - `hailo_tiling/types.py` — `CropRect(x,y,w,h,mode)`, `Det(cls,score,x,y,w,h)`.
  - `hailo_tiling/emitters/`, `hailo_tiling/modifiers/`, `dynamic_tiling/scheduler.py` (`TileScheduler(discovery_period=15, discovery_grid=(3,2), recovery_grid=(3,3), max_zoom=2.0)`) — the levers, already built.
  - `tiling_benchmark/tiling_record.py:_grid_to_static_tiles(tiles_x,tiles_y,overlap_x,overlap_y,mode)` — normalized grid → list of `"x,y,w,h"` tiles. **Reuse this as the single grid definition** for both warming and replay-key computation.
  - `scripts/cache_gst_replay_gate.py` — builds the canonical cropper pipeline + extracts per-tile `HailoROI` dets via pad probe. **Reuse its pipeline construction + extraction** for the warmer.
  - `tiling_benchmark/run_pxt_bench.py:78-135` — the canonical **static grid set**: `1x1-640x480`, `2x2`, `3x2`, `3x3`, `4x3`, `6x4`, `8x6` (all overlap 0.25) + `+vga3x` rescue variants + `GT-12x9-25` dense reference (overlap 0.25). Use this as the matrix's static rows + GT.
- Test clip for warming: clip **0026** = `/home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov{50,60,70}.mp4` (3840×2160, ~879 frames each). HEF: `/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef`.
- Cache file naming: `.tile_cache/<clipstem>__<fov>__<hefsha16>.sqlite3` (mirror `hailo_tiling/cli/warm.py`'s naming).

---

# TRACK A — Cache warming (chip). Build A1–A2 (no-chip), then A3 runs in the background.

## Task A1 — `[no-chip]` — Idempotent cache writes (`INSERT OR IGNORE`)

**Why:** warming may re-run / overlap grids; the writer's `put_many` uses plain `INSERT` and throws on duplicate `(frame_idx,crop,ppv)`. Make warming re-runnable.

**Files:**
- Modify: `gst-hailo-cache/src/tile_cache_db.cpp` (`put_many`, `put_frame_results`)
- Modify: `hailo_tiling/cache/store.py` (`put_many`)
- Test: `gst-hailo-cache/tests/test_tile_cache_db.cpp`, `hailo_tiling/tests/test_cache_store.py`

- [ ] **Step 1: C++ failing test** — in `test_tile_cache_db.cpp`, insert the same `Row` twice via two `put_many` calls; assert the second does NOT throw and the row count stays 1.
- [ ] **Step 2: Run → fails** (`meson test -C gst-hailo-cache/build tile_cache_db`) with a duplicate-PK throw.
- [ ] **Step 3: Implement** — change the C++ insert SQL in `put_many` (and `put_frame_results`) from `INSERT INTO` to `INSERT OR IGNORE INTO`. (First-writer-wins; identical content so semantics are preserved.)
- [ ] **Step 4: Run → passes.** Full `meson test -C gst-hailo-cache/build` → 5/5.
- [ ] **Step 5: Python parity** — add the same double-insert test to `test_cache_store.py`; change `store.py:put_many` insert to `INSERT OR IGNORE`. Run `pytest hailo_tiling/tests/test_cache_store.py -q` → green.
- [ ] **Step 6: Commit** — `git add` the four files; `git commit -m "gst-hailo-cache: idempotent cache writes (INSERT OR IGNORE) for re-runnable warming"`.

**Acceptance:** double-insert is a no-op in both C++ and Python; both suites green.

## Task A2 — `[no-chip]` — `scripts/warm_gst_cache.py` (GStreamer-golden warmer)

**Files:**
- Create: `scripts/warm_gst_cache.py`
- Test: `tests/integration/test_warm_gst_cache.py` (chip-gated for the live part; a no-chip unit test for grid expansion)

**Interface:** `warm_gst_cache.py --video PATH --hef PATH --out-cache PATH --grids "3x2:0.25;6x4:0.25;..." --source-width 3840 --source-height 2160 [--max-frames N]`. For each grid: expand to normalized tiles via `_grid_to_static_tiles`, run the canonical cropper pipeline (reuse `cache_gst_replay_gate.py` pass-1 construction) feeding that grid's `tiles-static`, with `hailocachewriter mode=tile_cache source-width/height out=<out-cache>` recording into the SAME cache file (append; idempotent via A1). Stamp `meta` once. Log per-grid progress + row counts.

- [ ] **Step 1: No-chip unit test** — `test_grid_spec_expands_to_tiles`: parse `"3x2:0.25"` → call `_grid_to_static_tiles(3,2,0.25,0.25)` → assert 6 normalized tiles, each within [0,1]. (Pure parsing/expansion; no chip.)
- [ ] **Step 2: Run → fails** (parser/module not present).
- [ ] **Step 3: Implement the warmer.** Reuse the gate helper's pipeline-string builder + GST_PLUGIN_PATH setup. Loop grids; for each, run the pipeline to EOS writing into `--out-cache`. Use `gst-hailo-cache/build/src` for the cache plugin `.so`. Idempotent re-runs (A1). Print `grid=<g> rows=<n>` per grid and a final `total_rows`/`stats()`.
- [ ] **Step 4: Run → unit test passes.**
- [ ] **Step 5: Chip smoke** (`[CHIP]`, gate behind `HAILO_CHIP=1`): warm clip 0026 fov50 with `--grids "3x2:0.25" --max-frames 4` → assert the cache exists, `detections` has rows, `meta` has `video_w=3840`. Keep this test fast/tiny.
- [ ] **Step 6: Commit** — `git commit -m "warm_gst_cache: GStreamer-golden cache warmer (per-grid tiles-static)"`.

**Acceptance:** unit test green; chip smoke writes a non-empty source-pixel-keyed cache for one grid.

## Task A3 — `[CHIP]` — Warm clip 0026 (background; the night's chip job)

**Not a code task — a long-running warm.** Run AFTER A1+A2 land. The autonomous manager launches this in the background and polls; it must not collide with any other `[CHIP]` task.

- [ ] **Step 1:** Define the warm grid set = the static rows from `run_pxt_bench.py` + the dynamic scheduler's fixed grids: `1x1` (640-shrink, i.e. `1x1:0.0`), `2x2:0.25`, `3x2:0.25`, `3x3:0.25`, `4x3:0.25`, `6x4:0.25`, `8x6:0.25`, and `12x9:0.25` (GT). (Recovery `3x3` and discovery `3x2` are already in the set.) Record this exact grid-spec string in the run log.
- [ ] **Step 2:** Warm **fov50 first** (validation pass): `warm_gst_cache.py --video .../0026...fov50.mp4 --hef <vga> --out-cache .tile_cache/0026__fov50__<sha>.sqlite3 --grids "<set>" --source-width 3840 --source-height 2160`. Capture per-grid row counts + total. Then **validate** via Task B-side (B3's `--backend replay` over a couple of static configs HITs this cache with 0 misses) before proceeding.
- [ ] **Step 3:** On a clean validation, warm **fov60** and **fov70** with the same grid set.
- [ ] **Step 4:** Record final `stats()` per cache file + total chip time in the run log / state file.

**Acceptance:** three cache files (`0026__fov{50,60,70}`), each non-empty with `user_version=1` and the full grid set's rows; a static-baseline replay over fov50 reports 0 cache misses.

---

# TRACK B — Ablation harness (no-chip). Builds in parallel with Track A.

## Task B1 — `[no-chip]` — Bench config matrix

**Files:**
- Create: `hailo_tiling/bench/__init__.py`, `hailo_tiling/bench/config.py`
- Test: `hailo_tiling/tests/test_bench_config.py`

**Design:** a `BenchConfig` dataclass describing one ablation row: `name`, `kind` ("static" | "dynamic"), grid/scheduler params, budget (tiles/frame), and the lever flags (`asahi: bool`, `altitude_zoom: bool`). A `default_matrix()` returns the v1 rows: the static grid set (one row per grid), `dynamic` (current scheduler), `dynamic+asahi`, `dynamic+altitude_zoom`, plus the `12x9` GT reference row marked `is_reference=True`.

- [ ] **Step 1: Failing test** `test_default_matrix_has_expected_rows`: assert `default_matrix()` includes a `1x1`, `3x2`, `6x4`, `dynamic`, `dynamic+asahi`, `dynamic+altitude_zoom`, and exactly one `is_reference` row.
- [ ] **Step 2: Run → fails.**
- [ ] **Step 3: Implement** `BenchConfig` (dataclass) + `default_matrix()`. Static rows carry their grid spec (`tiles_x,tiles_y,overlap`); dynamic rows carry scheduler kwargs + lever flags.
- [ ] **Step 4: Run → passes.**
- [ ] **Step 5: Commit** — `git commit -m "hailo_tiling: ablation bench config matrix (static + dynamic + levers)"`.

**Acceptance:** `default_matrix()` returns the v1 rows; test green.

## Task B2 — `[no-chip]` — Per-config crop generation + replay run

**Files:**
- Create: `hailo_tiling/bench/runner.py`
- Test: `hailo_tiling/tests/test_bench_runner.py` (uses the committed GST cache fixture `hailo_tiling/tests/fixtures/gst_tile_cache_fov50_small.sqlite3` from Task 8, or a MockBackend)

**Design:** `run_config(cfg, store, video_meta, frames) -> list[FrameResult]`. For each frame:
- **static** rows: tiles = `_grid_to_static_tiles(...)` (normalized) → convert to source-pixel `CropRect` via the Python `tile_norm_to_source_px` helper (the truncate-then-clamp rule matching the cropper). Look up via `ReplayBackend`.
- **dynamic** rows: drive `TileScheduler` (+ the configured emitters/modifiers/levers) to get per-frame tiles (normalized) → same conversion → lookup. (ROI-zoom tiles may miss the warmed cache — see the run-lazy note; B2 surfaces misses as a structured `n_misses` count, it does NOT silently swallow them.)
- Aggregate per frame: NMS + `BoundaryStripFilter` (reuse `hailo_tiling/aggregator/`), mapping tile-local dets → source-frame via `map_dets_to_source` (Task 8). Return per-frame aggregated dets + tiles/frame count.

- [ ] **Step 1: Failing test** `test_static_config_replays_from_cache`: open the fixture cache, run a `1x1` static config over its frames via `run_config`, assert it returns aggregated detections and `n_misses == 0` for the cached grid. (If the fixture lacks that grid, use a MockBackend-seeded store instead — pick whichever is clean.)
- [ ] **Step 2: Run → fails.**
- [ ] **Step 3: Implement** `run_config` + the `tile_norm_to_source_px` helper (if not already added in A2). Reuse the aggregator + `map_dets_to_source`. Count and return cache misses; raise only if a *static* config (which must be fully warmed) misses.
- [ ] **Step 4: Run → passes.**
- [ ] **Step 5: Commit** — `git commit -m "hailo_tiling: bench runner — per-config crop gen + chip-free replay + aggregate"`.

**Acceptance:** a static config replays a warmed/seeded cache with 0 misses and returns aggregated source-frame detections; test green.

## Task B3 — `[no-chip]` — `hailo-tiling-bench` CLI + ablation table

**Files:**
- Create: `hailo_tiling/cli/bench.py`; register `hailo-tiling-bench` entry point in the package `pyproject.toml`
- Test: `hailo_tiling/tests/test_cli_bench.py`

**Design:** `hailo-tiling-bench --cache PATH --video PATH [--configs default] [--max-frames N] [--out-dir DIR]`. Runs `default_matrix()` rows via `run_config` (replay), writes one `frames.json` per config under `--out-dir`, and prints + writes `ablation_table.md`: one row per config with columns `[mean_tiles_per_frame, n_dets, recall_vs_reference, precision_vs_reference, n_misses]`. The `is_reference` row (12×9 GT) defines recall/precision denominators (IoU-matched at 0.5, reuse any existing matcher in `tiling_benchmark/`; if none, a small IoU matcher in `hailo_tiling/bench/metrics.py`).

- [ ] **Step 1: Failing test** `test_bench_cli_emits_table`: invoke the CLI (via `subprocess` or `main([...])`) against the fixture cache with `--configs` limited to `1x1` + the reference, assert `ablation_table.md` exists with a row per config and a numeric `recall_vs_reference`.
- [ ] **Step 2: Run → fails.**
- [ ] **Step 3: Implement** the CLI + `metrics.py` (IoU matcher, recall/precision vs reference). Emit `frames.json` per config + `ablation_table.md`.
- [ ] **Step 4: Run → passes.** Full `pytest -q` green.
- [ ] **Step 5: Commit** — `git commit -m "hailo-tiling-bench: ablation CLI + table (recall/precision vs dense-GT reference)"`.

**Acceptance:** the CLI produces an ablation table from a warmed cache, chip-free; test green.

## Task B4 — `[no-chip → CHIP validate last]` — `GstCropperBackend` (Phase 9, canonical live path)

**Files:**
- Create: `hailo_tiling/backends/gst_cropper.py`
- Test: `hailo_tiling/tests/test_backend_gst_cropper.py` (no-chip: pipeline-string construction; `[CHIP]` smoke gated on `HAILO_CHIP=1`)

**Design:** `GstCropperBackend(hef, post_so, source_w, source_h)` implementing `InferenceBackend.infer(frame, crops, frame_idx)` — drives the canonical cropper pipeline with `crops` (converted to normalized `tiles-static`) and returns per-crop tile-local dets via the same pad-probe extraction as `cache_gst_replay_gate.py`. Wrapped in `CachingBackend` it becomes the warm-and-run path for **dynamic/lever** configs (whose ROI tiles can't be pre-warmed). This is what makes "every paper row runs through GStreamer" true for the live rows.

- [ ] **Step 1: No-chip test** `test_gst_cropper_builds_pipeline`: construct the backend, assert the pipeline string it builds for a set of crops contains `hailotilecropper_dynamic`, the crops as `tiles-static`, `hailonet`, `hailofilter`, and no extra `videoscale` (per the Task-1 caps finding).
- [ ] **Step 2: Run → fails.**
- [ ] **Step 3: Implement** `GstCropperBackend` reusing the gate helper's pipeline construction + extraction. `infer` feeds `crops` as `tiles-static`, runs the pipeline for the frame, returns per-crop dets in input order.
- [ ] **Step 4: Run → no-chip test passes.**
- [ ] **Step 5: `[CHIP]` smoke** (gated, run LAST after Track A warming frees the chip): `CachingBackend(GstCropperBackend)` over 4 frames of 0026 fov50 with the dynamic config → asserts dets returned and cache populated. **Do not run while Track A holds the chip.**
- [ ] **Step 6: Commit** — `git commit -m "hailo_tiling: GstCropperBackend — canonical GStreamer research path (live + warm)"`.

**Acceptance:** pipeline-string test green; chip smoke (when chip free) returns dets and populates the cache.

---

## Tonight's autonomous execution (for the autonomous-project-manager)

**Operating model:** `~/.claude/agents/autonomous-project-manager.md` + the `handling-anthropic-session-limits` skill. Maintain a state file at `docs/superpowers/overnight-manager-state.md`; resume margin = reset + 10 min; amend HEAD only, never older history.

**Ordering / chip-contention rule (critical — one chip):**
1. **First (no-chip):** A1 (idempotency) → A2 (warmer, incl. its tiny chip smoke — run that smoke, then release the chip) → B1 → B2 → B3. These are all no-chip except A2's 4-frame smoke.
2. **Then launch A3 in the background** (warm 0026 fov50; on clean B3-replay validation, fov60 + fov70). **A3 owns the chip for the rest of the night.**
3. **While A3 warms:** continue B-track coding that is **no-chip** (finish B3 table polish; build B4 up to and including its no-chip pipeline-string test — but DEFER B4's `[CHIP]` smoke until A3 finishes).
4. **After A3 completes:** run B4's chip smoke; then run `hailo-tiling-bench` for real against the warmed 0026 caches and commit the first **static-baseline ablation table** (+ reference) for fov50/60/70.

**Two-stage review per task** (spec → quality) as in subagent-driven-development. `[CHIP]` tasks never run in parallel.

**Definition of "good morning" (in priority order):**
1. A1+A2 landed; warmer works.
2. 0026 fov50 (ideally +60/70) caches warmed; static-baseline replay = 0 misses.
3. B1–B3 landed; a committed **static-baseline ablation table** for 0026 from the warmed caches (chip-free).
4. B4 landed (live path) — dynamic/lever rows are a stretch goal (need ROI-tile chip warming via `CachingBackend(GstCropperBackend)`; fine to leave for the next session).

**Known carry-forwards (do NOT rabbit-hole on these tonight):** `letterbox` back-mapping bug (use `stretch`); `full_frame` post-aggregator payload unwired (use `tile_cache`/per-tile — the harness reads per-tile + de-tiles in Python); cross-engine equality is value-exact not text-identical.

**Hard stops:** if warming produces 0 detections across all grids (likely a pipeline/HEF misconfig), STOP A3 and leave a note — don't warm three FOVs of empty cache. If a static-config replay shows misses against its own warmed grid, STOP — the crop-key consistency is broken and B-track is built on it.

---

## Dependency graph
- A1 → A2 → A3. (A3 also needs B3 to validate replay.)
- B1 → B2 → B3. B2 needs the Task-8 fixture or a seeded store (no chip).
- B4 independent of A/B except its chip smoke (after A3).
- First real ablation table = A3 (fov50 warmed) + B3. Dynamic/lever rows = B4 chip warm (after A3).
