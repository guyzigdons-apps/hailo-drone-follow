# Night Run 2 — Dynamic Ablation Rows + Scaling + Paper Scaffold

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (dispatch implementer + spec-reviewer + quality-reviewer per task) IF the `Agent` tool is available; otherwise self-execute each task with rigorous inline verification (write → task tests → full suites → self-review → commit). Steps use checkbox (`- [ ]`) syntax.

**Goal:** Produce the paper's headline comparison — **dynamic (track-guided) tiling vs static baselines at matched compute** — by wiring the live `CachingBackend(GstCropperBackend)` path, warming dynamic ROI tiles on-chip, and extending the ablation tables with `dynamic / +asahi / +altitude-zoom` rows; plus fill the night with safe parallel work (warm more clips, independent review of Night-1 commits, paper-with-code scaffold, Phase-14 full_frame payload).

**Architecture:** Static baselines already replay chip-free from warmed caches (Night 1). The dynamic scheduler's track-guided **ROI-zoom tiles** are run-specific (can't be pre-enumerated), so the dynamic configs warm themselves on a one-time on-chip pass via `CachingBackend(GstCropperBackend)` — which drives the production cropper pipeline with **per-frame** tiles (the cropper's dynamic-ROI injection via `identity signal-handoffs=true`, see `.claude/memory/` KB note `hailo-dynamic-cropper-roi-injection`). After that pass, dynamic rows replay chip-free like the static ones. Matched-compute comparison = compare each dynamic config against the static grid with the same `mean_tiles_per_frame`.

**Tech Stack:** Python (`hailo_tiling`), GStreamer (cropper per-frame ROI injection via signal-handoff), SQLite, C++ (`gst-hailo-cache` for the Phase-14 full_frame payload), pytest, Hailo HAILO10H.

---

## Context (read once)
- Repo `/home/giladn/tappas_apps/repos/hailo-drone-follow`, branch `tiling-benchmark` (commit freely; NOT main; NO push; submodule changes local).
- `source setup_env.sh` first. Baselines: pytest **261 passed + 8 skipped**, meson **5/5**. Commit trailer: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- HAILO10H present; `[CHIP]` tasks serialise on the one chip.
- **Night-1 deliverables to build on (head `b15623d`):**
  - `hailo_tiling/bench/{config,runner,metrics,grid}.py`, `hailo_tiling/cli/bench.py` (`hailo-tiling-bench`), `hailo_tiling/backends/gst_cropper.py` (GstCropperBackend — has the no-chip pipeline-string path + a verified chip smoke), `scripts/warm_gst_cache.py` + `scripts/_warm_one_fov.sh`.
  - Warmed caches: `.tile_cache/DJI_..._0026_..._fov{50,60,70}__<sha>.sqlite3` (static grid set, 0 misses).
  - Static ablation tables: `dynamic_tiling/runs/ablation_0026_fov{50,60,70}/ablation_table.md`.
  - `tile_norm_to_source_px` (bench/runner) — 0-deviation parity with the cropper crop rule.
- **Night-1 findings to respect:** (1) the warmer deadlocks running multiple grids in ONE process — use a subprocess per grid (Task A1 folds this in). (2) the GST cache keys `frame_idx` **per tile-buffer**, not per source frame — the bench already auto-detects + uses crop-ordered replay; dynamic rows must use the SAME convention.
- **Scheduler:** `dynamic_tiling/scheduler.py` `TileScheduler(discovery_period=15, discovery_grid=(3,2), recovery_grid=(3,3), max_zoom=2.0)` emits a per-frame crop list (discovery grid on cadence + recovery grid + per-target ROI zoom tile, budget-trimmed). Levers: `hailo_tiling/modifiers/{adaptive_sizing(ASAHI),altitude_zoom,budget_trim}.py`, emitters `{discovery_grid,track_roi,recovery}.py`.
- **Tracker:** dynamic rows need tracker state per frame. Reuse `drone_follow.pipeline_adapter.tracker_factory.create_tracker("byte", ...)` (the production ByteTracker) fed from the per-frame detections, exactly as the scheduler expects. Deterministic given the video.

---

# TRACK A — Chip work (serialised): warmer fix + scale + dynamic warming

## Task A1 — `[no-chip]` — Fold subprocess-per-grid into `warm_gst_cache.py`
The warmer deadlocks running N grids in one process (GStreamer teardown/relaunch). `_warm_one_fov.sh` worked around it externally; fold it in.
- [ ] Step 1: Test `test_warmer_spawns_subprocess_per_grid` — monkeypatch the subprocess runner; assert the warmer invokes one child per grid (not one in-process loop). 
- [ ] Step 2: Run → fails.
- [ ] Step 3: Implement — warmer spawns a fresh `subprocess` per grid (each child warms one grid into the shared cache; idempotent via INSERT OR IGNORE). Remove the in-process multi-grid loop. Keep `_warm_one_fov.sh` working or retire it (your call; note in commit).
- [ ] Step 4: Run → passes; no-chip suite green.
- [ ] Step 5: Commit `warm_gst_cache: spawn a subprocess per grid (fix multi-grid teardown deadlock)`.

## Task A2 — `[CHIP, background]` — Warm two more clips (more ablation coverage)
After A1. Warm clips **0027** and **0029** × `{fov50,60,70}` with the SAME static grid set as 0026 (`1x1,2x2,3x2,3x3,4x3,6x4,8x6,12x9` @ overlap 0.25). One cache per (clip,FOV). Validate each fov50 with a 0-miss static replay before continuing that clip's other FOVs. Record per-cache `stats()` + chip time in the state file. (This is mechanical, fills chip time; lower priority than A3 — but safe.)
- [ ] Acceptance: 6 new cache files (0027/0029 × 3 FOV), each non-empty, static replay 0 misses; static ablation tables generated for at least 0027/0029 fov50.

## Task A3 — `[CHIP]` — Warm 0026 dynamic-config ROI tiles (THE paper-result enabler)
Depends on **B1** (GstCropperBackend per-frame injection) + **B2** (dynamic runner). Run the dynamic configs (`dynamic`, `dynamic+asahi`, `dynamic+altitude_zoom`) over 0026 fov50 (then 60/70) through `CachingBackend(GstCropperBackend)` so their per-frame ROI tiles populate the existing 0026 caches. Deterministic tracker → stable tiles.
- [ ] Acceptance: after the pass, a `--backend replay` run of each dynamic config over 0026 fov50 reports **0 misses**; dynamic rows land in the ablation table.

---

# TRACK B — The dynamic ablation path (highest value)

## Task B1 — `[no-chip → CHIP validate]` — GstCropperBackend per-frame ROI injection
GstCropperBackend currently builds a static `tiles-static` pipeline. Dynamic configs need DIFFERENT tiles per frame. Implement per-frame injection via the dynamic cropper's `identity signal-handoffs=true` upstream hook (KB note `hailo-dynamic-cropper-roi-injection`): attach the frame's `HailoTileROI` list before the cropper so it crops those tiles.
- [ ] Step 1: No-chip test `test_gst_cropper_injects_per_frame_rois` — assert the backend, given a per-frame crop list, builds a pipeline with the signal-handoff injector and attaches the expected ROI count. 
- [ ] Step 2: Run → fails.
- [ ] Step 3: Implement per-frame ROI injection in `gst_cropper.py` (`infer` accepts the frame's crops and injects them as tile ROIs via the handoff; extraction unchanged).
- [ ] Step 4: No-chip test passes.
- [ ] Step 5: `[CHIP]` smoke (run when chip free): inject a 2-tile + a 4-tile frame, assert per-crop dets returned in order. 
- [ ] Step 6: Commit `GstCropperBackend: per-frame dynamic ROI injection (signal-handoff)`.
- **If signal-handoff injection proves unworkable unattended, STOP B1, record findings, and fall back: run dynamic configs by re-launching the static `tiles-static` pipeline per frame with that frame's tiles (slower but correct). Note the fallback in the commit.**

## Task B2 — `[no-chip]` — Dynamic config runner (scheduler + tracker → per-frame tiles → backend)
Extend `hailo_tiling/bench/runner.py`: for `kind="dynamic"` rows, drive `TileScheduler` (+ configured levers) fed by the production ByteTracker over the per-frame detections, producing per-frame tiles; run them via the configured backend (`CachingBackend(GstCropperBackend)` for warm, `ReplayBackend` for chip-free replay); aggregate as for static. Surface `n_misses` (replay path).
- [ ] Step 1: Test `test_dynamic_runner_produces_per_frame_tiles` (MockBackend + a 2-frame stub): assert the scheduler is driven per frame and tiles vary across frames; aggregated dets returned.
- [ ] Step 2: Run → fails.
- [ ] Step 3: Implement the dynamic branch (tracker wiring + scheduler loop + lever application via the config flags).
- [ ] Step 4: Passes; suite green.
- [ ] Step 5: Commit `bench runner: dynamic config path (scheduler + ByteTracker per-frame tiles)`.

## Task B3 — `[no-chip after A3]` — Dynamic-vs-static ablation tables (the result)
Run `hailo-tiling-bench` with the full `default_matrix()` (static + dynamic + levers) against the warmed 0026 caches (post-A3) in replay; emit tables that include a **matched-compute** column (each dynamic row paired with the static grid of equal `mean_tiles_per_frame`, showing the recall delta at equal budget). Commit the regenerated `ablation_0026_fov{50,60,70}` tables.
- [ ] Acceptance: tables contain `dynamic`, `dynamic+asahi`, `dynamic+altitude_zoom` rows with recall/precision vs the 12×9 reference AND a recall-at-matched-budget delta vs the equal-tiles static grid. 0 misses.

---

# TRACK C — Safe parallel work (no-chip; guarantees night progress regardless of A/B)

## Task C1 — `[no-chip]` — Independent review pass over Night-1 commits
Night 1 self-reviewed only (no subagent reviewers). Review the 12 commits `a5f2938..b15623d` (warmer, bench config/runner/metrics/cli, GstCropperBackend, idempotency, tables). For each: correctness, spec-fidelity, test quality, and the two flagged findings. If `Agent` is available, dispatch a code-quality reviewer per area; else do a thorough manual review. Record findings in `docs/superpowers/reviews/2026-05-31-night1-review.md`; fix any **blocking** correctness issues as small commits; log non-blocking nits.
- [ ] Acceptance: review doc committed; any blocking issues fixed + verified (suites green); nits logged.

## Task C2 — `[no-chip]` — Paper-with-code scaffold (master spec Phase 15, partial)
Create `docs/paper/technical-report.md` skeleton (sections: abstract, problem, method = emitter/modifier scheduler, experimental setup = FOV variants + cache/replay reproducibility, results = ablation table placeholder auto-filled from `ablation_table.md`, discussion, limitations). Add `docs/paper/reproducibility.md` (the chip-free replay recipe: fetch caches → `hailo-tiling-bench --backend replay` → table). Wire a tiny `scripts/render_ablation_into_report.py` that injects the latest 0026 tables into the report's results section. NO results claims invented — pull numbers from the committed tables only.
- [ ] Acceptance: report skeleton + reproducibility doc + renderer committed; renderer reproduces the results section from the real tables.

## Task C3 — `[no-chip → CHIP validate last]` — Phase-14 full_frame detection payload
`hailocachewriter mode=full_frame` records rows but empty `dets_json`/`tiles_json` (known limitation). Wire it like the tile_cache path: read the post-aggregator `HailoROI` detections (already source-frame coords) + the tile list, serialize via the shared `cache_keys` field constants. So `full_frame` caches carry real aggregated dets → unlocks the visualizer/overlay (Phase 12) on real data.
- [ ] Step 1: C++ test (writer) — full_frame mode with seeded ROI dets writes non-empty `dets_json` + `tiles_json`. Step 2: fails. Step 3: implement (mirror `read_tile_dets_json_`; add tiles_json from the ROI tile list). Step 4: meson 5/5. Step 5: `[CHIP]` smoke (when chip free): full_frame post-aggregator over a few frames records non-empty payloads. Step 6: commit + flip the INDEX "Open follow-ups" note for this item to resolved.

---

## Tonight's autonomous execution

**State file:** `docs/superpowers/overnight-manager-state.md` (append per task: SHAs, pending, decisions, blockers). Resume after session-limit at reset + 10 min (use `handling-anthropic-session-limits`). Amend HEAD only on salvage.

**Ordering / one-chip rule:**
1. No-chip first, in parallel-of-effort (single-threaded but interleavable): **A1**, **B1** (no-chip part), **B2**, **C1**, **C2**. These need no chip.
2. **Chip phase (serialised):** B1 chip smoke → **A3** (dynamic warm 0026, the priority chip job) → **B3** (regenerate tables). If A3 finishes with chip time left: **A2** (warm 0027/0029) → **C3** chip smoke.
3. Never run two `[CHIP]` steps at once.

**Priority order (do the high-value paper result first, keep safe work as filler):**
1. A1 → B1 → B2 → A3 → B3  ← **the dynamic-vs-static result (the paper's point)**.
2. C1 (review Night-1) — high value, no-chip, do early/interleaved.
3. C2 (paper scaffold) — no-chip filler.
4. A2 (more clips) + C3 (full_frame payload) — fill remaining chip/no-chip time.

**Hard stops (record + move to other track, don't rabbit-hole):**
- Per-frame ROI injection (B1) fundamentally unworkable → take the per-frame-relaunch fallback (noted in B1); if THAT fails too, stop the dynamic track, leave the static tables as-is, and pour the night into C1/C2/A2.
- Dynamic replay misses its own warmed tiles → crop-key/frame-id mismatch; stop B3, record the exact mismatch.
- Tracker non-deterministic across passes (tiles differ run-to-run) → dynamic caching is unsound; stop, record, keep static.
- Carry-forwards NOT to chase: `letterbox` bug (use stretch), cross-engine text-vs-value.

**Verification:** every task ends with its tests + `meson 5/5` + `pytest` (≥261 passed, grows). Nothing pushed; no `main`; submodule local.

**"Good morning" (priority):**
1. A1 landed; warmer robust. 2. **Dynamic-vs-static tables for 0026 (≥fov50)** — the headline. 3. C1 review doc + any blocking fixes. 4. C2 paper scaffold. 5. A2 extra clips / C3 full_frame as bonus.

There is plenty here for a full night; if everything lands, extend A2 to clip 0032/0033 and warm their static + dynamic caches.

---

## Dependency graph
- A1 → A2, A3. B1 → B2 → A3 → B3. C1, C2 independent (no-chip, any time). C3 independent (chip smoke last).
- Headline path: A1 → B1 → B2 → A3 → B3.
