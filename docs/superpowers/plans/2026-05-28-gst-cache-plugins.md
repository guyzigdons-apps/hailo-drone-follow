# Plan 5: GStreamer cache plugins

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Spec phases:** 8 (GStreamer cache plugin layer) and 14 (hailo-apps-core patches — only the parts that prove strictly unavoidable).

**Branch:** `tiling-benchmark`. No new branches. No pushes.

**Prereqs:**
- Plans 1–4 landed (currently at `192494f`; 164+ Python tests green, `hailo_tiling.cache.SqliteCacheStore` + `CachingBackend` + `ReplayBackend` shipped and stable).
- Pre-flight bit-exact E2E pytest committed at `tests/integration/test_cache_bit_exact_e2e.py` (lands in parallel with this plan; this plan **consumes** it but does not create it). That test defines the correctness contract and produces / refreshes a baseline JSON of expected detections.
- A Hailo board is reachable for the `[CHIP]` tasks. Only one chip exists — `[CHIP]` tasks must run serialised.

## Constraint summary

1. **The new plugins land in `gst-hailo-cache/` at the repo root**, NOT in the `hailo-apps/` submodule. The submodule is reference-only for the meson + ninja build pattern (`hailo-apps/hailo_apps/postprocess/meson.build`, `hailo-apps/install.sh`) and is the model for where the resulting `.so` files install (the system gstreamer-1.0 plugin dir; `pkg-config --variable=pluginsdir gstreamer-1.0` — `/usr/lib/aarch64-linux-gnu/gstreamer-1.0/` on RPi, `/usr/lib/x86_64-linux-gnu/gstreamer-1.0/` on dev).
2. **`hailofilter` `bypass-on-cache-hit` (Phase 14) is the ONE piece that may need to live in `hailo-apps/`.** Spec §7.9 wires the cache-hit bypass via this exact property on the upstream `hailofilter`. Task 12 inspects whether we can fake it with a thin wrapper element in `gst-hailo-cache/` (preferred) or whether we must patch `hailo-apps/hailo_apps/postprocess/cpp/hailofilter` (only if no wrapper covers the contract). Either way, no push of the submodule.
3. **Bit-exact gate.** The `[CHIP]` tasks and any task that touches cache reading/writing semantics MUST end with a rerun of `tests/integration/test_cache_bit_exact_e2e.py` (or its `--dry-run`-equivalent for `[no-chip]` tasks). The check passes iff every detection produced by the GStreamer path matches the Python `CachingBackend`/`ReplayBackend` baseline byte-for-byte. This is non-negotiable — a regression there blocks task completion.
4. **Chip ordering.** `[CHIP]` tasks must NOT run in parallel with each other. `[no-chip]` tasks may run alongside any number of running tasks. The dependency graph below is the authoritative ordering.
5. **`.so` layout.** Source under `gst-hailo-cache/src/`; built as ONE shared library `libgsthailocache.so` registering two GStreamer elements (`hailocachewriter` and `hailocachereader`). Inside that .so we also embed the shared cache I/O code (`tile_cache_db.{h,cpp}`), reusing the spec §7.10 contract. This avoids the relocation/symbol pitfalls captured in the `tappas-53-symbol-relocation` MEMORY note (one .so, one symbol table, no cross-.so static-vs-dynamic mismatch). The library is installed to the system gstreamer-1.0 plugin dir; no extra ld.so config needed.

## Naming clarification (versus spec §7.8 / §7.9)

The spec calls the recorder `hailodet_record` and the replayer `hailonet_cache`. To make it clear these are the in-repo plugins (not vendored from hailo-apps), this plan ships them as `hailocachewriter` and `hailocachereader` inside `libgsthailocache.so`. Functionality, properties, and on-disk format are identical to the spec; only the element class names differ. The plan keeps spec terminology in property names where the spec is explicit (`mode={tile_cache,full_frame}`, `record-cache-hits`, `on-miss`, etc.).

## File layout this plan creates

```
gst-hailo-cache/
  README.md                       # how to build, install, use; rendered as plugin docs
  meson.build                     # top-level meson, mirrors hailo-apps/hailo_apps/postprocess/meson.build
  meson_options.txt               # opt-in for symbol-visibility, opencv usage (none required), debug
  install.sh                      # idempotent build + install entry point (apt deps optional)
  src/
    meson.build                   # builds libgsthailocache.so → gst_plugins_dir
    plugin.cpp                    # GST_PLUGIN_DEFINE — registers writer + reader element classes
    tile_cache_db.{hpp,cpp}       # SQLite open/get_many/put_many/meta_get/meta_put — schema-faithful to hailo_tiling/cache/schema.sql
    gst_hailocachewriter.{hpp,cpp} # writer element (modes: tile_cache, full_frame)
    gst_hailocachereader.{hpp,cpp} # reader element (drop-in for hailonet on the replay path)
    cache_keys.{hpp,cpp}          # crop-rect canonicalisation + ppv key (matches hailo_tiling/cache/hashing.py)
  tests/
    meson.build
    golden/
      ref_cache.sqlite3           # 5-frame, 3-crop golden cache file produced by the Python warmer
      ref_baseline.json           # frame-by-frame expected detection JSON; same source as bit-exact pytest
    test_tile_cache_db.cpp        # gtest / catch2 unit tests for the C++ DB layer
    test_keys.cpp                 # canonicalisation == Python helper
    bench_lookup_latency.cpp      # microbench: 1k point lookups < 1 ms median
    test_round_trip.py            # Python smoke test: writer writes → SqliteCacheStore reads (no chip)
    test_bit_exact_gate.sh        # runs tests/integration/test_cache_bit_exact_e2e.py with `--use-gst-plugins`
```

Inline patches (applied with `git apply --3way` against a clean tree; never committed to the submodule unless absolutely forced):

```
hailo-apps/                       # touched only if Task 12 cannot avoid it
  hailo_apps/postprocess/cpp/hailofilter/   # +bypass-on-cache-hit property (Phase 14, defer-by-default)
```

## Spec mapping per task

| Spec section / phase | Task(s) |
|---|---|
| §7.2 schema (cache key + meta) | 2, 3 |
| §7.8 `hailodet_record` properties (`mode`, `output-file`, `flush-interval-ms`, `batch-size`, `frame-id-source`, `record-empty`, `record-cache-hits`, `hef-id-meta-key`) | 4, 5, 6 |
| §7.8 `full_frame` `frame_results` schema | 6 |
| §7.9 `hailonet_cache` properties (`cache-file`, `hef-path`, `video-id`, `on-miss`, `quantise`) + cache-hit semantics (no tensor emission, `hailo-cache-hit` buffer meta) | 8, 9 |
| §7.10 shared `libhailotile_cache.so` API | folded into 3 (single .so, internal C++ API) |
| §13 success criteria (writer < 100 μs steady-state; reader < 1 ms / crop) | 7, 10, 11 |
| Phase 14 `hailofilter` `bypass-on-cache-hit` | 12 |
| Bit-exact gate (Plans 1–4 ↔ Plan 5 contract) | 6, 11, 12, 14 (and every chip task) |

## Tasks

### Task 1 — `[no-chip]` — `gst-hailo-cache/` scaffold + README + meson skeleton
**Depends on:** none.
**Description:** Create the directory tree under `gst-hailo-cache/`. Write the top-level `meson.build` modeled on `hailo-apps/hailo_apps/postprocess/meson.build` lines 1–49 (project + dependency lookup of `gstreamer-1.0`, `gstreamer-base-1.0`, `gstreamer-video-1.0`, `sqlite3`; resolve `gst_plugins_dir` via `pkg-config --variable=pluginsdir gstreamer-1.0`). Add `meson_options.txt` (one option, `with_bench=true`). Write `src/meson.build` declaring `libgsthailocache.so` with placeholder sources that build (just `plugin.cpp` with a no-op `GST_PLUGIN_DEFINE`). Add an `install.sh` that runs `meson setup build`, `ninja -C build`, `sudo ninja -C build install`, then `rm -f ~/.cache/gstreamer-1.0/registry.*.bin`. The README documents the canonical pipeline placement diagrams from spec §7.8 and §7.9 verbatim.
**Acceptance:** `bash gst-hailo-cache/install.sh` succeeds end-to-end on the dev box. `gst-inspect-1.0 hailocache` shows the (empty) plugin description; the two elements appear in later tasks.
**Bit-exact gate:** N/A (no cache code yet).

### Task 2 — `[no-chip]` — C++ tile-cache DB helper (`tile_cache_db.{hpp,cpp}`)
**Depends on:** 1.
**Description:** Implement the C++ counterpart to `hailo_tiling/cache/store.py`. Header exposes: `open(path, create_if_missing)`, `close()`, `meta_get/meta_put`, `put_many(rows)` (single explicit `BEGIN`/`COMMIT`), `get(frame_idx, x, y, w, h, ppv)`, `get_many(...)`. Row struct: `{frame_idx:int64, crop_x:int32, crop_y:int32, crop_w:int32, crop_h:int32, ppv:int32, dets_json:std::string, ts_epoch:double}`. The .cpp opens with `PRAGMA journal_mode=WAL; PRAGMA synchronous=NORMAL; PRAGMA user_version=1`, then `CREATE TABLE IF NOT EXISTS detections … WITHOUT ROWID` and `meta` matching `hailo_tiling/cache/schema.sql` line-for-line. Reject mismatched `user_version` with a clear error (logged via `GST_ERROR_OBJECT` once the gst wrapper exists; for now `throw std::runtime_error`).
**Acceptance:** `tests/test_tile_cache_db.cpp` covers: open-empty → schema persisted; reopen idempotent; meta upsert; put_many transactional rollback on duplicate PK (same contract as `test_put_many_is_one_transaction` from Plan 4); get / get_many order preservation; mismatched user_version raises. All under `ninja test`.
**Bit-exact gate:** A new round-trip test (`tests/test_round_trip.py`) writes 10 rows via the C++ helper through a tiny test-only binary (`gst-hailo-cache/build/tests/cache_db_cli`), reads them back with `hailo_tiling.cache.SqliteCacheStore.get_many`, asserts dets-JSON is byte-identical to the input.

### Task 3 — `[no-chip]` — `cache_keys.{hpp,cpp}` and frame-id contract
**Depends on:** 2.
**Description:** Mirror `hailo_tiling/cache/hashing.py:canonicalize_crop`. C++ `canonicalize_crop(int32 x, y, w, h, int q)` rounds DOWN to multiples of `q` when `q > 1` (identity otherwise). Also define `frame_id_from_buffer(GstBuffer*, FrameIdSource src)` for `counter` and `pts` modes (spec §7.8 `frame-id-source`); `counter` is a monotonic int driven by the element instance, `pts` uses `GST_BUFFER_PTS(buf) / GST_NSECOND` rounded to a stable integer. Document the rule so the writer and reader compute the same key.
**Acceptance:** `tests/test_keys.cpp` parameterised over `(x, y, w, h, q)` with the same 6 cases as `test_cache_hashing.py`. A Python parity test (`tests/test_round_trip.py::test_keys_match_python`) invokes the C++ helper via a tiny CLI and asserts the 4-tuple matches `hailo_tiling.cache.hashing.canonicalize_crop` for 100 random inputs.
**Bit-exact gate:** rerun `tests/integration/test_cache_bit_exact_e2e.py --keys-only` (a fast subset that the pre-flight pytest exposes for non-chip use). Must pass.

### Task 4 — `[no-chip]` — `hailocachewriter` element skeleton (passthrough only)
**Depends on:** 1, 2, 3.
**Description:** Register a GStreamer element `hailocachewriter` inheriting `GstBaseTransform` with `passthrough=TRUE`. Declare ALL spec §7.8 properties — `mode` (enum `tile_cache`/`full_frame`), `output-file` (string, required), `flush-interval-ms` (uint, default 100), `batch-size` (uint, default 64), `frame-id-source` (enum, default `counter`), `record-empty` (bool, default `true`), `record-cache-hits` (bool, default `false`), `hef-id-meta-key` (string, default `"hailo-hef-sha"`). In `GstBaseTransformClass::transform_ip` for this task, do NOTHING but log the buffer count. The point is to land the plumbing so the pipeline composes before we wire DB writes.
**Acceptance:** `gst-inspect-1.0 hailocache | grep -A3 hailocachewriter` lists all properties with the right types and defaults. A `gst-launch-1.0 videotestsrc num-buffers=10 ! hailocachewriter mode=tile_cache output-file=/tmp/x.sqlite3 ! fakesink` runs to EOS with no errors and creates no file (writes land in Task 5).
**Bit-exact gate:** N/A.

### Task 5 — `[no-chip]` — `hailocachewriter` writer thread + `tile_cache` mode
**Depends on:** 4.
**Description:** Wire the SPSC ring buffer + background writer thread described in spec §7.8 "Thread model". The streaming `transform_ip` reads (a) the frame id (per `frame-id-source`), (b) the upstream-attached crop list (consult `GstHailoBaseCropperDyn` provenance metadata; if absent, fall back to a single full-frame crop), (c) detections attached to each crop's `HailoROI` (set by `hailofilter` upstream), serialises to the §7.2 row shape, pushes onto the ring. The writer thread drains in batches into `tile_cache_db::put_many` under a single transaction, flushing every `flush-interval-ms` OR every `batch-size` rows (whichever first). On EOS, synchronously drain and close. `record-empty=true` (default) emits a row with `dets_json='[]'`. `record-cache-hits=false` (default) skips buffers carrying buffer-meta `hailo-cache-hit=true`. On any background-thread error, post `GstMessage` on the bus and continue accepting buffers; the streaming thread is never blocked.
**Acceptance:** `gst-launch-1.0 videotestsrc num-buffers=30 ! identity ! hailocachewriter mode=tile_cache output-file=/tmp/w.sqlite3 ! fakesink` produces a 30-row SQLite. `python -c "from hailo_tiling.cache import SqliteCacheStore; s=SqliteCacheStore.open('/tmp/w.sqlite3'); print(s.stats())"` reports `n_rows=30`. A microbench in `tests/bench_lookup_latency.cpp` (writer side variant) measures p99 of `transform_ip` < 100 µs over 10 000 buffers.
**Bit-exact gate:** Rerun `tests/integration/test_cache_bit_exact_e2e.py --writer-only --mock-detections` (the pytest exposes a knob that injects a deterministic detection list into the pipeline without needing the chip). The resulting writer cache must match what `hailo_tiling.cache.SqliteCacheStore.put_many` would have written byte-for-byte (same row count, same JSON, same composite-key set).

### Task 6 — `[no-chip]` — `hailocachewriter` `full_frame` mode + `frame_results` schema
**Depends on:** 5.
**Description:** Add the second mode. When `mode=full_frame`, the element runs AFTER `hailodetiler` and writes a different table layout (spec §7.8 / §7.13): `CREATE TABLE frame_results (frame_idx INTEGER NOT NULL, ppv INTEGER NOT NULL, dets_json TEXT NOT NULL, tiles_json TEXT NOT NULL, ts_epoch REAL NOT NULL, PRIMARY KEY (frame_idx, ppv)) WITHOUT ROWID;`. The element reads source-frame-coord detections from the buffer's `HailoROI` and the tile layout used (passed through as `tiles_json` from upstream tile-list metadata — fall back to an empty `[]` and log a warning if the metadata channel is missing; that's the Phase 14 hook). Both modes coexist in the same .so — they branch on `mode`. `record-cache-hits` is documented as ignored in `full_frame` mode.
**Acceptance:** A two-element pipeline (writer in tile_cache mode + writer in full_frame mode) writes two distinct SQLite files in one run, schemas verified by `sqlite3 file.sqlite3 ".schema"`. The full_frame DB is rejected by `SqliteCacheStore.open` (its schema_version is the same `1`, but the read path looks at `detections` only — extend `SqliteCacheStore.open` if necessary OR introduce a sibling `FullFrameStore` reader; pick the lower-blast-radius option). Add `tests/test_round_trip.py::test_full_frame_schema_round_trip`.
**Bit-exact gate:** N/A for `full_frame` (not a tile cache; not replay-keyed); for the `tile_cache` half: rerun the bit-exact pytest as in Task 5.

### Task 7 — `[CHIP]` — wire writer into the canonical pipeline, populate a real cache
> **[SUPERSEDED / SATISFIED]** by `docs/superpowers/plans/2026-05-31-gst-cache-source-pixel-provenance.md`.
> The canonical-pipeline writer run + bit-exact gate now use **source-video-pixel** crop
> keys and a **per-tile GST-live-vs-GST-cached** diff (pre-aggregator), instead of comparing
> GST crops against a Python `ReplayBackend` baseline in cropped-caps space. The Python
> baseline diff is value-exact at float32 only (not byte-text-identical — `%.9g` vs Python
> shortest-repr), so the byte-equal `dets_json` acceptance below is replaced by the GST-vs-GST
> gate (`scripts/cache_gst_replay_gate.py`, `tests/integration/test_cache_gst_replay_gate.py`).
> Kept here for history; not re-run as written.

**Depends on:** 6.
**Description:** Run the spec §7.8 canonical pipeline against ONE FOV variant (`FOV-70` is fine — 5-minute clip is plenty) end-to-end on the Hailo board: `filesrc ! decodebin ! videoconvert ! hailotilecropper_dynamic ! hailonet ! hailofilter ! hailocachewriter mode=tile_cache output-file=run_tile.sqlite3 ! hailodetiler ! hailocachewriter mode=full_frame output-file=run_full.sqlite3 ! fakesink`. Capture both SQLite files. Use the existing tilecropper grid (`3x2`) so the cache content overlaps the Plan 4 warmed cache.
**Acceptance:** Both files exist, are non-empty, `n_rows ≥ 5*6` (30 frames × 6 tiles minimum). `sqlite3 run_tile.sqlite3 "PRAGMA user_version"` returns `1`. No bus errors during the run.
**Bit-exact gate:** YES — run `tests/integration/test_cache_bit_exact_e2e.py --gst-tile-cache=run_tile.sqlite3 --python-baseline=.tile_cache/<FOV-70>__<hef>.sqlite3`. Must report 0 diffs (every `(frame_idx, crop_rect)` in either set is present in the other, with byte-identical `dets_json`).

### Task 8 — `[no-chip]` — `hailocachereader` element skeleton + property surface
**Depends on:** 3.
**Description:** Register `hailocachereader` inheriting `GstBaseTransform` with `passthrough=FALSE`. Declare ALL spec §7.9 properties — `cache-file` (string, required), `hef-path` (string), `video-id` (string), `on-miss` (enum `error`/`drop`, default `error`), `quantise` (uint, default 0). Mirror all of `hailonet`'s public properties so a pipeline can do `s/hailonet/hailocachereader/` with zero other changes — pull the property list from `gst-inspect-1.0 hailonet` and stub them as no-ops (we never run inference; declaring the names is purely for caps/property compatibility). Set sink/src caps to match `hailonet`'s (raw video in, raw video out with HailoTensor meta paths bypassed).
**Acceptance:** `gst-inspect-1.0 hailocache | grep -A40 hailocachereader` lists all spec §7.9 properties AND every `hailonet` property (stubbed). Pipeline `videotestsrc ! hailocachereader cache-file=/tmp/empty.sqlite3 ! fakesink` accepts caps negotiation; on-miss=error throws bus error on the first buffer (because the cache is empty).
**Bit-exact gate:** N/A (no cache read yet).

### Task 9 — `[no-chip]` — `hailocachereader` lookup + cache-hit semantics
**Depends on:** 8, 2.
**Description:** Implement the §7.9 cache-hit path. In `transform_ip`: (1) compute `frame_idx` (same rule as the writer, controlled by a matching `frame-id-source` property), (2) read the upstream crop list, (3) for each crop call `tile_cache_db::get(frame_idx, x, y, w, h, ppv)`, with `ppv` read from the cache `meta` table on first call (cache once on `state_changed_to_PAUSED`). (4) On HIT: attach the cached detections as `HailoROI` children matching the format `hailofilter` produces (use the same `hailo::HailoDetectionPtr` constructor TAPPAS exposes; if pulling that header in is brittle, write the equivalent struct directly into `g_object_set_data` under a documented key — Task 12 reconciles either path), set buffer meta `hailo-cache-hit=true`, emit ZERO `HailoTensor` metas. (5) On MISS: respect `on-miss=error` (post `GST_ERROR` and EOS) or `on-miss=drop` (emit zero detections, set `hailo-cache-hit=false`, push the buffer through). Apply `quantise>0` before lookup (use `cache_keys::canonicalize_crop`).
**Acceptance:** A unit pipeline `appsrc (synthetic frames) ! hailocachereader cache-file=<task7-produced-cache> ! fakesink` with `appsrc` driving the same `frame_idx` sequence as Task 7's writer run produces zero bus errors. A second pipeline with `on-miss=error` against an EMPTY cache fails on buffer 1 with a clean error message.
**Bit-exact gate:** `tests/integration/test_cache_bit_exact_e2e.py --reader-only --cache=run_tile.sqlite3 --baseline=ref_baseline.json`. The pytest replays the reader against the Task-7 cache and asserts each emitted detection (read via a downstream `appsink` that captures `HailoROI` children) matches the Python `ReplayBackend` byte-for-byte.

### Task 10 — `[no-chip]` — Lookup latency microbench + writer steady-state microbench
**Depends on:** 5, 9.
**Description:** Flesh out `tests/bench_lookup_latency.cpp`. Two sub-benches:
- **Reader:** 10 000 `tile_cache_db::get` calls against a 10 000-row DB (random keys, 50% hit). Report median, p95, p99 latency. Spec §7.9 sets the bar at < 1 ms / crop.
- **Writer:** 10 000 `transform_ip` calls in a `hailocachewriter` driven by a synthetic GstBuffer; report streaming-thread p99. Spec §7.8 sets the bar at < 100 µs.

Wire as a meson `benchmark()` (so `meson test --benchmark` runs it). Make failure modes friendly: print the percentiles and PASS/FAIL relative to the bars.
**Acceptance:** `meson test -C build --benchmark` reports both benches PASS on the dev box. Numbers are logged in `gst-hailo-cache/tests/bench-results.txt` (gitignored).
**Bit-exact gate:** N/A.

### Task 11 — `[CHIP]` — bit-exact gate: GST reader replay vs Python ReplayBackend, full pipeline
> **[SUPERSEDED / SATISFIED]** by `docs/superpowers/plans/2026-05-31-gst-cache-source-pixel-provenance.md`.
> The decisive correctness gate is now **GST live vs GST cached**, per-tile, pre-aggregator
> (same pipeline run twice — live `hailonet` vs `hailocachereader`+`hailocachebypass`), which
> sidesteps the cross-engine float-formatting mismatch this task's "byte-for-byte vs Python
> ReplayBackend" acceptance would have hit. Cross-engine equality remains **value-exact at
> float32**, not byte-text-identical. Gate passes with 0 deviations.

**Depends on:** 7, 9, 10.
**Description:** The decisive end-to-end correctness gate. Compose the §7.9 replay pipeline with the production-equivalent post-process layout, using the Task-7 cache: `filesrc ! decodebin ! videoconvert ! hailotilecropper_dynamic ! hailocachereader cache-file=run_tile.sqlite3 hef-path=<same.hef> ! hailofilter bypass-on-cache-hit=true ! hailocachewriter mode=full_frame output-file=replay_full.sqlite3 ! fakesink`. (If `bypass-on-cache-hit` is not yet wired — see Task 12 — substitute a `hailofilter`-bypass wrapper for now and add a TODO; the gate still measures reader correctness.) Independently, run the Python `ReplayBackend` over the SAME cache + same crop sequence and dump per-frame detection lists.
**Acceptance:** `tests/integration/test_cache_bit_exact_e2e.py --gst-replay=replay_full.sqlite3 --python-replay=<baseline>` reports 0 diffs across every frame in the clip.
**Bit-exact gate:** This task IS the gate.

### Task 12 — `[no-chip]` — `hailofilter bypass-on-cache-hit` — wrapper-first, patch-only-if-forced
**Depends on:** 9.
**Description:** Decide between two implementations of spec Phase 14 (a) `hailofilter` patch:
1. **Preferred — wrapper in this repo.** Add a new tiny element `hailocachebypass` inside `libgsthailocache.so` that sits BETWEEN `hailocachereader` and `hailofilter` and SWALLOWS buffers carrying `hailo-cache-hit=true` (passes them straight to its src pad without invoking the postprocess `.so`). Then the canonical pipeline becomes `... ! hailocachereader ! hailocachebypass ! hailofilter ! ...`. This requires NO submodule patch and is the default plan-of-record.
2. **Fallback — patch the submodule.** If a real-world test shows `hailofilter` still attaches output metadata that confuses the downstream pipeline when the buffer carries `hailo-cache-hit`, patch `hailo-apps/hailo_apps/postprocess/cpp/hailofilter/<source>.cpp` to add the `bypass-on-cache-hit` property per spec §7.9. Local branch on the submodule only; never pushed. Document the diff in `gst-hailo-cache/docs/hailofilter-patch.md`.

Run a small experiment first: build option (1) and test against the Task-7 cache + a 100-frame clip. If detections at the sink match the Python baseline, ship (1). If not, escalate to (2).
**Acceptance:** Whichever path lands, `gst-inspect-1.0 hailocachebypass` (option 1) or `gst-inspect-1.0 hailofilter | grep bypass-on-cache-hit` (option 2) is the verification. The new element/property is documented in `gst-hailo-cache/README.md`.
**Bit-exact gate:** Rerun `tests/integration/test_cache_bit_exact_e2e.py --full-replay --gst-replay=replay_full.sqlite3` with the wrapper/property active. Must match the Python `ReplayBackend` baseline.

### Task 13 — `[no-chip]` — `install.sh` upgrade: idempotent + GStreamer registry cache flush
**Depends on:** 4.
**Description:** Harden the install.sh from Task 1. Add (a) detection of build state (`if [[ -d build ]]; then meson setup --reconfigure build; else meson setup build; fi`), (b) sudo for the install step only, (c) `rm -f ~/.cache/gstreamer-1.0/registry.*.bin` after install (lesson from `.claude/memory/hailotilecropper_dynamic.md` — stale registry causes the wrong .so to load), (d) a `--uninstall` mode that removes `${GST_PLUGINS_DIR}/libgsthailocache.so` and clears the registry, (e) a pre-flight check that `sqlite3 --version >= 3.31` (matching Plan 4 §Pre-flight note about `WITHOUT ROWID` requirements), (f) a `--check` mode that just runs `gst-inspect-1.0 hailocache` and prints the list of elements. The script must be safe to re-run, including after a partial failure.
**Acceptance:** `bash gst-hailo-cache/install.sh && bash gst-hailo-cache/install.sh` (run twice back-to-back) — second run is a fast no-op or rebuild-of-changes-only, no errors. `bash gst-hailo-cache/install.sh --uninstall && gst-inspect-1.0 hailocache` exits non-zero (plugin removed). `bash gst-hailo-cache/install.sh --check` lists `hailocachewriter`, `hailocachereader`, and (if Task 12 path 1) `hailocachebypass`.
**Bit-exact gate:** N/A.

### Task 14 — `[CHIP]` — end-to-end smoke: live → cache → replay → bit-exact diff
> **[SUPERSEDED / SATISFIED]** by `docs/superpowers/plans/2026-05-31-gst-cache-source-pixel-provenance.md`.
> The end-to-end live→cache→replay→diff loop is now realized by the per-tile GST-vs-GST
> replay gate (`tests/integration/test_cache_gst_replay_gate.py`, `HAILO_CHIP=1`), which
> proves the cached pass reproduces the live pass exactly at the per-tile (pre-NMS) point.
> The `live_full == replay_full` byte-equal assertion against a Python baseline is replaced
> by that GST-vs-GST gate; the Python path is value-exact (float32), not text-identical.

**Depends on:** 11, 12, 13.
**Description:** The big stitching task. Reuse `tests/integration/test_cache_bit_exact_e2e.py` as the driver. Sequence: (a) run live pipeline against a 30-second clip on the chip, producing `live_tile.sqlite3` + `live_full.sqlite3`; (b) run replay pipeline against `live_tile.sqlite3`, producing `replay_full.sqlite3`; (c) run Python `ReplayBackend` over `live_tile.sqlite3` independently, producing `python_replay.json`; (d) assert `live_full.sqlite3 == replay_full.sqlite3` (frame-level, dets_json byte-equal) AND `replay_full.sqlite3 ≡ python_replay.json` (detection-level, JSON canonicalised). Add a `make e2e` Makefile target wiring it. No new C++ — this is integration glue.
**Acceptance:** `make e2e` exits 0. The pytest's HTML/markdown summary lists the per-frame diff count: must be 0. Add a one-paragraph results note to `gst-hailo-cache/README.md` ("verified on \<chip-arch> with \<hef> against \<clip>").
**Bit-exact gate:** This task IS the final integration gate. If it fails, find the root cause before declaring Plan 5 done.

### Task 15 — `[no-chip]` — Docs + MEMORY.md update + INDEX flip + Plan 5 close-out
**Depends on:** 14.
**Description:** (a) Add `.claude/memory/gst-hailo-cache.md` capturing the gotchas learned during the plan (single-.so rationale, registry-cache flush, the `bypass-on-cache-hit` wrapper-vs-patch decision, microbench numbers, the bit-exact-gate pattern). (b) Update `.claude/memory/MEMORY.md` to reference it. (c) Update `docs/superpowers/plans/INDEX.md`: flip Plan 5 from `in flight` → `done`. (d) Append a short Plan 5 retrospective section to the spec doc — NOT modifying the spec body, just appending an "Implementation notes" section that records actual property names (e.g. `hailocachewriter` vs spec's `hailodet_record`).
**Acceptance:** `git diff --stat` shows the new memory file, the MEMORY.md and INDEX.md edits, and the spec appendix. All previous tests still pass.
**Bit-exact gate:** Rerun the full `tests/integration/test_cache_bit_exact_e2e.py` one last time before the close-out commit; must pass.

## Dependency graph

```
Task 1 ── Task 2 ── Task 3 ── Task 4 ── Task 5 ── Task 6 ── Task 7[CHIP]
                              │           │
                              │           └── Task 10
                              │
                              └── Task 8 ── Task 9 ── Task 11[CHIP] ── Task 14[CHIP] ── Task 15
                                              │                            ▲
                                              └── Task 12 ─────────────────┤
                                                                           │
                                Task 13 ──────────────────────────────────┘
```

Per-task `depends-on` (authoritative):
- 1: none
- 2: 1
- 3: 1
- 4: 1, 2, 3
- 5: 4
- 6: 5
- 7: 6 (CHIP)
- 8: 3
- 9: 8, 2
- 10: 5, 9
- 11: 7, 9, 10 (CHIP)
- 12: 9
- 13: 4
- 14: 11, 12, 13 (CHIP)
- 15: 14

Chip ordering (serial): Task 7 → Task 11 → Task 14. Nothing else runs on the chip. All other tasks are `[no-chip]` and can interleave freely with chip tasks or each other.

## Bit-exact gate — operational notes

- Pre-flight pytest (`tests/integration/test_cache_bit_exact_e2e.py`) lands in PARALLEL with this plan; this plan **does not** create it but treats it as authoritative once present.
- Every `[CHIP]` task ends with the gate.
- For `[no-chip]` tasks touching cache code (2, 3, 5, 6, 9, 12), the gate runs in `--mock-detections` mode where the pytest injects a deterministic detection sequence into the pipeline via a custom probe — no chip access needed, but the writer/reader side of the contract is still verified.
- Tasks that do NOT touch cache code (1, 4, 8, 10, 13, 15) skip the gate.
- The gate's PASS criterion is `0 diffs` reported by the pytest. Anything else blocks task completion.

## Open questions / risks

1. **Upstream crop-list metadata channel.** Spec §3.5 promises "a pass-through ROI list metadata channel" but Phase 14 places it in the same `hailo-apps-core` PR as `bypass-on-cache-hit`. Task 5 needs the list to compute writer rows; today `hailotilecropper_dynamic` doesn't expose it (see MEMORY note). **Proposed default:** the writer reconstructs the crop list from the upstream `identity` signal that drives the tilecropper (the existing `tile_setter` pattern), or — failing that — re-derives the crop list from the `cropping-period`/`tiles-static` properties of the tilecropper. The fallback path is good enough for the bit-exact gate as long as the writer and the Python warmer use the same reconstruction recipe. If the gate still fails because of a 1-pixel-rounding drift, fall back to `quantise=4`.

2. **TAPPAS `HailoROI` struct layout drift.** Task 9 attaches cached detections to the buffer in the format `hailofilter` would produce. The `tappas-53-symbol-relocation` MEMORY note warns of ABI churn. **Proposed default:** use the public `hailo_objects.hpp` API the tilecropper itself uses; if that header is not reachable from this repo, vendor the minimum subset into `gst-hailo-cache/src/third_party/` and document. Do NOT link against TAPPAS .so files directly (avoid Task 12 path 2 entanglement).

3. **`ppv` source of truth.** Spec §7.2 says `ppv` is "incremented when post-processing changes". The writer (Task 5) reads it from the buffer-meta key (`hef-id-meta-key`) — but who sets it? **Proposed default:** the upstream `hailofilter` is the natural setter (it owns the postprocess version); until Phase 14 lands, the writer reads `ppv` from an env var `HAILO_TILE_CACHE_PPV` (default 1) and stores it into the cache `meta` table on first write. The reader (Task 9) reads `ppv` exclusively from the cache's `meta` table.

4. **Plan 4's `SqliteCacheStore` already exists in Python.** Task 2 builds a C++ helper that produces files that `SqliteCacheStore.open` accepts. The schemas are identical by construction (Task 2 mandates this) — but the test-only `cache_db_cli` shim used in `test_round_trip.py` should NOT be installed; mark it `install: false` in the meson tests subdir.

5. **`record-cache-hits` interaction with `full_frame` mode.** Spec §7.8 says the property is ignored in `full_frame` mode but does not say whether it should warn. **Proposed default:** Task 6 logs a `GST_INFO` once per pipeline start when `mode=full_frame && record-cache-hits=false` was explicitly set, then ignores.

6. **One-chip serialisation enforcement.** The plan declares the ordering but doesn't enforce it. **Proposed default:** the dispatcher / executor reads the `[CHIP]` tags and acquires `/var/run/hailo_chip.lock` (or similar) before starting any chip task. This belongs in the harness, not in the plan.

### Critical Files for Implementation
- /home/giladn/tappas_apps/repos/hailo-drone-follow/gst-hailo-cache/src/gst_hailocachewriter.cpp
- /home/giladn/tappas_apps/repos/hailo-drone-follow/gst-hailo-cache/src/gst_hailocachereader.cpp
- /home/giladn/tappas_apps/repos/hailo-drone-follow/gst-hailo-cache/src/tile_cache_db.cpp
- /home/giladn/tappas_apps/repos/hailo-drone-follow/gst-hailo-cache/meson.build
- /home/giladn/tappas_apps/repos/hailo-drone-follow/gst-hailo-cache/install.sh
