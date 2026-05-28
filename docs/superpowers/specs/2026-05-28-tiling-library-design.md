# `hailo_tiling` — Reusable Tiling Library & Ablation Framework

**Date:** 2026-05-28
**Status:** Draft for review
**Builds on:**
- `dynamic_tiling/` (current working scheduler — promoted to library form by this spec)
- `tiling_benchmark/` (static baselines + GT pipeline — kept as a sibling consumer)
- `docs/research/2026-05-27-industry-tiling-drone-tracking.md` (research catalogue driving the lever set)
- `docs/superpowers/specs/2026-05-27-dynamic-tiling-design.md` (predecessor; ROI-tile + decimated grid v1)

## 1. Problem & Goal

The current `dynamic_tiling/` package proved that a budget-aware track-guided scheduler can beat static dense tiling at equal compute. The 2026-05-27 industry research catalogue identified ~10 additional published techniques (ASAHI, altitude-aware tiling, tile memory, CMC, spatial recovery search, …) that may further improve accuracy at the same budget. **We don't know which of these matter on our hardware and footage.** We need an architecture that lets us *implement each as an isolated lever, ablate them independently, and publish the result*.

Secondary goal: the library should be **reusable** — both as a pure-Python standalone runtime and via GStreamer plugins — so it ships as `hailo_tiling`, a piece of generic Hailo code with a "paper with code" release.

**Goal (one sentence):** Refactor `dynamic_tiling/` into a modular `hailo_tiling/` library whose scheduler is built from composable emitter+modifier classes, so each research lever becomes a swappable component for ablation, while the same library drives both a standalone CLI and a GStreamer pad-probe adapter.

## 2. Scope

**In scope (v1):**
- Library scaffold (`hailo_tiling/` top-level package in this repo)
- Emitter+Modifier scheduler architecture (Variant B)
- `TelemetryProvider` ABC + `MavsdkTelemetry` + `StaticTelemetry` + `RecordedTelemetry`
- `InferenceBackend` ABC + `HefBackend` (existing), `GstCropperBackend` (new), `CachingBackend` (decorator), `ReplayBackend` (chip-free replay from cache)
- Aggregator with composable `BoundaryStripFilter` and a `DetectionMemory` interface (no-op default in v1; functional carry-forward implementation deferred to v2)
- Ablation harness CLI (`hailo-tiling-bench`) — sweeps a matrix and emits per-config `frames.json`
- GStreamer cache plugins in `hailo-apps-core`: `hailodet_record` (two modes: `tile_cache` and `full_frame`) + `hailonet_cache` (replay) + small `hailofilter` `bypass-on-cache-hit` patch + detiler metadata hooks
- Source data prep: extension to `tiling_benchmark/prepare_video.py` to emit three FOV variants (`FOV-70` native, `FOV-60`, `FOV-50`) at 4K from each 6K DJI Mavic 4 Pro source clip
- Pi Global Shutter real-hardware variant (`RPI-GS`) and DJI 6× tele variant (`DJI-TELE-12`) recorded and warmed into the paper-reported ablation matrix alongside the emulated FOV variants — five columns total: `FOV-70 / FOV-60 / FOV-50 / RPI-GS / DJI-TELE-12`
- Geospatial telemetry capture: live `MavsdkTelemetry` persistence into a `telemetry` table; `hailo-tiling-import-ulg` for PX4 ULG files; `hailo-tiling-import-srt` for DJI SRT sidecars
- Visualizer / offline overlay renderer consuming the `full_frame` SQLite output, with drone-path-on-map overlay (`hailo-tiling-view`, `hailo-tiling-overlay`)
- Bonus task (best-effort, gated by data collection): DJI optical-zoom **maximum-range** demonstration with targets at measured slant distances at 1× / 2.5× / 6× hardware zoom. Independent of the `DJI-TELE-12` main-matrix column above (which uses everyday research footage)
- Paper-with-code release artifacts: license, reproducibility recipe, technical report, citation, public reference data

**Lever implementations included in v1:** the levers already in `dynamic_tiling/scheduler.py` (decimated discovery, ROI tile, recovery grid, motion-predicted placement) plus two new ones to validate the architecture: **ASAHI adaptive slice sizing** and **altitude-gated zoom** (both small implementations that exercise different parts of the framework).

**Tracker improvements are out of scope (v1).** The `ByteTrackAdapter` wraps the existing tracker constructed via `drone_follow.pipeline_adapter.tracker_factory.create_tracker("byte", ...)` so behaviour matches production. CMC, BoT-SORT, FOLT, and innovation-based KF go into a follow-up spec for the `hailo_tiling/tracking/` subpackage. The `tracking/` subpackage exists in v1 but only ships the `Tracker` ABC and `ByteTrackAdapter`.

**Other deferred items:**
- Attribute-filtered low-res ReID (separate spec under `reid_analysis/`)
- Spatially-weighted recovery search and tile carry-forward — architecture-supported, implementation in v2 once the v1 ablation tells us how much headroom is left for them
- Custom GStreamer element wrapping the whole scheduler (the pad-probe adapter is the v1 GStreamer story)

## 3. Architecture

### 3.1 Package layout

```
hailo_tiling/
  __init__.py
  types.py             # CropRect, Det, TrackState, FrameContext, Telemetry snapshot
  scheduler.py         # TileScheduler (composes emitters + modifiers)
  budget.py            # BudgetMeter (kept from dynamic_tiling)
  emitters/
    __init__.py
    discovery_grid.py  # DiscoveryGridEmitter (decimated full-frame grid)
    track_roi.py       # TrackROIEmitter (predicted-bbox ROI tile)
    recovery.py        # RecoveryGridEmitter (uniform; gaussian-weighted variant in v2)
  modifiers/
    __init__.py
    adaptive_sizing.py # AdaptiveSliceSizingModifier (ASAHI; reshapes discovery grid)
    altitude_zoom.py   # AltitudeZoomModifier (gates ROI tile's zoom from telemetry)
    budget_trim.py     # BudgetTrimModifier (final stage; drops excess tiles)
  aggregator/
    __init__.py
    nms.py             # per-class NMS
    boundary_strip.py  # BoundaryStripFilter (proven in tiling_benchmark/)
    memory.py          # DetectionMemory ABC + NoOpMemory (default); v2 adds CarryForwardMemory
    aggregator.py      # Aggregator (composes the above)
  telemetry/
    __init__.py
    provider.py        # TelemetryProvider ABC
    static.py          # StaticTelemetry
    mavsdk.py          # MavsdkTelemetry (optional import; gracefully degrades)
    recorded.py        # RecordedTelemetry (JSONL replay)
  backends/
    __init__.py
    backend.py         # InferenceBackend ABC
    hef.py             # HefBackend (direct HailoRT; lifted from dynamic_tiling/inference.py)
    gst_cropper.py     # GstCropperBackend (drives hailotilecropper_dynamic)
    caching.py         # CachingBackend (decorator; SQLite-backed tile-level cache)
    replay.py          # ReplayBackend (cache-only; raises on miss; for chip-free runs)
  cache/
    __init__.py
    store.py           # SqliteCacheStore (open, get, put_many, vacuum, stats)
    schema.sql         # CREATE TABLE statements, schema_version constant
    hashing.py         # file SHA-256, crop-rect canonicalisation
  tracking/
    __init__.py
    tracker.py         # Tracker ABC, TrackState
    bytetrack.py       # ByteTrackAdapter (current tracker)
  cli/
    __init__.py
    run.py             # `hailo-tiling-run` — single-config standalone runner
    bench.py           # `hailo-tiling-bench` — ablation matrix harness
    score.py           # `hailo-tiling-score` — frames.json vs GT
    warm.py            # `hailo-tiling-warm-cache` — pre-compute crops to cache

# In hailo-apps-core (separate repo, separate releases):
hailo-apps-core/
  core/hailo/plugins/cache/
    libhailotile_cache.{h,cpp}    # shared C++ lib: SQLite open/get/put + schema
    gst_hailodet_record.cpp        # GStreamer element: passthrough recorder
    gst_hailonet_cache.cpp         # GStreamer element: hailonet drop-in
    meson.build
    tests/

docs/
  paper/
    technical-report.md  # the paper draft
    figures/
    references.bib
```

`dynamic_tiling/` is removed; its callers (drive video, `compare_baselines.py`) move to `hailo_tiling`. `tiling_benchmark/` stays, but its analysis tools (`analyze_pxt.py`, `overlay_viewer.py`) become consumers of the shared `frames.json` format. The GStreamer cache plugins live in `hailo-apps-core` from day one; their development happens via the existing hailo-apps submodule in this repo (`./hailo-apps/`), with PRs upstreamed as they stabilise.

### 3.2 The Emitter / Modifier protocol

```python
class TileEmitter(Protocol):
    """Produces a list of CropRects given the current frame and track state."""
    name: str
    def emit(self, ctx: FrameContext, track: TrackState,
             telemetry: TelemetrySnapshot) -> list[CropRect]: ...

class TileModifier(Protocol):
    """Mutates the working tile list before submission. Pure-function ideal."""
    name: str
    def modify(self, tiles: list[CropRect], ctx: FrameContext, track: TrackState,
               telemetry: TelemetrySnapshot) -> list[CropRect]: ...

class TileScheduler:
    def __init__(self, emitters: list[TileEmitter], modifiers: list[TileModifier]): ...
    def decide(self, ctx, track, telemetry) -> list[CropRect]:
        tiles = []
        for e in self.emitters: tiles += e.emit(ctx, track, telemetry)
        for m in self.modifiers: tiles = m.modify(tiles, ctx, track, telemetry)
        return tiles
```

Every emitter / modifier carries a `.name` so the ablation harness can record which lever was active per row. `BudgetTrimModifier` is always the last modifier; the order of upstream modifiers is configurable.

### 3.3 Telemetry boundary

`TelemetryProvider` is the only thing that knows what MAVSDK is. Both tiling modifiers (`AltitudeZoomModifier`) and tracker components (future CMC) read from the *same provider instance* but consume different methods. This keeps the library MAVSDK-free at install time — `MavsdkTelemetry` is an optional extras-require.

```python
class TelemetryProvider(ABC):
    @abstractmethod def snapshot(self, t: float) -> TelemetrySnapshot: ...

@dataclass(frozen=True)
class TelemetrySnapshot:
    altitude_agl_m: float | None
    yaw_rate_rad_s: float | None
    velocity_world: tuple[float, float, float] | None
    attitude_quat: tuple[float, float, float, float] | None
    timestamp: float
```

Each field is `Optional` so consumers can gracefully degrade when a stream is missing (e.g. offline runs).

### 3.4 Inference backend boundary

`InferenceBackend.infer(frame, crops) → list[list[Det]]` is the seam between policy (scheduler) and mechanism. There are two concrete inference paths:

- **`GstCropperBackend` — canonical (research and production).** Drives the same GStreamer pipeline as production (`hailotilecropper_dynamic` → `hailonet` → post-process → detiler) so every paper-reported result comes from the same code path the drone runs. This is the default for ablation runs.
- **`HefBackend` — native HailoRT, dev/debug only.** Faster Python iteration, easier breakpointing. Not used for paper-reported numbers (Section 7.11 has the rationale).

The cache (Section 7) layers on top via two complementary mechanisms:
- **Python:** `CachingBackend` decorator wraps any `InferenceBackend`; `ReplayBackend` is chip-free.
- **GStreamer:** a `hailodet_record` element (passthrough recorder, production-ready) generates cache files; a `hailonet_cache` element (drop-in `hailonet` replacement) consumes them in research. Both share the SQLite schema with the Python layer.

### 3.5 GStreamer integration

A `hailo_tiling.gst.adapter.TileSchedulerProbe` class drives `hailotilecropper_dynamic` via an `identity signal-handoffs=true` upstream hook (mechanism already understood — see `concepts/hailo-dynamic-cropper-roi-injection`). The detiler-plugin patch (separate PR to `hailo-apps-core`) adds:
- A `tile-id` property on each emitted detection metadata buffer
- A pass-through ROI list metadata channel (so downstream consumers can render the scheduled tile layout)

The Python `Aggregator` consumes that metadata and runs NMS / boundary-strip / memory in Python — the detiler stays minimal.

## 4. Data Flow (per frame)

```
        ┌─────────────────────────────┐
        │ Frame source (camera/file)  │
        └──────────────┬──────────────┘
                       │  raw frame + timestamp
                       ▼
        ┌─────────────────────────────┐         ┌────────────────────┐
        │ TelemetryProvider.snapshot  │◀────────│ MAVSDK / static /  │
        └──────────────┬──────────────┘         │ recorded           │
                       │  TelemetrySnapshot     └────────────────────┘
                       ▼
        ┌─────────────────────────────┐
        │ TileScheduler.decide(       │◀── TrackState (from previous frame)
        │   emitters + modifiers)     │
        └──────────────┬──────────────┘
                       │  list[CropRect]
                       ▼
        ┌─────────────────────────────┐
        │ InferenceBackend.infer      │
        └──────────────┬──────────────┘
                       │  list[list[Det]] (tile-local)
                       ▼
        ┌─────────────────────────────┐
        │ Aggregator.aggregate        │
        │  (map + NMS + boundary +    │
        │   memory carry-forward)     │
        └──────────────┬──────────────┘
                       │  list[Det] (source-frame normalized)
                       ▼
        ┌─────────────────────────────┐
        │ Tracker.update              │
        └──────────────┬──────────────┘
                       │  TrackState
                       ▼
            (consumer: drone-follow,
             frames.json writer, viewer)
```

## 5. Error Handling

- Missing telemetry fields → snapshot has `None`; downstream modifiers no-op (e.g., `AltitudeZoomModifier` falls back to a fixed `max_zoom`).
- `MavsdkTelemetry` import fails → `ImportError` raised lazily on first use; library otherwise loads.
- Backend inference failure → the affected tile yields zero detections; aggregator continues with the rest. Failure is logged with the CropRect that triggered it.
- Budget exhausted → `BudgetTrimModifier` truncates the tile list deterministically (emitter order is the priority order; ROI > discovery > recovery is the recommended default).
- Detiler-plugin patch unavailable upstream → `GstCropperBackend` falls back to a Python pad probe that reconstructs provenance from CropRect-list order. The standalone path is unaffected.

## 6. Testing

Three layers:

1. **Unit tests** (`tests/`) for every emitter, modifier, aggregator stage, and telemetry implementation. Each emitter has a "no telemetry" test (degraded mode) and a "happy path" test. The current `dynamic_tiling/tests/` is the seed.
2. **Replay-based integration tests** using `ReplayBackend` against a small committed reference cache (covers a handful of frames + crops). Fast (< 5 s for a full ablation matrix), no chip required, runs in CI.
3. **Hardware acceptance test** — one canonical config + one ablation row run end-to-end against a known HEF + video, asserting per-target recall lands within a tolerance of the recorded reference. Not in CI; runs on the dev RPi via a separate `make accept` target.

## 7. Inference Cache

Re-running inference for every ablation row is wasteful: most tiles (especially the discovery-grid full-frame tiles) are deterministic functions of `(video, frame_idx, crop_rect, hef)` and produce byte-identical detections every run. A tile-level cache lets us pay the inference cost once per unique `(video, hef, crop)` tuple and then ablate aggregator/modifier/policy choices for free.

### 7.1 Goals

- **Hit the chip at most once per unique crop, per HEF, per video.** Re-runs of the same matrix are free; new matrices that share tiles with old ones inherit the hits.
- **Be the same code path whether or not the cache exists.** No `if cache: ... else: ...` branches in policy code — `CachingBackend` wraps any `InferenceBackend` and is invisible to the scheduler.
- **Be distributable.** A single file (per video × HEF) we can upload to Zenodo / S3 alongside the paper, so reviewers without a Hailo chip can reproduce every ablation row by running `ReplayBackend` over the published cache.
- **Be inspectable.** A regular SQLite file. `sqlite3 cache.db "SELECT COUNT(*) FROM detections;"` should just work.

### 7.2 Storage format — SQLite (one file per (video, HEF) pair)

```
.tile_cache/
  <video_sha256[:16]>__<hef_sha256[:16]>.sqlite3
  index.json          # human-readable: filename → {video_path, hef_path, video_info, hef_info, schema_version, n_rows}
```

Schema (single table, `WITHOUT ROWID` for fast point lookups; composite primary key):

```sql
PRAGMA user_version = 1;        -- schema_version; bump on breaking change
PRAGMA journal_mode = WAL;       -- safe concurrent reads while a row is warming the cache

CREATE TABLE detections (
  frame_idx    INTEGER NOT NULL,
  crop_x       INTEGER NOT NULL,
  crop_y       INTEGER NOT NULL,
  crop_w       INTEGER NOT NULL,
  crop_h       INTEGER NOT NULL,
  ppv          INTEGER NOT NULL,        -- post-process version (incremented when post-proc changes)
  dets_json    TEXT    NOT NULL,        -- JSON array of {cls, score, x, y, w, h} (tile-local normalized)
  ts_epoch     REAL    NOT NULL,        -- when this row was written (debugging / cache age)
  PRIMARY KEY (frame_idx, crop_x, crop_y, crop_w, crop_h, ppv)
) WITHOUT ROWID;

CREATE TABLE meta (k TEXT PRIMARY KEY, v TEXT NOT NULL);
-- meta keys: schema_version, video_sha256, video_path, video_fps, video_w, video_h,
--            hef_sha256, hef_path, hef_input_shape, score_floor, created_at, hailort_version
```

**Key decisions:**

- **One DB per (video, HEF) pair.** `video_sha` and `hef_sha` are *implicit* in the filename, not columns, which keeps rows small and lookups index-only. A single cache directory can hold many files.
- **`dets_json` is TEXT.** Human-inspectable, gzip-compresses well at the SQLite page level, and a typical entry is < 1 KB. We can switch to MessagePack BLOB if size becomes a problem (estimated < 3 GB for a 30-min 30 fps video × 10 crops/frame; acceptable).
- **`ppv` (post-process version) in the key.** When we change post-processing (e.g., a different on-chip NMS variant or score-floor threshold), bump `ppv`; old entries coexist and we re-compute new ones.
- **Score floor, not score threshold.** Cache at a low floor (default 0.01) so threshold ablations are served from cache. The aggregator applies the operating threshold at retrieval time.

### 7.3 Crop-rect canonicalisation

Cache hits require *exact* equality of `(crop_x, crop_y, crop_w, crop_h)` — a 1-pixel shift would change the input image and the detections. Two safeguards:

- **Deterministic emitters.** Every emitter must produce integer-clamped CropRects given the same inputs (this is already true of `dynamic_tiling/scheduler.py:_grid` and `_roi`). The `CropRect.from_center_width` constructor already rounds.
- **Optional quantisation (off by default).** `CachingBackend(quantise=4)` rounds `(x, y, w, h)` to multiples of 4 px before key lookup. Increases hit rate when an emitter's float→int rounding is unstable across runs. Documented as a "slightly fuzzy" mode; the unquantised default is the paper-correct one.

### 7.4 Flow

```
                       ┌─────────────────────────────┐
  Scheduler emits ───▶ │ CachingBackend.infer        │
  list[CropRect]       │   crops → split into        │
                       │     hits (from SQLite)      │
                       │     misses (forward)        │
                       └────┬───────────────┬────────┘
                            │ hits          │ misses
                            ▼               ▼
                       (cached dets)   wrapped.infer(frame, misses)
                                            │
                                            ▼
                                       store new (crop, dets)
                                            │
                            ┌───────────────┘
                            ▼
                       merge in original order ─▶ list[list[Det]]
```

A single SQLite transaction commits all new rows per `infer()` call. The cache layer is a < 200-line `SqliteCacheStore` plus the `CachingBackend` wrapper.

### 7.5 Cache warming (`hailo-tiling-warm-cache`)

A standalone CLI pre-computes a cache for a fixed crop set. Typical use:

```bash
# Warm dense-grid baselines for one video + HEF (run once, ~6 min for a 30-min clip):
hailo-tiling-warm-cache --video v.mp4 --hef yolov8n.hef \
    --grid 1x1 --grid 2x2 --grid 3x2 --grid 4x3 --grid 6x4 \
    --cache .tile_cache/

# Now any ablation row that uses any of those discovery grids hits the cache:
hailo-tiling-bench --matrix matrix.yaml --video v.mp4 --hef yolov8n.hef \
    --cache .tile_cache/  # transparent; CachingBackend is wired automatically
```

For dynamic ROI tiles (whose CropRect varies with track state), the cache fills opportunistically — first run pays the inference cost, subsequent ablation rows over the same matrix benefit.

### 7.6 Replay-only mode (chip-free reproducibility)

`ReplayBackend` raises on cache miss. Used in CI (where the runner has no Hailo) and by external reviewers reproducing the paper:

```bash
# Download the published cache (single tar, ~100 MB after gzip on the reference clip):
scripts/fetch_reference_cache.sh
# Reproduce the ablation table without any Hailo hardware:
hailo-tiling-bench --matrix matrix.yaml --video reference.mp4 --hef reference.hef \
    --cache .tile_cache/ --backend replay
```

This is a direct paper-with-code reproducibility win: the cache file is the experimental record.

### 7.7 Cache lifecycle

- **No automatic eviction.** Cache grows until `rm -rf .tile_cache/`. Document size (< 100 MB / 5-min clip / 4 grids is the working budget).
- **HEF or video change → new file.** Old caches stay on disk until manually pruned; `hailo-tiling-cache prune --older-than 30d` is a v2 nicety.
- **Schema migration.** `user_version` is checked on open; mismatched schema → reject the file with a clear error message. We don't auto-migrate in v1.
- **Concurrent writers.** WAL mode is safe; we still document "one bench process at a time per cache file" for clarity, and the harness takes an advisory file lock.
- **Determinism check (CI).** A small test runs `HefBackend` twice on the same crop and asserts byte-identical results. If a future HEF flavour proves non-deterministic, we'll need a hash-stability assertion or a switch to "first writer wins" semantics.

### 7.8 GStreamer recorder plugin — `hailodet_record` (production, passthrough)

A production-ready, **standalone** GStreamer element released in `hailo-apps-core` (separate plugin from `hailonet_cache`; separate source file, separate `.so`). Two operating modes, controlled by a `mode` property, with different pipeline placements and output schemas. The same plugin is typically instantiated **twice in a single pipeline**:

```
... ! hailotilecropper_dynamic ! hailonet ! hailofilter ! \
    hailodet_record mode=tile_cache  output-file=cache.sqlite3        ! \
    hailodetiler ! \
    hailodet_record mode=full_frame  output-file=flight_record.sqlite3 ! ...
```

**Mode `tile_cache` (after `hailofilter`, before `hailodetiler`):**
Records **per-crop, tile-local-coords** detections in the Section 7.2 schema. This is the file consumed by `hailonet_cache` for replay; on a cache hit, both inference and postprocess are skipped. Data shape per row: `(frame_idx, crop_x, crop_y, crop_w, crop_h, ppv, dets_json)`.

**Mode `full_frame` (after `hailodetiler`):**
Records **per-frame, source-frame-coord** aggregated detections plus the tile layout that was used. This is the file consumed by the visualizer and the overlay renderer; it is *the* artifact a real-world experiment (e.g., a Pi GS drone flight) leaves behind. Data shape (separate table; see Section 7.13 for the consumer side):

```sql
CREATE TABLE frame_results (
  frame_idx     INTEGER NOT NULL,
  ppv           INTEGER NOT NULL,
  dets_json     TEXT    NOT NULL,   -- aggregated detections in source-frame normalized coords
                                     -- each det carries optional "tile_idx" provenance
  tiles_json    TEXT    NOT NULL,   -- list of CropRects used this frame: [{x,y,w,h,mode}, ...]
  ts_epoch      REAL    NOT NULL,
  PRIMARY KEY (frame_idx, ppv)
) WITHOUT ROWID;
```

The full-frame file is **not** the tile cache. It cannot be replayed via `hailonet_cache` (which keys on tile-local crops). It is the record of "what the pipeline finally produced" — used for offline overlay rendering and the visualizer; it travels with the source video.

**Properties (common to both modes):**
- `mode={tile_cache,full_frame}` (required) — output schema and expected placement
- `output-file=<path>` (required) — SQLite file to append to (or create); separate files per mode
- `flush-interval-ms=100` — flush queued rows every N ms
- `batch-size=64` — flush after N rows queued, whichever comes first
- `frame-id-source={counter,pts}` (default `counter`) — frame indexing strategy (`counter` is robust; `pts` allows wall-clock correlation in live captures)
- `record-empty=true` (default, no opt-out planned) — frames/crops with zero detections **must** be recorded as `dets_json='[]'`. Without these rows, the absence of a row would be indistinguishable from "never inferred", causing every empty frame to trigger a cache miss + re-inference on the next replay run. An empty row is the correct positive signal that "we did look here, there was nothing."
- `record-cache-hits=false` (default; tile_cache mode only) — during a replay run, if a buffer carries `hailo-cache-hit=true`, the recorder skips writing it (the data is already in the cache file). In `full_frame` mode this property is ignored — full-frame recording always happens, including during replay (the visualizer needs the per-frame layout).
- `hef-id-meta-key="hailo-hef-sha"` — buffer-meta key the upstream pipeline sets; recorder reads it and stores into the meta table on first row

**Thread model:** streaming thread does only a lock-free SPSC push to a ring buffer; a background writer thread drains the queue in batches into a SQLite transaction (WAL mode). The streaming pad probe is < 5 μs in the steady state. EOS triggers a synchronous final flush before returning. Errors in the background thread are logged and surfaced via a GstMessage; the streaming pipeline is **never blocked or crashed** by the recorder.

**Why production-ready:** the live drone pipeline can run `hailodet_record` continuously without affecting frame latency. Each flight's cache file becomes:
- An audit trail (what did the drone see?)
- An overlay-rendering input (renderer reads cache + raw video, produces annotated MP4 offline; this can be done on a ground station / laptop)
- A research dataset for ablating policy/aggregator changes against real-flight data

This is the closure-of-the-loop: production telemetry generates research datasets at zero cost.

### 7.9 GStreamer replay plugin — `hailonet_cache` (research, hailonet drop-in)

A drop-in replacement for `hailonet` with identical sink/source caps and the same property surface. Pipeline string change is `s/hailonet/hailonet_cache/`. Released in `hailo-apps-core` (initially as "experimental"; promoted to stable once paper artifacts ship). **Research use only**; not intended for realtime / production pipelines.

**Pipeline placement:**
```
... ! hailotilecropper_dynamic ! hailonet_cache cache-file=flight.sqlite3 hef-path=...hef ! \
    hailofilter bypass-on-cache-hit=true ! hailodet_record record-cache-hits=false ! ...
```

**Cache-hit semantics (critical):**
1. Look up `(frame_idx, crop_rect, hef_sha)` in the cache.
2. **On hit:**
   - Attach the cached detection objects directly to the buffer's `HailoROI` (matching the format `hailofilter` would have produced)
   - Set buffer meta `hailo-cache-hit=true`
   - **Emit no raw tensors.** No `HailoTensor` meta is attached to the buffer; the network output payload does not exist on a cache hit.
   - Push the buffer downstream.
3. **On miss:** behaviour controlled by `on-miss` property (see below). Strict-by-default; this is research-only, so misses are loud errors.

**Required collaborator — `hailofilter` patch:** the downstream `hailofilter` (postprocess) element gains a new property `bypass-on-cache-hit=true` (default `false`, fully backwards compatible). When set, `hailofilter` checks for the `hailo-cache-hit` buffer meta and, if present, **passes the buffer through unchanged** — no `.so` invocation, no NMS, no decode. The detections are already attached upstream. This is the single-property patch that makes the post-postprocess cache architecture work; it lives in the same `hailo-apps-core` PR as the detiler metadata hooks (Phase 10).

**Properties:**
- All of `hailonet`'s public properties (so existing pipelines compose unchanged)
- `cache-file=<path>` (required) — SQLite cache to read
- `hef-path=<path>` — same property name as hailonet; used to compute `hef_sha` for cache key matching
- `video-id=<sha or label>` — identifies the video; auto-derived from source URI if a `hailosource-meta` is present
- `on-miss={error,drop}` (default `error`)
  - `error` — strict research mode; missing cache entry → bus error. Catches stale caches early. Default.
  - `drop` — emit a zero-detection result and continue; useful for noisy datasets where some frames are intentionally absent.
- `quantise=0` — crop-rect quantisation (Section 7.3) for hit-rate vs fidelity tradeoff

**Note:** no `fallback` mode. Cache warming is an explicit step via the production recorder (`hailodet_record`) or `hailo-tiling-warm-cache`. Mixing live inference into a replay path would muddy paper results; if the cache is incomplete, the user re-runs warming, not the ablation harness.

**Performance:** SQLite point query on the `WITHOUT ROWID` composite key is ~10–100 μs. `hailonet` itself is ~30–40 ms per inference (YOLOv8m on Hailo-8L). The cache path is **~300× faster** — well past the user-required "faster than the actual inference" bar. Skipping postprocess saves additional milliseconds on top. Multi-tile pipelines that issue dozens of lookups per frame still complete in well under 1 ms total cache time.

**Backwards compatibility:** because `hailonet_cache` mirrors `hailonet`'s properties and caps, any existing pipeline can swap one for the other with no other changes (modulo the `bypass-on-cache-hit=true` on the downstream `hailofilter`) — so the same ablation matrix runs against both live inference and replayed cache by toggling element names. This is what makes "the GStreamer pipeline is the canonical research flow" actually workable: we don't need a separate Python-only fast-replay loop.

### 7.10 Shared C library — `libhailotile_cache.so`

Both plugins are thin GStreamer wrappers over a shared C/C++ library:

```
libhailotile_cache.so
  ├ open(path, mode) → handle           # SQLite open + schema check
  ├ put_many(handle, rows)              # batched insert, transaction-wrapped
  ├ get(handle, key) → dets             # single-row point lookup
  ├ get_many(handle, keys) → dets[]     # multi-row lookup (one prepared statement)
  ├ meta_get / meta_put                 # key-value meta table accessor
  └ close(handle)                       # flush + close
```

The Python `hailo_tiling.cache.store.SqliteCacheStore` uses the same SQLite schema but goes through `sqlite3` (stdlib) directly — no FFI binding to `libhailotile_cache.so` is required because the schema *is* the API. Both ends produce files that the other can read.

Released as a separate `.deb` in the `hailo-apps-core` release alongside the two plugins.

### 7.11 Why not run research on native HailoRT?

`HefBackend` calls HailoRT directly from Python, which is fast to iterate on but **hides three categories of integration risk** that would surface only in production:

1. **Pre/post-processing divergence.** `hailonet`'s built-in resize / letterbox / NMS configuration is set via element properties; replicating it byte-perfectly in a Python wrapper is fragile. A 1% recall delta between native and GStreamer paths is easy to introduce and hard to debug.
2. **Cropper coordinate semantics.** `hailotilecropper_dynamic` defines exactly how a `CropRect` becomes an HEF input tensor (color space, sub-pixel sampling, boundary handling). OpenCV slicing in `HefBackend` does not match it pixel-for-pixel.
3. **Multi-tile timing.** GStreamer's queue depths, buffer pacing, and threading model exhibit ordering and back-pressure behaviour that Python's tile-at-a-time loop never sees.

`GstCropperBackend` is therefore the **canonical research path**; every ablation row reported in the paper runs through it. `HefBackend` stays in the library for local iteration, unit-test fixtures, and debugging — but its output is *not* paper-quotable.

### 7.12 Full-frame consumers (visualizer + offline overlay renderer)

The `full_frame` mode (Section 7.8) produces a SQLite file with one row per frame: `(frame_idx, dets_json, tiles_json)`. Two consumers:

**Offline overlay renderer (`hailo-tiling-overlay`).** Reads the source video + the `flight_record.sqlite3` file, writes an annotated MP4 with bounding boxes and (optionally) the tile-layout overlay. Same role as `tiling_benchmark/overlay_dets.py` today, but reads from SQLite and supports the tiles overlay. Pure CPU job; runs on the laptop / ground station after a flight.

**Interactive visualizer (`hailo-tiling-view`).** Frame-by-frame stepper that renders source frame + tile rectangles + detections + (optionally) tracker state. Designed to be the successor to `tiling_benchmark/overlay_viewer.py`; reads from `flight_record.sqlite3` so it works without chip access for any recorded run. Drone-follow's existing web UI can adopt the same data layer for post-flight review.

Both consumers operate on **per-frame source-coord data**, so they require nothing from the tile-cache SQLite — full-frame and tile-cache files are independent artifacts of the same flight.

### 7.13 Why not `frames.json`?

The existing per-config `frames.json` outputs are frame-level (after merge / NMS / boundary-strip). They're not reusable across configs because they bake in aggregator choices. The cache is one level deeper — tile-local, pre-aggregator — and is the *right* granularity for "compute once, ablate many."

`frames.json` stays as the per-config output format consumed by `analyze_pxt.py` and `overlay_viewer.py`. The cache feeds the aggregator; the aggregator writes `frames.json`.

## 8. Experimental Setup: Source Data and FOV Emulation

### 8.1 Source clips

**Research / paper-reported results:** 6K footage from a **DJI Mavic 4 Pro** (Hasselblad 4/3 CMOS, 28 mm equivalent, native horizontal FOV ≈ 70°, output 6016 × 3384). Already used in `tiling_benchmark/` today for GT generation. The 6K source is **never read by the inference pipeline directly** — it is preprocessed into per-FOV 4K variants (Section 8.3) and the pipeline reads only those.

**Production / real-world experiments:** **Raspberry Pi Global Shutter Camera** (Sony IMX296, 1456 × 1088, C/CS-mount lens chosen at integration time). Used on the drone in flight via the existing `--input rpi` path. Paper does **not** report numbers from this camera — the resolution, shutter type (global vs the Mavic's rolling), and lens stack differ from the research source. The paper discloses this gap explicitly; the Pi GS path is validation that the system runs end-to-end in production, not a paper-quoted benchmark.

### 8.2 FOV variants for paper results

All paper-reported ablation rows run at 4K output resolution (3840 × 2160) under three labelled effective horizontal FOVs:

| Label    | H-FOV | Crop from 6K        | Output | Equivalent zoom | Physical interpretation                |
|----------|-------|---------------------|--------|-----------------|----------------------------------------|
| `FOV-70` | 70° (native) | full 6016 × 3384 | 4K     | 1.0× (baseline) | wide / "no zoom"                        |
| `FOV-60` | 60°   | center 4963 × 2792  | 4K     | ~1.21×          | medium / mild digital zoom              |
| `FOV-50` | 50°   | center 4007 × 2254  | 4K     | ~1.50×          | narrow / equivalent of a ~1.5× tele     |

Crop dimensions derived from `crop_ratio = tan(FOV_target/2) / tan(70°/2)`. **No variant upscales** — the narrowest (FOV-50) crops to 4007 px wide and downscales by 1.04× to reach 3840. Going narrower than ~48° from this source would require interpolation, which we avoid (per the user requirement: interpolation degrades the results we are trying to measure).

The three-point spread is designed to expose the predicted accuracy-vs-FOV trend: narrower FOV → larger targets in the frame → less benefit from dense tiling, more benefit from the dynamic ROI tile. FOV-70 is where the tiling system should help most; FOV-50 is where the ROI-tile / track-guided emitters earn their keep.

### 8.3 Generating the variants — extension to `prepare_video.py`

`tiling_benchmark/prepare_video.py` is extended to emit per-FOV files in one pass:

```bash
python tiling_benchmark/prepare_video.py clip.MP4 --emit-fov 70,60,50
```

Outputs (alongside the existing rotation-stripped output):

```
clip__fov70.mp4    # 6016x3384 → scaled to 3840x2160 (no crop)
clip__fov60.mp4    # center crop 4963x2792 → scaled to 3840x2160
clip__fov50.mp4    # center crop 4007x2254 → scaled to 3840x2160
```

ffmpeg invocation per variant:

```
ffmpeg -i clip.MP4 \
    -vf "crop=W:H:(in_w-W)/2:(in_h-H)/2,scale=3840:2160:flags=lanczos" \
    -c:v libx265 -crf 18 -preset slow -an \
    clip__fov<N>.mp4
```

`crf 18 -preset slow` (high quality) so the emulation does not introduce codec artifacts that would confound the FOV ablation. Each output file's SHA-256 becomes part of the cache key (Section 7.2 `meta.video_sha256`), so cache entries from the three variants never collide.

### 8.4 Why one file per FOV (not a runtime parameter)

Separate physical files for each FOV variant:

- **Force the tiler to see only the claimed resolution.** The scheduler, the `hailotilecropper_dynamic`, and the ASAHI-style native-resolution tile selection all read width/height from the source buffer; with the 4K MP4 as input there is no codepath that can accidentally peek at the underlying 6K data. Native-tile sizing therefore picks the correct grid for the *paper-reported* resolution.
- **Make each variant independently reproducible.** A reviewer reproducing the paper downloads three files (one per FOV) plus three GT files plus one cache file. The experimental record per FOV is a single SHA-256.
- **Avoid runtime correctness risk.** A runtime FOV-emulation element (live `videocrop` + `videoscale`) is feasible but introduces another moving part in the inference pipeline; for paper-quality results we prefer the static-file approach. A live `hailofov_emulate` element is a non-blocking nice-to-have for interactive experimentation.

### 8.5 GT generation

GT trajectories are generated **per FOV variant** using the existing `GT-12x9-25` dense-tiling flow in `tiling_benchmark/`. Three GT files total (`GT__fov70.frames.json`, `…fov60.frames.json`, `…fov50.frames.json`). The cache file gets warmed once per (FOV variant × HEF), so GT generation amortises across all ablation rows that share the FOV and the HEF.

### 8.6 Real-hardware variant — `RPI-GS` (paper-reported)

The Pi Global Shutter Camera path is **part of the paper-reported results**. Its purpose is to validate that the algorithm transfers to real silicon, a real lens, and a real (global) shutter:

| Label    | Sensor                       | Resolution    | Lens                                                | Focal length | H-FOV (computed) |
|----------|------------------------------|---------------|-----------------------------------------------------|--------------|------------------|
| `RPI-GS` | Sony IMX296 (5.02 × 3.76 mm) | 1456 × 1088   | Arducam M12 telephoto, 20° rated for 1/2.3" sensor  | ~21.7 mm     | **~13°**         |

**Focal-length and FOV derivation.** The lens is rated 20° diagonal on a 1/2.3" sensor (~7.66 mm diagonal), giving focal length `f ≈ 7.66 / (2 · tan 10°) = 21.7 mm`. Applied to the smaller IMX296 sensor: H-FOV = `2 · atan(5.02 / (2 · 21.7)) ≈ 13.2°`. The lens has an M12 mount and ships with a C/CS-mount adapter that fits the Pi GS native C/CS mount. Measured H-FOV (single-checkerboard calibration in `scripts/measure_fov.py`) is recorded alongside the cache file in a sibling `lens_info.json` and stored in the cache `meta` table so a lens swap on the drone is impossible to silently miss in the published results.

**Comparability with the emulated variants.** This lens is **substantially narrower** than any of FOV-70/60/50 — it sits in a different region of the FOV/object-size trade-off. The paper does *not* claim direct equivalence to any emulated variant. Instead, `RPI-GS` extends the FOV axis of the ablation table beyond what the emulated set can reach (Section 8.2 floor was ~48° without upscaling); the trend "tiling helps less as FOV narrows" is the headline claim and `RPI-GS` is the strongest evidence point for the narrow-FOV end.

**Cross-validation with DJI 6× tele.** The Mavic 4 Pro's 168 mm tele camera has H-FOV ≈ 12° on its 1/1.5" sensor — almost the same FOV as the deployed Pi GS rig but with a completely different sensor and lens stack (rolling shutter, larger sensor, ~4K resolution). The DJI 6× tele was previously in the Phase 17 bonus task; the spec promotes it to the main ablation matrix (see Section 8.7 below) precisely because it gives a paired "same FOV, different hardware" data point to `RPI-GS` — much stronger paper evidence than either alone.

**Capture flow:**
- Live flight on the Pi (real PX4, real Hailo, real camera), `--input rpi`
- `hailodet_record mode=full_frame` enabled by default (Phase 16) — produces `flight_record.sqlite3`
- The same flight is also recorded as raw video (`--record`) so the cache can be warmed offline from the recorded MP4, decoupling paper reproducibility from owning a Pi GS rig
- The recorded lens H-FOV, focal length, and sensor format are written into the cache `meta` table and the sibling `lens_info.json`

**Resolution caveat.** At 1.6 MP the Pi GS is well below 4K; the tiler's native-tile sizing picks a different grid (fewer, smaller tiles). The paper reports `RPI-GS` results in its own column — directly comparable on per-target recall and IoU, but not pixel-equivalent to the 4K emulated variants.

### 8.7 DJI optical-zoom variants and maximum-range demo

The DJI Mavic 4 Pro's three-camera optical-zoom system provides two distinct kinds of data for the paper:

| Stage           | Equivalent focal length | H-FOV  | Sensor              | Use in paper                                       |
|-----------------|-------------------------|--------|---------------------|----------------------------------------------------|
| Main (1×)       | 28 mm                   | ~70°   | Hasselblad 4/3 CMOS | Source for `FOV-70/60/50` emulation (Section 8.2)  |
| Med-tele (2.5×) | 70 mm                   | ~28°   | 1/1.3" CMOS         | **Bonus: optical-zoom range demo** (Phase 17)      |
| Tele (6×)       | 168 mm                  | ~12°   | 1/1.5" CMOS         | **Main ablation matrix: `DJI-TELE-12` variant**    |

**Main matrix — `DJI-TELE-12` (promoted from bonus).**
The 6× tele camera's ~12° H-FOV is the natural FOV-comparable cross-validation point for the deployed `RPI-GS` rig (~13°). Capturing it alongside `RPI-GS` gives the paper a paired "same FOV, different hardware" data point that is much stronger evidence than either platform alone: it isolates the algorithm from the platform when reading the narrow-FOV column.

`DJI-TELE-12` is captured at the tele camera's native resolution (no FOV emulation — the optical zoom *is* the FOV change). Same recording protocol as the other DJI clips: MP4 + SRT sidecar.

**Bonus — optical-zoom maximum-range demo (Phase 17).**
Separate experiment, single figure-of-merit, kept as bonus because it requires a coordinated outdoor shoot with targets at measured distances. Records the **same scene at all three optical stages** (1×, 2.5×, 6×) with a person + a car at progressively longer slant ranges. Reports "maximum slant range at which the detector achieves ≥ 0.5 per-target recall over a 5-second window" per zoom stage. Headline result: "with optical zoom, this system detects person-sized targets out to N metres."

The bonus is independent of the main matrix's `DJI-TELE-12` column: even if the bonus shoot doesn't happen in time, `DJI-TELE-12` already lives in the main matrix from the everyday research footage. If the bonus shoot does happen, its 6× footage *also* feeds the `DJI-TELE-12` column, just with the controlled distance metadata.

**Updated main ablation matrix columns (final):**
`FOV-70` · `FOV-60` · `FOV-50` (emulated, 4K) · `RPI-GS` (~13°, real Pi GS) · `DJI-TELE-12` (~12°, real DJI tele). Five columns total; the `RPI-GS` + `DJI-TELE-12` pair anchors the narrow-FOV end and lets the paper claim platform-independence at the narrow end.

### 8.8 Geospatial telemetry capture (drone position + target distance)

For paper-reported runs we capture per-flight telemetry alongside the detection record. The visualizer renders the drone path on a map; the ablation harness can report "distance to detected target" alongside recall metrics. Two ingest paths:

**From PX4 (Pi + real flight, or SITL):**
- **Live: `MavsdkTelemetry` provider already in the spec.** Per-frame telemetry rows are written into a new `telemetry` table in `flight_record.sqlite3` (timestamp + lat + lon + alt_msl + alt_agl + yaw + pitch + roll + vx + vy + vz + source='mavsdk').
- **Offline backup: the `.ulg` file** (PX4's binary flight log). Preserved as an artifact alongside the video. Parsed with `pyulog` by an importer CLI (`hailo-tiling-import-ulg`) to fill any gaps or replace the live capture if it was incomplete.

**From DJI (Mavic 4 Pro research footage):**
- **Primary: the `.SRT` sidecar file** that the drone auto-generates alongside every video clip. Plain text, one entry per second containing GPS lat/lon, height-above-takeoff, datetime, and per-frame camera params (focal length, ISO, shutter, EV, colour temp). PTS-aligned to the MP4. Parsed by `hailo-tiling-import-srt` into the same `telemetry` table.
- **Secondary (only if needed): the `.DAT` flight log** from the drone's internal storage. Encrypted binary; parseable with community tools (`dat2csv`, DROID). Higher-rate attitude/velocity data. Not required for v1 — the SRT is sufficient for drone position on a map and rough distance-to-target estimation.
- **Tertiary: the `.txt` export from DJI Fly app** — human-readable flight summary. Only useful for the paper's data-collection description, not for computation.

**Recommended DJI capture protocol (one-line answer to "what should I save?"):**
> Save every video clip's matching `.SRT` file. That's it for v1. The DJI app saves both into the same folder; just copy both into the research dataset.

**Schema (added to `flight_record.sqlite3`):**

```sql
CREATE TABLE telemetry (
  timestamp_s  REAL NOT NULL,        -- monotonic seconds from flight start
  source       TEXT NOT NULL,        -- 'mavsdk' | 'ulg' | 'srt' | 'dat'
  lat          REAL,                  -- WGS84 degrees, NULL if unknown
  lon          REAL,                  -- WGS84 degrees
  alt_msl      REAL,                  -- meters above mean sea level
  alt_agl      REAL,                  -- meters above ground level (PX4 only; NULL from SRT)
  yaw          REAL,                  -- radians; NULL from SRT
  pitch        REAL,
  roll         REAL,
  vx           REAL,                  -- m/s; NED frame
  vy           REAL,
  vz           REAL,
  PRIMARY KEY (timestamp_s, source)
);
CREATE INDEX idx_telemetry_ts ON telemetry(timestamp_s);
```

**Distance-to-target (derived; v2 in the visualizer, optional for v1 paper):**
Computed at render time from `(drone_lat, drone_lon, alt_agl, yaw, pitch) + camera_FOV + bbox_center_in_frame` assuming flat ground at the take-off elevation. No extra capture required. The visualizer renders an estimated target ground position on a map and a slant-range readout; the ablation harness can optionally include "mean detection range over the flight" as an extra column. This is geometry, not measurement — disclosed accordingly in the paper.

## 9. Ablation Harness

`hailo-tiling-bench` reads a YAML matrix:

```yaml
base:
  emitters:
    - {kind: DiscoveryGrid, period: 15, grid: [3, 2]}
    - {kind: TrackROI, margin: 0.25}
    - {kind: RecoveryGrid, grid: [3, 3], span: 0.4}
  budget: 300
rows:
  - {name: baseline, modifiers: [BudgetTrim]}
  - {name: +asahi,   modifiers: [AdaptiveSliceSizing, BudgetTrim]}
  - {name: +alt,     modifiers: [AltitudeZoom, BudgetTrim]}
  - {name: +both,    modifiers: [AdaptiveSliceSizing, AltitudeZoom, BudgetTrim]}
```

For each row: runs the configured scheduler over a fixed video + telemetry trace, emits `runs/<name>/frames.json` + `runs/<name>/metadata.json` (lever list, git SHA, video hash). At the end, scores all rows against the GT trajectory and writes `runs/ablation_table.md` (and `.csv` for plotting). The table is what the paper reports.

When the cache (Section 7) is warm, the harness transparently serves all hits from disk — a full ablation matrix re-run takes seconds instead of minutes, and ablating aggregator/modifier choices on the same cropper output requires no chip access at all.

## 10. Paper-with-Code Release Workflow

This is a first-class deliverable in v1, not a polish step.

**License:** Apache 2.0. Matches `hailo-apps-core` and `hailo-apps-infra`; commercial-friendly; standard for ML-with-code releases.

**Reproducibility recipe:**
- One reference video + one reference HEF + one reference telemetry trace + one pre-computed **inference cache** (Section 7), all referenced by SHA-256 in the repo (downloaded by `scripts/fetch_reference_data.sh`).
- Public reference video: one of the captured DJI Mavic 4 Pro clips, released by the project under a permissive license (CC-BY-SA recommended) and uploaded to Zenodo alongside the published cache. The same clip used internally — no second-source variability between internal and public results.
- `make ablation` runs the full matrix on the reference data and regenerates `runs/ablation_table.md`. With the published cache + `ReplayBackend`, this runs on any laptop in minutes — no Hailo hardware needed. With `HefBackend` + an empty cache, expected runtime is < 30 min on a Hailo-8L RPi5.
- All randomness seeded; tile-order deterministic; cache file SHA published alongside the table so anyone can verify their replay matches the paper.

**Technical report (`docs/paper/technical-report.md`):**
- Sections: Introduction & related work (mostly already drafted in `docs/research/2026-05-27-...`); the budget framing; the Emitter+Modifier architecture; the lever catalogue; the ablation table; discussion; limitations; references.
- BibTeX bibliography in `docs/paper/references.bib`, seeded from the research catalogue.
- Targeted as a tech report first (arXiv-ready format); upgrade to a venue submission once results justify it.

**Citation:** Repo `README.md` carries a citation block; the repo is registered on Zenodo for a DOI. `CITATION.cff` at repo root.

**CI:** A GitHub Actions / GitLab CI job runs the `ReplayBackend` ablation matrix on every PR (against a small committed-to-repo reference cache subset) — verifies the harness keeps working and the ablation table renders. Hardware ablation runs nightly on the dev RPi outside CI and refreshes the published cache.

**Public-API stability:** every class exported from `hailo_tiling.__init__` is marked stable; experimental things live in `hailo_tiling.experimental`. The detiler-plugin patch is versioned independently in `hailo-apps-core`.

## 11. Phases

The implementation plan (next step after this spec) will decompose into these phases:

1. **Scaffold** — create `hailo_tiling/` package, move `dynamic_tiling/` contents, set up `pyproject.toml`, license, CI.
2. **Refactor scheduler to Emitter+Modifier** — extract current `_grid`, `_roi`, recovery logic into `DiscoveryGridEmitter`, `TrackROIEmitter`, `RecoveryGridEmitter`; introduce `BudgetTrimModifier`.
3. **Telemetry layer** — `TelemetryProvider` ABC + the three implementations.
4. **Two new modifiers** — `AdaptiveSliceSizingModifier` (ASAHI) and `AltitudeZoomModifier` (validates that telemetry flows correctly).
5. **Backends + Aggregator extraction** — `InferenceBackend` ABC, lift `BoundaryStripFilter` into the Aggregator. `HefBackend` stays as dev-only path.
6. **Source data prep + FOV emulation** — extend `tiling_benchmark/prepare_video.py` to emit `clip__fov{70,60,50}.mp4` variants per source. Re-generate GT for each variant. Document SHA-256 of all variants in the reference-data manifest.
7. **Cache schema + Python layer** — `libhailotile_cache` schema definition (both `detections` and `frame_results` tables); Python `SqliteCacheStore` + `CachingBackend` + `ReplayBackend` + `hailo-tiling-warm-cache` CLI; determinism CI test.
8. **GStreamer cache plugins (`hailo-apps-core`)** — `libhailotile_cache.so` shared library, `hailodet_record` (recorder, both `tile_cache` and `full_frame` modes), `hailonet_cache` (replay). Each plugin shipped with golden-file tests and a microbench asserting lookup latency < 1 ms / crop.
9. **`GstCropperBackend`** — Python adapter that drives the production-style GStreamer pipeline (`hailotilecropper_dynamic` → `hailonet` or `hailonet_cache` → `hailofilter` → `hailodet_record mode=tile_cache` → `hailodetiler` → `hailodet_record mode=full_frame`) from within the ablation harness. Makes the GStreamer pipeline the canonical research path.
10. **Ablation harness** — `hailo-tiling-bench` CLI + YAML schema; defaults to `GstCropperBackend`; `--backend replay` toggles `hailonet_cache` in the pipeline string. Reports a table with one column per FOV variant.
11. **Telemetry capture and import** — add `telemetry` table to `flight_record.sqlite3`; wire `MavsdkTelemetry` to persist live during flight; ship `hailo-tiling-import-ulg` (PX4 ULG → telemetry table) and `hailo-tiling-import-srt` (DJI SRT → telemetry table) CLIs. Distance-to-target derivation as an optional reporting column.
12. **Visualizer + overlay renderer** — `hailo-tiling-view` (interactive frame stepper with map overlay) and `hailo-tiling-overlay` (offline annotated-MP4 batch renderer), both consuming `flight_record.sqlite3` + `telemetry` table. Replaces / supersedes `tiling_benchmark/overlay_viewer.py` and `tiling_benchmark/overlay_dets.py` once feature-equivalent.
13. **RPI-GS data collection + cache** — capture a Pi Global Shutter flight with the chosen lens (default 6 mm); record raw video; warm a cache from the recording; ensure the paper's main ablation matrix has an `RPI-GS` column alongside the FOV variants.
14. **`hailo-apps-core` patches** — single PR adding (a) detiler metadata hooks (per-detection CropRect provenance + ROI list pass-through), and (b) `hailofilter` `bypass-on-cache-hit` property. Both are small, backwards-compatible, and unlock the cache architecture end-to-end. Parallelisable with phases 1-13.
15. **Paper-with-code artifacts** — license, README, citation, technical-report skeleton, reference-data + cache fetcher (downloads the three FOV variants, the RPI-GS recording, and their warmed caches), ablation table generation, published reference cache files (Zenodo / S3).
16. **Drone-follow migration** — switch `drone_follow/pipeline_adapter/` over to the production pipeline with `hailodet_record mode=full_frame` and live telemetry persistence enabled by default. Tracker subpackage stays minimal; ByteTracker keeps current behaviour.
17. **(Bonus, gated by data collection)** DJI optical-zoom maximum-range capture and analysis at 1× / 2.5× / 6×. Single-figure-of-merit result; ships in the paper only if data is available in time.

## 12. Open Questions

- **Public reference clip:** resolved — one of the project's captured DJI Mavic 4 Pro clips will be released under a permissive license (CC-BY-SA) and uploaded to Zenodo. Same clip used internally, so the public and internal results are byte-identical. Specific clip ID picked in Phase 15 from the day's shoot.
- **Pi GS lens (resolved):** Arducam M12 telephoto, 20° on 1/2.3" → **H-FOV ≈ 13° on the IMX296** (Section 8.6). Lens parameters stored in the cache `meta` table and `lens_info.json` sibling, so any future lens swap is impossible to silently miss.
- **DJI bonus shoot (Phase 17):** when does the controlled-distance shoot happen? Independent of `DJI-TELE-12` main-matrix data (which comes from regular research footage); the bonus shoot only adds the *measured-distance* metadata needed for the maximum-range figure-of-merit.
- **Detiler-plugin upstream timeline:** if `hailo-apps-core` maintainers are slow to take the patch, do we vendor the modified plugin temporarily? Default plan: live with the Python-pad-probe fallback until merged.
- **YAML vs Python config for the ablation matrix:** spec assumes YAML for ease of paper reproducibility; can switch to a Python `@dataclass` matrix if it proves clunky.

## 13. Success Criteria

- All v1 emitters and modifiers covered by unit tests.
- `hailo-tiling-bench` produces a reproducible ablation table from one command.
- **The table reports each ablation row at all five variants** — FOV-70, FOV-60, FOV-50, `RPI-GS`, `DJI-TELE-12` — so both the emulated FOV trend and the platform-independence claim at narrow FOV (Pi GS vs DJI tele at ~12–13°) are verifiable per-lever.
- **Telemetry is captured end-to-end**: every paper-reported flight produces a `telemetry` table populated from MAVSDK (PX4 path) or SRT (DJI path); the visualizer renders the drone path on a map without additional input.
- **DJI bonus task is captured or explicitly skipped** — if data is collected, the paper reports a maximum-range figure-of-merit; if not, the bonus phase is documented as deferred (not silently dropped).
- The table includes (at minimum): static-baseline, current `dynamic_tiling` config, +ASAHI, +altitude-zoom — measured on both the proprietary clip and the public reference clip.
- **`prepare_video.py --emit-fov` is deterministic** — two runs of the same source produce byte-identical 4K output files (so cache keys match across runs).
- **Visualizer + overlay renderer work without chip access** — given any `flight_record.sqlite3` + matching source video, `hailo-tiling-view` and `hailo-tiling-overlay` produce annotated views on a laptop.
- The library installs cleanly without MAVSDK present.
- **Inference cache delivers what it promises:** a fully-warmed cache makes a full ablation matrix re-run complete in seconds on a chip-free laptop; cache hit-rate is > 95% on repeated runs of the same matrix.
- **GStreamer cache plugins meet the latency bar:** `hailodet_record` adds < 100 μs to the streaming-thread pad probe in steady state; `hailonet_cache` lookups are < 1 ms / crop (vs ~30–40 ms for live `hailonet`), measured by a microbench shipped with each plugin.
- **Cache hits bypass postprocess:** on a cache hit, `hailofilter` is a verified no-op (no `.so` invocation, no NMS, no decode). A test asserts that the `.so`'s entry point is not called when `bypass-on-cache-hit=true` and the buffer carries `hailo-cache-hit`.
- **Empty-frame recording works:** a frame with zero detections is recorded with `dets_json='[]'`; the next replay run hits cache for that frame instead of re-running inference. A regression test covers this.
- **Canonical research path is GStreamer:** every paper-reported ablation row runs through `GstCropperBackend`. `HefBackend` results are explicitly marked "dev-only" wherever they appear.
- **Production telemetry feeds research:** at least one drone flight produces a cache file via `hailodet_record`, and the ablation harness successfully replays it via `hailonet_cache` to demonstrate the end-to-end loop.
- `drone-follow` runs on the RPi using `GstCropperBackend` with at least parity on the existing per-target recall metric.
- `docs/paper/technical-report.md` has a complete first draft suitable for internal review.
