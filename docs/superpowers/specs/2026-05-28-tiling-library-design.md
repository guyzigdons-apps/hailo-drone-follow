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
- `InferenceBackend` ABC + `HefBackend` (existing), `GstCropperBackend` (new), `MockBackend` (replay)
- Aggregator with composable `BoundaryStripFilter` and a `DetectionMemory` interface (no-op default in v1; functional carry-forward implementation deferred to v2)
- Ablation harness CLI (`hailo-tiling-bench`) — sweeps a matrix and emits per-config `frames.json`
- Detiler-plugin metadata patch (separate PR in `hailo-apps-core`): per-detection CropRect provenance + ROI list pass-through
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
    mock.py            # MockBackend (replays frames.json; fast ablation, no chip needed)
  tracking/
    __init__.py
    tracker.py         # Tracker ABC, TrackState
    bytetrack.py       # ByteTrackAdapter (current tracker)
  cli/
    __init__.py
    run.py             # `hailo-tiling-run` — single-config standalone runner
    bench.py           # `hailo-tiling-bench` — ablation matrix harness
    score.py           # `hailo-tiling-score` — frames.json vs GT
docs/
  paper/
    technical-report.md  # the paper draft
    figures/
    references.bib
```

`dynamic_tiling/` is removed; its callers (drive video, `compare_baselines.py`) move to `hailo_tiling`. `tiling_benchmark/` stays, but its analysis tools (`analyze_pxt.py`, `overlay_viewer.py`) become consumers of the shared `frames.json` format.

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

`InferenceBackend.infer(frame, crops) → list[list[Det]]` is the seam between policy (scheduler) and mechanism (HailoRT call / GStreamer plugin). `MockBackend` lets the ablation harness replay a previously-recorded `frames.json` against new aggregator/modifier combinations *without re-running the chip* — important for fast lever-by-lever ablation.

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
2. **Replay-based integration tests** using `MockBackend` over a fixed `frames.json` and a fixed `TrackState` trace. Fast (< 5 s for a full ablation matrix), no chip required, runs in CI.
3. **Hardware acceptance test** — one canonical config + one ablation row run end-to-end against a known HEF + video, asserting per-target recall lands within a tolerance of the recorded reference. Not in CI; runs on the dev RPi via a separate `make accept` target.

## 7. Ablation Harness

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

A second mode runs the ablation entirely on `MockBackend` against a previously-recorded backend output — this lets us ablate aggregator/modifier changes in seconds without booking the chip.

## 8. Paper-with-Code Release Workflow

This is a first-class deliverable in v1, not a polish step.

**License:** Apache 2.0. Matches `hailo-apps-core` and `hailo-apps-infra`; commercial-friendly; standard for ML-with-code releases.

**Reproducibility recipe:**
- One reference video + one reference HEF + one reference telemetry trace, all referenced by SHA-256 in the repo (downloaded by `scripts/fetch_reference_data.sh`).
- Public reference video: a clip from a CC-licensed aerial dataset (candidates: VisDrone-MOT, SeaDronesSee). The proprietary DJI footage we use today stays internal; the public ablation table uses the public clip so anyone can reproduce.
- `make ablation` runs the full matrix on the reference data and regenerates `runs/ablation_table.md`. Expected runtime documented (target: < 30 min on a Hailo-8L RPi5).
- All randomness seeded; tile-order deterministic.

**Technical report (`docs/paper/technical-report.md`):**
- Sections: Introduction & related work (mostly already drafted in `docs/research/2026-05-27-...`); the budget framing; the Emitter+Modifier architecture; the lever catalogue; the ablation table; discussion; limitations; references.
- BibTeX bibliography in `docs/paper/references.bib`, seeded from the research catalogue.
- Targeted as a tech report first (arXiv-ready format); upgrade to a venue submission once results justify it.

**Citation:** Repo `README.md` carries a citation block; the repo is registered on Zenodo for a DOI. `CITATION.cff` at repo root.

**CI:** A GitHub Actions / GitLab CI job runs the `MockBackend` ablation matrix on every PR — verifies the harness keeps working and the ablation table renders. Hardware ablation runs nightly on the dev RPi outside CI.

**Public-API stability:** every class exported from `hailo_tiling.__init__` is marked stable; experimental things live in `hailo_tiling.experimental`. The detiler-plugin patch is versioned independently in `hailo-apps-core`.

## 9. Phases

The implementation plan (next step after this spec) will decompose into these phases:

1. **Scaffold** — create `hailo_tiling/` package, move `dynamic_tiling/` contents, set up `pyproject.toml`, license, CI.
2. **Refactor scheduler to Emitter+Modifier** — extract current `_grid`, `_roi`, recovery logic into `DiscoveryGridEmitter`, `TrackROIEmitter`, `RecoveryGridEmitter`; introduce `BudgetTrimModifier`.
3. **Telemetry layer** — `TelemetryProvider` ABC + the three implementations.
4. **Two new modifiers** — `AdaptiveSliceSizingModifier` (ASAHI) and `AltitudeZoomModifier` (validates that telemetry flows correctly).
5. **Backends + Aggregator extraction** — `InferenceBackend` ABC, `MockBackend`, lift `BoundaryStripFilter` into the Aggregator.
6. **Ablation harness** — `hailo-tiling-bench` CLI + YAML schema + replay mode.
7. **Detiler-plugin patch** — separate PR to `hailo-apps-core` adding metadata hooks (parallelisable with phases 1-6).
8. **Paper-with-code artifacts** — license, README, citation, technical-report skeleton, reference-data fetcher, ablation table generation.
9. **Drone-follow migration** — switch `drone_follow/pipeline_adapter/` over to `GstCropperBackend` for the live pipeline. Tracker subpackage stays minimal; ByteTracker keeps current behaviour.

## 10. Open Questions

- **Public reference data:** which CC-licensed aerial clip do we settle on? Resolved as part of Phase 8; doesn't block Phases 1-6.
- **Detiler-plugin upstream timeline:** if `hailo-apps-core` maintainers are slow to take the patch, do we vendor the modified plugin temporarily? Default plan: live with the Python-pad-probe fallback until merged.
- **YAML vs Python config for the ablation matrix:** spec assumes YAML for ease of paper reproducibility; can switch to a Python `@dataclass` matrix if it proves clunky.

## 11. Success Criteria

- All v1 emitters and modifiers covered by unit tests.
- `hailo-tiling-bench` produces a reproducible ablation table from one command.
- The table includes (at minimum): static-baseline, current `dynamic_tiling` config, +ASAHI, +altitude-zoom — measured on both the proprietary clip and the public reference clip.
- The library installs cleanly without MAVSDK present.
- `drone-follow` runs on the RPi using `GstCropperBackend` with at least parity on the existing per-target recall metric.
- `docs/paper/technical-report.md` has a complete first draft suitable for internal review.
