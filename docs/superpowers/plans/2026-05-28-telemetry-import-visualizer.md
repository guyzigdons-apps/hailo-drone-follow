# Plan 7: Telemetry import (ULG/SRT) + visualizer — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Land spec phases 11 (telemetry import) and 12 (visualizer + overlay renderer) of `docs/superpowers/specs/2026-05-28-tiling-library-design.md`. After this plan, `hailo_tiling/telemetry/` gains two file-format parsers (`ulg.py`, `srt.py`) that produce JSONL timelines compatible with the existing `RecordedTelemetry` provider; `hailo_tiling/cli/` gains a single `import_telemetry` CLI (multiplexed by `--ulg | --srt`) plus a `visualize` CLI that renders an ffmpeg+ASS overlay on top of a source video. No `dynamic_tiling/`, scheduler, modifier, or backend code is touched.

**Spec reference:** `docs/superpowers/specs/2026-05-28-tiling-library-design.md` §3.3 (TelemetrySnapshot), §7.12 (full-frame consumers), §8.8 (geospatial telemetry capture — schema and ULG/SRT format), §11 phases 11 + 12, §13 (success criteria — chip-free visualizer requirement).

**Branch / starting HEAD:** `tiling-benchmark` (after Plans 1–4). Parallel-safe with Plan 5 (cache plugins) and Plan 6 (ablation harness): all of this plan's files live under `hailo_tiling/telemetry/`, `hailo_tiling/cli/`, `tests/`, plus `pyproject.toml` and `docs/superpowers/plans/INDEX.md` — no overlap with Plans 5/6 file trees.

**Tech Stack:** Python 3.10+, pytest. New hard dependency: `pyulog` (PX4-maintained, pure-Python). No new SRT library — the DJI SRT format is a small line-oriented dialect parsed directly. ffmpeg is invoked via subprocess for the visualizer (no Python ffmpeg binding required); a pure CPython unit-test path exists for every parser/aligner so the test suite still runs on a laptop without ffmpeg.

**Architecture / key design decisions:**

- **Adapter-only, no new provider.** The existing `RecordedTelemetry` (Plan 2) already reads a JSONL timeline keyed by monotonic `timestamp`. Plan 7 adds two **adapters** (ULG → JSONL, SRT → JSONL) and a CLI that calls them. The library does not grow a `UlgTelemetry` or `SrtTelemetry` provider — keeps the surface small, and the produced JSONL is the canonical artifact (matches spec §8.8 "preserved as an artifact alongside the video").
- **Time alignment is a separable concern.** Both parsers emit timelines in their *native* time base (PX4 microseconds-since-boot for ULG, wall-clock ISO timestamps for SRT). A small `align.py` module converts native time → monotonic seconds with a clear single-strategy default (`--align video-start`: telemetry-row-0 maps to video-PTS-0). A `--align offset:<seconds>` knob covers manual alignment for clips with a known sync point. Auto-sync via first-detection is **explicitly deferred** to a follow-up (see Open Questions §1) — it depends on detection records this plan does not produce.
- **Visualizer is ffmpeg + ASS, not OpenCV.** Reading source MP4, writing an annotated MP4, and burning overlay text are all things ffmpeg already does well. We render the overlay to a per-frame ASS subtitle file and run a single `ffmpeg -i video -vf "ass=overlay.ass" out.mp4` invocation. No OpenCV frame-loop dependency, no Qt, no per-frame Python draw. The visualizer is a **CLI-only batch tool** in this plan (matches spec §7.12's `hailo-tiling-overlay`). The interactive `hailo-tiling-view` stepper is out of scope here (the spec's `--record-out` UI affordance lands in a later plan once the SQLite full-frame table exists from Plan 5/6).
- **TelemetrySnapshot field mapping (only these five from §3.3):**
  | TelemetrySnapshot field | ULG (PX4 topic) | DJI SRT |
  |-------------------------|-----------------|---------|
  | `altitude_agl_m`        | `vehicle_local_position.dist_bottom` (preferred) or `vehicle_air_data.baro_alt_meter` − home | `[rel_alt: ...]` (rel-altitude above takeoff; close enough for AGL) |
  | `yaw_rate_rad_s`        | `vehicle_attitude.angular_velocity[2]` or `sensor_combined.gyro_rad[2]` | not present → `None` |
  | `velocity_world`        | `vehicle_local_position.{vx, vy, vz}` (NED) | not present (SRT only has position) → `None` |
  | `attitude_quat`         | `vehicle_attitude.q` | not present → `None` |
  | `timestamp`             | derived: `(timestamp_us − first_timestamp_us) / 1e6` | derived: `(ISO_ts − first_ISO_ts).total_seconds()` |

  Geo fields from spec §8.8 schema (`lat`, `lon`, `alt_msl`) are also extracted but stored under a **sidecar JSONL key** (e.g. `_geo: {lat, lon, alt_msl}`) so they survive the round-trip without polluting the in-library `TelemetrySnapshot`. The visualizer reads them; modifiers ignore them. This keeps Plan 2's `TelemetrySnapshot` shape stable.
- **No SQLite in this plan.** Spec §8.8 puts telemetry in a `telemetry` table inside `flight_record.sqlite3`. That SQLite file is owned by Plan 5 (cache plugins) and Plan 6 (ablation harness writes it). This plan writes only JSONL — the CLI's `--output` is a JSONL path. A short note in the CLI `--help` says "to ingest into `flight_record.sqlite3`, pipe through `hailo-tiling-bench --ingest-telemetry` once Plan 6 lands." Avoiding the SQLite coupling is what makes this plan parallel-safe with Plans 5/6.

**Out of scope (deferred):**
- The interactive `hailo-tiling-view` stepper with map overlay → next plan after Plan 5/6 land the SQLite schema.
- `.DAT` flight-log import (spec §8.8 "secondary, only if needed").
- Distance-to-target derivation (spec §8.8 calls this "optional for v1 paper"; computed at render time, depends on detection records).
- Auto-sync from first-detection (depends on detection record format from Plans 5/6).
- Ingesting telemetry into the SQLite `telemetry` table (owned by Plan 6).

---

## File Structure

**Files this plan creates:**

```
hailo_tiling/
  telemetry/
    ulg.py                            # ULG → list[dict] (JSONL rows)
    srt.py                            # DJI SRT → list[dict] (JSONL rows)
    align.py                          # time-alignment: native → monotonic seconds
  cli/
    import_telemetry.py               # hailo-tiling-import-telemetry CLI
    visualize.py                      # hailo-tiling-visualize CLI (ffmpeg+ASS)
  tests/
    fixtures/
      telemetry/
        tiny.ulg                      # ~50 KB PX4 SITL log, committed
        tiny.srt                      # ~5 KB DJI sidecar (anonymised lat/lon), committed
        tiny_video_120frames.mp4      # 30 fps × 4 s synthetic colorbar clip, ~80 KB
    test_telemetry_ulg.py
    test_telemetry_srt.py
    test_telemetry_align.py
    test_cli_import_telemetry.py
    test_recorded_from_imported.py    # round-trip: importer → RecordedTelemetry → snapshot()
    test_visualizer_ass.py            # ASS file generator unit tests
    test_cli_visualize.py             # end-to-end ffmpeg invocation, gated by FFMPEG_AVAILABLE
```

**Files this plan modifies:**

- `pyproject.toml`
  - add `pyulog>=1.0` to `[project.dependencies]` (it's pure Python and small; do **not** make it optional — both CLIs need it and the unit tests need it).
  - add to `[project.scripts]`:
    - `hailo-tiling-import-telemetry = "hailo_tiling.cli.import_telemetry:main"`
    - `hailo-tiling-visualize = "hailo_tiling.cli.visualize:main"`
- `hailo_tiling/cli/__init__.py` — no-op; module already exists. Optionally re-export the new `main` functions for `python -m` convenience (mirrors `warm.py` pattern).
- `hailo_tiling/telemetry/__init__.py` — add public re-exports: `parse_ulg`, `parse_srt`, `align_to_video`. The library does not gain a new provider class.
- `docs/superpowers/plans/INDEX.md` — flip Plan 7 status from `not started` → `in flight` at start of Task 1; → `done` at end of Task 10.
- `README.md` — append a "Telemetry import + visualizer" subsection under the existing tiling-library section (Task 10).

**Files this plan does NOT touch:**

- `hailo_tiling/scheduler.py`, `hailo_tiling/modifiers/`, `hailo_tiling/emitters/`, `hailo_tiling/backends/`, `hailo_tiling/aggregator/`, `hailo_tiling/cache/`, `hailo_tiling/budget.py`, `hailo_tiling/types.py`. The library shape is frozen for this plan; only the telemetry adapter surface and the CLI subpackage grow.
- `dynamic_tiling/` (legacy parity layer).
- `tiling_benchmark/overlay_dets.py`, `tiling_benchmark/overlay_viewer.py` — the spec says they get superseded "once feature-equivalent." That's a later plan; this plan adds the *new* tool alongside, not in place of.
- Any code that imports the chip (`hailort`, `hailo_apps`, `mavsdk`). Plan 7 is verifiable on a chip-free laptop end-to-end (spec §13 success criterion).

---

## Pre-flight: virtual environment

All commands assume the project venv is active. If you've never set it up, run `source setup_env.sh` once. After that, use the direct binaries — `/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo-apps/venv_hailo_apps/bin/python` and `.../bin/pytest` — because shell state doesn't persist between tool calls. For brevity, paths are written as `python` / `pytest`.

**Sanity check before starting:**
```bash
python -c "import hailo_tiling; from hailo_tiling.telemetry import RecordedTelemetry, TelemetrySnapshot; print('ok')"
pytest hailo_tiling/tests -q
```
Expected: `ok`, then all Plan 1–4 tests passing.

---

## Task 1 [no-chip] — `pyulog` dependency + scaffold

Add the runtime dependency, create empty module stubs, and confirm the import surface compiles. No behaviour yet — this task is the smallest possible commit that wires the new files into the package.

**Files:**
- Modify: `pyproject.toml` (add `pyulog>=1.0` and the two `project.scripts` entries).
- Create: `hailo_tiling/telemetry/ulg.py` (with a `parse_ulg(path: Path) -> list[dict]` stub that raises `NotImplementedError`).
- Create: `hailo_tiling/telemetry/srt.py` (with a `parse_srt(path: Path) -> list[dict]` stub).
- Create: `hailo_tiling/telemetry/align.py` (with `align_to_video(rows, video_path, strategy) -> list[dict]` stub).
- Create: `hailo_tiling/cli/import_telemetry.py` (argparse skeleton, `main` raises `SystemExit("not implemented")`).
- Create: `hailo_tiling/cli/visualize.py` (argparse skeleton).
- Create: `hailo_tiling/tests/fixtures/telemetry/` (empty dir, with a `.gitkeep`).
- Modify: `hailo_tiling/telemetry/__init__.py` — add the three new re-exports.
- Modify: `docs/superpowers/plans/INDEX.md` — Plan 7 status → `in flight`.

**Steps:**
- [ ] Verify `pyulog` installs cleanly into the venv: `pip install pyulog`. Confirm `python -c "import pyulog; print(pyulog.__version__)"` works.
- [ ] Edit `pyproject.toml`: add `pyulog>=1.0` to `dependencies`, register the two console scripts.
- [ ] Create each new module with a docstring-only body + the stub function signature, raising `NotImplementedError(__name__)`.
- [ ] Add re-exports to `hailo_tiling/telemetry/__init__.py`.
- [ ] Reinstall the project in editable mode so the new console scripts get registered: `pip install -e .`.
- [ ] Verify the scripts resolve: `which hailo-tiling-import-telemetry` and `which hailo-tiling-visualize` should point into the venv `bin/`.
- [ ] Run `pytest hailo_tiling/tests -q` — must still be green (the new stubs have no tests yet).

**Acceptance:** the package imports, the two CLIs are discoverable by name, all pre-existing tests pass, INDEX.md shows `in flight`.

---

## Task 2 [no-chip] — ULG → TelemetrySnapshot timeline mapping

Implement `parse_ulg(path) -> list[dict]`. Each output row is a JSONL-shaped dict carrying the five `TelemetrySnapshot` fields plus the `_geo` sidecar (lat/lon/alt_msl/pitch/roll for the visualizer). Time base is monotonic seconds from the first message's `timestamp_us`.

**Fixture acquisition (part of this task):**
- Generate a small SITL log: launch `sim/start_sim.sh`, arm + take off to ~5 m, wait 3 s, land, disarm. PX4 writes a `.ulg` to `~/.ros/log/`. Trim it to ≤ 1 MB via `ulog_extract_gps_dump` or `pyulog`'s `ulog_messages` filter to keep only the topics this plan reads (`vehicle_local_position`, `vehicle_attitude`, `vehicle_air_data`, `vehicle_global_position`). Commit as `hailo_tiling/tests/fixtures/telemetry/tiny.ulg`.
- If SITL generation fails for any reason, fall back to a synthesised ULG: pyulog ships `ulog-cli`'s test fixtures (BSD-licensed). Copy one of those, anonymise GPS to (0.0, 0.0), and commit. Note the fallback in a `tiny.ulg.README.md` sibling.

**Files:**
- Modify: `hailo_tiling/telemetry/ulg.py` (full implementation).
- Create: `hailo_tiling/tests/test_telemetry_ulg.py`.
- Create: `hailo_tiling/tests/fixtures/telemetry/tiny.ulg`.

**Implementation contract for `parse_ulg`:**
- Input: `pathlib.Path` to a `.ulg`.
- Output: `list[dict]`, sorted by `timestamp`, each row containing keys:
  - `timestamp: float` (seconds, monotonic from first message)
  - `altitude_agl_m: Optional[float]` (prefer `dist_bottom` from `vehicle_local_position`; else `None`)
  - `yaw_rate_rad_s: Optional[float]` (from `vehicle_attitude.angular_velocity[2]` if present; else gyro z; else `None`)
  - `velocity_world: Optional[list[float]]` (from `vehicle_local_position.{vx, vy, vz}` as a 3-tuple; NED; else `None`)
  - `attitude_quat: Optional[list[float]]` (`vehicle_attitude.q` as `[w, x, y, z]`; else `None`)
  - `_geo: dict` with keys `lat`, `lon`, `alt_msl`, `pitch`, `roll` (any can be `None`)
- Resampling: PX4 topics arrive at different rates (50–250 Hz typical). The parser **does not resample** — it emits one row per **`vehicle_local_position` message timestamp** and at each such timestamp, joins the most-recent-prior values from the other topics (last-value-carried-forward). This gives roughly 50 rows/sec, matching the `RecordedTelemetry.snapshot()` lookup cost.

**Tests:**
- `test_parse_ulg_smoke`: parse the fixture, assert `len(rows) > 0`, first row `timestamp == 0.0`, last row `timestamp > 0.5` seconds.
- `test_parse_ulg_fields_round_trip`: assert at least one row has non-None `altitude_agl_m`, at least one has non-None `velocity_world`, at least one has non-None `attitude_quat`.
- `test_parse_ulg_monotonic_timestamps`: assert `rows[i].timestamp <= rows[i+1].timestamp` for all `i`.
- `test_parse_ulg_geo_present`: at least one row has `_geo['lat']` non-None and `_geo['lon']` non-None.
- `test_parse_ulg_missing_topic_no_crash`: synthesise a `.ulg` (via `pyulog.writer.ULog`) that lacks `vehicle_attitude`; assert `parse_ulg` returns rows with `attitude_quat is None` rather than raising.

**Acceptance:** all five tests pass on the committed fixture. `parse_ulg` accepts a malformed file by raising a single `ValueError` with the underlying pyulog error chained — never a bare exception leak.

---

## Task 3 [no-chip] — SRT parser + DJI key-value extractor

Implement `parse_srt(path) -> list[dict]`. The DJI SRT (sample at top of plan) has one block per frame with:
- index line (`1`, `2`, …)
- SRT timing line (`00:00:00,000 --> 00:00:00,033`)
- HTML-formatted payload with `FrameCnt`, ISO timestamp, and bracketed key-value pairs like `[latitude: ...] [longitude: ...] [rel_alt: ...] [focal_len: ...]`.

Output rows mirror the ULG shape (same key set), but `velocity_world`, `attitude_quat`, and `yaw_rate_rad_s` are always `None` because SRT carries no inertial data.

**Fixture acquisition (part of this task):**
- Copy one of `/home/giladn/Videos/Drone/Training/DJI_*_D.SRT`, anonymise GPS by clamping lat/lon to `(0.000xxx, 0.000xxx)` (preserve precision pattern), keep first ~30 entries (~1 second at 30 fps), commit as `hailo_tiling/tests/fixtures/telemetry/tiny.srt`. **Do not hard-code the source path in the test** — copy the bytes once during fixture creation, then commit. Tests reference only the committed fixture.

**Files:**
- Modify: `hailo_tiling/telemetry/srt.py` (full implementation).
- Create: `hailo_tiling/tests/test_telemetry_srt.py`.
- Create: `hailo_tiling/tests/fixtures/telemetry/tiny.srt`.

**Implementation contract for `parse_srt`:**
- Pure-Python regex-based parser (no `pysubs2` or other SRT lib dep; the format here is too dialect-specific to be worth a library). One sub-function per concern:
  - `_split_blocks(text) -> list[str]` (split on blank lines).
  - `_parse_block(block) -> dict` (extract `frame_cnt`, ISO timestamp, and bracketed kv-pairs).
  - `_bracket_pairs(payload) -> dict[str, str]` (regex `\[(\w+):\s*([^\]]+)\]`).
- Time conversion: parse the ISO timestamp with `datetime.fromisoformat`, take the row-0 timestamp as the origin, emit `timestamp = (this_ts - origin).total_seconds()`. Fallback: if ISO parse fails for any block, use the SRT timing line's start-time as a secondary source.
- Geo fields: `_geo.lat = float(kv['latitude'])`, `_geo.lon = float(kv['longitude'])`, `_geo.alt_msl = float(kv['abs_alt'])`, plus a non-spec convenience `_geo.focal_len_mm = float(kv['focal_len'])` (the visualizer renders it).
- `altitude_agl_m`: spec §8.8 explicitly says SRT carries `rel_alt` ("height above takeoff") which is NULL-from-AGL in the schema. The parser maps it to `altitude_agl_m` anyway (it's the closest available signal) and records `_geo._agl_source = "rel_alt"` so the visualizer can disclaim.

**Tests:**
- `test_parse_srt_smoke`: ≥ 5 rows, monotonic timestamps, first row `timestamp == 0.0`.
- `test_parse_srt_geo_extraction`: at least one row has `_geo['lat']` matching the fixture's anonymised value to within float precision.
- `test_parse_srt_no_velocity_or_attitude`: every row has `velocity_world is None` and `attitude_quat is None`.
- `test_parse_srt_handles_missing_optional_keys`: feed `_parse_block` a synthetic block missing `[ev: ...]` — must not crash, must return the present keys.
- `test_parse_srt_handles_html_font_tags`: the real fixture contains `<font size="28">…</font>` wrappers; parser must strip them transparently.
- `test_parse_srt_empty_file`: empty input → empty list, no exception.

**Acceptance:** all six tests pass on the committed fixture.

---

## Task 4 [no-chip] — Time-alignment algorithm (video timestamp + telemetry offset)

Implement `align_to_video(rows: list[dict], video_path: Path, strategy: str) -> list[dict]`. The default strategy is the simplest one that works for all SRT clips and most ULGs: the telemetry timeline is **already** in monotonic seconds (Tasks 2 + 3 emit `timestamp[0] = 0.0`), and the video's PTS-0 corresponds to the same wall-clock moment, so **no shift is needed** for the default case. The function exists to make the implicit assumption explicit and to provide a manual-offset escape hatch.

**Files:**
- Modify: `hailo_tiling/telemetry/align.py`.
- Create: `hailo_tiling/tests/test_telemetry_align.py`.

**Strategies (supported in this plan):**
- `"video-start"` (default) — identity: `rows` are returned unchanged. Logs `info` that the assumption is "telemetry row 0 ⇔ video PTS 0."
- `"offset:<seconds>"` — adds the given float (positive or negative) to every row's `timestamp`. Rows that fall outside `[0, video_duration]` after the shift are **kept**, not dropped — `RecordedTelemetry.snapshot()` already handles out-of-range queries (returns last-≤ or NULL_SNAPSHOT). Callers can pre-clip if they want.
- `"video-creation"` — reads the video's `creation_time` MP4 metadata via `ffprobe -show_format -of json`, parses it as an ISO timestamp, computes the offset to align the SRT's first-block ISO time to the video PTS-0. **Only useful for SRT input**; for ULG input this strategy raises `ValueError("video-creation strategy requires SRT rows with absolute timestamps")` because ULG rows are already relativised in Task 2.

**Auto-sync via first-detection is explicitly NOT in this plan** — see Open Questions §1.

**Tests:**
- `test_align_video_start_identity`: input rows pass through unchanged.
- `test_align_offset_positive`: `strategy="offset:1.5"` shifts every timestamp by +1.5 s.
- `test_align_offset_negative`: `strategy="offset:-0.5"` allows negative timestamps in the output (don't drop).
- `test_align_video_creation_from_srt`: monkeypatch the ffprobe call to return a known `creation_time`; assert the resulting offset matches the SRT first-block ISO time minus that creation time.
- `test_align_video_creation_rejects_ulg`: feed a row dict whose `_geo` has no `_iso_ts` key (the marker the SRT parser adds); assert `ValueError`.
- `test_align_invalid_strategy`: `strategy="bogus"` → `ValueError` listing valid strategies.

**Acceptance:** all six tests pass; no test depends on ffmpeg being installed (the `ffprobe` call is monkeypatched).

---

## Task 5 [no-chip] — `hailo-tiling-import-telemetry` CLI

Wire the three parsers + aligner into a single CLI that writes a JSONL timeline consumable by `RecordedTelemetry.from_path()`.

**Files:**
- Modify: `hailo_tiling/cli/import_telemetry.py` (full implementation).
- Create: `hailo_tiling/tests/test_cli_import_telemetry.py`.

**CLI surface:**
```
hailo-tiling-import-telemetry
    (--ulg PATH | --srt PATH)            # mutually exclusive, exactly one required
    [--video PATH]                       # required if --align video-creation
    [--align {video-start, offset:<sec>, video-creation}]  # default: video-start
    [--output PATH]                      # default: <input>.jsonl alongside the input
    [--limit N]                          # debug knob: keep only first N rows
    [--strip-geo]                        # drop the _geo sidecar (smaller output, RecordedTelemetry doesn't need it)
```

**JSONL output format:** one row per line, JSON-serialised. Keys exactly match the `RecordedTelemetry._row_to_snapshot` schema (i.e. `timestamp`, `altitude_agl_m`, `yaw_rate_rad_s`, `velocity_world` as list-not-tuple, `attitude_quat` as list-not-tuple) plus the optional `_geo` sub-dict. Existing JSONL files from Plan 2 tests stay readable.

**Steps:**
- [ ] Argparse with mutually-exclusive `--ulg / --srt`.
- [ ] Dispatch to `parse_ulg` or `parse_srt`.
- [ ] Call `align_to_video` with the requested strategy.
- [ ] If `--strip-geo`, delete the `_geo` key from each row.
- [ ] If `--limit N`, truncate to first N.
- [ ] Open `--output` in write-text mode and emit one `json.dumps(row)` per line.
- [ ] Print a one-line summary to stderr: `wrote N rows from <source> to <output>`.

**Tests:**
- `test_cli_ulg_round_trip`: invoke `main(["--ulg", str(fixture), "--output", str(tmp_path/"out.jsonl")])`; load the JSONL and assert ≥ 1 row with non-None `altitude_agl_m`.
- `test_cli_srt_round_trip`: same shape for SRT fixture.
- `test_cli_requires_one_source`: invoking with neither `--ulg` nor `--srt` exits non-zero with a clear error.
- `test_cli_rejects_both_sources`: `--ulg X --srt Y` exits non-zero.
- `test_cli_default_output_path`: invoking without `--output` writes alongside input as `<input>.jsonl`.
- `test_cli_strip_geo_removes_sidecar`: with `--strip-geo`, no row in the output contains a `_geo` key.

**Acceptance:** all six tests pass; running `hailo-tiling-import-telemetry --help` from the venv shell shows the expected options.

---

## Task 6 [no-chip] — `RecordedTelemetry` integration test (round-trip)

Verify the end-to-end loop: importer reads a fixture → writes JSONL → `RecordedTelemetry.from_path()` consumes it → per-frame `snapshot()` returns the right values at the right monotonic times.

**Files:**
- Create: `hailo_tiling/tests/test_recorded_from_imported.py`.

**Tests:**
- `test_ulg_to_recorded_provider_smoke`: import the ULG fixture, load via `RecordedTelemetry.from_path`, call `snapshot(0.0)`, `snapshot(0.5)`, `snapshot(10.0)`. Assert each result is a `TelemetrySnapshot`, that `snapshot(0.0)` has `timestamp == 0.0`, that `snapshot(10.0)` has `timestamp == 10.0` (the requested t), and that out-of-range queries don't crash.
- `test_srt_to_recorded_provider_smoke`: same for SRT.
- `test_ulg_recorded_feeds_altitude_zoom_modifier`: build a `RecordedTelemetry` from the ULG fixture and feed three snapshots (early, mid, late) to `AltitudeZoomModifier.modify()`. Assert the modifier's output tile mode is `"s"` and that the zoom factor varies with altitude (compare ROI widths at low vs high altitude). This proves the spec §13 "telemetry flows correctly" claim end-to-end.
- `test_srt_recorded_no_velocity_does_not_break_adaptive_sizing`: feed SRT-sourced snapshots (where `velocity_world is None`) into `AdaptiveSliceSizingModifier.modify()`; assert it falls back to the bbox-only path (the Plan 2 modifier already supports this — this test is regression coverage for the join).

**Acceptance:** all four tests pass.

---

## Task 7 [no-chip] — Visualizer: ffmpeg + ASS overlay generator (pure-Python)

Build the **ASS file generator** as a pure-Python function. No ffmpeg call yet — that's Task 8. Separating these two lets us unit-test the overlay rendering without an ffmpeg dependency in CI.

**ASS format primer:** Advanced SubStation Alpha is a plain-text subtitle format ffmpeg burns natively via the `ass=` filter. One `Dialogue:` line per visible cue, with a start/end time and a styled text payload. Bottom-of-screen positioning, monospace font, and per-cue replacement give us a simple HUD.

**Files:**
- Create: `hailo_tiling/cli/visualize.py` — add `build_ass(rows, fps, duration_s) -> str` function plus an `_format_overlay_text(row) -> str` helper.
- Create: `hailo_tiling/tests/test_visualizer_ass.py`.

**Overlay text contents (minimum, per spec §7.12 / §12 visualizer requirement):**
- Altitude: `ALT  12.3 m` (from `altitude_agl_m`)
- Ground speed: `GS   4.5 m/s` (from `sqrt(vx² + vy²)` if `velocity_world` present, else from a per-row precomputed scalar)
- Yaw rate: `YAW  0.12 rad/s` (from `yaw_rate_rad_s`)
- Position: `LAT  0.000123  LON  0.000456` (from `_geo`, if present)
- Frame number: `F    1234` (computed: `int(round(timestamp_s * fps))`)
- Missing fields render as `--` rather than being omitted (so columns line up).

**ASS rendering rules:**
- Header: `[Script Info] / [V4+ Styles] / [Events]` with a single `Default` style: `Consolas, 24pt, white, 0.5px black outline, BottomLeft anchor`.
- One `Dialogue:` line per **video frame** (so the overlay updates smoothly). The dialogue's start time = `frame_idx / fps`, end time = `(frame_idx + 1) / fps`. The row value used is the latest telemetry row with `timestamp ≤ start_time` (same last-value-carried-forward semantics as `RecordedTelemetry`).
- For a 4-second test clip at 30 fps, that's 120 cues — trivial to render.

**Tests:**
- `test_ass_header_well_formed`: generated string starts with `[Script Info]` and contains the expected style line.
- `test_ass_one_dialogue_per_frame`: count `Dialogue:` lines for a 30-fps, 2-second input → exactly 60.
- `test_ass_cue_uses_latest_le_telemetry_row`: rows at `t=0.0, 1.0`; the cue at frame 45 (`t=1.5s`) must reflect the row at `t=1.0`.
- `test_ass_renders_missing_fields_as_dashes`: a row with `altitude_agl_m=None` produces `ALT  --` in its cue.
- `test_ass_format_overlay_text_is_fixed_width`: every output line has the same character length (regression for column alignment).
- `test_ass_escapes_ass_specials`: `_geo` strings containing `{` or `\` get escaped (defensive; lat/lon won't trigger this, but field labels could if extended).

**Acceptance:** all six tests pass; no test invokes ffmpeg.

---

## Task 8 [no-chip] — `hailo-tiling-visualize` CLI (ffmpeg invocation)

Wrap Task 7's `build_ass` in the CLI, write the ASS file to a temp location, and invoke ffmpeg.

**Files:**
- Modify: `hailo_tiling/cli/visualize.py` — full `main(argv)` body.
- Create: `hailo_tiling/tests/test_cli_visualize.py`.

**CLI surface:**
```
hailo-tiling-visualize
    --video PATH                        # source video
    --telemetry PATH                    # JSONL from import_telemetry
    --output PATH                       # annotated MP4
    [--fps FLOAT]                       # default: probed from video via ffprobe
    [--font SIZE]                       # default: 24
    [--ffmpeg-path PATH]                # default: `ffmpeg` on PATH
    [--dry-run]                         # build the ASS, print it, don't invoke ffmpeg
```

**ffmpeg command:**
```
ffmpeg -y -i <video> -vf "ass=<tmpfile.ass>" -c:v libx264 -preset fast -crf 20 -c:a copy <output>
```
The `ass=` filter accepts a file path; we render the ASS to a `NamedTemporaryFile` inside `tempfile.TemporaryDirectory()` so cleanup is automatic. Exit non-zero with ffmpeg's stderr on failure.

**ffprobe used to probe `fps`:** the parsed value comes from `ffprobe -v error -select_streams v:0 -show_entries stream=avg_frame_rate -of default=nw=1:nk=1 <video>`. Default to `30.0` if probing fails.

**Tests:**
- `test_visualize_dry_run_emits_ass`: monkeypatch `subprocess.run` so it never runs; assert `--dry-run` prints an ASS string to stdout and exits 0.
- `test_visualize_invokes_ffmpeg_when_present`: gated by `pytest.importorskip` of a tiny helper that checks `shutil.which("ffmpeg")`; run against the `tiny_video_120frames.mp4` fixture + `tiny.srt`-imported JSONL; assert the output file exists, has non-zero size, and is a valid MP4 (verify with ffprobe by reading `stream=codec_type`).
- `test_visualize_missing_ffmpeg_reports_clearly`: monkeypatch `shutil.which("ffmpeg")` to `None`; CLI exits with a helpful error citing the install hint, not a stack trace.
- `test_visualize_probes_fps_from_video`: monkeypatch ffprobe to return `"24000/1001"`; assert the resulting ASS has 23.976-fps cue spacing.

**Fixture `tiny_video_120frames.mp4`:** generated at test-fixture-creation time with `ffmpeg -y -f lavfi -i testsrc=duration=4:size=320x240:rate=30 -c:v libx264 -pix_fmt yuv420p tiny_video_120frames.mp4`. Commit the resulting ~80 KB MP4. (The fixture is reused by Task 6 if helpful.)

**Acceptance:** all four tests pass when ffmpeg is installed; the ffmpeg-gated test is auto-skipped (not failed) when it's not.

---

## Task 9 [no-chip] — Hardening pass: error messages, log format, JSONL determinism

A small but explicit task. Cleaning up the rough edges before the README update.

**Files:**
- Modify: `hailo_tiling/cli/import_telemetry.py`, `hailo_tiling/cli/visualize.py`, `hailo_tiling/telemetry/{ulg,srt,align}.py`.
- Create: `hailo_tiling/tests/test_import_telemetry_determinism.py`.

**Steps:**
- [ ] Every CLI prints a one-line `OK` or actionable error to stderr (matches `warm.py` style).
- [ ] `parse_ulg` and `parse_srt` produce **byte-identical JSONL** across two runs of the same input. The test loads the fixture twice, runs the importer, hashes both outputs, and asserts SHA-256 equality. This matters because Plan 6's reproducibility hinges on deterministic upstream artifacts (mirrors spec §13 "two runs of the same source produce byte-identical output").
- [ ] `json.dumps(..., sort_keys=True, separators=(",", ":"))` is used for every row write to remove ambiguity.
- [ ] Add a `--version` flag to both CLIs (reads `hailo_tiling.__version__`).
- [ ] No `print(...)` to stdout from library modules (only from CLI `main` functions); enforced by a quick `grep` in CI later.

**Tests:**
- `test_import_ulg_is_deterministic`: import twice, compare SHA-256.
- `test_import_srt_is_deterministic`: same for SRT.
- `test_cli_version_flag`: `main(["--version"])` exits 0 and prints the version.

**Acceptance:** three new tests pass; manual `hailo-tiling-import-telemetry --ulg tests/fixtures/telemetry/tiny.ulg | head -1` looks clean.

---

## Task 10 [no-chip] — README section + INDEX flip + Plan 7 closeout

Update user-facing docs and the plans index.

**Files:**
- Modify: `README.md` — add a new subsection under the tiling-library section:

  ```
  ### Telemetry import + visualizer

  Import a PX4 ULog or DJI SRT sidecar into a `RecordedTelemetry`-compatible
  JSONL timeline, then render an annotated MP4 from any video + that timeline.

      hailo-tiling-import-telemetry --srt clip.SRT --video clip.MP4 --output clip.jsonl
      hailo-tiling-visualize --video clip.MP4 --telemetry clip.jsonl --output clip.annot.mp4

  Both tools run on a laptop without Hailo hardware. See
  `docs/superpowers/plans/2026-05-28-telemetry-import-visualizer.md` for the
  field-mapping table and `docs/superpowers/specs/2026-05-28-tiling-library-design.md`
  §8.8 for the geospatial-telemetry schema.
  ```
- Modify: `docs/superpowers/plans/INDEX.md` — flip Plan 7 status from `in flight` → `done`.

**Closeout checklist:**
- [ ] `pytest hailo_tiling/tests -q` — all green, ≥ 25 new tests added in this plan.
- [ ] `hailo-tiling-import-telemetry --help` and `hailo-tiling-visualize --help` show the expected surface.
- [ ] The two committed fixtures total < 1.5 MB.
- [ ] No new dependency surface beyond `pyulog`.
- [ ] No file outside `hailo_tiling/telemetry/`, `hailo_tiling/cli/`, `hailo_tiling/tests/`, `pyproject.toml`, `README.md`, `INDEX.md` was modified.

**Acceptance:** README renders correctly, INDEX.md shows Plan 7 done, full test suite green.

---

## Open Questions

1. **Auto-sync from first-detection** — Plan 7 ships only manual / video-creation alignment. A future plan should add `--align first-detection`, which reads a `flight_record.sqlite3` (from Plan 5/6) and finds the offset that maximises overlap between telemetry-says-target-is-visible and detector-says-target-is-visible. Depends on the SQLite full-frame schema being landed first.
2. **ULG → AGL fallback** — `vehicle_local_position.dist_bottom` is only populated if PX4 has a distance sensor. For airframes without one (most SITL setups), the parser falls back to baro altitude minus home elevation. Document in the SRT/ULG help text that the AGL value is "AGL where available, else altitude-above-takeoff." Confirm with a domain expert whether the fallback is good enough for the `AltitudeZoomModifier`'s use case or whether we should leave it `None`.
3. **SRT framerate assumption** — DJI SRTs claim 33-ms intervals (≈ 30.3 fps). The fixture I sampled is exactly 30 fps. If the source video is 60 fps or 24 fps, the SRT's "one row per video frame" claim breaks. Mitigation: rely on the row's ISO timestamp, not its index — that's what Task 3 already does. No action required; documenting the gotcha.
4. **`pyulog` as a hard dependency** — adding it to base `dependencies` (not `optional-dependencies`) costs every install ~150 KB. Alternative: gate it behind `[telemetry-import]` extras and import lazily inside `parse_ulg`. Recommendation: keep it hard for v1 because both CLIs need it and the test suite needs it — making it optional adds a "you forgot to install the extra" failure mode. Revisit if `pyulog` ever picks up a heavier transitive dep.
5. **Visualizer overlay layers** — spec §7.12 mentions detection bbox, tile rectangles, track IDs as additional overlay layers. None of those are in Plan 7 because they need the SQLite full-frame table (Plan 5/6). Plan 7 is intentionally **telemetry-only overlay**; bbox / tile / track overlays are a follow-up.

---

### Critical Files for Implementation

- `/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo_tiling/telemetry/recorded.py` — the JSONL row schema this plan must match for round-trip compatibility.
- `/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo_tiling/telemetry/provider.py` — `TelemetrySnapshot` field list that constrains the parser output.
- `/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo_tiling/cli/warm.py` — reference for argparse style, lazy cv2 import pattern, console-script wiring, and CLI test approach.
- `/home/giladn/tappas_apps/repos/hailo-drone-follow/docs/superpowers/specs/2026-05-28-tiling-library-design.md` — §3.3 (snapshot fields), §7.12 (visualizer roles), §8.8 (ULG/SRT capture protocol and SQLite schema), §11 phases 11–12, §13 success criteria.
- `/home/giladn/tappas_apps/repos/hailo-drone-follow/pyproject.toml` — single source of truth for adding `pyulog` and the two new console scripts.
