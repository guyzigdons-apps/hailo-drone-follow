"""End-to-end round-trip integration tests (Plan 7 Task 6).

Proves the loop spec §13 requires: importer reads a fixture -> writes JSONL ->
``RecordedTelemetry.from_path()`` consumes it -> ``snapshot()`` returns valid
``TelemetrySnapshot`` instances -> the modifiers actually consume those
snapshots without crashing.

Uses public APIs only: the ``hailo-tiling-import-telemetry`` CLI's ``main``
entry point, ``RecordedTelemetry.from_path``, ``AltitudeZoomModifier``, and
``AdaptiveSliceSizingModifier``.
"""
from __future__ import annotations

from pathlib import Path

from hailo_tiling.budget import BudgetMeter
from hailo_tiling.cli.import_telemetry import main as import_main
from hailo_tiling.emitters import TrackROIEmitter
from hailo_tiling.emitters.discovery_grid import _grid_full
from hailo_tiling.modifiers import AdaptiveSliceSizingModifier, AltitudeZoomModifier
from hailo_tiling.telemetry import RecordedTelemetry, TelemetrySnapshot
from hailo_tiling.types import LockState


FIXTURES = Path(__file__).parent / "fixtures" / "telemetry"
ULG_FIXTURE = FIXTURES / "tiny.ulg"
SRT_FIXTURE = FIXTURES / "tiny.srt"


def _meter() -> BudgetMeter:
    return BudgetMeter(budget_inf_per_s=1000.0, fps=30.0)


def _tracking_lock() -> LockState:
    return LockState(
        track_id=42,
        bbox_norm=(0.45, 0.40, 0.05, 0.15),
        status="TRACKING",
        frames_since_seen=0,
        last_velocity=(0.0, 0.0),
    )


def _import_to_jsonl(tmp_path: Path, source_flag: str, fixture: Path) -> Path:
    """Drive the CLI ``main`` end-to-end and return the resulting JSONL path."""
    out = tmp_path / "out.jsonl"
    rc = import_main([source_flag, str(fixture), "--output", str(out)])
    assert rc == 0, f"import_main returned non-zero exit code: {rc}"
    assert out.exists(), "expected the CLI to write the JSONL output"
    return out


# ---------------------------------------------------------------------------
# Test 1 -- ULG -> RecordedTelemetry smoke
# ---------------------------------------------------------------------------

def test_ulg_to_recorded_provider_smoke(tmp_path: Path) -> None:
    jsonl = _import_to_jsonl(tmp_path, "--ulg", ULG_FIXTURE)
    provider = RecordedTelemetry.from_path(jsonl)

    s0 = provider.snapshot(0.0)
    s_mid = provider.snapshot(0.5)
    s_late = provider.snapshot(10.0)

    # All three must be TelemetrySnapshot instances.
    assert isinstance(s0, TelemetrySnapshot)
    assert isinstance(s_mid, TelemetrySnapshot)
    assert isinstance(s_late, TelemetrySnapshot)

    # ``RecordedTelemetry.snapshot(t)`` returns a snapshot whose ``timestamp``
    # is the REQUESTED ``t``, not the stored row's timestamp (see
    # ``recorded.py``: ``replace(self._snaps[idx], timestamp=t)``).
    assert s0.timestamp == 0.0
    assert s_mid.timestamp == 0.5
    # ``snapshot(10.0)`` is well beyond the fixture's last row (~1.9 s),
    # but the provider must still return a valid snapshot at the requested
    # ``t`` -- not crash, not raise.
    assert s_late.timestamp == 10.0

    # The fixture has at least one non-None altitude; verify the round-trip
    # preserves it (proves the JSONL writer + RecordedTelemetry loader agree
    # on the field names).
    assert s0.altitude_agl_m is not None


# ---------------------------------------------------------------------------
# Test 2 -- SRT -> RecordedTelemetry smoke
# ---------------------------------------------------------------------------

def test_srt_to_recorded_provider_smoke(tmp_path: Path) -> None:
    jsonl = _import_to_jsonl(tmp_path, "--srt", SRT_FIXTURE)
    provider = RecordedTelemetry.from_path(jsonl)

    s0 = provider.snapshot(0.0)
    s_mid = provider.snapshot(0.5)
    s_late = provider.snapshot(10.0)

    assert isinstance(s0, TelemetrySnapshot)
    assert isinstance(s_mid, TelemetrySnapshot)
    assert isinstance(s_late, TelemetrySnapshot)

    assert s0.timestamp == 0.0
    assert s_mid.timestamp == 0.5
    assert s_late.timestamp == 10.0

    # SRT carries rel_alt mapped to altitude_agl_m, so the first row must
    # have an altitude after round-trip.
    assert s0.altitude_agl_m is not None
    # SRT carries no inertial data -- velocity_world must round-trip as None.
    assert s0.velocity_world is None
    assert s0.attitude_quat is None


# ---------------------------------------------------------------------------
# Test 3 -- ULG round-trip feeds AltitudeZoomModifier end-to-end
# ---------------------------------------------------------------------------

def test_ulg_recorded_feeds_altitude_zoom_modifier(tmp_path: Path) -> None:
    """Spec §13: telemetry must flow from the imported JSONL through
    ``RecordedTelemetry`` into a modifier's ``modify()`` call without crashing,
    and the ROI tile produced for a TRACKING lock must keep mode ``"s"``
    (per ``AltitudeZoomModifier._rescale_roi`` -- it always emits ``mode="s"``).

    The committed ULG fixture is a tiny SITL log with nearly-flat altitude,
    so the modifier's ROI widths from real fixture snapshots won't vary
    meaningfully. We still feed three (early/mid/late) snapshots to verify
    the plumbing. To verify the altitude->zoom mapping itself, we then feed
    synthetic low- vs high-altitude snapshots through the same modifier and
    assert the ROI width changes -- this proves the altitude field on the
    snapshots round-tripped from the importer reaches the modifier's logic.
    """
    jsonl = _import_to_jsonl(tmp_path, "--ulg", ULG_FIXTURE)
    provider = RecordedTelemetry.from_path(jsonl)

    lock = _tracking_lock()
    src_w, src_h = 3840, 2160
    base_emitter = TrackROIEmitter(
        max_zoom=2.0, target_model_h=40.0, roi_margin_frac=0.25
    )
    base_tiles = list(base_emitter.emit(src_w, src_h, lock, 0, _meter()))

    modifier = AltitudeZoomModifier(
        zoom_at_low_agl=1.0,
        zoom_at_high_agl=2.0,
        low_agl_m=5.0,
        high_agl_m=40.0,
        fallback_max_zoom=2.0,
    )

    # 1) Plumbing: three snapshots from the ULG round-trip -> modifier.
    for t in (0.0, 0.9, 1.5):
        snap = provider.snapshot(t)
        out = modifier.modify(
            list(base_tiles), src_w, src_h, lock, 0, _meter(), snap
        )
        assert len(out) >= 1
        # AltitudeZoomModifier._rescale_roi always emits the ROI as mode "s".
        assert out[0].mode == "s"

    # 2) Altitude-driven width variation: prove the snapshot's altitude_agl_m
    #    field is what the modifier consumes by directly constructing low/high
    #    snapshots (same shape RecordedTelemetry would return) and confirming
    #    the ROI widens at low altitude and narrows at high altitude.
    low = TelemetrySnapshot(altitude_agl_m=5.0)
    high = TelemetrySnapshot(altitude_agl_m=40.0)
    tiles_low = modifier.modify(
        list(base_tiles), src_w, src_h, lock, 0, _meter(), low
    )
    tiles_high = modifier.modify(
        list(base_tiles), src_w, src_h, lock, 0, _meter(), high
    )
    assert tiles_low[0].w > tiles_high[0].w, (
        "ROI width should be larger at low altitude (less zoom) than at high "
        f"altitude (more zoom). Got low.w={tiles_low[0].w} vs "
        f"high.w={tiles_high[0].w}."
    )


# ---------------------------------------------------------------------------
# Test 4 -- SRT round-trip (velocity_world is None) doesn't break
#           AdaptiveSliceSizingModifier
# ---------------------------------------------------------------------------

def test_srt_recorded_no_velocity_does_not_break_adaptive_sizing(
    tmp_path: Path,
) -> None:
    """SRT-sourced snapshots have ``velocity_world is None``. The Plan 2
    ``AdaptiveSliceSizingModifier`` uses only ``lock.bbox_norm[3]`` as its
    scale signal and ignores velocity entirely -- this test pins that
    behaviour as a regression guard against a future change accidentally
    introducing a velocity dependency that would NPE on SRT input.
    """
    jsonl = _import_to_jsonl(tmp_path, "--srt", SRT_FIXTURE)
    provider = RecordedTelemetry.from_path(jsonl)

    # Confirm the fixture round-trip preserves the SRT "no velocity" shape.
    snap = provider.snapshot(0.0)
    assert snap.velocity_world is None, (
        "SRT round-trip should preserve velocity_world=None"
    )

    src_w, src_h = 3840, 2160
    # Use a small-target lock so the modifier's bbox-only scale signal
    # actually picks the small-grid branch (proves the modifier's logic
    # ran end-to-end, not just a noop early return).
    small_target = LockState(
        track_id=1,
        bbox_norm=(0.5, 0.5, 0.02, 0.04),
        status="TRACKING",
        frames_since_seen=0,
        last_velocity=(0.0, 0.0),
    )
    base = _grid_full(src_w, src_h, 3, 2, "m")

    modifier = AdaptiveSliceSizingModifier(
        target_h_thresholds=(0.05, 0.15),
        small_grid=(6, 4),
        medium_grid=(3, 2),
        large_grid=(2, 1),
    )

    # Feed an SRT-sourced snapshot (velocity_world is None) at a couple of
    # representative timestamps -- the modifier must produce the small-grid
    # output (6 * 4 = 24 tiles) without raising.
    for t in (0.0, 0.5, 10.0):
        s = provider.snapshot(t)
        assert s.velocity_world is None
        out = modifier.modify(
            list(base), src_w, src_h, small_target, 0, _meter(), s
        )
        assert len(out) == 24, (
            f"bbox-only path should produce small_grid (6x4=24) tiles, "
            f"got {len(out)} at t={t}"
        )
        # All produced tiles must be discovery-mode "m" (sanity that the
        # modifier rebuilt the discovery grid, not random shapes).
        assert all(tile.mode == "m" for tile in out)
