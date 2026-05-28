# FOV Emulation Source-Data Prep — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend `tiling_benchmark/prepare_video.py` so it can emit per-FOV 4K variants (`fov70`/`fov60`/`fov50`) from a rotation-stripped DJI 6K clip, and write a per-output SHA-256 manifest. The overnight video-prep agent's outputs and manifest at `/home/giladn/Videos/Drone/Training/` are the canonical artifacts; this plan codifies their **reproducibility recipe** so any of the variants can be regenerated from source on demand.

**Architecture:** A single `prepare_video.py` script gains an `--emit-fov 70,60,50` (also accepts `--emit-fov 50` singletons) flag. Each variant runs the spec §8.3 ffmpeg recipe (libx265 CRF 18 preset slow, lanczos scaling, no upscaling) into `<stem>__fov<N>.mp4` alongside the rotation-stripped output. After each variant completes, the script computes the file's SHA-256, builds a record, and **atomically appends** it to `fov_variants_manifest.json` in the same directory. The atomic append is read-modify-write into a temp file, then `os.replace()` onto the manifest, so a crash mid-run never corrupts the manifest. The crop dimensions, command builder, hashing helper, and manifest writer are extracted into pure helpers so they can be unit-tested without invoking ffmpeg.

**Tech Stack:** Python 3.10+, pytest, stdlib only (subprocess, hashlib, json, tempfile, pathlib, argparse). No new runtime dependencies. The existing project venv at `./hailo-apps/venv_hailo_apps` is reused. Tests do not invoke ffmpeg — they assert on the argv list constructed by the builder.

**Spec reference:** `docs/superpowers/specs/2026-05-28-tiling-library-design.md` §8 ("Experimental Setup: Source Data and FOV Emulation"), Phase 6 in §11.

**Branch:** `tiling-benchmark` (continuing from Plan 2; latest commit `2e9909f`).

---

## Background — what the overnight agent already produced

A parallel video-prep agent is running ffmpeg on 7 rotation-stripped DJI 6K clips in `/home/giladn/Videos/Drone/Training/`, producing FOV-70/60/50 sibling MP4s and a manifest at `fov_variants_manifest.json`. As of the time this plan was written, 13 of 21 outputs had landed. **Those artifacts are the canonical experimental record. This plan does not regenerate them.** Its job is to encode the recipe into `prepare_video.py` so:

- The manifest schema in the repo matches what the overnight agent produced.
- Re-running `python tiling_benchmark/prepare_video.py <prepared.MP4> --emit-fov 70,60,50` for any of the 7 source clips yields a variant whose SHA-256 may differ (x265 is not byte-deterministic across runs — see Open Question 1) but whose pixel content is functionally equivalent under the spec's definition of the experiment.

The overnight manifest's records use this shape (one record per variant, list-of-objects at the top level):

```json
{
  "input":       "DJI_20260528155741_0029_D_prepared.MP4",
  "variant":     "fov70",
  "output":      "DJI_20260528155741_0029_D_prepared__fov70.mp4",
  "output_bytes": 97821909,
  "sha256":      "00c7f232c6b7660cd5d20f5e6b6dfb7e8c4cf35fe2d43305caf721ec0f847ff6",
  "ffmpeg_cmd":  "nice -n 10 ionice -c 3 ffmpeg -y -i /…/<input> -vf scale=3840:2160:flags=lanczos -c:v libx265 -crf 18 -preset slow -an /…/<output>"
}
```

`input` and `output` are stored as **basenames** (relative to the manifest's parent directory). `ffmpeg_cmd` is stored as a **single string** with arguments joined by single spaces. The overnight agent prefixes `nice -n 10 ionice -c 3` for system politeness — this prefix is the only environmental difference and is captured exactly as it ran. The in-repo runner has these as opt-in flags so an in-repo `--emit-fov` invocation with `--nice 10 --ionice 3` produces a `ffmpeg_cmd` string that matches the overnight agent character-for-character.

---

## File Structure

**Files this plan creates:**

```
tiling_benchmark/
  tests/
    __init__.py
    test_prepare_video_fov.py        # unit tests: CLI parser, crop math, ffmpeg-cmd builder, manifest atomic-write, overnight-manifest schema importer
```

**Files this plan modifies:**

- `tiling_benchmark/prepare_video.py` — adds `--emit-fov`, `--nice`, `--ionice`, `--manifest` CLI flags; factors out helpers (`fov_to_crop_dims`, `build_fov_ffmpeg_cmd`, `sha256_of_file`, `append_manifest_record`, `import_overnight_manifest`); end-to-end flow extended so when `--emit-fov` is set, the script (a) ensures the rotation-stripped input exists, (b) for each requested FOV produces `<stem>__fov<N>.mp4`, (c) hashes the output, (d) atomic-appends a manifest entry.

**Files this plan does NOT touch:**

- `hailo_tiling/` package — the cache layer that consumes these videos lands in Plan 4+. Plan 3 only produces source data.
- `tiling_benchmark/run_pxt_bench.py`, `run_pxt_yolov8m.py`, `run_gt_batch.py`, `tiling_record.py`, etc. — these consume the prepared variants but do not change in Plan 3.
- **`tiling_benchmark/run_dynamic.py`** does not exist in this repo today; this plan does not create it (any rumours of it are Plan 8 work).
- `dynamic_tiling/` — unchanged.
- Anything under `/home/giladn/Videos/` — the overnight agent's outputs are read-only artifacts from Plan 3's perspective; the plan only verifies they conform to the schema.

---

## Pre-flight: virtual environment + ffmpeg sanity check

All commands assume the project venv is active. If you've never set it up, run `source setup_env.sh` once. After that, use direct binaries — `/home/giladn/tappas_apps/repos/hailo-drone-follow/hailo-apps/venv_hailo_apps/bin/python` and `.../bin/pytest`. Tests in this plan never shell out to ffmpeg, but the runtime entry point does.

Verify ffmpeg + ffprobe are on `$PATH`:

```bash
ffmpeg -version | head -1
ffprobe -version | head -1
```

Expected: both print a version banner. If missing, the plan's tests still pass (they assert on argv list construction, not on ffmpeg execution), but the runtime entry point will fail at first use. Document the dependency in the plan output but do not block on it.

For brevity in this plan, paths are written as `python` and `pytest`; the executor should resolve them to the venv binaries.

---

## Task 1: Add `tiling_benchmark/tests/` package + the failing CLI-parser test

**Files:**
- Create: `tiling_benchmark/tests/__init__.py`
- Create: `tiling_benchmark/tests/test_prepare_video_fov.py` (CLI parser portion only — other tests added in later tasks)

- [ ] **Step 1: Create the tests package.**

```bash
mkdir -p tiling_benchmark/tests
```

```python
# tiling_benchmark/tests/__init__.py
```
(empty file)

- [ ] **Step 2: Write the failing CLI-parser test.**

`tiling_benchmark/` is a script collection, not a Python package — `prepare_video.py` is imported as a module via `importlib.util` so the tests don't require an `__init__.py` at the top level. The test loads the module from its absolute path and asserts on the argparse spec.

```python
# tiling_benchmark/tests/test_prepare_video_fov.py
"""Unit tests for prepare_video.py --emit-fov flag and helpers.

These tests never invoke ffmpeg. They assert on:
  - argparse accepts --emit-fov 70,60,50 (and singletons like --emit-fov 50)
  - fov_to_crop_dims returns the exact dimensions from spec §8.2
  - build_fov_ffmpeg_cmd produces the exact argv list from spec §8.3
  - append_manifest_record is atomic (temp-file + os.replace)
  - import_overnight_manifest validates the schema of the overnight agent's output

The module under test is loaded via importlib because tiling_benchmark/
is a script collection, not a package.
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
PV_PATH = REPO_ROOT / "tiling_benchmark" / "prepare_video.py"


def _load_prepare_video():
    spec = importlib.util.spec_from_file_location("prepare_video", PV_PATH)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["prepare_video"] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def pv():
    return _load_prepare_video()


# ---------------------------------------------------------------------------
# CLI parser
# ---------------------------------------------------------------------------


def test_cli_accepts_emit_fov_triple(pv):
    """--emit-fov 70,60,50 yields [70, 60, 50]."""
    parser = pv.build_arg_parser()
    args = parser.parse_args(["clip.MP4", "--emit-fov", "70,60,50"])
    assert args.emit_fov == [70, 60, 50]


def test_cli_accepts_emit_fov_singleton(pv):
    """--emit-fov 50 yields [50]."""
    parser = pv.build_arg_parser()
    args = parser.parse_args(["clip.MP4", "--emit-fov", "50"])
    assert args.emit_fov == [50]


def test_cli_emit_fov_default_is_none(pv):
    """Without --emit-fov, the attribute is None (preserves old behaviour)."""
    parser = pv.build_arg_parser()
    args = parser.parse_args(["clip.MP4"])
    assert args.emit_fov is None


def test_cli_emit_fov_rejects_unknown_value(pv):
    """--emit-fov 40 is rejected — only 70, 60, 50 are valid in v1."""
    parser = pv.build_arg_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(["clip.MP4", "--emit-fov", "40"])


def test_cli_accepts_nice_and_ionice(pv):
    """--nice and --ionice are accepted (matches overnight agent's prefix)."""
    parser = pv.build_arg_parser()
    args = parser.parse_args(
        ["clip.MP4", "--emit-fov", "70", "--nice", "10", "--ionice", "3"]
    )
    assert args.nice == 10
    assert args.ionice == 3


def test_cli_manifest_path_override(pv):
    """--manifest selects a custom manifest path; default is sibling JSON."""
    parser = pv.build_arg_parser()
    args = parser.parse_args(
        ["clip.MP4", "--emit-fov", "70", "--manifest", "/tmp/x.json"]
    )
    assert args.manifest == Path("/tmp/x.json")
```

- [ ] **Step 3: Run the test, expect failure.**

```bash
pytest tiling_benchmark/tests/test_prepare_video_fov.py -v
```
Expected: FAIL — `prepare_video.build_arg_parser` does not exist; `args.emit_fov` etc. do not exist.

- [ ] **Step 4: Refactor `prepare_video.py` to expose `build_arg_parser()` and add the new flags. Do NOT add logic for the flags yet — only argparse plumbing.**

Replace the body of `main()` so the parser is factored into a module-level builder:

```python
# tiling_benchmark/prepare_video.py  (only the changed bits shown)

def _parse_fov_list(s: str) -> list[int]:
    """Parse `--emit-fov 70,60,50` → [70, 60, 50]. Validates the FOV set."""
    allowed = {70, 60, 50}
    try:
        out = [int(x) for x in s.split(",") if x.strip()]
    except ValueError as e:
        raise argparse.ArgumentTypeError(
            f"--emit-fov must be comma-separated integers; got {s!r}"
        ) from e
    if not out:
        raise argparse.ArgumentTypeError(
            f"--emit-fov requires at least one value; got {s!r}"
        )
    bad = [v for v in out if v not in allowed]
    if bad:
        raise argparse.ArgumentTypeError(
            f"--emit-fov values must be in {sorted(allowed)}; got {bad}"
        )
    return out


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("input", type=Path, help="Source video file.")
    ap.add_argument("--output", type=Path, default=None,
                    help="Explicit output path for the rotation-stripped "
                         "result (default: <stem>_prepared<ext> next to input).")
    ap.add_argument("--force", action="store_true",
                    help="Re-encode even if cached output is newer.")
    ap.add_argument("--copy", action="store_true",
                    help="Copy unmodified input to output when no re-encode "
                         "is needed.")
    ap.add_argument("--verify", action="store_true",
                    help="Verify output via ffprobe + one-frame gst decode.")
    ap.add_argument("--emit-fov", type=_parse_fov_list, default=None,
                    metavar="LIST",
                    help="Comma-separated list of FOV variants to emit "
                         "from the rotation-stripped output. Valid values: "
                         "70, 60, 50. Example: --emit-fov 70,60,50")
    ap.add_argument("--nice", type=int, default=None,
                    help="If set, prefix ffmpeg with 'nice -n <N>'.")
    ap.add_argument("--ionice", type=int, default=None,
                    help="If set, prefix ffmpeg with 'ionice -c <N>'.")
    ap.add_argument("--manifest", type=Path, default=None,
                    help="Path to the FOV-variants manifest JSON. "
                         "Default: fov_variants_manifest.json in the "
                         "output's parent directory.")
    return ap


def main() -> None:
    args = build_arg_parser().parse_args()
    # … existing logic unchanged for this task …
```

- [ ] **Step 5: Run the test, expect pass.**

```bash
pytest tiling_benchmark/tests/test_prepare_video_fov.py -v
```
Expected: 6 passed.

- [ ] **Step 6: Run the existing prepare-video smoke (read-only ffprobe path on an existing file) to confirm no regression.**

```bash
python tiling_benchmark/prepare_video.py --help
```
Expected: prints the help text including `--emit-fov`, `--nice`, `--ionice`, `--manifest`. No exception.

- [ ] **Step 7: Commit.**

```bash
git add tiling_benchmark/tests/__init__.py tiling_benchmark/tests/test_prepare_video_fov.py tiling_benchmark/prepare_video.py
git commit -m "tiling_benchmark: prepare_video CLI gains --emit-fov / --nice / --ionice / --manifest"
```

---

## Task 2: Implement `fov_to_crop_dims` (FOV → crop dimensions math)

**Files:**
- Modify: `tiling_benchmark/prepare_video.py` — add `fov_to_crop_dims()`.
- Modify: `tiling_benchmark/tests/test_prepare_video_fov.py` — add crop-math tests.

- [ ] **Step 1: Add the failing test.**

Append to `tiling_benchmark/tests/test_prepare_video_fov.py`:

```python
# ---------------------------------------------------------------------------
# Crop dimensions math (spec §8.2)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "fov,expected",
    [
        (70, (6016, 3384)),  # full 6K (no crop; "FOV-70" native)
        (60, (4963, 2792)),  # crop_ratio = tan(30°) / tan(35°) ≈ 0.8247
        (50, (4007, 2254)),  # crop_ratio = tan(25°) / tan(35°) ≈ 0.6661
    ],
)
def test_fov_to_crop_dims(pv, fov, expected):
    """Crop dimensions match the table in spec §8.2."""
    assert pv.fov_to_crop_dims(fov) == expected


def test_fov_to_crop_dims_rejects_invalid(pv):
    """Anything other than 70/60/50 raises ValueError."""
    with pytest.raises(ValueError):
        pv.fov_to_crop_dims(40)
    with pytest.raises(ValueError):
        pv.fov_to_crop_dims(80)
```

- [ ] **Step 2: Run, expect failure.**

```bash
pytest tiling_benchmark/tests/test_prepare_video_fov.py::test_fov_to_crop_dims -v
```
Expected: FAIL — `prepare_video.fov_to_crop_dims` does not exist.

- [ ] **Step 3: Implement `fov_to_crop_dims`.**

Add to `tiling_benchmark/prepare_video.py`:

```python
import math

# 6K source dimensions (DJI Mavic 4 Pro main camera; spec §8.1)
SRC_6K_W = 6016
SRC_6K_H = 3384
SRC_NATIVE_FOV_DEG = 70

# Allowed FOV variants. Anything outside this set would require upscaling
# from the 6K source, which is disallowed (spec §8.2).
ALLOWED_FOVS = (70, 60, 50)


def fov_to_crop_dims(fov_deg: int) -> tuple[int, int]:
    """Return the (crop_w, crop_h) needed to emulate `fov_deg` from a 6K source.

    Derived from `crop_ratio = tan(fov_deg/2) / tan(70°/2)` per spec §8.2.
    Result is `int(round(SRC_6K_{W,H} * crop_ratio))`.

    Verifies the result is ≤ source dims and ≥ 4K output dims (3840×2160);
    raises ValueError on either violation (we never upscale).
    """
    if fov_deg not in ALLOWED_FOVS:
        raise ValueError(
            f"fov_deg must be one of {ALLOWED_FOVS}; got {fov_deg}"
        )
    ratio = (
        math.tan(math.radians(fov_deg) / 2.0)
        / math.tan(math.radians(SRC_NATIVE_FOV_DEG) / 2.0)
    )
    crop_w = int(round(SRC_6K_W * ratio))
    crop_h = int(round(SRC_6K_H * ratio))
    if crop_w > SRC_6K_W or crop_h > SRC_6K_H:
        raise ValueError(
            f"fov_deg={fov_deg} requires crop {crop_w}x{crop_h} > source "
            f"{SRC_6K_W}x{SRC_6K_H}; refusing to upscale"
        )
    if crop_w < 3840 or crop_h < 2160:
        raise ValueError(
            f"fov_deg={fov_deg} requires crop {crop_w}x{crop_h} < 4K output "
            f"3840x2160; refusing to upscale"
        )
    return crop_w, crop_h
```

- [ ] **Step 4: Run, expect pass.**

```bash
pytest tiling_benchmark/tests/test_prepare_video_fov.py::test_fov_to_crop_dims tiling_benchmark/tests/test_prepare_video_fov.py::test_fov_to_crop_dims_rejects_invalid -v
```
Expected: 4 passed (3 parametrised + 1 rejection).

- [ ] **Step 5: Cross-check against the spec table.**

```bash
python -c "from tiling_benchmark.prepare_video import fov_to_crop_dims" 2>/dev/null || python -c "import importlib.util, sys; spec=importlib.util.spec_from_file_location('pv','tiling_benchmark/prepare_video.py'); m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m); [print(f, m.fov_to_crop_dims(f)) for f in (70,60,50)]"
```
Expected output:
```
70 (6016, 3384)
60 (4963, 2792)
50 (4007, 2254)
```

- [ ] **Step 6: Commit.**

```bash
git add tiling_benchmark/prepare_video.py tiling_benchmark/tests/test_prepare_video_fov.py
git commit -m "tiling_benchmark: fov_to_crop_dims — FOV → 6K crop-dimension math"
```

---

## Task 3: Implement `build_fov_ffmpeg_cmd` (the canonical ffmpeg argv builder)

**Files:**
- Modify: `tiling_benchmark/prepare_video.py` — add `build_fov_ffmpeg_cmd()`.
- Modify: `tiling_benchmark/tests/test_prepare_video_fov.py` — add builder tests.

The builder takes `(input, output, fov, nice, ionice)` and returns the argv list. The matching shell command string (with single-space joins) is what gets stored in the manifest's `ffmpeg_cmd` field. The string form is reproduced from the argv list via `" ".join(argv)` — tests assert on both.

- [ ] **Step 1: Add failing builder tests.**

Append to `tiling_benchmark/tests/test_prepare_video_fov.py`:

```python
# ---------------------------------------------------------------------------
# ffmpeg command builder (spec §8.3)
# ---------------------------------------------------------------------------


def test_build_fov_ffmpeg_cmd_fov70(pv):
    """FOV-70 = no crop, only scale (full 6K → 4K)."""
    argv = pv.build_fov_ffmpeg_cmd(
        input_path=Path("/x/clip_prepared.MP4"),
        output_path=Path("/x/clip_prepared__fov70.mp4"),
        fov_deg=70,
        nice=None,
        ionice=None,
    )
    assert argv == [
        "ffmpeg", "-y",
        "-i", "/x/clip_prepared.MP4",
        "-vf", "scale=3840:2160:flags=lanczos",
        "-c:v", "libx265", "-crf", "18", "-preset", "slow",
        "-an",
        "/x/clip_prepared__fov70.mp4",
    ]


def test_build_fov_ffmpeg_cmd_fov60(pv):
    """FOV-60 = center crop 4963x2792, then scale to 4K."""
    argv = pv.build_fov_ffmpeg_cmd(
        input_path=Path("/x/clip_prepared.MP4"),
        output_path=Path("/x/clip_prepared__fov60.mp4"),
        fov_deg=60,
        nice=None,
        ionice=None,
    )
    assert argv == [
        "ffmpeg", "-y",
        "-i", "/x/clip_prepared.MP4",
        "-vf",
        "crop=4963:2792:(in_w-4963)/2:(in_h-2792)/2,"
        "scale=3840:2160:flags=lanczos",
        "-c:v", "libx265", "-crf", "18", "-preset", "slow",
        "-an",
        "/x/clip_prepared__fov60.mp4",
    ]


def test_build_fov_ffmpeg_cmd_fov50(pv):
    """FOV-50 = center crop 4007x2254, then scale to 4K."""
    argv = pv.build_fov_ffmpeg_cmd(
        input_path=Path("/x/clip_prepared.MP4"),
        output_path=Path("/x/clip_prepared__fov50.mp4"),
        fov_deg=50,
        nice=None,
        ionice=None,
    )
    # Check the -vf value precisely.
    vf_idx = argv.index("-vf")
    assert argv[vf_idx + 1] == (
        "crop=4007:2254:(in_w-4007)/2:(in_h-2254)/2,"
        "scale=3840:2160:flags=lanczos"
    )
    assert argv[0] == "ffmpeg"
    assert argv[-1] == "/x/clip_prepared__fov50.mp4"


def test_build_fov_ffmpeg_cmd_with_nice_and_ionice(pv):
    """nice + ionice prefixes prepend in this order: nice, ionice, ffmpeg."""
    argv = pv.build_fov_ffmpeg_cmd(
        input_path=Path("/x/in.MP4"),
        output_path=Path("/x/out__fov70.mp4"),
        fov_deg=70,
        nice=10,
        ionice=3,
    )
    assert argv[:6] == ["nice", "-n", "10", "ionice", "-c", "3"]
    assert argv[6] == "ffmpeg"


def test_build_fov_ffmpeg_cmd_string_matches_overnight_manifest(pv):
    """Sanity-check: the joined argv reproduces the overnight agent's string format."""
    argv = pv.build_fov_ffmpeg_cmd(
        input_path=Path("/home/giladn/Videos/Drone/Training/DJI_20260528155741_0029_D_prepared.MP4"),
        output_path=Path("/home/giladn/Videos/Drone/Training/DJI_20260528155741_0029_D_prepared__fov70.mp4"),
        fov_deg=70,
        nice=10,
        ionice=3,
    )
    s = " ".join(argv)
    assert s == (
        "nice -n 10 ionice -c 3 ffmpeg -y "
        "-i /home/giladn/Videos/Drone/Training/DJI_20260528155741_0029_D_prepared.MP4 "
        "-vf scale=3840:2160:flags=lanczos "
        "-c:v libx265 -crf 18 -preset slow -an "
        "/home/giladn/Videos/Drone/Training/DJI_20260528155741_0029_D_prepared__fov70.mp4"
    )
```

> **Note on the `-vf` value for FOV-60/50.** The overnight agent's manifest serializes the `-vf` argument inside single quotes (because it was constructed via shell). When we render the same argv list as a single space-joined string, the parentheses in `(in_w-4963)/2` are not shell-quoted because we are joining a Python `list[str]` for storage in JSON, not for re-execution. Tests on the FOV-60/50 string form therefore check the argv list, not a quoted shell string. The plan-wide reproducibility property is "argv list passed to `subprocess.run` is byte-identical"; the `ffmpeg_cmd` JSON field is a human-readable record, not something we re-execute.

- [ ] **Step 2: Run, expect failure.**

```bash
pytest tiling_benchmark/tests/test_prepare_video_fov.py -k build_fov_ffmpeg_cmd -v
```
Expected: FAIL — `prepare_video.build_fov_ffmpeg_cmd` does not exist.

- [ ] **Step 3: Implement `build_fov_ffmpeg_cmd`.**

Add to `tiling_benchmark/prepare_video.py`:

```python
def _vf_filter_for_fov(fov_deg: int) -> str:
    """Build the -vf filter chain string for a given FOV.

    - fov_deg=70 → no crop, only lanczos scale to 4K.
    - fov_deg<70 → center crop to fov-derived dims, then lanczos scale to 4K.
    """
    if fov_deg == SRC_NATIVE_FOV_DEG:
        return "scale=3840:2160:flags=lanczos"
    crop_w, crop_h = fov_to_crop_dims(fov_deg)
    return (
        f"crop={crop_w}:{crop_h}:(in_w-{crop_w})/2:(in_h-{crop_h})/2,"
        f"scale=3840:2160:flags=lanczos"
    )


def build_fov_ffmpeg_cmd(
    input_path: Path,
    output_path: Path,
    fov_deg: int,
    nice: int | None = None,
    ionice: int | None = None,
) -> list[str]:
    """Construct the ffmpeg argv list for one FOV variant.

    Encodes the canonical recipe from spec §8.3:
        libx265 CRF 18, preset slow, no audio, lanczos scaling, center crop
        for fov<70, no upscaling.

    Optional `nice` / `ionice` prefixes are prepended in the order
    `nice -n N ionice -c N ffmpeg …` to match the overnight prep agent.
    """
    if fov_deg not in ALLOWED_FOVS:
        raise ValueError(
            f"fov_deg must be one of {ALLOWED_FOVS}; got {fov_deg}"
        )
    prefix: list[str] = []
    if nice is not None:
        prefix += ["nice", "-n", str(nice)]
    if ionice is not None:
        prefix += ["ionice", "-c", str(ionice)]
    return prefix + [
        "ffmpeg", "-y",
        "-i", str(input_path),
        "-vf", _vf_filter_for_fov(fov_deg),
        "-c:v", "libx265", "-crf", "18", "-preset", "slow",
        "-an",
        str(output_path),
    ]
```

- [ ] **Step 4: Run, expect pass.**

```bash
pytest tiling_benchmark/tests/test_prepare_video_fov.py -k build_fov_ffmpeg_cmd -v
```
Expected: 5 passed.

- [ ] **Step 5: Commit.**

```bash
git add tiling_benchmark/prepare_video.py tiling_benchmark/tests/test_prepare_video_fov.py
git commit -m "tiling_benchmark: build_fov_ffmpeg_cmd — canonical FOV-variant recipe argv"
```

---

## Task 4: Implement `sha256_of_file` + `append_manifest_record` (atomic manifest writer)

**Files:**
- Modify: `tiling_benchmark/prepare_video.py` — add hashing + atomic manifest writer.
- Modify: `tiling_benchmark/tests/test_prepare_video_fov.py` — add hashing + manifest tests.

The atomic-write pattern: read the existing manifest (or `[]` if missing), append the new record, write the full list to a sibling temp file (`<manifest>.tmp.<pid>`), then `os.replace()` it onto the destination. `os.replace()` is atomic on POSIX (`rename(2)` semantics) — the manifest is either the old version or the new version at every observation point, never partial.

- [ ] **Step 1: Add failing hashing + manifest tests.**

Append to `tiling_benchmark/tests/test_prepare_video_fov.py`:

```python
import hashlib
import json
import os


# ---------------------------------------------------------------------------
# SHA-256 helper
# ---------------------------------------------------------------------------


def test_sha256_of_file_matches_hashlib(pv, tmp_path):
    """sha256_of_file returns the hex digest of the file's bytes."""
    p = tmp_path / "x.bin"
    p.write_bytes(b"hello world\n")
    expected = hashlib.sha256(b"hello world\n").hexdigest()
    assert pv.sha256_of_file(p) == expected


def test_sha256_of_file_chunks_large_file(pv, tmp_path):
    """A multi-MB file hashes correctly when read in chunks."""
    p = tmp_path / "big.bin"
    data = b"A" * (5 * 1024 * 1024 + 17)  # not a chunk multiple
    p.write_bytes(data)
    assert pv.sha256_of_file(p) == hashlib.sha256(data).hexdigest()


# ---------------------------------------------------------------------------
# Atomic manifest writer
# ---------------------------------------------------------------------------


def _record(input_name: str, variant: str, output_name: str, sha: str) -> dict:
    return {
        "input": input_name,
        "variant": variant,
        "output": output_name,
        "output_bytes": 12345,
        "sha256": sha,
        "ffmpeg_cmd": f"ffmpeg -y -i {input_name} … {output_name}",
    }


def test_append_manifest_record_creates_new_file(pv, tmp_path):
    """When the manifest doesn't exist, create it as a list with one record."""
    manifest = tmp_path / "fov_variants_manifest.json"
    rec = _record("a.MP4", "fov70", "a__fov70.mp4", "aa" * 32)
    pv.append_manifest_record(manifest, rec)
    data = json.loads(manifest.read_text())
    assert data == [rec]


def test_append_manifest_record_appends_to_existing(pv, tmp_path):
    """Appending preserves prior entries in order."""
    manifest = tmp_path / "fov_variants_manifest.json"
    r1 = _record("a.MP4", "fov70", "a__fov70.mp4", "aa" * 32)
    r2 = _record("a.MP4", "fov60", "a__fov60.mp4", "bb" * 32)
    pv.append_manifest_record(manifest, r1)
    pv.append_manifest_record(manifest, r2)
    data = json.loads(manifest.read_text())
    assert data == [r1, r2]


def test_append_manifest_record_is_atomic_via_temp_file(pv, tmp_path, monkeypatch):
    """The writer writes to a sibling temp file then os.replace()s it.

    We verify by patching os.replace and asserting it was called with a
    sibling temp file → the manifest path. This proves we never write
    directly to the manifest path (which would leave a half-written file
    on crash mid-write).
    """
    manifest = tmp_path / "fov_variants_manifest.json"
    replace_calls: list[tuple[str, str]] = []
    real_replace = os.replace

    def fake_replace(src, dst):
        replace_calls.append((str(src), str(dst)))
        real_replace(src, dst)

    monkeypatch.setattr(pv.os, "replace", fake_replace)
    pv.append_manifest_record(
        manifest, _record("a.MP4", "fov70", "a__fov70.mp4", "aa" * 32)
    )
    assert len(replace_calls) == 1
    src, dst = replace_calls[0]
    # The temp file must be a sibling (same parent) of the manifest.
    assert Path(src).parent == manifest.parent
    assert Path(src).name != manifest.name
    assert dst == str(manifest)


def test_append_manifest_record_rejects_non_list_existing(pv, tmp_path):
    """If the existing file isn't a JSON array, raise — never overwrite silently."""
    manifest = tmp_path / "fov_variants_manifest.json"
    manifest.write_text(json.dumps({"not": "a list"}))
    with pytest.raises(ValueError):
        pv.append_manifest_record(
            manifest, _record("a.MP4", "fov70", "a__fov70.mp4", "aa" * 32)
        )
```

> The `monkeypatch.setattr(pv.os, "replace", …)` requires `prepare_video.py` to `import os` at module top level. Confirm before running.

- [ ] **Step 2: Run, expect failure.**

```bash
pytest tiling_benchmark/tests/test_prepare_video_fov.py -k "sha256 or append_manifest" -v
```
Expected: FAIL — neither symbol exists.

- [ ] **Step 3: Implement the helpers.**

Add to `tiling_benchmark/prepare_video.py` (ensure `import hashlib`, `import os`, `import json` at the top):

```python
import hashlib
import os


_SHA_CHUNK = 1024 * 1024  # 1 MiB


def sha256_of_file(path: Path) -> str:
    """Return the SHA-256 hex digest of `path`'s bytes (read in 1 MiB chunks)."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(_SHA_CHUNK), b""):
            h.update(chunk)
    return h.hexdigest()


def _read_manifest_or_empty(manifest: Path) -> list[dict]:
    """Load the manifest JSON or return []. Raises if the file isn't a list."""
    if not manifest.is_file():
        return []
    raw = manifest.read_text()
    if not raw.strip():
        return []
    data = json.loads(raw)
    if not isinstance(data, list):
        raise ValueError(
            f"manifest at {manifest} is not a JSON array; refusing to overwrite"
        )
    return data


def append_manifest_record(manifest: Path, record: dict) -> None:
    """Append `record` to the manifest list, writing atomically.

    Atomicity: write the full updated list to a sibling temp file, fsync,
    then os.replace() it onto the manifest path. Any crash mid-run leaves
    the old manifest intact.
    """
    manifest.parent.mkdir(parents=True, exist_ok=True)
    entries = _read_manifest_or_empty(manifest)
    entries.append(record)
    tmp = manifest.with_name(f".{manifest.name}.tmp.{os.getpid()}")
    serialized = json.dumps(entries, indent=2)
    with open(tmp, "w") as f:
        f.write(serialized)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, manifest)
```

- [ ] **Step 4: Run, expect pass.**

```bash
pytest tiling_benchmark/tests/test_prepare_video_fov.py -k "sha256 or append_manifest" -v
```
Expected: 5 passed.

- [ ] **Step 5: Commit.**

```bash
git add tiling_benchmark/prepare_video.py tiling_benchmark/tests/test_prepare_video_fov.py
git commit -m "tiling_benchmark: sha256_of_file + atomic append_manifest_record"
```

---

## Task 5: Wire `--emit-fov` into `main()` (the runtime entry point)

This task is mostly plumbing — it ties the previously-tested helpers together into the runtime flow. There is no unit test that exercises ffmpeg; the assertion is "the code path exists and is wired correctly". An optional smoke step calls it against a tiny synthetic 6K source (10 frames, 6016×3384, generated by `ffmpeg -f lavfi`) at the end of the task — gated behind `[ -n "$RUN_PREPARE_VIDEO_SMOKE" ]` so CI doesn't pay the cost.

**Files:**
- Modify: `tiling_benchmark/prepare_video.py` — extend `main()` to handle `--emit-fov`.

- [ ] **Step 1: Extend `main()` to drive the FOV pipeline.**

Replace the tail of `main()` (after the rotation-strip path) with:

```python
def _resolve_manifest_path(output: Path, override: Path | None) -> Path:
    """Default manifest is fov_variants_manifest.json next to the output."""
    if override is not None:
        return override
    return output.parent / "fov_variants_manifest.json"


def _emit_fov_variant(
    src_4k_input: Path,
    fov_deg: int,
    manifest: Path,
    nice: int | None,
    ionice: int | None,
    force: bool,
) -> Path:
    """Produce one FOV variant + append to the manifest. Return output path."""
    stem = src_4k_input.stem  # e.g. 'clip_prepared'
    output = src_4k_input.parent / f"{stem}__fov{fov_deg}.mp4"

    if output.is_file() and not force:
        # Variant already exists; trust the on-disk file. If the manifest
        # already has a matching entry (same output basename + sha256), do
        # not re-append. If it doesn't, hash the file and append.
        existing = _read_manifest_or_empty(manifest)
        if any(e.get("output") == output.name for e in existing):
            print(f"  fov{fov_deg}: cached {output.name}; manifest already records it; skipping")
            return output
        print(f"  fov{fov_deg}: cached {output.name}; recording into manifest")
    else:
        argv = build_fov_ffmpeg_cmd(
            input_path=src_4k_input,
            output_path=output,
            fov_deg=fov_deg,
            nice=nice,
            ionice=ionice,
        )
        proc = run(argv)
        if proc.returncode != 0:
            print(
                f"ERROR: ffmpeg exited with code {proc.returncode} "
                f"for fov{fov_deg}", file=sys.stderr,
            )
            sys.exit(1)

    sha = sha256_of_file(output)
    record = {
        "input": src_4k_input.name,
        "variant": f"fov{fov_deg}",
        "output": output.name,
        "output_bytes": output.stat().st_size,
        "sha256": sha,
        "ffmpeg_cmd": " ".join(
            build_fov_ffmpeg_cmd(src_4k_input, output, fov_deg, nice, ionice)
        ),
    }
    append_manifest_record(manifest, record)
    return output
```

And in `main()`, after the rotation-strip path finishes (i.e. after the existing `print(f"done: {output}")` line on the rotation branch, **and** after the no-rotation early-exit branch), add:

```python
    # --- FOV emulation (Plan 3) ------------------------------------------
    if args.emit_fov:
        # Determine the 4K source for FOV emission: it is the rotation-
        # stripped output (`output` in the rotation branch) or the
        # original input (no-rotation branch). The no-rotation branch
        # short-circuits with sys.exit(0) above; we restructure that
        # branch so it falls through here when --emit-fov is set.
        src_4k = output if rot != 0 else args.input
        manifest_path = _resolve_manifest_path(src_4k, args.manifest)
        print(f"emitting FOV variants: {args.emit_fov}")
        print(f"  source: {src_4k}")
        print(f"  manifest: {manifest_path}")
        for fov_deg in args.emit_fov:
            _emit_fov_variant(
                src_4k_input=src_4k,
                fov_deg=fov_deg,
                manifest=manifest_path,
                nice=args.nice,
                ionice=args.ionice,
                force=args.force,
            )
        print(f"done: {len(args.emit_fov)} FOV variant(s) emitted")
    sys.exit(0)
```

> **Important refactor:** the existing no-rotation branch (`if rot == 0:`) calls `sys.exit(0)` after handling `--copy` / `--verify`. To let `--emit-fov` work on already-landscape inputs, replace those early exits with `pass`-through statements when `args.emit_fov` is set. Concretely:
>
> ```python
>     if rot == 0:
>         if args.copy:
>             print(f"video has no rotation metadata; copying to: {output}")
>             output.parent.mkdir(parents=True, exist_ok=True)
>             shutil.copy2(args.input, output)
>             if args.verify:
>                 verify_output(output)
>             if not args.emit_fov:
>                 sys.exit(0)
>         else:
>             print(f"video is already landscape-oriented; using directly: "
>                   f"{args.input}")
>             if args.verify:
>                 verify_output(args.input)
>             if not args.emit_fov:
>                 sys.exit(0)
> ```
>
> The same `if not args.emit_fov: sys.exit(0)` guard replaces the rotation-branch early exit. The FOV block above runs only when control reaches it.

- [ ] **Step 2: Run the full test suite — no behaviour change to the unit tests.**

```bash
pytest tiling_benchmark/tests/ -v
```
Expected: all tests pass (CLI, crop math, builder, hashing, manifest).

- [ ] **Step 3: Optional smoke test (gated behind env var).**

```bash
if [ -n "$RUN_PREPARE_VIDEO_SMOKE" ]; then
  TMP=$(mktemp -d)
  ffmpeg -y -f lavfi -i 'testsrc=duration=1:size=6016x3384:rate=2' \
      -c:v libx265 -crf 28 -preset ultrafast "$TMP/synthetic.MP4" 2>&1 | tail -5
  python tiling_benchmark/prepare_video.py "$TMP/synthetic.MP4" \
      --emit-fov 70,60,50
  ls -la "$TMP"
  cat "$TMP/fov_variants_manifest.json"
fi
```
Expected (when run): three sibling `synthetic__fov{70,60,50}.mp4` files exist; manifest is a JSON array of three records. Skipped silently otherwise.

- [ ] **Step 4: Commit.**

```bash
git add tiling_benchmark/prepare_video.py
git commit -m "tiling_benchmark: prepare_video --emit-fov drives FOV-variant pipeline end-to-end"
```

---

## Task 6: Implement `import_overnight_manifest` + verify it on the overnight agent's artifacts

The overnight video-prep agent has already produced 13+ of 21 expected outputs under `/home/giladn/Videos/Drone/Training/` and is writing them to `fov_variants_manifest.json` in that directory. Plan 3 owns the **schema definition** for that manifest — this task adds an importer that validates the schema and, optionally, re-hashes the on-disk files to confirm they match the recorded `sha256`. The validator is the single source of truth for "is the overnight artifact set conformant".

**Files:**
- Modify: `tiling_benchmark/prepare_video.py` — add `import_overnight_manifest()`.
- Modify: `tiling_benchmark/tests/test_prepare_video_fov.py` — add importer tests.

- [ ] **Step 1: Add failing importer tests.**

Append to `tiling_benchmark/tests/test_prepare_video_fov.py`:

```python
# ---------------------------------------------------------------------------
# Overnight-manifest schema importer
# ---------------------------------------------------------------------------


REQUIRED_KEYS = {"input", "variant", "output", "output_bytes", "sha256", "ffmpeg_cmd"}
ALLOWED_VARIANTS = {"fov70", "fov60", "fov50"}


def test_import_overnight_manifest_validates_required_keys(pv, tmp_path):
    """A record missing a required key triggers a validation error."""
    manifest = tmp_path / "m.json"
    manifest.write_text(json.dumps([{"input": "a.MP4", "variant": "fov70"}]))
    with pytest.raises(ValueError, match="missing keys"):
        pv.import_overnight_manifest(manifest, verify_sha=False)


def test_import_overnight_manifest_validates_variant_value(pv, tmp_path):
    """An unknown variant value (e.g. 'fov40') is rejected."""
    manifest = tmp_path / "m.json"
    manifest.write_text(json.dumps([{
        "input": "a.MP4", "variant": "fov40", "output": "a__fov40.mp4",
        "output_bytes": 1, "sha256": "00" * 32, "ffmpeg_cmd": "x",
    }]))
    with pytest.raises(ValueError, match="variant"):
        pv.import_overnight_manifest(manifest, verify_sha=False)


def test_import_overnight_manifest_accepts_good_records(pv, tmp_path):
    """A conformant manifest returns the parsed list unchanged."""
    manifest = tmp_path / "m.json"
    good = [{
        "input": "a.MP4", "variant": "fov70", "output": "a__fov70.mp4",
        "output_bytes": 100, "sha256": "a" * 64, "ffmpeg_cmd": "x",
    }]
    manifest.write_text(json.dumps(good))
    out = pv.import_overnight_manifest(manifest, verify_sha=False)
    assert out == good


def test_import_overnight_manifest_verifies_sha_when_requested(pv, tmp_path):
    """With verify_sha=True, the importer re-hashes each output file."""
    blob = b"some bytes\n"
    sha = hashlib.sha256(blob).hexdigest()
    out_file = tmp_path / "a__fov70.mp4"
    out_file.write_bytes(blob)
    manifest = tmp_path / "fov_variants_manifest.json"
    manifest.write_text(json.dumps([{
        "input": "a.MP4", "variant": "fov70", "output": "a__fov70.mp4",
        "output_bytes": len(blob), "sha256": sha, "ffmpeg_cmd": "x",
    }]))
    # Passes verification:
    pv.import_overnight_manifest(manifest, verify_sha=True)

    # Now break the recorded sha and confirm we error.
    manifest.write_text(json.dumps([{
        "input": "a.MP4", "variant": "fov70", "output": "a__fov70.mp4",
        "output_bytes": len(blob), "sha256": "ff" * 32, "ffmpeg_cmd": "x",
    }]))
    with pytest.raises(ValueError, match="sha256 mismatch"):
        pv.import_overnight_manifest(manifest, verify_sha=True)
```

- [ ] **Step 2: Run, expect failure.**

```bash
pytest tiling_benchmark/tests/test_prepare_video_fov.py -k import_overnight_manifest -v
```
Expected: FAIL — `prepare_video.import_overnight_manifest` does not exist.

- [ ] **Step 3: Implement `import_overnight_manifest`.**

Add to `tiling_benchmark/prepare_video.py`:

```python
_OVERNIGHT_REQUIRED_KEYS = {
    "input", "variant", "output", "output_bytes", "sha256", "ffmpeg_cmd",
}
_OVERNIGHT_ALLOWED_VARIANTS = {"fov70", "fov60", "fov50"}


def import_overnight_manifest(manifest: Path, verify_sha: bool = True) -> list[dict]:
    """Load and validate the overnight FOV manifest at `manifest`.

    Raises ValueError on:
      - file not a JSON list at top level
      - missing required keys in any record
      - variant value not in {fov70, fov60, fov50}
      - (when verify_sha=True) recomputed SHA-256 doesn't match the recorded one
      - (when verify_sha=True) output file missing or wrong byte count

    Returns the parsed list of records on success.
    """
    if not manifest.is_file():
        raise FileNotFoundError(f"manifest not found: {manifest}")
    data = json.loads(manifest.read_text())
    if not isinstance(data, list):
        raise ValueError(f"manifest at {manifest} is not a JSON array")
    for i, rec in enumerate(data):
        if not isinstance(rec, dict):
            raise ValueError(f"manifest[{i}] is not an object: {rec!r}")
        missing = _OVERNIGHT_REQUIRED_KEYS - rec.keys()
        if missing:
            raise ValueError(
                f"manifest[{i}] missing keys {sorted(missing)}; got {sorted(rec.keys())}"
            )
        if rec["variant"] not in _OVERNIGHT_ALLOWED_VARIANTS:
            raise ValueError(
                f"manifest[{i}] variant {rec['variant']!r} not in "
                f"{sorted(_OVERNIGHT_ALLOWED_VARIANTS)}"
            )
        if not isinstance(rec["sha256"], str) or len(rec["sha256"]) != 64:
            raise ValueError(
                f"manifest[{i}] sha256 must be a 64-char hex string; got {rec['sha256']!r}"
            )
    if verify_sha:
        for rec in data:
            out = manifest.parent / rec["output"]
            if not out.is_file():
                raise ValueError(f"output file not found: {out}")
            actual_bytes = out.stat().st_size
            if actual_bytes != rec["output_bytes"]:
                raise ValueError(
                    f"{out.name}: output_bytes mismatch (recorded "
                    f"{rec['output_bytes']}, actual {actual_bytes})"
                )
            actual_sha = sha256_of_file(out)
            if actual_sha != rec["sha256"]:
                raise ValueError(
                    f"{out.name}: sha256 mismatch (recorded {rec['sha256']}, "
                    f"actual {actual_sha})"
                )
    return data
```

- [ ] **Step 4: Run, expect pass.**

```bash
pytest tiling_benchmark/tests/test_prepare_video_fov.py -k import_overnight_manifest -v
```
Expected: 4 passed.

- [ ] **Step 5: Run the importer against the overnight agent's actual manifest.**

This is an **observational** check, not a unit test — the overnight agent may still be writing, so a record-count mismatch is informational, not a failure.

```bash
python - <<'PY'
import importlib.util, sys
from pathlib import Path
spec = importlib.util.spec_from_file_location(
    "pv", Path("tiling_benchmark/prepare_video.py")
)
pv = importlib.util.module_from_spec(spec); spec.loader.exec_module(pv)
m = Path("/home/giladn/Videos/Drone/Training/fov_variants_manifest.json")
if not m.is_file():
    print("overnight manifest not present yet; skipping import check")
    sys.exit(0)
records = pv.import_overnight_manifest(m, verify_sha=True)
print(f"overnight manifest: {len(records)} records, all schema-valid, all sha-verified")
inputs = sorted({r["input"] for r in records})
variants = sorted({r["variant"] for r in records})
print(f"  inputs: {inputs}")
print(f"  variants: {variants}")
PY
```
Expected (best case, when overnight agent has finished all 21 outputs):
```
overnight manifest: 21 records, all schema-valid, all sha-verified
  inputs: [7 prepared MP4 basenames]
  variants: ['fov50', 'fov60', 'fov70']
```
Expected (mid-run): N < 21 records, all valid. If any record fails sha verification or schema check, the import raises — that's a hard signal that the overnight agent's output deviates from Plan 3's schema and we need to reconcile.

- [ ] **Step 6: Commit.**

```bash
git add tiling_benchmark/prepare_video.py tiling_benchmark/tests/test_prepare_video_fov.py
git commit -m "tiling_benchmark: import_overnight_manifest validates the FOV-variants JSON schema"
```

---

## Task 7: README section documenting the recipe + reproducibility procedure

**Files:**
- Modify: `tiling_benchmark/README.md` — add an "FOV emulation" section near the top.

(If `tiling_benchmark/README.md` doesn't have a clear "Source data prep" section, create one beneath the existing introduction.)

- [ ] **Step 1: Add the section.**

Append to `tiling_benchmark/README.md`:

```markdown
## FOV emulation (source-data prep for the paper-reported ablation table)

The paper-reported ablation rows are evaluated at three emulated horizontal FOVs — 70° (native), 60°, and 50° — all rendered to 4K (3840×2160). Each FOV variant is a separate physical MP4 file alongside the rotation-stripped source, so the inference pipeline sees only the claimed resolution at runtime.

### FOV → crop dimensions

Crop dimensions are derived from the source's native FOV (70° on the DJI Mavic 4 Pro main camera, 6016×3384) via:

```
crop_ratio = tan(fov_target / 2) / tan(70° / 2)
crop_w = round(6016 * crop_ratio)
crop_h = round(3384 * crop_ratio)
```

| FOV  | Crop from 6K        | Output resolution | Scale factor (downscale) |
|------|---------------------|-------------------|--------------------------|
| 70°  | full 6016 × 3384    | 3840 × 2160       | 1.57× down               |
| 60°  | center 4963 × 2792  | 3840 × 2160       | 1.29× down               |
| 50°  | center 4007 × 2254  | 3840 × 2160       | 1.04× down (no upscale)  |

### ffmpeg recipe (spec §8.3)

For each FOV, the canonical ffmpeg invocation is:

```bash
# FOV-70 (no crop):
ffmpeg -y -i clip_prepared.MP4 \
    -vf "scale=3840:2160:flags=lanczos" \
    -c:v libx265 -crf 18 -preset slow -an \
    clip_prepared__fov70.mp4

# FOV-60 / FOV-50 (center crop, then scale):
ffmpeg -y -i clip_prepared.MP4 \
    -vf "crop=W:H:(in_w-W)/2:(in_h-H)/2,scale=3840:2160:flags=lanczos" \
    -c:v libx265 -crf 18 -preset slow -an \
    clip_prepared__fov<N>.mp4
```

Where `W` and `H` come from the table above. `crf 18 -preset slow` is the spec's quality target — the emulation must not introduce codec artifacts that confound the FOV ablation.

### Reproducibility

To regenerate the FOV variants for any prepared clip:

```bash
python tiling_benchmark/prepare_video.py /path/to/clip.MP4 \
    --emit-fov 70,60,50
```

Outputs land alongside the prepared clip:
- `clip_prepared__fov70.mp4`
- `clip_prepared__fov60.mp4`
- `clip_prepared__fov50.mp4`
- `fov_variants_manifest.json` (appended atomically; one record per variant)

To match the overnight prep agent's environmental settings (nice + ionice for politeness):

```bash
python tiling_benchmark/prepare_video.py clip.MP4 \
    --emit-fov 70,60,50 \
    --nice 10 --ionice 3
```

### Manifest schema

`fov_variants_manifest.json` is a JSON array of records. Each record describes one variant:

```json
{
  "input":        "clip_prepared.MP4",
  "variant":      "fov70",
  "output":       "clip_prepared__fov70.mp4",
  "output_bytes": 97821909,
  "sha256":       "00c7f232…",
  "ffmpeg_cmd":   "ffmpeg -y -i clip_prepared.MP4 -vf scale=3840:2160:flags=lanczos -c:v libx265 -crf 18 -preset slow -an clip_prepared__fov70.mp4"
}
```

`input` and `output` are basenames (relative to the manifest's parent directory). `sha256` is the SHA-256 hex digest of the output file's bytes — used as a cache key by the inference cache (`hailo_tiling.cache`, Plan 4).

### Determinism caveat

H.265 (libx265) encoding is **not byte-deterministic across runs** in general (encoder threading, optimisation passes). Two runs of `--emit-fov 70` on the same source may produce MP4s with identical pixel content but different SHA-256s. The overnight agent's outputs are the canonical artifacts for paper-reported runs; in-repo regeneration is functionally equivalent (same pixels, same crop math) but not byte-equivalent. Plan 4's cache layer will use SHA-256 to key entries, so swapping in regenerated variants invalidates that variant's cache.

To verify the on-disk overnight artifacts match the schema, run:

```bash
python -c "
import importlib.util; from pathlib import Path
s = importlib.util.spec_from_file_location('pv','tiling_benchmark/prepare_video.py')
pv = importlib.util.module_from_spec(s); s.loader.exec_module(pv)
recs = pv.import_overnight_manifest(
    Path('/home/giladn/Videos/Drone/Training/fov_variants_manifest.json'),
    verify_sha=True,
)
print(f'{len(recs)} records validated; SHA-256s match on-disk files')
"
```
```

- [ ] **Step 2: Commit.**

```bash
git add tiling_benchmark/README.md
git commit -m "tiling_benchmark: README section on FOV-emulation recipe + reproducibility"
```

---

## Plan-wide success criteria (self-check before declaring this plan done)

- [ ] `pytest tiling_benchmark/tests -v` reports **all** tests passing (≥ 22 cases across CLI, crop math, ffmpeg builder, hashing, manifest, overnight importer).
- [ ] `python tiling_benchmark/prepare_video.py --help` shows the new flags: `--emit-fov`, `--nice`, `--ionice`, `--manifest`.
- [ ] Running `python tiling_benchmark/prepare_video.py <some_prepared.MP4> --emit-fov 70` against an existing 4K-equivalent file produces a sibling `__fov70.mp4` and appends one record to the sibling manifest. (Manual verification step; not in CI.)
- [ ] The overnight agent's manifest at `/home/giladn/Videos/Drone/Training/fov_variants_manifest.json` passes `import_overnight_manifest(verify_sha=True)` for the records that exist at the time the plan is reviewed. (Observational; the agent may still be running.)
- [ ] No `hailo_tiling/` code changed by this plan.
- [ ] No `dynamic_tiling/` code changed by this plan.
- [ ] No file under `/home/giladn/Videos/` written by this plan's tests (tests use `tmp_path` exclusively).
- [ ] All commits are on the current branch (`tiling-benchmark`); no force-pushes; each task ended with at least one commit.
- [ ] The `pyproject.toml` test discovery in `[tool.pytest.ini_options]` continues to cover `hailo_tiling/tests`; the new `tiling_benchmark/tests` is picked up by pytest's auto-discovery when run from the repo root with no `testpaths` argument override. If running `pytest tiling_benchmark/tests -v` doesn't discover the file, extend `testpaths` in a follow-up commit (or pass the explicit path, which already works).

---

## Out of scope for this plan (handled later)

- **No video re-encoding of the overnight outputs.** This plan adopts the overnight agent's existing artifacts as canonical. The `--emit-fov` runtime path exists for regenerating individual variants on demand, not for bulk re-encoding.
- **No GT generation.** GT for each FOV variant is gated behind Plan 4 (cache machinery) and Plan 5 (GStreamer cache plugins) — once those land, `tiling_benchmark/run_gt_batch.py` can produce per-FOV GT files keyed by the manifest's SHA-256s.
- **No `tiling_benchmark/run_dynamic.py` changes** — that's Plan 8 work (drone-follow migration).
- **No new `hailo_tiling/` package.** `prepare_video.py` lives in `tiling_benchmark/`; the FOV emission is a source-data concern, not a library concern.
- **No `RPI-GS` or `DJI-TELE-12` ablation columns.** Those are real-hardware captures (spec §8.6, §8.7) handled in Plan 8.
- **No runtime FOV emulator** (`hailofov_emulate` GStreamer element) — explicitly deferred per spec §8.4 ("non-blocking nice-to-have").
- **No Zenodo upload / reference data fetcher.** That ships in the paper-artifacts plan (Plan 9).
- **No determinism enforcement** — see Open Question 1; this plan accepts that x265 is non-byte-deterministic and tracks it as a known limitation rather than working around it.

---

## Open Questions

1. **Is byte-determinism of libx265 encoding required, or is "same pixels, same crop math" sufficient?**
   Practically, x265 is not byte-deterministic across runs (encoder threading, internal heuristics). The overnight agent's outputs become the canonical artifacts whose SHA-256s key the Plan 4 cache. Regenerating a variant via `prepare_video.py --emit-fov` produces a functionally equivalent file but with a different SHA-256, which would invalidate that variant's cache. **Recommendation:** treat the overnight outputs as canonical; document the determinism caveat in README; do not attempt to coerce x265 to deterministic mode (it would require `-x265-params pools=1:frame-threads=1` plus a fixed compile-time tune, and even then the bitstream is not guaranteed reproducible across libx265 versions). Revisit if Plan 4 reveals that cache invalidation is a frequent operational burden.

2. **Should `prepare_video.py --emit-fov` also produce the rotation-strip output in the same run, or assume the rotation strip ran first?**
   The current plan's wiring (Task 5) handles both: when called on a rotated source, the script first re-encodes to `<stem>_prepared<ext>`, then emits FOV variants from that. When called on an already-landscape source, it emits FOV variants directly from the input. The overnight agent's manifest shows the latter mode: `input` is a `_prepared.MP4` because the overnight agent ran rotation strip as a separate step. **Recommendation:** keep the current behaviour — `--emit-fov` opportunistically runs rotation strip if needed, but the common case (already-prepared source) is also supported. If experiments reveal the dual-mode wiring is confusing, split into two CLIs in a follow-up.

3. **Does the manifest need a `created_at` timestamp per record?**
   The overnight agent's manifest records do not include a timestamp. Adding one would help with debugging ("when was this variant produced?") but introduces a non-deterministic field that would make manifest-level diffs noisy. **Recommendation:** do NOT add `created_at` to the per-record schema in v1 — the file's mtime on disk already provides this signal. If a use case emerges (e.g. correlating manifest entries to a specific overnight prep run), add a top-level `manifest_metadata` object alongside the records in a follow-up. Keep the v1 schema minimal so the in-repo writer and the overnight agent stay aligned.

4. **Should the importer (Task 6) be exposed as its own CLI (`hailo-tiling-validate-manifest`) or stay as an internal helper?**
   For v1, internal helper is sufficient — Task 6's Step 5 observational check is a one-liner that anyone can run. Promote to CLI in Plan 4 if the cache layer needs frequent manifest validation.

5. **What happens if the overnight manifest already contains a record for `<output>` and `--emit-fov` is re-run on the same source without `--force`?**
   The current `_emit_fov_variant` (Task 5) skips re-encoding when the output file exists and the manifest already records it; it doesn't re-hash. With `--force`, it always re-encodes and re-appends — producing a second record with potentially-different SHA-256 (see Open Question 1). **Recommendation:** acceptable; the manifest is append-only and reviewers can de-duplicate on `output` if needed. Document in the README "rerunning with --force creates a second record per variant" if it surprises anyone.

---

### Critical Files for Implementation

- `/home/giladn/tappas_apps/repos/hailo-drone-follow/tiling_benchmark/prepare_video.py`
- `/home/giladn/tappas_apps/repos/hailo-drone-follow/tiling_benchmark/tests/test_prepare_video_fov.py`
- `/home/giladn/tappas_apps/repos/hailo-drone-follow/tiling_benchmark/tests/__init__.py`
- `/home/giladn/tappas_apps/repos/hailo-drone-follow/tiling_benchmark/README.md`
- `/home/giladn/Videos/Drone/Training/fov_variants_manifest.json` (canonical artifact; read-only reference for Task 6's schema-import verification)
