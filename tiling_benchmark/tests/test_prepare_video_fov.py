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
    argv = pv.build_fov_ffmpeg_cmd(
        input_path=Path("/x/clip_prepared.MP4"),
        output_path=Path("/x/clip_prepared__fov50.mp4"),
        fov_deg=50,
        nice=None,
        ionice=None,
    )
    vf_idx = argv.index("-vf")
    assert argv[vf_idx + 1] == (
        "crop=4007:2254:(in_w-4007)/2:(in_h-2254)/2,"
        "scale=3840:2160:flags=lanczos"
    )
    assert argv[0] == "ffmpeg"
    assert argv[-1] == "/x/clip_prepared__fov50.mp4"


def test_build_fov_ffmpeg_cmd_with_nice_and_ionice(pv):
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


import hashlib
import json
import os


# ---------------------------------------------------------------------------
# SHA-256 helper
# ---------------------------------------------------------------------------


def test_sha256_of_file_matches_hashlib(pv, tmp_path):
    p = tmp_path / "x.bin"
    p.write_bytes(b"hello world\n")
    expected = hashlib.sha256(b"hello world\n").hexdigest()
    assert pv.sha256_of_file(p) == expected


def test_sha256_of_file_chunks_large_file(pv, tmp_path):
    p = tmp_path / "big.bin"
    data = b"A" * (5 * 1024 * 1024 + 17)
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
    manifest = tmp_path / "fov_variants_manifest.json"
    rec = _record("a.MP4", "fov70", "a__fov70.mp4", "aa" * 32)
    pv.append_manifest_record(manifest, rec)
    data = json.loads(manifest.read_text())
    assert data == [rec]


def test_append_manifest_record_appends_to_existing(pv, tmp_path):
    manifest = tmp_path / "fov_variants_manifest.json"
    r1 = _record("a.MP4", "fov70", "a__fov70.mp4", "aa" * 32)
    r2 = _record("a.MP4", "fov60", "a__fov60.mp4", "bb" * 32)
    pv.append_manifest_record(manifest, r1)
    pv.append_manifest_record(manifest, r2)
    data = json.loads(manifest.read_text())
    assert data == [r1, r2]


def test_append_manifest_record_is_atomic_via_temp_file(pv, tmp_path, monkeypatch):
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
    assert Path(src).parent == manifest.parent
    assert Path(src).name != manifest.name
    assert dst == str(manifest)


def test_append_manifest_record_rejects_non_list_existing(pv, tmp_path):
    manifest = tmp_path / "fov_variants_manifest.json"
    manifest.write_text(json.dumps({"not": "a list"}))
    with pytest.raises(ValueError):
        pv.append_manifest_record(
            manifest, _record("a.MP4", "fov70", "a__fov70.mp4", "aa" * 32)
        )
