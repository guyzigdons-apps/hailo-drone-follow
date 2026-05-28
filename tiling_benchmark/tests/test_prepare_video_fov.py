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
