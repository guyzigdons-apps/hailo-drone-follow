"""hashing helpers — file SHA-256 + crop-rect canonicalisation."""
from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from hailo_tiling.cache.hashing import canonicalize_crop, file_sha256
from hailo_tiling.types import CropRect


def test_file_sha256_matches_hashlib(tmp_path: Path):
    p = tmp_path / "a.bin"
    payload = b"hello-cache-" * 1024
    p.write_bytes(payload)
    assert file_sha256(p) == hashlib.sha256(payload).hexdigest()


def test_file_sha256_empty_file(tmp_path: Path):
    p = tmp_path / "empty"
    p.write_bytes(b"")
    assert file_sha256(p) == hashlib.sha256(b"").hexdigest()


def test_file_sha256_streams_large_file(tmp_path: Path):
    """Larger than the 1 MiB chunk — verify chunked read is correct."""
    p = tmp_path / "big.bin"
    p.write_bytes(b"x" * (3 * 1024 * 1024 + 17))
    expected = hashlib.sha256(p.read_bytes()).hexdigest()
    assert file_sha256(p) == expected


def test_canonicalize_crop_no_quantise_passes_through():
    r = CropRect(x=123, y=456, w=789, h=321, mode="s")
    assert canonicalize_crop(r) == (123, 456, 789, 321)


def test_canonicalize_crop_quantise_4_rounds_down_to_multiple():
    r = CropRect(x=123, y=457, w=790, h=322, mode="s")
    assert canonicalize_crop(r, quantise=4) == (120, 456, 788, 320)


def test_canonicalize_crop_quantise_none_equivalent_to_default():
    r = CropRect(x=100, y=100, w=640, h=480, mode="s")
    assert canonicalize_crop(r, quantise=None) == canonicalize_crop(r)


def test_canonicalize_crop_quantise_one_is_identity():
    r = CropRect(x=7, y=13, w=11, h=5, mode="s")
    assert canonicalize_crop(r, quantise=1) == (7, 13, 11, 5)
