"""Bit-exact parity: C++ canonicalize_crop matches Python's helper.

Plan 5 Task 3 (docs/superpowers/plans/2026-05-28-gst-cache-plugins.md):
the C++ ``hailo_cache::canonicalize_crop`` MUST produce the same 4-tuple
as ``hailo_tiling.cache.hashing.canonicalize_crop`` for any input — the
writer and reader compute keys with the C++ helper and must collide
with Python-written cache rows.

This test:
1. Invokes the test-only ``cache_keys_cli`` binary built under
   ``gst-hailo-cache/build/tests/``.
2. Feeds it 100 random ``(x, y, w, h, q)`` tuples on stdin.
3. Asserts the printed 4-tuple matches Python's helper for each row.

If the C++ helper hasn't been built yet, the test is SKIPPED with a
clear message (we don't want the Python test matrix to fail just
because the C++ side hasn't been built on this checkout).
"""
from __future__ import annotations

import random
import subprocess
from pathlib import Path

import pytest

from hailo_tiling.cache.hashing import canonicalize_crop
from hailo_tiling.types import CropRect


REPO_ROOT = Path(__file__).resolve().parents[2]
CLI_PATH = REPO_ROOT / "gst-hailo-cache" / "build" / "tests" / "cache_keys_cli"


def _python_canonical(x: int, y: int, w: int, h: int, q: int) -> tuple[int, int, int, int]:
    """Apply the Python helper, mapping q==0 → Python's quantise=None."""
    quantise = q if q > 1 else None
    return canonicalize_crop(
        CropRect(x=x, y=y, w=w, h=h, mode="s"),
        quantise=quantise,
    )


def test_cpp_canonicalize_crop_matches_python_100_random_inputs():
    if not CLI_PATH.exists():
        pytest.skip(
            f"C++ cache_keys_cli not built at {CLI_PATH}. "
            "Run `cd gst-hailo-cache && meson setup build && ninja -C build`."
        )

    # Deterministic seed — same suite of inputs across runs / CI.
    rng = random.Random(20260528)

    # Mix of identity (q ∈ {0, 1}) and quantised (q ∈ {2..32}) cases,
    # plus a handful of boundary values where x or w < q (so the
    # floor-rounding produces 0).
    cases: list[tuple[int, int, int, int, int]] = []
    for _ in range(80):
        x = rng.randint(0, 4096)
        y = rng.randint(0, 4096)
        w = rng.randint(1, 1920)
        h = rng.randint(1, 1080)
        # 30% identity (q==0 or q==1), 70% quantised in [2, 32].
        if rng.random() < 0.3:
            q = rng.choice([0, 1])
        else:
            q = rng.randint(2, 32)
        cases.append((x, y, w, h, q))

    # 20 boundary cases: small values vs large q.
    for _ in range(20):
        x = rng.randint(0, 31)
        y = rng.randint(0, 31)
        w = rng.randint(1, 31)
        h = rng.randint(1, 31)
        q = rng.randint(8, 64)
        cases.append((x, y, w, h, q))

    assert len(cases) == 100

    stdin = "\n".join(f"{x} {y} {w} {h} {q}" for (x, y, w, h, q) in cases) + "\n"
    proc = subprocess.run(
        [str(CLI_PATH)],
        input=stdin,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert proc.returncode == 0, (
        f"cache_keys_cli failed: rc={proc.returncode}\n"
        f"stdout: {proc.stdout}\nstderr: {proc.stderr}"
    )

    out_lines = [ln for ln in proc.stdout.strip().splitlines() if ln]
    assert len(out_lines) == len(cases), (
        f"cli emitted {len(out_lines)} lines for {len(cases)} inputs"
    )

    for (x, y, w, h, q), line in zip(cases, out_lines):
        parts = line.split()
        assert len(parts) == 4, f"bad cli line: {line!r}"
        cpp = (int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3]))
        py = _python_canonical(x, y, w, h, q)
        assert cpp == py, (
            f"parity mismatch for ({x},{y},{w},{h}, q={q}):\n"
            f"  C++:    {cpp}\n"
            f"  Python: {py}"
        )
