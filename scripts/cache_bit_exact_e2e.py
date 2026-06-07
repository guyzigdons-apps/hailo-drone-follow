"""Pre-flight cache bit-exact E2E validator.

Three-mode comparison that proves the hailo_tiling cache layer is bit-exact:

    Mode A — HefBackend only (no cache). Source of truth.
    Mode B — CachingBackend wrapping HefBackend. Two passes:
             pass 1 populates the SQLite cache and forwards every crop to the
             chip; pass 2 must serve every crop from cache (zero chip calls)
             and return IDENTICAL detections to pass 1.
    Mode C — ReplayBackend reading the populated cache (chip-free). Must
             produce IDENTICAL detections to Mode A.

Extra checks (still part of the validator):
    - floor-quantise hit: a crop offset by 1-3 px from a cached crop must hit
      the cache (CachingBackend with quantise=N>1), chip is NOT invoked.
    - CacheMissError path: an empty cache + ReplayBackend must raise on first
      inference and the message must mention the missing crop tuple.

The script writes one JSON per mode to ``out_dir`` and a diff report. Returns
0 on success, non-zero on any mismatch. Designed to be re-runnable; intermediate
SQLite files are placed in a tmp dir that is reused on re-run.

Usage:
    source setup_env.sh
    python scripts/cache_bit_exact_e2e.py \
        --video /home/giladn/Videos/Drone/Training/DJI_20260528155741_0029_D_prepared__fov50.mp4 \
        --hef /usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef \
        --out-dir tiling_benchmark/runs/cache_e2e_baseline \
        --max-frames 8
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

# Make the repo root importable so hailo_tiling resolves when run as a script.
_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from hailo_tiling.backends.caching import CachingBackend  # noqa: E402
from hailo_tiling.backends.hef import HefBackend  # noqa: E402
from hailo_tiling.backends.replay import ReplayBackend  # noqa: E402
from hailo_tiling.cache import CacheMissError  # noqa: E402
from hailo_tiling.cache.hashing import file_sha256  # noqa: E402
from hailo_tiling.cache.store import SqliteCacheStore  # noqa: E402
from hailo_tiling.types import CropRect  # noqa: E402


# ---------------------------------------------------------------- helpers


def _det_to_tuple(d) -> tuple:
    """Canonicalise a detection (either hef_runtime.Detection or
    hailo_tiling.types.Det) into a comparable tuple.

    Both types use Python floats for x/y/w/h/score and int for cls, so the
    tuple representation is bit-exact across the JSON round-trip the cache
    performs.
    """
    return (int(d.cls), float(d.score), float(d.x), float(d.y),
            float(d.w), float(d.h))


def _normalise_run(run: list[list]) -> list[list[tuple]]:
    """Flatten a (frame_idx -> list[list[Det]]) run to plain tuples."""
    return [[[_det_to_tuple(d) for d in dets] for dets in crops_dets]
            for crops_dets in run]


def _crop_to_dict(c: CropRect) -> dict:
    return {"x": c.x, "y": c.y, "w": c.w, "h": c.h, "mode": c.mode}


def _det_tuple_to_dict(t: tuple) -> dict:
    return {"cls": t[0], "score": t[1], "x": t[2], "y": t[3], "w": t[4], "h": t[5]}


def _dump_run_json(out_path: Path, label: str, video_sha: str, hef_sha: str,
                   crops: Sequence[CropRect], frames: list[list[list[tuple]]]) -> None:
    """Write a deterministic JSON describing one mode's run."""
    doc = {
        "label": label,
        "video_sha256": video_sha,
        "hef_sha256": hef_sha,
        "n_frames": len(frames),
        "n_crops_per_frame": len(crops),
        "crops": [_crop_to_dict(c) for c in crops],
        "frames": [
            {
                "frame": i,
                "per_crop": [
                    [_det_tuple_to_dict(t) for t in dets] for dets in frame_dets
                ],
            }
            for i, frame_dets in enumerate(frames)
        ],
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(doc, indent=2, sort_keys=False))


# ---------------------------------------------------------------- video


def _open_capture(video: Path):
    import cv2  # noqa: WPS433
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise SystemExit(f"cannot open video: {video}")
    return cap


def _read_frames(video: Path, max_frames: int) -> tuple[list, int, int]:
    """Read up to `max_frames` frames into memory + return (frames, w, h).

    For the validator we materialise frames so the same numpy arrays feed all
    three modes — eliminates "different decode pass" as a source of drift.
    """
    import cv2  # noqa: WPS433
    cap = _open_capture(video)
    try:
        src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        frames: list = []
        for _ in range(max_frames):
            ok, frame = cap.read()
            if not ok:
                break
            frames.append(frame)
    finally:
        cap.release()
    return frames, src_w, src_h


def _build_crop_set(src_w: int, src_h: int) -> list[CropRect]:
    """Return a deterministic 3x2 grid of 4:3 crops covering the whole frame.

    The HEF input is 640x480 (4:3); we size each tile's width to floor(src_w/3)
    and derive the 4:3 height so the resize step preserves aspect. The grid is
    placed so the union covers the full frame top-to-bottom, even if individual
    tile heights overlap. Using 6 tiles (vs 4) gives the model enough resolution
    on each subject for detections to actually fire on drone footage at 4K.
    """
    from hailo_tiling.types import MODEL_ASPECT
    tw = src_w // 3
    th = int(round(tw / MODEL_ASPECT))
    # 2-row layout: top row anchored at y=0, bottom row anchored at y=src_h - th.
    y_rows = [0, max(0, src_h - th)]
    crops: list[CropRect] = []
    for y in y_rows:
        for gx in range(3):
            crops.append(CropRect(
                x=gx * tw, y=y, w=tw, h=th, mode="m",
            ))
    return crops


# ---------------------------------------------------------------- chip serialisation


def _wait_for_chip(retries: int = 3, backoff_s: float = 30.0) -> bool:
    """Best-effort: probe `hailortcli scan`, retry while busy.

    3 retries x 30s ~= typical chip-busy window when another process is
    mid-inference.
    """
    import shutil
    import subprocess
    if shutil.which("hailortcli") is None:
        return True  # nothing to gate on; HefBackend.open() will raise if no chip
    for attempt in range(retries):
        proc = subprocess.run(
            ["hailortcli", "scan"], capture_output=True, text=True, timeout=10,
        )
        out = (proc.stdout or "") + (proc.stderr or "")
        if proc.returncode == 0 and "Device" in out:
            return True
        if attempt == retries - 1:
            return False
        time.sleep(backoff_s)
    return False


# ---------------------------------------------------------------- counted hef wrapper


class _CountedHefBackend:
    """Wraps a HefBackend; counts and forwards `infer` calls.

    We can't just monkeypatch HefBackend.infer because CachingBackend holds a
    direct reference; subclassing/wrapping gives us a clean per-mode counter.
    """

    def __init__(self, inner: HefBackend):
        self._inner = inner
        self.call_count = 0
        self.crop_calls: int = 0  # total crops sent to chip

    def infer(self, frame, crops, frame_idx: int):
        self.call_count += 1
        crops_list = list(crops)
        self.crop_calls += len(crops_list)
        return self._inner.infer(frame, crops_list, frame_idx)

    def close(self) -> None:
        self._inner.close()


# ---------------------------------------------------------------- mode runners


def _run_mode_a(frames: list, crops: list[CropRect], hef_path: Path,
                nms_thresh: float) -> tuple[list[list[list[tuple]]], int]:
    """Mode A — HefBackend only, no cache."""
    # raw ids OK: cache bit-exactness validator — IDs pass through symmetrically
    # across modes A/B/C; class_offset would shift all three identically and is
    # irrelevant to the cache-fidelity comparison.
    inner = HefBackend(hef_path=str(hef_path), nms_score_threshold=nms_thresh)
    counted = _CountedHefBackend(inner)
    try:
        out: list[list[list[tuple]]] = []
        for fi, frame in enumerate(frames):
            res = counted.infer(frame, crops, fi)
            out.append([[_det_to_tuple(d) for d in dets] for dets in res])
    finally:
        counted.close()
    return out, counted.crop_calls


def _run_mode_b(frames: list, crops: list[CropRect], hef_path: Path,
                nms_thresh: float, cache_path: Path,
                ) -> tuple[
                    list[list[list[tuple]]],
                    list[list[list[tuple]]],
                    int,
                    int,
                ]:
    """Mode B — CachingBackend(HefBackend), two passes.

    Returns (pass1, pass2, pass1_chip_crop_calls, pass2_chip_crop_calls).
    """
    # raw ids OK: cache bit-exactness validator (see _run_mode_a).
    inner = HefBackend(hef_path=str(hef_path), nms_score_threshold=nms_thresh)
    counted = _CountedHefBackend(inner)
    if cache_path.exists():
        cache_path.unlink()
    store = SqliteCacheStore.open(cache_path)
    try:
        cb = CachingBackend(wrapped=counted, store=store, ppv=1)
        pass1: list[list[list[tuple]]] = []
        for fi, frame in enumerate(frames):
            res = cb.infer(frame, crops, fi)
            pass1.append([[_det_to_tuple(d) for d in dets] for dets in res])
        before_p2 = counted.crop_calls
        pass2: list[list[list[tuple]]] = []
        for fi, frame in enumerate(frames):
            res = cb.infer(frame, crops, fi)
            pass2.append([[_det_to_tuple(d) for d in dets] for dets in res])
        pass2_crop_calls = counted.crop_calls - before_p2
    finally:
        store.close()
        counted.close()
    return pass1, pass2, before_p2, pass2_crop_calls


def _run_mode_c(frames: list, crops: list[CropRect],
                cache_path: Path) -> list[list[list[tuple]]]:
    """Mode C — ReplayBackend, chip-free."""
    store = SqliteCacheStore.open(cache_path)
    try:
        be = ReplayBackend(store=store, ppv=1)
        out: list[list[list[tuple]]] = []
        for fi, frame in enumerate(frames):
            res = be.infer(frame, crops, fi)
            out.append([[_det_to_tuple(d) for d in dets] for dets in res])
    finally:
        store.close()
    return out


# ---------------------------------------------------------------- extra checks


@dataclass
class ExtraCheckResult:
    floor_quantise_ok: bool
    floor_quantise_msg: str
    cache_miss_ok: bool
    cache_miss_msg: str


def _check_floor_quantise_hit(frames: list, hef_path: Path, nms_thresh: float,
                              cache_path: Path) -> tuple[bool, str]:
    """Populate cache with one crop; query offset-by-2px crop with quantise=4
    and assert it hits cache (chip not invoked)."""
    if not frames:
        return False, "no frames"
    if cache_path.exists():
        cache_path.unlink()
    base = CropRect(x=100, y=100, w=640, h=480, mode="m")
    # Offset within q=4 bucket (100 // 4 == 25; 102 // 4 == 25).
    offset = CropRect(x=102, y=103, w=641, h=482, mode="m")
    # raw ids OK: cache bit-exactness validator (see _run_mode_a).
    inner = HefBackend(hef_path=str(hef_path), nms_score_threshold=nms_thresh)
    counted = _CountedHefBackend(inner)
    store = SqliteCacheStore.open(cache_path)
    try:
        cb = CachingBackend(wrapped=counted, store=store, ppv=1, quantise=4)
        # Warm with the base crop.
        cb.infer(frames[0], [base], 0)
        chip_before = counted.crop_calls
        # Query with the offset crop — must hit, chip must NOT be invoked.
        cb.infer(frames[0], [offset], 0)
        chip_after = counted.crop_calls
        if chip_after != chip_before:
            return False, (
                f"floor-quantise miss: chip crop_calls went {chip_before} -> "
                f"{chip_after} (expected no chip invocation for offset crop "
                f"{offset} after warming {base} with quantise=4)"
            )
    finally:
        store.close()
        counted.close()
    return True, "offset crop served from cache (no chip invocation)"


def _check_cache_miss_error(frames: list, tmp_cache: Path) -> tuple[bool, str]:
    """Empty cache + ReplayBackend must raise CacheMissError on first infer.

    Precondition: ``frames`` is non-empty. ``run()`` early-returns when the
    video produced zero frames, so by the time this helper runs we always
    have at least one frame to feed the backend.
    """
    assert frames, "_check_cache_miss_error requires non-empty frames; run() must early-return otherwise"
    if tmp_cache.exists():
        tmp_cache.unlink()
    store = SqliteCacheStore.open(tmp_cache)
    crop = CropRect(x=0, y=0, w=640, h=480, mode="m")
    try:
        be = ReplayBackend(store=store, ppv=1)
        try:
            be.infer(frames[0], [crop], frame_idx=99)
        except CacheMissError as exc:
            msg = str(exc)
            ok = ("frame_idx=99" in msg
                  and ("0,0,640,480" in msg or "(0, 0, 640, 480)" in msg))
            return ok, msg
        return False, "expected CacheMissError, none raised"
    finally:
        store.close()


# ---------------------------------------------------------------- main


def run(video: Path, hef: Path, out_dir: Path, max_frames: int,
        nms_thresh: float, work_dir: Path) -> int:
    if not video.exists():
        print(f"ERROR: video does not exist: {video}", file=sys.stderr)
        return 2
    if not hef.exists():
        print(f"ERROR: hef does not exist: {hef}", file=sys.stderr)
        return 2

    if not _wait_for_chip():
        print("ERROR: Hailo chip not available after 3 retries", file=sys.stderr)
        return 3

    out_dir.mkdir(parents=True, exist_ok=True)
    work_dir.mkdir(parents=True, exist_ok=True)

    print(f"[0/5] hashing video + hef ...", flush=True)
    video_sha = file_sha256(video)
    hef_sha = file_sha256(hef)
    print(f"      video_sha={video_sha[:16]}...  hef_sha={hef_sha[:16]}...")

    print(f"[1/5] reading frames (max={max_frames}) ...", flush=True)
    frames, src_w, src_h = _read_frames(video, max_frames)
    if not frames:
        print("ERROR: read zero frames from video", file=sys.stderr)
        return 4
    crops = _build_crop_set(src_w, src_h)
    print(f"      n_frames={len(frames)}  src={src_w}x{src_h}  n_crops={len(crops)}")

    print(f"[2/5] Mode A: HefBackend only ...", flush=True)
    mode_a, a_calls = _run_mode_a(frames, crops, hef, nms_thresh)
    _dump_run_json(out_dir / "mode_a_hef.json", "mode_a_hef",
                   video_sha, hef_sha, crops, mode_a)
    total_a_dets = sum(len(d) for f in mode_a for d in f)
    print(f"      chip crop_calls={a_calls}  total_dets={total_a_dets}")

    print(f"[3/5] Mode B: CachingBackend(HefBackend), 2 passes ...", flush=True)
    cache_b = work_dir / "mode_b.sqlite3"
    pass1, pass2, p1_calls, p2_calls = _run_mode_b(
        frames, crops, hef, nms_thresh, cache_b,
    )
    _dump_run_json(out_dir / "mode_b_caching.json", "mode_b_caching",
                   video_sha, hef_sha, crops, pass2)
    print(f"      pass1 chip crop_calls={p1_calls}  pass2 chip crop_calls={p2_calls}")

    print(f"[4/5] Mode C: ReplayBackend (chip-free) ...", flush=True)
    mode_c = _run_mode_c(frames, crops, cache_b)
    _dump_run_json(out_dir / "mode_c_replay.json", "mode_c_replay",
                   video_sha, hef_sha, crops, mode_c)
    print(f"      replayed {len(mode_c)} frames OK")

    print(f"[5/5] extra checks (floor-quantise + cache miss) ...", flush=True)
    fq_ok, fq_msg = _check_floor_quantise_hit(
        frames, hef, nms_thresh, work_dir / "fq_check.sqlite3",
    )
    cm_ok, cm_msg = _check_cache_miss_error(frames, work_dir / "cm_check.sqlite3")
    extra = ExtraCheckResult(
        floor_quantise_ok=fq_ok, floor_quantise_msg=fq_msg,
        cache_miss_ok=cm_ok, cache_miss_msg=cm_msg,
    )
    print(f"      floor-quantise: {'OK' if fq_ok else 'FAIL'}  {fq_msg}")
    print(f"      cache-miss:     {'OK' if cm_ok else 'FAIL'}  {cm_msg!r}")

    # -------- diff -------------------------------------------------------
    print(f"\n[diff] comparing modes ...", flush=True)
    deviations: list[str] = []

    if pass1 != pass2:
        deviations.append(
            f"Mode B pass1 != pass2 (cache round-trip drift)"
        )
    if pass2 != mode_a:
        deviations.append(
            f"Mode B pass2 != Mode A (CachingBackend output drift vs HefBackend)"
        )
    if mode_c != mode_a:
        deviations.append(
            f"Mode C (ReplayBackend) != Mode A (HefBackend)"
        )
    if p2_calls != 0:
        deviations.append(
            f"Mode B pass2 invoked chip {p2_calls} times (expected 0 — full cache hit)"
        )
    if not fq_ok:
        deviations.append(f"floor-quantise hit FAILED: {fq_msg}")
    if not cm_ok:
        deviations.append(f"cache-miss-error path FAILED: {cm_msg!r}")

    report = {
        "video": str(video),
        "video_sha256": video_sha,
        "hef": str(hef),
        "hef_sha256": hef_sha,
        "n_frames": len(frames),
        "n_crops_per_frame": len(crops),
        "mode_a_chip_crop_calls": a_calls,
        "mode_b_pass1_chip_crop_calls": p1_calls,
        "mode_b_pass2_chip_crop_calls": p2_calls,
        "floor_quantise_ok": fq_ok,
        "floor_quantise_msg": fq_msg,
        "cache_miss_ok": cm_ok,
        "cache_miss_msg": cm_msg,
        "deviations": deviations,
        "status": "OK" if not deviations else "FAIL",
    }
    (out_dir / "diff_report.json").write_text(json.dumps(report, indent=2))

    if deviations:
        print("\n[diff] FAIL")
        for d in deviations:
            print(f"   - {d}")
        return 1
    print("\n[diff] OK — all three modes bit-exact, extra checks pass")
    return 0


def _build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="cache_bit_exact_e2e")
    ap.add_argument(
        "--video",
        required=True,
        type=Path,
        help="Input video file; first --max-frames are decoded once and reused across all three modes.",
    )
    ap.add_argument(
        "--hef",
        default=Path(
            "/usr/local/hailo/resources/models/hailo10h/"
            "hailo_yolov8n_4_classes_vga.hef"
        ),
        type=Path,
        help="HEF model file (default: hailo10h yolov8n 4-classes VGA).",
    )
    ap.add_argument(
        "--out-dir",
        required=True,
        type=Path,
        help="Output directory for per-mode JSON dumps and diff_report.json.",
    )
    ap.add_argument(
        "--max-frames",
        type=int,
        default=8,
        help="Maximum number of frames to decode and run through every mode (default: 8).",
    )
    ap.add_argument(
        "--nms-thresh",
        type=float,
        default=0.25,
        help="NMS score threshold passed to HefBackend; baseline JSONs are pinned at this value, do not change without re-baselining.",
    )
    ap.add_argument(
        "--work-dir",
        type=Path,
        default=None,
        help="Where intermediate SQLite caches live (default: <out-dir>/_work).",
    )
    return ap


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_argparser().parse_args(argv)
    work_dir = args.work_dir or (args.out_dir / "_work")
    return run(
        video=args.video, hef=args.hef, out_dir=args.out_dir,
        max_frames=args.max_frames, nms_thresh=args.nms_thresh,
        work_dir=work_dir,
    )


if __name__ == "__main__":
    raise SystemExit(main())
