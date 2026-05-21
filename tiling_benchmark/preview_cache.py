"""In-RAM / on-disk preview cache for the tiling_benchmark overlay viewer.

The overlay viewer needs fast random-access to source-video frames at a
display-friendly resolution. Re-decoding the original 6016x3384 HEVC stream
on every UI event costs ~64 ms per forward step and ~7 s per reverse step
(full GOP rewind). This module decodes the video sequentially in a
background thread, resizes each frame to a target preview resolution with
letterboxing, and stores the result as a numpy ``memmap`` so subsequent
viewer launches re-use the cache when the source video has not changed.

Public API: see :class:`PreviewCache`.
"""

from __future__ import annotations

import hashlib
import json
import os
import threading
from pathlib import Path
from typing import Callable, Optional

import cv2
import numpy as np


def _video_fingerprint(video_path: str, n_frames: int,
                       target_w: int, target_h: int) -> tuple[str, float, int]:
    """Return (hash8, mtime, size) for the given video."""
    abs_path = os.path.abspath(video_path)
    st = os.stat(abs_path)
    mtime = st.st_mtime
    size = st.st_size
    payload = f"{abs_path}|{mtime}|{size}|{n_frames}|{target_w}x{target_h}"
    h = hashlib.sha1(payload.encode("utf-8")).hexdigest()[:8]
    return h, mtime, size


def _letterbox_layout(src_w: int, src_h: int,
                      target_w: int, target_h: int
                      ) -> tuple[float, int, int, int, int]:
    """Compute letterbox scale + offsets for fitting (src_w, src_h) into
    (target_w, target_h) while preserving aspect ratio.

    Returns (scale, resized_w, resized_h, top, left).
    """
    scale = min(target_w / src_w, target_h / src_h)
    resized_w = max(1, int(src_w * scale))
    resized_h = max(1, int(src_h * scale))
    top = (target_h - resized_h) // 2
    left = (target_w - resized_w) // 2
    return scale, resized_w, resized_h, top, left


class PreviewCache:
    """Memmap-backed RGB preview cache for a single source video.

    Construction is non-blocking: a background populator thread is spawned
    that sequentially decodes the video, resizes each frame (with
    letterboxing — borders are left as the mmap's initial zeros), and writes
    the result into the mmap. UI callers poll :meth:`get` and fall back to a
    full-resolution decode when the cache entry is not yet ready or when the
    zoom is so high that cached pixels would be sub-pixel-undersampled.
    """

    def __init__(self, video_path: str, cache_dir: Path,
                 target_w: int = 1920, target_h: int = 1080,
                 on_progress: Optional[Callable[[int, int], None]] = None):
        self._video_path = str(video_path)
        self._cache_dir = Path(cache_dir)
        self._cache_dir.mkdir(parents=True, exist_ok=True)
        self._target_w = int(target_w)
        self._target_h = int(target_h)
        self._on_progress = on_progress

        # Probe video for n_frames + native dims.
        cap = cv2.VideoCapture(self._video_path)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"cannot open video {video_path}")
        self._n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self._src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()
        if self._n_frames <= 0:
            raise RuntimeError(f"video has 0 frames: {video_path}")

        # Compute letterbox layout (constant for life of cache).
        (self._scale, self._resized_w, self._resized_h,
         self._top, self._left) = _letterbox_layout(
            self._src_w, self._src_h, self._target_w, self._target_h,
        )

        # Filenames.
        h8, mtime, size = _video_fingerprint(
            self._video_path, self._n_frames, self._target_w, self._target_h,
        )
        stem = Path(self._video_path).stem
        base = f"{stem}_{h8}_{self._target_w}x{self._target_h}"
        self._bin_path = self._cache_dir / f"{base}.bin"
        self._json_path = self._cache_dir / f"{base}.bin.json"

        # State (mutated by populator + readers).
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._frames_ready: int = 0
        self._complete = False
        self._thread: Optional[threading.Thread] = None
        self._mmap: Optional[np.memmap] = None

        self._video_mtime = mtime
        self._video_size = size

        # Decide: open existing, or create fresh.
        if self._sidecar_matches():
            self._open_existing()
        else:
            self._create_and_spawn()

    # ----------------------------------------------------------- public API

    @property
    def n_frames(self) -> int:
        return self._n_frames

    @property
    def frame_shape(self) -> tuple[int, int]:
        """(h, w) of the cached frames (= target_h, target_w)."""
        return (self._target_h, self._target_w)

    @property
    def source_shape(self) -> tuple[int, int]:
        """(h, w) of the original source video — useful for callers."""
        return (self._src_h, self._src_w)

    @property
    def letterbox_offsets(self) -> tuple[int, int]:
        """(top, left) pixel offset of the image area inside the cached frame."""
        return (self._top, self._left)

    @property
    def image_area(self) -> tuple[int, int, int, int]:
        """(top, left, resized_h, resized_w) of the image inside the cache."""
        return (self._top, self._left, self._resized_h, self._resized_w)

    @property
    def scale(self) -> float:
        """Source-px to cache-px scale factor (preserving aspect ratio)."""
        return self._scale

    @property
    def is_complete(self) -> bool:
        return self._complete

    @property
    def progress(self) -> tuple[int, int]:
        return (self._frames_ready, self._n_frames)

    def get(self, frame_n: int) -> Optional[np.ndarray]:
        """Return a VIEW of the cached RGB frame, or None if not yet ready.

        Callers MUST NOT mutate the returned array — it aliases the mmap.
        """
        if self._mmap is None:
            return None
        if frame_n < 0 or frame_n >= self._n_frames:
            return None
        # int reads/writes are atomic in CPython, no lock needed.
        if frame_n >= self._frames_ready:
            return None
        return self._mmap[frame_n]

    def close(self) -> None:
        """Stop populator thread, flush mmap, release."""
        self._cancel.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=5.0)
        # Flush + drop the mmap reference. numpy.memmap doesn't expose an
        # explicit close, but deleting all references + a flush is enough.
        m = self._mmap
        if m is not None:
            try:
                m.flush()
            except Exception:
                pass
        self._mmap = None

    # ----------------------------------------------------------- internals

    def _sidecar_matches(self) -> bool:
        """Return True iff an existing sidecar describes a still-valid cache."""
        if not self._json_path.is_file() or not self._bin_path.is_file():
            return False
        try:
            meta = json.loads(self._json_path.read_text())
        except (OSError, json.JSONDecodeError):
            return False
        try:
            return (
                os.path.abspath(meta["video_path"]) ==
                os.path.abspath(self._video_path)
                and float(meta["video_mtime"]) == self._video_mtime
                and int(meta["video_size"]) == self._video_size
                and int(meta["n_frames"]) == self._n_frames
                and int(meta["frame_w"]) == self._target_w
                and int(meta["frame_h"]) == self._target_h
                and meta.get("dtype", "uint8") == "uint8"
            )
        except (KeyError, TypeError, ValueError):
            return False

    def _open_existing(self) -> None:
        """Memmap the existing cache file read-only and mark complete."""
        expected_size = (self._n_frames * self._target_h
                         * self._target_w * 3)
        actual_size = self._bin_path.stat().st_size
        if actual_size != expected_size:
            # Size mismatch — treat as stale, recreate.
            self._bin_path.unlink(missing_ok=True)
            self._json_path.unlink(missing_ok=True)
            self._create_and_spawn()
            return
        self._mmap = np.memmap(
            self._bin_path, dtype=np.uint8, mode="r",
            shape=(self._n_frames, self._target_h, self._target_w, 3),
        )
        self._frames_ready = self._n_frames
        self._complete = True

    def _write_sidecar(self) -> None:
        meta = {
            "video_path": os.path.abspath(self._video_path),
            "video_mtime": self._video_mtime,
            "video_size": self._video_size,
            "n_frames": self._n_frames,
            "frame_w": self._target_w,
            "frame_h": self._target_h,
            "dtype": "uint8",
            "letterbox_offsets": [self._top, self._left],
            "resized_w": self._resized_w,
            "resized_h": self._resized_h,
            "scale": self._scale,
            "source_w": self._src_w,
            "source_h": self._src_h,
        }
        self._json_path.write_text(json.dumps(meta, indent=2))

    def _create_and_spawn(self) -> None:
        """Allocate a fresh memmap (w+ creates / truncates) and start
        populating in a background thread."""
        # Numpy memmap w+ creates the file zero-filled at the right size.
        self._mmap = np.memmap(
            self._bin_path, dtype=np.uint8, mode="w+",
            shape=(self._n_frames, self._target_h, self._target_w, 3),
        )
        self._write_sidecar()
        self._thread = threading.Thread(
            target=self._populate,
            name="PreviewCache.populator",
            daemon=True,
        )
        self._thread.start()

    def _populate(self) -> None:
        """Sequentially decode the source video and write resized RGB frames."""
        cap = cv2.VideoCapture(self._video_path)
        if not cap.isOpened():
            return
        try:
            mm = self._mmap
            if mm is None:
                return
            top, left = self._top, self._left
            rw, rh = self._resized_w, self._resized_h
            n = self._n_frames
            i = 0
            while i < n:
                if (i % 10) == 0 and self._cancel.is_set():
                    break
                ok, frame = cap.read()
                if not ok:
                    # End-of-stream or decode error — leave remaining frames
                    # as zeros, but stop trying.
                    break
                # Resize then BGR->RGB. INTER_AREA is the right choice for
                # downscaling.
                resized = cv2.resize(frame, (rw, rh),
                                     interpolation=cv2.INTER_AREA)
                rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                mm[i, top:top + rh, left:left + rw, :] = rgb
                i += 1
                # Publish the new high-water mark (atomic int assignment).
                self._frames_ready = i
                if self._on_progress is not None and (i % 30) == 0:
                    try:
                        self._on_progress(i, n)
                    except Exception:
                        pass
            # Final flush + progress notification.
            try:
                mm.flush()
            except Exception:
                pass
            self._complete = (i == n)
            if self._on_progress is not None:
                try:
                    self._on_progress(i, n)
                except Exception:
                    pass
        finally:
            cap.release()
