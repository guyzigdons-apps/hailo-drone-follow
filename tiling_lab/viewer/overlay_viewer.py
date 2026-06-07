"""Interactive detection-overlay viewer for pxt sweep frames.json files.

A tkinter-based viewer that loads a source video and one or more
``frames.json`` files, then renders the detections on top of the video
with zoom/pan, scrubbable timeline, reverse playback, per-run visibility
toggles, and a HUD that reports the source-pixel under the cursor.

Phase 1 features:
  * Sidebar with per-run visibility checkbutton + colour swatch.
  * Frame scrubber (Scale 0..total-1).
  * Discrete speed selector: -4x, -2x, -1x, pause, 1x, 2x, 4x.
  * Transport controls: first / step-back / play-pause / step-forward / last.
  * Mouse-wheel zoom around cursor (clamped [0.5, 16.0]).
  * Drag-pan with left mouse button.
  * Double-click (or 'f') to reset zoom + pan.
  * HUD drawn on the rescaled canvas (frame #, zoom, src px under cursor,
    visible run legend).
  * Keyboard shortcuts: q/Esc=quit, space=play/pause, [ ]=step,
    Home/End=jump to ends, f=reset view.

Phase 2 features:
  * Tile-rectangle overlay (toggleable) with per-run tile source.
  * Live confidence-floor slider that filters bboxes as you drag.

Usage:
  python -m tiling_lab.viewer.overlay_viewer \\
      --video /path/to/video.MP4 \\
      --frames /path/to/pxt_GT-12x9-25.frames.json:GT \\
      --frames /path/to/pxt_3x2-native.frames.json:3x2
"""

from __future__ import annotations

import argparse
import json
import time
import tkinter as tk
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageTk

# Viewer-support helpers (PreviewCache / phantom + containment merge) still live
# in the frozen ``tiling_benchmark`` package — they are shared with the legacy
# pxt/upscale runners that were NOT moved by the tiling_lab restructure. Import
# them lazily so module import (and ``--help``) never reaches into
# tiling_benchmark; only an actual viewer session needs them.
def _load_preview_cache():
    from tiling_benchmark.preview_cache import PreviewCache  # noqa: E402
    return PreviewCache


def _load_analyze():
    from tiling_benchmark.analyze_pxt import containment_merge, is_phantom  # noqa: E402
    return containment_merge, is_phantom

# Distinct BGR colours — matches bench/overlay_dets.py palette order.
PALETTE = [
    (0, 255, 0),    # green
    (0, 0, 255),    # red
    (255, 0, 0),    # blue
    (0, 255, 255),  # yellow
    (255, 0, 255),  # magenta
    (255, 255, 0),  # cyan
    (255, 128, 0),  # orange
    (128, 0, 255),  # purple
]

# Discrete playback speeds for the speed selector.
SPEEDS = [-4.0, -2.0, -1.0, 0.0, 1.0, 2.0, 4.0]
SPEED_LABELS = ["-4x", "-2x", "-1x", "Pause", "1x", "2x", "4x"]

ZOOM_MIN = 0.5
ZOOM_MAX = 16.0
ZOOM_STEP = 1.25  # multiplicative step per wheel tick

SIDEBAR_WIDTH = 270  # includes the ~16px vertical scrollbar


def bgr_to_hex(bgr: tuple[int, int, int]) -> str:
    b, g, r = bgr
    return f"#{r:02x}{g:02x}{b:02x}"


def load_frames_indexed(
    path: Path,
) -> tuple[str, dict[int, list[dict]], dict, dict[int, list[tuple]]]:
    with path.open() as f:
        doc = json.load(f)
    idx = {int(fr["frame"]): fr["detections"] for fr in doc["frames"]}
    tiles_by_frame: dict[int, list[tuple]] = {}
    for fr in doc["frames"]:
        tlist = fr.get("tiles") or []
        if tlist:
            tiles_by_frame[int(fr["frame"])] = [
                (float(t["x"]), float(t["y"]), float(t["w"]), float(t["h"]),
                 str(t.get("category", "dynamic")))
                for t in tlist
            ]
    config = doc.get("config") or {}
    return doc.get("label") or path.stem, idx, config, tiles_by_frame


def grid_tiles(tiles_x: int, tiles_y: int,
               ox: float, oy: float) -> list[tuple[float, float, float, float]]:
    """Return list of (x, y, w, h) in normalized [0,1] frame coords.

    Mirrors bench/tiling_record.py:_grid_to_static_tiles. For N==1 along an
    axis, the single tile spans the full axis.
    """
    if tiles_x < 1 or tiles_y < 1:
        return []

    def axis(n: int, o: float) -> list[tuple[float, float]]:
        if n == 1:
            return [(0.0, 1.0)]
        T = 1.0 / (n - (n - 1) * o)
        S = T * (1.0 - o)
        return [(i * S, T) for i in range(n)]

    rects: list[tuple[float, float, float, float]] = []
    for (y, h) in axis(tiles_y, oy):
        for (x, w) in axis(tiles_x, ox):
            rects.append((x, y, w, h))
    return rects


def tile_rects_from_config(
    cfg: dict,
) -> list[tuple[float, float, float, float, str]]:
    """Reconstruct the full tile list (grid + extras) from a frames.json
    ``config`` block. Output is normalized [0,1] frame coordinates with a
    category label per rect:

      * ``'multi-scale'`` — the main dense grid; the aggregator's
        boundary-strip filter (remove_exceeded_bboxes) runs on these tiles.
        Tagged ',m' in tiles-static. Fragment-producing layer.
      * ``'single-scale'`` — coarser rescue tiles (full-frame, center-tile,
        extra_grids, extra_rects). The aggregator preserves detections in
        these. Tagged ',s' in tiles-static.
      * ``'dynamic'`` — per-frame HailoTileROI attached upstream (e.g. a
        future tracker-tile). Reserved category — no run-side config
        reconstruction possible because the geometry varies per frame.

    The category is purely a visualization aid; downstream code (e.g.
    ``is_phantom``) only needs the (x, y, w, h) shape and ignores the 5th
    field.
    """
    if not cfg:
        return []
    out: list[tuple[float, float, float, float, str]] = []
    tiles_x = int(cfg.get("tiles_x", 0) or 0)
    tiles_y = int(cfg.get("tiles_y", 0) or 0)
    ox = float(cfg.get("overlap_x", 0.0) or 0.0)
    oy = float(cfg.get("overlap_y", 0.0) or 0.0)
    for (x, y, w, h) in grid_tiles(tiles_x, tiles_y, ox, oy):
        out.append((x, y, w, h, "multi-scale"))
    if cfg.get("include_full_frame"):
        out.append((0.0, 0.0, 1.0, 1.0, "single-scale"))
    if cfg.get("include_center_tile"):
        s = float(cfg.get("center_tile_size", 0.0) or 0.0)
        if s > 0.0:
            s = min(s, 1.0)
            off = (1.0 - s) / 2.0
            out.append((off, off, s, s, "single-scale"))
    for entry in (cfg.get("extra_grids") or []):
        tx, ty, gox, goy = (int(entry[0]), int(entry[1]),
                            float(entry[2]), float(entry[3]))
        for (x, y, w, h) in grid_tiles(tx, ty, gox, goy):
            out.append((x, y, w, h, "single-scale"))
    for entry in (cfg.get("extra_rects") or []):
        rx, ry, rw, rh = (float(entry[0]), float(entry[1]),
                          float(entry[2]), float(entry[3]))
        out.append((rx, ry, rw, rh, "single-scale"))
    return out


def parse_frames_arg(arg: str) -> tuple[Path, str | None]:
    if ":" in arg:
        p, lbl = arg.rsplit(":", 1)
        return Path(p), lbl
    return Path(arg), None


class Run:
    """One frames.json overlay source."""

    def __init__(self, label: str, idx: dict[int, list[dict]],
                 colour_bgr: tuple[int, int, int],
                 config: dict | None = None,
                 tiles_by_frame: dict[int, list[tuple]] | None = None):
        self.label = label
        self.idx = idx
        self.colour_bgr = colour_bgr
        self.config: dict = config or {}
        # Typed list with category in the 5th field (for colour-coding).
        self.tile_rects_typed: list[
            tuple[float, float, float, float, str]
        ] = tile_rects_from_config(self.config)
        # 4-tuple version for downstream consumers (is_phantom etc.) that
        # don't care about category.
        self.tile_rects: list[tuple[float, float, float, float]] = [
            (x, y, w, h) for (x, y, w, h, _cat) in self.tile_rects_typed
        ]
        # Per-frame dynamic tile rects from frames.json (overrides tile_rects_typed
        # during rendering when present for the current frame).
        self.tiles_by_frame: dict[int, list[tuple]] = tiles_by_frame or {}
        self.visible_var: tk.BooleanVar | None = None  # set after Tk root exists
        # Containment-merge per-frame cache: key = (frame_no, area_ratio_max,
        # center_slack, hide_phantoms). Value = (kept_dets_list, n_suppressed).
        # We key on the filter params so toggling phantoms or tuning the ratio
        # doesn't surface stale results, but identical-params re-renders (zoom,
        # pan) are O(1).
        self._cm_cache: dict[tuple, tuple[list[dict], int]] = {}


FULL_RES_LRU_CAP = 8  # Phase 3: small full-res frame cache for zoomed-in views.


class OverlayViewer:
    def __init__(self, root: tk.Tk, video_path: Path, runs: list[Run],
                 start_frame: int = 0, conf_min: float = 0.25,
                 use_cache: bool = True,
                 cache_target_w: int = 1920,
                 cache_target_h: int = 1080):
        self.root = root
        self.video_path = video_path
        self.runs = runs
        self.conf_min = conf_min

        self.cap = cv2.VideoCapture(str(video_path))
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open video {video_path}")
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 29.97
        self.src_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.src_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # State
        self.frame_no = max(0, min(start_frame, self.total_frames - 1))
        self.speed = 0.0  # paused initially
        self._after_id: str | None = None
        self._last_work_ms = 0.0  # decode+render cost of the last frame
        self._suppress_scrubber_cb = False

        # View transform — pan is the source-px coord of the centre of the viewport.
        self.zoom_factor = 1.0
        self.pan_x = self.src_w / 2.0
        self.pan_y = self.src_h / 2.0

        # Mouse state
        self._drag_last: tuple[int, int] | None = None
        self._cursor_canvas: tuple[int, int] | None = None  # last (x,y) of <Motion>

        # Current decoded RGB frame (numpy) + source tag for HUD.
        self._current_frame_rgb: np.ndarray | None = None
        self._current_frame_source: str = "full"
        # Full-res RGB frame LRU (used when zoomed past cache resolution OR
        # when the cache hasn't reached this frame yet). Keys are frame_no.
        self._full_res_lru: OrderedDict[int, np.ndarray] = OrderedDict()
        # Keep a reference to the last ImageTk to prevent GC
        self._photo_ref: ImageTk.PhotoImage | None = None

        # ---- Phase 3: preview cache ----
        self.cache: PreviewCache | None = None
        self._cache_target = (cache_target_w, cache_target_h)
        if use_cache:
            try:
                PreviewCache = _load_preview_cache()
                cache_dir = (Path(__file__).resolve().parent
                             / "pxt_runs" / ".cache")
                self.cache = PreviewCache(
                    str(video_path), cache_dir,
                    target_w=cache_target_w, target_h=cache_target_h,
                    on_progress=self._on_cache_progress,
                )
            except Exception as e:
                # Cache is best-effort — log and continue with the slow path.
                print(f"[overlay_viewer] preview cache disabled: {e}")
                self.cache = None

        # Create per-run Tk visibility BooleanVars now that root exists.
        # Trace each so the tile-source radio can react when its currently
        # selected run gets hidden.
        for r in self.runs:
            r.visible_var = tk.BooleanVar(value=True)
            r.visible_var.trace_add("write", self._on_visibility_changed)

        # Phase 2 state: live confidence floor + tile overlay controls.
        self._conf_min_var = tk.DoubleVar(value=self.conf_min)
        self._show_tiles_var = tk.BooleanVar(value=False)
        # Phantom filter: default ON. Hides class-0 person detections whose
        # bbox is shaped/sized like a parent tile (yolov8n_4_classes_vga
        # low-contrast-tile artefact; see analyze_pxt.is_phantom).
        self._hide_phantoms_var = tk.BooleanVar(value=True)
        # Per-run phantom count for the current frame (rebuilt each render
        # when hiding is ON; used for the HUD legend).
        self._phantoms_hidden_per_run: dict[str, int] = {}
        # Containment-merge filter: default ON. Suppresses same-class dets
        # whose centre lies inside a larger det and whose area is below
        # `area_ratio_max * big.area`. See analyze_pxt.containment_merge and
        # PERF_REPORT sec 7.1.
        self._containment_merge_var = tk.BooleanVar(value=True)
        # Per-run merged count for the current frame (used for HUD legend).
        self._merged_per_run: dict[str, int] = {}
        # Phase 4 state: sidebar zoom slider (two-way synced with wheel) +
        # opt-in native-res zoom (full-res cv2 decode past cache density).
        self._zoom_var = tk.DoubleVar(value=1.0)
        self._suppress_zoom_callback = False
        self._native_zoom_var = tk.BooleanVar(value=False)
        # Tile-source radio holds the label of the currently selected run.
        # Default to the first run (all runs start visible).
        self._tile_source_var = tk.StringVar(
            value=self.runs[0].label if self.runs else ""
        )
        # Container for tile-source radiobutton widgets so we can rebuild it
        # when visibility changes (only visible runs are listed).
        self._tile_source_menu: tk.OptionMenu | None = None

        self._build_ui()
        self._bind_events()

        # Initial seek + render
        self._seek_to(self.frame_no)
        # Defer first render until canvas has a real size (after geometry pass).
        self.root.after(50, self._render)

    # ------------------------------------------------------------------ UI

    def _build_ui(self) -> None:
        self.root.title("pxt overlay viewer")
        self.root.geometry("1400x900")

        # Main horizontal layout: canvas (expand) | sidebar (fixed)
        main = tk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True)

        # --- Canvas ---
        self.canvas = tk.Canvas(main, bg="black", highlightthickness=0,
                                cursor="crosshair")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # --- Sidebar (scrollable) ---
        # Fixed-width container holding a canvas + vertical scrollbar, so that
        # with many runs the lower sections (Video / Keys) can be scrolled to
        # instead of being pushed off the bottom of the screen.
        sidebar_container = tk.Frame(main, width=SIDEBAR_WIDTH, bg="#202020")
        sidebar_container.pack(side=tk.RIGHT, fill=tk.Y)
        sidebar_container.pack_propagate(False)

        sidebar_canvas = tk.Canvas(sidebar_container, bg="#202020",
                                   highlightthickness=0)
        sb_scroll = tk.Scrollbar(sidebar_container, orient=tk.VERTICAL,
                                 command=sidebar_canvas.yview)
        sidebar_canvas.configure(yscrollcommand=sb_scroll.set)
        sb_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        sidebar_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Inner frame that actually holds the controls. Keeping the variable
        # name ``sidebar`` means every downstream ``tk.LabelFrame(sidebar, …)``
        # call below is unchanged.
        sidebar = tk.Frame(sidebar_canvas, bg="#202020")
        sidebar_window = sidebar_canvas.create_window((0, 0), window=sidebar,
                                                      anchor="nw")

        def _on_sidebar_configure(_event=None):
            sidebar_canvas.configure(scrollregion=sidebar_canvas.bbox("all"))
        sidebar.bind("<Configure>", _on_sidebar_configure)

        def _on_sidebar_canvas_configure(event):
            # Make the inner frame track the canvas width so content fills it.
            sidebar_canvas.itemconfigure(sidebar_window, width=event.width)
        sidebar_canvas.bind("<Configure>", _on_sidebar_canvas_configure)

        # Mouse-wheel scrolling for the sidebar. Bound globally but gated on the
        # pointer being over the sidebar subtree, so the image canvas keeps its
        # own widget-level wheel-zoom binding when the pointer is over it.
        def _sidebar_wheel(event):
            w = self.root.winfo_containing(event.x_root, event.y_root)
            while w is not None:
                if w is sidebar_container:
                    if getattr(event, "num", None) == 4:
                        sidebar_canvas.yview_scroll(-1, "units")
                    elif getattr(event, "num", None) == 5:
                        sidebar_canvas.yview_scroll(1, "units")
                    else:
                        sidebar_canvas.yview_scroll(
                            -1 if event.delta > 0 else 1, "units")
                    return
                w = getattr(w, "master", None)
        sidebar_canvas.bind_all("<MouseWheel>", _sidebar_wheel, add="+")
        sidebar_canvas.bind_all("<Button-4>", _sidebar_wheel, add="+")
        sidebar_canvas.bind_all("<Button-5>", _sidebar_wheel, add="+")

        # Runs section
        runs_lf = tk.LabelFrame(sidebar, text="Runs", fg="white", bg="#202020",
                                labelanchor="nw", padx=6, pady=4)
        runs_lf.pack(fill=tk.X, padx=6, pady=(8, 4))
        # Single-column list of run-visibility toggles (swatch + checkbox).
        # One row per run so long labels never clip past the fixed-width
        # sidebar (a two-column grid overflowed the right edge).
        # Re-render is driven by the trace_add on each r.visible_var, which
        # also refreshes the tile-source dropdown.
        for i, r in enumerate(self.runs):
            cell = tk.Frame(runs_lf, bg="#202020")
            cell.grid(row=i, column=0, sticky="w", pady=1)
            swatch = tk.Label(cell, text="  ", bg=bgr_to_hex(r.colour_bgr),
                              width=2, relief=tk.RAISED, bd=1)
            swatch.pack(side=tk.LEFT, padx=(0, 4))
            cb = tk.Checkbutton(cell, text=r.label, variable=r.visible_var,
                                fg="white", bg="#202020",
                                activebackground="#202020",
                                activeforeground="white",
                                selectcolor="#404040")
            cb.pack(side=tk.LEFT, anchor="w")

        # Tiles overlay
        tiles_lf = tk.LabelFrame(sidebar, text="Overlays", fg="white",
                                 bg="#202020", labelanchor="nw",
                                 padx=6, pady=4)
        tiles_lf.pack(fill=tk.X, padx=6, pady=4)
        tk.Checkbutton(tiles_lf, text="Show tiles",
                       variable=self._show_tiles_var,
                       fg="white", bg="#202020",
                       activebackground="#202020",
                       activeforeground="white",
                       selectcolor="#404040",
                       command=self._render).pack(anchor="w")
        # Tile-source dropdown (only one run's tiles drawn at a time).
        ts_row = tk.Frame(tiles_lf, bg="#202020")
        ts_row.pack(fill=tk.X, padx=(16, 0), pady=(2, 2))
        tk.Label(ts_row, text="from:", fg="white", bg="#202020").pack(side=tk.LEFT)
        self._tile_source_menu = tk.OptionMenu(ts_row, self._tile_source_var, "")
        self._tile_source_menu.config(bg="#303030", fg="white",
                                      activebackground="#404040",
                                      activeforeground="white",
                                      highlightthickness=0, bd=1,
                                      anchor="w")
        self._tile_source_menu["menu"].config(bg="#303030", fg="white",
                                              activebackground="#404040",
                                              activeforeground="white")
        self._tile_source_menu.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self._rebuild_tile_source_radios()
        # Phase 4: opt-in native-res zoom. When OFF (default), zooming past
        # cache resolution stays on the cache with INTER_NEAREST upscale (fast
        # but pixelated). When ON, restores the Phase-3 auto-fallback to a
        # full-res cv2 decode for pixel-accurate inspection (slow).
        tk.Checkbutton(tiles_lf, text="Native-res when zoomed",
                       variable=self._native_zoom_var,
                       fg="white", bg="#202020",
                       activebackground="#202020",
                       activeforeground="white",
                       selectcolor="#404040",
                       command=self._render).pack(anchor="w")
        # Phantom filter: hides class-0 person detections matching the
        # tile-shape / large-person fallback heuristic (see
        # analyze_pxt.is_phantom).
        tk.Checkbutton(tiles_lf, text="Hide phantoms",
                       variable=self._hide_phantoms_var,
                       fg="white", bg="#202020",
                       activebackground="#202020",
                       activeforeground="white",
                       selectcolor="#404040",
                       command=self._render).pack(anchor="w")
        # Containment-merge: suppresses small same-class dets contained
        # inside a larger same-class det (post-NMS NMS-fragment fix; see
        # analyze_pxt.containment_merge).
        tk.Checkbutton(tiles_lf, text="Containment-merge",
                       variable=self._containment_merge_var,
                       fg="white", bg="#202020",
                       activebackground="#202020",
                       activeforeground="white",
                       selectcolor="#404040",
                       command=self._render).pack(anchor="w")

        # Conf-min slider (live).
        conf_lf = tk.LabelFrame(sidebar, text="Confidence",
                                fg="white", bg="#202020", labelanchor="nw",
                                padx=6, pady=4)
        conf_lf.pack(fill=tk.X, padx=6, pady=4)
        self._conf_slider = tk.Scale(conf_lf, from_=0.0, to=1.0,
                                     resolution=0.01,
                                     orient=tk.HORIZONTAL,
                                     variable=self._conf_min_var,
                                     bg="#202020", fg="white",
                                     troughcolor="#404040",
                                     highlightthickness=0,
                                     label="Min confidence",
                                     command=self._on_conf_changed)
        self._conf_slider.pack(fill=tk.X)

        # Frame scrubber
        scrub_lf = tk.LabelFrame(sidebar, text="Frame", fg="white",
                                 bg="#202020", labelanchor="nw",
                                 padx=6, pady=4)
        scrub_lf.pack(fill=tk.X, padx=6, pady=4)
        self._scrubber = tk.Scale(scrub_lf, from_=0, to=max(0, self.total_frames - 1),
                                  orient=tk.HORIZONTAL,
                                  bg="#202020", fg="white",
                                  troughcolor="#404040",
                                  highlightthickness=0,
                                  command=self._on_scrubber)
        self._scrubber.set(self.frame_no)
        self._scrubber.pack(fill=tk.X)

        # Speed selector
        speed_lf = tk.LabelFrame(sidebar, text="Speed", fg="white",
                                 bg="#202020", labelanchor="nw",
                                 padx=6, pady=4)
        speed_lf.pack(fill=tk.X, padx=6, pady=4)
        self._speed_var = tk.DoubleVar(value=0.0)
        speed_row = tk.Frame(speed_lf, bg="#202020")
        speed_row.pack(fill=tk.X)
        for spd, lbl in zip(SPEEDS, SPEED_LABELS):
            rb = tk.Radiobutton(speed_row, text=lbl, variable=self._speed_var,
                                value=spd,
                                indicatoron=False,
                                width=4,
                                fg="white", bg="#303030",
                                activebackground="#505050",
                                activeforeground="white",
                                selectcolor="#0066cc",
                                command=self._on_speed_change)
            rb.pack(side=tk.LEFT, padx=1)

        # Zoom slider (two-way synced with mouse-wheel zoom).
        zoom_lf = tk.LabelFrame(sidebar, text="Zoom", fg="white",
                                bg="#202020", labelanchor="nw",
                                padx=6, pady=4)
        zoom_lf.pack(fill=tk.X, padx=6, pady=4)
        self._zoom_scale = tk.Scale(
            zoom_lf, from_=ZOOM_MIN, to=ZOOM_MAX, resolution=0.1,
            orient=tk.HORIZONTAL,
            variable=self._zoom_var,
            bg="#202020", fg="white",
            troughcolor="#404040",
            highlightthickness=0,
            label="Zoom",
            command=self._on_zoom_slider_changed,
        )
        self._zoom_scale.pack(fill=tk.X)

        # Transport
        tr_lf = tk.LabelFrame(sidebar, text="Transport", fg="white",
                              bg="#202020", labelanchor="nw",
                              padx=6, pady=4)
        tr_lf.pack(fill=tk.X, padx=6, pady=4)
        tr_row = tk.Frame(tr_lf, bg="#202020")
        tr_row.pack(fill=tk.X)
        btn_kwargs = dict(bg="#303030", fg="white",
                          activebackground="#505050",
                          activeforeground="white",
                          relief=tk.RAISED, width=3)
        tk.Button(tr_row, text="|<", command=self._goto_start,
                  **btn_kwargs).pack(side=tk.LEFT, padx=1)
        tk.Button(tr_row, text="<", command=self._step_back,
                  **btn_kwargs).pack(side=tk.LEFT, padx=1)
        self._play_btn = tk.Button(tr_row, text="Play",
                                   command=self._toggle_play,
                                   **btn_kwargs)
        self._play_btn.pack(side=tk.LEFT, padx=1)
        tk.Button(tr_row, text=">", command=self._step_fwd,
                  **btn_kwargs).pack(side=tk.LEFT, padx=1)
        tk.Button(tr_row, text=">|", command=self._goto_end,
                  **btn_kwargs).pack(side=tk.LEFT, padx=1)

        # Info section
        info_lf = tk.LabelFrame(sidebar, text="Video", fg="white",
                                bg="#202020", labelanchor="nw",
                                padx=6, pady=4)
        info_lf.pack(fill=tk.X, padx=6, pady=4)
        info_text = (f"{self.src_w}x{self.src_h}\n"
                     f"{self.total_frames} frames @ {self.fps:.2f} fps")
        tk.Label(info_lf, text=info_text, fg="white", bg="#202020",
                 justify=tk.LEFT, anchor="w").pack(fill=tk.X)

        # Help section
        help_lf = tk.LabelFrame(sidebar, text="Keys", fg="white",
                                bg="#202020", labelanchor="nw",
                                padx=6, pady=4)
        help_lf.pack(fill=tk.X, padx=6, pady=4)
        help_txt = ("q/Esc: quit\n"
                    "space: play/pause\n"
                    "[ / ]: step -1/+1\n"
                    "Home/End: ends\n"
                    "f or dbl-click: reset view\n"
                    "wheel: zoom\n"
                    "drag: pan")
        tk.Label(help_lf, text=help_txt, fg="white", bg="#202020",
                 justify=tk.LEFT, anchor="w").pack(fill=tk.X)

    def _rebuild_tile_source_radios(self) -> None:
        """Repopulate the tile-source dropdown with the visible runs.

        Called on init and whenever a run's visibility checkbox toggles. If
        the currently selected tile source has just been hidden, fall back
        to the first remaining visible run. If no runs are visible the
        dropdown is empty (tile overlay will be skipped at draw time).
        """
        menu_widget = self._tile_source_menu
        if menu_widget is None:
            return

        visible_runs = [r for r in self.runs
                        if r.visible_var is not None and r.visible_var.get()]
        visible_labels = [r.label for r in visible_runs]
        if self._tile_source_var.get() not in visible_labels:
            self._tile_source_var.set(visible_labels[0] if visible_labels else "")

        menu = menu_widget["menu"]
        menu.delete(0, "end")
        for lbl in visible_labels:
            menu.add_command(
                label=lbl,
                command=lambda v=lbl: (self._tile_source_var.set(v),
                                       self._render()))

    def _on_visibility_changed(self, *_args) -> None:
        # Triggered by Tk trace_add on each Run.visible_var.
        self._rebuild_tile_source_radios()
        self._render()

    def _on_conf_changed(self, _val: str) -> None:
        # Slider drag callback. The DoubleVar already holds the new value.
        self._render()

    def _on_zoom_slider_changed(self, _val: str) -> None:
        # If the wheel handler (or reset) just programmatically set
        # _zoom_var to keep the slider in sync, swallow the resulting
        # callback to avoid an infinite render loop.
        if self._suppress_zoom_callback:
            return
        new_zoom = float(self._zoom_var.get())
        new_zoom = max(ZOOM_MIN, min(ZOOM_MAX, new_zoom))
        if new_zoom == self.zoom_factor:
            return
        # Slider has no cursor anchor — zoom around the current viewport
        # centre (i.e. keep pan_x/pan_y as-is). The clamping in
        # _viewport_src_rect will re-centre if the new viewport overruns
        # the source extents.
        self.zoom_factor = new_zoom
        self._render()

    def _sync_zoom_slider(self) -> None:
        """Push self.zoom_factor into the slider DoubleVar without firing
        the slider's command callback (avoids infinite recursion)."""
        self._suppress_zoom_callback = True
        try:
            self._zoom_var.set(self.zoom_factor)
        finally:
            self._suppress_zoom_callback = False

    def _bind_events(self) -> None:
        # Canvas mouse
        self.canvas.bind("<MouseWheel>", self._on_wheel)        # Win/macOS
        self.canvas.bind("<Button-4>", self._on_wheel)          # X11 up
        self.canvas.bind("<Button-5>", self._on_wheel)          # X11 down
        self.canvas.bind("<ButtonPress-1>", self._on_drag_start)
        self.canvas.bind("<B1-Motion>", self._on_drag_motion)
        self.canvas.bind("<ButtonRelease-1>", self._on_drag_end)
        self.canvas.bind("<Double-Button-1>", lambda e: self._reset_view())
        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<Leave>", self._on_leave)
        self.canvas.bind("<Configure>", lambda e: self._render())

        # Root keyboard
        self.root.bind("<Escape>", lambda e: self._quit())
        self.root.bind("q", lambda e: self._quit())
        self.root.bind("<space>", lambda e: self._toggle_play())
        self.root.bind("[", lambda e: self._step_back())
        self.root.bind("]", lambda e: self._step_fwd())
        self.root.bind("<Home>", lambda e: self._goto_start())
        self.root.bind("<End>", lambda e: self._goto_end())
        self.root.bind("f", lambda e: self._reset_view())
        self.root.protocol("WM_DELETE_WINDOW", self._quit)

    # -------------------------------------------------- canvas/view math

    def _canvas_size(self) -> tuple[int, int]:
        cw = max(1, self.canvas.winfo_width())
        ch = max(1, self.canvas.winfo_height())
        return cw, ch

    def _effective_scale(self) -> float:
        """Pixels-on-canvas per source-pixel.

        At zoom_factor=1.0 the entire source frame fits the canvas
        (letterboxed on the smaller-ratio axis). At zoom_factor>1, we see
        a proportionally smaller piece of source at higher density.
        """
        cw, ch = self._canvas_size()
        fit_scale = min(cw / self.src_w, ch / self.src_h)
        return fit_scale * self.zoom_factor

    def _viewport_src_rect(self) -> tuple[float, float, float, float]:
        """Return (x, y, w, h) in source-pixel coords currently visible.

        The viewport is centred on (pan_x, pan_y) in source coords and
        spans canvas_w/scale by canvas_h/scale source pixels.
        """
        cw, ch = self._canvas_size()
        scale = self._effective_scale()
        vw = cw / scale
        vh = ch / scale
        # Clamp pan so viewport stays inside [0, src_w] x [0, src_h] when
        # the viewport is smaller than the source. When viewport is
        # larger, just centre it.
        if vw >= self.src_w:
            cx = self.src_w / 2.0
        else:
            half = vw / 2.0
            cx = min(max(self.pan_x, half), self.src_w - half)
            self.pan_x = cx
        if vh >= self.src_h:
            cy = self.src_h / 2.0
        else:
            half = vh / 2.0
            cy = min(max(self.pan_y, half), self.src_h - half)
            self.pan_y = cy
        return cx - vw / 2.0, cy - vh / 2.0, vw, vh

    def _canvas_to_src(self, cx: float, cy: float) -> tuple[float, float]:
        vx, vy, vw, vh = self._viewport_src_rect()
        cw, ch = self._canvas_size()
        sx = vx + (cx / cw) * vw
        sy = vy + (cy / ch) * vh
        return sx, sy

    # -------------------------------------------------- frame I/O

    def _viewport_src_width(self) -> float:
        """Source-pixel width currently visible in the viewport."""
        cw, _ch = self._canvas_size()
        scale = self._effective_scale()
        if scale <= 0:
            return float(self.src_w)
        return cw / scale

    def _cache_is_dense_enough(self) -> bool:
        """True when cached preview pixels are at least as dense (per
        source-pixel) as what we'd render to canvas — i.e. the cache has not
        been zoomed past its native resolution.

        Threshold (per Phase-3 spec): cache wins when
            canvas_w / effective_src_w_visible <= cache_w / source_w
        i.e. the per-canvas-pixel density of source-pixels we'd be sampling
        does not exceed what the cache stores per source-pixel.
        """
        if self.cache is None:
            return False
        cw, _ch = self._canvas_size()
        viewport_src_w = self._viewport_src_width()
        if viewport_src_w <= 0:
            return True
        canvas_density = cw / viewport_src_w           # canvas-px / src-px
        cache_density = self.cache.frame_shape[1] / self.src_w
        return canvas_density <= cache_density

    def _should_use_cache(self) -> bool:
        """Whether to read this frame from the preview cache.

        Default (native-res checkbox OFF): always use the cache when one is
        available — even past its native resolution. This costs an
        INTER_NEAREST upscale (pixelated but fast) and avoids the ~60 ms
        HEVC software decode that the full-res path requires per frame.

        Native-res checkbox ON: fall back to the Phase-3 auto-detection —
        cache when its density covers the canvas, full-res otherwise.
        """
        if self.cache is None:
            return False
        if not self._native_zoom_var.get():
            return True
        return self._cache_is_dense_enough()

    def _full_res_get(self, n: int) -> np.ndarray:
        """Return a full-resolution RGB frame for frame n, using a small LRU
        backed by direct cv2 reads. Always returns a frame (raises on read
        failure)."""
        if n in self._full_res_lru:
            # Promote to most-recently-used.
            self._full_res_lru.move_to_end(n)
            return self._full_res_lru[n]
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, n)
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError(f"failed to read frame {n} from {self.video_path}")
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._full_res_lru[n] = rgb
        # Evict oldest.
        while len(self._full_res_lru) > FULL_RES_LRU_CAP:
            self._full_res_lru.popitem(last=False)
        return rgb

    def _read_frame(self, n: int) -> tuple[np.ndarray, str]:
        """Return (rgb_frame, source_tag) for frame n.

        source_tag is "cache" if served from the preview cache, "full" if
        served from the full-resolution cv2.VideoCapture path.
        """
        if self._should_use_cache() and self.cache is not None:
            cached = self.cache.get(n)
            if cached is not None:
                return cached, "cache"
        return self._full_res_get(n), "full"

    def _seek_to(self, n: int) -> None:
        n = max(0, min(n, self.total_frames - 1))
        try:
            frame_rgb, source = self._read_frame(n)
        except RuntimeError:
            return
        self._current_frame_rgb = frame_rgb
        self._current_frame_source = source
        self.frame_no = n
        # Update scrubber without retriggering the callback loop.
        self._suppress_scrubber_cb = True
        self._scrubber.set(self.frame_no)
        self._suppress_scrubber_cb = False

    # -------------------------------------------------- rendering

    def _render(self, *_args) -> None:
        # Re-evaluate which source we should use given current zoom. A change
        # in canvas size or zoom may force us from cache->full or back.
        if self._current_frame_rgb is None:
            return
        cw, ch = self._canvas_size()
        if cw <= 1 or ch <= 1:
            return

        # If zoom now demands the full-res path but we currently hold a
        # cached frame (or vice-versa), re-read.
        wants_cache = self._should_use_cache()
        have_cache = (self._current_frame_source == "cache")
        if wants_cache != have_cache:
            try:
                frame_rgb, source = self._read_frame(self.frame_no)
                self._current_frame_rgb = frame_rgb
                self._current_frame_source = source
            except RuntimeError:
                pass

        src_rgb = self._current_frame_rgb
        source_tag = self._current_frame_source

        # Compute viewport, crop, resize.
        vx, vy, vw, vh = self._viewport_src_rect()
        x0 = int(np.floor(max(0.0, vx)))
        y0 = int(np.floor(max(0.0, vy)))
        x1 = int(np.ceil(min(float(self.src_w), vx + vw)))
        y1 = int(np.ceil(min(float(self.src_h), vy + vh)))
        x1 = max(x1, x0 + 1)
        y1 = max(y1, y0 + 1)

        # Map source-px viewport to whatever array we actually hold.
        if source_tag == "cache" and self.cache is not None:
            crop = self._crop_from_cache(src_rgb, x0, y0, x1, y1)
        else:
            crop = src_rgb[y0:y1, x0:x1]

        # Aspect-preserving letterbox: pick a uniform canvas-per-SOURCE-pixel
        # scale that fits the clamped viewport entirely inside the canvas,
        # then resize the crop to that disp size and paste centred. The
        # `crop` array can be either source pixels (full-res path) or cache
        # pixels (preview-cache path); either way, its data covers the
        # source viewport (x1-x0) × (y1-y0). We MUST compute the scale in
        # source-px terms so bbox drawing — which uses source-px coords —
        # ends up at the right canvas location regardless of which crop
        # path supplied the pixels.
        viewport_src_w = float(x1 - x0)
        viewport_src_h = float(y1 - y0)
        scale_uniform = min(cw / viewport_src_w, ch / viewport_src_h)
        disp_w = max(1, int(round(viewport_src_w * scale_uniform)))
        disp_h = max(1, int(round(viewport_src_h * scale_uniform)))
        # Interpolation: zoom-in (canvas-per-src-px >= 1) → NEAREST so the
        # user sees real pixels. Zoom-out → INTER_AREA.
        interp = cv2.INTER_NEAREST if scale_uniform >= 1.0 else cv2.INTER_AREA
        resized_crop = cv2.resize(crop, (disp_w, disp_h), interpolation=interp)
        disp = np.zeros((ch, cw, 3), dtype=resized_crop.dtype)
        off_x = (cw - disp_w) // 2
        off_y = (ch - disp_h) // 2
        disp[off_y:off_y + disp_h, off_x:off_x + disp_w] = resized_crop

        # Effective uniform scale for drawing bboxes / tile rects onto
        # disp. Same value for both axes; bbox draw code maps source-px
        # → disp-px via `int((src - x0) * scale + off)`, so aspect is
        # preserved regardless of canvas geometry.
        scale_x = scale_uniform
        scale_y = scale_uniform
        _draw_off_x = off_x
        _draw_off_y = off_y

        # The disp image is RGB (cache is RGB; full-res path converts at
        # read time). cv2 draw fns are channel-agnostic — they just write
        # the colour tuple verbatim — but our palette is BGR by convention,
        # so swap each colour to RGB at draw time.
        def _rgb(bgr: tuple[int, int, int]) -> tuple[int, int, int]:
            return (bgr[2], bgr[1], bgr[0])

        # Draw bboxes for visible runs.
        containment_merge, is_phantom = _load_analyze()
        conf_min = float(self._conf_min_var.get())
        hide_phantoms = bool(self._hide_phantoms_var.get())
        apply_cm = bool(self._containment_merge_var.get())
        self._phantoms_hidden_per_run = {}
        self._merged_per_run = {}
        visible_runs: list[Run] = []
        for r in self.runs:
            if not r.visible_var.get():
                continue
            visible_runs.append(r)
            colour = _rgb(r.colour_bgr)
            n_phantoms = 0
            # Stage 1: filter the per-frame det list by phantom rule.
            # (Confidence is checked later in the draw loop so the slider
            # doesn't bust the containment-merge cache.)
            raw_dets = r.idx.get(self.frame_no, [])
            after_phantom: list[dict] = []
            for det in raw_dets:
                if hide_phantoms and r.tile_rects and is_phantom(det, r.tile_rects):
                    n_phantoms += 1
                    continue
                after_phantom.append(det)
            # Stage 2: containment-merge over the post-phantom list. Cache
            # keyed on (frame, params, post-phantom-list identity).
            if apply_cm:
                cache_key = (self.frame_no, hide_phantoms, 0.5, 0.0)
                cached = r._cm_cache.get(cache_key)
                if cached is None:
                    kept = containment_merge(after_phantom,
                                             area_ratio_max=0.5,
                                             center_slack=0.0)
                    n_merged = len(after_phantom) - len(kept)
                    r._cm_cache[cache_key] = (kept, n_merged)
                    # Keep cache bounded — drop oldest entries beyond a small
                    # window so scrubbing through the video doesn't leak.
                    if len(r._cm_cache) > 128:
                        first_key = next(iter(r._cm_cache))
                        r._cm_cache.pop(first_key, None)
                else:
                    kept, n_merged = cached
                if n_merged > 0:
                    self._merged_per_run[r.label] = n_merged
                draw_dets = kept
            else:
                draw_dets = after_phantom
            for det in draw_dets:
                conf = float(det.get("confidence", 0.0))
                if conf < conf_min:
                    continue
                bx, by, bw, bh = det["bbox"]
                # bbox in source-px
                sx1 = bx * self.src_w
                sy1 = by * self.src_h
                sx2 = (bx + bw) * self.src_w
                sy2 = (by + bh) * self.src_h
                # Cull if entirely outside the (clamped) viewport.
                if sx2 < x0 or sx1 > x1 or sy2 < y0 or sy1 > y1:
                    continue
                # Map to canvas px (with letterbox offset).
                dx1 = int((sx1 - x0) * scale_x) + _draw_off_x
                dy1 = int((sy1 - y0) * scale_y) + _draw_off_y
                dx2 = int((sx2 - x0) * scale_x) + _draw_off_x
                dy2 = int((sy2 - y0) * scale_y) + _draw_off_y
                cv2.rectangle(disp, (dx1, dy1), (dx2, dy2), colour, 2)
                tag = f"{det.get('label', '?')} {conf:.2f}"
                cv2.putText(disp, tag, (dx1, max(15, dy1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1,
                            cv2.LINE_AA)
            if hide_phantoms and n_phantoms > 0:
                self._phantoms_hidden_per_run[r.label] = n_phantoms

        # Tile rectangles, drawn between bboxes and HUD so the bboxes win
        # the foreground but the HUD overpaints the tiles where they
        # overlap.
        tile_source_run, tile_rects_drawn = self._draw_tiles(
            disp, x0, y0, x1, y1, scale_x, scale_y, visible_runs,
            _draw_off_x, _draw_off_y,
        )

        # HUD on the rescaled canvas (NOT on source — that's the key fix).
        self._draw_hud(disp, visible_runs, tile_source_run, tile_rects_drawn,
                       source_tag=source_tag)

        # disp is already RGB — push straight to PIL.
        img = Image.fromarray(disp)
        self._photo_ref = ImageTk.PhotoImage(img)
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self._photo_ref)

    # -------------------------------------------------- cache helpers

    def _crop_from_cache(self, cache_rgb: np.ndarray,
                         x0: int, y0: int, x1: int, y1: int) -> np.ndarray:
        """Crop the source-px viewport (x0,y0)-(x1,y1) out of a cached frame.

        Maps source coords -> cache coords using the cache's letterbox
        layout (cache_x = left + src_x * scale; cache_y = top + src_y * scale).
        Returns an array view; no copy is performed.
        """
        assert self.cache is not None
        top, left, rh, rw = self.cache.image_area
        s = self.cache.scale
        cx0 = int(np.floor(left + x0 * s))
        cy0 = int(np.floor(top + y0 * s))
        cx1 = int(np.ceil(left + x1 * s))
        cy1 = int(np.ceil(top + y1 * s))
        # Clamp to the actual image area (don't crop into letterbox border).
        cx0 = max(left, min(cx0, left + rw))
        cy0 = max(top, min(cy0, top + rh))
        cx1 = max(cx0 + 1, min(cx1, left + rw))
        cy1 = max(cy0 + 1, min(cy1, top + rh))
        return cache_rgb[cy0:cy1, cx0:cx1]

    def _on_cache_progress(self, _done: int, _total: int) -> None:
        # Called from the populator thread. Don't touch Tk directly — the
        # HUD reads PreviewCache.progress on the next render. We do, however,
        # poke the UI thread to re-render via after_idle so the HUD progress
        # line updates without waiting for user input.
        try:
            self.root.after_idle(self._render)
        except Exception:
            pass

    def _draw_tiles(self, disp: np.ndarray,
                    x0: int, y0: int, x1: int, y1: int,
                    scale_x: float, scale_y: float,
                    visible_runs: list[Run],
                    off_x: int = 0, off_y: int = 0) -> tuple[Run | None, int]:
        """Overlay the tile rectangles from the selected source run.

        Returns ``(source_run, n_rects_drawn)`` for the HUD legend. If the
        overlay is disabled, no source is selected, or no runs are visible,
        returns ``(None, 0)``.
        """
        if not self._show_tiles_var.get() or not visible_runs:
            return None, 0
        wanted = self._tile_source_var.get()
        source_run: Run | None = next(
            (r for r in visible_runs if r.label == wanted), None
        )
        if source_run is None:
            return None, 0

        # Category → BGR. Multi-scale tiles are the fragment-producing
        # dense grid (boundary-stripped by the aggregator) — drawn in cyan
        # to stand out from common detection colours (green person /
        # orange vehicle). Single-scale rescue tiles in white-yellow.
        # Dynamic tiles (future per-frame attachments) in lime green.
        CAT_COLOURS = {
            "multi-scale":    (255, 200, 0),    # cyan-ish blue (BGR)
            "single-scale":   (0, 255, 255),    # yellow (BGR)
            "dynamic":        (0, 255, 100),    # lime green (BGR)
            "dynamic-merged": (255, 0, 200),    # magenta (BGR)
        }
        DEFAULT_TILE_COLOUR = (255, 255, 255)
        drawn = 0
        per_frame = source_run.tiles_by_frame.get(self.frame_no)
        rects_to_draw = per_frame if per_frame else source_run.tile_rects_typed
        for (nx, ny, nw, nh, cat) in rects_to_draw:
            sx1 = nx * self.src_w
            sy1 = ny * self.src_h
            sx2 = (nx + nw) * self.src_w
            sy2 = (ny + nh) * self.src_h
            # Cull rectangles entirely outside the viewport.
            if sx2 < x0 or sx1 > x1 or sy2 < y0 or sy1 > y1:
                continue
            dx1 = int((sx1 - x0) * scale_x) + off_x
            dy1 = int((sy1 - y0) * scale_y) + off_y
            dx2 = int((sx2 - x0) * scale_x) + off_x
            dy2 = int((sy2 - y0) * scale_y) + off_y
            colour = CAT_COLOURS.get(cat, DEFAULT_TILE_COLOUR)
            cv2.rectangle(disp, (dx1, dy1), (dx2, dy2), colour, 1)
            drawn += 1
        return source_run, drawn

    def _draw_hud(self, disp: np.ndarray, visible_runs: list[Run],
                  tile_source_run: "Run | None" = None,
                  tile_rects_drawn: int = 0,
                  source_tag: str = "full") -> None:
        # If the frame came from the cache but the user has zoomed past the
        # cache's native resolution, badge the HUD with "px*" — they're
        # looking at upscaled cache pixels, not real source pixels. The "*"
        # hints "would be sharper with native-res zoom enabled".
        if source_tag == "cache" and not self._cache_is_dense_enough():
            badge = "cache px*"
        else:
            badge = source_tag
        lines: list[str] = []
        lines.append(
            f"frame {self.frame_no} / {self.total_frames - 1}  [{badge}]"
        )
        lines.append(f"zoom {self.zoom_factor:.2f}x")
        if self._cursor_canvas is not None:
            cx, cy = self._cursor_canvas
            sx, sy = self._canvas_to_src(cx, cy)
            lines.append(f"src px: ({int(sx)}, {int(sy)})")
        else:
            lines.append("src px: -")

        # Preview-cache progress (suppressed once complete or disabled).
        cache_line: str | None = None
        if self.cache is not None and not self.cache.is_complete:
            done, total = self.cache.progress
            pct = (100.0 * done / total) if total else 0.0
            cache_line = f"preview cache: {done}/{total} ({pct:.1f}%)"
        if cache_line is not None:
            lines.append(cache_line)

        tile_line: str | None = None
        if tile_source_run is not None:
            cat_counts: dict[str, int] = {}
            per_frame_hud = tile_source_run.tiles_by_frame.get(self.frame_no)
            rects_for_hud = per_frame_hud if per_frame_hud else tile_source_run.tile_rects_typed
            for (_x, _y, _w, _h, cat) in rects_for_hud:
                cat_counts[cat] = cat_counts.get(cat, 0) + 1
            cat_summary = " ".join(
                f"{cat}={n}"
                for cat, n in sorted(cat_counts.items())
            ) or "(none)"
            tile_line = (f"tile src: {tile_source_run.label}"
                         f"  (n={tile_rects_drawn}; {cat_summary})")

        # Background strip behind HUD for legibility.
        line_h = 22
        n_lines = len(lines) + len(visible_runs) + (1 if tile_line else 0)
        pad = 6
        box_w = 320
        box_h = pad * 2 + line_h * max(1, n_lines)
        sub = disp[0:box_h, 0:box_w]
        # 50% darken
        if sub.size > 0:
            sub[:] = (sub.astype(np.int32) // 2).astype(np.uint8)

        y = pad + 16
        for txt in lines:
            cv2.putText(disp, txt, (pad, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 255), 1, cv2.LINE_AA)
            y += line_h
        for r in visible_runs:
            # disp is RGB; palette is BGR — swap channels for legend swatches.
            rgb_colour = (r.colour_bgr[2], r.colour_bgr[1], r.colour_bgr[0])
            n_ph = self._phantoms_hidden_per_run.get(r.label, 0)
            n_mg = self._merged_per_run.get(r.label, 0)
            extras: list[str] = []
            if n_ph > 0:
                extras.append(f"phantoms hidden: {n_ph}")
            if n_mg > 0:
                extras.append(f"merged: {n_mg}")
            txt = r.label if not extras else f"{r.label}  ({'; '.join(extras)})"
            cv2.putText(disp, txt, (pad, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        rgb_colour, 2, cv2.LINE_AA)
            y += line_h
        if tile_line is not None:
            cv2.putText(disp, tile_line, (pad, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 255), 1, cv2.LINE_AA)
            y += line_h

    # -------------------------------------------------- events

    def _on_scrubber(self, val: str) -> None:
        if self._suppress_scrubber_cb:
            return
        try:
            n = int(float(val))
        except (TypeError, ValueError):
            return
        if n == self.frame_no:
            return
        self._seek_to(n)
        self._render()

    def _on_speed_change(self) -> None:
        self.speed = float(self._speed_var.get())
        if self.speed == 0.0:
            self._cancel_playback()
            self._play_btn.config(text="Play")
        else:
            self._play_btn.config(text="Pause")
            self._schedule_advance()

    def _toggle_play(self) -> None:
        if self.speed == 0.0:
            # Default to 1x forward.
            self.speed = 1.0
            self._speed_var.set(1.0)
            self._play_btn.config(text="Pause")
            self._schedule_advance()
        else:
            self.speed = 0.0
            self._speed_var.set(0.0)
            self._cancel_playback()
            self._play_btn.config(text="Play")

    def _pause(self) -> None:
        if self.speed != 0.0:
            self.speed = 0.0
            self._speed_var.set(0.0)
            self._cancel_playback()
            self._play_btn.config(text="Play")

    def _step_fwd(self) -> None:
        self._pause()
        self._seek_to(self.frame_no + 1)
        self._render()

    def _step_back(self) -> None:
        self._pause()
        self._seek_to(self.frame_no - 1)
        self._render()

    def _goto_start(self) -> None:
        self._pause()
        self._seek_to(0)
        self._render()

    def _goto_end(self) -> None:
        self._pause()
        self._seek_to(self.total_frames - 1)
        self._render()

    def _reset_view(self) -> None:
        self.zoom_factor = 1.0
        self.pan_x = self.src_w / 2.0
        self.pan_y = self.src_h / 2.0
        # Keep the sidebar slider in sync without retriggering its callback.
        self._sync_zoom_slider()
        self._render()

    def _cancel_playback(self) -> None:
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None

    def _schedule_advance(self) -> None:
        self._cancel_playback()
        if self.speed == 0.0:
            return
        # Real-time pacing: subtract the time already spent decoding+rendering
        # the current frame from the target frame period, so a heavy frame
        # (e.g. 4K decode + overlay draw) doesn't add on top of the period and
        # drag playback into slow motion. Clamps to >=1ms when work overruns.
        target_ms = 1000.0 / (self.fps * abs(self.speed))
        delay_ms = max(1, int(round(target_ms - self._last_work_ms)))
        self._after_id = self.root.after(delay_ms, self._advance_frame)

    def _advance_frame(self) -> None:
        self._after_id = None
        if self.speed == 0.0:
            return
        step = 1 if self.speed > 0 else -1
        new_n = self.frame_no + step
        if new_n < 0 or new_n >= self.total_frames:
            # Hit an end — pause.
            self._pause()
            return
        t0 = time.perf_counter()
        self._seek_to(new_n)
        self._render()
        self._last_work_ms = (time.perf_counter() - t0) * 1000.0
        self._schedule_advance()

    # ----- mouse

    def _on_wheel(self, event) -> None:
        # Determine zoom direction.
        if event.num == 4:           # X11 wheel up
            direction = +1
        elif event.num == 5:         # X11 wheel down
            direction = -1
        else:                        # Win/macOS — event.delta
            direction = +1 if getattr(event, "delta", 0) > 0 else -1

        # Keep the source-px under the cursor anchored after zoom.
        cw, ch = self._canvas_size()
        cx = max(0, min(event.x, cw - 1))
        cy = max(0, min(event.y, ch - 1))
        sx_before, sy_before = self._canvas_to_src(cx, cy)

        old_zoom = self.zoom_factor
        if direction > 0:
            new_zoom = old_zoom * ZOOM_STEP
        else:
            new_zoom = old_zoom / ZOOM_STEP
        new_zoom = max(ZOOM_MIN, min(ZOOM_MAX, new_zoom))
        if new_zoom == old_zoom:
            return
        self.zoom_factor = new_zoom

        # Re-derive viewport at new zoom but with the OLD pan; figure out
        # where the cursor would now land in source coords, then nudge pan
        # so the original source pixel lands back under the cursor.
        sx_after, sy_after = self._canvas_to_src(cx, cy)
        self.pan_x += (sx_before - sx_after)
        self.pan_y += (sy_before - sy_after)

        # Keep the sidebar slider in sync without retriggering its callback.
        self._sync_zoom_slider()
        self._render()

    def _on_drag_start(self, event) -> None:
        self._drag_last = (event.x, event.y)
        self.canvas.config(cursor="fleur")

    def _on_drag_motion(self, event) -> None:
        if self._drag_last is None:
            return
        lx, ly = self._drag_last
        dx_canvas = event.x - lx
        dy_canvas = event.y - ly
        self._drag_last = (event.x, event.y)
        scale = self._effective_scale()
        if scale <= 0:
            return
        # Dragging right should move the view right (i.e. pan_x decreases
        # because we're showing more of the left side of source).
        self.pan_x -= dx_canvas / scale
        self.pan_y -= dy_canvas / scale
        self._cursor_canvas = (event.x, event.y)
        self._render()

    def _on_drag_end(self, _event) -> None:
        self._drag_last = None
        self.canvas.config(cursor="crosshair")

    def _on_motion(self, event) -> None:
        self._cursor_canvas = (event.x, event.y)
        # Full re-render keeps things simple; performance is fine for
        # Phase 1's "it works" bar. If this gets sluggish, Phase 2 can
        # repaint only the HUD region.
        self._render()

    def _on_leave(self, _event) -> None:
        self._cursor_canvas = None
        self._render()

    # ----- shutdown

    def _quit(self) -> None:
        self._cancel_playback()
        try:
            self.cap.release()
        except Exception:
            pass
        if self.cache is not None:
            try:
                self.cache.close()
            except Exception:
                pass
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def export_runs(video_path: Path, runs: list["Run"], out_path: Path, *,
                fps: float | None = None,
                out_width: int | None = None,
                start: int = 0,
                end: int | None = None,
                conf_min: float = 0.25,
                hide_phantoms: bool = True,
                containment: bool = True,
                tiles_source: str | None = None) -> int:
    """Headless render of all loaded runs' overlays straight to an MP4.

    Re-uses the viewer's draw logic (phantom hide, containment merge, per-run
    BGR palette, per-frame dynamic tiles) but skips Tk entirely: each frame is
    decoded, overlays are drawn at the chosen output resolution, and the frame
    is written via cv2.VideoWriter. Output colours match the GUI swatches
    because both the source frame and the palette are BGR.
    """
    containment_merge, is_phantom = _load_analyze()
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"ERROR: cannot open video {video_path}")
        return 1
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    out_fps = fps or src_fps
    if out_width and out_width < src_w:
        out_w = out_width
        out_h = int(round(src_h * out_width / src_w))
        out_h -= out_h % 2  # keep even dims for the encoder
    else:
        out_w, out_h = src_w, src_h
    first = max(0, start)
    last = (total - 1) if end is None else min(end, total - 1)

    tile_run: "Run | None" = None
    if tiles_source:
        tile_run = next((r for r in runs if r.label == tiles_source), None)
        if tile_run is None:
            print(f"WARN: --export-tiles source '{tiles_source}' not found; "
                  f"available labels: {[r.label for r in runs]}")

    writer = cv2.VideoWriter(str(out_path),
                             cv2.VideoWriter_fourcc(*"mp4v"),
                             out_fps, (out_w, out_h))
    if not writer.isOpened():
        print(f"ERROR: cannot open VideoWriter for {out_path}")
        cap.release()
        return 1

    CAT_COLOURS = {
        "multi-scale":    (255, 200, 0),
        "single-scale":   (0, 255, 255),
        "dynamic":        (0, 255, 100),
        "dynamic-merged": (255, 0, 200),
    }
    sx, sy = float(out_w), float(out_h)
    resize_needed = (out_w, out_h) != (src_w, src_h)

    cap.set(cv2.CAP_PROP_POS_FRAMES, first)
    n = first
    written = 0
    while n <= last:
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        if resize_needed:
            frame = cv2.resize(frame, (out_w, out_h),
                               interpolation=cv2.INTER_AREA)

        # Tiles first so bboxes win the foreground.
        if tile_run is not None:
            per_frame = tile_run.tiles_by_frame.get(n)
            rects = per_frame if per_frame else tile_run.tile_rects_typed
            for (tx, ty, tw, th, cat) in rects:
                cv2.rectangle(frame,
                              (int(tx * sx), int(ty * sy)),
                              (int((tx + tw) * sx), int((ty + th) * sy)),
                              CAT_COLOURS.get(cat, (255, 255, 255)), 1)

        for r in runs:
            colour = r.colour_bgr  # frame and palette are both BGR
            dets = []
            for det in r.idx.get(n, []):
                if hide_phantoms and r.tile_rects and is_phantom(det, r.tile_rects):
                    continue
                dets.append(det)
            if containment:
                dets = containment_merge(dets, area_ratio_max=0.5,
                                         center_slack=0.0)
            for det in dets:
                conf = float(det.get("confidence", 0.0))
                if conf < conf_min:
                    continue
                bx, by, bw, bh = det["bbox"]
                p1 = (int(bx * sx), int(by * sy))
                p2 = (int((bx + bw) * sx), int((by + bh) * sy))
                cv2.rectangle(frame, p1, p2, colour, 2)
                tag = f"{det.get('label', '?')} {conf:.2f}"
                cv2.putText(frame, tag, (p1[0], max(15, p1[1] - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1,
                            cv2.LINE_AA)

        cv2.putText(frame, f"frame {n}", (8, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
                    cv2.LINE_AA)
        writer.write(frame)
        written += 1
        n += 1
        if written % 100 == 0:
            print(f"  ... {written} frames written (frame {n - 1})")

    writer.release()
    cap.release()
    print(f"Wrote {written} frames to {out_path} "
          f"({out_w}x{out_h} @ {out_fps:.2f} fps)")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--video", type=Path, required=True,
                    help="Source video file (e.g. .MP4).")
    ap.add_argument("--frames", action="append", required=True,
                    help="frames.json path, optionally suffixed with ':LABEL'. "
                         "Repeat for multiple overlays.")
    ap.add_argument("--start-frame", type=int, default=0,
                    help="Frame number to seek to initially (default 0).")
    ap.add_argument("--conf-min", type=float, default=0.25,
                    help="Hide boxes below this confidence (default 0.25).")
    ap.add_argument("--no-cache", action="store_true",
                    help="Disable the in-RAM / on-disk preview cache; always "
                         "decode from the full-res source. Useful when disk "
                         "is tight or for performance debugging.")
    ap.add_argument("--cache-width", type=int, default=1920,
                    help="Preview cache target width (default 1920).")
    ap.add_argument("--cache-height", type=int, default=1080,
                    help="Preview cache target height (default 1080).")
    # --- Headless export (no GUI): render overlays straight to an MP4 ---
    ap.add_argument("--export", type=Path, default=None,
                    help="Headless mode: render all --frames overlays onto the "
                         "video and write to this MP4 path (no GUI window).")
    ap.add_argument("--export-fps", type=float, default=None,
                    help="Output FPS for --export (default: source FPS).")
    ap.add_argument("--export-width", type=int, default=None,
                    help="Downscale --export output to this width, preserving "
                         "aspect (default: full source resolution).")
    ap.add_argument("--export-end", type=int, default=None,
                    help="Last frame (inclusive) for --export (default: last).")
    ap.add_argument("--export-tiles", metavar="RUN_LABEL", default=None,
                    help="Draw tile rectangles from the run with this label "
                         "in the exported video.")
    ap.add_argument("--no-hide-phantoms", action="store_true",
                    help="Keep tile-edge phantom detections in --export "
                         "(default: hidden, matching the GUI default).")
    ap.add_argument("--no-containment-merge", action="store_true",
                    help="Disable containment-merge in --export "
                         "(default: on, matching the GUI default).")
    args = ap.parse_args(argv)

    if not args.video.is_file():
        print(f"ERROR: video file not found: {args.video}")
        return 1

    runs: list[Run] = []
    for i, raw in enumerate(args.frames):
        p, lbl_override = parse_frames_arg(raw)
        if not p.is_file():
            print(f"ERROR: frames file not found: {p}")
            return 1
        lbl, idx, cfg, tiles_by_frame = load_frames_indexed(p)
        if lbl_override:
            lbl = lbl_override
        colour = PALETTE[i % len(PALETTE)]
        runs.append(Run(label=lbl, idx=idx, colour_bgr=colour, config=cfg,
                        tiles_by_frame=tiles_by_frame))

    if args.export is not None:
        return export_runs(
            args.video, runs, args.export,
            fps=args.export_fps,
            out_width=args.export_width,
            start=args.start_frame,
            end=args.export_end,
            conf_min=args.conf_min,
            hide_phantoms=not args.no_hide_phantoms,
            containment=not args.no_containment_merge,
            tiles_source=args.export_tiles,
        )

    root = tk.Tk()
    try:
        OverlayViewer(root, args.video, runs,
                      start_frame=args.start_frame,
                      conf_min=args.conf_min,
                      use_cache=not args.no_cache,
                      cache_target_w=args.cache_width,
                      cache_target_h=args.cache_height)
    except RuntimeError as e:
        print(f"ERROR: {e}")
        return 1
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
