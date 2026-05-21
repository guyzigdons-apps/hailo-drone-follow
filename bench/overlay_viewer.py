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

Phase 2 (stubbed disabled in sidebar):
  * Tile-rectangle overlay.
  * Conf-min slider.

Usage:
  python bench/overlay_viewer.py \\
      --video /path/to/video.MP4 \\
      --frames /path/to/pxt_GT-12x9-25.frames.json:GT \\
      --frames /path/to/pxt_3x2-native.frames.json:3x2
"""

from __future__ import annotations

import argparse
import json
import tkinter as tk
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageTk

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

SIDEBAR_WIDTH = 250


def bgr_to_hex(bgr: tuple[int, int, int]) -> str:
    b, g, r = bgr
    return f"#{r:02x}{g:02x}{b:02x}"


def load_frames_indexed(path: Path) -> tuple[str, dict[int, list[dict]]]:
    with path.open() as f:
        doc = json.load(f)
    idx = {int(fr["frame"]): fr["detections"] for fr in doc["frames"]}
    return doc.get("label") or path.stem, idx


def parse_frames_arg(arg: str) -> tuple[Path, str | None]:
    if ":" in arg:
        p, lbl = arg.rsplit(":", 1)
        return Path(p), lbl
    return Path(arg), None


class Run:
    """One frames.json overlay source."""

    def __init__(self, label: str, idx: dict[int, list[dict]],
                 colour_bgr: tuple[int, int, int]):
        self.label = label
        self.idx = idx
        self.colour_bgr = colour_bgr
        self.visible_var: tk.BooleanVar | None = None  # set after Tk root exists


class OverlayViewer:
    def __init__(self, root: tk.Tk, video_path: Path, runs: list[Run],
                 start_frame: int = 0, conf_min: float = 0.25):
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
        self._suppress_scrubber_cb = False

        # View transform — pan is the source-px coord of the centre of the viewport.
        self.zoom_factor = 1.0
        self.pan_x = self.src_w / 2.0
        self.pan_y = self.src_h / 2.0

        # Mouse state
        self._drag_last: tuple[int, int] | None = None
        self._cursor_canvas: tuple[int, int] | None = None  # last (x,y) of <Motion>

        # Current decoded BGR frame (numpy)
        self._current_frame_bgr: np.ndarray | None = None
        # Keep a reference to the last ImageTk to prevent GC
        self._photo_ref: ImageTk.PhotoImage | None = None

        # Create per-run Tk visibility BooleanVars now that root exists.
        for r in self.runs:
            r.visible_var = tk.BooleanVar(value=True)

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

        # --- Sidebar ---
        sidebar = tk.Frame(main, width=SIDEBAR_WIDTH, bg="#202020")
        sidebar.pack(side=tk.RIGHT, fill=tk.Y)
        sidebar.pack_propagate(False)

        # Runs section
        runs_lf = tk.LabelFrame(sidebar, text="Runs", fg="white", bg="#202020",
                                labelanchor="nw", padx=6, pady=4)
        runs_lf.pack(fill=tk.X, padx=6, pady=(8, 4))
        for r in self.runs:
            row = tk.Frame(runs_lf, bg="#202020")
            row.pack(fill=tk.X, pady=1)
            swatch = tk.Label(row, text="    ", bg=bgr_to_hex(r.colour_bgr),
                              width=2, relief=tk.RAISED, bd=1)
            swatch.pack(side=tk.LEFT, padx=(0, 6))
            cb = tk.Checkbutton(row, text=r.label, variable=r.visible_var,
                                fg="white", bg="#202020",
                                activebackground="#202020",
                                activeforeground="white",
                                selectcolor="#404040",
                                command=self._render)
            cb.pack(side=tk.LEFT, anchor="w")

        # Tiles (Phase 2 stub)
        tiles_lf = tk.LabelFrame(sidebar, text="Overlays", fg="white",
                                 bg="#202020", labelanchor="nw",
                                 padx=6, pady=4)
        tiles_lf.pack(fill=tk.X, padx=6, pady=4)
        self._show_tiles_var = tk.BooleanVar(value=False)
        tk.Checkbutton(tiles_lf, text="Show tiles  (Phase 2)",
                       variable=self._show_tiles_var,
                       fg="white", bg="#202020",
                       activebackground="#202020",
                       activeforeground="white",
                       selectcolor="#404040",
                       state=tk.DISABLED).pack(anchor="w")

        # Conf-min slider stub (Phase 2)
        conf_lf = tk.LabelFrame(sidebar, text=f"Conf-min ({self.conf_min:.2f})",
                                fg="white", bg="#202020", labelanchor="nw",
                                padx=6, pady=4)
        conf_lf.pack(fill=tk.X, padx=6, pady=4)
        self._conf_slider = tk.Scale(conf_lf, from_=0.0, to=1.0,
                                     resolution=0.01,
                                     orient=tk.HORIZONTAL,
                                     bg="#202020", fg="white",
                                     troughcolor="#404040",
                                     highlightthickness=0,
                                     state=tk.DISABLED,
                                     label="(Phase 2)")
        self._conf_slider.set(self.conf_min)
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

    def _seek_to(self, n: int) -> None:
        n = max(0, min(n, self.total_frames - 1))
        # Always seek; cheap correctness wins over speed in Phase 1.
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, n)
        ok, frame = self.cap.read()
        if ok:
            self._current_frame_bgr = frame
            self.frame_no = n
        # Update scrubber without retriggering the callback loop.
        self._suppress_scrubber_cb = True
        self._scrubber.set(self.frame_no)
        self._suppress_scrubber_cb = False

    # -------------------------------------------------- rendering

    def _render(self, *_args) -> None:
        if self._current_frame_bgr is None:
            return
        cw, ch = self._canvas_size()
        if cw <= 1 or ch <= 1:
            return

        src = self._current_frame_bgr
        # Compute viewport, crop, resize.
        vx, vy, vw, vh = self._viewport_src_rect()
        x0 = int(np.floor(max(0.0, vx)))
        y0 = int(np.floor(max(0.0, vy)))
        x1 = int(np.ceil(min(float(self.src_w), vx + vw)))
        y1 = int(np.ceil(min(float(self.src_h), vy + vh)))
        x1 = max(x1, x0 + 1)
        y1 = max(y1, y0 + 1)
        crop = src[y0:y1, x0:x1]

        # Interpolation: zoom-in (resize factor >= 1) => NEAREST so the
        # user sees real pixels. Zoom-out (factor < 1) => INTER_AREA.
        crop_h, crop_w = crop.shape[:2]
        resize_factor = (cw / crop_w + ch / crop_h) / 2.0
        interp = cv2.INTER_NEAREST if resize_factor >= 1.0 else cv2.INTER_AREA
        # cv2.resize wants (w, h). We resize to canvas-fill (the crop
        # already matches the viewport aspect ratio modulo clamping).
        disp = cv2.resize(crop, (cw, ch), interpolation=interp)

        # Effective scale for drawing bboxes onto disp.
        scale_x = cw / float(x1 - x0)
        scale_y = ch / float(y1 - y0)

        # Draw bboxes for visible runs.
        visible_runs: list[Run] = []
        for r in self.runs:
            if not r.visible_var.get():
                continue
            visible_runs.append(r)
            colour = r.colour_bgr
            for det in r.idx.get(self.frame_no, []):
                conf = float(det.get("confidence", 0.0))
                if conf < self.conf_min:
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
                # Map to canvas px.
                dx1 = int((sx1 - x0) * scale_x)
                dy1 = int((sy1 - y0) * scale_y)
                dx2 = int((sx2 - x0) * scale_x)
                dy2 = int((sy2 - y0) * scale_y)
                cv2.rectangle(disp, (dx1, dy1), (dx2, dy2), colour, 2)
                tag = f"{det.get('label', '?')} {conf:.2f}"
                cv2.putText(disp, tag, (dx1, max(15, dy1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1,
                            cv2.LINE_AA)

        # HUD on the rescaled canvas (NOT on source — that's the key fix).
        self._draw_hud(disp, visible_runs)

        # Push to canvas.
        rgb = cv2.cvtColor(disp, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb)
        self._photo_ref = ImageTk.PhotoImage(img)
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self._photo_ref)

    def _draw_hud(self, disp: np.ndarray, visible_runs: list[Run]) -> None:
        lines: list[str] = []
        lines.append(f"frame {self.frame_no} / {self.total_frames - 1}")
        lines.append(f"zoom {self.zoom_factor:.2f}x")
        if self._cursor_canvas is not None:
            cx, cy = self._cursor_canvas
            sx, sy = self._canvas_to_src(cx, cy)
            lines.append(f"src px: ({int(sx)}, {int(sy)})")
        else:
            lines.append("src px: -")

        # Background strip behind HUD for legibility.
        line_h = 22
        n_lines = len(lines) + len(visible_runs)
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
            cv2.putText(disp, r.label, (pad, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        r.colour_bgr, 2, cv2.LINE_AA)
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
        delay_ms = max(1, int(1000.0 / (self.fps * abs(self.speed))))
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
        self._seek_to(new_n)
        self._render()
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
        try:
            self.root.destroy()
        except tk.TclError:
            pass


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
        lbl, idx = load_frames_indexed(p)
        if lbl_override:
            lbl = lbl_override
        colour = PALETTE[i % len(PALETTE)]
        runs.append(Run(label=lbl, idx=idx, colour_bgr=colour))

    root = tk.Tk()
    try:
        OverlayViewer(root, args.video, runs,
                      start_frame=args.start_frame,
                      conf_min=args.conf_min)
    except RuntimeError as e:
        print(f"ERROR: {e}")
        return 1
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
