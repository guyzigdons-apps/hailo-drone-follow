"""Click-through GUI for human GT review.

Shows each flagged review case (a pre-rendered, zoomed, annotated PNG) one at a
time with verdict buttons, and writes ``review_decisions.json`` into the same
outdir for ``run_gt_verify --finalize`` to consume.

    source setup_env.sh
    DISPLAY=:1 XAUTHORITY=/run/user/10615/gdm/Xauthority \
      python -m tiling_lab.gt.gt_review_gui \
        --outdir tiling_lab/runs/gt_verify_0026_fov50

Verdicts (also keyboard):
  merge case       -> Merge (m)   | Keep separate (k)
  keep_short case  -> Keep  (k)   | Drop (d)
Navigation: Back (Left arrow), Next/skip (Right arrow / space).
Decisions autosave on every click; closing the window also saves.
A missing verdict is treated CONSERVATIVELY by finalize (merge->kept separate,
keep_short->kept), so you only need to click the cases you want to change.
"""
from __future__ import annotations

import argparse
import json
import tkinter as tk
from pathlib import Path

from PIL import Image, ImageTk

_MAX_W = 960  # max display width for a case image


def _verdict_buttons(kind: str):
    """Return [(label, hotkey, verdict_value), ...] for a case kind."""
    if kind == "merge":
        return [("Merge (m)", "m", "merge"),
                ("Keep separate (k)", "k", "keep")]
    if kind == "keep_short":
        return [("Keep (k)", "k", "keep"),
                ("Drop (d)", "d", "drop")]
    return [("OK (k)", "k", "keep")]


class ReviewApp:
    def __init__(self, root: tk.Tk, outdir: Path):
        self.root = root
        self.outdir = outdir
        self.review_dir = outdir / "review"
        self.dec_path = outdir / "review_decisions.json"
        self.cases = json.loads((outdir / "review_queue.json").read_text())["cases"]
        self.decisions: dict[str, str] = {}
        if self.dec_path.exists():
            self.decisions = json.loads(self.dec_path.read_text())
        self.idx = 0
        self._photo = None  # keep a ref so Tk doesn't GC the image

        root.title("GT review")
        self.progress = tk.Label(root, font=("TkDefaultFont", 12, "bold"))
        self.progress.pack(pady=4)
        self.img_label = tk.Label(root)
        self.img_label.pack(padx=8)
        self.caption = tk.Label(root, wraplength=_MAX_W, fg="#222")
        self.caption.pack(pady=4)
        self.current = tk.Label(root, fg="#0a0")
        self.current.pack()
        self.btn_row = tk.Frame(root)
        self.btn_row.pack(pady=8)
        nav = tk.Frame(root)
        nav.pack(pady=4)
        tk.Button(nav, text="◀ Back", command=self.back).pack(side=tk.LEFT, padx=4)
        tk.Button(nav, text="Next ▶", command=self.next).pack(side=tk.LEFT, padx=4)
        tk.Button(nav, text="Save & Quit", command=self.quit).pack(side=tk.LEFT, padx=20)

        root.bind("<Left>", lambda e: self.back())
        root.bind("<Right>", lambda e: self.next())
        root.bind("<space>", lambda e: self.next())
        root.protocol("WM_DELETE_WINDOW", self.quit)
        self.render()

    def _save(self):
        self.dec_path.write_text(json.dumps(self.decisions, indent=2))

    def render(self):
        n = len(self.cases)
        if not (0 <= self.idx < n):
            return
        case = self.cases[self.idx]
        kind = case["kind"]
        decided = sum(1 for c in range(n) if str(c) in self.decisions)
        self.progress.config(
            text=f"Case {self.idx + 1} / {n}   ({decided} decided)   kind: {kind}")

        img_path = self.review_dir / f"case_{self.idx:04d}.png"
        if img_path.exists():
            im = Image.open(img_path)
            if im.width > _MAX_W:
                h = int(im.height * _MAX_W / im.width)
                im = im.resize((_MAX_W, h))
            self._photo = ImageTk.PhotoImage(im)
            self.img_label.config(image=self._photo)
        else:
            self.img_label.config(image="", text=f"(missing {img_path.name})")

        ids = "+".join(str(i) for i in case["track_ids"])
        self.caption.config(text=f"ids {ids}  frame {case['frame']}  ({case.get('reason','')})")
        cur = self.decisions.get(str(self.idx))
        self.current.config(text=f"current verdict: {cur}" if cur else "current verdict: (none)")

        for w in self.btn_row.winfo_children():
            w.destroy()
        for (label, _hot, value) in _verdict_buttons(kind):
            tk.Button(self.btn_row, text=label, width=16,
                      command=lambda v=value: self.decide(v)).pack(side=tk.LEFT, padx=6)
        # hotkeys (rebound per case so 'd'/'m' only fire where they apply)
        for key in ("m", "k", "d"):
            self.root.unbind(key)
        for (_label, hot, value) in _verdict_buttons(kind):
            self.root.bind(hot, lambda e, v=value: self.decide(v))

    def decide(self, verdict: str):
        self.decisions[str(self.idx)] = verdict
        self._save()
        self.next()

    def next(self):
        if self.idx < len(self.cases) - 1:
            self.idx += 1
            self.render()
        else:
            self.current.config(text="(last case — Save & Quit when done)")

    def back(self):
        if self.idx > 0:
            self.idx -= 1
            self.render()

    def quit(self):
        self._save()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", required=True, type=Path,
                    help="gt_verify dir with review_queue.json + review/ images")
    args = ap.parse_args()
    root = tk.Tk()
    app = ReviewApp(root, args.outdir)
    root.mainloop()
    print(f"decisions written: {app.dec_path}  ({len(app.decisions)} verdicts)")


if __name__ == "__main__":
    main()
