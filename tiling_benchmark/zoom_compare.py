"""Apples-to-apples zoom-factor comparison plot.

Reads a zoom_probe_*.csv and produces a plot where:

  - X-axis = SOURCE pixel height of the target (`src_h_px`). Same axis
    for every curve — same real-world question across zoom factors.
  - Y-axis = recall %.
  - One curve per discrete zoom factor (the cropper `scale` rounded
    to the nearest integer): x1 (native, no rescale), x2, x3, x4, ...

The native curve shows what the model achieves when the source pixels
go straight into the model input with no rescaling (crop_w = 640).
Each zoom-x curve shows what happens when we crop tighter around the
target and upscale to the model input — for a fixed real-world target
size, this exposes whether x2 is "good enough" or whether x3 / x4
gives meaningfully better recall.

Use:
    python zoom_compare.py \
        --csv pxt_runs/zoom_probe_720p_factors.csv \
        --out pxt_runs/zoom_compare_720p.png
"""
from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


# src_h_px bins on the x-axis (in source pixels).
SRC_BIN_EDGES = [0, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256, 384, 512]


def bucket_scale(scale: float) -> str | None:
    """Round to a nominal zoom factor; return None if not on the grid."""
    # Allow ±5% jitter around each integer factor.
    nominal = [1, 2, 3, 4, 5]
    for n in nominal:
        if abs(scale - n) / max(n, 1) <= 0.05:
            return f"x{n}"
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--title-suffix", default="",
                    help="Appended to the figure title — e.g. '(720p source)'.")
    args = ap.parse_args()

    rows = list(csv.DictReader(args.csv.open()))
    print(f"loaded {len(rows)} rows from {args.csv}")

    # Group: (cls, scale_factor, src_bin) -> [detected,...]
    grp: dict = defaultdict(list)
    for r in rows:
        cls = r["cls"]
        s = float(r["scale"])
        sb = bucket_scale(s)
        if sb is None:
            continue
        h = float(r["src_h_px"])
        # Bin by src_h
        bin_label = None
        for lo, hi in zip(SRC_BIN_EDGES[:-1], SRC_BIN_EDGES[1:]):
            if lo < h <= hi:
                bin_label = (lo, hi)
                break
        if bin_label is None:
            continue
        grp[(cls, sb, bin_label)].append(int(r["detected"]))

    # Style: native = bold black; zoom curves go light → dark blue.
    zoom_colors = {
        "x1": "#000000",      # native, black
        "x2": "#9ec5fe",      # light blue
        "x3": "#4292c6",
        "x4": "#08519c",
        "x5": "#08306b",      # darkest blue
    }
    zoom_order = ["x1", "x2", "x3", "x4", "x5"]

    classes = sorted({r["cls"] for r in rows})
    fig, axes = plt.subplots(1, len(classes), figsize=(14, 6),
                             sharex=True, sharey=True)
    if len(classes) == 1:
        axes = [axes]

    for ax_i, cls in enumerate(classes):
        ax = axes[ax_i]
        for sb in zoom_order:
            xs = []; ys = []; ns = []
            for lo, hi in zip(SRC_BIN_EDGES[:-1], SRC_BIN_EDGES[1:]):
                bucket = grp.get((cls, sb, (lo, hi)), [])
                if not bucket:
                    continue
                xs.append((lo + hi) / 2.0)
                ys.append(100.0 * sum(bucket) / len(bucket))
                ns.append(len(bucket))
            if not xs:
                continue
            label = f"native (x1)" if sb == "x1" else f"zoom {sb}"
            lw = 3.0 if sb == "x1" else 1.7
            ls = "-" if sb == "x1" else "-"
            marker = "o" if sb == "x1" else "."
            ax.plot(xs, ys, ls, color=zoom_colors[sb], lw=lw, marker=marker,
                    markersize=5 if sb == "x1" else 4,
                    label=f"{label}  (n={sum(ns)})")
        ax.set_xscale("log")
        ax.set_xlabel("Target height in SOURCE frame (src-px) — log scale")
        ax.set_ylabel("Recall %  (best-case: target centred in crop)")
        ax.set_title(f"{cls} — recall vs source-px size, one curve per zoom factor")
        ax.set_ylim(-2, 105)
        ax.set_xlim(5, 600)
        ax.grid(True, which="both", alpha=0.3)
        ax.axhline(80, color="gray", linestyle=":", alpha=0.4)
        ax.legend(loc="lower right", fontsize=9)

    fig.suptitle(f"Apples-to-apples zoom-factor comparison {args.title_suffix}\n"
                 f"(same X axis = real-world target size in source pixels; "
                 f"each curve is a fixed cropper zoom factor)",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
