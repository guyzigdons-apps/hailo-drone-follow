#!/usr/bin/env python3
"""Adapt new `hailo_tiling.bench` frames.json into the
`tiling_lab.viewer.overlay_viewer` schema.

The two halves of the project diverged in frames.json schema:

  new (hailo_tiling.bench)        viewer (tiling_lab.viewer.overlay_viewer)
  ------------------------        --------------------------
  frame_idx                       frame
  dets                            detections
  det.score                       det.confidence
  det.{x,y,w,h} (normalized)      det.bbox = [x,y,w,h] (normalized)
  det.cls (int)                   det.label (string) + cls kept
  config = "8x6" (name string)    config = {tiles_x, tiles_y, overlap_x/y}

Class ids use the authoritative hailo_4_classes.json mapping (leading
"unlabeled"; network emits 1..4). The new runner records only `n_tiles`
(a count), not the tile rectangles, so dynamic ROI tile positions cannot
be reconstructed; static grids are rebuilt from the `NxM` config name with
the benchmark's standard 0.25 overlap so the viewer's tile overlay works.
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

CLASS_LABELS = ("unlabeled", "person", "vehicle", "face", "license_plate")
STATIC_OVERLAP = 0.25  # the ablation static grid set uses overlap 0.25


def _label(cls: int) -> str:
    return CLASS_LABELS[cls] if 0 <= cls < len(CLASS_LABELS) else f"cls{cls}"


def _static_config(name: str) -> dict:
    """Rebuild a viewer config dict from an `NxM` config name (e.g. "8x6")."""
    m = re.fullmatch(r"(\d+)x(\d+)", name.strip())
    if not m:
        return {}
    return {
        "tiles_x": int(m.group(1)),
        "tiles_y": int(m.group(2)),
        "overlap_x": STATIC_OVERLAP,
        "overlap_y": STATIC_OVERLAP,
    }


def adapt(doc: dict) -> dict:
    cfg_name = doc.get("config")
    config = _static_config(cfg_name) if isinstance(cfg_name, str) else (cfg_name or {})
    out_frames = []
    for f in doc.get("frames", []):
        dets = []
        for d in f.get("dets", []):
            cls = int(d["cls"])
            dets.append({
                "bbox": [d["x"], d["y"], d["w"], d["h"]],
                "confidence": d.get("score", 0.0),
                "label": _label(cls),
                "cls": cls,
            })
        out = {"frame": f["frame_idx"], "detections": dets}
        # Pass through per-frame tile rectangles if the runner recorded them
        # (new schema) so the viewer draws the actual tile layout, incl.
        # dynamic ROI tiles. Falls back to config-reconstruction otherwise.
        if f.get("tiles"):
            out["tiles"] = [
                {"x": t["x"], "y": t["y"], "w": t["w"], "h": t["h"],
                 "category": t.get("category", "dynamic")}
                for t in f["tiles"]
            ]
        out_frames.append(out)
    return {
        "label": cfg_name if isinstance(cfg_name, str) else doc.get("label", "run"),
        "config": config,
        "frames": out_frames,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="adapt_frames_for_viewer")
    ap.add_argument("inputs", nargs="+", type=Path,
                    help="new-format frames.json file(s).")
    ap.add_argument("--out-dir", type=Path, default=Path("/tmp/viewer_frames"),
                    help="Where to write adapted frames.json files.")
    args = ap.parse_args(argv)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for inp in args.inputs:
        doc = json.loads(inp.read_text())
        out = args.out_dir / inp.name
        out.write_text(json.dumps(adapt(doc)))
        n = sum(len(fr["detections"]) for fr in adapt(doc)["frames"])
        print(f"{inp.name}: {len(doc.get('frames', []))} frames, {n} dets -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
