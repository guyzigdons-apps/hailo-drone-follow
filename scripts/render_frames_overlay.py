#!/usr/bin/env python3
"""Draw per-frame detection boxes from a `*.frames.json` onto the source video.

The ablation runner writes one `<config>.frames.json` per config under
`dynamic_tiling/runs/<run>/`, each carrying
`frames: [{frame_idx, n_tiles, dets: [{cls, score, x, y, w, h}]}]` with
**normalized source coordinates** (top-left x/y + w/h, fractions of frame).

This is a research/inspection tool only (not the production overlay). It can
emit either selected annotated stills (`--frames 100,400`) or a full annotated
MP4 (`--video-out path.mp4`). A class allowlist (`--classes`) filters which
detections are drawn; discarded classes can optionally be drawn dimmed
(`--show-discarded`) so you can see what the allowlist removes.

Class id -> label uses the authoritative `hailo_4_classes.json` mapping, which
has a leading "unlabeled" entry (the network never emits class 0):
    0 unlabeled, 1 person, 2 vehicle, 3 face, 4 license_plate
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2

# Authoritative mapping from /usr/local/hailo/resources/json/hailo_4_classes.json
# (leading "unlabeled"; the network emits ids 1..4).
CLASS_LABELS = ("unlabeled", "person", "vehicle", "face", "license_plate")
# BGR colors per class.
CLASS_COLORS = {
    0: (180, 180, 180),  # unlabeled -> grey
    1: (0, 255, 0),      # person    -> green
    2: (255, 128, 0),    # vehicle   -> blue-ish
    3: (0, 165, 255),    # face      -> orange
    4: (0, 0, 255),      # plate     -> red
}
DIM_COLOR = (110, 110, 110)


def _label(cls: int) -> str:
    return CLASS_LABELS[cls] if 0 <= cls < len(CLASS_LABELS) else f"cls{cls}"


def _load_frames(frames_json: Path) -> list[dict]:
    d = json.loads(frames_json.read_text())
    return d["frames"] if isinstance(d, dict) and "frames" in d else d


def _draw(img, dets, W, H, keep: set[int], show_discarded: bool):
    """Draw boxes on `img` (mutated in place). Returns (n_kept, n_dropped)."""
    n_kept = n_drop = 0
    for d in dets:
        cls = int(d["cls"])
        kept = cls in keep
        if not kept and not show_discarded:
            n_drop += 1
            continue
        x = int(round(d["x"] * W))
        y = int(round(d["y"] * H))
        w = int(round(d["w"] * W))
        h = int(round(d["h"] * H))
        if kept:
            color = CLASS_COLORS.get(cls, (255, 255, 255))
            thick = 3
            n_kept += 1
        else:
            color = DIM_COLOR
            thick = 1
            n_drop += 1
        cv2.rectangle(img, (x, y), (x + w, y + h), color, thick)
        if kept:
            txt = f"{_label(cls)} {d.get('score', 0):.2f}"
            cv2.putText(img, txt, (x, max(0, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2, cv2.LINE_AA)
    return n_kept, n_drop


def _frame_at(cap, idx: int):
    cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
    ok, img = cap.read()
    return img if ok else None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="render_frames_overlay")
    ap.add_argument("--frames-json", type=Path, required=True)
    ap.add_argument("--video", type=Path, required=True)
    ap.add_argument("--classes", default="1,2",
                    help="Comma class-id allowlist to draw solid. Default 1,2 "
                         "(person,vehicle); face=3, license_plate=4 dropped.")
    ap.add_argument("--show-discarded", action="store_true",
                    help="Also draw non-allowlisted classes, dimmed/thin.")
    ap.add_argument("--frames", default=None,
                    help="Comma frame indices to dump as stills.")
    ap.add_argument("--stills-out", type=Path, default=Path("/tmp/overlay"),
                    help="Directory for still PNGs.")
    ap.add_argument("--video-out", type=Path, default=None,
                    help="If set, write a full annotated MP4 here.")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="Output scale factor (e.g. 0.5 for half-res).")
    args = ap.parse_args(argv)

    keep = {int(c) for c in args.classes.split(",") if c.strip() != ""}
    frames = _load_frames(args.frames_json)
    by_idx = {f["frame_idx"]: f for f in frames}

    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        print(f"error: cannot open {args.video}", file=sys.stderr)
        return 2
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    if args.frames:
        args.stills_out.mkdir(parents=True, exist_ok=True)
        for idx in (int(x) for x in args.frames.split(",")):
            img = _frame_at(cap, idx)
            if img is None:
                print(f"warn: no video frame {idx}", file=sys.stderr)
                continue
            dets = by_idx.get(idx, {}).get("dets", [])
            nk, nd = _draw(img, dets, W, H, keep, args.show_discarded)
            if args.scale != 1.0:
                img = cv2.resize(img, None, fx=args.scale, fy=args.scale)
            suffix = "all" if args.show_discarded else "kept"
            out = args.stills_out / f"{args.frames_json.stem}_f{idx:05d}_{suffix}.png"
            cv2.imwrite(str(out), img)
            print(f"frame {idx}: {nk} kept, {nd} discarded -> {out}")
        cap.release()
        return 0

    if args.video_out:
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        outW = int(W * args.scale)
        outH = int(H * args.scale)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        vw = cv2.VideoWriter(str(args.video_out), fourcc, fps, (outW, outH))
        n = 0
        while True:
            ok, img = cap.read()
            if not ok:
                break
            dets = by_idx.get(n, {}).get("dets", [])
            _draw(img, dets, W, H, keep, args.show_discarded)
            if args.scale != 1.0:
                img = cv2.resize(img, (outW, outH))
            vw.write(img)
            n += 1
        vw.release()
        cap.release()
        print(f"wrote {n} frames -> {args.video_out}")
        return 0

    print("error: pass --frames (stills) or --video-out (mp4).", file=sys.stderr)
    cap.release()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
