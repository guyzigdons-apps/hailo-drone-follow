"""On-the-fly overlay viewer for pxt sweep frames.json files.

Reads the source video frame-by-frame; for each frame, looks up the
detections from one or more frames.json files (matched by frame index) and
draws them with a per-file colour. Press SPACE to pause, Q to quit, [/] to
step backward/forward when paused, +/- to change playback speed.

Usage:
  python bench/overlay_dets.py \\
      --video /home/giladn/Videos/Drone/Training/DJI_20260430103421_0010_D.MP4 \\
      --frames /home/giladn/Videos/Drone/Training/pxt_runs/pxt_GT-12x9-25.frames.json:GT \\
      --frames /home/giladn/Videos/Drone/Training/pxt_runs/pxt_3x2-native.frames.json:3x2

The `:label` suffix is optional; defaults to the JSON's own label field or
the filename stem if absent.
"""

import argparse
import json
from pathlib import Path

import cv2

# Distinct BGR colours — keep small so the legend stays readable.
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


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--video", type=Path, required=True)
    ap.add_argument("--frames", action="append", required=True,
                    help="frames.json path, optionally suffixed with ':LABEL'. "
                         "Repeat for multiple overlays.")
    ap.add_argument("--start-frame", type=int, default=0)
    ap.add_argument("--conf-min", type=float, default=0.25,
                    help="Don't draw boxes below this confidence (default 0.25).")
    args = ap.parse_args(argv)

    overlays = []
    for raw in args.frames:
        p, lbl_override = parse_frames_arg(raw)
        if not p.is_file():
            print(f"ERROR: frames file not found: {p}")
            return 1
        lbl, idx = load_frames_indexed(p)
        if lbl_override:
            lbl = lbl_override
        overlays.append((lbl, idx))

    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        print(f"ERROR: cannot open video {args.video}")
        return 1

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    base_delay_ms = max(1, int(1000.0 / fps))
    delay_ms = base_delay_ms
    paused = False
    frame_no = args.start_frame
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_no)

    print(f"Video: {total} frames @ {fps:.2f} fps. "
          f"Loaded {len(overlays)} overlay(s). "
          f"Controls: SPACE=pause, Q=quit, [/]=step (paused), +/-=speed.")
    print("Legend:")
    for i, (lbl, _) in enumerate(overlays):
        c = PALETTE[i % len(PALETTE)]
        print(f"  {lbl}: BGR{c}")

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok:
                print("End of video.")
                break
        else:
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_no)
            ok, frame = cap.read()
            if not ok:
                break

        h, w = frame.shape[:2]
        for i, (lbl, idx) in enumerate(overlays):
            colour = PALETTE[i % len(PALETTE)]
            for det in idx.get(frame_no, []):
                if float(det.get("confidence", 0.0)) < args.conf_min:
                    continue
                bx, by, bw, bh = det["bbox"]
                x1 = int(bx * w); y1 = int(by * h)
                x2 = int((bx + bw) * w); y2 = int((by + bh) * h)
                cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                tag = f"{det.get('label', '?')} {det.get('confidence', 0):.2f}"
                cv2.putText(frame, tag, (x1, max(15, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)

        # Legend / HUD overlay
        hud = f"frame {frame_no}/{total - 1}  speed={base_delay_ms / delay_ms:.2f}x  "
        hud += "PAUSED" if paused else "PLAY"
        cv2.putText(frame, hud, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2, cv2.LINE_AA)
        for i, (lbl, _) in enumerate(overlays):
            cv2.putText(frame, lbl, (10, 50 + 22 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        PALETTE[i % len(PALETTE)], 2, cv2.LINE_AA)

        # Fit-to-screen-ish: 6016x3384 won't fit; downscale for display only.
        if w > 1920:
            scale = 1920 / w
            disp = cv2.resize(frame, (int(w * scale), int(h * scale)))
        else:
            disp = frame
        cv2.imshow("pxt overlay", disp)

        key = cv2.waitKey(0 if paused else delay_ms) & 0xFF
        if key == ord('q') or key == 27:  # Q or ESC
            break
        elif key == ord(' '):
            paused = not paused
        elif paused and key == ord(']'):
            frame_no = min(total - 1, frame_no + 1)
        elif paused and key == ord('['):
            frame_no = max(0, frame_no - 1)
        elif key in (ord('+'), ord('=')):
            delay_ms = max(1, delay_ms // 2)
        elif key == ord('-'):
            delay_ms = min(2000, delay_ms * 2)

        if not paused:
            frame_no += 1

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
