"""Click/drag to select the target on a video frame -> normalized seed bbox.

Manual target selection for the showcase (mirrors how the final app lets a user
pick the target). Opens the chosen frame in a window; drag a box around the
target, press ENTER/SPACE to confirm (c to cancel). Prints the run_showcase seed
flags and, with --save, records them in a per-clip seeds JSON.

    # window shows on the laptop's monitor, so set DISPLAY:
    DISPLAY=:0 python -m tiling_lab.live.pick_target \
        --video /home/giladn/Videos/Drone/Training/Car/DJI_..._0007_D.MP4 \
        --frame 350 --save tiling_lab/runs/seeds.json

Then:
    python -m tiling_lab.live.run_showcase --video <same> --out <dir> \
        --init-bbox <x,y,w,h> --init-frame <N>
"""
import argparse
import json
import os
import sys

import cv2

DISPLAY_MAX_W = 1920


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--frame", type=int, default=0,
                    help="frame to pick the target on (seed is applied here).")
    ap.add_argument("--save", default=None,
                    help="append the pick to this seeds JSON (keyed by clip name).")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        sys.exit(f"ERROR: cannot open {args.video}")
    cap.set(cv2.CAP_PROP_POS_FRAMES, args.frame)
    ok, img = cap.read()
    cap.release()
    if not ok:
        sys.exit(f"ERROR: cannot read frame {args.frame}")

    full_h, full_w = img.shape[:2]
    scale = min(1.0, DISPLAY_MAX_W / full_w)
    disp = cv2.resize(img, (int(full_w * scale), int(full_h * scale))) \
        if scale < 1.0 else img
    dh, dw = disp.shape[:2]

    win = f"pick target @frame {args.frame} - drag box, ENTER=confirm, c=cancel"
    x, y, w, h = cv2.selectROI(win, disp, showCrosshair=True, fromCenter=False)
    cv2.destroyAllWindows()
    if w == 0 or h == 0:
        sys.exit("no selection made")

    # selectROI returns display-pixel coords; normalize by display dims.
    nx, ny, nw, nh = x / dw, y / dh, w / dw, h / dh
    bbox = f"{nx:.4f},{ny:.4f},{nw:.4f},{nh:.4f}"
    print(f"\nselected (normalized): {bbox}  @frame {args.frame}")
    print(f"run_showcase flags:  --init-bbox {bbox} --init-frame {args.frame}")

    if args.save:
        clip = os.path.basename(args.video)
        data = {}
        if os.path.isfile(args.save):
            data = json.loads(open(args.save, encoding="utf-8").read() or "{}")
        data.setdefault(clip, []).append(
            {"bbox": [round(nx, 4), round(ny, 4), round(nw, 4), round(nh, 4)],
             "frame": args.frame})
        with open(args.save, "w", encoding="utf-8") as fp:
            json.dump(data, fp, indent=2)
        print(f"saved to {args.save} under '{clip}'")


if __name__ == "__main__":
    main()
