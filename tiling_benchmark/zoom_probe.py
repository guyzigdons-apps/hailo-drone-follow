"""Camera-agnostic model resolution probe.

For each selected GT target (person or vehicle), crop the source frame at
multiple sizes centered on the target's bbox, then run the HEF on each crop
(resized to the model's 640x480 input) and record whether the target was
re-detected.

Two regimes:

  - Downscale (no upscaling): crop_w >= 640 AND crop_h >= 480 — the crop
    contains the target plus surrounding context, resized DOWN to model
    input. This corresponds to real tiling where the cropped tile is larger
    than the model input.

  - Digital zoom (upscaling): crop_w < 640 OR crop_h < 480 — the source crop
    is smaller than model input, resized UP to 640x480. This corresponds to
    "zoom in" tiling where we crop a tight region around the target. There
    is a "zoom limit" beyond which detection accuracy degrades because the
    model sees a blurry upscaled patch.

For every (target, crop_size) pair we record:
  - effective object pixel height in model input (= target_h_src * 480/crop_h)
  - effective object pixel area in model input
  - was the target redetected (best IoU >= 0.3 in model output) and at what conf
  - the regime (downscale / zoom)

Output:
  - /tmp/zoom_probe_<class>.csv per class
  - /tmp/zoom_probe.png — combined recall + conf plot

NOTES:
  - The target is CENTERED in every crop → this is the BEST case.
    Real-world tiling places the target somewhere inside the tile and may
    clip it at boundaries; expect worse recall there. The assumption is
    that good dynamic tiling can approach the curve produced here.
  - "Distance to target" in the camera-agnostic sense is the bbox's source
    pixel size: a far target has fewer source pixels. We sweep the crop
    size to vary the effective model pixel size; the per-target curve
    isolates the model's pixel-size sensitivity from camera resolution.
"""
from __future__ import annotations

import argparse
import csv
import json
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

# Reuse the HefHandle / decode_nms_output scaffolding.
HERE = Path(__file__).resolve().parent
import sys
sys.path.insert(0, str(HERE))
from probe_phantom_hef import HefHandle, decode_nms_output, label_for  # noqa: E402

DEFAULT_VIDEO = "/home/giladn/Videos/Drone/Training/DJI_20260430103421_0010_D_rotated.MP4"
DEFAULT_GT = HERE / "pxt_runs" / "pxt_GT-12x9-25-multi.frames.json"
DEFAULT_HEF = "/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef"

MODEL_W, MODEL_H = 640, 480
MODEL_ASPECT = MODEL_W / MODEL_H  # 4:3

# Crop widths to sweep, in source pixels. Heights derived from MODEL_ASPECT.
# Mixed bag: native-ish baselines + tile-like + zoom regime.
DEFAULT_CROP_WIDTHS = [4512, 3008, 2256, 1920, 1504, 1280, 1024, 880, 752, 640,
                       560, 480, 400, 320, 240, 192, 160, 128, 96, 80, 64]

# IoU >= this in MODEL-INPUT-NORMALIZED coords counts as a re-detection.
IOU_THRESH = 0.3


@dataclass
class Target:
    frame_idx: int
    cls_label: str        # 'person' | 'vehicle'
    cls_id: int           # HEF class id (0=person, 1=vehicle in 4-class HEF)
    src_bbox: tuple       # (x, y, w, h) NORMALIZED to source frame
    src_w: int
    src_h: int
    gt_conf: float
    bin_label: str        # size bin label for stratification


def src_size_bin(src_h_px: float) -> str:
    """Stable size-bin label keyed on bbox height in source pixels."""
    if src_h_px <= 8:    return "(0-8]"
    if src_h_px <= 12:   return "(8-12]"
    if src_h_px <= 16:   return "(12-16]"
    if src_h_px <= 24:   return "(16-24]"
    if src_h_px <= 32:   return "(24-32]"
    if src_h_px <= 64:   return "(32-64]"
    if src_h_px <= 128:  return "(64-128]"
    if src_h_px <= 256:  return "(128-256]"
    return "(256+]"


def load_gt_targets(gt_path: Path, src_w: int, src_h: int,
                    source_downscale_factor: float = 1.0,
                    conf_min: float = 0.7,
                    edge_margin_frac: float = 0.05,
                    aspect_ok: tuple = (0.2, 5.0),
                    classes: tuple = ("person", "vehicle"),
                    ) -> list[Target]:
    """Load every GT target meeting quality gates (conf, not-edge, sane aspect).
    Returns a flat list — caller stratifies + samples.

    `source_downscale_factor` is the synthetic-small factor — when > 1, the
    effective source dims are src_w/N × src_h/N, and target src_h_px is N×
    smaller (= "drone is N× farther / camera is N× lower res")."""
    HEF_CLS_ID = {"person": 0, "vehicle": 1, "face": 2, "license_plate": 3}
    with gt_path.open() as f:
        doc = json.load(f)
    eff_src_w = int(src_w / source_downscale_factor)
    eff_src_h = int(src_h / source_downscale_factor)
    out: list[Target] = []
    for fr in doc.get("frames", []):
        for det in fr.get("detections", []):
            lbl = det.get("label")
            if lbl not in classes:
                continue
            conf = float(det.get("confidence", 0.0))
            if conf < conf_min:
                continue
            x, y, w, h = det["bbox"]
            if w <= 0 or h <= 0:
                continue
            # Edge margin — keep targets fully inside the frame with breathing
            # room so larger crops centred on them still fit.
            if x < edge_margin_frac or y < edge_margin_frac:
                continue
            if x + w > 1.0 - edge_margin_frac or y + h > 1.0 - edge_margin_frac:
                continue
            ar = (w * eff_src_w) / (h * eff_src_h) if h > 0 else 0
            if not (aspect_ok[0] <= ar <= aspect_ok[1]):
                continue
            # Bin on effective source-pixel height.
            src_h_px = h * eff_src_h
            out.append(Target(
                frame_idx=fr["frame"],
                cls_label=lbl,
                cls_id=HEF_CLS_ID[lbl],
                src_bbox=(x, y, w, h),
                src_w=eff_src_w, src_h=eff_src_h,
                gt_conf=conf,
                bin_label=src_size_bin(src_h_px),
            ))
    return out


def stratified_sample(targets: list[Target], per_bin: int,
                      rng: random.Random) -> list[Target]:
    """Pick up to `per_bin` targets from each (class, size-bin) bucket."""
    buckets: dict[tuple, list[Target]] = {}
    for t in targets:
        key = (t.cls_label, t.bin_label)
        buckets.setdefault(key, []).append(t)
    picked: list[Target] = []
    for key, lst in sorted(buckets.items()):
        rng.shuffle(lst)
        picked.extend(lst[:per_bin])
    return picked


def crop_for_size(src_w: int, src_h: int, target_bbox_norm: tuple,
                  crop_w: int) -> Optional[tuple]:
    """Return (cx, cy, cw, ch) in source-pixel coords for a 4:3 crop of width
    crop_w (px), centred on the target bbox centre, clipped to source. Returns
    None if the requested crop doesn't fit in source."""
    cw = int(crop_w)
    ch = int(round(cw / MODEL_ASPECT))
    if cw > src_w or ch > src_h:
        return None  # crop bigger than source — would require padding/upscale
    tx, ty, tw, th = target_bbox_norm
    cx_centre = (tx + tw / 2.0) * src_w
    cy_centre = (ty + th / 2.0) * src_h
    cx = int(round(cx_centre - cw / 2.0))
    cy = int(round(cy_centre - ch / 2.0))
    cx = max(0, min(src_w - cw, cx))
    cy = max(0, min(src_h - ch, cy))
    # Bbox must still be fully inside crop after clipping.
    bx_src = tx * src_w
    by_src = ty * src_h
    bw_src = tw * src_w
    bh_src = th * src_h
    if not (cx <= bx_src and cy <= by_src and
            cx + cw >= bx_src + bw_src and cy + ch >= by_src + bh_src):
        return None
    return (cx, cy, cw, ch)


def iou_xyxy(a: tuple, b: tuple) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    ub = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = ua + ub - inter
    return inter / union if union > 0 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", default=DEFAULT_VIDEO)
    ap.add_argument("--gt", default=str(DEFAULT_GT))
    ap.add_argument("--hef", default=DEFAULT_HEF)
    ap.add_argument("--out-csv", default="/tmp/zoom_probe.csv")
    ap.add_argument("--out-png", default="/tmp/zoom_probe.png")
    ap.add_argument("--per-bin", type=int, default=4,
                    help="Targets per (class, size-bin)")
    ap.add_argument("--conf-min-gt", type=float, default=0.5,
                    help="Keep GT targets with confidence >= this (small "
                         "targets often have lower GT conf because they were "
                         "only seen by the dense-tile branch of the GT)")
    ap.add_argument("--seed", type=int, default=17)
    ap.add_argument("--crop-widths", type=int, nargs="+",
                    default=DEFAULT_CROP_WIDTHS)
    ap.add_argument("--nms-thresh", type=float, default=0.05,
                    help="On-chip NMS score threshold")
    ap.add_argument("--detect-conf-min", type=float, default=0.25,
                    help="Drop detections below this conf when matching")
    ap.add_argument("--source-downscale-factor", type=float, default=1.0,
                    help="Pre-downscale every source frame by this factor "
                         "(INTER_AREA). Use to synthesize very-small-src-px "
                         "targets — e.g. N=4 turns a 40-src-px person in the "
                         "original 6K frame into an effective 10-src-px "
                         "target at 1504x846 source resolution. The whole "
                         "probe then runs on the downscaled source.")
    args = ap.parse_args()

    print(f"video: {args.video}")
    print(f"gt:    {args.gt}")
    print(f"hef:   {args.hef}")

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"cannot open video: {args.video}")
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"source dims: {src_w}x{src_h}")

    rng = random.Random(args.seed)
    all_targets = load_gt_targets(Path(args.gt), src_w, src_h,
                                  conf_min=args.conf_min_gt)
    print(f"loaded {len(all_targets)} GT targets (conf>={args.conf_min_gt}, "
          f"not edge, sane aspect)")
    targets = stratified_sample(all_targets, args.per_bin, rng)
    by_bin: dict = {}
    for t in targets:
        by_bin.setdefault((t.cls_label, t.bin_label), 0)
        by_bin[(t.cls_label, t.bin_label)] += 1
    print(f"sampled {len(targets)} targets:")
    for k, n in sorted(by_bin.items()):
        print(f"   {k[0]:<8s} {k[1]:<24s}  n={n}")

    handle = HefHandle.open(args.hef, nms_score_threshold=args.nms_thresh)
    label_for_cls = lambda cls: label_for(Path(args.hef).name, cls)

    # Cache: only read each frame once.
    frame_cache: dict[int, np.ndarray] = {}

    def get_frame(idx: int) -> Optional[np.ndarray]:
        if idx in frame_cache:
            return frame_cache[idx]
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, fr = cap.read()
        if not ok or fr is None:
            return None
        if len(frame_cache) > 50:
            # bounded; least-recently-added eviction
            frame_cache.pop(next(iter(frame_cache)))
        frame_cache[idx] = fr
        return fr

    rows = []
    for ti, t in enumerate(targets):
        frame_bgr = get_frame(t.frame_idx)
        if frame_bgr is None:
            print(f"  frame {t.frame_idx}: read failed; skip")
            continue
        # Target bbox in source pixels.
        tx, ty, tw, th = t.src_bbox
        bx1 = tx * src_w; by1 = ty * src_h
        bx2 = (tx + tw) * src_w; by2 = (ty + th) * src_h
        src_h_px = th * src_h
        src_w_px = tw * src_w
        src_area_px = src_w_px * src_h_px

        for cw in args.crop_widths:
            crop = crop_for_size(src_w, src_h, t.src_bbox, cw)
            if crop is None:
                continue
            cx, cy, ch_w, ch_h = crop
            crop_bgr = frame_bgr[cy:cy + ch_h, cx:cx + ch_w]
            # Choose interpolation by regime — downscale uses INTER_AREA
            # (box-average, sharper) and upscale uses INTER_LINEAR (the
            # default GStreamer videoscale also uses linear, so this
            # mirrors what the on-chip pipeline would do).
            regime = "downscale" if (ch_w >= MODEL_W and ch_h >= MODEL_H) else "digital_zoom"
            interp = cv2.INTER_AREA if regime == "downscale" else cv2.INTER_LINEAR
            resized_bgr = cv2.resize(crop_bgr, (MODEL_W, MODEL_H),
                                     interpolation=interp)
            resized_rgb = cv2.cvtColor(resized_bgr, cv2.COLOR_BGR2RGB)

            nms_out = handle.infer(resized_rgb)
            dets = decode_nms_output(nms_out)

            # Map our GT target into MODEL-INPUT-NORMALIZED coords for IoU.
            # Crop coords: subtract crop origin, scale by crop dims.
            gx1_crop = (bx1 - cx) / ch_w
            gy1_crop = (by1 - cy) / ch_h
            gx2_crop = (bx2 - cx) / ch_w
            gy2_crop = (by2 - cy) / ch_h
            gt_box_model = (gx1_crop, gy1_crop, gx2_crop, gy2_crop)

            # Best matching detection of the right class.
            best_iou = 0.0; best_conf = 0.0
            for d in dets:
                if d.score < args.detect_conf_min:
                    continue
                if d.cls != t.cls_id:
                    continue
                db = (d.x, d.y, d.x + d.w, d.y + d.h)
                iou = iou_xyxy(gt_box_model, db)
                if iou > best_iou:
                    best_iou = iou
                    best_conf = d.score
            detected = best_iou >= IOU_THRESH

            # Effective pixel size in MODEL INPUT.
            scale = MODEL_W / ch_w  # == MODEL_H / ch_h for 4:3 crop
            model_h_px = src_h_px * scale
            model_w_px = src_w_px * scale
            model_area_px = model_w_px * model_h_px

            rows.append({
                "target_id": ti,
                "frame": t.frame_idx,
                "cls": t.cls_label,
                "bin": t.bin_label,
                "src_w_px": round(src_w_px, 1),
                "src_h_px": round(src_h_px, 1),
                "src_area_px": round(src_area_px, 1),
                "crop_w": ch_w,
                "crop_h": ch_h,
                "regime": regime,
                "scale": round(scale, 4),
                "model_w_px": round(model_w_px, 1),
                "model_h_px": round(model_h_px, 1),
                "model_area_px": round(model_area_px, 1),
                "best_iou": round(best_iou, 3),
                "best_conf": round(best_conf, 3),
                "detected": int(detected),
            })
        if (ti + 1) % 5 == 0:
            print(f"  done {ti+1}/{len(targets)} targets")

    handle.close()
    cap.release()

    # Write CSV.
    out_csv = Path(args.out_csv)
    if rows:
        with out_csv.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"wrote {out_csv} ({len(rows)} rows)")
    else:
        print("no rows collected; nothing to plot")
        return

    # Plot: recall vs model_h_px, bucketed; per class; downscale vs zoom marked.
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"matplotlib unavailable ({e}); skipping plot")
        return

    classes = sorted({r["cls"] for r in rows})

    # --- Rescue analysis ------------------------------------------------
    # "native_1to1" = scale ≈ 1.0 (crop_w = 640): the model sees the
    # target at its true source-pixel size. This is the "src-px = model-px"
    # reference the user asked for. Anything below this is downscale;
    # anything above is digital zoom.
    by_target: dict = {}
    for r in rows:
        by_target.setdefault(int(r["target_id"]), []).append(r)

    print("\n=== Rescue analysis ===")
    print("Comparing recall at scale=1.0 (model sees target at src-px size) "
          "vs best result with digital zoom (scale > 1).")
    print(f"\n{'cls':<8} {'src bin (src-px h)':>22} {'n':>4} "
          f"{'native_1to1_det%':>18} {'zoom_best_det%':>16} "
          f"{'rescued_by_zoom%':>18}")
    bins: dict = {}
    for tid, lst in sorted(by_target.items()):
        cls = lst[0]["cls"]
        src_h = lst[0]["src_h_px"]
        bins.setdefault((cls, src_size_bin(src_h)), []).append(lst)
    for k in sorted(bins):
        lists = bins[k]
        n = len(lists)
        native_det = 0
        zoom_det = 0
        rescued = 0
        for lst in lists:
            # native_1to1 = the row whose scale is closest to 1.0
            native_row = min(lst, key=lambda r: abs(r["scale"] - 1.0))
            nd = native_row["detected"]
            native_det += nd
            # zoom_best = any zoom row (scale > 1.05 to exclude the 1.0
            # boundary) detected.
            zoom_rows = [r for r in lst if r["scale"] > 1.05]
            zd = max((r["detected"] for r in zoom_rows), default=0)
            zoom_det += zd
            if not nd and zd:
                rescued += 1
        print(f"{k[0]:<8} {k[1]:>22} {n:>4} "
              f"{100*native_det/n:>17.1f}% {100*zoom_det/n:>15.1f}% "
              f"{100*rescued/n:>17.1f}%")
    # Need src_size_bin in the plot section below.
    src_bin = src_size_bin

    # Bucket on log-spaced model_h_px bins.
    bin_edges = [0, 8, 12, 16, 20, 24, 28, 32, 40, 48, 60, 80, 100, 130, 170, 220, 280, 360, 480]

    # 3 rows of panels:
    #   row 0: recall vs model_h_px, downscale vs zoom (the original chart)
    #   row 1: mean confidence (when detected) vs model_h_px
    #   row 2: zoom-rescue — recall vs SCALE (= 640/crop_w), one line per
    #          src-px size bin, holding the same set of targets and varying
    #          the crop size = zoom factor.
    fig, axes = plt.subplots(3, 2, figsize=(14, 13))
    for col_i, cls in enumerate(classes):
        ax_rec = axes[0, col_i]
        ax_conf = axes[1, col_i]
        ax_resc = axes[2, col_i]
        for regime in ("downscale", "digital_zoom"):
            xs = []; rec = []; cnf = []; ns = []
            for lo, hi in zip(bin_edges[:-1], bin_edges[1:]):
                sub = [r for r in rows if r["cls"] == cls
                       and r["regime"] == regime
                       and lo < r["model_h_px"] <= hi]
                if not sub:
                    continue
                recall = sum(r["detected"] for r in sub) / len(sub)
                # conf averaged only over detected samples (otherwise zeros
                # from missed detections drag conf down even at sizes the
                # model can clearly find).
                det_confs = [r["best_conf"] for r in sub if r["detected"]]
                meanconf = (sum(det_confs) / len(det_confs)) if det_confs else 0.0
                xs.append((lo + hi) / 2.0)
                rec.append(recall * 100.0)
                cnf.append(meanconf)
                ns.append(len(sub))
            if not xs:
                continue
            style = "-o" if regime == "downscale" else "--s"
            ax_rec.plot(xs, rec, style, label=f"{regime} (n_total={sum(ns)})")
            ax_conf.plot(xs, cnf, style, label=f"{regime} (n_total={sum(ns)})")

        for ax, ylabel, ylim in [(ax_rec, "Recall %", (-2, 105)),
                                 (ax_conf, "Mean conf (when detected)", (0.0, 1.0))]:
            ax.set_xscale("log")
            ax.set_xlabel("Object height in MODEL INPUT (px) — log scale")
            ax.set_ylabel(ylabel)
            ax.set_ylim(*ylim)
            ax.grid(True, which="both", alpha=0.3)
            ax.legend(loc="lower right")
        ax_rec.set_title(f"Recall — {cls}")
        ax_conf.set_title(f"Mean confidence — {cls}")
        ax_rec.axhline(80, color="gray", linestyle=":", alpha=0.4)
        ax_conf.axhline(0.5, color="gray", linestyle=":", alpha=0.4)

        # --- Row 2: zoom-rescue per src-px bin -----------------
        # For each src_h bin, plot recall vs zoom-scale (= 640/crop_w).
        # Each line: same set of targets at varying zoom factors.
        scale_bin_edges = [0.10, 0.16, 0.25, 0.40, 0.60, 0.90, 1.10, 1.5,
                           2.0, 2.5, 3.5, 5.0, 7.5, 11.0]
        src_labels = ["(0-8]", "(8-12]", "(12-16]", "(16-24]", "(24-32]",
                      "(32-64]", "(64-128]", "(128-256]"]
        for src_label in src_labels:
            xs = []; ys = []; ns = []
            for s_lo, s_hi in zip(scale_bin_edges[:-1], scale_bin_edges[1:]):
                sub = [r for r in rows if r["cls"] == cls
                       and src_bin(r["src_h_px"]) == src_label
                       and s_lo < r["scale"] <= s_hi]
                if not sub:
                    continue
                xs.append((s_lo + s_hi) / 2.0)
                ys.append(100.0 * sum(r["detected"] for r in sub) / len(sub))
                ns.append(len(sub))
            if not xs:
                continue
            ax_resc.plot(xs, ys, "-o",
                         label=f"src h {src_label}  (n_total={sum(ns)})")
        ax_resc.set_xscale("log")
        ax_resc.set_xlabel("Zoom scale (= 640 / crop_w);  s<1=downscale, "
                          "s>1=digital zoom")
        ax_resc.set_ylabel("Recall %")
        ax_resc.set_ylim(-2, 105)
        ax_resc.set_title(f"Zoom rescue — recall vs zoom factor — {cls}")
        ax_resc.grid(True, which="both", alpha=0.3)
        ax_resc.axhline(80, color="gray", linestyle=":", alpha=0.4)
        ax_resc.axvline(1.0, color="black", linestyle=":", alpha=0.5)
        ax_resc.legend(loc="lower right", fontsize=8)

    fig.suptitle("Camera-agnostic model resolution requirement\n"
                 "(target centred in crop = best case; "
                 "real-world tiling will be worse near tile boundaries)",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out_png, dpi=120)
    print(f"wrote {args.out_png}")


if __name__ == "__main__":
    main()
