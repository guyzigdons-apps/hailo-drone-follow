#!/usr/bin/env python3
"""
Standalone MOT (Multiple Object Tracking) evaluation tool.

Runs the Hailo detection pipeline + ByteTracker on a video file and outputs
tracking results in MOT Challenge format.  Optionally computes MOT metrics
(MOTA, MOTP, IDF1, …) when ground-truth annotations are provided.

Usage
-----
    # Run tracker on a video, write results to mot_out.txt
    mot-eval --input video.mp4 -o mot_out.txt

    # Evaluate against ground truth
    mot-eval --input video.mp4 --gt gt.txt -o mot_out.txt

    # Use pre-computed detections instead of Hailo (no HW needed)
    mot-eval --detections dets.txt -o mot_out.txt

MOT Challenge format (each line):
    <frame>,<id>,<bb_left>,<bb_top>,<bb_width>,<bb_height>,<conf>,<x>,<y>,<z>

Ground-truth format (same, but columns 7-9 are <flag>,<class>,<visibility>).
"""

from __future__ import annotations

import argparse
import csv
import logging
import os
import signal
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

LOGGER = logging.getLogger("mot_eval")


# ---------------------------------------------------------------------------
# MOT I/O helpers
# ---------------------------------------------------------------------------

def load_mot_file(path: str) -> dict[int, list[tuple]]:
    """Load a MOT Challenge format file.

    Returns {frame_number: [(id, x, y, w, h, conf), ...]}.
    """
    data: dict[int, list[tuple]] = defaultdict(list)
    with open(path) as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            frame = int(row[0])
            tid = int(row[1])
            x, y, w, h = float(row[2]), float(row[3]), float(row[4]), float(row[5])
            conf = float(row[6]) if len(row) > 6 else 1.0
            data[frame].append((tid, x, y, w, h, conf))
    return dict(data)


def write_mot_line(f, frame: int, tid: int, x: float, y: float,
                   w: float, h: float, conf: float):
    f.write(f"{frame},{tid},{x:.2f},{y:.2f},{w:.2f},{h:.2f},{conf:.4f},-1,-1,-1\n")


# ---------------------------------------------------------------------------
# Detections-only mode (no Hailo needed)
# ---------------------------------------------------------------------------

def run_from_detections(det_path: str, output_path: str,
                        gt_path: str | None, args: argparse.Namespace):
    """Run tracker on pre-computed detections in MOT format.

    Detection file format (per line):
        <frame>,<id_ignored>,<bb_left>,<bb_top>,<bb_width>,<bb_height>,<conf>
    """
    from drone_follow.pipeline_adapter.byte_tracker import ByteTrackerAdapter
    from drone_follow.pipeline_adapter.tracker import MetricsTracker

    raw = load_mot_file(det_path)
    if not raw:
        LOGGER.error("No detections found in %s", det_path)
        return

    t_init = time.monotonic()
    tracker = MetricsTracker(ByteTrackerAdapter(
        track_thresh=args.track_thresh,
        track_buffer=args.track_buffer,
        match_thresh=args.match_thresh,
        frame_rate=args.frame_rate,
    ), init_time_ms=(time.monotonic() - t_init) * 1000.0)

    frames = sorted(raw.keys())
    # Fill gaps so tracker sees empty frames
    all_frames = range(frames[0], frames[-1] + 1)

    results: dict[int, list[tuple]] = {}

    t_start = time.monotonic()
    with open(output_path, "w") as out:
        for frame_num in all_frames:
            dets = raw.get(frame_num, [])

            if dets:
                # Build Nx5 array [x1, y1, x2, y2, conf]
                det_array = np.empty((len(dets), 5), dtype=np.float32)
                for i, (_, x, y, w, h, conf) in enumerate(dets):
                    det_array[i] = [x, y, x + w, y + h, conf]
            else:
                det_array = np.empty((0, 5), dtype=np.float32)

            tracks = tracker.update(det_array)
            frame_results = []

            for t in tracks:
                if not t.is_activated:
                    continue
                # Get bbox from matched detection
                if 0 <= t.input_index < len(dets):
                    _, x, y, w, h, _ = dets[t.input_index]
                else:
                    continue
                write_mot_line(out, frame_num, t.track_id, x, y, w, h, t.score)
                frame_results.append((t.track_id, x, y, w, h, t.score))

            results[frame_num] = frame_results

    wall_time = time.monotonic() - t_start

    m = tracker.metrics
    LOGGER.info("Processed %d frames in %.2fs (%.1f fps), %d ID switches",
                m.total_frames, wall_time, m.total_frames / max(wall_time, 1e-6),
                m.id_switches)
    LOGGER.info("Results written to %s", output_path)

    # Derive sequence name from detection path (e.g. .../MOT17-04-SDP/det/det.txt → MOT17-04-SDP)
    seq_name = Path(det_path).resolve().parent.parent.name

    if gt_path:
        gt = load_mot_file(gt_path)
        _compute_metrics(results, gt, tracker_metrics=tracker,
                         wall_time=wall_time, seq_name=seq_name)
    else:
        _compute_metrics(results, {}, tracker_metrics=tracker,
                         wall_time=wall_time, seq_name=seq_name)


# ---------------------------------------------------------------------------
# Pipeline mode (Hailo detection + tracker)
# ---------------------------------------------------------------------------

def run_from_pipeline(output_path: str, gt_path: str | None,
                      args: argparse.Namespace, remaining_args: list[str]):
    """Run the full Hailo pipeline on a video, collect tracker output."""
    from drone_follow.follow_api import SharedDetectionState
    from drone_follow.follow_api.state import FollowTargetState

    shared_state = SharedDetectionState()
    target_state = FollowTargetState()
    eos_reached = threading.Event()

    from hailo_apps.python.core.common.core import get_pipeline_parser
    parser = get_pipeline_parser()
    # Inject remaining CLI args so the pipeline parser sees --input etc.
    _inject_args(remaining_args)

    from drone_follow.pipeline_adapter import create_app
    app = create_app(shared_state, target_state=target_state,
                     eos_reached=eos_reached, parser=parser)

    pipeline_args = app.options_menu
    video_w = getattr(pipeline_args, "video_width", 1920)
    video_h = getattr(pipeline_args, "video_height", 1080)

    # Hook into the tracker to capture per-frame results
    _hook_tracker_output(app, results={}, output_path=output_path,
                         video_w=video_w, video_h=video_h)

    def on_signal(*_):
        eos_reached.set()
        try:
            app.loop.quit()
        except Exception:
            pass

    signal.signal(signal.SIGINT, on_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, on_signal)

    def eos_handler():
        eos_reached.wait()
        try:
            app.loop.quit()
        except Exception:
            pass
    threading.Thread(target=eos_handler, daemon=True).start()

    LOGGER.info("Running pipeline on video... (Ctrl+C to stop)")
    t_start = time.monotonic()
    try:
        app.run()
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        eos_reached.set()
    wall_time = time.monotonic() - t_start

    LOGGER.info("Results written to %s", output_path)

    all_results = _read_back_results(output_path)
    tracker_metrics = getattr(shared_state, "tracker_metrics", None)
    gt = load_mot_file(gt_path) if gt_path else {}
    _compute_metrics(all_results, gt, tracker_metrics=tracker_metrics,
                     wall_time=wall_time,
                     seq_name=Path(output_path).stem)


def _inject_args(extra_args: list[str]):
    """Inject extra args into sys.argv for the pipeline parser."""
    # The pipeline parser reads sys.argv, so we append our extra args
    sys.argv = [sys.argv[0]] + extra_args


def _hook_tracker_output(app, results: dict, output_path: str,
                         video_w: int, video_h: int):
    """Monkey-patch the tracker to intercept per-frame results.

    We wrap tracker.update() to capture every frame's output and write
    MOT lines as they come in.
    """
    from drone_follow.pipeline_adapter import hailo_drone_detection_manager as mgr

    original_run_tracker = mgr._run_tracker
    out_file = open(output_path, "w")
    frame_counter = [0]
    lock = threading.Lock()

    SCALE = 1000.0  # same scale used in _run_tracker

    def patched_run_tracker(tracker, persons):
        available_ids, person_by_id, person_to_id = original_run_tracker(tracker, persons)

        with lock:
            frame_counter[0] += 1
            frame_num = frame_counter[0]

            for tid, person in person_by_id.items():
                bbox = person.get_bbox()
                # Convert from normalized [0,1] to pixel coordinates
                x = bbox.xmin() * video_w
                y = bbox.ymin() * video_h
                w = bbox.width() * video_w
                h = bbox.height() * video_h
                conf = person.get_confidence()
                write_mot_line(out_file, frame_num, tid, x, y, w, h, conf)

        return available_ids, person_by_id, person_to_id

    mgr._run_tracker = patched_run_tracker

    # Register cleanup
    import atexit
    atexit.register(out_file.close)


def _read_back_results(path: str) -> dict[int, list[tuple]]:
    return load_mot_file(path)


# ---------------------------------------------------------------------------
# MOT metrics
# ---------------------------------------------------------------------------

def _iou_matrix(gt_boxes: np.ndarray, pred_boxes: np.ndarray) -> np.ndarray:
    """Compute IoU between two sets of [x1, y1, x2, y2] boxes."""
    if len(gt_boxes) == 0 or len(pred_boxes) == 0:
        return np.zeros((len(gt_boxes), len(pred_boxes)))
    g = gt_boxes[:, None, :]  # (G, 1, 4)
    p = pred_boxes[None, :, :]  # (1, P, 4)
    xx1 = np.maximum(g[..., 0], p[..., 0])
    yy1 = np.maximum(g[..., 1], p[..., 1])
    xx2 = np.minimum(g[..., 2], p[..., 2])
    yy2 = np.minimum(g[..., 3], p[..., 3])
    inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
    area_g = (g[..., 2] - g[..., 0]) * (g[..., 3] - g[..., 1])
    area_p = (p[..., 2] - p[..., 0]) * (p[..., 3] - p[..., 1])
    return inter / np.maximum(area_g + area_p - inter, 1e-6)


def _match_frame(gt_boxes, pred_boxes, iou_thresh):
    """Hungarian matching at a given IoU threshold.

    Returns list of (gt_idx, pred_idx) matches.
    """
    from scipy.optimize import linear_sum_assignment

    if len(gt_boxes) == 0 or len(pred_boxes) == 0:
        return []
    iou = _iou_matrix(gt_boxes, pred_boxes)
    cost = 1.0 - iou
    row, col = linear_sum_assignment(cost)
    return [(r, c) for r, c in zip(row, col) if iou[r, c] >= iou_thresh]


def _to_xyxy(dets: list[tuple]) -> np.ndarray:
    """Convert list of (id, x, y, w, h, conf) to Nx4 [x1,y1,x2,y2]."""
    if not dets:
        return np.empty((0, 4))
    return np.array([[d[1], d[2], d[1] + d[3], d[2] + d[4]] for d in dets])


def _compute_hota(results: dict[int, list[tuple]],
                  gt: dict[int, list[tuple]]) -> dict:
    """Compute HOTA, DetA, and AssA averaged over IoU thresholds 0.05–0.95."""
    all_frames = sorted(set(list(gt.keys()) + list(results.keys())))
    alphas = np.arange(0.05, 1.0, 0.05)

    hota_per_alpha = []
    deta_per_alpha = []
    assa_per_alpha = []

    for alpha in alphas:
        total_tp = 0
        total_fp = 0
        total_fn = 0

        # For AssA: track per-TP-pair co-occurrence across frames
        # Key: (gt_id, pred_id), value: count of frames matched together
        pair_tpa: dict[tuple, int] = defaultdict(int)
        # For each gt_id: total frames it appears as TP (matched to anything)
        gt_tp_frames: dict[int, int] = defaultdict(int)
        # For each pred_id: total frames it appears as TP
        pred_tp_frames: dict[int, int] = defaultdict(int)

        for frame in all_frames:
            gt_dets = gt.get(frame, [])
            pred_dets = results.get(frame, [])

            gt_boxes = _to_xyxy(gt_dets)
            pred_boxes = _to_xyxy(pred_dets)

            matches = _match_frame(gt_boxes, pred_boxes, alpha)
            tp = len(matches)
            fp = len(pred_dets) - tp
            fn = len(gt_dets) - tp
            total_tp += tp
            total_fp += fp
            total_fn += fn

            for gi, pi in matches:
                gt_id = gt_dets[gi][0]
                pred_id = pred_dets[pi][0]
                pair_tpa[(gt_id, pred_id)] += 1
                gt_tp_frames[gt_id] += 1
                pred_tp_frames[pred_id] += 1

        # DetA
        deta = total_tp / max(total_tp + total_fp + total_fn, 1)

        # AssA: for each TP association (gt_id, pred_id), compute
        # TPA / (TPA + FPA + FNA) where:
        #   TPA = frames this exact pair matched
        #   FPA = frames pred_id was TP but matched to different gt
        #   FNA = frames gt_id was TP but matched to different pred
        if pair_tpa:
            ass_scores = []
            for (gt_id, pred_id), tpa in pair_tpa.items():
                fpa = pred_tp_frames[pred_id] - tpa
                fna = gt_tp_frames[gt_id] - tpa
                ass_scores.append(tpa / max(tpa + fpa + fna, 1))
            # Weight by TPA count (each TP instance contributes equally)
            total_tpa = sum(pair_tpa.values())
            assa = sum(s * c for s, (_, c) in zip(ass_scores, pair_tpa.items())) / max(total_tpa, 1)
        else:
            assa = 0.0

        hota = np.sqrt(deta * assa)
        hota_per_alpha.append(hota)
        deta_per_alpha.append(deta)
        assa_per_alpha.append(assa)

    return {
        "HOTA": float(np.mean(hota_per_alpha)),
        "DetA": float(np.mean(deta_per_alpha)),
        "AssA": float(np.mean(assa_per_alpha)),
    }


def _compute_metrics(results: dict[int, list[tuple]],
                     gt: dict[int, list[tuple]],
                     tracker_metrics=None, wall_time: float = 0.0,
                     seq_name: str = ""):
    """Compute and print MOTA, IDF1, HOTA, FP, FN, IDs in MOT Challenge table format."""
    all_frames = sorted(set(list(gt.keys()) + list(results.keys())))

    # --- HOTA (always available, no extra deps) ---
    hota_scores = _compute_hota(results, gt)

    # --- MOTA / IDF1 / FP / FN / IDs (requires motmetrics) ---
    mota = idf1 = fp = fn = ids = None
    try:
        import motmetrics as mm

        acc = mm.MOTAccumulator(auto_id=True)
        for frame in all_frames:
            gt_dets = gt.get(frame, [])
            pred_dets = results.get(frame, [])
            gt_ids = [d[0] for d in gt_dets]
            pred_ids = [d[0] for d in pred_dets]
            gt_bboxes = _to_xyxy(gt_dets)
            pred_bboxes = _to_xyxy(pred_dets)
            distances = mm.distances.iou_matrix(gt_bboxes, pred_bboxes, max_iou=0.5)
            acc.update(gt_ids, pred_ids, distances)

        mh = mm.metrics.create()
        summary = mh.compute(acc, metrics=[
            "mota", "idf1", "num_false_positives", "num_misses",
            "num_switches",
        ], name="MOT")
        mota = summary.loc["MOT", "mota"] * 100
        idf1 = summary.loc["MOT", "idf1"] * 100
        fp = int(summary.loc["MOT", "num_false_positives"])
        fn = int(summary.loc["MOT", "num_misses"])
        ids = int(summary.loc["MOT", "num_switches"])
    except ImportError:
        LOGGER.warning("motmetrics not installed — MOTA/IDF1 unavailable. "
                       "Install with: pip install motmetrics")

    hota_pct = hota_scores["HOTA"] * 100

    # --- Performance stats ---
    n_frames = len(all_frames)
    fps_val = n_frames / wall_time if wall_time > 0 else 0.0
    init_ms = None
    avg_update = None
    if tracker_metrics is not None:
        init_ms = getattr(tracker_metrics, "_init_time_ms", None)
        m = getattr(tracker_metrics, "metrics", tracker_metrics)
        avg_update = m.update_ms

    # --- Print MOT Challenge style table ---
    label = seq_name or "Sequence"

    print()
    header = f"{'Dataset':<16} {'MOTA':>6} {'IDF1':>6} {'HOTA':>6} {'FP':>8} {'FN':>8} {'IDs':>6}"
    sep = "-" * len(header)
    print(sep)
    print(header)
    print(sep)

    mota_s = f"{mota:.1f}" if mota is not None else "N/A"
    idf1_s = f"{idf1:.1f}" if idf1 is not None else "N/A"
    fp_s = str(fp) if fp is not None else "N/A"
    fn_s = str(fn) if fn is not None else "N/A"
    ids_s = str(ids) if ids is not None else "N/A"

    print(f"{label:<16} {mota_s:>6} {idf1_s:>6} {hota_pct:>6.1f} {fp_s:>8} {fn_s:>8} {ids_s:>6}")
    print(sep)

    # Performance line
    perf_parts = [f"Frames: {n_frames}"]
    if wall_time > 0:
        perf_parts.append(f"Time: {wall_time:.2f}s")
        perf_parts.append(f"FPS: {fps_val:.1f}")
    if init_ms is not None:
        perf_parts.append(f"Init: {init_ms:.1f}ms")
    if avg_update is not None:
        perf_parts.append(f"Avg update: {avg_update:.2f}ms")
    print("  " + "  |  ".join(perf_parts))

    # HOTA breakdown
    print(f"  DetA: {hota_scores['DetA']*100:.1f}  |  AssA: {hota_scores['AssA']*100:.1f}")
    print()


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------

# Distinct colors for track IDs (BGR for OpenCV)
_COLORS = [
    (230, 25, 75), (60, 180, 75), (255, 225, 25), (0, 130, 200),
    (245, 130, 48), (145, 30, 180), (70, 240, 240), (240, 50, 230),
    (210, 245, 60), (250, 190, 212), (0, 128, 128), (220, 190, 255),
    (170, 110, 40), (255, 250, 200), (128, 0, 0), (170, 255, 195),
    (128, 128, 0), (255, 215, 180), (0, 0, 128), (128, 128, 128),
]


def _render_video(results_path: str, images_dir: str, output_video: str,
                  frame_rate: int = 30, gt_path: str | None = None):
    """Draw tracker boxes on frame images and write a video."""
    try:
        import cv2
    except ImportError:
        LOGGER.error("opencv-python not installed. Install with: pip install opencv-python")
        return

    results = load_mot_file(results_path)
    gt = load_mot_file(gt_path) if gt_path else {}

    # Find all images sorted
    img_files = sorted(Path(images_dir).glob("*.jpg"))
    if not img_files:
        img_files = sorted(Path(images_dir).glob("*.png"))
    if not img_files:
        LOGGER.error("No images found in %s", images_dir)
        return

    # Read first frame to get dimensions
    sample = cv2.imread(str(img_files[0]))
    h, w = sample.shape[:2]

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(output_video, fourcc, frame_rate, (w, h))

    LOGGER.info("Rendering %d frames to %s ...", len(img_files), output_video)

    for idx, img_path in enumerate(img_files):
        frame_num = idx + 1
        frame = cv2.imread(str(img_path))

        # Draw GT boxes (thin, gray)
        for gt_det in gt.get(frame_num, []):
            tid, x, y, bw, bh, _ = gt_det
            cv2.rectangle(frame, (int(x), int(y)), (int(x + bw), int(y + bh)),
                          (180, 180, 180), 1)

        # Draw tracker boxes (thick, colored by ID)
        for det in results.get(frame_num, []):
            tid, x, y, bw, bh, conf = det
            color = _COLORS[tid % len(_COLORS)]
            x1, y1, x2, y2 = int(x), int(y), int(x + bw), int(y + bh)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            label = f"ID {tid} {conf:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
            cv2.putText(frame, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Frame counter
        cv2.putText(frame, f"Frame {frame_num}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        writer.write(frame)

    writer.release()
    LOGGER.info("Video saved: %s", output_video)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run ByteTracker on video and compute MOT metrics.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument(
        "seq_dir", nargs="?", metavar="SEQ_DIR",
        help="MOT sequence directory (e.g. MOT17/train/MOT17-04-SDP). "
             "Auto-detects det/det.txt, gt/gt.txt, img1/, and seqinfo.ini.")

    input_group = parser.add_argument_group("input")
    input_group.add_argument(
        "--detections", metavar="FILE",
        help="Pre-computed detections in MOT format (skip Hailo pipeline)")

    eval_group = parser.add_argument_group("evaluation")
    eval_group.add_argument(
        "--gt", metavar="FILE",
        help="Ground-truth annotations in MOT Challenge format")
    eval_group.add_argument(
        "-o", "--output", default="mot_results.txt", metavar="FILE",
        help="Output file for tracking results (default: mot_results.txt)")

    vis_group = parser.add_argument_group("visualization")
    vis_group.add_argument(
        "--images-dir", metavar="DIR",
        help="Directory with frame images (e.g. MOT17/.../img1). "
             "Auto-detected from --detections path if not specified.")
    vis_group.add_argument(
        "--visualize", metavar="FILE",
        help="Output video file with tracker boxes drawn (e.g. output.mp4)")

    tracker_group = parser.add_argument_group("tracker")
    tracker_group.add_argument("--track-thresh", type=float, default=0.4,
                               help="Detection confidence threshold (default: 0.4)")
    tracker_group.add_argument("--track-buffer", type=int, default=90,
                               help="Frames to keep lost tracks (default: 90)")
    tracker_group.add_argument("--match-thresh", type=float, default=0.5,
                               help="IOU matching threshold (default: 0.5)")
    tracker_group.add_argument("--frame-rate", type=int, default=30,
                               help="Video frame rate (default: 30)")

    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Enable debug logging")

    return parser


def _parse_seqinfo(seq_dir: str) -> dict[str, str]:
    """Parse seqinfo.ini from a MOT sequence directory."""
    ini_path = Path(seq_dir) / "seqinfo.ini"
    info = {}
    if ini_path.is_file():
        import configparser
        cfg = configparser.ConfigParser()
        cfg.read(str(ini_path))
        if cfg.has_section("Sequence"):
            info = dict(cfg["Sequence"])
    return info


def _resolve_seq_dir(args, parser):
    """Resolve --detections, --gt, --images-dir, --frame-rate from seq_dir."""
    seq = Path(args.seq_dir)
    if not seq.is_dir():
        parser.error(f"Sequence directory not found: {args.seq_dir}")

    # Parse seqinfo.ini for metadata
    info = _parse_seqinfo(str(seq))

    # Detections
    if not args.detections:
        det = seq / "det" / "det.txt"
        if det.is_file():
            args.detections = str(det)
        else:
            parser.error(f"No det/det.txt in {seq}")

    # Ground truth
    if not args.gt:
        gt = seq / "gt" / "gt.txt"
        if gt.is_file():
            args.gt = str(gt)

    # Images
    if not args.images_dir:
        im_dir = info.get("imdir", "img1")
        img = seq / im_dir
        if img.is_dir():
            args.images_dir = str(img)

    # Frame rate from seqinfo.ini (only if user didn't override)
    if "framerate" in info:
        # argparse default is 30; treat it as "not overridden" if still 30
        args.frame_rate = int(info["framerate"])

    # Output file default: <seq_name>_results.txt
    if args.output == "mot_results.txt":
        seq_name = info.get("name", seq.name)
        args.output = f"results_{seq_name}.txt"

    # Auto-enable visualization: <seq_name>.mp4
    if not args.visualize:
        seq_name = info.get("name", seq.name)
        args.visualize = f"{seq_name}.mp4"


def main():
    parser = build_parser()
    args, remaining = parser.parse_known_args()

    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    # Resolve paths from sequence directory
    if args.seq_dir:
        _resolve_seq_dir(args, parser)

    if args.detections:
        # Detections-only mode — no Hailo hardware needed
        if not os.path.isfile(args.detections):
            parser.error(f"Detections file not found: {args.detections}")
        LOGGER.info("Running tracker on pre-computed detections: %s", args.detections)
        run_from_detections(args.detections, args.output, args.gt, args)

    elif any(a.startswith("--input") for a in remaining):
        # Pipeline mode — requires Hailo
        LOGGER.info("Running full Hailo pipeline")
        run_from_pipeline(args.output, args.gt, args, remaining)

    else:
        parser.error(
            "Provide either SEQ_DIR, --detections <file>, "
            "or --input <video> (full Hailo pipeline)")

    # --- Visualization ---
    if args.visualize:
        images_dir = args.images_dir
        if not images_dir and args.detections:
            # Auto-detect: .../MOT17-04-SDP/det/det.txt → .../MOT17-04-SDP/img1/
            candidate = Path(args.detections).resolve().parent.parent / "img1"
            if candidate.is_dir():
                images_dir = str(candidate)
                LOGGER.info("Auto-detected images dir: %s", images_dir)
        if not images_dir:
            LOGGER.warning("--visualize skipped: no --images-dir found")
        else:
            _render_video(args.output, images_dir, args.visualize,
                          frame_rate=args.frame_rate, gt_path=args.gt)


if __name__ == "__main__":
    main()
