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

def load_mot_file(path: str, is_gt: bool = False) -> dict[int, list[tuple]]:
    """Load a MOT Challenge format file.

    Args:
        path: Path to the MOT file.
        is_gt: If True, apply MOT Challenge GT filtering:
               keep only flag==1 (active) and class==1 (pedestrian).

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

            if is_gt and len(row) >= 8:
                flag = int(row[6])
                cls = int(row[7])
                if flag != 1 or cls != 1:
                    continue

            data[frame].append((tid, x, y, w, h, conf))
    return dict(data)


def write_mot_line(f, frame: int, tid: int, x: float, y: float,
                   w: float, h: float, conf: float):
    f.write(f"{frame},{tid},{x:.2f},{y:.2f},{w:.2f},{h:.2f},{conf:.4f},-1,-1,-1\n")


# ---------------------------------------------------------------------------
# Detections-only mode (no Hailo needed)
# ---------------------------------------------------------------------------

def run_from_detections(det_path: str, output_path: str,
                        gt_path: str | None, args: argparse.Namespace,
                        quiet: bool = False) -> dict | None:
    """Run tracker on pre-computed detections in MOT format.

    Detection file format (per line):
        <frame>,<id_ignored>,<bb_left>,<bb_top>,<bb_width>,<bb_height>,<conf>
    """
    from drone_follow.pipeline_adapter.tracker import MetricsTracker
    from drone_follow.pipeline_adapter.tracker_factory import create_tracker

    raw = load_mot_file(det_path)
    if not raw:
        LOGGER.error("No detections found in %s", det_path)
        return

    t_init = time.monotonic()
    tracker = MetricsTracker(create_tracker(
        args.tracker,
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

    gt = load_mot_file(gt_path, is_gt=True) if gt_path else {}
    return _compute_metrics(results, gt, tracker_metrics=tracker,
                            wall_time=wall_time, seq_name=seq_name,
                            quiet=quiet)


# ---------------------------------------------------------------------------
# Image sequence → video
# ---------------------------------------------------------------------------

def _images_to_video(images_dir: str, output_path: str, frame_rate: int = 30) -> str:
    """Convert an image sequence to a video file using OpenCV.

    Returns the path to the created video.
    """
    import cv2

    img_files = sorted(Path(images_dir).glob("*.jpg"))
    if not img_files:
        img_files = sorted(Path(images_dir).glob("*.png"))
    if not img_files:
        raise FileNotFoundError(f"No images found in {images_dir}")

    sample = cv2.imread(str(img_files[0]))
    h, w = sample.shape[:2]

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(output_path, fourcc, frame_rate, (w, h))

    LOGGER.info("Converting %d images to video %s ...", len(img_files), output_path)
    for img_path in img_files:
        writer.write(cv2.imread(str(img_path)))
    writer.release()
    LOGGER.info("Video created: %s (%dx%d @ %d fps)", output_path, w, h, frame_rate)
    return output_path


# ---------------------------------------------------------------------------
# Pipeline mode (Hailo detection + tracker)
# ---------------------------------------------------------------------------

def run_from_pipeline(output_path: str, gt_path: str | None,
                      args: argparse.Namespace, remaining_args: list[str],
                      reference_detections: dict[int, list[tuple]] | None = None,
                      image_size: tuple[int, int] | None = None,
                      quiet: bool = False) -> dict:
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
    tracker = getattr(args, "tracker", None)
    app = create_app(shared_state, target_state=target_state,
                     eos_reached=eos_reached, parser=parser,
                     tracker_name=tracker)

    # Hook into the tracker to capture per-frame results (normalized [0,1] coords)
    # Also capture raw detections (before tracking) for detection AP
    raw_detections: dict[int, list[tuple]] = {}
    _hook_tracker_output(app, results={}, output_path=output_path,
                         raw_detections=raw_detections)

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
    gt = load_mot_file(gt_path, is_gt=True) if gt_path else {}
    return _compute_metrics(all_results, gt, tracker_metrics=tracker_metrics,
                            wall_time=wall_time,
                            seq_name=Path(output_path).stem,
                            raw_detections=raw_detections or None,
                            reference_detections=reference_detections,
                            image_size=image_size,
                            quiet=quiet)


def _build_pipeline_args(input_source: str, args: argparse.Namespace,
                         extra: list[str] | None = None) -> list[str]:
    """Build the CLI args list to inject into the Hailo pipeline parser."""
    pipeline_args = ["--input", input_source]
    if getattr(args, "model", None) is not None:
        pipeline_args += ["--hef-path", args.model]
    if getattr(args, "tiles_x", None) is not None:
        pipeline_args += ["--tiles-x", str(args.tiles_x)]
    if getattr(args, "tiles_y", None) is not None:
        pipeline_args += ["--tiles-y", str(args.tiles_y)]
    if getattr(args, "multi_scale", False):
        pipeline_args += ["--multi-scale"]
    if getattr(args, "scale_levels", None) is not None:
        pipeline_args += ["--scale-levels", str(args.scale_levels)]
    if getattr(args, "iou_threshold", None) is not None:
        pipeline_args += ["--iou-threshold", str(args.iou_threshold)]
    if getattr(args, "min_overlap", None) is not None:
        pipeline_args += ["--min-overlap", str(args.min_overlap)]
    if getattr(args, "detection_threshold", None) is not None:
        pipeline_args += ["--detection-threshold", str(args.detection_threshold)]
    if getattr(args, "border_threshold", None) is not None:
        pipeline_args += ["--border-threshold", str(args.border_threshold)]
    if extra:
        pipeline_args += extra
    return pipeline_args


def _inject_args(extra_args: list[str]):
    """Inject extra args into sys.argv for the pipeline parser."""
    # The pipeline parser reads sys.argv, so we append our extra args
    sys.argv = [sys.argv[0]] + extra_args


def _hook_tracker_output(app, results: dict, output_path: str,
                         raw_detections: dict | None = None, **_kw):
    """Monkey-patch the tracker to intercept per-frame results.

    We wrap tracker.update() to capture every frame's output and write
    MOT lines as they come in.  Coordinates are stored as **normalized**
    [0,1] values so they can be scaled to any resolution at render time.

    If *raw_detections* dict is provided, also captures raw detections
    (before tracking) for detection-level mAP evaluation.
    """
    from drone_follow.pipeline_adapter import hailo_drone_detection_manager as mgr

    original_run_tracker = mgr._run_tracker
    out_file = open(output_path, "w")
    frame_counter = [0]
    lock = threading.Lock()

    def patched_run_tracker(tracker, persons):
        available_ids, person_by_id, person_to_id = original_run_tracker(tracker, persons)

        with lock:
            frame_counter[0] += 1
            frame_num = frame_counter[0]

            for tid, person in person_by_id.items():
                bbox = person.get_bbox()
                # Store normalized [0,1] coordinates — scaled at render time
                x = bbox.xmin()
                y = bbox.ymin()
                w = bbox.width()
                h = bbox.height()
                conf = person.get_confidence()
                write_mot_line(out_file, frame_num, tid, x, y, w, h, conf)

            # Capture raw detections (before tracking) for mAP
            if raw_detections is not None:
                frame_dets = []
                for person in persons:
                    bbox = person.get_bbox()
                    frame_dets.append((
                        -1,  # no track id for raw detections
                        bbox.xmin(), bbox.ymin(),
                        bbox.width(), bbox.height(),
                        person.get_confidence(),
                    ))
                raw_detections[frame_num] = frame_dets

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


def _compute_detection_ap(raw_detections: dict[int, list[tuple]],
                          gt: dict[int, list[tuple]],
                          iou_thresh: float = 0.5) -> dict:
    """Compute detection-level AP (Average Precision) at a given IoU threshold.

    Compares raw detections (before tracking) against ground truth.
    Returns dict with AP, precision, recall, total_tp, total_fp, total_fn.
    """
    all_frames = sorted(set(list(gt.keys()) + list(raw_detections.keys())))

    # Collect all predictions with scores for ranking
    all_preds = []  # (score, is_tp)
    total_gt = 0

    for frame in all_frames:
        gt_dets = gt.get(frame, [])
        pred_dets = raw_detections.get(frame, [])
        total_gt += len(gt_dets)

        if not pred_dets:
            continue

        gt_boxes = _to_xyxy(gt_dets)
        pred_boxes = _to_xyxy(pred_dets)
        pred_scores = [d[5] for d in pred_dets]

        if len(gt_boxes) == 0:
            for s in pred_scores:
                all_preds.append((s, False))
            continue

        iou = _iou_matrix(gt_boxes, pred_boxes)
        gt_matched = set()

        # Sort predictions by score (descending) for greedy matching
        sorted_idx = sorted(range(len(pred_scores)),
                            key=lambda i: pred_scores[i], reverse=True)
        for pi in sorted_idx:
            best_iou = 0.0
            best_gi = -1
            for gi in range(len(gt_boxes)):
                if gi in gt_matched:
                    continue
                if iou[gi, pi] > best_iou:
                    best_iou = iou[gi, pi]
                    best_gi = gi
            if best_iou >= iou_thresh and best_gi >= 0:
                all_preds.append((pred_scores[pi], True))
                gt_matched.add(best_gi)
            else:
                all_preds.append((pred_scores[pi], False))

    if not all_preds or total_gt == 0:
        return {"AP": 0.0, "precision": 0.0, "recall": 0.0,
                "total_tp": 0, "total_fp": 0, "total_fn": total_gt}

    # Sort by score descending and compute precision-recall curve
    all_preds.sort(key=lambda x: x[0], reverse=True)
    tp_cumsum = 0
    fp_cumsum = 0
    precisions = []
    recalls = []

    for score, is_tp in all_preds:
        if is_tp:
            tp_cumsum += 1
        else:
            fp_cumsum += 1
        precisions.append(tp_cumsum / (tp_cumsum + fp_cumsum))
        recalls.append(tp_cumsum / total_gt)

    # 11-point interpolated AP
    ap = 0.0
    for t in np.arange(0.0, 1.1, 0.1):
        p_at_r = max((p for p, r in zip(precisions, recalls) if r >= t), default=0.0)
        ap += p_at_r / 11.0

    return {
        "AP": ap,
        "precision": precisions[-1] if precisions else 0.0,
        "recall": recalls[-1] if recalls else 0.0,
        "total_tp": tp_cumsum,
        "total_fp": fp_cumsum,
        "total_fn": total_gt - tp_cumsum,
    }


def _scale_normalized_results(results: dict[int, list[tuple]],
                              image_width: float, image_height: float,
                              ) -> dict[int, list[tuple]]:
    """Scale normalized [0,1] results to pixel coordinates."""
    LOGGER.info("Scaling normalized results to %dx%d", int(image_width), int(image_height))
    scaled: dict[int, list[tuple]] = {}
    for frame, dets in results.items():
        scaled[frame] = [
            (tid, x * image_width, y * image_height,
             w * image_width, h * image_height, conf)
            for tid, x, y, w, h, conf in dets
        ]
    return scaled


def _compute_metrics(results: dict[int, list[tuple]],
                     gt: dict[int, list[tuple]],
                     tracker_metrics=None, wall_time: float = 0.0,
                     seq_name: str = "",
                     raw_detections: dict[int, list[tuple]] | None = None,
                     reference_detections: dict[int, list[tuple]] | None = None,
                     image_size: tuple[int, int] | None = None,
                     quiet: bool = False) -> dict:
    """Compute and print MOTA, IDF1, HOTA, FP, FN, IDs in MOT Challenge table format.

    Args:
        raw_detections: Raw detections (before tracking) for AP computation.
        reference_detections: Reference detections to compare against (e.g. det.txt).
            If provided, detection AP is computed against these instead of GT.
        image_size: (width, height) for scaling normalized results. If None,
            inferred from seqinfo.ini or GT.
        quiet: If True, suppress printing (just return the dict).

    Returns:
        Dict with all computed metrics.
    """
    # Scale normalized results to pixel space if needed
    if _is_normalized(results) and results:
        if image_size:
            w, h = image_size
        else:
            # Fallback: estimate from reference detections or GT max coordinates
            ref = reference_detections or gt
            w = h = 0.0
            for dets in ref.values():
                for _, x, y, bw, bh, _ in dets:
                    w = max(w, x + bw)
                    h = max(h, y + bh)
            if w <= 1.5 and h <= 1.5:
                w = h = 1.0  # both normalized, no scaling needed
        if w > 1.5 or h > 1.5:
            results = _scale_normalized_results(results, w, h)
            if raw_detections:
                raw_detections = _scale_normalized_results(raw_detections, w, h)

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

    # --- Build result dict ---
    det_ap = None
    if raw_detections and reference_detections:
        det_ap = _compute_detection_ap(raw_detections, reference_detections, iou_thresh=0.5)
    elif raw_detections and gt:
        det_ap = _compute_detection_ap(raw_detections, gt, iou_thresh=0.5)

    result = {
        "seq_name": seq_name or "Sequence",
        "MOTA": mota,
        "IDF1": idf1,
        "HOTA": hota_pct,
        "DetA": hota_scores["DetA"] * 100,
        "AssA": hota_scores["AssA"] * 100,
        "FP": fp,
        "FN": fn,
        "IDs": ids,
        "frames": n_frames,
        "wall_time": wall_time,
        "fps": fps_val,
        "init_ms": init_ms,
        "avg_update_ms": avg_update,
        "det_ap": det_ap,
    }

    if quiet:
        return result

    # --- Print MOT Challenge style table ---
    label = result["seq_name"]

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

    # Detection-level AP
    if det_ap is not None:
        ap_label = "vs ref" if (raw_detections and reference_detections) else "vs GT"
        print(f"  Detection AP@50 ({ap_label}): {det_ap['AP']*100:.1f}  |  "
              f"Precision: {det_ap['precision']*100:.1f}  |  "
              f"Recall: {det_ap['recall']*100:.1f}  |  "
              f"TP: {det_ap['total_tp']}  FP: {det_ap['total_fp']}  FN: {det_ap['total_fn']}")
    print()

    return result


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


def _draw_detections(frame, frame_num: int, results: dict, gt: dict,
                     normalized: bool = False):
    """Draw GT and tracker boxes on a single frame (in-place).

    If *normalized* is True, result coordinates are in [0,1] and are scaled
    to the actual frame dimensions.  GT coordinates are always in pixels.
    """
    import cv2

    h_frame, w_frame = frame.shape[:2]

    # Draw GT boxes (thin, gray) — always pixel coordinates
    for gt_det in gt.get(frame_num, []):
        tid, x, y, bw, bh, _ = gt_det
        cv2.rectangle(frame, (int(x), int(y)), (int(x + bw), int(y + bh)),
                      (180, 180, 180), 1)

    # Draw tracker boxes (thick, colored by ID)
    for det in results.get(frame_num, []):
        tid, x, y, bw, bh, conf = det
        if normalized:
            x, y, bw, bh = x * w_frame, y * h_frame, bw * w_frame, bh * h_frame
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


def _is_normalized(results: dict[int, list[tuple]]) -> bool:
    """Heuristic: if all bbox coordinates are in [0, 1], assume normalized."""
    for dets in results.values():
        for _, x, y, w, h, _ in dets:
            if x + w > 1.5 or y + h > 1.5:
                return False
    return True


def _render_video(results_path: str, images_dir: str, output_video: str,
                  frame_rate: int = 30, gt_path: str | None = None):
    """Draw tracker boxes on frame images and write a video."""
    try:
        import cv2
    except ImportError:
        LOGGER.error("opencv-python not installed. Install with: pip install opencv-python")
        return

    results = load_mot_file(results_path)
    gt = load_mot_file(gt_path, is_gt=True) if gt_path else {}
    norm = _is_normalized(results)

    img_files = sorted(Path(images_dir).glob("*.jpg"))
    if not img_files:
        img_files = sorted(Path(images_dir).glob("*.png"))
    if not img_files:
        LOGGER.error("No images found in %s", images_dir)
        return

    sample = cv2.imread(str(img_files[0]))
    h, w = sample.shape[:2]

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(output_video, fourcc, frame_rate, (w, h))

    LOGGER.info("Rendering %d frames to %s (normalized=%s) ...",
                len(img_files), output_video, norm)

    for idx, img_path in enumerate(img_files):
        frame_num = idx + 1
        frame = cv2.imread(str(img_path))
        _draw_detections(frame, frame_num, results, gt, normalized=norm)
        writer.write(frame)

    writer.release()
    LOGGER.info("Video saved: %s", output_video)


def _render_video_from_video(results_path: str, input_video: str,
                             output_video: str, gt_path: str | None = None):
    """Draw tracker boxes on frames from an input video and write output video."""
    try:
        import cv2
    except ImportError:
        LOGGER.error("opencv-python not installed. Install with: pip install opencv-python")
        return

    results = load_mot_file(results_path)
    gt = load_mot_file(gt_path, is_gt=True) if gt_path else {}
    norm = _is_normalized(results)

    cap = cv2.VideoCapture(input_video)
    if not cap.isOpened():
        LOGGER.error("Cannot open video: %s", input_video)
        return

    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(output_video, fourcc, fps, (w, h))

    LOGGER.info("Rendering %d frames from %s to %s (normalized=%s) ...",
                total, input_video, output_video, norm)

    frame_num = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame_num += 1
        _draw_detections(frame, frame_num, results, gt, normalized=norm)
        writer.write(frame)

    cap.release()
    writer.release()
    LOGGER.info("Video saved: %s (%d frames)", output_video, frame_num)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run tracker on video and compute MOT metrics.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument(
        "seq_dir", nargs="?", metavar="SEQ_DIR",
        help="MOT sequence directory (e.g. MOT17/train/MOT17-04-SDP). "
             "Auto-detects det/det.txt, gt/gt.txt, img1/, and seqinfo.ini.")

    input_group = parser.add_argument_group("input")
    input_group.add_argument(
        "--input", metavar="VIDEO",
        help="Input video file for Hailo pipeline mode (e.g. video.mp4, udp://..., usb, rpi)")
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
    vis_group.add_argument(
        "--visualize-pipeline", metavar="FILE",
        help="Run Hailo pipeline on images, track, and render video (requires Hailo HW)")
    vis_group.add_argument(
        "--run-pipeline-args", nargs=argparse.REMAINDER, default=[],
        help="Extra args passed to the Hailo pipeline (e.g. --hef-path model.hef)")

    tracker_group = parser.add_argument_group("tracker")
    tracker_group.add_argument("--tracker", choices=("byte", "fast"), default="byte",
                               help="Tracker algorithm: byte (ByteTracker) or fast (FastTracker)")
    tracker_group.add_argument("--track-thresh", type=float, default=0.4,
                               help="Detection confidence threshold (default: 0.4)")
    tracker_group.add_argument("--track-buffer", type=int, default=90,
                               help="Frames to keep lost tracks (default: 90)")
    tracker_group.add_argument("--match-thresh", type=float, default=0.5,
                               help="IOU matching threshold (default: 0.5)")
    tracker_group.add_argument("--frame-rate", type=int, default=30,
                               help="Video frame rate (default: 30)")

    pipeline_group = parser.add_argument_group("pipeline (for --input and --visualize-pipeline)")
    pipeline_group.add_argument("--model", default=None, metavar="NAME_OR_PATH",
                                help="HEF model name or path (passed as --hef-path to the Hailo pipeline)")
    pipeline_group.add_argument("--tiles-x", type=int, default=None,
                                help="Number of tiles horizontally (passed to Hailo pipeline)")
    pipeline_group.add_argument("--tiles-y", type=int, default=None,
                                help="Number of tiles vertically (passed to Hailo pipeline)")
    pipeline_group.add_argument("--multi-scale", action="store_true", default=False,
                                help="Enable multi-scale tiling (adds full-frame pass)")
    pipeline_group.add_argument("--scale-levels", type=int, default=None, choices=[1, 2, 3],
                                help="Scale levels for multi-scale: 1={1x1}, 2={1x1+2x2}, 3={1x1+2x2+3x3}")
    pipeline_group.add_argument("--iou-threshold", type=float, default=None,
                                help="NMS IOU threshold for tile aggregation (default: 0.3)")
    pipeline_group.add_argument("--min-overlap", type=float, default=None,
                                help="Minimum tile overlap ratio (default: 0.1)")
    pipeline_group.add_argument("--detection-threshold", type=float, default=None,
                                help="Override detection confidence threshold in post-processing")
    pipeline_group.add_argument("--border-threshold", type=float, default=None,
                                help="Border threshold for multi-scale mode (default: 0.15)")

    parser.add_argument("--compare", action="store_true", default=False,
                        help="Run both pre-computed detections and Hailo pipeline, "
                             "then print a side-by-side gap analysis (requires SEQ_DIR with img1/)")

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

    # Image size from seqinfo.ini (for scaling normalized coords)
    if "imwidth" in info and "imheight" in info:
        args.image_size = (int(info["imwidth"]), int(info["imheight"]))
    else:
        args.image_size = None

    # Output file default: results_<seq_name>-<tracker>.txt
    seq_name = info.get("name", seq.name)
    tracker = args.tracker
    if args.output == "mot_results.txt":
        args.output = f"results_{seq_name}-{tracker}.txt"

    # Auto-enable visualization: <seq_name>-<tracker>.mp4
    if not args.visualize:
        args.visualize = f"{seq_name}-{tracker}.mp4"
    # Don't auto-enable pipeline visualization (requires Hailo HW)


def _print_comparison(baseline: dict, pipeline: dict):
    """Print a side-by-side gap analysis between baseline and pipeline metrics."""
    def _fmt(v, fmt=".1f"):
        if v is None:
            return "N/A"
        return f"{v:{fmt}}"

    def _delta(base, pipe, fmt=".1f", invert=False):
        """Format delta. invert=True means lower is better (FP, FN, IDs)."""
        if base is None or pipe is None:
            return ""
        d = pipe - base
        if invert:
            d = -d
        sign = "+" if d > 0 else ""
        color = "" if d == 0 else (" ^^^" if d < 0 else "")
        return f"{sign}{d:{fmt}}{color}"

    print()
    print("=" * 78)
    print("  GAP ANALYSIS: Pre-computed Detections vs Hailo Pipeline")
    print("=" * 78)

    header = f"{'Metric':<22} {'Baseline':>10} {'Pipeline':>10} {'Delta':>12}  {'Note'}"
    print(header)
    print("-" * 78)

    rows = [
        ("MOTA",        baseline["MOTA"],  pipeline["MOTA"],  False, ".1f", "higher=better"),
        ("IDF1",        baseline["IDF1"],  pipeline["IDF1"],  False, ".1f", "higher=better"),
        ("HOTA",        baseline["HOTA"],  pipeline["HOTA"],  False, ".1f", "higher=better"),
        ("  DetA",      baseline["DetA"],  pipeline["DetA"],  False, ".1f", "detection quality"),
        ("  AssA",      baseline["AssA"],  pipeline["AssA"],  False, ".1f", "association quality"),
        ("FP",          baseline["FP"],    pipeline["FP"],    True,  "d",   "lower=better"),
        ("FN",          baseline["FN"],    pipeline["FN"],    True,  "d",   "lower=better"),
        ("ID Switches", baseline["IDs"],   pipeline["IDs"],   True,  "d",   "lower=better"),
    ]

    for label, bv, pv, invert, fmt, note in rows:
        print(f"{label:<22} {_fmt(bv, fmt):>10} {_fmt(pv, fmt):>10} "
              f"{_delta(bv, pv, fmt, invert):>12}  {note}")

    print("-" * 78)

    # Performance
    print(f"{'FPS':<22} {baseline['fps']:>10.1f} {pipeline['fps']:>10.1f}")
    if baseline["avg_update_ms"] and pipeline["avg_update_ms"]:
        print(f"{'Avg update (ms)':<22} {baseline['avg_update_ms']:>10.2f} "
              f"{pipeline['avg_update_ms']:>10.2f}")

    # Detection AP (pipeline only)
    det_ap = pipeline.get("det_ap")
    if det_ap:
        print()
        print(f"  Pipeline Detection AP@50: {det_ap['AP']*100:.1f}%")
        print(f"  Precision: {det_ap['precision']*100:.1f}%  "
              f"Recall: {det_ap['recall']*100:.1f}%  "
              f"TP: {det_ap['total_tp']}  FP: {det_ap['total_fp']}  FN: {det_ap['total_fn']}")

    # Diagnosis
    print()
    print("  DIAGNOSIS:")
    if baseline["MOTA"] is not None and pipeline["MOTA"] is not None:
        mota_gap = baseline["MOTA"] - pipeline["MOTA"]
        deta_gap = baseline["DetA"] - pipeline["DetA"]
        assa_gap = baseline["AssA"] - pipeline["AssA"]
        fn_ratio = (pipeline["FN"] / max(baseline["FN"], 1)) if baseline["FN"] else 0

        if deta_gap > assa_gap * 2:
            print(f"  >> Detection is the bottleneck (DetA gap: {deta_gap:.1f} vs AssA gap: {assa_gap:.1f})")
            print(f"     FN ratio: {fn_ratio:.1f}x — pipeline misses {fn_ratio:.1f}x more objects")
            if det_ap and det_ap["recall"] < 0.7:
                print(f"     Low recall ({det_ap['recall']*100:.0f}%) suggests:")
                print(f"       - Try lowering --detection-threshold (currently in labels JSON)")
                print(f"       - Try lowering --border-threshold for tile-edge detections")
                print(f"       - Use a stronger model (yolov8s/m instead of nano)")
        elif assa_gap > deta_gap * 2:
            print(f"  >> Tracking association is the bottleneck (AssA gap: {assa_gap:.1f} vs DetA gap: {deta_gap:.1f})")
            print(f"     Consider: longer --track-buffer or a different tracker")
        else:
            print(f"  >> Both detection ({deta_gap:.1f}) and association ({assa_gap:.1f}) contribute to the gap")

        if mota_gap < 5:
            print(f"  >> Gap is small ({mota_gap:.1f} MOTA points) — pipeline is competitive")

    print("=" * 78)
    print()


def main():
    parser = build_parser()
    args, remaining = parser.parse_known_args()

    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    # Resolve paths from sequence directory
    if args.seq_dir:
        _resolve_seq_dir(args, parser)

    # Default output naming for --input mode (no seq_dir)
    if args.input and not args.seq_dir:
        stem = Path(args.input).stem if os.path.isfile(args.input) else "pipeline"
        tracker = args.tracker
        if args.output == "mot_results.txt":
            args.output = f"results_{stem}-{tracker}.txt"
        if not args.visualize:
            args.visualize = f"{stem}-{tracker}.mp4"

    image_size = getattr(args, "image_size", None)

    # --- Compare mode: run both baseline and pipeline, print gap analysis ---
    if args.compare:
        if not args.seq_dir:
            parser.error("--compare requires SEQ_DIR (a MOT sequence directory)")
        if not args.detections or not args.gt:
            parser.error("--compare requires detections (det/det.txt) and GT (gt/gt.txt)")
        images_dir = getattr(args, "images_dir", None)
        if not images_dir:
            parser.error("--compare requires images (img1/) for Hailo pipeline")

        import tempfile

        # 1) Baseline: pre-computed detections
        baseline_output = args.output
        LOGGER.info("=== BASELINE: Running %s tracker on pre-computed detections ===",
                    args.tracker)
        baseline = run_from_detections(args.detections, baseline_output, args.gt, args)

        # 2) Pipeline: Hailo detection + tracking
        ref_dets = load_mot_file(args.detections)
        tmp_video = tempfile.NamedTemporaryFile(suffix=".mp4", delete=False).name
        try:
            _images_to_video(images_dir, tmp_video, frame_rate=args.frame_rate)
            pipeline_output = baseline_output.replace(".txt", "_pipeline.txt")
            pipeline_args = _build_pipeline_args(
                tmp_video, args, args.run_pipeline_args or remaining)
            LOGGER.info("=== PIPELINE: Running Hailo pipeline with %s tracker ===",
                        args.tracker)
            pipeline = run_from_pipeline(pipeline_output, args.gt, args, pipeline_args,
                                         reference_detections=ref_dets,
                                         image_size=image_size)
        finally:
            try:
                os.unlink(tmp_video)
            except OSError:
                pass

        # 3) Gap analysis
        _print_comparison(baseline, pipeline)
        return

    if args.input and not args.detections:
        # Pipeline mode — run Hailo detection + tracking on video
        pipeline_args = _build_pipeline_args(args.input, args, remaining)
        LOGGER.info("Running Hailo pipeline (%s tracker) on: %s",
                    args.tracker, args.input)
        run_from_pipeline(args.output, args.gt, args, pipeline_args,
                          image_size=image_size)

    elif args.detections:
        # Detections-only mode — no Hailo hardware needed
        if not os.path.isfile(args.detections):
            parser.error(f"Detections file not found: {args.detections}")
        LOGGER.info("Running %s tracker on pre-computed detections: %s",
                    args.tracker, args.detections)
        run_from_detections(args.detections, args.output, args.gt, args)

    else:
        parser.error(
            "Provide either SEQ_DIR, --detections <file>, "
            "or --input <video> (Hailo pipeline mode)")

    # --- Render tracked video for --input mode (source is a video file) ---
    if args.input and args.visualize and os.path.isfile(args.input):
        _render_video_from_video(args.output, args.input, args.visualize,
                                 gt_path=args.gt)

    # --- Visualization from image sequences ---
    images_dir = args.images_dir
    if not images_dir and args.detections:
        # Auto-detect: .../MOT17-04-SDP/det/det.txt → .../MOT17-04-SDP/img1/
        candidate = Path(args.detections).resolve().parent.parent / "img1"
        if candidate.is_dir():
            images_dir = str(candidate)
            LOGGER.info("Auto-detected images dir: %s", images_dir)

    if images_dir:
        if args.visualize:
            _render_video(args.output, images_dir, args.visualize,
                          frame_rate=args.frame_rate, gt_path=args.gt)

        # Pipeline visualization: run Hailo detector → tracker → render video
        if getattr(args, "visualize_pipeline", None):
            import tempfile
            # Load det.txt as reference for detection AP comparison
            ref_dets = None
            if args.detections:
                ref_dets = load_mot_file(args.detections)
                LOGGER.info("Using %s as reference for detection AP", args.detections)

            # Convert image sequence to temporary video for the Hailo pipeline
            tmp_video = tempfile.NamedTemporaryFile(suffix=".mp4", delete=False).name
            try:
                _images_to_video(images_dir, tmp_video, frame_rate=args.frame_rate)
                # Run Hailo pipeline on the video to get detections
                pipeline_output = args.visualize_pipeline.replace(".mp4", "_results.txt")
                pipeline_args = _build_pipeline_args(
                    tmp_video, args, args.run_pipeline_args or [])
                LOGGER.info("Running Hailo pipeline on %s ...", tmp_video)
                run_from_pipeline(pipeline_output, args.gt, args, pipeline_args,
                                  reference_detections=ref_dets,
                                  image_size=image_size)
                # Render the pipeline results as a video
                _render_video(pipeline_output, images_dir, args.visualize_pipeline,
                              frame_rate=args.frame_rate, gt_path=args.gt)
            finally:
                try:
                    os.unlink(tmp_video)
                except OSError:
                    pass

    elif not args.input and (args.visualize or getattr(args, "visualize_pipeline", None)):
        LOGGER.warning("Visualization skipped: no --images-dir found")


if __name__ == "__main__":
    main()
