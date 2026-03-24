#!/usr/bin/env python3
"""
ReID Video Analysis App
=======================
Processes a video file to detect persons (YOLO on Hailo NPU), extract ReID
embeddings, build a gallery from the first frame, and match subsequent frames
against the gallery — saving crops organized by identity.

Usage:
    source setup_env.sh
    python reid_analysis_app.py --video 12354541-hd_1280_720_25fps.mp4
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np

# ── Hailo-apps imports ──
from hailo_apps.python.core.common.hailo_inference import HailoInfer
from hailo_apps.python.core.common.toolbox import default_preprocess
from hailo_apps.python.standalone_apps.object_detection.object_detection_post_process import (
    extract_detections,
)

# ── Local ReID imports ──
from reid_embedding_extractor import RepVGG512Extractor, OSNetExtractor

COCO_PERSON_CLASS = 0

# Tiling config
TILES_X = 2
TILES_Y = 3
TILE_OVERLAP = 0.15  # 15% overlap between tiles
NMS_IOU_THRESHOLD = 0.4


def _compute_iou(box_a, box_b):
    """Compute IoU between two boxes in (ymin, xmin, ymax, xmax) format."""
    y1 = max(box_a[0], box_b[0])
    x1 = max(box_a[1], box_b[1])
    y2 = min(box_a[2], box_b[2])
    x2 = min(box_a[3], box_b[3])
    inter = max(0, y2 - y1) * max(0, x2 - x1)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def _nms(boxes, scores, iou_threshold):
    """Apply NMS on (ymin, xmin, ymax, xmax) boxes. Returns kept indices."""
    if not boxes:
        return []
    order = sorted(range(len(scores)), key=lambda i: scores[i], reverse=True)
    keep = []
    suppressed = set()
    for i in order:
        if i in suppressed:
            continue
        keep.append(i)
        for j in order:
            if j in suppressed or j == i:
                continue
            if _compute_iou(boxes[i], boxes[j]) > iou_threshold:
                suppressed.add(j)
    return keep


def _run_yolo_single(hailo_infer, image, model_w, model_h, config_data):
    """Run YOLO on a single image, return person boxes as (ymin, xmin, ymax, xmax) in image pixel coords and scores."""
    preprocessed = default_preprocess(image, model_w, model_h)
    bindings = hailo_infer._create_bindings(hailo_infer.configured_model, [preprocessed])
    hailo_infer.configured_model.wait_for_async_ready(timeout_ms=10000)
    job = hailo_infer.configured_model.run_async(bindings, lambda *args, **kwargs: None)
    job.wait(timeout_ms=10000)

    result = bindings[0].output().get_buffer()
    detections = extract_detections(image, result, config_data)

    boxes, scores = [], []
    for i in range(detections["num_detections"]):
        if detections["detection_classes"][i] == COCO_PERSON_CLASS:
            boxes.append(detections["detection_boxes"][i])
            scores.append(detections["detection_scores"][i])
    return boxes, scores


def detect_persons(hailo_infer, frame, model_w, model_h, config_data):
    """
    Run YOLO detection with 2x3 tiling on a single frame.
    Splits the frame into overlapping tiles, runs YOLO on each,
    maps detections back to full-frame coordinates, and applies NMS.

    Returns:
        List of (ymin, xmin, ymax, xmax) pixel-coordinate tuples for persons.
    """
    frame_h, frame_w = frame.shape[:2]

    # Compute tile sizes with overlap
    tile_w = int(frame_w / (TILES_X - (TILES_X - 1) * TILE_OVERLAP))
    tile_h = int(frame_h / (TILES_Y - (TILES_Y - 1) * TILE_OVERLAP))
    step_x = int(tile_w * (1 - TILE_OVERLAP))
    step_y = int(tile_h * (1 - TILE_OVERLAP))

    all_boxes = []
    all_scores = []

    for ty in range(TILES_Y):
        for tx in range(TILES_X):
            x_start = min(tx * step_x, frame_w - tile_w)
            y_start = min(ty * step_y, frame_h - tile_h)
            x_start = max(0, x_start)
            y_start = max(0, y_start)
            x_end = min(x_start + tile_w, frame_w)
            y_end = min(y_start + tile_h, frame_h)

            tile = frame[y_start:y_end, x_start:x_end]
            tile_boxes, tile_scores = _run_yolo_single(hailo_infer, tile, model_w, model_h, config_data)

            # Map tile-local pixel coords back to full frame
            for box, score in zip(tile_boxes, tile_scores):
                ymin, xmin, ymax, xmax = box
                all_boxes.append((ymin + y_start, xmin + x_start, ymax + y_start, xmax + x_start))
                all_scores.append(score)

    # Apply NMS to merge duplicates from overlapping tiles
    kept = _nms(all_boxes, all_scores, NMS_IOU_THRESHOLD)
    return [all_boxes[i] for i in kept]


def crop_persons(frame, boxes):
    """Crop person regions from the frame. Returns list of BGR crops."""
    h, w = frame.shape[:2]
    crops = []
    for box in boxes:
        ymin, xmin, ymax, xmax = box
        ymin, xmin = max(0, int(ymin)), max(0, int(xmin))
        ymax, xmax = min(h, int(ymax)), min(w, int(xmax))
        if ymax > ymin and xmax > xmin:
            crops.append(frame[ymin:ymax, xmin:xmax].copy())
    return crops


def match_to_gallery(query_embedding, gallery_matrix, threshold):
    """
    Match a query embedding against the gallery.
    Returns (person_id, similarity) if match found, else (None, best_sim).
    """
    if gallery_matrix.shape[0] == 0:
        return None, 0.0
    similarities = gallery_matrix @ query_embedding
    best_idx = int(np.argmax(similarities))
    best_sim = float(similarities[best_idx])
    if best_sim >= threshold:
        return best_idx, best_sim
    return None, best_sim


def main():
    parser = argparse.ArgumentParser(description="ReID Video Analysis — detect, embed, match persons across frames")
    parser.add_argument("--video", type=str, default="12354541-hd_1280_720_25fps.mp4", help="Input video file")
    parser.add_argument("--yolo-hef", type=str, default="/usr/local/hailo/resources/models/hailo8/yolov8m.hef",
                        help="YOLO HEF model path")
    parser.add_argument("--reid-model", type=str, choices=["repvgg", "osnet"], default="repvgg",
                        help="ReID model to use")
    parser.add_argument("--threshold", type=float, default=0.7, help="Cosine similarity threshold for matching")
    parser.add_argument("--score-threshold", type=float, default=0.5, help="YOLO confidence threshold")
    parser.add_argument("--output-dir", type=str, default=".", help="Base output directory")
    args = parser.parse_args()

    # ── Validate video ──
    if not os.path.isfile(args.video):
        print(f"Video not found: {args.video}")
        sys.exit(1)

    # ── Output dirs ──
    output_dir = Path(args.output_dir)
    orig_dir = output_dir / "orig_person_images"
    match_dir = output_dir / "person_images"
    orig_dir.mkdir(parents=True, exist_ok=True)
    match_dir.mkdir(parents=True, exist_ok=True)

    # ── Config for YOLO post-processing ──
    config_data = {
        "visualization_params": {
            "score_thres": args.score_threshold,
            "max_boxes_to_draw": 50,
        }
    }

    # ── Init models ──
    print(f"Loading YOLO model: {args.yolo_hef}")
    hailo_infer = HailoInfer(args.yolo_hef, batch_size=1)
    input_shape = hailo_infer.get_input_shape()
    if len(input_shape) == 4:
        model_h, model_w = input_shape[1], input_shape[2]
    else:
        model_h, model_w = input_shape[0], input_shape[1]
    print(f"YOLO input: {model_w}x{model_h}")

    print(f"Loading ReID model: {args.reid_model}")
    if args.reid_model == "repvgg":
        reid_extractor = RepVGG512Extractor()
    else:
        reid_extractor = OSNetExtractor()
    print(f"ReID: {reid_extractor.model_name}, dim={reid_extractor.embedding_dim}")

    # ── Open video ──
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"Cannot open video: {args.video}")
        sys.exit(1)

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Video: {total_frames} frames, {fps:.1f} FPS")

    # ── Gallery state ──
    gallery_embeddings = []  # List of 1D embedding vectors
    gallery_names = []       # List of person names ("person_0", "person_1", ...)

    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_idx += 1

        # Detect persons
        person_boxes = detect_persons(hailo_infer, frame, model_w, model_h, config_data)

        if not person_boxes:
            if frame_idx % 100 == 0:
                print(f"  Frame {frame_idx}/{total_frames}: no persons detected")
            continue

        # Crop persons
        crops = crop_persons(frame, person_boxes)
        if not crops:
            continue

        # Extract ReID embeddings
        embeddings = reid_extractor.extract_embeddings_batch(crops)

        if frame_idx == 1:
            # ── First frame: build gallery ──
            for i, (crop, emb) in enumerate(zip(crops, embeddings)):
                name = f"person_{i}"
                gallery_embeddings.append(emb)
                gallery_names.append(name)

                # Save original crop
                cv2.imwrite(str(orig_dir / f"{name}.jpg"), crop)

                # Create match directory
                (match_dir / name).mkdir(exist_ok=True)

            gallery_matrix = np.stack(gallery_embeddings)  # (N, D)
            print(f"Frame 1: gallery built with {len(gallery_names)} persons: {gallery_names}")
        else:
            # ── Subsequent frames: match against gallery ──
            for crop, emb in zip(crops, embeddings):
                person_id, sim = match_to_gallery(emb, gallery_matrix, args.threshold)
                if person_id is not None:
                    name = gallery_names[person_id]
                    save_path = match_dir / name / f"frame_{frame_idx}.jpg"
                    cv2.imwrite(str(save_path), crop)

        if frame_idx % 100 == 0:
            print(f"  Frame {frame_idx}/{total_frames} processed")

    cap.release()
    hailo_infer.close()
    reid_extractor.release()

    # ── Summary ──
    print(f"\nDone! Processed {frame_idx} frames.")
    print(f"Gallery: {len(gallery_names)} persons")
    print(f"Original crops: {orig_dir}/")
    for name in gallery_names:
        person_dir = match_dir / name
        n_matches = len(list(person_dir.glob("*.jpg")))
        print(f"  {name}: {n_matches} matched frames in {person_dir}/")


if __name__ == "__main__":
    main()
