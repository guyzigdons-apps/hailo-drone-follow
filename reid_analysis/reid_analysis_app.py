#!/usr/bin/env python3
"""
ReID Video Analysis App
=======================
Uses GStreamerTilingApp (same as drone_follow) for person detection with
Hailo NPU tiling pipeline. Extracts ReID embeddings, builds a gallery
from the first frame, and matches subsequent frames against the gallery.

Usage:
    source setup_env.sh
    python reid_analysis_app.py --input 12354541-hd_1280_720_25fps.mp4 \
        --tiles-x 2 --tiles-y 3 --reid-model repvgg --threshold 0.7
"""

import os
import sys
import threading
from pathlib import Path

import cv2
import hailo
import numpy as np

from hailo_apps.python.core.common.buffer_utils import get_caps_from_pad, get_numpy_from_buffer
from hailo_apps.python.core.common.core import get_pipeline_parser
from hailo_apps.python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.python.pipeline_apps.tiling.tiling_pipeline import GStreamerTilingApp

from reid_embedding_extractor import RepVGG512Extractor, OSNetExtractor


# ---------------------------------------------------------------------------
# User data — shared between callback and main thread
# ---------------------------------------------------------------------------

class ReIDUserData(app_callback_class):
    def __init__(self, reid_extractor, output_dir, threshold):
        super().__init__()
        self.reid_extractor = reid_extractor
        self.threshold = threshold

        self.orig_dir = Path(output_dir) / "orig_person_images"
        self.match_dir = Path(output_dir) / "person_images"
        self.orig_dir.mkdir(parents=True, exist_ok=True)
        self.match_dir.mkdir(parents=True, exist_ok=True)

        # Gallery state (grows as new persons are discovered)
        self.gallery_embeddings = []
        self.gallery_names = []

        self.total_crops = 0
        self._lock = threading.Lock()


# ---------------------------------------------------------------------------
# Pipeline callback
# ---------------------------------------------------------------------------

def _add_to_gallery(user_data, crop, emb, frame_count):
    """Add a new person to the gallery. Returns the new person_id."""
    person_id = len(user_data.gallery_embeddings)
    name = f"person_{person_id}"
    user_data.gallery_embeddings.append(emb)
    user_data.gallery_names.append(name)
    # Save first-seen crop as the reference image
    cv2.imwrite(str(user_data.orig_dir / f"{name}.jpg"), crop)
    # Create per-person directory and save this crop there too
    person_dir = user_data.match_dir / name
    person_dir.mkdir(exist_ok=True)
    cv2.imwrite(str(person_dir / f"frame_{frame_count:04d}.jpg"), crop)
    return person_id


def app_callback(element, buffer, user_data):
    if buffer is None:
        return

    frame_count = user_data.get_count()

    # Extract person detections from Hailo metadata
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    persons = [d for d in detections if d.get_label() == "person"]

    if not persons:
        return

    # Extract frame as numpy array
    pad = element.get_static_pad("src")
    format, width, height = get_caps_from_pad(pad)
    frame = get_numpy_from_buffer(buffer, format, width, height)
    if frame is None:
        return

    # Convert RGB to BGR for OpenCV
    if format == "RGB":
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    else:
        frame_bgr = frame

    # Crop persons from frame using normalized bbox coordinates
    crops = []
    for person in persons:
        bbox = person.get_bbox()
        x1 = max(0, int(bbox.xmin() * width))
        y1 = max(0, int(bbox.ymin() * height))
        x2 = min(width, int((bbox.xmin() + bbox.width()) * width))
        y2 = min(height, int((bbox.ymin() + bbox.height()) * height))
        if x2 > x1 and y2 > y1:
            crops.append(frame_bgr[y1:y2, x1:x2].copy())

    if not crops:
        return

    # Extract ReID embeddings
    embeddings = user_data.reid_extractor.extract_embeddings_batch(crops)

    with user_data._lock:
        for j, (crop, emb) in enumerate(zip(crops, embeddings)):
            user_data.total_crops += 1

            if not user_data.gallery_embeddings:
                # First person ever — create new gallery entry
                person_id = _add_to_gallery(user_data, crop, emb, frame_count)
                print(f"Frame {frame_count}: new person_{person_id} (first detection)")
            else:
                # Match against gallery
                gallery_matrix = np.stack(user_data.gallery_embeddings)
                similarities = gallery_matrix @ emb
                best_idx = int(np.argmax(similarities))
                best_sim = float(similarities[best_idx])

                if best_sim >= user_data.threshold:
                    # Matched existing person
                    name = user_data.gallery_names[best_idx]
                    save_path = user_data.match_dir / name / f"frame_{frame_count:04d}_{j}.jpg"
                    cv2.imwrite(str(save_path), crop)
                else:
                    # New person — add to gallery
                    person_id = _add_to_gallery(user_data, crop, emb, frame_count)
                    print(f"Frame {frame_count}: new person_{person_id} (best match {best_sim:.3f} < {user_data.threshold})")

    if frame_count % 100 == 0:
        print(f"  Frame {frame_count} processed, {user_data.total_crops} total crops")




# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    # Build parser with pipeline args + our custom args
    parser = get_pipeline_parser()
    parser.add_argument("--reid-model", type=str, choices=["repvgg", "osnet"], default="repvgg",
                        help="ReID model to use")
    parser.add_argument("--threshold", type=float, default=0.7,
                        help="Cosine similarity threshold for matching")
    parser.add_argument("--output-dir", type=str,
                        default=os.path.dirname(os.path.abspath(__file__)),
                        help="Base output directory")
    # Pre-parse to get our custom args before GStreamerTilingApp consumes them
    args, _ = parser.parse_known_args()

    # Init ReID extractor
    print(f"Loading ReID model: {args.reid_model}")
    if args.reid_model == "repvgg":
        reid_extractor = RepVGG512Extractor()
    else:
        reid_extractor = OSNetExtractor()
    print(f"ReID: {reid_extractor.model_name}, dim={reid_extractor.embedding_dim}")

    # Create user data
    user_data = ReIDUserData(
        reid_extractor=reid_extractor,
        output_dir=args.output_dir,
        threshold=args.threshold,
    )

    # Create and run tiling pipeline app
    app = GStreamerTilingApp(app_callback, user_data, parser=parser)
    app.run()

    # Cleanup
    reid_extractor.release()

    # Summary
    print(f"\nDone!")
    print(f"Total person crops saved: {user_data.total_crops}")
    print(f"Unique persons found: {len(user_data.gallery_names)}")
    print(f"First-seen crops: {user_data.orig_dir}/")
    for name in user_data.gallery_names:
        person_dir = user_data.match_dir / name
        n_crops = len(list(person_dir.glob("*.jpg")))
        print(f"  {name}: {n_crops} crops in {person_dir}/")


if __name__ == "__main__":
    main()
