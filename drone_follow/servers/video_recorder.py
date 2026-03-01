"""Video recorder that captures raw frames from SharedUIState and logs detections.

On stop, writes a raw video + detection JSON, then burns annotations into an
annotated copy using OpenCV.
"""

import json
import logging
import os
import threading
import time
from datetime import datetime

import cv2
import numpy as np

LOGGER = logging.getLogger("drone_follow.video_recorder")


def generate_annotated_video(raw_video_path: str, detections_json_path: str, output_path: str) -> None:
    """Burn bounding-box annotations onto a raw video using a detections JSON file.

    The JSON file is a list of per-frame records:
        [{"frame_idx": 0, "timestamp": 1.23, "detections": [...]}, ...]
    Each detection has: {"bbox": {"x": .., "y": .., "w": .., "h": ..},
                         "confidence": .., "id": .., "label": ..}
    """
    LOGGER.info("Generating annotated video: %s", output_path)

    with open(detections_json_path, "r") as f:
        records = json.load(f)

    # Build frame_idx -> detections lookup
    dets_by_frame = {r["frame_idx"]: r["detections"] for r in records}

    cap = cv2.VideoCapture(raw_video_path)
    if not cap.isOpened():
        LOGGER.error("Cannot open raw video: %s", raw_video_path)
        return

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 15.0

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        for det in dets_by_frame.get(frame_idx, []):
            bbox = det["bbox"]
            x = int(bbox["x"] * width)
            y = int(bbox["y"] * height)
            w = int(bbox["w"] * width)
            h = int(bbox["h"] * height)

            color = (0, 255, 0) if det.get("following") else (0, 255, 255)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

            conf = det.get("confidence", 0)
            tid = det.get("id")
            label = f"ID:{tid} " if tid is not None else ""
            label += f"{conf:.0%}"
            cv2.putText(frame, label, (x, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        out.write(frame)
        frame_idx += 1

    cap.release()
    out.release()
    LOGGER.info("Annotated video written: %s (%d frames)", output_path, frame_idx)


class VideoRecorder:
    """Records raw video frames + detection metadata from SharedUIState.

    Usage::

        rec = VideoRecorder(ui_state, output_dir="recordings")
        rec.start()      # begins background capture
        ...
        rec.stop()        # finalizes video, writes JSON, generates annotated copy
    """

    def __init__(self, ui_state, output_dir: str = "recordings", fps: float = 15.0):
        self._ui_state = ui_state
        self._output_dir = output_dir
        self._fps = fps

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # Paths filled on start
        self._raw_path: str | None = None
        self._json_path: str | None = None
        self._annotated_path: str | None = None

    def start(self) -> None:
        os.makedirs(self._output_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self._raw_path = os.path.join(self._output_dir, f"rec_{ts}.mp4")
        self._json_path = os.path.join(self._output_dir, f"rec_{ts}_detections.json")
        self._annotated_path = os.path.join(self._output_dir, f"rec_{ts}_annotated.mp4")

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        LOGGER.info("VideoRecorder started -> %s", self._raw_path)

    def stop(self) -> None:
        """Signal stop, wait for writer thread, then generate annotated video."""
        if self._thread is None:
            return
        self._stop_event.set()
        self._thread.join(timeout=10.0)
        self._thread = None

        # Post-process: burn annotations
        if (self._raw_path and os.path.isfile(self._raw_path)
                and self._json_path and os.path.isfile(self._json_path)):
            try:
                generate_annotated_video(self._raw_path, self._json_path, self._annotated_path)
            except Exception:
                LOGGER.exception("Failed to generate annotated video")

    # ------------------------------------------------------------------
    # Background thread
    # ------------------------------------------------------------------

    def _run(self) -> None:
        writer: cv2.VideoWriter | None = None
        detection_log: list[dict] = []
        frame_idx = 0
        frame_interval = 1.0 / self._fps

        try:
            while not self._stop_event.is_set():
                jpeg = self._ui_state.wait_frame(timeout=2.0)
                if jpeg is None:
                    continue

                # Decode JPEG -> BGR
                arr = np.frombuffer(jpeg, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                # Lazy-init writer once we know the frame size
                if writer is None:
                    h, w = frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    writer = cv2.VideoWriter(self._raw_path, fourcc, self._fps, (w, h))
                    if not writer.isOpened():
                        LOGGER.error("Failed to open VideoWriter for %s", self._raw_path)
                        return

                writer.write(frame)

                # Log detections for this frame
                det_data = self._ui_state.get_detections()
                detections = det_data.get("detections", [])
                following_id = det_data.get("following_id")
                for d in detections:
                    d["following"] = (d.get("id") is not None and d.get("id") == following_id)

                detection_log.append({
                    "frame_idx": frame_idx,
                    "timestamp": time.time(),
                    "detections": detections,
                })
                frame_idx += 1

                # Pace to target FPS
                time.sleep(frame_interval)

        except Exception:
            LOGGER.exception("VideoRecorder thread error")
        finally:
            if writer is not None:
                writer.release()
            # Write detection JSON
            if detection_log and self._json_path:
                try:
                    with open(self._json_path, "w") as f:
                        json.dump(detection_log, f)
                    LOGGER.info("Detections written: %s (%d frames)", self._json_path, len(detection_log))
                except Exception:
                    LOGGER.exception("Failed to write detection JSON")
