"""Hailo tiling pipeline adapter — all Hailo/GStreamer imports are confined here.

Translates Hailo detection objects into the pure Detection domain type.
No other module needs to import hailo or gi.repository.
"""

import argparse
import logging
import os
import threading
import time
from collections import deque
from datetime import datetime
from typing import Optional

import hailo
import numpy as np

from drone_follow.follow_api.types import Detection

from .byte_tracker import iou_batch
from .tracker import MetricsTracker
from .tracker_factory import create_tracker

LOGGER = logging.getLogger(__name__)

_EMPTY_DET_ARRAY = np.empty((0, 5), dtype=np.float32)


# ---------------------------------------------------------------------------
# Performance tracker
# ---------------------------------------------------------------------------

def _parse_device_utilization(data: bytes) -> float:
    """Extract device utilization (%) from a Hailo monitor protobuf file.

    The monitor file uses protobuf encoding. Field 4 is the device info
    message containing sub-field 2 as a fixed64 (double) with the device
    NN core utilization percentage.

    Returns 0.0 if the field cannot be found.
    """
    import struct

    pos = 0
    while pos < len(data):
        if pos >= len(data):
            break
        # Read varint tag
        tag_byte = data[pos]
        pos += 1
        field_number = tag_byte >> 3
        wire_type = tag_byte & 0x7

        if wire_type == 0:  # varint
            while pos < len(data) and data[pos] & 0x80:
                pos += 1
            pos += 1
        elif wire_type == 1:  # fixed64
            pos += 8
        elif wire_type == 5:  # fixed32
            pos += 4
        elif wire_type == 2:  # length-delimited
            length = 0
            shift = 0
            while pos < len(data):
                b = data[pos]
                pos += 1
                length |= (b & 0x7F) << shift
                if not (b & 0x80):
                    break
                shift += 7
            if field_number == 4:
                # Parse sub-fields of the device info message
                end = pos + length
                sub_pos = pos
                while sub_pos < end:
                    sub_tag = data[sub_pos]
                    sub_pos += 1
                    sub_field = sub_tag >> 3
                    sub_wire = sub_tag & 0x7
                    if sub_wire == 1 and sub_field == 2:  # fixed64 = utilization
                        return struct.unpack("<d", data[sub_pos:sub_pos + 8])[0]
                    elif sub_wire == 0:
                        while sub_pos < end and data[sub_pos] & 0x80:
                            sub_pos += 1
                        sub_pos += 1
                    elif sub_wire == 1:
                        sub_pos += 8
                    elif sub_wire == 5:
                        sub_pos += 4
                    elif sub_wire == 2:
                        sub_len = 0
                        s = 0
                        while sub_pos < end:
                            b = data[sub_pos]
                            sub_pos += 1
                            sub_len |= (b & 0x7F) << s
                            if not (b & 0x80):
                                break
                            s += 7
                        sub_pos += sub_len
                    else:
                        break
            pos += length
        else:
            break
    return 0.0


class _PerfTracker:
    """Lightweight performance tracker for the pipeline callback."""

    def __init__(self, *, log_perf=False, tracker_metrics=None):
        self._frame_times = deque(maxlen=60)
        self._latencies = deque(maxlen=60)
        # CPU sampling state
        self._last_cpu_total = 0
        self._last_cpu_idle = 0
        self._cpu_percent = 0.0
        # Hailo device handle (lazy)
        self._hailo_device = None
        self._hailo_init_tried = False
        # Cached values
        self._hailo_temp = 0.0
        self._hailo_utilization = 0.0
        self._memory_mb = 0.0
        self._last_system_sample = 0.0
        # Periodic logging
        self._log_perf = log_perf
        self._tracker_metrics = tracker_metrics
        self._last_log_time = 0.0

    def frame_start(self):
        return time.monotonic()

    def frame_end(self, t0, ui_state):
        now = time.monotonic()
        self._frame_times.append(now)
        self._latencies.append((now - t0) * 1000)
        # Sample system metrics every ~2 seconds
        if now - self._last_system_sample > 2.0:
            self._last_system_sample = now
            self._sample_cpu()
            self._sample_memory()
            self._sample_hailo_temp()
            self._sample_hailo_utilization()
        # Push to UI every frame (values are cached between system samples)
        if ui_state is not None:
            ui_state.update_perf(self.get_stats())
        # Periodic logging (~5 seconds)
        if self._log_perf and now - self._last_log_time >= 5.0:
            self._last_log_time = now
            stats = self.get_stats()
            parts = [f"fps={stats['fps']}", f"latency={stats['latency_ms']}ms",
                     f"cpu={stats['cpu_percent']}%", f"mem={stats['memory_mb']}MB",
                     f"hailo_temp={stats['hailo_temp_c']}C",
                     f"hailo_util={stats['hailo_util_percent']}%"]
            if self._tracker_metrics is not None:
                tm = self._tracker_metrics.snapshot()
                parts.extend([f"tracker_fps={tm['fps']}",
                              f"tracker_update={tm['update_ms']}ms",
                              f"tracks={tm['active_tracks']}",
                              f"id_sw={tm['id_switches']}"])
            LOGGER.info("[PERF] %s", " | ".join(parts))

    def get_stats(self):
        ft = self._frame_times
        if len(ft) > 1:
            fps = (len(ft) - 1) / (ft[-1] - ft[0])
        else:
            fps = 0.0
        lat = sum(self._latencies) / len(self._latencies) if self._latencies else 0.0
        return {
            "fps": round(fps, 1),
            "latency_ms": round(lat, 1),
            "cpu_percent": round(self._cpu_percent, 1),
            "memory_mb": round(self._memory_mb, 0),
            "hailo_temp_c": round(self._hailo_temp, 1),
            "hailo_util_percent": round(self._hailo_utilization, 1),
        }

    # -- System sampling helpers --

    def _sample_cpu(self):
        try:
            with open("/proc/stat", "r") as f:
                parts = f.readline().split()
            total = sum(int(x) for x in parts[1:8])
            idle = int(parts[4])
            d_total = total - self._last_cpu_total
            d_idle = idle - self._last_cpu_idle
            if d_total > 0:
                self._cpu_percent = 100.0 * (1.0 - d_idle / d_total)
            self._last_cpu_total = total
            self._last_cpu_idle = idle
        except (OSError, ValueError, IndexError):
            pass

    def _sample_memory(self):
        try:
            with open("/proc/self/status", "r") as f:
                for line in f:
                    if line.startswith("VmRSS:"):
                        self._memory_mb = int(line.split()[1]) / 1024.0
                        return
        except (OSError, ValueError, IndexError):
            pass

    def _sample_hailo_temp(self):
        if not self._hailo_init_tried:
            self._hailo_init_tried = True
            try:
                from hailo_platform import Device
                self._hailo_device = Device()
            except (ImportError, OSError):
                pass
        if self._hailo_device is None:
            return
        try:
            temp = self._hailo_device.control.get_chip_temperature()
            self._hailo_temp = temp.ts0_temperature
        except (OSError, AttributeError):
            pass

    def _sample_hailo_utilization(self):
        """Read NN core utilization from the Hailo monitor protobuf file.

        Requires HAILO_MONITOR=1 in the process environment. The HailoRT
        runtime writes a protobuf file per process under /tmp/hmon_files/
        containing device utilization as a double in field 4.2.
        """
        try:
            hmon_dir = "/tmp/hmon_files"
            pid = str(os.getpid())
            for fname in os.listdir(hmon_dir):
                fpath = os.path.join(hmon_dir, fname)
                with open(fpath, "rb") as f:
                    data = f.read()
                # Quick check: file starts with field 1 (tag 0x0a) containing our PID
                if len(data) < 4:
                    continue
                # Protobuf field 1 (string): tag=0x0a, then varint length, then PID bytes
                tag = data[0]
                if tag != 0x0a:
                    continue
                pid_len = data[1]
                file_pid = data[2:2 + pid_len].decode("ascii", errors="ignore")
                if file_pid != pid:
                    continue
                # Found our file — extract device utilization from field 4
                self._hailo_utilization = _parse_device_utilization(data)
                return
        except (OSError, IndexError, ValueError):
            pass

_gst_module = None


def _get_gst():
    """Import and cache GStreamer bindings (deferred to avoid import at module level)."""
    global _gst_module
    if _gst_module is None:
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        _gst_module = Gst
    return _gst_module


# ---------------------------------------------------------------------------
# Callback helpers
# ---------------------------------------------------------------------------

def _build_det_info(person, track_id=None):
    """Build a UI detection dict from a Hailo detection object."""
    pbbox = person.get_bbox()
    det_info = {
        "label": "person",
        "confidence": round(person.get_confidence(), 3),
        "bbox": {
            "x": round(pbbox.xmin(), 4),
            "y": round(pbbox.ymin(), 4),
            "w": round(pbbox.width(), 4),
            "h": round(pbbox.height(), 4),
        },
    }
    if track_id is not None:
        det_info["id"] = track_id
    return det_info


def _update_ui(ui_state, persons, person_to_id, following_id):
    """Push detection metadata to the web UI if enabled."""
    if ui_state is None:
        return
    all_dets = [_build_det_info(p, person_to_id.get(id(p))) for p in persons]
    ui_state.update_detections(all_dets, following_id)


def _run_tracker(tracker, persons):
    """Run tracker and return (available_ids, person_by_id, person_to_id).

    person_by_id:  {track_id -> person detection}
    person_to_id:  {id(person) -> track_id}  (reverse lookup)
    """
    available_ids = set()
    person_by_id = {}

    SCALE = 1000.0
    det_array = np.empty((len(persons), 5), dtype=np.float32)
    for i, person in enumerate(persons):
        bbox = person.get_bbox()
        det_array[i, 0] = bbox.xmin() * SCALE
        det_array[i, 1] = bbox.ymin() * SCALE
        det_array[i, 2] = (bbox.xmin() + bbox.width()) * SCALE
        det_array[i, 3] = (bbox.ymin() + bbox.height()) * SCALE
        det_array[i, 4] = person.get_confidence()

    all_tracks = tracker.update(det_array)

    for t in all_tracks:
        if t.is_activated and 0 <= t.input_index < len(persons):
            available_ids.add(t.track_id)
            person_by_id[t.track_id] = persons[t.input_index]
        elif t.is_activated:
            available_ids.add(t.track_id)

    person_to_id = {id(p): tid for tid, p in person_by_id.items()}
    return available_ids, person_by_id, person_to_id


_SOT_IOU_THRESH = 0.3
_SOT_MOT_REFRESH_INTERVAL = 150  # frames (~5s at 30fps)


def _run_sot(persons, last_bbox):
    """Lightweight single-object tracking: IOU match against last known bbox.

    Returns (matched_person, new_bbox_scaled) or (None, None) if lost.
    last_bbox is [x1, y1, x2, y2] in SCALE (1000) coordinates.
    """
    SCALE = 1000.0
    n = len(persons)
    det_bboxes = np.empty((n, 4), dtype=np.float32)
    for i, person in enumerate(persons):
        bbox = person.get_bbox()
        det_bboxes[i, 0] = bbox.xmin() * SCALE
        det_bboxes[i, 1] = bbox.ymin() * SCALE
        det_bboxes[i, 2] = (bbox.xmin() + bbox.width()) * SCALE
        det_bboxes[i, 3] = (bbox.ymin() + bbox.height()) * SCALE

    ious = iou_batch(last_bbox.reshape(1, 4), det_bboxes)  # shape (1, N)
    if ious.size == 0:
        return None, None

    best_idx = np.argmax(ious[0])
    if ious[0, best_idx] < _SOT_IOU_THRESH:
        return None, None

    return persons[best_idx], det_bboxes[best_idx]


# ---------------------------------------------------------------------------
# ReID frame extraction helper
# ---------------------------------------------------------------------------

_buffer_utils = None

def _get_frame_bgr(buffer, user_data):
    """Extract BGR frame from GStreamer buffer for ReID cropping.

    Only called when ReID needs a frame (gallery update or re-identification).
    """
    global _buffer_utils
    if _buffer_utils is None:
        from hailo_apps.python.core.common import buffer_utils
        _buffer_utils = buffer_utils
    try:
        import cv2
        frame_rgb = _buffer_utils.get_numpy_from_buffer(
            buffer, "RGB", user_data.video_width, user_data.video_height)
        if frame_rgb is not None:
            return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    except Exception as e:
        LOGGER.debug("[REID] Frame extraction failed: %s", e)
    return None


# ---------------------------------------------------------------------------
# Main app callback
# ---------------------------------------------------------------------------

def app_callback(element, buffer, user_data):
    """Tiling pipeline callback: follow operator-selected person, update shared state.

    Tracker runs synchronously in the callback:
    1. Convert detections to Nx5 array, run tracker.update() synchronously
    2. Each returned track has input_index pointing to the matched detection
    3. Build person_by_id directly -- no cross-frame IoU re-matching needed
    """
    _perf_t0 = user_data.perf.frame_start()
    try:
        _app_callback_inner(element, buffer, user_data)
    finally:
        user_data.perf.frame_end(_perf_t0, user_data.ui_state)


def _app_callback_inner(element, buffer, user_data):
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    persons = [d for d in detections if d.get_label() == "person"]

    target_state = user_data.target_state
    ui_state = user_data.ui_state

    if not persons:
        user_data.tracker.update(_EMPTY_DET_ARRAY)
        user_data.sot_active = False
        user_data.sot_last_bbox = None
        user_data.sot_frames = 0
        user_data.shared_state.update(None, available_ids=set())
        if target_state is not None and target_state.get_target() is not None:
            # If ReID has a gallery, keep the target set so we keep searching
            # when persons reappear. Otherwise give up immediately.
            reid_mgr = user_data.reid_manager
            if reid_mgr is not None and reid_mgr.has_gallery:
                LOGGER.debug("[REID SEARCH] No persons in frame — holding target ID %s, waiting",
                             target_state.get_target())
            else:
                was_explicit = target_state.is_explicit_lock()
                target_state.set_target(None)
                if was_explicit:
                    target_state.set_paused(True)
                    target_state.set_explicit_lock(False)
                    LOGGER.info("[IDLE FALLBACK] Explicit lock lost (no persons) — entering idle")
        _update_ui(ui_state, [], {}, None)
        if target_state is None or target_state.get_target() is None:
            LOGGER.debug("[SEARCH MODE] No person detected in frame")
        return

    # --- SOT/MOT dispatch ---
    target_id = target_state.get_target() if target_state is not None else None
    reid_manager = user_data.reid_manager

    use_sot = (
        target_id is not None
        and user_data.sot_active
        and user_data.sot_last_bbox is not None
        and user_data.sot_target_id == target_id
        and user_data.sot_frames < _SOT_MOT_REFRESH_INTERVAL
    )

    if use_sot:
        matched, new_bbox = _run_sot(persons, user_data.sot_last_bbox)
        if matched is not None:
            # SOT succeeded
            user_data.sot_last_bbox = new_bbox
            user_data.sot_frames += 1
            matched.add_object(hailo.HailoUniqueID(target_id, hailo.TRACKING_ID))
            person_by_id = {target_id: matched}
            person_to_id = {id(matched): target_id}
            available_ids = {target_id}
        else:
            # SOT lost the target — fall back to MOT
            LOGGER.info("[SOT→MOT] Target lost after %d SOT frames — resetting tracker",
                        user_data.sot_frames)
            user_data.sot_active = False
            user_data.sot_last_bbox = None
            user_data.sot_frames = 0
            user_data.tracker.reset()
            available_ids, person_by_id, person_to_id = _run_tracker(
                user_data.tracker, persons)
            for person in persons:
                tid = person_to_id.get(id(person))
                if tid is not None:
                    person.add_object(hailo.HailoUniqueID(tid, hailo.TRACKING_ID))
    else:
        # Full MOT — no target, SOT not active, or periodic refresh
        if user_data.sot_active and user_data.sot_frames >= _SOT_MOT_REFRESH_INTERVAL:
            LOGGER.debug("[SOT] Periodic MOT refresh after %d frames", user_data.sot_frames)
            user_data.tracker.reset()
            user_data.sot_frames = 0
        available_ids, person_by_id, person_to_id = _run_tracker(
            user_data.tracker, persons)
        for person in persons:
            tid = person_to_id.get(id(person))
            if tid is not None:
                person.add_object(hailo.HailoUniqueID(tid, hailo.TRACKING_ID))

    # --- Target selection ---
    best = None
    follow_mode = ""
    if target_id is not None:
        best = person_by_id.get(target_id)

        if best is not None:
            # Successfully tracking target
            target_state.update_last_seen()
            follow_mode = f"ID {target_id}"

            # ReID: build/update gallery while following
            if reid_manager is not None:
                reid_manager.on_target_selected(target_id)
                if reid_manager.should_update():
                    frame_bgr = _get_frame_bgr(buffer, user_data)
                    if frame_bgr is not None:
                        reid_manager.update_gallery(
                            frame_bgr, best.get_bbox(),
                            user_data.video_width, user_data.video_height)

            # Activate SOT for next frame
            SCALE = 1000.0
            tbbox = best.get_bbox()
            user_data.sot_last_bbox = np.array([
                tbbox.xmin() * SCALE,
                tbbox.ymin() * SCALE,
                (tbbox.xmin() + tbbox.width()) * SCALE,
                (tbbox.ymin() + tbbox.height()) * SCALE,
            ], dtype=np.float32)
            user_data.sot_target_id = target_id
            if not user_data.sot_active:
                user_data.sot_active = True
                user_data.sot_frames = 0
                LOGGER.info("[MOT→SOT] Entering SOT mode for target ID %d", target_id)
        else:
            # Target lost by tracker — try ReID re-identification
            user_data.sot_active = False
            user_data.sot_last_bbox = None
            user_data.sot_frames = 0

            if reid_manager is not None and reid_manager.has_gallery and person_by_id:
                frame_bgr = _get_frame_bgr(buffer, user_data)
                if frame_bgr is not None:
                    new_tid = reid_manager.try_reidentify(
                        frame_bgr, person_by_id,
                        user_data.video_width, user_data.video_height)
                    if new_tid is not None:
                        # Re-identified — resume following with the new track ID
                        target_state.set_target(new_tid)
                        reid_manager.on_reidentified(new_tid)
                        best = person_by_id[new_tid]
                        target_state.update_last_seen()
                        follow_mode = f"REID→ID {new_tid}"

            if best is None:
                # Target not recovered — hold position
                user_data.shared_state.update(None, available_ids=available_ids)
                if reid_manager is not None and reid_manager.has_gallery:
                    # Keep target set so we retry ReID on the next frame
                    _update_ui(ui_state, persons, person_to_id, None)
                    return
                # No ReID gallery — give up
                was_explicit = target_state.is_explicit_lock()
                if target_state.get_target() is not None:
                    target_state.set_target(None)
                if was_explicit:
                    target_state.set_paused(True)
                    target_state.set_explicit_lock(False)
                    LOGGER.info("[IDLE FALLBACK] Explicit lock on ID %s lost — entering idle. Available: %s",
                                target_id, sorted(available_ids) if available_ids else "none")
                else:
                    LOGGER.debug("[SEARCH MODE] Target ID %s not in frame. Available: %s",
                                target_id, sorted(available_ids) if available_ids else "none")
                _update_ui(ui_state, persons, person_to_id, None)
                return
    else:
        # No target — idle: hold position until operator picks a followee
        user_data.sot_active = False
        user_data.sot_last_bbox = None
        user_data.sot_frames = 0
        user_data.shared_state.update(None, available_ids=available_ids)
        _update_ui(ui_state, persons, person_to_id, None)
        LOGGER.debug("[IDLE] No target set. Available: %s",
                    sorted(available_ids) if available_ids else "none")
        return

    bbox = best.get_bbox()
    cx = bbox.xmin() + bbox.width() / 2
    cy = bbox.ymin() + bbox.height() / 2
    user_data.shared_state.update(Detection(
        label="person",
        confidence=best.get_confidence(),
        center_x=cx,
        center_y=cy,
        bbox_height=bbox.height(),
        timestamp=time.monotonic(),
    ), available_ids=available_ids)

    # Use the original ID for the UI so the operator sees a stable ID
    # even after ReID re-identifies the person with a new tracker ID.
    ui_following_id = target_state.get_target() if target_state else None
    ui_person_to_id = person_to_id
    if reid_manager is not None and reid_manager.original_id is not None:
        orig = reid_manager.original_id
        cur = target_state.get_target() if target_state else None
        if orig != cur and cur is not None:
            # Remap the followed detection's ID to the original so the
            # green highlight and "Following: ID X" both use it.
            ui_following_id = orig
            ui_person_to_id = dict(person_to_id)
            ui_person_to_id[id(best)] = orig
    _update_ui(ui_state, persons, ui_person_to_id, ui_following_id)

    available_str = f"Available: {sorted(available_ids)}" if available_ids else ""
    LOGGER.debug("[FOLLOWING %s] conf=%.2f center=(%.2f,%.2f) h=%.2f %s",
                follow_mode, best.get_confidence(), cx, cy, bbox.height(), available_str)


# ---------------------------------------------------------------------------
# OpenHD pipeline helpers (local to drone-follow; not in hailo-apps core)
# ---------------------------------------------------------------------------

def _openhd_stream_pipeline(port=5500, host="127.0.0.1", bitrate=3917, name="openhd_stream"):
    """H264 SW encode + RTP + UDP sink for OpenHD input.

    Uses x264enc with ultrafast/zerolatency settings.
    RPi5 has no hardware H264 encoder; Hailo inference runs on the accelerator,
    leaving CPU available for software encoding.
    """
    from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import QUEUE
    encoder = (
        f"x264enc name={name}_encoder bitrate={bitrate} "
        f"speed-preset=ultrafast tune=zerolatency "
        f"sliced-threads=false threads=2 key-int-max=5"
    )
    return (
        f"{QUEUE(name=f'{name}_convert_q')} ! "
        f"videoconvert n-threads=2 ! video/x-raw,format=I420 ! "
        f"{QUEUE(name=f'{name}_enc_q')} ! "
        f"{encoder} ! "
        f"rtph264pay config-interval=1 pt=96 mtu=1440 ! "
        f"udpsink host={host} port={port} sync=false async=false"
    )


# Sideband metadata file written by OpenHD with current SHM resolution
_SHM_META_PATH = "/tmp/openhd_raw_video.meta"


def _read_shm_resolution():
    """Read current SHM resolution from OpenHD's sideband metadata file.

    Returns (width, height, fps) or None if the file doesn't exist or is invalid.
    OpenHD writes this file every time the camera pipeline (re)starts, so it
    always reflects the active capture resolution.
    """
    import json as _json
    try:
        with open(_SHM_META_PATH, "r") as f:
            meta = _json.loads(f.read())
        w = int(meta["width"])
        h = int(meta["height"])
        fps = int(meta.get("fps", 30))
        if w > 0 and h > 0 and fps > 0:
            return (w, h, fps)
    except (FileNotFoundError, KeyError, ValueError, _json.JSONDecodeError) as e:
        LOGGER.debug("Cannot read SHM metadata from %s: %s", _SHM_META_PATH, e)
    return None


def _shm_source_pipeline(video_source, video_width, video_height, frame_rate, name="source"):
    """Build a GStreamer source pipeline for OpenHD shared-memory NV12 passthrough.

    shmsrc buffers are read-only (mmap'd shared memory).  Force an immediate
    NV12->I420 conversion to create writable buffers (cheap UV deinterleave).

    The caps MUST match the resolution that OpenHD is actually writing into
    shared memory.  We read the sideband metadata file that OpenHD writes on
    every pipeline (re)start to auto-detect the correct resolution, falling
    back to the caller-supplied video_width/video_height if the file is absent.
    """
    from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import QUEUE

    # Auto-detect resolution from OpenHD metadata
    shm_res = _read_shm_resolution()
    if shm_res is not None:
        shm_w, shm_h, shm_fps = shm_res
        if shm_w != video_width or shm_h != video_height or shm_fps != frame_rate:
            LOGGER.info(
                "SHM resolution from metadata (%dx%d@%d) differs from "
                "CLI/defaults (%dx%d@%d) — using metadata values",
                shm_w, shm_h, shm_fps, video_width, video_height, frame_rate,
            )
            video_width = shm_w
            video_height = shm_h
            frame_rate = shm_fps

    socket_path = str(video_source).split('://', 1)[1]

    source_element = (
        f'shmsrc socket-path={socket_path} do-timestamp=true is-live=true name={name} ! '
        f'video/x-raw,format=NV12,width={video_width},height={video_height},'
        f'framerate={frame_rate}/1,pixel-aspect-ratio=1/1 ! '
        f'videoconvert ! video/x-raw,format=I420 ! '
    )
    return (
        f"{source_element} "
        f"{QUEUE(name=f'{name}_scale_q')} ! "
        f"videoscale name={name}_videoscale n-threads=2 ! "
        f"{QUEUE(name=f'{name}_convert_q')} ! "
        f"videoconvert n-threads=3 name={name}_convert qos=false ! "
        f"video/x-raw, pixel-aspect-ratio=1/1, format=RGB, "
        f"width={video_width}, height={video_height}"
    )


# ---------------------------------------------------------------------------
# Pipeline app factory
# ---------------------------------------------------------------------------


def create_app(shared_state, target_state=None, eos_reached=None, ui_state=None, ui_fps=10,
               parser: Optional[argparse.ArgumentParser] = None,
               record_enabled=False, record_dir=None, reid_manager=None,
               tracker_name=None, log_perf=False):
    """Create the tiling pipeline app with drone-follow callback.

    Follows the hailo-app pattern: build parser, create user_data,
    instantiate GStreamerTilingApp. If eos_reached is a threading.Event,
    EOS will set it instead of calling GStreamer shutdown (so we can land first).

    Args:
        shared_state: SharedDetectionState for passing detections to control loop
        target_state: FollowTargetState for tracking-based target selection (optional)
        eos_reached: threading.Event to signal EOS instead of shutdown (optional)
        ui_state: SharedUIState for web UI (optional)
        ui_fps: MJPEG stream frame rate (default: 10)
        parser: Pre-built argparse parser with all domain args already registered.
                If None, a bare pipeline parser is created (for backward compat).
        record_dir: Directory for recording output files (optional)
        reid_manager: ReIDManager for appearance-based re-identification (optional)
    """
    from hailo_apps.python.pipeline_apps.tiling.tiling_pipeline import (
        GStreamerTilingApp,
    )
    from hailo_apps.python.core.gstreamer.gstreamer_app import app_callback_class
    from hailo_apps.python.core.common.core import get_pipeline_parser
    from hailo_apps.python.core.gstreamer.gstreamer_helper_pipelines import (
        QUEUE, DISPLAY_PIPELINE, OVERLAY_PIPELINE,
        INFERENCE_PIPELINE, USER_CALLBACK_PIPELINE,
        TILE_CROPPER_PIPELINE, SOURCE_PIPELINE,
    )

    if parser is None:
        parser = get_pipeline_parser()

    class DroneFollowUserData(app_callback_class):
        def __init__(self, shared_state, target_state=None, ui_state=None,
                     tracker=None, reid_manager=None, log_perf=False):
            super().__init__()
            self.shared_state = shared_state
            self.target_state = target_state
            self.ui_state = ui_state
            self.tracker = tracker
            self.reid_manager = reid_manager
            self.perf = _PerfTracker(
                log_perf=log_perf,
                tracker_metrics=tracker.metrics if tracker else None,
            )
            # Set after app creation so callback can extract frames for ReID
            self.video_width = 0
            self.video_height = 0
            # SOT (single-object tracking) state
            self.sot_active = False
            self.sot_last_bbox = None   # [x1,y1,x2,y2] in SCALE coords
            self.sot_target_id = None
            self.sot_frames = 0

    class DroneFollowTilingApp(GStreamerTilingApp):
        """Tiling app with EOS handling and optional MJPEG appsink for web UI."""
        def __init__(self, app_callback, user_data, parser=None, eos_reached=None,
                     ui_enabled=False, ui_state=None, ui_fps=30,
                     record_enabled=False, record_dir=None):
            self._eos_reached = eos_reached
            self._ui_enabled = ui_enabled
            self._record_enabled = record_enabled
            self._ui_state = ui_state
            self._ui_fps = ui_fps
            self._recording = False
            self._record_dir = record_dir or os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "..", "recordings")
            self._record_lock = threading.Lock()
            self._shm_rebuild_pending = False

            # Pre-detect SHM resolution BEFORE super().__init__() so that
            # the tiling configuration (tile grid, overlap, batch size) is
            # computed with the correct frame dimensions.  Without this,
            # the base class configures tiling for the CLI default (e.g.
            # 1280x720) while the SHM source actually delivers 640x480,
            # causing a buffer size mismatch → Hailo DMA crash.
            if parser is not None:
                _pre_args, _ = parser.parse_known_args()
                _pre_input = getattr(_pre_args, 'input', None)
                if _pre_input and str(_pre_input).startswith('shm://'):
                    shm_res = _read_shm_resolution()
                    if shm_res is not None:
                        shm_w, shm_h, shm_fps = shm_res
                        LOGGER.info(
                            "Pre-init: SHM metadata says %dx%d@%d — "
                            "injecting into parser defaults",
                            shm_w, shm_h, shm_fps)
                        # Override the parser defaults so the base class
                        # sees the correct resolution during configure().
                        parser.set_defaults(
                            width=shm_w, height=shm_h,
                            frame_rate=shm_fps)

            super().__init__(app_callback, user_data, parser=parser)
            # After base class init, sync resolution from SHM metadata if in
            # SHM mode so that self.video_width/height are correct for any
            # future rebuild (watchdog, manual, etc.).
            if str(getattr(self, 'video_source', '')).startswith('shm://'):
                shm_res = _read_shm_resolution()
                if shm_res is not None:
                    self.video_width, self.video_height, self.frame_rate = shm_res
            # Connect appsink after pipeline is created by super().__init__
            if self._ui_enabled:
                self._connect_mjpeg_sink()

        def _connect_mjpeg_sink(self):
            """Connect the MJPEG appsink's new-sample signal."""
            self._Gst = _get_gst()
            mjpeg_sink = self.pipeline.get_by_name("mjpeg_sink")
            if mjpeg_sink:
                mjpeg_sink.connect("new-sample", self._on_mjpeg_sample)

        def _on_mjpeg_sample(self, appsink):
            """appsink callback: extract pre-encoded JPEG bytes."""
            Gst = self._Gst
            sample = appsink.emit("pull-sample")
            if sample:
                buf = sample.get_buffer()
                success, map_info = buf.map(Gst.MapFlags.READ)
                if success:
                    self._ui_state.update_frame(bytes(map_info.data))
                    buf.unmap(map_info)
            return Gst.FlowReturn.OK

        def bus_call(self, bus, message, loop):
            """Override to rebuild pipeline on errors in SHM mode instead of shutting down."""
            import gi
            gi.require_version("Gst", "1.0")
            from gi.repository import Gst, GLib
            t = message.type
            if t == Gst.MessageType.ERROR:
                err, debug = message.parse_error()
                is_shm = str(getattr(self, 'video_source', '')).startswith('shm://')
                if is_shm and not self._shm_rebuild_pending:
                    self._shm_rebuild_pending = True
                    LOGGER.warning("SHM pipeline error (%s) — waiting for socket + rebuilding", err)
                    # Start polling for the SHM socket to reappear (OpenHD may
                    # be restarting its pipeline with a new resolution).
                    self._shm_poll_count = 0
                    GLib.timeout_add(500, self._shm_wait_for_socket)
                    return True
                elif is_shm:
                    # Additional error while rebuild already pending — ignore
                    return True
            return super().bus_call(bus, message, loop)

        def _shm_wait_for_socket(self):
            """Poll until the SHM socket and metadata file exist, then rebuild.

            OpenHD removes the socket and recreates it during pipeline restart.
            The metadata file is written first (during setup()), then the socket
            appears when shmsink enters PLAYING.  We poll every 500ms for up to
            30s (60 attempts) before giving up.
            """
            self._shm_poll_count += 1
            socket_path = str(self.video_source).split('://', 1)[1]
            meta_ok = _read_shm_resolution() is not None
            socket_ok = os.path.exists(socket_path)
            if socket_ok and meta_ok:
                LOGGER.info("SHM socket + metadata ready after %d polls — rebuilding pipeline",
                            self._shm_poll_count)
                self._shm_rebuild()
                return False  # stop polling
            if self._shm_poll_count >= 60:
                LOGGER.warning("SHM socket/metadata did not reappear after 30s — rebuilding anyway")
                self._shm_rebuild()
                return False
            if self._shm_poll_count % 10 == 0:
                LOGGER.debug("Waiting for SHM socket (exists=%s) + metadata (exists=%s) [poll %d]",
                             socket_ok, meta_ok, self._shm_poll_count)
            return True  # keep polling

        def _shm_rebuild(self):
            """Rebuild pipeline after SHM error (e.g. OpenHD resolution change).

            Re-reads the OpenHD metadata file so the new pipeline uses
            the correct caps for the (potentially changed) resolution.
            Performs a careful teardown to avoid kernel warnings from the
            Hailo PCIe driver's DMA buffer mapping (find_vma race on
            kernel 6.12+).
            """
            import gi
            gi.require_version("Gst", "1.0")
            from gi.repository import Gst

            self._shm_rebuild_pending = False
            # Update our video dimensions from the metadata file so the
            # rebuilt pipeline negotiates the correct SHM buffer layout.
            shm_res = _read_shm_resolution()
            if shm_res is not None:
                new_w, new_h, new_fps = shm_res
                if new_w != self.video_width or new_h != self.video_height:
                    LOGGER.info(
                        "SHM rebuild: resolution changed %dx%d -> %dx%d",
                        self.video_width, self.video_height, new_w, new_h,
                    )
                self.video_width = new_w
                self.video_height = new_h
                self.frame_rate = new_fps

            # Pre-teardown: transition the old pipeline through READY/NULL
            # explicitly so in-flight Hailo inference buffers are drained
            # before we create the new pipeline.
            if self.pipeline:
                LOGGER.debug("SHM rebuild: pipeline PLAYING -> NULL (drain Hailo buffers)")
                self.pipeline.set_state(Gst.State.NULL)
                self.pipeline.get_state(5 * Gst.SECOND)
                bus = self.pipeline.get_bus()
                if bus:
                    bus.remove_signal_watch()
                self.pipeline = None
                # The Hailo PCIe driver needs time to fully release DMA
                # buffer mappings before new ones can be allocated.
                # Without this delay, hailo_vdma_buffer_map triggers
                # kernel warnings (find_vma race) and a segfault.
                LOGGER.debug("SHM rebuild: waiting for Hailo DMA release")
                time.sleep(2.0)

            # Reset tracker to clear stale predictions from old resolution
            if hasattr(self, 'user_data') and hasattr(self.user_data, 'tracker'):
                self.user_data.tracker.reset()
                LOGGER.debug("SHM rebuild: tracker reset")

            # Now build a fresh pipeline from scratch (skip base class
            # teardown since we already did it above).
            self.watchdog_paused = True
            self.rebuild_count += 1
            try:
                LOGGER.debug("SHM rebuild: creating new pipeline")
                pipeline_string = self.get_pipeline_string()
                LOGGER.debug("SHM rebuild: pipeline string: %s", pipeline_string)

                self.pipeline = Gst.parse_launch(pipeline_string)

                bus = self.pipeline.get_bus()
                bus.add_signal_watch()
                bus.connect("message", self.bus_call, self.loop)

                self._connect_callback()
                self._on_pipeline_rebuilt()

                from hailo_apps.python.core.gstreamer.gstreamer_app import disable_qos
                disable_qos(self.pipeline)

                ret = self.pipeline.set_state(Gst.State.PLAYING)
                if ret == Gst.StateChangeReturn.FAILURE:
                    LOGGER.error("SHM rebuild: failed to start new pipeline")
                    self.loop.quit()
                    return False

                LOGGER.info("SHM rebuild: pipeline rebuilt and playing")
                self.watchdog_paused = False
            except Exception:
                LOGGER.error("SHM rebuild: exception", exc_info=True)
                self.loop.quit()
            return False

        def on_eos(self):
            if self._eos_reached is not None:
                self._eos_reached.set()
            else:
                super().on_eos()

        def _on_pipeline_rebuilt(self):
            super()._on_pipeline_rebuilt()
            if self._ui_enabled:
                self._connect_mjpeg_sink()

        # ---- Recording control ----

        @property
        def is_recording(self):
            return self._recording

        def _generate_record_path(self):
            os.makedirs(self._record_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            return os.path.join(self._record_dir, f"rec_{ts}.mp4")

        def start_recording(self, path=None):
            """Start GStreamer-native recording. Returns the output file path."""
            Gst = _get_gst()

            with self._record_lock:
                if self._recording:
                    LOGGER.warning("[record] Already recording")
                    return None
                if not self._record_enabled:
                    LOGGER.error("[record] Recording requires --record flag")
                    return None

                valve = self.pipeline.get_by_name("record_valve")
                encoder = self.pipeline.get_by_name("record_enc")
                muxer = self.pipeline.get_by_name("record_mux")
                filesink = self.pipeline.get_by_name("record_sink")
                if valve is None or filesink is None:
                    LOGGER.error("[record] Recording elements not found in pipeline")
                    return None

                record_path = path or self._generate_record_path()

                # Cycle encoder, muxer, and filesink through NULL to clear
                # any leftover EOS state from a previous recording session
                for el in (filesink, muxer, encoder):
                    if el is not None:
                        el.set_state(Gst.State.NULL)
                filesink.set_property("location", record_path)
                for el in (encoder, muxer, filesink):
                    if el is not None:
                        el.sync_state_with_parent()

                valve.set_property("drop", False)
                self._recording = True
                self._current_record_path = record_path
                LOGGER.info("[record] Started recording to %s", record_path)
                return record_path

        def stop_recording(self):
            """Stop recording and finalize the file."""
            Gst = _get_gst()

            with self._record_lock:
                if not self._recording:
                    return None

                valve = self.pipeline.get_by_name("record_valve")
                encoder = self.pipeline.get_by_name("record_enc")
                muxer = self.pipeline.get_by_name("record_mux")
                filesink = self.pipeline.get_by_name("record_sink")
                if valve is None:
                    self._recording = False
                    return None

                self._recording = False
                path = getattr(self, "_current_record_path", None)

                # Close the valve first to stop new buffers entering
                valve.set_property("drop", True)

                # Transition encoder → muxer → filesink to NULL.
                # matroskamux finalises the file (writes Cues/SeekHead/duration)
                # during its state change — no EOS needed.
                for el in (encoder, muxer, filesink):
                    if el is not None:
                        el.set_state(Gst.State.NULL)

                LOGGER.info("[record] Stopped recording: %s", path)
                return path

        def cleanup_recording_branch(self):
            """Force recording branch elements to NULL so they don't block pipeline shutdown."""
            if not self._record_enabled:
                return
            Gst = _get_gst()
            with self._record_lock:
                for name in ("record_enc", "record_mux", "record_sink"):
                    el = self.pipeline.get_by_name(name)
                    if el is not None:
                        el.set_state(Gst.State.NULL)

        def get_pipeline_string(self):
            openhd_stream = getattr(self.options_menu, 'openhd_stream', False)
            no_display = getattr(self.options_menu, 'no_display', False)
            is_shm = str(self.video_source).startswith('shm://')

            # If no custom output needed, delegate to parent
            if not self._ui_enabled and not self._record_enabled and not openhd_stream and not is_shm and not no_display:
                return super().get_pipeline_string()

            if is_shm:
                source_pipeline = _shm_source_pipeline(
                    self.video_source, self.video_width, self.video_height,
                    self.frame_rate,
                )
            else:
                source_pipeline = SOURCE_PIPELINE(
                    video_source=self.video_source,
                    video_width=self.video_width,
                    video_height=self.video_height,
                    frame_rate=self.frame_rate,
                    sync=self.sync,
                    horizontal_mirror=False,
                )

            detection_pipeline = INFERENCE_PIPELINE(
                hef_path=self.hef_path,
                post_process_so=self.post_process_so,
                post_function_name=self.post_function,
                batch_size=self.batch_size,
                config_json=self.labels_json,
            )

            # Detect identity case: 1x1 tiles where frame matches model
            # input exactly.  The hailotilecropper has a DMA buffer-pool
            # negotiation bug in this passthrough path (no scaling needed)
            # that crashes the Hailo PCIe driver.  Skip the tile cropper
            # entirely — the inference pipeline processes the full frame
            # directly and produces identical results since coordinates
            # map 1:1 when frame_size == model_input_size.
            skip_tiling = (
                self.tiles_x == 1 and self.tiles_y == 1
                and not self.use_multi_scale
                and self.video_width == self.model_input_width
                and self.video_height == self.model_input_height
            )

            if skip_tiling:
                LOGGER.info(
                    "Bypassing tile cropper: 1x1 tiles with frame "
                    "(%dx%d) matching model input — direct inference",
                    self.video_width, self.video_height,
                )
            else:
                tiling_mode = 1 if self.use_multi_scale else 0
                scale_level = self.scale_level if self.use_multi_scale else 0
                tile_cropper_pipeline = TILE_CROPPER_PIPELINE(
                    detection_pipeline,
                    name='tile_cropper_wrapper',
                    internal_offset=True,
                    scale_level=scale_level,
                    tiling_mode=tiling_mode,
                    tiles_along_x_axis=self.tiles_x,
                    tiles_along_y_axis=self.tiles_y,
                    overlap_x_axis=self.overlap_x,
                    overlap_y_axis=self.overlap_y,
                    iou_threshold=self.iou_threshold,
                    border_threshold=self.border_threshold,
                )

            user_callback_pipeline = USER_CALLBACK_PIPELINE()

            # Primary output branch: OpenHD stream, display, or fakesink
            if openhd_stream:
                openhd_port = getattr(self.options_menu, 'openhd_port', 5500)
                openhd_bitrate = getattr(self.options_menu, 'openhd_bitrate', 3917)
                primary_branch = (
                    f"{OVERLAY_PIPELINE(name='openhd_overlay')} ! " +
                    _openhd_stream_pipeline(port=openhd_port, bitrate=openhd_bitrate)
                )
            elif no_display:
                primary_branch = f"fakesink sync={self.sync}"
            else:
                primary_branch = DISPLAY_PIPELINE(
                    video_sink=self.video_sink, sync=self.sync, show_fps=self.show_fps,
                )

            # Build extra branches beyond primary
            extra_branches = []

            if self._ui_enabled:
                mjpeg_branch = (
                    f"videoconvert n-threads=2 ! "
                    f"videorate max-rate={self._ui_fps} ! "
                    f"video/x-raw,framerate={self._ui_fps}/1 ! "
                    f"jpegenc quality=70 ! "
                    f"appsink name=mjpeg_sink sync=false drop=true emit-signals=true"
                )
                extra_branches.append(
                    f"t. ! {QUEUE(name='mjpeg_branch_q', leaky='downstream')} ! {mjpeg_branch}"
                )

            if self._record_enabled:
                record_branch = (
                    f"valve name=record_valve drop=true ! "
                    f"{OVERLAY_PIPELINE(name='record_overlay')} ! "
                    f"videoconvert n-threads=2 ! "
                    f"x264enc name=record_enc tune=zerolatency bitrate=5000 speed-preset=ultrafast ! "
                    f"mp4mux name=record_mux faststart=true ! filesink name=record_sink async=false location=/dev/null"
                )
                extra_branches.append(
                    f"t. ! {QUEUE(name='record_branch_q', max_size_buffers=1, leaky='downstream')} ! {record_branch}"
                )

            if extra_branches:
                # For file sources, the leaky queues after the tee prevent
                # sink sync=true from applying backpressure, so filesrc
                # decodes at full speed.  Insert identity sync=true before
                # the tee to pace data at real-time rate.
                sync_element = "identity sync=true ! " if self.source_type == "file" else ""
                output_pipeline = (
                    f"{sync_element}tee name=t "
                    f"t. ! {QUEUE(name='primary_branch_q', leaky='downstream')} ! {primary_branch} "
                    + " ".join(extra_branches)
                )
            else:
                output_pipeline = primary_branch

            if skip_tiling:
                # Direct pipeline: source → inference → callback → output
                pipeline_parts = [source_pipeline, detection_pipeline]
            else:
                pipeline_parts = [source_pipeline, tile_cropper_pipeline]
            pipeline_parts.extend([user_callback_pipeline, output_pipeline])

            return ' ! '.join(pipeline_parts)

    if tracker_name is None:
        tracker_name = "byte"
    t0 = time.monotonic()
    inner_tracker = create_tracker(
        tracker_name, track_thresh=0.4, track_buffer=90,
        match_thresh=0.5, frame_rate=30,
    )
    init_ms = (time.monotonic() - t0) * 1000.0
    tracker = MetricsTracker(inner_tracker, init_time_ms=init_ms)
    LOGGER.info("[tracking] %s ready (init %.1f ms), running synchronously in callback",
                tracker_name, init_ms)
    shared_state.tracker_metrics = tracker.metrics

    user_data = DroneFollowUserData(
        shared_state, target_state, ui_state=ui_state, tracker=tracker,
        reid_manager=reid_manager, log_perf=log_perf,
    )
    app = DroneFollowTilingApp(
        app_callback, user_data, parser=parser, eos_reached=eos_reached,
        ui_enabled=(ui_state is not None), ui_state=ui_state, ui_fps=ui_fps,
        record_enabled=record_enabled, record_dir=record_dir,
    )
    # Store video dimensions on user_data so the callback can extract
    # frames for ReID cropping without needing a reference to the app.
    user_data.video_width = app.video_width
    user_data.video_height = app.video_height
    return app
