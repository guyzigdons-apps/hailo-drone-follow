"""OpenHD Bridge — UDP bridge for controlling drone follow parameters via MAVLink.

Listens for parameter change messages from OpenHD (C++ HailoFollowBridge) and
applies them to the shared ControllerConfig. Periodically reports current values
back so OpenHD can serve accurate read-back to the ground station.

Float params use native MAVLink REAL32 — no scaling needed.
Int/bool params use MAVLink INT32.

Wire protocol (JSON over UDP):
  OpenHD -> Python (port 5510): {"param": "<field_name>", "value": <number>}
  Python -> OpenHD (port 5511): {"params": {<field_name>: <number>, ...},
                                  "avail_ids": [<int>, ...],
                                  "bboxes": [{"id": <int>, "cx": <float>,
                                              "cy": <float>, "w": <float>,
                                              "h": <float>, "tracked": <bool>}]}

follow_id semantics (DF_FOLLOW_ID):
  -1  → IDLE: drone holds position, ignores all detections
   0  → AUTO: system auto-follows largest person in frame
   N  → LOCKED: operator explicitly selected person N

active_id (DF_ACTIVE_ID):
  The currently active tracking ID reported by FollowTargetState regardless of
  whether it was auto-selected or operator-locked. 0 means no one being tracked.
  QOpenHD uses this to distinguish AUTO-tracking-someone from no-one-in-view.
"""

import json
import logging
import socket
import threading
import time

LOGGER = logging.getLogger("drone_follow.openhd_bridge")

# Mapping: python_field -> (mavlink_id, python_type)
# Values are sent as native float or int (no scaling).
_CONFIG_PARAMS = {
    "kp_yaw":                   ("DF_KP_YAW",    float),
    "kp_forward":               ("DF_KP_FWD",     float),
    "kp_backward":              ("DF_KP_BACK",    float),
    "max_forward":              ("DF_MAX_FWD",    float),
    "max_backward":             ("DF_MAX_BACK",   float),
    "target_distance_m":        ("DF_TGT_DIST",   float),
    "dead_zone_height_percent": ("DF_DZ_H_PCT",   float),
    "yaw_alpha":                ("DF_YAW_ALPHA",  float),
    "forward_alpha":            ("DF_FWD_ALPHA",  float),
    "takeoff_altitude":         ("DF_TAKEOFF_M",  float),
    "yaw_only":                 ("DF_YAW_ONLY",   bool),
    "fixed_altitude":           ("DF_FIX_ALT",    bool),
    "smooth_yaw":               ("DF_SMTH_YAW",   bool),
    "smooth_forward":           ("DF_SMTH_FWD",   bool),
}

# Fields where value 0 maps to Python None
_NULLABLE_FIELDS = {"target_distance_m"}

# Special params for follow target control (not in ControllerConfig)
_FOLLOW_ID_PARAM = "follow_id"
_ACTIVE_ID_PARAM = "active_id"
_BITRATE_PARAM = "bitrate_kbps"


class OpenHDBridge:
    """UDP bridge between OpenHD MAVLink params and ControllerConfig."""

    def __init__(self, controller_config, target_state=None, detection_state=None,
                 ui_state=None, gst_app=None,
                 listen_port=5510, report_port=5511, report_interval=0.1):
        self._config = controller_config
        self._target_state = target_state
        self._detection_state = detection_state
        self._ui_state = ui_state
        self._gst_app = gst_app  # GstApp for dynamic encoder control
        self._listen_port = listen_port
        self._report_port = report_port
        self._report_interval = report_interval
        self._running = False
        self._listen_thread = None
        self._report_thread = None
        self._sock = None

        # Tracks operator's explicit follow_id intent:
        #   -1 = IDLE (drone hold), 0 = AUTO, N = locked to person N
        # Reported back to QOpenHD instead of target_state.get_target() so that
        # the badge reflects the operator's choice, not the auto-selected ID.
        self._explicit_follow_id = 0
        self._current_bitrate_kbps = 0  # dedup repeated bitrate updates

    def start(self):
        """Start listener and reporter daemon threads."""
        if self._running:
            return
        self._running = True

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", self._listen_port))
        self._sock.settimeout(1.0)

        self._listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._listen_thread.start()

        self._report_thread = threading.Thread(target=self._report_loop, daemon=True)
        self._report_thread.start()

        LOGGER.info("[openhd_bridge] Started (listen=%d, report=%d)",
                    self._listen_port, self._report_port)

    def stop(self):
        """Stop the bridge threads."""
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._listen_thread:
            self._listen_thread.join(timeout=3.0)
        if self._report_thread:
            self._report_thread.join(timeout=3.0)
        LOGGER.info("[openhd_bridge] Stopped")

    # -- Listener: OpenHD -> Python ------------------------------------------

    def _listen_loop(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    LOGGER.warning("[openhd_bridge] Socket error in listener")
                break

            try:
                msg = json.loads(data.decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError):
                LOGGER.warning("[openhd_bridge] Invalid JSON received")
                continue

            param_name = msg.get("param")
            value = msg.get("value")
            if param_name is None or value is None:
                continue

            if param_name == _FOLLOW_ID_PARAM:
                self._apply_follow_id(int(value))
            elif param_name == _ACTIVE_ID_PARAM:
                pass  # read-only from Python's side; ignore any set forwarded by OpenHD
            elif param_name == _BITRATE_PARAM:
                self._apply_bitrate(int(value))
            elif param_name in _CONFIG_PARAMS:
                self._apply_config_param(param_name, value)
            else:
                LOGGER.warning("[openhd_bridge] Unknown param: %s", param_name)

    def _apply_follow_id(self, value: int):
        """Handle follow target selection from ground station.

        -1 → IDLE (drone holds position, ignores detections)
         0 → AUTO (follow largest)
         N → LOCKED to person N
        """
        if self._target_state is None:
            LOGGER.warning("[openhd_bridge] follow_id received but no target_state")
            return
        self._explicit_follow_id = value
        if value < 0:
            self._target_state.set_paused(True)
            self._target_state.set_target(None)
            self._target_state.set_explicit_lock(False)
            LOGGER.info("[openhd_bridge] IDLE mode (drone holding position)")
        elif value == 0:
            self._target_state.set_paused(False)
            self._target_state.set_target(None)
            self._target_state.set_explicit_lock(False)
            LOGGER.info("[openhd_bridge] AUTO mode (following largest)")
        else:
            self._target_state.set_paused(False)
            self._target_state.set_target(value)
            self._target_state.set_explicit_lock(True)
            LOGGER.info("[openhd_bridge] LOCKED to ID %d", value)
        # Immediately push state back so QOpenHD badge updates without waiting
        # for the next periodic report cycle.
        self._send_immediate_report()

    def _apply_bitrate(self, kbps: int):
        """Dynamically set x264enc bitrate from WFB link recommendation.

        Only applies in --openhd-stream mode where the drone-follow app owns the
        x264enc encoder. In SHM mode OpenHD handles encoding directly.
        """
        if kbps == self._current_bitrate_kbps:
            return
        if self._gst_app is None or not hasattr(self._gst_app, 'pipeline'):
            return
        pipeline = self._gst_app.pipeline
        if pipeline is None:
            return
        encoder = pipeline.get_by_name("openhd_stream_encoder")
        if encoder is None:
            # SHM mode — no local encoder; OpenHD handles bitrate directly
            return
        encoder.set_property("bitrate", kbps)
        self._current_bitrate_kbps = kbps
        LOGGER.info("[openhd_bridge] x264enc bitrate set to %d kbps", kbps)

    def _apply_config_param(self, python_name, value):
        """Apply a single parameter change from OpenHD to ControllerConfig."""
        _, py_type = _CONFIG_PARAMS[python_name]

        # Convert to Python type
        if python_name in _NULLABLE_FIELDS and value == 0:
            py_value = None
        elif py_type is bool:
            py_value = bool(int(value))
        elif py_type is float:
            py_value = float(value)
        else:
            py_value = value

        # Save old value for rollback on validation failure
        old_value = getattr(self._config, python_name, None)
        try:
            setattr(self._config, python_name, py_value)
            self._config.validate()
            LOGGER.info("[openhd_bridge] %s = %s", python_name, py_value)
        except ValueError as e:
            setattr(self._config, python_name, old_value)
            LOGGER.warning("[openhd_bridge] Rejected %s=%s: %s",
                           python_name, py_value, e)

    # -- Reporter: Python -> OpenHD ------------------------------------------

    def _send_immediate_report(self):
        """Send a one-shot report on a transient socket (callable from any thread)."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._send_report(sock)
        except OSError:
            pass
        finally:
            sock.close()

    def _report_loop(self):
        """Periodically send current config values to OpenHD for read-back sync."""
        report_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._send_report(report_sock)
            while self._running:
                time.sleep(self._report_interval)
                if self._running:
                    self._send_report(report_sock)
        finally:
            report_sock.close()

    def _send_report(self, sock):
        """Send all current parameter values to OpenHD."""
        params = {}
        for python_name, (_, py_type) in _CONFIG_PARAMS.items():
            py_value = getattr(self._config, python_name, None)
            if python_name in _NULLABLE_FIELDS and py_value is None:
                params[python_name] = 0.0
            elif py_type is bool:
                params[python_name] = int(py_value) if py_value is not None else 0
            elif py_type is float:
                params[python_name] = float(py_value) if py_value is not None else 0.0
            else:
                params[python_name] = py_value if py_value is not None else 0

        if self._target_state is not None:
            actual_target = self._target_state.get_target()

            # If the callback fell back to idle after losing an explicit lock,
            # sync follow_id so QOpenHD badge shows IDLE instead of a stale lock.
            if self._explicit_follow_id > 0 and self._target_state.is_paused():
                LOGGER.info("[openhd_bridge] Explicit lock on #%d lost — syncing to IDLE",
                            self._explicit_follow_id)
                self._explicit_follow_id = -1

            # Report operator's intent (not the auto-selected ID) as follow_id.
            # This ensures QOpenHD badge shows AUTO when no explicit selection was made.
            params[_FOLLOW_ID_PARAM] = self._explicit_follow_id

            # Report the actual currently-tracked ID so QOpenHD can show
            # "AUTO · #N" when in auto mode and a person is actively followed.
            params[_ACTIVE_ID_PARAM] = actual_target if actual_target is not None else 0

        payload = {"params": params}

        # Report available tracking IDs so QOpenHD can show a selection list
        if self._detection_state is not None:
            avail = self._detection_state.get_available_ids()
            payload["avail_ids"] = sorted(avail) if avail else []

        # Report all bounding boxes so QOpenHD can render a ground-side overlay
        if self._ui_state is not None:
            det_data = self._ui_state.get_detections()
            active_id = det_data.get("following_id")
            bboxes = []
            for det in det_data.get("detections", []):
                bbox = det.get("bbox", {})
                x = bbox.get("x", 0.0)
                y = bbox.get("y", 0.0)
                w = bbox.get("w", 0.0)
                h = bbox.get("h", 0.0)
                det_id = det.get("id")
                bboxes.append({
                    "id": det_id if det_id is not None else 0,
                    "cx": round(x + w / 2, 4),
                    "cy": round(y + h / 2, 4),
                    "w": round(w, 4),
                    "h": round(h, 4),
                    "tracked": det_id is not None and det_id == active_id,
                })
            payload["bboxes"] = bboxes

        msg = json.dumps(payload).encode("utf-8")
        try:
            sock.sendto(msg, ("127.0.0.1", self._report_port))
        except OSError as e:
            LOGGER.debug("[openhd_bridge] Report send failed: %s", e)
