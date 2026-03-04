"""OpenHD Bridge — UDP bridge for controlling drone follow parameters via MAVLink.

Listens for parameter change messages from OpenHD (C++ HailoFollowBridge) and
applies them to the shared ControllerConfig. Periodically reports current values
back so OpenHD can serve accurate read-back to the ground station.

Float params use native MAVLink REAL32 — no scaling needed.
Int/bool params use MAVLink INT32.

Wire protocol (JSON over UDP):
  OpenHD -> Python (port 5510): {"param": "<field_name>", "value": <number>}
  Python -> OpenHD (port 5511): {"params": {"<field_name>": <number>, ...}}
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

# Special param for follow target control (not in ControllerConfig)
_FOLLOW_ID_PARAM = "follow_id"


class OpenHDBridge:
    """UDP bridge between OpenHD MAVLink params and ControllerConfig."""

    def __init__(self, controller_config, target_state=None,
                 listen_port=5510, report_port=5511, report_interval=2.0):
        self._config = controller_config
        self._target_state = target_state
        self._listen_port = listen_port
        self._report_port = report_port
        self._report_interval = report_interval
        self._running = False
        self._listen_thread = None
        self._report_thread = None
        self._sock = None

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
            elif param_name in _CONFIG_PARAMS:
                self._apply_config_param(param_name, value)
            else:
                LOGGER.warning("[openhd_bridge] Unknown param: %s", param_name)

    def _apply_follow_id(self, value):
        """Handle follow target selection from ground station."""
        if self._target_state is None:
            LOGGER.warning("[openhd_bridge] follow_id received but no target_state")
            return
        if value <= 0:
            self._target_state.set_target(None)
            LOGGER.info("[openhd_bridge] follow_id cleared (following largest)")
        else:
            self._target_state.set_target(value)
            LOGGER.info("[openhd_bridge] follow_id = %d", value)

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

        # Report follow target state
        if self._target_state is not None:
            target_id = self._target_state.get_target()
            params[_FOLLOW_ID_PARAM] = target_id if target_id is not None else 0

        msg = json.dumps({"params": params}).encode("utf-8")
        try:
            sock.sendto(msg, ("127.0.0.1", self._report_port))
        except OSError as e:
            LOGGER.debug("[openhd_bridge] Report send failed: %s", e)
