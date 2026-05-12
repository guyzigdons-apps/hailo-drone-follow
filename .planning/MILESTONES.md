# Milestones

Shipped milestones, chronological. Newest at top.

---

## v1.0 — Drone follow (shipped pre-GSD)

**Phases:** Pre-GSD baseline (no per-phase records).
**Status:** ✓ Shipped, in production use on the hailo-drone-follow air unit + ground station.
**Phase numbering ended at:** 0 (pre-GSD baseline). Next milestone starts at Phase 1.

**What shipped:**

- Hailo-NPU vision pipeline (Hailo-8L on RPi5) with multi-input support: CSI camera (`rpi`), USB webcam (`usb`), UDP (`udp://...`), OpenHD shared-memory (`shm://...`).
- ByteTracker multi-person tracking + ReID-based re-acquisition (gallery, drift gate, duplicate gate, raw-detection fallback). Full algorithm documented in `docs/tracking-reid-algorithm.md`.
- Pure follow controller (`follow_api/`) with bbox-distance forward loop, yaw centring, altitude-hold P, frame-edge safety, per-axis EMA, forward-axis slew-rate cap.
- MAVSDK adapter for PX4 (USB serial `/dev/ttyACM0` or UDP for sim). Offboard handshake (pilot-driven), optional `--takeoff-landing` lifecycle, detached `mavsdk_server` for graceful Ctrl+C → land.
- Web UI (MJPEG + click-to-follow + live PID tuning) on port 5001.
- OpenHD integration: drone-follow either owns the camera and encodes for WFB (Mode A) or reads SHM raw frames OpenHD provides (Mode B). Configured at install via `scripts/install_air.sh --mode {stream|shm}`. MAVLink param bridge for QOpenHD-driven control (follow_id, bitrate, recording toggle, tunable params).
- Recording branch (x264 → matroskamux → filesink) toggleable mid-flight from UI or OpenHD.
- PX4 SITL + Gazebo Garden sim (bundled via `sim/PX4-Autopilot` submodule v1.14) with 6 actor-walk worlds, video bridge (`sim/bridge/`), MAVLink relay (`sim/mavlink_relay.py`) for remote-sim setups.
- Ground-station install path (`scripts/install_ground_station.sh`) — no Hailo dependency; runs `openhd --ground` + QOpenHD, with headless eglfs fallback for kiosk boots.
- Dual-WiFi networking (wlan0 home + MAC-pinned wlan1 field AP).
- Boot service (`drone-follow-boot.service`) controlled by `~/Desktop/drone-follow.conf` (`ENABLED`, `MODE`).

**Validated requirements:** see `.planning/PROJECT.md` § Requirements / Validated (VIS-01..03, CTRL-01..06, DRONE-01..03, UI-01..03, REC-01, SIM-01..02, OPS-01..03).

**Out-of-band planning artifacts from this era:** `docs/.planning/plans/2026-04-16-repo-review-bugfixes.md`, `docs/.planning/plans/2026-04-29-post-merge-stabilization.md` (preserved as historical context; not GSD-managed).
