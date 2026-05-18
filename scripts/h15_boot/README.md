# H15 Drone Follow — Service & Launch Reference

Reference for running `drone_follow_h15` on the Hailo15 target.

## Files on the H15 (after `install.sh`)

| File | Purpose |
|---|---|
| `/etc/init.d/drone-follow` | SysV init script |
| `/etc/default/drone-follow` | CLI args passed to the app (edit this to change flags) |
| `/etc/rcS.d/S99drone-follow` | Symlink for auto-start at boot |
| `/var/log/drone-follow.log` | Service stdout/stderr (appended each run) |
| `/var/run/drone-follow.pid` | PID file while service is running |
| `/home/root/drone_follow/` | Synced Python package |
| `/home/root/drone_follow/ui/build/` | Built React UI |
| `/home/root/recordings/` | Video recordings (`.mkv`) + detection logs (`.jsonl`) |
| `/home/root/df_config.json` | Saved controller config (if "Save Config" pressed in UI) |

## Installation (from build host)

```bash
cd hailo-drone-follow/scripts/h15_boot

./install.sh                          # full install: UI build + app + service (target=root@10.0.0.1)
./install.sh root@<ip>                # custom target
TARGET=root@<ip> ./install.sh         # via env var

./build_ui.sh                         # just rebuild React UI (drone_follow/ui/build/)
./install_app.sh                      # just sync drone_follow/ code (assumes UI already built)
./install_service.sh                  # just install/refresh init service
./uninstall.sh                        # remove service files (keeps log)
```

The full `install.sh` runs `build_ui.sh`, then `install_app.sh`, then `install_service.sh`.
Run `build_ui.sh` standalone whenever you change `drone_follow/ui/src/`.

## Service control (on target)

```bash
/etc/init.d/drone-follow start
/etc/init.d/drone-follow stop
/etc/init.d/drone-follow restart
/etc/init.d/drone-follow status
```

After reboot the service starts automatically via `/etc/rcS.d/S99drone-follow`.

To **disable** auto-start without uninstalling:
```bash
rm /etc/rcS.d/S99drone-follow
```

To **re-enable**:
```bash
ln -sf /etc/init.d/drone-follow /etc/rcS.d/S99drone-follow
```

## Changing CLI args

Edit `/etc/default/drone-follow` on the device:
```bash
nano /etc/default/drone-follow
/etc/init.d/drone-follow restart
```

The file sets `DRONE_FOLLOW_ARGS`. Examples:

```bash
# Real drone via Pixhawk serial (waits for OFFBOARD via GCS — safest):
DRONE_FOLLOW_ARGS="--serial /dev/ttyACM0"

# Bench testing with Pixhawk (auto-OFFBOARD, no GCS):
DRONE_FOLLOW_ARGS="--serial /dev/ttyACM0 --auto-offboard"

# PX4 SITL on PC over UDP:
DRONE_FOLLOW_ARGS=""

# Sim with auto takeoff + tracking + landing:
DRONE_FOLLOW_ARGS="--takeoff-landing"

# Dry-run (no drone connection):
DRONE_FOLLOW_ARGS="--dry-run"

# Add local recording:
DRONE_FOLLOW_ARGS="--serial /dev/ttyACM0 --record"

# Custom recording path:
DRONE_FOLLOW_ARGS="--serial /dev/ttyACM0 --record /home/root/recordings/test.mkv"

# Load a saved config:
DRONE_FOLLOW_ARGS="--serial /dev/ttyACM0 --config /home/root/df_config.json"
```

## Manual launch (without service)

If you want to run the app manually (e.g., to see live logs):

```bash
# Stop the service first
/etc/init.d/drone-follow stop

# Run manually
PYTHONPATH=/home/root python3 -m drone_follow.drone_follow_h15 --serial /dev/ttyACM0
```

## Viewing live

| Endpoint | What |
|---|---|
| `http://10.0.0.1:5001/` | Web UI (React) — video + detection overlay + config sliders |
| `http://10.0.0.1:5001/api/video` | Raw MJPEG stream (works without React build) |
| `http://10.0.0.1:5001/api/config` | Current config as JSON |
| `udp://10.0.0.2:5002` | Raw H264 stream (from H15 to PC) |

H264 stream via gst-launch on the PC:
```bash
gst-launch-1.0 udpsrc port=5002 ! "application/x-rtp,encoding-name=H264" \
    ! rtph264depay ! decodebin ! autovideosink
```

## SSH tunnel (browser on a different host)

If the browser machine can't reach `10.0.0.1` directly but can reach an intermediate PC:
```bash
ssh -L 5001:10.0.0.1:5001 user@<intermediate-PC>
```
Then open `http://localhost:5001` in the browser.

## Recordings

**Location on target:** `/home/root/recordings/`

When `--record` is set, the service writes two files per run:
- `drone_<YYYYMMDD_HHMMSS>.mkv` — H264 video (no overlays)
- `drone_<YYYYMMDD_HHMMSS>.jsonl` — detection data per inference frame

**List recordings on target:**
```bash
ssh root@10.0.0.1 "ls -lt /home/root/recordings/"
```

**Copy recordings off the target:**
```bash
scp 'root@10.0.0.1:/home/root/recordings/drone_*' ./
```

**Disk usage / free space on target:**
```bash
ssh root@10.0.0.1 "df -h /home/root && du -sh /home/root/recordings/"
```

**Delete old recordings on target:**
```bash
ssh root@10.0.0.1 "rm /home/root/recordings/drone_*"
```

**Generate annotated video (post-flight, on a machine with OpenCV):**

Requires `pip install opencv-python` on the machine running the script.

```bash
# Pull both files (the .jsonl must be next to the .mkv)
scp root@10.0.0.1:/home/root/recordings/drone_xxx.{mkv,jsonl} .

# Default: auto-finds drone_xxx.jsonl next to drone_xxx.mkv
# Writes drone_xxx_annotated.mp4
./scripts/post_process/overlay_detections.py drone_xxx.mkv

# Explicit detection log file
./scripts/post_process/overlay_detections.py drone_xxx.mkv path/to/dets.jsonl

# Custom output path
./scripts/post_process/overlay_detections.py drone_xxx.mkv -o my_overlay.mp4

# Different codec (mp4v default; avc1 for H264; XVID for AVI)
./scripts/post_process/overlay_detections.py drone_xxx.mkv --codec avc1

# Help
./scripts/post_process/overlay_detections.py --help
```

Output: green boxes around the followed target, white boxes around the rest, with `ID N XX%` labels.

## Logs

```bash
# Service log (appended across runs)
tail -f /var/log/drone-follow.log

# Last 200 lines
tail -200 /var/log/drone-follow.log
```

## Quick health check on target

```bash
/etc/init.d/drone-follow status
ps aux | grep drone_follow
netstat -tln | grep -E "5001|5002"        # WebServer + H264 UDP
tail /var/log/drone-follow.log
```
