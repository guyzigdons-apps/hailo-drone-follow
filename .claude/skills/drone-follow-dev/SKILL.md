---
name: drone-follow-dev
description: Day-to-day dev loop for drone-follow on the laptop — run sim + drone-follow with web UI, free port conflicts, kill cleanly. Use when the user says "run drone-follow with sim" / "start the simulator" / "kill drone-follow".
---

# Drone-follow dev loop

Two terminals: `start_sim.sh` in one, `drone-follow` in the other. Repo at
`/home/giladn/tappas_apps/repos/hailo-apps-infra`; sub-project at
`community/apps/hailo_drone_follow`.

## Pre-checks

Run these first when the user asks to start a session.

```bash
# 1. Are sim processes already running?
ps -ef | grep -E "px4|gz sim|video_bridge|start_sim" | grep -v grep

# 2. Is UDP 5600 (video) free? Common offender: QOpenHD.
ss -tunlp 2>/dev/null | grep -E ":5600|:14540"

# 3. Is the venv on PATH? `which drone-follow` will fail in non-login shells —
# remind the user they need `source setup_env.sh` in their interactive terminal.
ls /home/giladn/tappas_apps/repos/hailo-apps-infra/venv_hailo_apps/bin/drone-follow
```

If 5600 is held by QOpenHD, kill QOpenHD first or close its window. Drone-follow can't receive video while it's bound.

## Worlds available
List with `ls community/apps/hailo_drone_follow/sim/worlds/`. Common picks:
- `2_persons_diagonal` — two static persons, separated diagonally — good for testing AUTO-mode largest-person selection and lock switching.
- `2_person_world` — two persons close together — ReID drift / reacquire stress.
- `random_walk` — moving target — ByteTracker continuity test.
- `walk_across_then_approach` — distance + size variation.

## Run

**Terminal 1 — sim:**
```bash
cd /home/giladn/tappas_apps/repos/hailo-apps-infra/community/apps/hailo_drone_follow
sim/start_sim.sh --bridge --world <world-name>
```
First run takes 10–20 min for PX4 build (`sim/setup_sim.sh` if it's a fresh clone). Subsequent runs ~10 s to PLAYING.

**Terminal 2 — drone-follow with web UI:**
```bash
cd /home/giladn/tappas_apps/repos/hailo-apps-infra/community/apps/hailo_drone_follow
source setup_env.sh
drone-follow --input udp://0.0.0.0:5600 --takeoff-landing --webui
```
Web UI: http://localhost:5001 (override via `--webui-port`).

**Common variations:**
- Add `--display` if you also want the local X11 window with overlay (the implicit-display rule turns it off when `--webui` is set).
- Add `--no-yaw-only` for full follow with forward/backward control. Default `--yaw-only` is the safe choice with the simulator's vision camera.
- Drop `--takeoff-landing` if you want the drone already airborne (e.g., joystick-controlled).
- USB camera + sim ⇒ **must** add `--yaw-only` — webcam sees the real world, forward commands based on bbox-size are unsafe.

## Kill cleanly

```bash
ps -ef | grep -E "drone[-_]follow|start_sim|video_bridge|px4|gz sim" | grep -v grep | awk '{print $2}' | xargs -r kill
sleep 2
# if anything's still alive:
ps -ef | grep -E "drone[-_]follow|start_sim|video_bridge|px4|gz sim" | grep -v grep | awk '{print $2}' | xargs -r kill -9
ss -tunlp 2>/dev/null | grep -E ":5600|:14540"   # both should be empty
```

`start_sim.sh` is the parent of the bridge + cmake + px4 + gz chain — it usually propagates SIGTERM to its children, but force-killing each PID is safer when the chain is half-stuck.

## After build / pull / config change

Order:
1. **Plugin .cpp/.hpp changed?** → `hailo-compile-postprocess install` + `rm -f ~/.cache/gstreamer-1.0/registry.x86_64.bin`. See `../../memory/hailotilecropper_dynamic.md`.
2. **Web UI files changed?** → `cd drone_follow/ui && nvm use 20 && npm install && npm run build`. See `../../memory/webui_build.md`.
3. **Just Python changed?** → No rebuild. Just restart `drone-follow`.

## Don't do

- Don't `sudo` install built `.so` files into `/usr/lib/...` outside of `hailo-compile-postprocess install` — the install script knows which artifacts go where.
- Don't run `gst-launch-1.0 --gst-plugin-load=<built.so>` while the system version of the same plugin is also installed; type-registration collides and segfaults at PLAYING. Install via `hailo-compile-postprocess` instead.
- Don't use `openhd --clean-start` to "reset" the air unit if it's paired with a working ground unit — see `../../memory/openhd_pairing.md`.
