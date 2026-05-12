# Pitfalls Research

**Domain:** Adding ROS 2 (rclpy) + Gazebo Garden rover sim to a multi-threaded async Python app
**Researched:** 2026-05-12
**Confidence:** MEDIUM — rclpy threading docs are sparse; most signal/threading findings from GitHub issues + community reports

---

## Critical Pitfalls

### Pitfall 1: rclpy.init() installs SIGINT handler that silently overrides drone-follow's Ctrl+C landing sequence

**What goes wrong:**
`rclpy.init()` calls into rcl, which installs its own SIGINT handler by default. That handler raises a `KeyboardInterrupt`-style interrupt into the rclpy executor. In drone-follow, `main()` installs a custom `on_signal` at line 402 of `drone_follow_app.py` — this handler sets `shutdown`, calls `_quit_pipeline()`, and then `drone_thread.join(5.0)` followed by graceful landing inside `run_live_drone`. If `rclpy.init()` is called after the custom handler is installed (or from a background thread), rclpy's handler may silently replace it, meaning Ctrl+C kills the process immediately rather than landing the drone first.

**Why it happens:**
`rclpy.init()` passes `signal_handler_options` defaulting to `SignalHandlerOptions.ALL` on Humble, installing handlers for both SIGINT and SIGTERM. The Python `signal` module only allows signal handlers to be set from the main thread; if `rclpy.init()` is called from a background thread (the rclpy executor thread) the C-level rcl handler may still be armed but the Python-level handler won't be.

**How to avoid:**
Call `rclpy.init(args=None, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)` to disable rclpy's signal handler entirely. drone-follow's `on_signal` already handles graceful shutdown. Add this as a mandatory parameter when constructing the rover adapter. Verify after init: `signal.getsignal(signal.SIGINT)` should still be drone-follow's `on_signal`.

**Warning signs:**
Ctrl+C with the rover adapter running causes immediate process exit without the "Ctrl+C received, shutting down..." log line; or the drone thread `join(5.0)` is never reached.

**Phase to address:** Phase 2 (rover adapter construction) — enforce `SignalHandlerOptions.NO` in `ros2_rover.py.__init__`.

---

### Pitfall 2: `rclpy._rclpy_pybind11` import fails silently in the hailo-apps venv

**What goes wrong:**
The hailo-apps venv is built with `--system-site-packages`. rclpy's Python package is in `/opt/ros/humble/lib/python3.10/site-packages/rclpy/` but its compiled C extension `_rclpy_pybind11.cpython-310-...so` is loaded via a relative import. If the `PYTHONPATH` prepends the venv's site-packages ahead of `/opt/ros/humble/lib/python3.10/site-packages`, Python finds the rclpy directory but cannot locate `_rclpy_pybind11`, raising `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`. This is distinct from rclpy simply not being installed — the directory exists but the `.so` is inaccessible.

**Why it happens:**
`setup_env.sh` exports `PYTHONPATH` for hailo-apps. If that path comes before the ROS site-packages, any pure-Python stub in the venv that shadows an rclpy submodule will hide the `.so`. Also occurs if `/opt/ros/humble/setup.bash` was never sourced — `LD_LIBRARY_PATH` won't include `/opt/ros/humble/lib`, which rclpy's `.so` needs at load time.

**How to avoid:**
(1) In `setup_env.sh`, append (not prepend) ROS paths: `source /opt/ros/humble/setup.bash` must run *after* hailo-apps paths are set but with ROS site-packages at the end of `PYTHONPATH`, not the start. (2) Gate rclpy import defensively: `try: import rclpy except ImportError as e: raise SystemExit(f"[rover] rclpy unavailable — run: source /opt/ros/humble/setup.bash ({e})")`. Never let the app proceed silently with `rclpy = None` hidden behind a flag — it produces confusing `None has no attribute 'init'` errors at runtime.

**Warning signs:**
`ImportError: No module named 'rclpy._rclpy_pybind11'` in the rover adapter thread; `python3 -c "import rclpy"` fails inside the venv even though `python3 -c "import rclpy"` works outside it.

**Phase to address:** Phase 2 (rover adapter) — add the defensive import guard; Phase 1 (abstraction) — document venv/ROS sourcing order in `setup_env.sh`.

---

### Pitfall 3: Blocking `rclpy.spin()` called instead of executor spin in a thread

**What goes wrong:**
`rclpy.spin(node)` is a blocking call that runs forever. If called on the drone-follow background rover thread, it blocks that thread indefinitely, preventing the `shutdown` asyncio Event from ever being checked. The rover adapter never stops, `drone_thread.join(5.0)` will always timeout, and the `pkill mavsdk_server` fallback at app shutdown fires unnecessarily.

**Why it happens:**
Every rclpy tutorial uses `rclpy.spin(node)` as the canonical "run the node" idiom. It looks right. The background-thread pattern requires `executor.spin_once(timeout_sec=0.1)` in a loop that checks `shutdown.is_set()`, or `executor.spin()` with the node destroyed and `rclpy.shutdown()` called in the `finally` block of the thread function.

**How to avoid:**
In `ros2_rover.py`, the rover thread must follow this pattern:
```python
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(node)
while not shutdown.is_set():
    executor.spin_once(timeout_sec=0.05)
executor.shutdown()
node.destroy_node()
rclpy.shutdown()
```
Never call `rclpy.spin(node)` or `executor.spin()` (the no-timeout variant) in a thread that needs to be stopped externally.

**Warning signs:**
App hangs on `drone_thread.join(5.0)` after Ctrl+C; rover thread still alive 5 s after shutdown set; `pkill mavsdk_server` fires on every clean exit.

**Phase to address:** Phase 2 (rover adapter) — enforce the spin-loop pattern in the adapter; add a shutdown integration test.

---

### Pitfall 4: rclpy + asyncio cross-thread Event coordination deadlock

**What goes wrong:**
The asyncio `shutdown` Event lives on the drone-follow background thread's event loop (`asyncio.new_event_loop()`). If rover adapter code calls `shutdown.set()` from the rclpy executor thread (a third, non-asyncio thread), it works — `threading.Event.set()` is thread-safe. However, if the rover adapter tries to call any `asyncio` primitive (e.g., `loop.call_soon_threadsafe`, awaiting a future) on the wrong loop, it silently does nothing or raises `RuntimeError: no running event loop`. The existing `run_drone` thread creates its own loop; the main thread has no asyncio loop (GStreamer owns main). A third rclpy thread must never assume it can use `asyncio.get_event_loop()`.

**Why it happens:**
Developers see `shutdown` (an `asyncio.Event`) and assume they need `asyncio` to set it. `asyncio.Event` is not thread-safe for cross-thread use; only `threading.Event` is. The fix is to use `threading.Event` for the shutdown signal, or to use `loop.call_soon_threadsafe(shutdown.set)` with a captured reference to the correct loop.

**How to avoid:**
Pass the rover adapter a `threading.Event` for shutdown (the existing `eos_reached` / `shutdown` events in `drone_follow_app.py` are `asyncio.Event` — a new `threading.Event rover_shutdown` should be introduced, or the drone asyncio loop reference must be passed and used via `loop.call_soon_threadsafe`). Document this explicitly in the `Robot` protocol.

**Warning signs:**
Rover adapter calls `shutdown.set()` (looks fine in code review) but drone never lands; asyncio task hangs waiting on `shutdown.wait()` indefinitely.

**Phase to address:** Phase 1 (Robot protocol design) — decide on threading.Event vs asyncio.Event at the boundary; Phase 2 (adapter) — enforce the correct signalling pattern.

---

### Pitfall 5: Gazebo Garden DiffDrive plugin SDF name is version-specific and tutorial-incompatible

**What goes wrong:**
Tutorials and Stack Overflow answers for Gazebo pre-Garden use plugin filenames like `libgazebo_ros_diff_drive.so` (Gazebo Classic) or class names like `ignition::gazebo::systems::DiffDrive` (Fortress/Citadel). In Garden the correct plugin name in SDF is `gz::sim::systems::DiffDrive` with filename `gz-sim-diff-drive-system`. Using the `ignition::` prefix causes a silent load failure — Gazebo starts but the robot does not move, and `/cmd_vel` messages are silently ignored with no error.

**Why it happens:**
The Ignition → Gz rename happened between Fortress and Garden. Tutorials from 2022 and earlier use the old names. GitHub search results mix both eras heavily.

**How to avoid:**
Use the Garden-era SDF from the official gz-sim repository at the `gz-sim7` branch (Garden = gz-sim 7) as the reference. The working plugin block is:
```xml
<plugin filename="gz-sim-diff-drive-system"
        name="gz::sim::systems::DiffDrive">
```
Pin the `sim/rover/` SDF to this and add a comment referencing the gz-sim7 example path.

**Warning signs:**
`gz sim` starts without errors but `ros2 topic echo /cmd_vel` shows messages arriving while the rover model remains stationary; no "loaded plugin" log line for DiffDrive in Gazebo output.

**Phase to address:** Phase 3 (sim construction) — validate DiffDrive plugin loads by checking Gazebo stdout for "Loaded system" during sim bring-up.

---

### Pitfall 6: ros_gz_bridge topic name mismatch between ROS and Gazebo

**What goes wrong:**
ros_gz_bridge requires explicit topic-name mappings. The Gazebo topic for DiffDrive by default is `/model/<model_name>/cmd_vel`. If the bridge config maps ROS `/cmd_vel` to Gazebo `/cmd_vel` (omitting the model prefix), messages are delivered to the bridge but never reach the plugin — no error, rover doesn't move.

**Why it happens:**
The bridge uses a YAML or CLI config that must match both sides exactly. Many examples show `/cmd_vel` on both sides for simplicity; the DiffDrive plugin registers on the model-namespaced topic by default unless `<topic>` is overridden in the SDF.

**How to avoid:**
Either (A) override the DiffDrive `<topic>` in the SDF to `/cmd_vel` directly (simpler), or (B) configure the bridge to map `ROS:/cmd_vel` → `GZ:/model/rover/cmd_vel`. Prefer option A — it keeps the bridge config trivial and the rover adapter publishes to `/cmd_vel` with no remapping needed.

**Warning signs:**
`ros2 topic echo /cmd_vel` shows Twist messages; `gz topic -e -t /cmd_vel` shows nothing; `gz topic -e -t /model/rover/cmd_vel` shows them instead.

**Phase to address:** Phase 3 (sim) — verify with `gz topic -l` that the DiffDrive plugin's subscribed topic name matches the bridge mapping before writing any Python.

---

### Pitfall 7: Gazebo Garden is EOL (Nov 2024) — apt packages may not receive security fixes

**What goes wrong:**
Gazebo Garden reached end-of-life in November 2024. The repo still has binary packages but they receive no further updates. For a sim-only milestone on Ubuntu 22.04 this is acceptable, but if any CVE or ABI break appears in a transitive dependency the packages will not be patched. More concretely: ROS 2 Humble's officially paired sim is Fortress, not Garden; Garden packages for Humble come from `packages.osrfoundation.org` and could be removed or unmaintained.

**Why it happens:**
The existing `sim/PX4-Autopilot` already uses Garden (the PX4 SITL integration targets Garden). Reusing Garden for the rover sim avoids introducing a second Gazebo version. But the EOL status is a known technical debt.

**How to avoid:**
For v1.1 (sim-only) use Garden to stay consistent with the PX4 sim. Document in `sim/rover/README` that the migration path to Harmonic (the current LTS, EOL 2028) is straightforward: plugin names remain `gz::sim::systems::*`, SDF format is identical, only the apt package names change (`ros-humble-ros-gz-bridge` → `ros-humble-ros-gzharmonic-bridge`). Do not architect rover SDF in a way that requires Garden-specific behaviour. Harmonic migration cost is low if SDF uses the `gz::` prefix throughout.

**Warning signs:**
`apt upgrade` breaks `ros-humble-ros-gz*` packages; `gz sim --version` reports 7.x (Garden) while PX4 SITL also pulls Garden — this is expected and fine for v1.1.

**Phase to address:** Phase 3 (sim) — document EOL status and Harmonic migration path; defer actual migration to post-v1.1.

---

### Pitfall 8: Open-loop cmd_vel hides real-world latency and actuator dead-band

**What goes wrong:**
A sim differential-drive rover responds to `/cmd_vel` within one simulation step (~10 ms). Real rovers have CAN/serial actuator latency (50–200 ms), dead-band below which the wheels don't move, and wheel slip. The follow controller's yaw P-loop will be tuned against the sim's instant response; on real hardware it will oscillate or hunt because the same gains overshoot a slow actuator. There is no `/odom` feedback in v1.1 to detect this discrepancy.

**Why it happens:**
No sensor → model-reality gap is invisible. The controller looks well-tuned in sim and breaks on hardware v1.2 bring-up.

**How to avoid:**
(1) Add artificial latency to the rover sim: insert a 100 ms delay shim on `/cmd_vel` in `sim/rover/` to represent real actuator lag. (2) Add a rover-specific `configs/rover_simulation.json` with conservatively low gains vs the drone sim config. (3) Document in `sim/rover/README` that the sim does not model dead-band or slip. At v1.2 bring-up, the first calibration step must be measuring actual dead-band and adding it to the controller's deadband parameter.

**Warning signs:**
Rover follows perfectly in sim with sharp direction changes; real rover oscillates side-to-side at low speed or doesn't respond to small corrections.

**Phase to address:** Phase 3 (sim) — add latency shim and conservative config; Phase 4 (integration) — document v1.2 hardware bring-up calibration checklist.

---

## Integration Gotchas

| Integration | Common Mistake | Correct Approach |
|-------------|----------------|------------------|
| rclpy + venv | Not sourcing `/opt/ros/humble/setup.bash` before activating the hailo-apps venv | Source ROS first, then `source setup_env.sh`; document this order in `setup_env.sh` comments |
| ros_gz_bridge | Using `@` separator in topic bridge config for wrong direction (`gz@ros_type` vs `ros@gz_type`) | `geometry_msgs/msg/Twist@gz.msgs.Twist` for bidirectional; `]gz.msgs.Twist` suffix for GZ→ROS only |
| rclpy init | Calling `rclpy.init()` from a non-main thread and relying on Python signal hooks | Always call `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` from any non-main-thread context |
| DiffDrive + bridge | Bridging a compressed image topic that Gazebo publishes as raw | Use `ros_gz_image` package (`image_bridge`) for image topics, not `ros_gz_bridge` |
| PX4 sim + rover sim | Running both sims simultaneously on the same machine | PX4 SITL uses ports 14540 (MAVLink) and 5600 (video); rover sim must not bind these. Use different Gazebo world files in different terminals with isolated port assignments |

---

## Technical Debt Patterns

| Shortcut | Immediate Benefit | Long-term Cost | When Acceptable |
|----------|-------------------|----------------|-----------------|
| Skip `/odom` subscription in v1.1 | Simpler rover adapter, no odometry noise handling | Controller blind to wheel slip; v1.2 hardware tuning is harder without feedback baseline | Acceptable for sim-only v1.1; must add in v1.2 |
| Reuse Garden instead of migrating to Harmonic | Consistent with existing PX4 SITL | Garden is EOL Nov 2024; no security updates | Acceptable for sim-only dev; migrate before shipping hardware |
| `SingleThreadedExecutor` for rclpy | Simple, no lock complexity | Cannot run parallel callbacks (not needed for publish-only) | Always acceptable for v1.1 publish-only adapter |

---

## "Looks Done But Isn't" Checklist

- [ ] **rclpy import guard:** `import rclpy` succeeds inside the hailo-apps venv with a helpful error message if ROS is not sourced — verify with `setup_env.sh` only (without `source /opt/ros/humble/setup.bash`).
- [ ] **Signal handler preserved:** After `rclpy.init()`, `signal.getsignal(signal.SIGINT)` still returns drone-follow's `on_signal` function, not rclpy's default handler.
- [ ] **Rover stops on shutdown:** After Ctrl+C, rover publishes a zero Twist before the thread exits — verify no `cmd_vel` messages arrive after the shutdown log line.
- [ ] **Bridge topic verified:** `gz topic -e -t /cmd_vel` (or the model-namespaced variant) shows Twist messages when the rover adapter publishes — not just `ros2 topic echo /cmd_vel`.
- [ ] **DiffDrive plugin loaded:** Gazebo stdout contains `Loaded system [gz::sim::systems::DiffDrive]` — not just "no errors at startup."
- [ ] **Port isolation:** Starting PX4 SITL and rover sim simultaneously does not produce "address already in use" on 14540 or 5600.

---

## Pitfall-to-Phase Mapping

| Pitfall | Prevention Phase | Verification |
|---------|------------------|--------------|
| rclpy SIGINT override | Phase 2 (rover adapter) | `signal.getsignal(signal.SIGINT)` check in rover adapter constructor |
| `_rclpy_pybind11` import failure | Phase 2 (adapter) + Phase 1 (setup_env.sh docs) | Defensive import guard with friendly error; CI test without ROS sourced |
| Blocking `rclpy.spin()` | Phase 2 (rover adapter) | Shutdown integration test: Ctrl+C lands cleanly in < 8 s |
| asyncio.Event cross-thread | Phase 1 (Robot protocol) | Review boundary type before any adapter code is written |
| DiffDrive plugin name | Phase 3 (rover sim) | `gz topic -l` shows plugin-registered topics before any ROS code |
| ros_gz_bridge topic mismatch | Phase 3 (rover sim) | `gz topic -e` confirms messages reach Gazebo before rover adapter is wired |
| Gazebo Garden EOL | Phase 3 (rover sim) | Document migration notes; do not use Garden-specific APIs |
| Open-loop sim hides latency | Phase 3 (sim) + Phase 4 (integration) | Add latency shim; document v1.2 calibration steps |

---

## Sources

- [rclpy asyncio executor PR #971 — ros2/rclpy](https://github.com/ros2/rclpy/pull/971)
- [rclpy asyncio feature request Issue #1461 — ros2/rclpy](https://github.com/ros2/rclpy/issues/1461)
- [rclpy signal handling refactor Issue #400 — ros2/rclpy](https://github.com/ros2/rclpy/issues/400)
- [rclpy SIGINT threads Issue #192 — ros2/rclpy](https://github.com/ros2/rclpy/issues/192)
- [rclpy venv issue #1469 — ros2/ros2](https://github.com/ros2/ros2/issues/1469)
- [rclpy._rclpy_pybind11 import failure Issue #1194 — ros2/rclpy](https://github.com/ros2/rclpy/issues/1194)
- [ros2 publisher thread safety — ROS Answers](https://answers.ros.org/question/342625/ros2-publisher-thread-safe/)
- [DiffDrive plugin in Gazebo Sim — The Construct](https://get-help.theconstruct.ai/t/issues-with-differential-drive-plugin-in-gazebo-sim/27014)
- [gz-sim diff_drive.sdf example (gz-sim8 branch)](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/diff_drive.sdf)
- [ros_gz_bridge README — gazebosim/ros_gz](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md)
- [gz bridge remap — ROS Answers](https://answers.ros.org/question/416665/gz-bridge-remap/)
- [Gazebo Garden releases and EOL schedule](https://gazebosim.org/docs/garden/releases/)
- [ROS 2 Integration — Gazebo Garden docs](https://gazebosim.org/docs/garden/ros2_integration/)
- [ros_gz_bridge camera performance Issue #368 — gazebosim/ros_gz](https://github.com/gazebosim/ros_gz/issues/368)

---
*Pitfalls research for: ROS 2 rclpy + Gazebo Garden rover sim integration into drone-follow*
*Researched: 2026-05-12*
