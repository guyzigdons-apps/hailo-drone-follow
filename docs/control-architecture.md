# Drone Control Architecture

How `drone-follow` turns a camera stream into drone motion — from a person detection to a MAVLink velocity setpoint — and the parameters that shape every step.

Audience: someone who wants to understand or tune the control loop, not someone onboarding to the codebase generally.

---

## 1. Control pipeline, end-to-end

```
 GStreamer pipeline (Hailo NPU)        Thread-safe bridge                 Async control loop                     MAVSDK                PX4
 ──────────────────────────────        ─────────────────                  ──────────────────                     ──────                ───
 camera / SHM / udp                                                                                                                       
      │                                                                                                                                   
      ▼                                                                                                                                   
 hailo-inference (YOLO) ─► ByteTracker ─► app_callback ─► SharedDetectionState.update(Detection)                                          
                                         (pipeline_adapter/                                                                                
                                          hailo_drone_detection_manager.py)                                                                
                                                                    │                                                                      
                                                                    ▼                                                                      
                                                           10 Hz loop                                                                      
                                                   shared_state.get_latest()                                                               
                                                                    │                                                                      
                                                                    ▼                                                                      
                                                   compute_velocity_command(...)  ◄── ControllerConfig                                     
                                                   (follow_api/controller.py)                                                              
                                                                    │                                                                      
                                                                    ▼                                                                      
                                            alt-hold P loop             (always active)                                                    
                                                                    │                                                                      
                                                                    ▼                                                                      
                                               VelocityCommandAPI.send()  ──► drone.offboard.set_velocity_body()                           
                                               (clamp + per-axis EMA)       (mavsdk_drone.py)                                              
                                                                                                  │                                        
                                                                                                  ▼                                        
                                                                                    MAVLink SET_POSITION_TARGET_LOCAL_NED                  
                                                                                    (body frame, velocity-only mask, yawspeed)             
                                                                                                  │                                        
                                                                                                  ▼                                        
                                                                                    PX4 OFFBOARD → mc_pos_control                          
```

Key idea: the app does NOT command attitude, thrust, waypoints, or position. It emits **one 4-DOF velocity setpoint per tick in the drone body frame**, and PX4's position controller does the rest. This is the standard MAVLink/PX4 offboard pattern for companion computers.

---

## 2. The one primitive: body-frame velocity + yawrate

Every control output is a `VelocityCommand` (`follow_api/types.py`):

```python
@dataclass
class VelocityCommand:
    forward_m_s:     float   # +X body (nose direction)
    right_m_s:       float   # +Y body (starboard)
    down_m_s:        float   # +Z body (down is positive — NED convention)
    yawspeed_deg_s:  float   # +ve = clockwise viewed from above
```

Sent via `mavsdk.offboard.VelocityBodyYawspeed`, which maps to MAVLink's `SET_POSITION_TARGET_LOCAL_NED` with:
- coordinate frame = `MAV_FRAME_BODY_NED`
- type mask = position+acceleration ignored, velocity + yawrate only

This is the **standard** offboard primitive recommended by PX4 for vision-based follow applications. Reference: https://docs.px4.io/main/en/flight_modes/offboard.html

The app never touches:
- setpoint_raw_attitude
- position targets (waypoints)
- thrust / actuator controls
- flight-mode switches (except `takeoff` / `land` when `--takeoff-landing` is passed)

---

## 3. Tracking input → four axes of output

The detection arriving at the control loop (`Detection` in `follow_api/types.py`) has four normalized scalars:

| Field | Range | Meaning |
|---|---|---|
| `center_x` | 0..1 | bbox centre x (0 = left, 1 = right) |
| `center_y` | 0..1 | bbox centre y (0 = top, 1 = bottom) |
| `bbox_height` | 0..1 | bbox height as fraction of frame height |
| `confidence` | 0..1 | detector confidence |

The controller maps these to the four output axes:

### 3.1 Yaw (`yawspeed_deg_s`) ← `center_x`

```python
error_x_deg = (center_x - 0.5) * hfov        # signed angular offset
if |error_x_deg| < dead_zone_deg:
    yawspeed = 0
else:
    yawspeed = sign(error_x_deg) * kp_yaw * sqrt(|error_x_deg|)
yawspeed = clamp(yawspeed, ±max_yawspeed)
# then: EMA low-pass filter in VelocityCommandAPI (yaw_alpha)
```

- **P controller with a square-root response** — softer near zero error, still quick on large errors.  Standard practice in vision-servo yawing to avoid the step-step-step feeling of pure-P at low gain.
- **Dead zone** (`dead_zone_deg = 2°`) suppresses jitter from noisy detections.
- **EMA low-pass** (`yaw_alpha = 0.3`) in `VelocityCommandAPI.send()` — filters the commanded yawspeed before it hits MAVSDK. All four axes have per-axis EMA in `send()`: yaw (α=0.3), forward (α=0.07), right (α=0.3), down (α=0.2).

### 3.2 Altitude — single mode, always fixed

Altitude is **always** held at `config.target_altitude` by a P loop in
`live_control_loop`. There is no follow-driven pitch/altitude mode and no
`fixed_altitude` flag — `compute_velocity_command` always returns
`down_m_s = 0`. See §3.5 for the alt-hold formula.

### 3.3 Forward / backward (`forward_m_s`) ← `bbox_height`

Plain P on the bbox-height error, with a single dead zone, two safety
bypasses, and a first-order low-pass filter at the output. The low-pass
is the key piece — it attenuates the ~1–2 Hz pitch-induced feedback loop
(no gimbal → body pitch tilts image) at the filter's ~0.11 Hz cutoff,
without any hysteresis, direction-hold, or derivative machinery.

```python
# 1. Absolute safety: too close → full reverse.
if bbox_height > max_bbox_height_safety:     # default 0.8
    return -max_backward

# 2. Bottom-of-frame safety: person directly beneath → retreat.
bbox_bottom = center_y + bbox_height/2
if bbox_bottom > bottom_y_threshold:         # default 0.7
    return -kp_backward * sqrt(overshoot)

# 3. Single-threshold dead zone.
height_delta = target_bbox_height - bbox_height
dead_zone = (dead_zone_height_percent / 100) * target_bbox_height   # default 20%
if abs(height_delta) < dead_zone:
    return 0.0

# 4. Signed P with asymmetric gains.
gain = kp_forward if height_delta > 0 else kp_backward
raw = gain * height_delta
forward = clamp(raw, -max_backward, max_forward)

# 5. Output low-pass (VelocityCommandAPI per-axis EMA): first-order EMA.
#    forward_alpha = 0.07 @ 10 Hz  →  τ ≈ 1.4 s, cutoff ≈ 0.11 Hz
smoothed = alpha * forward + (1 - alpha) * prev_smoothed
```

Why the low-pass is the right fix:
- The pitch-induced error signal sits at ~1–2 Hz (driven by the drone's
  pitch dynamics). An EMA with cutoff at 0.11 Hz attenuates this band by
  ~25 dB while leaving the sub-0.1 Hz tracking bandwidth intact.
- No hysteresis, no direction-hold, no state machine — just a single
  scalar knob (`forward_alpha`) that trades phase lag for attenuation.
- Safety bypasses fire before the filter so emergency retreat is still
  instantaneous.

**Evolution:** earlier versions of this axis stacked spatial hysteresis
(25 % / 12 %), a smooth ramp, signed square-root P, a 0.8 s temporal
direction-hold filter, and a derivative feed-forward. Each was motivated
empirically but collectively they were hard to tune. Commit `65c4318`
("simplify: replace stacked oscillation mitigations with low-pass P")
replaced the stack with the simpler form above.

### 3.4 Lateral (`right_m_s`) ← orbit mode

Only non-zero when `follow_mode == "orbit"`:

```python
right = orbit_speed_m_s * orbit_direction   # direction = ±1
```

Constant lateral velocity while yaw keeps the person centred → drone orbits the subject. Standard cinematographic follow pattern.

In default `follow` mode, `right_m_s = 0`.

### 3.5 Altitude hold (the only altitude loop)

A single P controller in `live_control_loop` holds altitude at
`target_altitude`:

```python
alt_error = target_altitude - current_altitude
if |alt_error| > 0.1:                       # dead zone
    down = clamp(-0.5 * alt_error, ±1.0)    # kp=0.5, max ±1 m/s
```

`target_altitude` is live-mutable from the web UI. `current_altitude` comes
from `drone.telemetry.position().relative_altitude_m`. There is no
follow-driven alternative; `compute_velocity_command` does not produce a
`down_m_s` at all.

---

## 4. Modes (finite-state machine around the same controller)

The control loop itself has no explicit FSM — it computes a command based on (detection, last detection, time-since-detection). But externally it behaves as:

| Mode | Entered when | Behaviour |
|---|---|---|
| **TRACK** | valid detection <0.5s old | full 4-axis command as above |
| **SEARCH-WAIT** | no detection for <2s | hold last command (`hold_velocity = _prev_cmd`) |
| **SEARCH** | no detection for ≥2s | slow yaw spin toward last-seen side at `search_yawspeed_slow = 10°/s`, dampened forward |
| **IDLE** | operator pressed pause in UI | `shared_state.update(None)` + the loop treats it as "no detection, no search" → hovers |
| **ORBIT** | `follow_mode = "orbit"` | same as TRACK but with constant lateral velocity |
| **Landing** | search timeout (60s) exceeded | shutdown + `action.land()` |

TRACK ↔ SEARCH transitions are naturally hysteretic because `detection_timeout_s = 0.5` and `search_enter_delay_s = 2.0` — different thresholds for "lost" vs "start spinning".

---

## 5. How parameters map to behaviour

The following table gives a field-tuner's view of what to change when. Every value below is a field on `ControllerConfig` (`follow_api/config.py`), settable via CLI flag, JSON file (`--config`), or the web UI's POST `/config` endpoint.

### Framing & target size
| Param | Default | Tune if |
|---|---|---|
| `hfov` / `vfov` | 66 / 41 ° | Camera FOV changed |
| `target_bbox_height` | 0.3 | Want subject bigger/smaller in frame |
| `target_distance_m` | None | Prefer absolute distance (requires `--fixed-altitude`) |
| `person_height_m` | 1.7 m | Target is a child / taller person |

### Yaw
| Param | Default | Tune if |
|---|---|---|
| `kp_yaw` | 5.0 | Yaw too slow / too twitchy |
| `dead_zone_deg` | 2 ° | Twitching on a still target |
| `max_yawspeed` | 90 °/s | Too aggressive on fast cuts |
| `yaw_alpha` (EMA) | 0.3 | 0 = never responds, 1 = no smoothing |

### Altitude hold
| Param | Default | Tune if |
|---|---|---|
| `target_altitude` | 3.0 m | Flight altitude (live-mutable) |
| `max_down_speed` | 1.5 m/s | Safety clamp on the down axis |
| alt-hold `kp` | 0.5 (hardcoded) | Altitude hold too soft/stiff — requires code change |
| alt-hold max speed | 1.0 m/s (hardcoded) | Altitude correction too aggressive |
| alt-hold dead zone | 0.1 m (hardcoded) | Jitters near target |

### Forward / backward
| Param | Default | Tune if |
|---|---|---|
| `kp_forward` / `kp_backward` | 1.5 / 2.5 | Approach/retreat too slow; raise cautiously |
| `max_forward` / `max_backward` | 1.0 / 1.5 m/s | Hard cap on speeds |
| `dead_zone_height_percent` | 20 % | Oscillating → widen; unresponsive → narrow |
| `forward_alpha` (EMA) | 0.07 | Lower = more pitch-oscillation attenuation, more phase lag |
| `max_bbox_height_safety` | 0.8 | Hard emergency-retreat threshold |
| `bottom_y_threshold` | 0.7 | "Person directly below me" trigger |

### Search behaviour
| Param | Default | Tune if |
|---|---|---|
| `search_enter_delay_s` | 2.0 | Too quick/slow to start spinning |
| `search_yawspeed_slow` | 10 °/s | Spin too fast/slow |
| `search_timeout_s` | 60 s | Give up and land |
| `search_vel_damp` | 0.3 | Dampen forward during search |

### Orbit
| Param | Default | Tune if |
|---|---|---|
| `orbit_speed_m_s` | 1.0 | Orbit radius / speed |
| `orbit_direction` | +1 | CW / CCW |

### Safety / flight envelope
| Param | Default | Notes |
|---|---|---|
| `yaw_only` | True | Safest default — no translation |
| `control_loop_hz` | 10 Hz | Rarely needs tuning |
| `detection_timeout_s` | 0.5 | Staleness cutoff for a detection |

### Runtime mutation
All of the above fields (those exposed in `_CONFIG_FIELDS` in `servers/web_server.py`) can be changed at runtime via the UI. The control loop reads `config.*` every tick, so a mid-flight edit takes effect within one control period.

---

## 6. OFFBOARD integration with PX4

Two flight-lifecycle modes, controlled by `--takeoff-landing`:

### Default: pilot-managed lifecycle (recommended)
- Drone is already airborne under RC or another autonomous mode.
- App connects, streams `VelocityBodyYawspeed(0,0,0,0)` at 20 Hz as a keep-alive setpoint.
- Pilot switches the flight mode to OFFBOARD via GCS/RC.
- On OFFBOARD detection (`telemetry.flight_mode() == OFFBOARD`), the control loop starts producing real commands.
- If the pilot switches out of OFFBOARD at any point, the control loop pauses and the app waits for re-entry.
- **The app never commands mode changes.** This is the safe handover pattern.

### With `--takeoff-landing`: app-managed lifecycle
- `drone.action.set_takeoff_altitude()` → `action.arm()` (retried 6×) → `action.takeoff()` → `_start_offboard()`.
- `_start_offboard` streams zero setpoints for 2 s (PX4 rejects `offboard.start()` if there's no setpoint history — `NO_SETPOINT_SET`) then calls `offboard.start()` with 3× retry.
- On Ctrl+C or mission-duration expiry → `_land_safely()` → `offboard.stop()` + `action.land()`, with SIGINT ignored during landing so a second Ctrl+C can't abort mid-flare.

### Required PX4 parameters (per project CLAUDE.md)
- `COM_RC_IN_MODE = 4` — allow flight without RC
- `COM_RCL_EXCEPT` bit 2 set — ignore RC loss in offboard
- `COM_OF_LOSS_T` — offboard signal loss timeout (~1 s)
- `COM_OBL_RC_ACT` — failsafe action on offboard signal loss

These are entirely standard for companion-computer offboard.

---

## 7. Target selection (who to follow)

Target-ID selection lives in the **pipeline callback**, not the controller. The controller only ever sees one `Detection` (or `None`).

1. `app_callback` in `hailo_drone_detection_manager.py` receives all detections + tracker IDs.
2. It picks the single target based on `FollowTargetState`:
   - If operator explicitly locked an ID via UI → follow only that ID. If lost, fall back to IDLE (hover).
   - If no explicit lock → automatically follow the largest visible person's bbox (`max(persons, key=bbox_area)`).
   - If no persons at all → `Detection = None`.
3. `shared_state.update(Detection(...))` — the control loop picks it up next tick.

Target ID persistence is provided by **ByteTracker** (standard MOT algorithm, `pipeline_adapter/byte_tracker.py`) running synchronously in the callback. `track_thresh=0.4, track_buffer=90, match_thresh=0.5` — reasonably persistent; a track survives ~3s of occlusion at 30fps.

---

## 8. What's standard, what's custom

| Piece | Standard? | Notes |
|---|---|---|
| Body-velocity + yawrate MAVLink setpoint | ✅ Canonical PX4 offboard | `SET_POSITION_TARGET_LOCAL_NED` body frame |
| Streaming zero-setpoint pre-offboard | ✅ Required by PX4 | Avoids `NO_SETPOINT_SET` |
| `drone.action.arm/takeoff/land` via MAVSDK | ✅ Standard MAVSDK | |
| Dual-path (app-managed vs pilot-managed) lifecycle | ✅ Common pattern | Safer for real flight |
| ByteTracker for ID persistence | ✅ Standard MOT | Widely used in vision + follow apps |
| P controllers per axis | ✅ Standard | Yaw, forward, alt-hold are all plain P |
| Per-axis EMA low-pass in VelocityCommandAPI | ✅ Standard first-order filter | yaw α=0.3, forward α=0.07, right α=0.3, down α=0.2 — all applied in `send()` |
| Dead zones around zero error | ✅ Standard | Suppresses sensor noise |
| Clamps / max-speed saturation | ✅ Standard | |
| Image-based visual servoing (center_x → yaw, bbox_height → distance) | ✅ Textbook IBVS | Classic Chaumette/Hutchinson formulation at a very simplified level |
| Single-loop altitude hold (no follow-driven pitch) | ✅ Standard | One loop, one target, no dual mode |
| **Signed square-root response** (yaw) | ⚠️ Common in robotics, not classical PID | Softens step near zero; widely used in ArduPilot / UAV loops |
| **Bottom-of-frame backup safety** | ❌ Custom | Vision-specific safety on "person underneath me" |
| **Bbox-height safety → full reverse** | ❌ Custom | Vision-specific emergency retreat |
| **IDLE ↔ auto-target fallback** on explicit-lock loss | ❌ Custom | UX choice, not a control-theory one |

Summary: the control architecture is now almost entirely textbook — offboard body-velocity commands, plain P per axis, dead zones, saturation, EMA smoothing. The only non-standard elements are the two vision-specific safety bypasses on the forward axis (bbox too big, person underneath) and the signed-sqrt yaw response. The previous stacked forward-oscillation mitigation (spatial hysteresis, direction-hold, D feed-forward, ramp, signed-sqrt forward) was replaced by a single low-pass filter in commit `65c4318`.

---

## 9. Suggested review / audit path

If you want to review end-to-end, read in this order:

1. **`follow_api/types.py`** — domain primitives (5 fields, 30 lines)
2. **`follow_api/config.py::ControllerConfig`** — every knob in one place
3. **`follow_api/controller.py::compute_velocity_command`** — the math
4. **`drone_api/mavsdk_drone.py::VelocityCommandAPI.send`** — clamp + per-axis EMA (yaw, forward, right, down)
5. **`drone_api/mavsdk_drone.py::live_control_loop`** — the 10 Hz loop + alt hold
6. **`drone_api/mavsdk_drone.py::run_live_drone`** — lifecycle, takeoff-landing, offboard-handover
7. **`pipeline_adapter/hailo_drone_detection_manager.py::app_callback`** — target selection, IDLE fallback, ByteTracker
8. **`servers/web_server.py::_CONFIG_FIELDS`** — runtime-mutable subset of config

Each of those is short and single-purpose. The entire control surface fits in ~1000 lines.

---

## 10. Known risks / things I'd look at next

- **Forward low-pass adds phase lag.** With α=0.07 the time constant is ~1.4 s. The drone reacts slowly to abrupt distance changes (a person suddenly sprinting away will be chased with ~1 s delay before the commanded velocity catches up). If flight tests show this is too sluggish, raise α to 0.10 and retest for oscillation; if oscillation returns, the right fix is a gimbal, not more filter.
- **Alt-hold gains are hardcoded** (`_ALT_HOLD_KP = 0.5`, `_ALT_HOLD_MAX_SPEED = 1.0`, `0.1 m` dead zone). Promote to `ControllerConfig` if tuning needs emerge.
- **No integral term anywhere.** Any steady-state error (e.g. wind pushing the drone sideways while trying to hold position) is not corrected by this controller — PX4's inner loops handle it, but if you notice persistent offsets under wind, consider adding I to the alt-hold loop.
- **No gimbal.** The camera is rigidly mounted, so body pitch couples into visual pitch (the original oscillation driver). The low-pass fix works because the coupling is at a higher frequency than the useful tracking band — but a stabilised gimbal would eliminate the coupling at source and let you run a faster forward controller.
