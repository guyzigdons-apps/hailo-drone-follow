# DroneFollow Parameter System — Architecture Reference

## Overview

The drone-follow parameter control path spans three software layers across
two Raspberry Pi units connected via wifibroadcast:

```
┌──────────────────────── AIR UNIT (RPi5 + Hailo8) ─────────────────────────┐
│                                                                            │
│  ┌──────────────────────┐    UDP JSON      ┌───────────────────────────┐  │
│  │  drone-follow        │◄────────────────►│  OpenHD air               │  │
│  │  (Python)            │  port 5510/5511  │  hailo_follow_bridge.cpp  │  │
│  │                      │                  │                           │  │
│  │  • Applies params    │  5510: OpenHD→Py │  • Loads df_params.json   │  │
│  │    to control loop   │  5511: Py→OpenHD │  • MAVLink param server   │  │
│  │  • Reports current   │                  │  • Persists values to disk│  │
│  │    values back       │                  │  • Translates MAVLink ↔   │  │
│  └──────────────────────┘                  │    UDP JSON               │  │
│                                            └─────────────┬─────────────┘  │
│                                              wifibroadcast (MAVLink relay) │
└──────────────────────────────────────────────────────────┼─────────────────┘
                                                           │ RF link
┌──────────────────────────────────────────────────────────┼─────────────────┐
│                                          GROUND (RPi4)   │                 │
│                                            ┌─────────────▼─────────────┐  │
│                                            │  OpenHD ground            │  │
│                                            │  • MAVLink relay          │  │
│                                            └─────────────┬─────────────┘  │
│                                                          │                 │
│                                            ┌─────────────▼─────────────┐  │
│                                            │  QOpenHD                  │  │
│                                            │  • Loads df_params.json   │  │
│                                            │    (UI metadata only)     │  │
│                                            │  • DroneFollow settings   │  │
│                                            │    tab: sliders/switches  │  │
│                                            │  • PARAM_EXT_SET/GET      │  │
│                                            └───────────────────────────┘  │
└───────────────────────────────────────────────────────────────────────────┘
```

---

## Parameter Set Flow

```
QOpenHD slider
  → MAVLink PARAM_EXT_SET
    → OpenHD ground (relay)
      → wifibroadcast RF
        → OpenHD air / hailo_follow_bridge
          → persist value to disk
          → UDP JSON {"param": "kp_yaw", "value": 5.1} to port 5510
            → drone-follow Python (applies immediately)
          → MAVLink PARAM_EXT_ACK back to QOpenHD
```

## Parameter Readback Flow

```
drone-follow Python
  → UDP JSON {"params": {"kp_yaw": 5.1, ...}} to port 5511
    → OpenHD air / hailo_follow_bridge (updates cache)
      → MAVLink PARAM_EXT_VALUE
        → wifibroadcast RF
          → OpenHD ground (relay)
            → QOpenHD (updates slider position)
```

---

## The Bridge (hailo_follow_bridge.cpp)

The bridge sits inside OpenHD on the air unit and translates between:

- **MAVLink side**: Registers all DF_ parameters from `df_params.json` as
  MAVLink extended parameters. QOpenHD can get/set them using standard
  PARAM_EXT_SET/PARAM_EXT_REQUEST_READ messages.

- **UDP JSON side**: Communicates with drone-follow on localhost:
  - Port **5510** (OpenHD → Python): `{"param": "kp_yaw", "value": 5.1}`
  - Port **5511** (Python → OpenHD): `{"params": {"kp_yaw": 5.1, ...}}`

---

## The df_params.json Schema

A single JSON file defines every DF_ parameter. All three layers read it:

- **OpenHD air** (C++): Loads at startup to register MAVLink params
- **QOpenHD** (QML): Loads at tab open to generate UI controls
- **drone-follow** (Python): Can read defaults from this file

**Location:** `/usr/local/share/openhd/df_params.json` (deployed on both units)

### Example

```json
{
  "version": 1,
  "groups": [
    {"id": "yaw", "label": "YAW CONTROL", "order": 1}
  ],
  "params": [
    {
      "id": "kp_yaw",
      "mavlink_id": "DF_KP_YAW",
      "type": "float",
      "default": 5.0,
      "min": 0.0,
      "max": 20.0,
      "step": 0.1,
      "group": "yaw",
      "order": 1,
      "label": "Kp Yaw",
      "description": "Proportional gain for yaw tracking.",
      "read_only": false
    }
  ]
}
```

### Field Reference

| Field | Description |
|-------|-------------|
| `id` | Internal Python field name (used in UDP JSON IPC) |
| `mavlink_id` | MAVLink parameter name (max 16 chars, must start with `DF_`) |
| `type` | `"float"` → slider, `"int"` → spin box, `"bool"` → toggle |
| `default` | Default value (used when no persisted value exists) |
| `min`/`max` | Range limits |
| `step` | Increment step for the UI control |
| `group` | Must match a group `id` from the `groups` array |
| `order` | Sort order within the group |
| `label` | Display name in QOpenHD |
| `description` | Tooltip / help text |
| `read_only` | If `true`, displayed as read-only text |
| `hidden` | If `true`, registered in MAVLink but hidden from DroneFollow tab |

### Adding a New Parameter

1. Add entry to `df_params.json`
2. Copy to `/usr/local/share/openhd/df_params.json` on both units
3. Restart OpenHD (air) and QOpenHD (ground) — no recompilation needed
4. (Optional) Handle in Python: `controller_config.get("my_param", 1.0)`
