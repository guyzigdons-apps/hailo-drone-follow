---
name: openhd_pairing
description: WFB pairing keys, recovery from key drift, and the dangerous `openhd --clean-start` flag.
type: reference
---

# OpenHD pairing — txrx.key

## Authoritative location
`/usr/local/share/openhd/txrx.key` (root-owned, 128 bytes). Both air and ground sides must hold byte-identical copies for WFB encryption/decryption to round-trip.

## Symptom of mismatch
Ground unit sees a black screen, scan-for-air-unit menu finds nothing, no obvious error. WFB packets are encrypted on one side and silently dropped on the other.

## **Never** run `openhd --clean-start`
On a paired air/ground link, `--clean-start`:
1. Regenerates `/usr/local/share/openhd/txrx.key` to a new random key. Pair breaks.
2. Resets `/usr/local/share/openhd/video/air_camera_generic.json` to defaults (`primary_camera_type=31`), undoing Mode A — Picamera2 then loses the camera with `Device or resource busy`.

## Recovery options (when keys have drifted)
1. **Sync keys** — `scp` one side's `txrx.key` to the other. Any 128-byte file works as long as both sides match.
2. **Reset both to OpenHD's example key** (deterministic, no scp if both Pis have the source tree):
   ```bash
   sudo cp <openhd-src>/OpenHD/ohd_interface/lib/wifibroadcast/example_key/txrx.key \
           /usr/local/share/openhd/txrx.key
   ```
   Restart OpenHD on each side **without** `--clean-start`.

## Mode-switch (Mode A ↔ Mode B) the safe way
Use `scripts/install_air.sh --mode <stream|shm>` — it only flips `primary_camera_type` + the `/boot/openhd/hailo.txt` flag and does NOT regenerate the key.

## Other txrx.key copies on the dev box (informational, NOT authoritative)
- `/home/giladn/Downloads/txrx.key` — stray
- `/home/giladn/OpenHD-bak.<date>/.../example_key/txrx.key` — backup of OpenHD repo's example key
- `<repo>/community/apps/hailo_drone_follow/txrx.key` — copy in this repo's root
- `<repo>/community/apps/hailo_drone_follow/OpenHD/.../example_key/txrx.key` — vendored OpenHD example key

When the user asks "where is the txrx.key", they almost always mean the runtime one at `/usr/local/share/openhd/txrx.key`.
