# tiny.ulg — PX4 ULog fixture

A ~166 KB trimmed copy of a real PX4 SITL flight log, used by
`hailo_tiling.tests.test_telemetry_ulg` (Plan 7 Task 2).

## Provenance

Source log: `~/Desktop/DroneLogs/log_0_2026-3-23-11-12-10.ulg` (~233 KB),
generated on the development workstation from a short PX4 SITL session.

## Build procedure

Trimmed in place via `pyulog`'s `message_name_filter_list` to keep only the
topics this plan reads, then anonymised GPS fields by zeroing
`vehicle_gps_position.{latitude_deg, longitude_deg}` and
`vehicle_local_position.{ref_lat, ref_lon}`. `altitude_msl_m` is left as-is
(it's relative to MSL on the sim's home elevation; not personally
identifying).

Reproduce with:

```python
import pyulog
keep = ['vehicle_local_position', 'vehicle_attitude', 'vehicle_air_data',
        'vehicle_gps_position', 'vehicle_global_position',
        'vehicle_angular_velocity', 'sensor_combined']
u = pyulog.ULog("<source>.ulg", message_name_filter_list=keep)
for d in u.data_list:
    if d.name == 'vehicle_gps_position':
        d.data['latitude_deg'][:] = 0.0
        d.data['longitude_deg'][:] = 0.0
    if d.name == 'vehicle_local_position':
        d.data['ref_lat'][:] = 0.0
        d.data['ref_lon'][:] = 0.0
u.write_ulog("tiny.ulg")
```

## Properties

- Duration: ~1.9 s of flight data (boot-relative).
- Topics present: `vehicle_local_position`, `vehicle_attitude`,
  `vehicle_air_data`, `vehicle_gps_position`, `vehicle_angular_velocity`,
  `sensor_combined`.
- `vehicle_local_position` sample count: 21 (≈ 11 Hz).
- GPS: lat/lon are zeroed; `altitude_msl_m` is sim-original.
