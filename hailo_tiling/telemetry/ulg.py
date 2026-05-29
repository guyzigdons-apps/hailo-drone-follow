"""PX4 ULog (.ulg) -> JSONL row adapter.

Reads a PX4 ULog flight log and emits a list of dicts shaped like a
`RecordedTelemetry` JSONL timeline (one row per `vehicle_local_position`
message, with last-value-carried-forward joins from the other topics).

See `docs/superpowers/plans/2026-05-28-telemetry-import-visualizer.md`
Task 2 for the field-mapping contract.
"""
from __future__ import annotations

import bisect
import math
from pathlib import Path
from typing import Optional

import pyulog


# Topics we read; pyulog's `message_name_filter_list` limits parsing cost.
# We list both legacy (`vehicle_attitude.angular_velocity[2]`) and current
# (`vehicle_angular_velocity.xyz[2]`) sources for yaw-rate, plus
# `sensor_combined.gyro_rad[2]` as final fallback.
_TOPICS = (
    "vehicle_local_position",
    "vehicle_attitude",
    "vehicle_air_data",
    "vehicle_gps_position",
    "vehicle_global_position",
    "vehicle_angular_velocity",
    "sensor_combined",
)


def _get_dataset_or_none(ulog: "pyulog.ULog", name: str):
    """Return the dataset for `name`, or None if absent.

    pyulog's `get_dataset` raises `KeyError`-equivalent (an Exception) when the
    topic is missing. We turn that into None so the caller can skip cleanly.
    """
    try:
        return ulog.get_dataset(name)
    except Exception:
        return None


def _last_le(timestamps: list[int], values, t_us: int):
    """Return the value whose timestamp is the largest <= t_us, or None.

    `timestamps` must be sorted ascending. `values` is index-aligned with
    `timestamps`. Returns None if no timestamp <= t_us exists.
    """
    if not timestamps:
        return None
    idx = bisect.bisect_right(timestamps, t_us) - 1
    if idx < 0:
        return None
    v = values[idx]
    # numpy scalars -> Python floats; NaN -> None
    try:
        if isinstance(v, float) and math.isnan(v):
            return None
        f = float(v)
        if math.isnan(f):
            return None
        return f
    except (TypeError, ValueError):
        return v


def _column(dataset, field: str):
    """Return dataset.data[field] if available, else None."""
    if dataset is None:
        return None
    try:
        return dataset.data[field]
    except (KeyError, IndexError, TypeError):
        return None


def parse_ulg(path: Path) -> list[dict]:
    """Parse a PX4 ULog file into a list of telemetry row dicts.

    Each row has keys: ``timestamp``, ``altitude_agl_m``, ``yaw_rate_rad_s``,
    ``velocity_world``, ``attitude_quat``, plus a ``_geo`` sidecar.

    Time base: monotonic seconds, with the first ``vehicle_local_position``
    message mapped to ``timestamp=0.0``.

    Resampling rule: one row per ``vehicle_local_position`` message; values
    from other topics are joined last-value-carried-forward at that timestamp.

    On a malformed input, raises ``ValueError`` with the underlying pyulog
    error chained via ``raise ... from err``.
    """
    path = Path(path)
    try:
        ulog = pyulog.ULog(str(path), message_name_filter_list=list(_TOPICS))
    except Exception as err:  # pragma: no cover - depends on file corruption
        raise ValueError(f"failed to parse ULog file: {path}") from err

    lp = _get_dataset_or_none(ulog, "vehicle_local_position")
    if lp is None:
        # No backbone topic = no rows. Return an empty timeline rather than
        # raising; the caller can decide whether that's a hard error.
        return []

    lp_ts = list(lp.data["timestamp"])  # microseconds since boot
    if not lp_ts:
        return []

    lp_vx = _column(lp, "vx")
    lp_vy = _column(lp, "vy")
    lp_vz = _column(lp, "vz")
    lp_dist_bottom = _column(lp, "dist_bottom")

    # Attitude: q (and optional legacy angular_velocity)
    att = _get_dataset_or_none(ulog, "vehicle_attitude")
    att_ts = list(att.data["timestamp"]) if att is not None else []
    att_q0 = _column(att, "q[0]")
    att_q1 = _column(att, "q[1]")
    att_q2 = _column(att, "q[2]")
    att_q3 = _column(att, "q[3]")
    # Legacy PX4 versions embedded angular_velocity inside vehicle_attitude.
    att_av_z = _column(att, "angular_velocity[2]")

    # Newer PX4: yaw rate lives in vehicle_angular_velocity.xyz[2].
    avel = _get_dataset_or_none(ulog, "vehicle_angular_velocity")
    avel_ts = list(avel.data["timestamp"]) if avel is not None else []
    avel_z = _column(avel, "xyz[2]")

    # Last-resort fallback: raw gyro.
    sc = _get_dataset_or_none(ulog, "sensor_combined")
    sc_ts = list(sc.data["timestamp"]) if sc is not None else []
    sc_gyro_z = _column(sc, "gyro_rad[2]")

    # Geo: GPS and altitude.
    gps = _get_dataset_or_none(ulog, "vehicle_gps_position")
    gps_ts = list(gps.data["timestamp"]) if gps is not None else []
    gps_lat = _column(gps, "latitude_deg")
    gps_lon = _column(gps, "longitude_deg")
    gps_alt_msl = _column(gps, "altitude_msl_m")

    # vehicle_global_position is the alternative source on some PX4 builds.
    gp = _get_dataset_or_none(ulog, "vehicle_global_position")
    gp_ts = list(gp.data["timestamp"]) if gp is not None else []
    gp_lat = _column(gp, "lat")
    gp_lon = _column(gp, "lon")
    gp_alt = _column(gp, "alt")

    # Pitch / roll derived from attitude quaternion (last-value-carried-forward,
    # joined at vehicle_local_position cadence).
    # We compute pitch / roll lazily per row from att_q if available; this
    # keeps the parser self-contained (no scipy / numpy quaternion dep).

    def quat_to_roll_pitch(q):
        # q = [w, x, y, z]; returns (roll, pitch) in radians, NaN -> None.
        if q is None or any(v is None for v in q):
            return (None, None)
        try:
            w, x, y, z = (float(v) for v in q)
        except (TypeError, ValueError):
            return (None, None)
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
        if math.isnan(roll) or math.isnan(pitch):
            return (None, None)
        return (roll, pitch)

    t0 = float(lp_ts[0])
    rows: list[dict] = []

    for i, t_us in enumerate(lp_ts):
        t_rel_s = (float(t_us) - t0) / 1e6

        # altitude_agl_m: prefer vehicle_local_position.dist_bottom.
        agl: Optional[float] = None
        if lp_dist_bottom is not None:
            v = lp_dist_bottom[i]
            try:
                fv = float(v)
                if not math.isnan(fv):
                    agl = fv
            except (TypeError, ValueError):
                pass

        # velocity_world (NED): vx, vy, vz from vehicle_local_position.
        velocity_world: Optional[list[float]] = None
        if lp_vx is not None and lp_vy is not None and lp_vz is not None:
            try:
                vx = float(lp_vx[i])
                vy = float(lp_vy[i])
                vz = float(lp_vz[i])
                if not (math.isnan(vx) or math.isnan(vy) or math.isnan(vz)):
                    velocity_world = [vx, vy, vz]
            except (TypeError, ValueError):
                pass

        # attitude_quat [w, x, y, z] from vehicle_attitude.q.
        attitude_quat: Optional[list[float]] = None
        if att_q0 is not None and att_q1 is not None and att_q2 is not None and att_q3 is not None:
            qw = _last_le(att_ts, att_q0, t_us)
            qx = _last_le(att_ts, att_q1, t_us)
            qy = _last_le(att_ts, att_q2, t_us)
            qz = _last_le(att_ts, att_q3, t_us)
            if all(v is not None for v in (qw, qx, qy, qz)):
                attitude_quat = [float(qw), float(qx), float(qy), float(qz)]

        # yaw_rate_rad_s: prefer attitude.angular_velocity[2], then
        # vehicle_angular_velocity.xyz[2], then sensor_combined.gyro_rad[2].
        yaw_rate: Optional[float] = None
        if att_av_z is not None:
            v = _last_le(att_ts, att_av_z, t_us)
            if v is not None:
                yaw_rate = float(v)
        if yaw_rate is None and avel_z is not None:
            v = _last_le(avel_ts, avel_z, t_us)
            if v is not None:
                yaw_rate = float(v)
        if yaw_rate is None and sc_gyro_z is not None:
            v = _last_le(sc_ts, sc_gyro_z, t_us)
            if v is not None:
                yaw_rate = float(v)

        # _geo sidecar: lat, lon, alt_msl, pitch, roll.
        lat: Optional[float] = None
        lon: Optional[float] = None
        alt_msl: Optional[float] = None
        if gps_lat is not None:
            v = _last_le(gps_ts, gps_lat, t_us)
            lat = float(v) if v is not None else None
        if gps_lon is not None:
            v = _last_le(gps_ts, gps_lon, t_us)
            lon = float(v) if v is not None else None
        if gps_alt_msl is not None:
            v = _last_le(gps_ts, gps_alt_msl, t_us)
            alt_msl = float(v) if v is not None else None
        # vehicle_global_position fallback.
        if lat is None and gp_lat is not None:
            v = _last_le(gp_ts, gp_lat, t_us)
            lat = float(v) if v is not None else None
        if lon is None and gp_lon is not None:
            v = _last_le(gp_ts, gp_lon, t_us)
            lon = float(v) if v is not None else None
        if alt_msl is None and gp_alt is not None:
            v = _last_le(gp_ts, gp_alt, t_us)
            alt_msl = float(v) if v is not None else None

        roll, pitch = quat_to_roll_pitch(attitude_quat)

        row = {
            "timestamp": t_rel_s,
            "altitude_agl_m": agl,
            "yaw_rate_rad_s": yaw_rate,
            "velocity_world": velocity_world,
            "attitude_quat": attitude_quat,
            "_geo": {
                "lat": lat,
                "lon": lon,
                "alt_msl": alt_msl,
                "pitch": pitch,
                "roll": roll,
            },
        }
        rows.append(row)

    # Output is already sorted by construction (lp_ts is monotonic), but be
    # defensive: sort by timestamp in case of timestamp jitter or a future
    # pyulog change to data ordering.
    rows.sort(key=lambda r: r["timestamp"])
    return rows
