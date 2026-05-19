"""drone_api — DEPRECATED backward-compat shim.

The MAVSDK drone adapter has moved to
``robot_follow.robot_api.adapters.mavsdk_drone`` (Phase 3 plan 03-05).
Phase 3 plan 03-07 deleted VelocityCommandAPI / run_live_drone /
live_control_loop; only ``add_drone_args`` and ``_reap_mavsdk_server``
remain re-exported here for legacy callers.

This entire package is DELETED in Phase 3 plan 03-09.
Do NOT add new imports from here — migrate to::

    from robot_follow.robot_api.adapters.mavsdk_drone import ...
"""

from robot_follow.robot_api.adapters.mavsdk_drone import (
    add_drone_args,
    _reap_mavsdk_server,
)

__all__ = [
    "add_drone_args",
    "_reap_mavsdk_server",
]
