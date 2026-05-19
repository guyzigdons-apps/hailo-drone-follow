"""drone_api — DEPRECATED backward-compat shim.

The MAVSDK drone adapter has moved to
``robot_follow.robot_api.adapters.mavsdk_drone`` (Phase 3 plan 03-05).
This package re-exports the same symbols so legacy importers
(anything still doing ``from robot_follow.drone_api import ...``)
continue to work during the migration window.

This entire package is DELETED in Phase 3 plan 03-09.
Do NOT add new imports from here — migrate to::

    from robot_follow.robot_api.adapters.mavsdk_drone import ...
"""

from robot_follow.robot_api.adapters.mavsdk_drone import (
    VelocityCommandAPI,
    run_live_drone,
    add_drone_args,
    _reap_mavsdk_server,
)

__all__ = [
    "VelocityCommandAPI",
    "run_live_drone",
    "add_drone_args",
    "_reap_mavsdk_server",
]
