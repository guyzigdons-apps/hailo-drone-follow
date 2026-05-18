"""ABS-03 file-layout smoke test.

Gates two facts:
1. `robot_follow.robot_api.adapters.mavsdk_drone` imports cleanly (lands in 03-05 file-move).
2. `robot_follow.drone_api` no longer exists (lands in 03-09 cleanup).

xfail until 03-09 completes both halves.
"""

import importlib

import pytest

XFAIL_REASON_ADAPTER = "adapter module lands in 03-05-PLAN (file move)"
XFAIL_REASON_DRONE_API_DELETED = "drone_api/ deleted in 03-09-PLAN"


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ADAPTER)
def test_robot_api_adapters_mavsdk_drone_imports():
    """robot_api.adapters.mavsdk_drone must be importable after 03-05 file move."""
    mod = importlib.import_module("robot_follow.robot_api.adapters.mavsdk_drone")
    assert hasattr(mod, "MavsdkDroneAdapter")


@pytest.mark.xfail(strict=False, reason=XFAIL_REASON_DRONE_API_DELETED)
def test_drone_api_module_is_gone():
    """drone_api/ must be deleted after 03-09 cleanup."""
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module("robot_follow.drone_api")
