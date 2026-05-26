"""ABS-03 file-layout smoke test.

Gates two facts:
1. `robot_follow.robot_api.adapters.mavsdk_drone` imports cleanly (passes
   after 03-05 file-move + 03-06 MavsdkDroneAdapter class).
2. `robot_follow.drone_api` no longer exists (passes after 03-09 cleanup
   deleted the legacy shim).
"""

import importlib

import pytest


def test_robot_api_adapters_mavsdk_drone_imports():
    """robot_api.adapters.mavsdk_drone must be importable + expose MavsdkDroneAdapter."""
    mod = importlib.import_module("robot_follow.robot_api.adapters.mavsdk_drone")
    assert hasattr(mod, "MavsdkDroneAdapter")


def test_drone_api_module_is_gone():
    """drone_api/ must be deleted after 03-09 cleanup."""
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module("robot_follow.drone_api")
