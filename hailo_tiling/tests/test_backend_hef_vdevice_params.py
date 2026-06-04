"""make_shared_vdevice_params — shared-group ROUND_ROBIN VDevice params.

Cold-cache runs construct both the detection HefBackend and the reid_analysis
ReID extractor. If the detection backend grabs an exclusive VDevice, the ReID
extractor's VDevice.create fails with HAILO_OUT_OF_PHYSICAL_DEVICES(74). The
backend must join the same shared VDevice group (group_id "SHARED",
ROUND_ROBIN) the ReID extractor uses so the HailoRT scheduler multiplexes both
HEFs onto one physical device.

Building the params is chipless (VDevice.create_params() needs no device), so
this runs without a chip. It is skipped only if hailo_platform itself cannot be
imported on the host (matching the suite's import-availability skip pattern).
"""
from __future__ import annotations

import pytest

# Skip only when the HailoRT Python bindings are absent; params construction
# itself touches no physical device.
pytest.importorskip("hailo_platform", reason="hailo_platform not installed on this host")

from hailo_tiling.backends.hef import (  # noqa: E402
    SHARED_VDEVICE_GROUP_ID,
    make_shared_vdevice_params,
)


def test_shared_group_id_matches_reid_extractor_constant():
    # reid_analysis/reid_embedding_extractor.py uses
    # hailo_apps...defines.SHARED_VDEVICE_GROUP_ID == "SHARED". hailo_tiling
    # duplicates the literal (must not import reid_analysis/hailo_apps).
    assert SHARED_VDEVICE_GROUP_ID == "SHARED"


def test_make_shared_vdevice_params_sets_round_robin_and_group():
    from hailo_platform import HailoSchedulingAlgorithm

    params = make_shared_vdevice_params()
    assert params.scheduling_algorithm == HailoSchedulingAlgorithm.ROUND_ROBIN
    assert params.group_id == SHARED_VDEVICE_GROUP_ID
