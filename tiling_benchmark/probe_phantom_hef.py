"""Frozen-legacy shim.

The HEF runtime pieces (``HefHandle``, ``decode_nms_output``, ``label_for``,
``Detection``, the phantom-probe CLI, …) moved to
``hailo_tiling.backends.hef_runtime``. This shim re-exports them so legacy
tiling_benchmark scripts (e.g. ``zoom_probe.py``) keep importing
``probe_phantom_hef`` unchanged. New / live code should import from
``hailo_tiling.backends.hef_runtime`` directly.
"""
from hailo_tiling.backends.hef_runtime import *  # noqa: F401,F403


if __name__ == "__main__":
    import sys

    from hailo_tiling.backends.hef_runtime import main

    sys.exit(main())
