"""Detection aggregator — maps tile-local dets to source-frame and applies NMS + filters."""
from .boundary_strip import BoundaryStripFilter  # noqa: F401
from .memory import DetectionMemory, NoOpMemory  # noqa: F401
from .nms import map_to_source, nms  # noqa: F401
