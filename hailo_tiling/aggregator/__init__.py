"""Detection aggregator — maps tile-local dets to source-frame and applies NMS + filters."""
from .nms import map_to_source, nms  # noqa: F401
