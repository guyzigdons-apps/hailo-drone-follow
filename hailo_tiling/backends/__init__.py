"""Inference backends — the seam between scheduler policy and execution mechanism."""
from .backend import InferenceBackend, MockBackend  # noqa: F401
from .hef import HefBackend  # noqa: F401
