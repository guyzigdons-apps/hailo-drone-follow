"""Ablation bench package (Plan 6 Track B).

Chip-free ablation harness: sweep a config matrix of tiling strategies, replay
each row's per-frame crops against a warmed source-pixel-keyed cache, aggregate
+ score into an ablation table.
"""
from .config import BenchConfig, default_matrix

__all__ = ["BenchConfig", "default_matrix"]
