"""robot_api — actuator-boundary layer for robot-generic follow.

Depends on follow_api (which is the pure-leaf domain core).
Contains the Robot protocol, the run_robot_loop orchestrator, and
per-robot adapters (drone, future rover).

Layer rule: robot_api may import from follow_api; follow_api may
NEVER import from robot_api. See Phase 3 R1 (CONTEXT 2026-05-17).
"""

from robot_follow.robot_api.robot import Robot

__all__ = ["Robot"]
