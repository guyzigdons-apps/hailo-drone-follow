"""Phase-3-only snapshot fixture for the drone-controller rewrite.

Captures the OLD ``compute_velocity_command`` output for ~100 representative
``Detection`` inputs at HEAD = 3470796 (pre-rewrite). Phase 3's verifier asserts
that the new ``controller.compute(detection, DRONE_CAPS, config)`` plus the
``MavsdkDroneAdapter`` internal pipeline (altitude_p + retreat_from_tilt +
smoothing) produces equivalent ``VelocityBodyYawspeed`` outputs.

Lifecycle: this file is created in 03-01 (Wave 0) with placeholder
``expected_velocity_command=(0.0, 0.0, 0.0)`` values. Wave 5 (plan 03-07) runs a
one-shot capture script against the OLD ``compute_velocity_command`` and
overwrites the placeholders with the captured tuple. After the Phase 3
verifier passes, the file may be archived.

Why a Python module (not JSON): the fixtures benefit from type hints, default
arguments on the dataclass, and module-level constants such as
``DRONE_CAPS_STUB``. See ``.planning/phases/03-abstraction/03-RESEARCH.md``
§ "Snapshot fixture design" for the design discussion.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from robot_follow.follow_api.types import Detection


# Stand-in for the real Capabilities frozenset (lands in 03-03). The snapshot
# test references ``DRONE_CAPS_STUB`` only as a sentinel; once 03-06 lands the
# real ``DRONE_CAPS``, callers can switch the import and this constant can be
# deleted alongside the xfail strip in 03-07.
DRONE_CAPS_STUB = frozenset({"FORWARD", "YAW", "ALTITUDE"})


@dataclass(frozen=True)
class BaselineCase:
    """A single snapshot entry.

    ``expected_velocity_command`` is ``(forward_m_s, down_m_s, yawspeed_deg_s)``.
    Defaults to ``(0.0, 0.0, 0.0)`` in 03-01; populated in 03-07 by a one-shot
    capture script that runs the OLD ``compute_velocity_command`` against each
    case and records the tuple.
    """

    name: str
    config_overrides: dict
    detection: Optional[Detection]
    last_detection: Optional[Detection] = None
    expected_velocity_command: tuple = (0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Helper builders
# ---------------------------------------------------------------------------


def _det(cx: float, cy: float, bh: float, conf: float = 0.9) -> Detection:
    """Concise Detection constructor for the case table."""
    return Detection(
        label="person",
        confidence=conf,
        center_x=cx,
        center_y=cy,
        bbox_height=bh,
        timestamp=0.0,
    )


_FULL_FOLLOW: dict = {"yaw_only": False}
_YAW_ONLY: dict = {"yaw_only": True}


# ---------------------------------------------------------------------------
# Case definitions — 15 categories totalling 100 entries
# ---------------------------------------------------------------------------
# Each category mirrors the table in .planning/phases/03-abstraction/03-RESEARCH.md
# § "Snapshot fixture design" § "Category breakdown".

CASES: list[BaselineCase] = []

# Category 1: target-centered, at-target-bbox (5 cases) -----------------------
# Detection sitting near the centre with bbox_height ~= target — expect all
# zeros in the dominant axes once expected values are captured.
CASES += [
    BaselineCase(
        name="target-centered-at-target-bbox-exact",
        config_overrides=_FULL_FOLLOW,
        detection=_det(0.5, 0.5, 0.25),
    ),
    BaselineCase(
        name="target-centered-at-target-bbox-slight-left",
        config_overrides=_FULL_FOLLOW,
        detection=_det(0.498, 0.5, 0.25),
    ),
    BaselineCase(
        name="target-centered-at-target-bbox-slight-right",
        config_overrides=_FULL_FOLLOW,
        detection=_det(0.502, 0.5, 0.25),
    ),
    BaselineCase(
        name="target-centered-at-target-bbox-slight-up",
        config_overrides=_FULL_FOLLOW,
        detection=_det(0.5, 0.48, 0.25),
    ),
    BaselineCase(
        name="target-centered-at-target-bbox-slight-down",
        config_overrides=_FULL_FOLLOW,
        detection=_det(0.5, 0.52, 0.25),
    ),
]

# Category 2: target-offset-left (8 cases) ------------------------------------
# Sweep cx left of centre per RESEARCH.
for cx in (0.10, 0.20, 0.30, 0.40, 0.45, 0.48, 0.49, 0.50):
    CASES.append(
        BaselineCase(
            name=f"target-offset-left-cx-{cx:.3f}",
            config_overrides=_FULL_FOLLOW,
            detection=_det(cx, 0.5, 0.25),
        )
    )

# Category 3: target-offset-right (8 cases) -----------------------------------
for cx in (0.51, 0.52, 0.55, 0.60, 0.70, 0.80, 0.90, 1.00):
    CASES.append(
        BaselineCase(
            name=f"target-offset-right-cx-{cx:.3f}",
            config_overrides=_FULL_FOLLOW,
            detection=_det(cx, 0.5, 0.25),
        )
    )

# Category 4: bbox-too-small (approach) (10 cases) ----------------------------
for bh in (0.05, 0.08, 0.10, 0.12, 0.15, 0.18, 0.20, 0.22, 0.23, 0.24):
    CASES.append(
        BaselineCase(
            name=f"bbox-too-small-approach-bh-{bh:.3f}",
            config_overrides=_FULL_FOLLOW,
            detection=_det(0.5, 0.5, bh),
        )
    )

# Category 5: bbox-too-large (retreat) (8 cases) ------------------------------
for bh in (0.26, 0.30, 0.35, 0.40, 0.50, 0.60, 0.70, 0.79):
    CASES.append(
        BaselineCase(
            name=f"bbox-too-large-retreat-bh-{bh:.3f}",
            config_overrides=_FULL_FOLLOW,
            detection=_det(0.5, 0.5, bh),
        )
    )

# Category 6: emergency safety (bbox > 0.8) (4 cases) -------------------------
for bh in (0.81, 0.85, 0.90, 0.95):
    CASES.append(
        BaselineCase(
            name=f"emergency-safety-bh-{bh:.3f}",
            config_overrides=_FULL_FOLLOW,
            detection=_det(0.5, 0.5, bh),
        )
    )

# Category 7: bbox-at-top-margin (8 cases) ------------------------------------
# Small cy means top of frame; bh=0.20 keeps us inside the at-target band so the
# only forcing input is the top-edge fade/push.
for cy in (0.04, 0.08, 0.10, 0.12, 0.15, 0.18, 0.20, 0.22):
    CASES.append(
        BaselineCase(
            name=f"bbox-at-top-margin-cy-{cy:.3f}",
            config_overrides=_FULL_FOLLOW,
            detection=_det(0.5, cy, 0.20),
        )
    )

# Category 8: bbox-at-bottom-margin (12 cases) --------------------------------
for cy in (0.65, 0.70, 0.75, 0.80, 0.85, 0.88, 0.90, 0.92, 0.93, 0.95, 0.97, 0.99):
    CASES.append(
        BaselineCase(
            name=f"bbox-at-bottom-margin-cy-{cy:.3f}",
            config_overrides=_FULL_FOLLOW,
            detection=_det(0.5, cy, 0.20),
        )
    )

# Category 9: bbox-in-fade-zone (8 cases) -------------------------------------
# Mixed cy/bh choices so bbox_bottom = cy + bh/2 lands in [0.50, 0.75], the
# bottom fade zone for the default bottom_margin_safety=0.25.
_FADE_ZONE: list[tuple[float, float]] = [
    (0.60, 0.10),  # bottom = 0.65
    (0.55, 0.15),  # bottom = 0.625
    (0.50, 0.20),  # bottom = 0.60
    (0.62, 0.10),  # bottom = 0.67
    (0.58, 0.14),  # bottom = 0.65
    (0.65, 0.10),  # bottom = 0.70
    (0.55, 0.30),  # bottom = 0.70, but bh>target → retreat overlaid on fade
    (0.50, 0.40),  # bottom = 0.70, retreat dominant
]
for cy, bh in _FADE_ZONE:
    CASES.append(
        BaselineCase(
            name=f"bbox-in-fade-zone-cy-{cy:.3f}-bh-{bh:.3f}",
            config_overrides=_FULL_FOLLOW,
            detection=_det(0.5, cy, bh),
        )
    )

# Category 10: yaw_only=True config (5 cases) ---------------------------------
# Confirm forward axis is always zero regardless of bbox geometry.
_YAW_ONLY_GEOMS: list[tuple[float, float, float]] = [
    (0.5, 0.5, 0.25),
    (0.3, 0.5, 0.10),
    (0.7, 0.5, 0.50),
    (0.5, 0.9, 0.20),
    (0.5, 0.1, 0.20),
]
for cx, cy, bh in _YAW_ONLY_GEOMS:
    CASES.append(
        BaselineCase(
            name=f"yaw-only-cx-{cx:.2f}-cy-{cy:.2f}-bh-{bh:.2f}",
            config_overrides=_YAW_ONLY,
            detection=_det(cx, cy, bh),
        )
    )

# Category 11: dead-zone-holds-zero (4 cases) ---------------------------------
# cx clustered tightly around 0.5 so the yaw error stays inside dead_zone_deg.
for cx in (0.495, 0.500, 0.505, 0.510):
    CASES.append(
        BaselineCase(
            name=f"dead-zone-holds-zero-cx-{cx:.3f}",
            config_overrides=_FULL_FOLLOW,
            detection=_det(cx, 0.5, 0.25),
        )
    )

# Category 12: forward-velocity-deadband (3 cases) ----------------------------
# bbox-error factor placing |natural forward| just below the deadband cutoff.
_DEADBAND_BH: tuple[float, ...] = (0.248, 0.250, 0.252)
for bh in _DEADBAND_BH:
    CASES.append(
        BaselineCase(
            name=f"forward-velocity-deadband-bh-{bh:.3f}",
            config_overrides=_FULL_FOLLOW,
            detection=_det(0.5, 0.5, bh),
        )
    )

# Category 13: asymmetric-retreat (4 cases) -----------------------------------
# Retreat is ~3x more aggressive than approach for the same |factor|. We pair
# matching forward/backward bh offsets and document the intent in the name.
_ASYMMETRIC: list[tuple[str, float]] = [
    ("approach-factor-0.20", 0.20),   # bh below target by 0.05
    ("retreat-factor-0.20", 0.30),    # bh above target by 0.05 (mirror)
    ("approach-factor-0.40", 0.15),   # below target by 0.10
    ("retreat-factor-0.40", 0.35),    # above target by 0.10
]
for label, bh in _ASYMMETRIC:
    CASES.append(
        BaselineCase(
            name=f"asymmetric-retreat-{label}",
            config_overrides=_FULL_FOLLOW,
            detection=_det(0.5, 0.5, bh),
        )
    )

# Category 14: search-direction-from-last-detection (8 cases) -----------------
# detection=None triggers search; last_detection.center_x drives the spin sign.
for cx in (0.10, 0.20, 0.30, 0.40, 0.60, 0.70, 0.80, 0.90):
    CASES.append(
        BaselineCase(
            name=f"search-direction-from-last-cx-{cx:.2f}",
            config_overrides=_FULL_FOLLOW,
            detection=None,
            last_detection=_det(cx, 0.5, 0.25),
        )
    )

# Category 15: hold-velocity (search-enter delay not yet reached) (5 cases) ---
# detection=None AND the orchestrator hasn't promoted to search yet
# (search_active=False). The snapshot expects the controller to return
# hold_velocity (or zero). We encode the "not yet searching" state via the
# absence of last_detection; the snapshot driver in 03-07 will set
# search_active=False explicitly when calling compute_velocity_command.
_HOLD_VELOCITY_NAMES: tuple[str, ...] = (
    "hold-velocity-no-history",
    "hold-velocity-recent-target-left",
    "hold-velocity-recent-target-right",
    "hold-velocity-recent-target-top",
    "hold-velocity-recent-target-bottom",
)
_HOLD_LASTS: tuple[Optional[Detection], ...] = (
    None,
    _det(0.30, 0.50, 0.25),
    _det(0.70, 0.50, 0.25),
    _det(0.50, 0.20, 0.25),
    _det(0.50, 0.80, 0.25),
)
for name, last in zip(_HOLD_VELOCITY_NAMES, _HOLD_LASTS):
    CASES.append(
        BaselineCase(
            name=name,
            config_overrides=_FULL_FOLLOW,
            detection=None,
            last_detection=last,
        )
    )


# End-of-file sanity: an accidental truncation fails at import time.
assert len(CASES) >= 95, f"need >=95 cases, got {len(CASES)}"
