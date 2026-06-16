from hailo_tiling.types import Det
from tiling_lab.live.controller import DynamicTilingController


def _det_dict(x, y, w=0.04, h=0.03, label="vehicle", conf=0.9):
    return {"label": label, "confidence": conf, "bbox": [x, y, w, h]}


def _target_det(x, y, w=0.04, h=0.03, conf=0.9):
    # cls value is irrelevant to TargetLock (it locks the largest).
    return Det(cls=2, score=conf, x=x, y=y, w=w, h=h)


def test_step_showcase_returns_tiles_string_and_records():
    ctrl = DynamicTilingController(3840, 2160, fps=60.0, budget_inf_per_s=300.0,
                                   striped=True, persist=True,
                                   dense_grid=(8, 6), cadence_fps=2.0)
    tgt = _target_det(0.5, 0.5)
    others = [_det_dict(0.5, 0.5, label="vehicle"),   # the target itself
              _det_dict(0.1, 0.1, label="person")]    # background context
    tiles, records = ctrl.step_showcase([tgt], others)
    assert isinstance(tiles, str) and tiles          # non-empty tiles-static
    assert isinstance(records, list)
    # Background person should appear; lock takes a few frames, so just assert
    # the persisted union is exposed.
    labels = {r["label"] for r in records}
    assert "person" in labels


def test_step_showcase_flat_tile_count_no_spike():
    ctrl = DynamicTilingController(3840, 2160, fps=60.0, budget_inf_per_s=300.0,
                                   striped=True, persist=True,
                                   dense_grid=(8, 6), cadence_fps=2.0)
    tgt = _target_det(0.5, 0.5)
    counts = []
    for _ in range(30):
        tiles, _records = ctrl.step_showcase([tgt], [_det_dict(0.5, 0.5)])
        counts.append(tiles.count(";") + 1 if tiles else 0)
    assert max(counts) - min(counts) <= 1            # no discovery spike


def test_step_showcase_dedups_persisted_target_box():
    ctrl = DynamicTilingController(3840, 2160, fps=60.0, budget_inf_per_s=300.0,
                                   striped=True, persist=True,
                                   dense_grid=(8, 6), cadence_fps=2.0)
    tgt = _target_det(0.5, 0.5)
    # A persisted vehicle that sits right on top of the locked target, plus a
    # far-away background person.
    overlap = _det_dict(0.5, 0.5, label="vehicle")
    bg = _det_dict(0.05, 0.9, label="person")
    records = []
    # Run a full dense cycle + a few frames so (a) the lock reaches TRACKING and
    # (b) the target's dense cell has been refreshed at least once.
    for _ in range(35):
        _tiles, records = ctrl.step_showcase([tgt], [overlap, bg])
    assert ctrl.status == "TRACKING"
    # Exactly one record sits at the target location: the live "target" record.
    near_target = [r for r in records
                   if abs(r["bbox"][0] - 0.5) < 0.02 and abs(r["bbox"][1] - 0.5) < 0.02]
    assert len(near_target) == 1
    assert near_target[0]["label"] == "target"


def test_seed_mode_draws_real_bbox_not_seed_box():
    ctrl = DynamicTilingController(3840, 2160, fps=60.0, budget_inf_per_s=300.0,
                                   striped=True, persist=True, dense_grid=(7, 6),
                                   grid_overlap=0.15, acquire_mode="seed")
    # Seed a POSITION; the real car is a slightly different box nearby.
    ctrl.seed((0.50, 0.30, 0.04, 0.04))           # seed centre (0.52, 0.32)
    real = _target_det(0.521, 0.318, w=0.05, h=0.035)

    # Frame with NO target detection: a tile must still be placed (ROI at seed)
    # but NO "target" record is emitted (no made-up seed box drawn).
    tiles, records = ctrl.step_showcase([], [])
    assert tiles
    assert not [r for r in records if r["label"] == "target"]

    # Frame WITH the real detection: exactly one "target" record, the REAL bbox.
    tiles, records = ctrl.step_showcase([real], [_det_dict(0.521, 0.318, 0.05, 0.035)])
    tgt = [r for r in records if r["label"] == "target"]
    assert len(tgt) == 1
    assert tgt[0]["bbox"] == [0.521, 0.318, 0.05, 0.035]


def test_seed_mode_nms_merges_stale_friend_into_target():
    """A stale persisted box that partially overlaps the live target (IoU ~0.37,
    below the old 0.5 dedup threshold but above the 0.3 NMS threshold) must be
    absorbed by nms_merge so only the 'target' record survives near that location.
    """
    ctrl = DynamicTilingController(3840, 2160, fps=60.0, budget_inf_per_s=600.0,
                                   striped=True, persist=True, dense_grid=(7, 6),
                                   grid_overlap=0.15, acquire_mode="seed")
    ctrl.seed((0.50, 0.50, 0.04, 0.04))
    real = _target_det(0.521, 0.518, w=0.05, h=0.035)
    # real_dict: the target's vehicle detection (same box)
    real_dict = {"label": "vehicle", "confidence": 0.9,
                 "bbox": [0.521, 0.518, 0.05, 0.035]}
    # stale_friend: a slightly shifted copy of the target (IoU ~0.37 with target).
    # This escapes the old _SOT_DEDUP_IOU=0.5 guard but is caught by NMS at 0.3.
    stale_friend = {"label": "vehicle", "confidence": 0.85,
                    "bbox": [0.53, 0.53, 0.05, 0.035]}
    far = _det_dict(0.05, 0.90, 0.06, 0.05, label="vehicle")
    records = []
    for _ in range(30):
        _tiles, records = ctrl.step_showcase([real], [real_dict, stale_friend, far])
    near = [r for r in records
            if abs(r["bbox"][0] - 0.521) < 0.05 and abs(r["bbox"][1] - 0.518) < 0.05]
    assert len(near) == 1 and near[0]["label"] == "target"   # friend merged away
    assert any(abs(r["bbox"][0] - 0.05) < 0.03 for r in records)  # far car kept


def test_seed_mode_does_not_switch_to_a_different_car():
    ctrl = DynamicTilingController(3840, 2160, fps=60.0, budget_inf_per_s=300.0,
                                   striped=True, persist=True, dense_grid=(7, 6),
                                   grid_overlap=0.15, acquire_mode="seed")
    ctrl.seed((0.50, 0.30, 0.04, 0.04))           # centre (0.52, 0.32)
    far = _target_det(0.10, 0.85, w=0.06, h=0.05)  # a different, bigger car
    _tiles, records = ctrl.step_showcase([far], [_det_dict(0.10, 0.85, 0.06, 0.05)])
    # The lock must NOT adopt the far car as the target.
    assert not [r for r in records if r["label"] == "target"]
