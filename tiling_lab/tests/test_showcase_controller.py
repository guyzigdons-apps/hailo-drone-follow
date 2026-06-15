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
