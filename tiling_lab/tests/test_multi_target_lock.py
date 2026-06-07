from hailo_tiling.types import Det, TargetState
from tiling_lab.harness.target_lock import MultiTargetLock


def _person(x, y, w=0.08, h=0.20, score=0.9):
    return Det(cls=0, score=score, x=x, y=y, w=w, h=h)


def _vehicle(x, y, w=0.15, h=0.10, score=0.85):
    return Det(cls=1, score=score, x=x, y=y, w=w, h=h)


def test_multi_target_lock_tracks_one_person_one_vehicle():
    lock = MultiTargetLock(target_classes={0, 1})
    dets = [_person(0.20, 0.40), _vehicle(0.70, 0.50)]
    states = None
    for _ in range(3):
        states = lock.step(dets)
    # Both should be TRACKING after 3 frames.
    assert any(s.cls == 0 and s.status == "TRACKING" for s in states)
    assert any(s.cls == 1 and s.status == "TRACKING" for s in states)
    # Keys are (cls, track_id) tuples.
    keys = [s.key for s in states]
    assert all(isinstance(k, tuple) and len(k) == 2 for k in keys)
    # Person and vehicle have different class in their key.
    cls_set = {s.key[0] for s in states}
    assert cls_set == {0, 1}


def test_multi_target_lock_selects_from_gt():
    lock = MultiTargetLock(target_classes={0, 1})
    dets = [_person(0.20, 0.40), _vehicle(0.70, 0.50)]
    # Feed for 2 frames without GT to establish tracks.
    lock.step(dets)
    lock.step(dets)
    # Feed with GT bbox matching the person.
    person_bbox = (0.20, 0.40, 0.08, 0.20)
    states = lock.step(dets, gt_bbox_norm=person_bbox, gt_cls=0)
    assert lock.selected_key is not None
    assert lock.selected_key[0] == 0   # class 0 = person
    selected = next(s for s in states if s.selected)
    assert selected.cls == 0


def test_multi_target_lock_loses_track_after_buffer():
    track_buffer = 5
    lock = MultiTargetLock(target_classes={0}, track_buffer=track_buffer)
    # Establish a track.
    for _ in range(2):
        lock.step([_person(0.40, 0.40)])
    # Confirm TRACKING.
    states = lock.step([_person(0.40, 0.40)])
    assert any(s.status == "TRACKING" for s in states)
    # Remove detections for track_buffer + 1 frames.
    last_states = None
    for _ in range(track_buffer + 1):
        last_states = lock.step([])
    # Target should be LOST (not returned from step, or returned as LOST).
    # step() returns only non-LOST states; so either empty OR contains LOST.
    alive = [s for s in last_states if s.status != "LOST"]
    assert len(alive) == 0


def test_multi_target_lock_seeds_selected_after_track_activation():
    """GT-seed fix regression: selected_key must be set eventually even when
    ByteTracker needs a frame or two to activate its first track."""
    lock = MultiTargetLock(target_classes={0, 1})
    p = Det(cls=0, score=0.9, x=0.40, y=0.40, w=0.08, h=0.20)
    gt_bbox = (0.40, 0.40, 0.08, 0.20)

    # Frame 0: ByteTracker may not activate yet — that's OK.
    lock.step([p], gt_bbox_norm=gt_bbox, gt_cls=0)

    # Frames 1-3: keep feeding same detection + GT every frame (mimics
    # the fixed run_multi which passes gt_bbox_norm each frame while
    # selected_key is None).
    for _ in range(3):
        if lock.selected_key is None:
            lock.step([p], gt_bbox_norm=gt_bbox, gt_cls=0)
        else:
            lock.step([p])

    # By frame 3 the track must be activated and selected.
    assert lock.selected_key is not None, (
        "selected_key should be set within 4 frames once detection is stable"
    )
    assert lock.selected_key[0] == 0, "selected target must be class 0 (person)"
