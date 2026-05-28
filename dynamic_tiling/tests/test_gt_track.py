from dynamic_tiling.gt_track import build_target_trajectory


def _doc():
    frames = []
    for f in range(5):
        frames.append({"frame": f, "detections": [
            {"label": "person", "confidence": 0.9,
             "bbox": [0.40 + 0.02 * f, 0.40, 0.10, 0.25]},   # A (large, moving)
            {"label": "person", "confidence": 0.9,
             "bbox": [0.10, 0.10, 0.05, 0.12]},              # B (small, static)
        ]})
    return {"frames": frames}


def test_builds_trajectory_for_largest_person():
    traj = build_target_trajectory(_doc(), label="person", anchor="largest")
    assert set(traj.keys()) == {0, 1, 2, 3, 4}
    assert abs(traj[0][0] - 0.40) < 1e-6
    assert abs(traj[4][0] - 0.48) < 1e-6  # moved right


def test_handles_missing_frame_gap():
    doc = _doc()
    doc["frames"][2]["detections"] = []  # gap on frame 2
    traj = build_target_trajectory(doc, label="person", anchor="largest")
    assert 2 not in traj            # no GT box on frame 2
    assert traj[3][0] > traj[1][0]  # re-associates A after the gap
