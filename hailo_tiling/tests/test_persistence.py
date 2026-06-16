from hailo_tiling.dynamic.persistence import DetectionPersistence


def _det(x, y, w=0.02, h=0.02, label="vehicle", conf=0.9):
    return {"label": label, "confidence": conf, "bbox": [x, y, w, h]}


def test_cell_of_maps_center_to_rowmajor_index():
    p = DetectionPersistence(dense_grid=(8, 6))
    assert p.cell_of(_det(0.0, 0.0)) == 0
    assert p.cell_of(_det(0.99, 0.99)) == 8 * 6 - 1
    assert p.cell_of(_det(0.13, 0.0)) == 1


def test_cell_of_clamps_out_of_bounds_coords():
    p = DetectionPersistence(dense_grid=(8, 6))
    assert p.cell_of(_det(-0.01, -0.005)) == 0
    assert p.cell_of(_det(0.99, 0.99, w=0.05, h=0.05)) == 8 * 6 - 1


def test_update_is_detection_driven_each_det_refreshes_its_own_cell():
    # Latency-robust: a detection is stored in WHATEVER cell it falls in — no
    # need to tell persistence which cells were scheduled this frame.
    p = DetectionPersistence(dense_grid=(8, 6))
    d_cell0 = _det(0.01, 0.01)     # cell 0
    d_cell1 = _det(0.13, 0.01)     # cell 1
    p.update([d_cell0, d_cell1])
    assert len(p.published()) == 2

    # A new det in cell 0 replaces cell 0; cell 1 (no new det) persists.
    d_cell0b = _det(0.02, 0.02)
    p.update([d_cell0b])
    pub_bboxes = [d["bbox"] for d in p.published()]
    assert d_cell0["bbox"] not in pub_bboxes   # old cell-0 det replaced
    assert d_cell0b["bbox"] in pub_bboxes       # new cell-0 det present
    assert d_cell1["bbox"] in pub_bboxes        # cell-1 det carried forward


def test_empty_update_does_not_clear_existing_cells():
    # Unlike the old schedule-gated design, an empty frame must NOT wipe carried
    # detections — they persist until refreshed or TTL expiry.
    p = DetectionPersistence(dense_grid=(8, 6))
    p.update([_det(0.01, 0.01)])
    p.update([])
    assert len(p.published()) == 1


def test_published_detections_carry_increasing_age_and_reset_on_refresh():
    p = DetectionPersistence(dense_grid=(7, 6))
    p.update([_det(0.50, 0.50)])                 # fresh: age 0
    assert p.published()[0]["age"] == 0
    p.tick()
    p.tick()
    assert p.published()[0]["age"] == 2          # aged, not re-detected
    p.update([_det(0.50, 0.50)])                 # re-detected: age resets
    assert p.published()[0]["age"] == 0


def test_ttl_expires_stale_detections():
    p = DetectionPersistence(dense_grid=(7, 6), ttl=3)
    p.update([_det(0.50, 0.50)])
    for _ in range(3):                            # age -> 3 (<= ttl): still alive
        p.tick()
    assert len(p.published()) == 1
    p.tick()                                       # age -> 4 (> ttl): expired
    assert p.published() == []


def test_no_ttl_means_never_expire():
    p = DetectionPersistence(dense_grid=(7, 6), ttl=None)
    p.update([_det(0.50, 0.50)])
    for _ in range(500):
        p.tick()
    assert len(p.published()) == 1


def test_update_does_not_mutate_caller_dicts():
    p = DetectionPersistence(dense_grid=(7, 6))
    d = _det(0.5, 0.5)
    p.update([d])
    p.tick()
    assert "age" not in d                          # caller's dict untouched
