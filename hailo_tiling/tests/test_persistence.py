from hailo_tiling.dynamic.persistence import DetectionPersistence


def _det(x, y, w=0.02, h=0.02, label="vehicle", conf=0.9):
    return {"label": label, "confidence": conf, "bbox": [x, y, w, h]}


def test_cell_of_maps_center_to_rowmajor_index():
    p = DetectionPersistence(dense_grid=(8, 6))
    # center near (0,0) -> cell 0; center near (1,1) -> last cell 8*6-1.
    assert p.cell_of(_det(0.0, 0.0)) == 0
    assert p.cell_of(_det(0.99, 0.99)) == 8 * 6 - 1
    # column 1 (i=1), row 0 (j=0) -> index 1.
    assert p.cell_of(_det(0.13, 0.0)) == 1


def test_update_replaces_only_run_cells_others_persist():
    p = DetectionPersistence(dense_grid=(8, 6))
    d_cell0 = _det(0.01, 0.01)     # cell 0
    d_cell1 = _det(0.13, 0.01)     # cell 1
    # Run cells {0,1}: both stored.
    p.update([0, 1], [d_cell0, d_cell1])
    assert len(p.published()) == 2
    # Run only cell {0} with a new det: cell 0 replaced, cell 1 persists.
    d_cell0b = _det(0.02, 0.02)
    p.update([0], [d_cell0b])
    pub = p.published()
    assert d_cell0 not in pub          # old cell-0 det gone
    assert d_cell0b in pub             # new cell-0 det present
    assert d_cell1 in pub              # cell-1 det persisted


def test_run_cell_with_no_detection_clears_that_cell():
    p = DetectionPersistence(dense_grid=(8, 6))
    p.update([0], [_det(0.01, 0.01)])
    assert len(p.published()) == 1
    # Re-run cell 0 with no detections falling in it -> cell emptied.
    p.update([0], [])
    assert p.published() == []


def test_detection_landing_outside_run_cells_is_ignored():
    p = DetectionPersistence(dense_grid=(8, 6))
    # Run cell {0} but the det belongs to cell 1 -> not stored.
    p.update([0], [_det(0.13, 0.01)])
    assert p.published() == []


def test_cell_of_clamps_out_of_bounds_coords():
    p = DetectionPersistence(dense_grid=(8, 6))
    # Slightly negative center clamps to cell 0.
    assert p.cell_of(_det(-0.01, -0.005)) == 0
    # Center beyond 1.0 (wide bbox near the edge) clamps to the last cell.
    assert p.cell_of(_det(0.99, 0.99, w=0.05, h=0.05)) == 8 * 6 - 1
