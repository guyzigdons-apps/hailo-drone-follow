from tiling_lab.viewer.overlay_viewer import color_for_label


def test_target_has_distinct_color():
    target = color_for_label("target")
    vehicle = color_for_label("vehicle")
    person = color_for_label("person")
    assert target != vehicle and target != person
    # 3-tuple in 0..255
    assert len(target) == 3 and all(0 <= c <= 255 for c in target)
