from hailo_tiling.dynamic.acquisition import pyramid_crops


def test_pyramid_has_increasing_zoom_centred_on_click():
    crops = pyramid_crops(0.5, 0.5, 3840, 2160, widths=(640, 320))
    assert len(crops) == 2
    assert abs(crops[0].scale - 1.0) < 1e-6        # native level
    assert abs(crops[1].scale - 2.0) < 1e-6        # 2x zoom level
    for c in crops:                                 # all centred on the click
        assert abs((c.x + c.w / 2) / 3840 - 0.5) < 0.01
        assert abs((c.y + c.h / 2) / 2160 - 0.5) < 0.01
    assert all(c.mode == "s" for c in crops)


def test_pyramid_skips_widths_larger_than_source():
    crops = pyramid_crops(0.5, 0.5, 600, 480, widths=(640, 320))
    assert [c.w for c in crops] == [320]            # 640 > src_w → skipped
