import numpy as np


def _v(*xs):
    a = np.array(xs, dtype=np.float32)
    return a / np.linalg.norm(a)


def test_gallery_add_bands_and_match():
    from dynamic_tiling.reid_gallery import ReidGallery
    g = ReidGallery(size=3, drift_threshold=0.6, duplicate_threshold=0.9,
                    refresh_every=2, min_gallery_for_drift_check=1)
    assert g.add(_v(1, 0, 0)) == "added"            # seed
    assert g.add(_v(0.8, 0.6, 0)) == "added"        # sim ~0.8: enrichment band
    assert g.add(_v(1, 0.01, 0)) == "duplicate"     # sim ~1.0: dup band, skipped
    assert g.add(_v(1, 0, 0.01)) == "refreshed"     # 2nd consecutive dup -> refresh
    assert g.add(_v(0, 0, 1)) == "drift"            # sim ~0 vs gallery: NOT stored
    assert g.match(_v(1, 0, 0)) > 0.9
    assert g.match(_v(0, 1, 0)) < 0.9


def test_fifo_replacement_at_capacity():
    from dynamic_tiling.reid_gallery import ReidGallery
    g = ReidGallery(size=2, drift_threshold=0.0, duplicate_threshold=0.99,
                    min_gallery_for_drift_check=99)
    g.add(_v(1, 0, 0)); g.add(_v(0.7, 0.7, 0)); g.add(_v(0, 1, 0))
    assert len(g) == 2                              # oldest evicted


def test_ema_anchor():
    from dynamic_tiling.reid_gallery import ReidGallery
    g = ReidGallery(size=4, ema_alpha=0.5, min_gallery_for_drift_check=99,
                    drift_threshold=0.0, duplicate_threshold=0.999)
    g.add(_v(1, 0, 0)); g.add(_v(0, 1, 0))
    m = g.match(_v(1, 1, 0))
    assert m > 0.9                                   # EMA anchor sits between the two
