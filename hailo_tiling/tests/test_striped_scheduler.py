from hailo_tiling.dynamic.striped import StripedDenseScheduler


def test_stripes_partition_full_grid_with_no_overlap():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0)
    # 60/2 = 30 frames per full refresh.
    assert s.K == 30
    # 8*6 = 48 dense crops built once.
    assert len(s._dense) == 48
    # Union of all stripe index-sets over one cycle == every dense cell, once.
    seen = []
    for f in range(s.K):
        seen.extend(s.stripe_indices(f))
    assert sorted(seen) == list(range(48))


def test_stripes_are_interleaved_and_balanced():
    s = StripedDenseScheduler(3840, 2160, dense_grid=(8, 6),
                              fps=60.0, cadence_fps=2.0)
    sizes = [len(s.stripe_indices(f)) for f in range(s.K)]
    # 48 tiles over 30 stripes => sizes differ by at most 1.
    assert max(sizes) - min(sizes) <= 1
    # Interleaved: stripe 0 starts at cell 0, stripe 1 at cell 1, etc.
    assert s.stripe_indices(0)[0] == 0
    assert s.stripe_indices(1)[0] == 1
