// gst-hailo-cache — unit tests for cache_keys helpers.
//
// Plan 5 Task 3. Six parameterised canonicalize_crop cases mirror the
// assertions in hailo_tiling/tests/test_cache_hashing.py
// (test_canonicalize_crop_* + the implicit "quantise=0 means identity"
// case the Python helper covers through its `q <= 1` guard).
//
// Plus a small batch of frame_id_from_buffer sanity tests for both
// the COUNTER and PTS branches, including the GST_CLOCK_TIME_NONE
// fallback (returns -1, caller decides).

#include "cache_keys.hpp"

#include <gst/gst.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <tuple>

namespace {

using hailo_cache::CanonicalCrop;
using hailo_cache::canonicalize_crop;
using hailo_cache::FrameIdSource;
using hailo_cache::frame_id_from_buffer;

struct CropCase {
    std::int32_t x, y, w, h;
    int q;
    CanonicalCrop expected;
};

class CanonicalizeCropP : public ::testing::TestWithParam<CropCase> {};

TEST_P(CanonicalizeCropP, MatchesPythonHelper) {
    const auto c = GetParam();
    auto got = canonicalize_crop(c.x, c.y, c.w, c.h, c.q);
    EXPECT_EQ(got.x, c.expected.x) << "x mismatch on q=" << c.q;
    EXPECT_EQ(got.y, c.expected.y) << "y mismatch on q=" << c.q;
    EXPECT_EQ(got.w, c.expected.w) << "w mismatch on q=" << c.q;
    EXPECT_EQ(got.h, c.expected.h) << "h mismatch on q=" << c.q;
}

// Six cases mirroring hailo_tiling/tests/test_cache_hashing.py:
//   1. no-quantise / q == 0  (Python `quantise=None`)  → identity
//   2. q == 1                                          → identity
//   3. q == 4 rounds DOWN to multiples of 4            (the canonical
//      "fuzzy mode" case the Python test pins)
//   4. q == 4 with values already multiples of 4       → identity-ish
//   5. larger q (16) — small values floor to zero
//   6. weird-but-legal: q == 4 with one component < q  → that comp = 0
INSTANTIATE_TEST_SUITE_P(
    Hashing, CanonicalizeCropP,
    ::testing::Values(
        // 1. q=0 is treated as identity (matches Python `quantise=None`).
        CropCase{123, 456, 789, 321, 0, {123, 456, 789, 321}},
        // 2. q=1 is identity.
        CropCase{7, 13, 11, 5, 1, {7, 13, 11, 5}},
        // 3. q=4 rounds DOWN — exact Python test_canonicalize_crop_quantise_4 case.
        CropCase{123, 457, 790, 322, 4, {120, 456, 788, 320}},
        // 4. q=4, already a multiple — identity.
        CropCase{120, 456, 788, 320, 4, {120, 456, 788, 320}},
        // 5. q=16, small w/h floor to 0.
        CropCase{32, 48, 15, 7, 16, {32, 48, 0, 0}},
        // 6. q=4 with x < q — floors to 0.
        CropCase{3, 100, 100, 100, 4, {0, 100, 100, 100}}
    )
);

// -- frame_id_from_buffer ---------------------------------------------------

TEST(FrameIdFromBuffer, CounterPostIncrementsStartingAtZero) {
    std::int64_t state = 0;
    // We don't need a real GstBuffer for the COUNTER branch; the
    // function never dereferences it in that mode.
    EXPECT_EQ(frame_id_from_buffer(nullptr, FrameIdSource::COUNTER, state), 0);
    EXPECT_EQ(state, 1);
    EXPECT_EQ(frame_id_from_buffer(nullptr, FrameIdSource::COUNTER, state), 1);
    EXPECT_EQ(state, 2);
    EXPECT_EQ(frame_id_from_buffer(nullptr, FrameIdSource::COUNTER, state), 2);
    EXPECT_EQ(state, 3);
}

TEST(FrameIdFromBuffer, CounterRespectsCallerProvidedSeed) {
    std::int64_t state = 100;
    EXPECT_EQ(frame_id_from_buffer(nullptr, FrameIdSource::COUNTER, state), 100);
    EXPECT_EQ(state, 101);
}

TEST(FrameIdFromBuffer, PtsReturnsRawNanoseconds) {
    // GST_NSECOND == 1 (GstClockTime is already in nanoseconds). The
    // spec's rule (`pts / GST_NSECOND`) therefore returns the raw ns
    // count — that's exactly what we want as the cache key, because
    // the writer and reader both apply the SAME rule so they agree
    // on a single integer per frame (no risk of float-seconds drift).
    GstBuffer* buf = gst_buffer_new();
    std::int64_t unused = 0;

    GST_BUFFER_PTS(buf) = static_cast<GstClockTime>(0);
    EXPECT_EQ(frame_id_from_buffer(buf, FrameIdSource::PTS, unused), 0);
    EXPECT_EQ(unused, 0)
        << "PTS mode must not touch the counter state";

    GST_BUFFER_PTS(buf) = static_cast<GstClockTime>(1500000000ULL);
    EXPECT_EQ(frame_id_from_buffer(buf, FrameIdSource::PTS, unused),
              1500000000LL);

    GST_BUFFER_PTS(buf) = static_cast<GstClockTime>(2999999999ULL);
    EXPECT_EQ(frame_id_from_buffer(buf, FrameIdSource::PTS, unused),
              2999999999LL);
    EXPECT_EQ(unused, 0);

    gst_buffer_unref(buf);
}

TEST(FrameIdFromBuffer, PtsReturnsMinusOneWhenInvalid) {
    GstBuffer* buf = gst_buffer_new();
    GST_BUFFER_PTS(buf) = GST_CLOCK_TIME_NONE;
    std::int64_t unused = 0;
    EXPECT_EQ(frame_id_from_buffer(buf, FrameIdSource::PTS, unused), -1);
    gst_buffer_unref(buf);
}

TEST(FrameIdFromBuffer, PtsReturnsMinusOneOnNullBuffer) {
    std::int64_t unused = 0;
    EXPECT_EQ(frame_id_from_buffer(nullptr, FrameIdSource::PTS, unused), -1);
}

// gst_buffer_new requires gst_init to have run. We do it once via a
// custom main that defers to the gtest_main behaviour, but since this
// test binary links gtest_main (per tests/meson.build), we instead
// gst_init lazily via an environment fixture.
class GstEnv : public ::testing::Environment {
public:
    void SetUp() override { gst_init(nullptr, nullptr); }
};

// Register the env.
static ::testing::Environment* const kGstEnv =
    ::testing::AddGlobalTestEnvironment(new GstEnv);

}  // namespace
