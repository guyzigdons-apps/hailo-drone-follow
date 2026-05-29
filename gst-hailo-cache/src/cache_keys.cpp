// gst-hailo-cache — cache key helpers (impl).
//
// Plan 5, Task 3. See cache_keys.hpp for the contract.
//
// Quantisation rule MUST match hailo_tiling/cache/hashing.py exactly:
//   q <= 1  →  identity
//   q  > 1  →  (coord // q) * q   (Python floor division)
//
// C++ integer division truncates toward zero, which DIFFERS from Python
// floor division for negative numerators. The plan explicitly leaves
// negative inputs out of scope, but we still implement the floor rule
// so writer and reader stay aligned if a negative ever slips through.

#include "cache_keys.hpp"

#include <gst/gst.h>

namespace hailo_cache {

namespace {

// Python-style floor division: floor(a / b) for b > 0.
inline std::int32_t floor_div_(std::int32_t a, std::int32_t b) {
    std::int32_t q = a / b;
    // C++ integer division truncates toward zero. Python's `//`
    // floors. They agree iff the signs match OR a % b == 0.
    if ((a % b != 0) && ((a < 0) != (b < 0))) {
        q -= 1;
    }
    return q;
}

}  // namespace

CanonicalCrop canonicalize_crop(std::int32_t x,
                                std::int32_t y,
                                std::int32_t w,
                                std::int32_t h,
                                int q) {
    if (q <= 1) {
        return CanonicalCrop{x, y, w, h};
    }
    const std::int32_t qi = static_cast<std::int32_t>(q);
    return CanonicalCrop{
        floor_div_(x, qi) * qi,
        floor_div_(y, qi) * qi,
        floor_div_(w, qi) * qi,
        floor_div_(h, qi) * qi,
    };
}

std::int64_t frame_id_from_buffer(GstBuffer* buf,
                                  FrameIdSource src,
                                  std::int64_t& counter_state) {
    switch (src) {
        case FrameIdSource::COUNTER: {
            // Post-increment; first buffer is frame 0.
            std::int64_t v = counter_state;
            counter_state = v + 1;
            return v;
        }
        case FrameIdSource::PTS: {
            if (!buf) return -1;
            GstClockTime pts = GST_BUFFER_PTS(buf);
            if (!GST_CLOCK_TIME_IS_VALID(pts)) {
                return -1;
            }
            // GST_NSECOND == 1; the divisor is documented in the spec
            // for clarity and to make the dimensionality explicit
            // (PTS is in nanoseconds). The Python warmer uses the
            // same floor-to-seconds-as-int64 rule.
            return static_cast<std::int64_t>(pts / GST_NSECOND);
        }
    }
    return -1;  // unreachable; keeps strict compilers happy.
}

}  // namespace hailo_cache
