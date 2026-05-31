// gst-hailo-cache — cache key helpers.
//
// Plan 5, Task 3 (docs/superpowers/plans/2026-05-28-gst-cache-plugins.md).
//
// Mirrors hailo_tiling/cache/hashing.py:canonicalize_crop and defines the
// frame-id contract shared by the writer (hailocachewriter) and the
// reader (hailocachereader). Both elements MUST compute identical
// (frame_idx, crop_x, crop_y, crop_w, crop_h, ppv) keys for the same
// buffer; the two helpers in this header are the only place those
// rules are encoded.
//
// canonicalize_crop:
//   When `q > 1`, each component is rounded DOWN to the nearest
//   multiple of q (floor(coord / q) * q) — byte-identical to the Python
//   helper. When `q <= 1` (also when caller passes q==0, matching
//   Python's `quantise=None`), the input is returned unchanged.
//
// frame_id_from_buffer:
//   COUNTER — opaque per-instance int64 counter. The caller passes
//     `counter_state` by reference; this function post-increments it
//     and returns the pre-increment value (so the first buffer in a
//     stream is frame 0).
//   PTS — `(int64_t)(GST_BUFFER_PTS(buf) / GST_NSECOND)`. The Python
//     warmer uses the same nanosecond-floored rule when running off
//     PTS, so writer and reader stay aligned across runs.
//   If `src == PTS` and the buffer has no valid PTS (i.e. PTS is
//   GST_CLOCK_TIME_NONE), the function returns -1; the caller decides
//   whether that's an error or a fallback condition.
//
// Both helpers are pure C++ — they don't touch the SQLite layer.

#pragma once

#include <cstdint>

// Forward-declared so we don't drag gst/gst.h into every consumer.
struct _GstBuffer;
typedef struct _GstBuffer GstBuffer;

namespace hailo_cache {

// Canonical crop rectangle. Distinct from TileCacheDb::CropKey only to
// keep canonicalisation and DB lookup independently testable; callers
// that already have a CropKey can construct one with `{x, y, w, h}`
// and copy fields directly.
struct CanonicalCrop {
    std::int32_t x;
    std::int32_t y;
    std::int32_t w;
    std::int32_t h;
};

// Round-down quantisation to multiples of q. q<=1 (including zero and
// negative) is identity. Matches hailo_tiling.cache.hashing.canonicalize_crop
// byte-for-byte.
CanonicalCrop canonicalize_crop(std::int32_t x,
                                std::int32_t y,
                                std::int32_t w,
                                std::int32_t h,
                                int q);

// Source-vs-caps dimension selection shared by the writer and reader
// (Task 5 DRY). When the explicit source/network-input property is set
// (>0) it wins; otherwise fall back to the negotiated caps dimension.
// Keeping this in one place guarantees both elements agree on which
// dimension the normalized bbox is scaled by.
inline std::int32_t pick_dim(unsigned int prop, std::int32_t caps) {
    return (prop > 0) ? static_cast<std::int32_t>(prop) : caps;
}

// Convert a normalized tile bbox to an absolute pixel crop rect using the
// EXACT TAPPAS `HailoMat::get_bounding_rect` rule (hailomat.cpp):
//
//     rect.x      = CLAMP((int)(xmin   * W), 0, W)
//     rect.y      = CLAMP((int)(ymin   * H), 0, H)
//     rect.width  = CLAMP((int)(width  * W), 0, W - rect.x)
//     rect.height = CLAMP((int)(height * H), 0, H - rect.y)
//
// The float->int cast truncates toward zero (== floor() for the >=0
// bbox components the cropper produces); width/height come from
// width*W / height*H INDEPENDENTLY of x/y, then clamp to the residual
// frame extent. This is the single source of truth for the crop key the
// writer records and the reader looks up — replicating it byte-for-byte
// is what makes replay HIT. The bbox is passed as plain doubles (not a
// HailoBBox) so this header stays free of any TAPPAS include; each
// element extracts the four components from its ROI locally.
//
// Caller must ensure W > 0 && H > 0.
CanonicalCrop tile_crop_to_source_px(double xmin, double ymin,
                                     double width, double height,
                                     std::int32_t W, std::int32_t H);

// True when a normalized ROI bbox is the whole-frame default (0,0,1,1).
// Such a bbox carries no per-tile provenance — both writer and reader use
// this to fall back to the full-frame (0,0,W,H) crop for non-tiled
// pipelines, so their behaviour stays byte-identical.
//
// Exact float equality (no epsilon) is DELIBERATE and shared by both call
// sites: the default ROI is constructed with bit-exact literal `1.0f`
// (and 0.0f) components, so the comparison is reliable and an epsilon
// would only risk swallowing a genuine near-whole-frame tile. Callers pass
// the four bbox components widened to double; the comparison result is
// identical to comparing the original floats against 0.0f/1.0f.
inline bool is_whole_frame_bbox(double xmin, double ymin,
                                double w, double h) {
    return xmin == 0.0 && ymin == 0.0 && w == 1.0 && h == 1.0;
}

enum class FrameIdSource {
    COUNTER,
    PTS,
};

// Derive a stable frame id for `buf` using the rule `src` selects.
//
// COUNTER: post-increments `counter_state` and returns the
//   pre-increment value. The first call with counter_state==0
//   returns 0 and leaves counter_state==1.
//
// PTS: returns (int64_t)(GST_BUFFER_PTS(buf) / GST_NSECOND); returns
//   -1 if PTS is GST_CLOCK_TIME_NONE. `counter_state` is left
//   untouched in this mode.
std::int64_t frame_id_from_buffer(GstBuffer* buf,
                                  FrameIdSource src,
                                  std::int64_t& counter_state);

}  // namespace hailo_cache
