// gst-hailo-cache — hailocachebypass element.
//
// Plan 5, Task 12 (docs/superpowers/plans/2026-05-28-gst-cache-plugins.md):
//   Phase 14 `hailofilter bypass-on-cache-hit` — wrapper-first.
//
// This element is the OPTION 1 (preferred) implementation of spec §7.9's
// cache-hit bypass contract: when the upstream `hailocachereader` reports
// a cache HIT (via the GST_HAILO_CACHE_HIT_QDATA_KEY qdata on the buffer,
// see gst_hailocachereader.hpp), the postprocess `hailofilter .so` MUST
// NOT run on the buffer — the cached detection JSON is already attached
// and is the authoritative result.
//
// Wrapper strategy (no submodule patch required):
//
// `hailocachebypass` REPLACES `hailofilter` in the cache-replay pipeline.
// It is a thin GstBaseTransform passthrough that:
//
//   - HIT  (qdata == GST_HAILO_CACHE_HIT_VALUE_HIT, == 1):
//       Forwards the buffer untouched. The cached detection JSON is left
//       on the buffer under GST_HAILO_CACHED_DETECTIONS_QDATA_KEY so the
//       downstream visualiser / writer / overlay sees identical state to
//       what a live hailofilter would have produced.
//
//   - MISS (qdata == GST_HAILO_CACHE_HIT_VALUE_MISS, == 2):
//       Forwards the buffer untouched. Downstream is expected to handle
//       the absence of detections (matches `hailocachereader on-miss=drop`,
//       which sets MISS on the buffer and pushes it through with zero
//       detections).
//
//   - qdata absent:
//       Forwards the buffer untouched WITH A `GST_WARNING`. This element
//       is only meaningful in the cache-replay pipeline downstream of
//       `hailocachereader`; running it in any other context is a
//       configuration mistake but not a fatal error (we don't want to
//       break live pipelines by accident if someone adds it the wrong way
//       round).
//
// Canonical replay pipeline (spec §7.9, Plan 5 README "Reader" section):
//
//   ... ! hailotilecropper_dynamic !
//       hailocachereader cache-file=flight.sqlite3 hef-path=...hef !
//       hailocachebypass !
//       hailocachewriter mode=full_frame ! ...
//
// (Where the live pipeline would have `hailonet ! hailofilter` between
// the cropper and the writer.)
//
// Pad caps are ANY/ANY — like `hailocachereader`, this element only
// touches metadata.

#pragma once

#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>

G_BEGIN_DECLS

#define GST_TYPE_HAILO_CACHE_BYPASS (gst_hailo_cache_bypass_get_type())
G_DECLARE_FINAL_TYPE(GstHailoCacheBypass,
                     gst_hailo_cache_bypass,
                     GST, HAILO_CACHE_BYPASS,
                     GstBaseTransform)

// Register the element with the plugin. Returns TRUE on success.
gboolean gst_hailo_cache_bypass_plugin_init(GstPlugin* plugin);

G_END_DECLS
