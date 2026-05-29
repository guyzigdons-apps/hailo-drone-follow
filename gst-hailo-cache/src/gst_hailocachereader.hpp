// gst-hailo-cache — hailocachereader element.
//
// Plan 5, Task 8 (docs/superpowers/plans/2026-05-28-gst-cache-plugins.md).
//
// Drop-in replacement for `hailonet` on the cache-replay path.
//
// This is the SKELETON only — the spec §7.9 cache lookup logic lands in
// Task 9. Task 8 delivers:
//
//   - Element registration (GstBaseTransform, passthrough=FALSE).
//   - All spec §7.9 properties (cache-file, hef-path, video-id, on-miss,
//     quantise).
//   - Mirror of every `hailonet` public property (as no-op stubs) so a
//     pipeline can swap `hailonet` ↔ `hailocachereader` with zero other
//     edits.
//   - on-miss=error semantics on transform_ip — without opening the
//     cache yet, every buffer is a miss; an `error` policy posts
//     GST_ERROR + EOS on the first buffer. `drop` passes through.
//
// Caps are ANY/ANY for now; tightening them is a Task 9 TODO.

#pragma once

#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>

G_BEGIN_DECLS

#define GST_TYPE_HAILO_CACHE_READER (gst_hailo_cache_reader_get_type())
G_DECLARE_FINAL_TYPE(GstHailoCacheReader,
                     gst_hailo_cache_reader,
                     GST, HAILO_CACHE_READER,
                     GstBaseTransform)

// Spec §7.9 `on-miss` enum.
typedef enum {
    GST_HAILO_CACHE_READER_ON_MISS_ERROR = 0,
    GST_HAILO_CACHE_READER_ON_MISS_DROP  = 1,
} GstHailoCacheReaderOnMiss;

#define GST_TYPE_HAILO_CACHE_READER_ON_MISS \
    (gst_hailo_cache_reader_on_miss_get_type())
GType gst_hailo_cache_reader_on_miss_get_type(void);

// Hailonet `input-format-type` / `output-format-type` enum (mirrored).
// Values match GstHailoFormatType in hailonet itself.
typedef enum {
    GST_HAILO_CACHE_READER_FORMAT_AUTO    = 0,
    GST_HAILO_CACHE_READER_FORMAT_UINT8   = 1,
    GST_HAILO_CACHE_READER_FORMAT_UINT16  = 2,
    GST_HAILO_CACHE_READER_FORMAT_FLOAT32 = 3,
} GstHailoCacheReaderFormatType;

#define GST_TYPE_HAILO_CACHE_READER_FORMAT_TYPE \
    (gst_hailo_cache_reader_format_type_get_type())
GType gst_hailo_cache_reader_format_type_get_type(void);

// Hailonet `scheduling-algorithm` enum (mirrored).
typedef enum {
    GST_HAILO_CACHE_READER_SCHED_NONE        = 0,
    GST_HAILO_CACHE_READER_SCHED_ROUND_ROBIN = 1,
} GstHailoCacheReaderScheduling;

#define GST_TYPE_HAILO_CACHE_READER_SCHEDULING \
    (gst_hailo_cache_reader_scheduling_get_type())
GType gst_hailo_cache_reader_scheduling_get_type(void);

// Spec §7.9 / Task 9 — frame-id derivation source. Mirrors the writer's
// `GstHailoCacheWriterFrameIdSource` (values must match so writer / reader
// produce identical frame_idx for the same buffer stream).
typedef enum {
    GST_HAILO_CACHE_READER_FRAME_ID_COUNTER = 0,
    GST_HAILO_CACHE_READER_FRAME_ID_PTS     = 1,
} GstHailoCacheReaderFrameIdSource;

#define GST_TYPE_HAILO_CACHE_READER_FRAME_ID_SOURCE \
    (gst_hailo_cache_reader_frame_id_source_get_type())
GType gst_hailo_cache_reader_frame_id_source_get_type(void);

// Buffer-meta qdata key string. The reader marks a buffer's cache-hit
// outcome by attaching one of the GST_HAILO_CACHE_HIT_VALUE_* tokens
// below under this quark on the GstBuffer via
// `gst_mini_object_set_qdata`. Downstream elements (e.g. the Task 12
// hailocachebypass wrapper) read it back via
// `gst_mini_object_get_qdata` + `g_quark_from_static_string(...)`.
//
// Encoding (NOT 0/1 — qdata can't distinguish "nullptr stored" from
// "key absent", and GINT_TO_POINTER(0) is nullptr):
//   - HIT     → GINT_TO_POINTER(GST_HAILO_CACHE_HIT_VALUE_HIT)  = 1
//   - MISS    → GINT_TO_POINTER(GST_HAILO_CACHE_HIT_VALUE_MISS) = 2
//   - absent  → reader did not process the buffer.
//
// Downstream code that only cares about "was this a hit" should test
//   `GPOINTER_TO_INT(p) == GST_HAILO_CACHE_HIT_VALUE_HIT`
// rather than `p != NULL`.
//
// This is the Task-9 fallback for the real `GstMeta` that Phase 14
// will add to hailo-apps-core; see the TODO in gst_hailocachereader.cpp
// transform_ip.
#define GST_HAILO_CACHE_HIT_QDATA_KEY "hailo-cache-hit"
#define GST_HAILO_CACHE_HIT_VALUE_HIT  1
#define GST_HAILO_CACHE_HIT_VALUE_MISS 2

// Same fallback pattern for the cached detection list when the public
// TAPPAS `HailoROI` / `HailoDetectionPtr` API is not used.
#define GST_HAILO_CACHED_DETECTIONS_QDATA_KEY "hailo-cached-detections"

// Register the element with the plugin. Returns TRUE on success.
gboolean gst_hailo_cache_reader_plugin_init(GstPlugin* plugin);

G_END_DECLS
